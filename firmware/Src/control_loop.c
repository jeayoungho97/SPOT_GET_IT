#include "control_loop.h"
#include "robot_state.h"
#include "spi_protocol.h"
#include "joint_control.h"
#include "telemetry.h"
#include "robot.h"
#include "system_hal.h"
#include "config.h"
#include "spi.h"
#include <stdio.h>
#include <math.h>

#define STALE_TIMEOUT_MS       200     /* 10 cycle 이상 패킷 없으면 stale (Linux non-RT jitter 흡수) */
#define TEMP_LIMIT_C           70.0f   /* 서보 온도 한계 */
#define VOLTAGE_LOW_V          10.0f   /* 3S LiPo 저전압 한계 */
#define VOLTAGE_VALID_V        0.1f    /* ADC 유효 판별 최소값 */
#define DEBUG_PRINT_MS         1000    /* 디버그 출력 주기 */

/* SPI buffers (DMA용, 영구 할당 — 스택 X) */
static uint8_t spi_tx_buffer[SPI_FRAME_SIZE];
static uint8_t spi_rx_buffer[SPI_FRAME_SIZE];

/* SPI 통신 상태 */
static volatile bool spi_transfer_done = false;

/* DMA 완료 콜백 (HAL weak override) */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spi_transfer_done = true;
    }
}

/* === Helper: torque 상태 갱신 === */
static void update_torque_from_flags(void) {
    bool requested = (g_robot_state.flags & SPI_FLAG_TORQUE_EN) != 0;

    if (requested && !g_robot_state.torque_enabled) {
        robot_torque_on_all();
        g_robot_state.torque_enabled = true;
        g_robot_state.status |= STATUS_BIT_TORQUE_ON;
        /* 토크 ON 시점에 현재 position을 prev_target으로 캡처 → 점프 방지 */
        joint_control_capture_current_as_prev();
    } else if (!requested && g_robot_state.torque_enabled) {
        robot_torque_off_all();
        g_robot_state.torque_enabled = false;
        g_robot_state.status &= ~STATUS_BIT_TORQUE_ON;
    }
}

/* === Helper: stale 체크 === */
static void check_stale(uint32_t now_ms) {
    uint32_t age = now_ms - g_robot_state.last_cmd_time_ms;
    if (age > STALE_TIMEOUT_MS) {
        g_robot_state.status &= ~STATUS_BIT_CMD_FRESH;
        if (g_robot_state.fault_code == FAULT_OK) {
            g_robot_state.fault_code = FAULT_STALE_COMMAND;
        }
        /* stale 시 강제 HOLD (last target 유지) — 안전 fallback */
        g_robot_state.mode = MODE_HOLD;
    }
}

/* === Helper: 안전 검사 ===
 *
 * Fault 처리 정책:
 *  - Transient fault (SAFETY, TEMP, VOLTAGE, IMU, STALE, SERVO_TIMEOUT):
 *    매 cycle 재평가. 조건 해소되면 자동 FAULT_OK로 복귀.
 *  - Sticky fault (CRC_ERROR, NAN_IN_TARGET): 한 번 set 되면 유지.
 *    명시적 reset 또는 다음 transient fault 가 더 우선시되면 변경 가능.
 *
 * Priority (높은 게 우선): SAFETY > NAN > TEMP > VOLTAGE > IMU > STALE > SERVO_TMO > CRC
 */
static void check_safety(void) {
    fault_code_t new_fault = FAULT_OK;

    /* 1. 자세 한계 (pitch/roll) — 가장 위험, 즉시 torque OFF */
    if (!safety_check(g_robot_state.imu.pitch, g_robot_state.imu.roll)) {
        new_fault = FAULT_SAFETY_LIMIT;
        g_robot_state.status |= STATUS_BIT_IN_SAFE_STATE;
        robot_torque_off_all();
        g_robot_state.torque_enabled = false;
        g_robot_state.status &= ~STATUS_BIT_TORQUE_ON;
        g_robot_state.mode = MODE_IDLE;
        g_robot_state.fault_code = new_fault;
        return;
    } else {
        g_robot_state.status &= ~STATUS_BIT_IN_SAFE_STATE;
    }

    /* 2. 서보 온도 한계 */
    if (new_fault == FAULT_OK) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (g_robot_state.temperature[i] > TEMP_LIMIT_C) {
                new_fault = FAULT_TEMP_HIGH;
                break;
            }
        }
    }

    /* 3. 전압 한계 (ADC가 유효한 경우만) */
    if (new_fault == FAULT_OK
        && g_robot_state.bus_voltage > VOLTAGE_VALID_V
        && g_robot_state.bus_voltage < VOLTAGE_LOW_V) {
        new_fault = FAULT_VOLTAGE_LOW;
    }

    /* Sticky fault (CRC, NAN) 는 보존 — transient 조건 다 해소돼도 OK로 안 돌림.
     * 다만 새 transient fault 가 생기면 그게 우선. */
    fault_code_t cur = g_robot_state.fault_code;
    bool cur_is_sticky = (cur == FAULT_CRC_ERROR || cur == FAULT_NAN_IN_TARGET);
    if (cur_is_sticky && new_fault == FAULT_OK) {
        /* sticky 유지 */
    } else {
        g_robot_state.fault_code = new_fault;
    }
}

/* === Helper: 모드 디스패치 === */
static void dispatch_mode(void) {
    switch (g_robot_state.mode) {
        case MODE_IDLE:
            /* 토크 OFF 유지 */
            if (g_robot_state.torque_enabled) {
                robot_torque_off_all();
                g_robot_state.torque_enabled = false;
                g_robot_state.status &= ~STATUS_BIT_TORQUE_ON;
            }
            break;

        case MODE_POSITION:
            update_torque_from_flags();
            if (g_robot_state.torque_enabled) {
                if (!joint_control_apply_target()) {
                    /* NaN 또는 servo timeout → hold fallback */
                    joint_control_hold();
                }
            }
            break;

        case MODE_HOLD:
            update_torque_from_flags();
            if (g_robot_state.torque_enabled) {
                joint_control_hold();
            }
            break;

        case MODE_CALIBRATION:
            /* 토크 OFF + 측정만 (telemetry는 계속 갱신) */
            g_robot_state.status |= STATUS_BIT_CALIBRATING;
            if (g_robot_state.torque_enabled) {
                robot_torque_off_all();
                g_robot_state.torque_enabled = false;
                g_robot_state.status &= ~STATUS_BIT_TORQUE_ON;
            }
            break;

        default:
            /* 알 수 없는 모드 → IDLE 강제 */
            g_robot_state.mode = MODE_IDLE;
            break;
    }
}

/* === 50Hz 메인 루프 === */
void control_loop_run(void) {
    /* 초기화 */
    robot_state_init();

    /* 부팅 직후: torque OFF, IDLE 모드 */
    robot_torque_off_all();
    g_robot_state.torque_enabled = false;
    g_robot_state.mode = MODE_IDLE;

    /* 첫 telemetry read → prev_target_rad 초기화 */
    telemetry_update_all();
    joint_control_capture_current_as_prev();

    /* last_cmd_time_ms 초기화 — 부팅 직후 즉시 stale 발동 방지 */
    g_robot_state.last_cmd_time_ms = HAL_GetTick();

    printf("\r\n=== Control loop started (50Hz, RL-driven) ===\r\n");

    /* 첫 SPI transfer 시작 */
    spi_encode_feedback(spi_tx_buffer);
    HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buffer, spi_rx_buffer,
                                SPI_FRAME_SIZE);
    DATA_READY_HIGH();
    spi_transfer_done = false;

    /* 50Hz 루프 */
    uint32_t next_tick = HAL_GetTick();
    uint32_t last_print = 0;

    while (1) {
        uint32_t t_start = HAL_GetTick();

        /* ESC 체크 */
        if (check_esc()) emergency_stop();

        /* 1. SPI RX 처리 */
        if (spi_transfer_done) {
            spi_transfer_done = false;
            DATA_READY_LOW();

            /* E-STOP flag 체크 (디코드 전 확인) */
            decode_result_t dr = spi_decode_command(spi_rx_buffer);
            if (dr == DECODE_OK
                && (g_robot_state.flags & SPI_FLAG_E_STOP)) {
                emergency_stop();
            }
        }

        /* 2. Stale check */
        check_stale(t_start);

        /* 3. Telemetry (서보 12ch + IMU) */
        telemetry_update_all();

        /* 4. Safety (자세/온도/전압) */
        check_safety();

        /* 5. Mode dispatch */
        dispatch_mode();

        /* 6. SPI TX 준비 + 다음 transfer 시작 */
        spi_encode_feedback(spi_tx_buffer);
        if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
            HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buffer,
                                        spi_rx_buffer, SPI_FRAME_SIZE);
            DATA_READY_HIGH();
        }

        /* 7. 디버그 출력 (1초마다) — 실제 max temp 도 함께 출력해서 fault=5 진위 확인 */
        if (t_start - last_print >= DEBUG_PRINT_MS) {
            last_print = t_start;
            float max_temp = 0.0f;
            int   max_temp_idx = 0;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (g_robot_state.temperature[i] > max_temp) {
                    max_temp = g_robot_state.temperature[i];
                    max_temp_idx = i;
                }
            }
            printf("[%lu] mode=%u st=0x%02X fault=%u torque=%u seq=%u "
                   "maxT=%.0fC(j%d) Vbus=%.1fV\r\n",
                   (unsigned long)t_start,
                   (unsigned)g_robot_state.mode,
                   (unsigned)g_robot_state.status,
                   (unsigned)g_robot_state.fault_code,
                   (unsigned)g_robot_state.torque_enabled,
                   (unsigned)g_robot_state.cmd_seq,
                   (double)max_temp, max_temp_idx,
                   (double)g_robot_state.bus_voltage);
        }

        /* 8. 주기 유지 (50Hz = 20ms) */
        next_tick += LOOP_PERIOD_MS;
        uint32_t now = HAL_GetTick();
        if (now < next_tick) {
            HAL_Delay(next_tick - now);
        } else {
            /* deadline miss — 다음 tick 재정렬 */
            next_tick = now;
        }
    }
}
