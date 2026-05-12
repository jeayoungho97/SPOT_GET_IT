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
static volatile bool     spi_transfer_done = false;
static volatile uint32_t spi_rx_count      = 0;   /* DMA RX 완료 횟수 (1초 카운트, 진단용) */
static volatile uint32_t spi_error_count   = 0;   /* HAL_SPI_ErrorCallback 발생 횟수 */
static volatile uint32_t spi_recover_count = 0;   /* wedge 감지 → Abort 재시작 횟수 */
static volatile uint32_t spi_last_event_ms = 0;   /* 마지막 ISR (complete or error) 시각 */

/* wedge watchdog: ISR 이 이 시간 (ms) 이상 fire 안 되면 SPI wedge 로 간주 */
#define SPI_WEDGE_TIMEOUT_MS  200

/* DMA 완료 콜백 (HAL weak override)
 * ISR context: DMA 끝나는 즉시 DATA_READY_LOW 로 떨어뜨려서
 * Jetson 이 "STM 이 지금 처리 중 — 다음 프레임 아직 보내지 마" 를 즉시 감지하도록.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        DATA_READY_LOW();
        spi_transfer_done = true;
        spi_rx_count++;
        spi_last_event_ms = HAL_GetTick();
    }
}

/* SPI error 콜백 (HAL weak override)
 * OVR/MODF/FRE/DMA 에러로 ISR fire. ErrorCode 는 hspi->ErrorCode 에 set 됨.
 * 그냥 카운터만 올리고 — 실제 recovery 는 main loop watchdog 이 abort + re-arm.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        DATA_READY_LOW();
        spi_error_count++;
        spi_last_event_ms = HAL_GetTick();
    }
}

/* SPI wedge 복구: state != READY 인데 마지막 ISR 이후 N ms 지났으면 강제 재시작.
 * 호출 시점에 DMA armed 일 수도 있고 idle 일 수도 있음. Abort 후 main loop 의
 * 다음 step 6 에서 자연스럽게 re-arm 됨 (GetState 가 READY 로 돌아오므로).
 */
static void spi_wedge_recovery(uint32_t now_ms) {
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) return;
    if (spi_last_event_ms == 0) return;   /* 부팅 직후 아직 ISR 없으면 skip */
    if ((now_ms - spi_last_event_ms) < SPI_WEDGE_TIMEOUT_MS) return;

    /* wedge 확정 — abort */
    HAL_SPI_Abort(&hspi1);
    spi_transfer_done = false;
    DATA_READY_LOW();
    spi_recover_count++;
    spi_last_event_ms = now_ms;
}

/* === Helper: torque 자동 ON ===
 * Jetson 측에서 flag 를 사용하지 않기로 결정 (RL/stand 모두 flags=0 송신).
 * Wire mode 가 OPERATE 면 dispatch 가 알아서 torque on 한다.
 * Torque off 는 MODE_IDLE / MODE_CALIBRATION dispatch 에서 처리.
 * E-STOP 은 STM 내부 트리거 (UART ESC / safety_check / 향후 sit-down) 로만 발동.
 */
static void ensure_torque_on(void) {
    if (!g_robot_state.torque_enabled) {
        robot_torque_on_all();
        g_robot_state.torque_enabled = true;
        g_robot_state.status |= STATUS_BIT_TORQUE_ON;
        /* 토크 ON 시점에 현재 position을 prev_target으로 캡처 → 점프 방지 */
        joint_control_capture_current_as_prev();
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
            /* Jetson 이 OPERATE 명령 보냄 → 자동 torque on */
            ensure_torque_on();
            if (!joint_control_apply_target()) {
                /* NaN 또는 servo timeout → hold fallback */
                joint_control_hold();
            }
            break;

        case MODE_HOLD:
            /* stale fallback — Jetson 짧게 끊겨도 마지막 자세 유지하면서 자세 안 무너지게 */
            ensure_torque_on();
            joint_control_hold();
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

    /* 첫 SPI transfer 시작
     * 순서 중요: DATA_READY_LOW + flag clear → DMA arm → 성공 시에만 DATA_READY_HIGH.
     * DMA arm 이 실패한 상태에서 DATA_READY_HIGH 떴다 하면 Jetson 이 헛 프레임 보낸다.
     */
    spi_encode_feedback(spi_tx_buffer);
    DATA_READY_LOW();
    spi_transfer_done = false;
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buffer, spi_rx_buffer,
                                    SPI_FRAME_SIZE) == HAL_OK) {
        DATA_READY_HIGH();
    }

    /* 50Hz 루프 */
    uint32_t next_tick = HAL_GetTick();
    uint32_t last_print = 0;

    /* === 진단: cycle time 측정 ===
     * 주기적으로 control loop 의 실제 소요 시간 추적.
     * 20ms 초과한 cycle 수 + 최대 cycle 시간 1초마다 reset 후 출력.
     */
    uint32_t worst_cycle_ms = 0;
    uint32_t cycle_over_20 = 0;
    uint32_t cycle_count   = 0;

    /* === 진단: 마지막 도달 step + DMA arm 결과 추적 ===
     * 1Hz print 에서 보고. main loop 어디서 hang 됐는지 / DMA arm 이 어떻게
     * 반환하는지 / DR HIGH 가 실제로 set 되는지 확인용.
     */
    static volatile uint8_t  last_step       = 0;
    static volatile uint8_t  last_arm_ret    = 0xFF;   /* 0=HAL_OK, 1=HAL_ERROR, 2=HAL_BUSY, 0xFF=not arm'd */
    static volatile uint32_t arm_skipped     = 0;       /* GetState != READY 라 arm 안 한 횟수 */
    static volatile uint32_t arm_ok          = 0;       /* HAL_OK 받은 횟수 */
    static volatile uint32_t arm_fail        = 0;       /* arm 실패 횟수 */
    static volatile uint32_t dr_high_set     = 0;       /* DATA_READY_HIGH() 실제 호출된 횟수 */

    while (1) {
        last_step = 0;
        uint32_t t_start = HAL_GetTick();

        /* ESC 체크 */
        if (check_esc()) emergency_stop();

        /* 0. SPI wedge 복구 (state != READY 인데 ISR 200ms 안 fire 됐으면 abort) */
        spi_wedge_recovery(t_start);

        /* 1. SPI RX 처리 */
        last_step = 1;
        if (spi_transfer_done) {
            spi_transfer_done = false;
            DATA_READY_LOW();
            (void)spi_decode_command(spi_rx_buffer);
        }

        /* 2. Stale check */
        last_step = 2;
        check_stale(t_start);

        /* 3. Telemetry (서보 12ch + IMU) */
        last_step = 3;
        telemetry_update_all();

        /* 4. Safety (자세/온도/전압) */
        last_step = 4;
        check_safety();

        /* 5. Mode dispatch */
        last_step = 5;
        dispatch_mode();

        /* 6. SPI TX 준비 + 다음 transfer 시작 */
        last_step = 6;
        spi_encode_feedback(spi_tx_buffer);
        HAL_SPI_StateTypeDef pre_st = HAL_SPI_GetState(&hspi1);
        if (pre_st == HAL_SPI_STATE_READY) {
            HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(
                &hspi1, spi_tx_buffer, spi_rx_buffer, SPI_FRAME_SIZE);
            last_arm_ret = (uint8_t)ret;
            if (ret == HAL_OK) {
                DATA_READY_HIGH();
                dr_high_set++;
                arm_ok++;
            } else {
                arm_fail++;
            }
        } else {
            arm_skipped++;
        }
        last_step = 7;

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
            /* === SPI 수신 + peripheral state 진단 ===
             *   rx/s     : 지난 1초간 SPI DMA RX 완료 횟수
             *   crc_err  : 지난 1초간 CRC 불일치 횟수
             *   spi_st   : HAL_SPI_GetState (1=READY, 5=BUSY_TX_RX, 6=ERROR, 7=ABORT)
             *   spi_err  : ErrorCode bitmask (1=MODF, 2=CRC, 4=OVR, 8=FRE, 0x10=DMA, ...)
             * 50Hz 정상이면 rx~50, crc_err=0, spi_st=1 또는 5, spi_err=0.
             */
            static uint32_t prev_crc_err = 0;
            uint32_t cur_rx_count   = spi_rx_count;
            spi_rx_count = 0;
            uint32_t crc_err_delta  = g_robot_state.crc_error_count - prev_crc_err;
            prev_crc_err = g_robot_state.crc_error_count;

            uint32_t spi_state_now = (uint32_t)HAL_SPI_GetState(&hspi1);
            uint32_t spi_err_now   = (uint32_t)HAL_SPI_GetError(&hspi1);

            /* DR 핀 실제 register 값 — ODR (우리가 쓴 값) vs IDR (실제 전기 상태) */
            uint8_t dr_odr = (GPIOB->ODR & GPIO_PIN_0) ? 1 : 0;
            uint8_t dr_idr = (GPIOB->IDR & GPIO_PIN_0) ? 1 : 0;

            /* 지난 1초간 arm 통계 delta */
            static uint32_t prev_arm_ok = 0, prev_arm_fail = 0, prev_arm_skipped = 0, prev_dr_high = 0;
            uint32_t d_arm_ok       = arm_ok       - prev_arm_ok;
            uint32_t d_arm_fail     = arm_fail     - prev_arm_fail;
            uint32_t d_arm_skipped  = arm_skipped  - prev_arm_skipped;
            uint32_t d_dr_high      = dr_high_set  - prev_dr_high;
            prev_arm_ok       = arm_ok;
            prev_arm_fail     = arm_fail;
            prev_arm_skipped  = arm_skipped;
            prev_dr_high      = dr_high_set;

            printf("[%lu] mode=%u st=0x%02X fault=%u torque=%u seq=%u "
                   "rx=%lu/s crc_err=%lu/s spi_st=%lu spi_err=0x%lX "
                   "dr_odr=%u dr_idr=%u step=%u "
                   "arm_ok=%lu/s skip=%lu/s fail=%lu/s drH=%lu/s arm_ret=%u "
                   "isr_err=%lu recover=%lu "
                   "maxT=%.0fC(j%d) Vbus=%.1fV "
                   "worst_cyc=%lums over20=%lu/%lu\r\n",
                   (unsigned long)t_start,
                   (unsigned)g_robot_state.mode,
                   (unsigned)g_robot_state.status,
                   (unsigned)g_robot_state.fault_code,
                   (unsigned)g_robot_state.torque_enabled,
                   (unsigned)g_robot_state.cmd_seq,
                   (unsigned long)cur_rx_count,
                   (unsigned long)crc_err_delta,
                   (unsigned long)spi_state_now,
                   (unsigned long)spi_err_now,
                   (unsigned)dr_odr, (unsigned)dr_idr, (unsigned)last_step,
                   (unsigned long)d_arm_ok, (unsigned long)d_arm_skipped,
                   (unsigned long)d_arm_fail, (unsigned long)d_dr_high,
                   (unsigned)last_arm_ret,
                   (unsigned long)spi_error_count, (unsigned long)spi_recover_count,
                   (double)max_temp, max_temp_idx,
                   (double)g_robot_state.bus_voltage,
                   (unsigned long)worst_cycle_ms,
                   (unsigned long)cycle_over_20,
                   (unsigned long)cycle_count);

            /* 1초마다 리셋 (직전 1초 통계) */
            worst_cycle_ms = 0;
            cycle_over_20 = 0;
            cycle_count = 0;
        }

        /* 8. 주기 유지 (50Hz = 20ms) — cycle time 측정 후 delay */
        uint32_t cycle_body_ms = HAL_GetTick() - t_start;
        if (cycle_body_ms > worst_cycle_ms) worst_cycle_ms = cycle_body_ms;
        if (cycle_body_ms > 20) cycle_over_20++;
        cycle_count++;

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

