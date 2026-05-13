#include "stm32f4xx_hal.h"
#include "config.h"
#include "system_hal.h"
#include "robot.h"
#include "imu_bno055.h"
#include "gait.h"
#include "robot_state.h"
#include "telemetry.h"
#include "joint_control.h"
#include "calibration.h"
#include "spi_protocol.h"
#include "uart_jetson.h"
#include "control_loop.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

int main(void) {
    HAL_Init();
    system_hal_init_all();

    /* === Banner === */
    printf("\r\n=== STM32F446RE Boot ===\r\n");
    printf("\r\n=================================================\r\n");
    printf("  Trot Walking — Continuous Bezier Swing\r\n");
    printf("\r\n");
#if DEMO_MODE == MODE_TROT
    printf("  Demo: TROT  (%d cycles)\r\n", N_CYCLES);
#elif DEMO_MODE == MODE_STAND_ONLY
    printf("  Demo: STAND-ONLY  (ESC to exit)\r\n");
#elif DEMO_MODE == MODE_TELEMETRY_TEST
    printf("  Demo: TELEMETRY-TEST  (ESC to exit)\r\n");
#elif DEMO_MODE == MODE_JOINT_TEST
    printf("  Demo: JOINT-TEST  (ESC to exit)\r\n");
#elif DEMO_MODE == MODE_CAL_MEASURE
    printf("  Demo: CALIBRATION  (ESC to exit)\r\n");
#elif DEMO_MODE == MODE_RL_CONTROL
    printf("  Demo: RL-CONTROL  (Jetson 50Hz control loop)\r\n");
#elif DEMO_MODE == MODE_UART_HELLO
    printf("  Demo: UART-HELLO  (USART1 hello/echo — 통신 검증)\r\n");
#endif
#if IN_HAND_MODE
    printf("  Safety: IN-HAND  (비활성, 손에 들고 시연)\r\n");
#else
    printf("  Safety: FLOOR  (활성: pitch>%d°, roll>%d°)\r\n",
           (int)MAX_PITCH_DEG, (int)MAX_ROLL_DEG);
#endif
    printf("  Body height: %.0f mm  [%.0f ~ %.0f]\r\n",
           (double)BODY_HEIGHT_MM, (double)BODY_HEIGHT_MIN_MM, (double)BODY_HEIGHT_MAX_MM);
    printf("  Gait period: %d ms  Duty: %.2f\r\n",
           GAIT_PERIOD_MS, DUTY_FACTOR);
    printf("  Stride: %.0f mm  Lift: %.0f mm (peak = LIFT_Z, 대칭 arch + smoothstep)\r\n",
           STRIDE_X, LIFT_Z);
    printf("  Pair A: FR + RL  (offset 0.0)\r\n");
    printf("  Pair B: FL + RR  (offset 0.5)\r\n");
    printf("  Monitor: %s\r\n", ENABLE_MONITOR ? "ON" : "OFF");
    printf("\r\n");
#if IN_HAND_MODE
    printf("  >>> 로봇을 단단히 잡고 시작 <<<\r\n");
#else
    printf("  >>> 평면 위에 두고 catch 준비 <<<\r\n");
#endif
    printf("=================================================\r\n");

#if DEMO_MODE == MODE_UART_HELLO
    /* === USART1 (Jetson 통신) hello/echo 테스트 ===
     * 서보/IMU 의존 없이 UART 만 검증.
     * STM→Jetson: 1초마다 "hello UART N" 송신 (UART TX DMA)
     * Jetson→STM: 받은 바이트는 ST-Link VCP 콘솔에 hex dump
     *
     * Jetson 측 검증 명령:
     *   screen /dev/ttyTHS1 921600
     *   (또는 picocom -b 921600 /dev/ttyTHS1)
     * 키 입력하면 STM 콘솔에 [RX] XX XX ... 로 찍힘.
     * 종료: 보드 리셋.
     */
    printf("\r\n=== UART Hello Test (USART1, 921600 8N1) ===\r\n");
    printf("    PB6 (TX, CN10-17)  -> Jetson Pin 10 (RX)\r\n");
    printf("    PB7 (RX, CN7-21)   <- Jetson Pin 8  (TX)\r\n");
    printf("    Loopback: PB6 <-> PB7 (점퍼 와이어)\r\n");
    printf("    Jetson:  picocom -b 921600 /dev/ttyTHS1\r\n\r\n");

    uart_jetson_start_rx();

    uint16_t tail = 0;
    uint32_t cnt = 0;
    uint32_t last_tx = 0;
    uint32_t last_diag = 0;
    char msg[64];
    while (1) {
        if (check_esc()) emergency_stop();

        uint32_t now = HAL_GetTick();

        /* 1초마다 UART 상태 진단 출력 */
        if (now - last_diag >= 1000) {
            last_diag = now;
            uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart_jetson.hdmarx);
            uint16_t idr  = (uint16_t)(GPIOB->IDR & 0xFFFF);
            printf("[DIAG] NDTR=%u gState=0x%02X RxState=0x%02X head=%u tail=%u\r\n"
                   "       SR=0x%04lX CR1=0x%04lX CR3=0x%04lX BRR=0x%04lX\r\n"
                   "       GPIOB: AFR0=0x%08lX MODER=0x%08lX PUPDR=0x%08lX IDR=0x%04X (PB6=%u PB7=%u)\r\n",
                   (unsigned)ndtr,
                   (unsigned)huart_jetson.gState,
                   (unsigned)huart_jetson.RxState,
                   (unsigned)uart_jetson_rx_head(),
                   (unsigned)tail,
                   (unsigned long)USART1->SR,
                   (unsigned long)USART1->CR1,
                   (unsigned long)USART1->CR3,
                   (unsigned long)USART1->BRR,
                   (unsigned long)GPIOB->AFR[0],
                   (unsigned long)GPIOB->MODER,
                   (unsigned long)GPIOB->PUPDR,
                   (unsigned)idr,
                   (unsigned)((idr >> 6) & 1),
                   (unsigned)((idr >> 7) & 1));
        }

        /* 1초마다 hello 송신 — DMA + Blocking 두 방식 모두 시도해 진단 */
        if (now - last_tx >= 1000) {
            last_tx = now;
            int n = snprintf(msg, sizeof(msg),
                             "hello UART %lu\r\n",
                             (unsigned long)cnt++);
            if (n > 0 && uart_jetson_transmit_dma((const uint8_t *)msg, (uint16_t)n)) {
                printf("[TX %lu DMA] %s", (unsigned long)cnt, msg);
            } else {
                printf("[TX %lu DMA] BUSY (skip)\r\n", (unsigned long)cnt);
            }

            /* Blocking TX — DMA 우회해서 USART 자체가 송신 가능한지 검증 */
            const char *blk = "BLOCK\r\n";
            HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart_jetson,
                                                     (uint8_t *)blk, 7, 100);
            printf("[TX %lu BLOCK] ret=%d\r\n", (unsigned long)cnt, (int)ret);
        }

        /* RX 도착 바이트 hex dump (head 가 갱신됐을 때만) */
        uint16_t head = uart_jetson_rx_head();
        if (head != tail) {
            const uint8_t *buf = uart_jetson_rx_buffer();
            printf("[RX] ");
            while (tail != head) {
                printf("%02X ", buf[tail]);
                tail = (uint16_t)((tail + 1) % JETSON_UART_RX_BUF_SIZE);
            }
            printf("\r\n");
        }

        HAL_Delay(10);
    }
    /* 도달 안 함 */
#endif

    /* === Boot sequence === */
    printf("\r\n[1] Pinging 12 servos...\r\n");
    int alive = robot_ping_all();
    printf("    %d / %d alive\r\n", alive, NUM_SERVOS);
    if (alive < NUM_SERVOS) { while (1) {} }

    printf("\r\n[2] BNO055 IMUPLUS init...\r\n");
    if (!bno055_init_imuplus(&hi2c1)) { printf("[ABORT]\r\n"); while (1) {} }
    printf("    OK.\r\n");

    printf("\r\n[3] Torque OFF (current pose 읽기 위해)...\r\n");
    robot_torque_off_all();

#if DEMO_MODE == MODE_CAL_MEASURE
    calibration_mode();
#endif

#if DEMO_MODE == MODE_RL_CONTROL
    /* RL control: boot 직후 control_loop 진입 — torque/pose는 Jetson이 제어 */
    printf("\r\n[4] Entering RL control loop (50Hz)...\r\n");
    printf("    Torque/pose는 Jetson 명령에 의해 제어됩니다.\r\n");
    printf("    Jetson 연결 전까지 IDLE (torque OFF) 상태.\r\n");
    control_loop_run();
    /* 도달 안 함 */
#endif

    printf("\r\n[4] Starting in %d s...\r\n", COUNTDOWN_SEC);
    for (int i = COUNTDOWN_SEC; i > 0; i--) {
        printf("    %d...\r\n", i);
        delay_with_estop(1000);
    }

    pose_t start_pose;
    if (!robot_read_current_pose(start_pose)) {
        printf("[ABORT — read_current_pose failed]\r\n");
        while (1) {}
    }
    robot_apply_pose(start_pose);
    delay_with_estop(50);

    printf("\r\n[5] Torque ON 12 servos...\r\n");
    robot_torque_on_all();
    delay_with_estop(300);

    /* boot pose → 첫 standing pose
       (foot_pos[]가 아직 비어있어서 stand_at_height 못 씀 → 직접 transition) */
    for (int l = 0; l < NUM_LEGS; l++) {
        foot_pos[l][0] = DEFAULT_FOOT_X;
        foot_pos[l][1] = -BODY_HEIGHT_MM;
    }
    pose_t default_pose;
    robot_update_pose_from_foot(default_pose);

    printf("\r\n[6] start -> default pose (height=%.0f mm)\r\n", (double)BODY_HEIGHT_MM);
    robot_transition(start_pose, default_pose, SPEED_DEG_PER_SEC);

    printf("\r\n[7] Initial settle %d ms...\r\n", INITIAL_SETTLE_MS);
    delay_with_estop(INITIAL_SETTLE_MS);

    /* === Mode dispatch === */
#if DEMO_MODE == MODE_TROT
    printf("\r\n========== TROT START ==========\r\n");
    run_trot(N_CYCLES);
    if (g_abort) {
        printf("========== TROT ABORTED ==========\r\n");
    } else {
        printf("========== TROT COMPLETE — %d cycles ==========\r\n", N_CYCLES);
    }
    stand_at_height(BODY_HEIGHT_MM);
    delay_with_estop(500);

#elif DEMO_MODE == MODE_STAND_ONLY
    printf("\r\n========== STAND ONLY ==========\r\n");
    printf("Holding %.0f mm. ESC to exit.\r\n", (double)BODY_HEIGHT_MM);
    uint32_t last_print = HAL_GetTick();
    while (1) {
        if (check_esc()) emergency_stop();
        if (HAL_GetTick() - last_print >= 1000) {
            body_attitude_t b = {0};
            if (bno055_read_body(&hi2c1, &b)) {
                float qn = sqrtf(b.quat[0]*b.quat[0] + b.quat[1]*b.quat[1]
                                + b.quat[2]*b.quat[2] + b.quat[3]*b.quat[3]);
                printf("[stand] eul y=%+5.1f p=%+5.1f r=%+5.1f\r\n",
                       (double)b.yaw, (double)b.pitch, (double)b.roll);
                printf("        gyr x=%+6.3f y=%+6.3f z=%+6.3f rad/s\r\n",
                       (double)b.gyro[0], (double)b.gyro[1], (double)b.gyro[2]);
                printf("        acc x=%+6.2f y=%+6.2f z=%+6.2f m/s2\r\n",
                       (double)b.accel[0], (double)b.accel[1], (double)b.accel[2]);
                printf("        quat w=%+5.3f x=%+5.3f y=%+5.3f z=%+5.3f |q|=%.3f\r\n",
                       (double)b.quat[0], (double)b.quat[1],
                       (double)b.quat[2], (double)b.quat[3], (double)qn);
            }
            last_print = HAL_GetTick();
        }
        HAL_Delay(POLL_PERIOD_MS);
    }

#elif DEMO_MODE == MODE_TELEMETRY_TEST
    robot_state_init();
    printf("\r\n========== TELEMETRY TEST (50Hz) ==========\r\n");
    printf("ESC to exit. Torque OFF — 다리 자유 상태에서 검증.\r\n");
    robot_torque_off_all();

    while (1) {
        uint32_t t_start = HAL_GetTick();
        if (check_esc()) emergency_stop();

        telemetry_update_all();

        static uint32_t last_tel_print = 0;
        if (t_start - last_tel_print >= 250) {
            last_tel_print = t_start;

            printf("pos(rad): ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%+5.2f ", (double)g_robot_state.position_rad[i]);
            printf("\r\n");

            printf("vel(r/s): ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%+5.2f ", (double)g_robot_state.velocity_rad_s[i]);
            printf("\r\n");

            printf("tmp( C ): ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%4.0f ", (double)g_robot_state.temperature[i]);
            printf("\r\n");

            printf("IMU yaw=%+5.1f gyro_z=%+6.3f Vbus=%.1fV\r\n",
                   (double)g_robot_state.imu.yaw,
                   (double)g_robot_state.imu.gyro[2],
                   (double)g_robot_state.bus_voltage);

            uint32_t dt = HAL_GetTick() - t_start;
            printf("dt=%lums\r\n\r\n", (unsigned long)dt);
        }

        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed < 20) HAL_Delay(20 - elapsed);
    }

#elif DEMO_MODE == MODE_JOINT_TEST
    robot_state_init();
    printf("\r\n========== JOINT CONTROL TEST (50Hz) ==========\r\n");
    printf("Torque ON. Slew-limited move to default angles.\r\n");

    telemetry_update_all();
    joint_control_capture_current_as_prev();

    float default_angles[NUM_JOINTS] = {
        0.0f, -0.6f, 1.1f,
        0.0f, -0.6f, 1.1f,
        0.0f, -0.6f, 1.1f,
        0.0f, -0.6f, 1.1f,
    };
    for (int i = 0; i < NUM_JOINTS; i++) {
        g_robot_state.target_rad[i] = default_angles[i];
        g_robot_state.max_delta_rad[i] = 0.02f;
    }

    while (1) {
        uint32_t t_start = HAL_GetTick();
        if (check_esc()) emergency_stop();

        telemetry_update_all();
        joint_control_apply_target();

        static uint32_t last_jt_print = 0;
        if (t_start - last_jt_print >= 250) {
            last_jt_print = t_start;

            printf("target: ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%+5.2f ", (double)g_robot_state.target_rad[i]);
            printf("\r\n");

            printf("actual: ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%+5.2f ", (double)g_robot_state.position_rad[i]);
            printf("\r\n");

            printf("prev:   ");
            for (int i = 0; i < NUM_JOINTS; i++)
                printf("%+5.2f ", (double)g_robot_state.prev_target_rad[i]);
            printf("\r\n");

            printf("fault=%d dt=%lums\r\n\r\n",
                   (int)g_robot_state.fault_code,
                   (unsigned long)(HAL_GetTick() - t_start));
        }

        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed < 20) HAL_Delay(20 - elapsed);
    }

#endif

    /* === default → start, torque off === */
    pose_t end_pose;
    robot_update_pose_from_foot(end_pose);

    printf("\r\n[Final] default -> start\r\n");
    robot_transition(end_pose, start_pose, SPEED_DEG_PER_SEC);

    printf("\r\n[Final] Torque OFF.\r\n");
    robot_torque_off_all();

    printf("\r\n=== TEST COMPLETE ===\r\n");
    while (1) {
        if (check_esc()) emergency_stop();
        HAL_Delay(POLL_PERIOD_MS);
    }
}
