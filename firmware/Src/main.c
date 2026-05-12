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
#include "spi.h"
#include "control_loop.h"
#include <stdio.h>
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
#elif DEMO_MODE == MODE_SPI_TEST
    printf("  Demo: SPI-TEST  (ESC to exit)\r\n");
#elif DEMO_MODE == MODE_RL_CONTROL
    printf("  Demo: RL-CONTROL  (Jetson 50Hz control loop)\r\n");
#elif DEMO_MODE == MODE_DR_TOGGLE_TEST
    printf("  Demo: DR-TOGGLE-TEST  (PB0 1Hz 토글 — 배선 검증)\r\n");
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

#if DEMO_MODE == MODE_DR_TOGGLE_TEST
    /* === DATA_READY (PB0) LOW 유지 테스트 ===
     * Jetson 쪽 gpiod 배선/코드 검증용. 서보·IMU 의존 없이 GPIO 를 LOW 로 고정.
     * 확인 방법 (Jetson):
     *   sudo gpioget <chip> <line>      # 0 이 찍히면 OK
     * heartbeat 1초마다 UART 로 찍어서 STM32 가 살아 있음을 확인.
     * 종료: ESC (delay_with_estop 안에서 폴링) 또는 보드 리셋.
     */
    printf("\r\n=== DATA_READY (PB0) LOW hold test ===\r\n");
    printf("    PB0 = LOW (constant). ESC to exit.\r\n");
    DATA_READY_LOW();
    {
    	uint32_t cnt = 0;
    	while (1) {
    	    DATA_READY_HIGH();
    	    printf("[%lu] DR=HIGH\r\n", (unsigned long)cnt++);
    	    delay_with_estop(500);
    	    DATA_READY_LOW();
    	    printf("[%lu] DR=LOW\r\n", (unsigned long)cnt++);
    	    delay_with_estop(500);
    	}
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

#elif DEMO_MODE == MODE_SPI_TEST
    robot_state_init();
    printf("\r\n========== SPI PROTOCOL TEST ==========\r\n");
    printf("Waiting for Jetson SPI master (%d byte frames).\r\n", SPI_FRAME_SIZE);
    printf("Torque OFF. Telemetry + encode/decode verification.\r\n");
    robot_torque_off_all();

    static uint8_t spi_tx_buf[SPI_FRAME_SIZE];
    static uint8_t spi_rx_buf[SPI_FRAME_SIZE];

    telemetry_update_all();
    spi_encode_feedback(spi_tx_buf);

    HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE);
    DATA_READY_HIGH();

    uint32_t spi_frame_count = 0;
    while (1) {
        if (check_esc()) emergency_stop();

        if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
            DATA_READY_LOW();
            spi_frame_count++;

            decode_result_t dr = spi_decode_command(spi_rx_buf);

            telemetry_update_all();
            spi_encode_feedback(spi_tx_buf);

            HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE);
            DATA_READY_HIGH();

            static uint32_t last_spi_print = 0;
            uint32_t now = HAL_GetTick();
            if (now - last_spi_print >= 250) {
                last_spi_print = now;
                printf("[%lu] decode=%d seq=%u mode=%u tgt[0]=%+.3f\r\n",
                       (unsigned long)spi_frame_count,
                       (int)dr, g_robot_state.cmd_seq,
                       (unsigned)g_robot_state.mode,
                       (double)g_robot_state.target_rad[0]);
            }
        }

        HAL_Delay(POLL_PERIOD_MS);
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
