#include "stm32f4xx_hal.h"
#include "config.h"
#include "system_hal.h"
#include "robot.h"
#include "imu_bno055.h"
#include "gait.h"
#include <stdio.h>

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
            body_attitude_t b;
            if (bno055_read_body(&hi2c1, &b)) {
                printf("[stand] y=%+5.1f p=%+5.1f r=%+5.1f\r\n",
                       (double)b.yaw, (double)b.pitch, (double)b.roll);
            }
            last_print = HAL_GetTick();
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
