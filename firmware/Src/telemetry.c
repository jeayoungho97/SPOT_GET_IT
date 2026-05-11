#include "telemetry.h"
#include "robot_state.h"
#include "servo_sts3215.h"
#include "robot.h"
#include "imu_bno055.h"
#include "system_hal.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* raw position (0~4095) → rad. per-joint sign/ZERO_POS는 A4에서 적용 */
static inline float position_raw_to_rad(uint16_t raw) {
    return ((float)raw - 2048.0f) * (2.0f * (float)M_PI / 4096.0f);
}

/* STS3215 speed: sign-magnitude, 0.732 RPM/LSB → rad/s */
static inline float speed_raw_to_rad_s(int16_t raw) {
    return (float)raw * 0.732f * (2.0f * (float)M_PI / 60.0f);
}

/* load raw → -1.0 ~ +1.0 정규화 */
static inline float load_raw_to_normalized(int16_t raw) {
    float norm = (float)raw / 1000.0f;
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm;
}

bool telemetry_read_all_servos(void) {
    bool all_ok = true;

    for (int l = 0; l < NUM_LEGS; l++) {
        for (int j = 0; j < SERVOS_PER_LEG; j++) {
            int idx = l * SERVOS_PER_LEG + j;

            sts_full_state_t s = sts_read_full_state(legs[l].huart,
                                                     legs[l].servo_ids[j]);
            if (!s.ok) {
                all_ok = false;
                continue;
            }

            g_robot_state.position_rad[idx]   = position_raw_to_rad(s.position);
            g_robot_state.velocity_rad_s[idx] = speed_raw_to_rad_s(s.speed);
            g_robot_state.load[idx]           = load_raw_to_normalized(s.load);
            g_robot_state.temperature[idx]    = (float)s.temperature_C;

            if (idx == NUM_JOINTS - 1) {
                g_robot_state.bus_voltage = (float)s.voltage_dV * 0.1f;
            }
        }
    }

    if (all_ok) {
        g_robot_state.status |= STATUS_BIT_ALL_SERVOS_OK;
    } else {
        g_robot_state.status &= ~STATUS_BIT_ALL_SERVOS_OK;
    }

    return all_ok;
}

bool telemetry_read_imu(void) {
    body_attitude_t tmp;
    bool ok = bno055_read_body(&hi2c1, &tmp);

    if (ok) {
        g_robot_state.imu = tmp;
        g_robot_state.status |= STATUS_BIT_IMU_OK;
    } else {
        g_robot_state.status &= ~STATUS_BIT_IMU_OK;
    }

    return ok;
}

bool telemetry_update_all(void) {
    bool servo_ok = telemetry_read_all_servos();
    bool imu_ok = telemetry_read_imu();
    return servo_ok && imu_ok;
}
