#include "imu_bno055.h"
#include <math.h>

#define BNO055_I2C_ADDR_DEFAULT   0x28
#define BNO055_I2C_ADDR_ALT       0x29
#define BNO055_CHIP_ID            0x00
#define BNO055_ACC_DATA_X_LSB     0x08
#define BNO055_GYR_DATA_X_LSB     0x14
#define BNO055_EUL_HEADING_LSB    0x1A
#define BNO055_QUA_DATA_W_LSB     0x20
#define BNO055_OPR_MODE           0x3D
#define BNO055_OPR_CONFIG         0x00
#define BNO055_OPR_IMUPLUS        0x08
#define BNO055_CHIP_ID_EXPECTED   0xA0

/* === Axis remap registers (BNO055 datasheet 4.4.1) ===
 * 실제 마운트 (사용자 측정):
 *   gravity along +Y_imu  (정자세)  -> robot +Z (up) = -Y_imu
 *   gravity along +Z_imu  (왼쪽 눕힘) -> robot +Y (right) = -Z_imu
 *   gravity along +X_imu  (머리 위)   -> robot +X (forward) = -X_imu
 *
 * BNO055 의 placement P6 와 일치:
 *   AXIS_MAP_CONFIG bits [5:4]=NEW_X, [3:2]=NEW_Y, [1:0]=NEW_Z
 *     00=physical X, 01=physical Y, 10=physical Z
 *   NEW_X = physical X (00), NEW_Y = physical Z (10), NEW_Z = physical Y (01)
 *   -> 0b00 10 01 = 0x21
 *   AXIS_MAP_SIGN bit2=X_neg, bit1=Y_neg, bit0=Z_neg
 *   세 축 모두 negate -> 0x07
 *
 * 효과: chip 이 자체적으로 robot body frame 으로 출력. STM/Jetson 측 변환 불필요.
 */
#define BNO055_AXIS_MAP_CONFIG    0x41
#define BNO055_AXIS_MAP_SIGN      0x42
#define BNO055_AXIS_CONFIG_P6     0x21
#define BNO055_AXIS_SIGN_P6       0x07

#define BNO055_ACCEL_SCALE        (1.0f / 100.0f)
#define BNO055_GYRO_SCALE         (1.0f / 16.0f * ((float)M_PI / 180.0f))
#define BNO055_QUAT_SCALE         (1.0f / 16384.0f)
#define BNO055_EULER_SCALE        (1.0f / 16.0f)
#define QUAT_NORM_MIN             0.5f

/* init에서 결정된 주소를 read_body가 사용 */
static uint8_t bno_addr = BNO055_I2C_ADDR_DEFAULT;

static bool bno_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* val) {
    return (HAL_I2C_Mem_Read(hi2c, bno_addr << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK);
}

static bool bno_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
    return (HAL_I2C_Mem_Write(hi2c, bno_addr << 1, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

static bool bno_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* buf, uint8_t len) {
    return (HAL_I2C_Mem_Read(hi2c, bno_addr << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK);
}

bool bno055_init_imuplus(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(700);
    if (HAL_I2C_IsDeviceReady(hi2c, BNO055_I2C_ADDR_DEFAULT << 1, 3, 50) == HAL_OK) {
        bno_addr = BNO055_I2C_ADDR_DEFAULT;
    } else if (HAL_I2C_IsDeviceReady(hi2c, BNO055_I2C_ADDR_ALT << 1, 3, 50) == HAL_OK) {
        bno_addr = BNO055_I2C_ADDR_ALT;
    } else {
        return false;
    }
    uint8_t chip_id = 0;
    if (!bno_read_byte(hi2c, BNO055_CHIP_ID, &chip_id) || chip_id != BNO055_CHIP_ID_EXPECTED) {
        return false;
    }
    bno_write_byte(hi2c, BNO055_OPR_MODE, BNO055_OPR_CONFIG);
    HAL_Delay(25);

    /* Axis remap — CONFIG 모드에서만 변경 가능.
     * P6 placement: chip 출력이 robot body frame 으로 통일됨 (모든 downstream 일관). */
    bno_write_byte(hi2c, BNO055_AXIS_MAP_CONFIG, BNO055_AXIS_CONFIG_P6);
    HAL_Delay(10);
    bno_write_byte(hi2c, BNO055_AXIS_MAP_SIGN, BNO055_AXIS_SIGN_P6);
    HAL_Delay(10);

    bno_write_byte(hi2c, BNO055_OPR_MODE, BNO055_OPR_IMUPLUS);
    HAL_Delay(20);
    return true;
}

bool bno055_read_body(I2C_HandleTypeDef *hi2c, body_attitude_t *body) {
    /* 0x08~0x27: accel(6) + mag(6,skip) + gyro(6) + euler(6) + quat(8) = 32 byte burst */
    uint8_t buf[32];
    if (!bno_read_bytes(hi2c, BNO055_ACC_DATA_X_LSB, buf, 32)) {
        body->data_valid = false;
        return false;
    }

    /* --- Accel (0x08~0x0D) → buf[0..5] --- */
    int16_t ax_raw = (int16_t)((uint16_t)buf[0]  | ((uint16_t)buf[1]  << 8));
    int16_t ay_raw = (int16_t)((uint16_t)buf[2]  | ((uint16_t)buf[3]  << 8));
    int16_t az_raw = (int16_t)((uint16_t)buf[4]  | ((uint16_t)buf[5]  << 8));
    /* buf[6..11]: MAG — skip */

    /* --- Gyro (0x14~0x19) → buf[12..17] --- */
    int16_t gx_raw = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
    int16_t gy_raw = (int16_t)((uint16_t)buf[14] | ((uint16_t)buf[15] << 8));
    int16_t gz_raw = (int16_t)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8));

    /* --- Euler (0x1A~0x1F) → buf[18..23] --- */
    int16_t h_raw  = (int16_t)((uint16_t)buf[18] | ((uint16_t)buf[19] << 8));
    int16_t r_raw  = (int16_t)((uint16_t)buf[20] | ((uint16_t)buf[21] << 8));
    int16_t p_raw  = (int16_t)((uint16_t)buf[22] | ((uint16_t)buf[23] << 8));

    /* --- Quaternion (0x20~0x27) → buf[24..31] --- */
    int16_t qw_raw = (int16_t)((uint16_t)buf[24] | ((uint16_t)buf[25] << 8));
    int16_t qx_raw = (int16_t)((uint16_t)buf[26] | ((uint16_t)buf[27] << 8));
    int16_t qy_raw = (int16_t)((uint16_t)buf[28] | ((uint16_t)buf[29] << 8));
    int16_t qz_raw = (int16_t)((uint16_t)buf[30] | ((uint16_t)buf[31] << 8));

    /* Euler — 기존 본체 마운팅 보정 유지 */
    body->yaw   =  h_raw * BNO055_EULER_SCALE;
    body->pitch = -(r_raw * BNO055_EULER_SCALE);
    body->roll  = -(p_raw * BNO055_EULER_SCALE);

    /* Gyro → rad/s, 본체 마운팅 보정 (Euler와 동일: X↔X, Y↔Z swap, 부호 반전) */
    float gx = gx_raw * BNO055_GYRO_SCALE;
    float gy = gy_raw * BNO055_GYRO_SCALE;
    float gz = gz_raw * BNO055_GYRO_SCALE;
    body->gyro[0] =  gx;       /* body X (forward) */
    body->gyro[1] = -gz;       /* body Y (left) — BNO Z → body Y, 부호 반전 */
    body->gyro[2] = -gy;       /* body Z (up)   — BNO Y → body Z, 부호 반전 */

    /* Accel → m/s², 본체 마운팅 보정 (gyro와 동일 축 매핑) */
    float ax = ax_raw * BNO055_ACCEL_SCALE;
    float ay = ay_raw * BNO055_ACCEL_SCALE;
    float az = az_raw * BNO055_ACCEL_SCALE;
    body->accel[0] =  ax;      /* body X (forward) */
    body->accel[1] = -az;      /* body Y (left) */
    body->accel[2] = -ay;      /* body Z (up) */

    /* Quaternion → normalized, 본체 마운팅 보정 (gyro와 동일 축 매핑) */
    float qw = qw_raw * BNO055_QUAT_SCALE;
    float qx = qx_raw * BNO055_QUAT_SCALE;
    float qy = qy_raw * BNO055_QUAT_SCALE;
    float qz = qz_raw * BNO055_QUAT_SCALE;

    /* 축 swap: BNO(x,y,z) → body(x,-z,-y) */
    float bqx =  qx;
    float bqy = -qz;
    float bqz = -qy;

    float norm = sqrtf(qw * qw + bqx * bqx + bqy * bqy + bqz * bqz);
    if (norm < QUAT_NORM_MIN) {
        body->quat[0] = 1.0f;
        body->quat[1] = 0.0f;
        body->quat[2] = 0.0f;
        body->quat[3] = 0.0f;
    } else {
        float inv = 1.0f / norm;
        body->quat[0] = qw  * inv;
        body->quat[1] = bqx * inv;
        body->quat[2] = bqy * inv;
        body->quat[3] = bqz * inv;
    }

    body->data_valid = true;
    return true;
}
