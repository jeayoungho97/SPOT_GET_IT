#include "imu_bno055.h"
#include <math.h>
#include <stdio.h>

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
 * 목표: chip output 을 RL 학습 컨벤션 (FLU body frame + 표준 proper accel)
 *       으로 출력. STM/ROS 측 변환 모두 identity 유지.
 *
 * 학습 컨벤션 (legged_gym/legged_robot.py:518, :533):
 *   - body frame: X=forward, Y=left, Z=up
 *   - 정자세 IMU accel = (0, 0, +9.8)   (proper accel, 중력 반대 방향)
 *   - projected_gravity = -accel / |accel| = (0, 0, -1)
 *
 * 실제 마운트 (chip 0x09+0x02 임시 빌드의 3자세 측정으로 역산):
 *   정자세                  -> 중력 along +phys_Z
 *   왼쪽 눕힘 (left 바닥)    -> 중력 along +phys_X
 *   머리 위 (forward 하늘)   -> 중력 along -phys_Y
 *
 * 도출된 remap (학습 컨벤션 출력 만들기):
 *   out_X = -phys_Y   (정자세 0,    왼쪽 0,    머리위 +9.8)
 *   out_Y = -phys_X   (정자세 0,    왼쪽 -9.8, 머리위 0)
 *   out_Z = +phys_Z   (정자세 +9.8, 왼쪽 0,    머리위 0)
 *
 *   AXIS_MAP_CONFIG bits [5:4]=NEW_X, [3:2]=NEW_Y, [1:0]=NEW_Z
 *     00=physical X, 01=physical Y, 10=physical Z
 *   -> NEW_X=01(Y), NEW_Y=00(X), NEW_Z=10(Z) = 0b00_01_00_10 = 0x12
 *   AXIS_MAP_SIGN bit2=X_neg, bit1=Y_neg, bit0=Z_neg
 *   -> X,Y negate, Z positive = 0b00000_110 = 0x06
 *
 * 참고: datasheet table 3-7 의 표준 P0~P7 placement 어디에도 해당하지 않는
 *       custom orientation. 측정 기반 직접 계산값.
 *
 * 효과: chip 이 학습 frame 으로 직접 출력 → SW remap 불필요.
 */
#define BNO055_AXIS_MAP_CONFIG    0x41
#define BNO055_AXIS_MAP_SIGN      0x42
#define BNO055_AXIS_CONFIG        0x12
#define BNO055_AXIS_SIGN          0x06

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
    bno_write_byte(hi2c, BNO055_AXIS_MAP_CONFIG, BNO055_AXIS_CONFIG);
    HAL_Delay(10);
    bno_write_byte(hi2c, BNO055_AXIS_MAP_SIGN, BNO055_AXIS_SIGN);
    HAL_Delay(10);

    /* === 진단: P6 remap register read-back (boot 1회만) ===
     * write 가 실제 chip 에 들어갔는지 확인.
     * IMUPLUS 모드 진입 전이므로 register access 안전. */
    uint8_t cfg_rb = 0xFF, sign_rb = 0xFF;
    bool cfg_ok  = bno_read_byte(hi2c, BNO055_AXIS_MAP_CONFIG, &cfg_rb);
    bool sign_ok = bno_read_byte(hi2c, BNO055_AXIS_MAP_SIGN, &sign_rb);
    printf("[BNO055] AXIS_MAP_CONFIG read-back: 0x%02X (expected 0x%02X, read_ok=%d)\r\n",
           cfg_rb, BNO055_AXIS_CONFIG, (int)cfg_ok);
    printf("[BNO055] AXIS_MAP_SIGN   read-back: 0x%02X (expected 0x%02X, read_ok=%d)\r\n",
           sign_rb, BNO055_AXIS_SIGN, (int)sign_ok);

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

    /* Gyro → rad/s. Chip P6 remap 적용 후 출력이 이미 body frame.
     * SW remap 불필요 — chip output 그대로 사용 (identity). */
    float gx = gx_raw * BNO055_GYRO_SCALE;
    float gy = gy_raw * BNO055_GYRO_SCALE;
    float gz = gz_raw * BNO055_GYRO_SCALE;
    body->gyro[0] = gx;
    body->gyro[1] = gy;
    body->gyro[2] = gz;

    /* Accel → m/s². Chip P6 remap 후 출력이 이미 body frame.
     * 정자세 검증: az ≈ -9.8 (Z up → 중력은 -Z 방향). */
    float ax = ax_raw * BNO055_ACCEL_SCALE;
    float ay = ay_raw * BNO055_ACCEL_SCALE;
    float az = az_raw * BNO055_ACCEL_SCALE;

    /* === 진단: chip output (= body frame), 1Hz 출력 ===
     * 다른 자세 (왼쪽 눕힘, 머리 위) 검증 후 제거 예정. */
    static uint32_t last_print_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    if (now_tick - last_print_tick >= 1000) {
        last_print_tick = now_tick;
        printf("[BNO055-RAW] ax=%+.2f ay=%+.2f az=%+.2f\r\n",
               (double)ax, (double)ay, (double)az);
    }

    body->accel[0] = ax;
    body->accel[1] = ay;
    body->accel[2] = az;

    /* Quaternion → normalized. Chip P6 remap 후 출력이 이미 body frame. */
    float qw = qw_raw * BNO055_QUAT_SCALE;
    float qx = qx_raw * BNO055_QUAT_SCALE;
    float qy = qy_raw * BNO055_QUAT_SCALE;
    float qz = qz_raw * BNO055_QUAT_SCALE;

    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < QUAT_NORM_MIN) {
        body->quat[0] = 1.0f;
        body->quat[1] = 0.0f;
        body->quat[2] = 0.0f;
        body->quat[3] = 0.0f;
    } else {
        float inv = 1.0f / norm;
        body->quat[0] = qw * inv;
        body->quat[1] = qx * inv;
        body->quat[2] = qy * inv;
        body->quat[3] = qz * inv;
    }

    body->data_valid = true;
    return true;
}
