#include "imu_bno055.h"

#define BNO055_I2C_ADDR_DEFAULT   0x28
#define BNO055_I2C_ADDR_ALT       0x29
#define BNO055_CHIP_ID            0x00
#define BNO055_EUL_HEADING_LSB    0x1A
#define BNO055_OPR_MODE           0x3D
#define BNO055_OPR_CONFIG         0x00
#define BNO055_OPR_IMUPLUS        0x08
#define BNO055_CHIP_ID_EXPECTED   0xA0

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
    bno_write_byte(hi2c, BNO055_OPR_MODE, BNO055_OPR_IMUPLUS);
    HAL_Delay(20);
    return true;
}

bool bno055_read_body(I2C_HandleTypeDef *hi2c, body_attitude_t *body) {
    uint8_t buf[6];
    if (!bno_read_bytes(hi2c, BNO055_EUL_HEADING_LSB, buf, 6)) return false;
    int16_t h_raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t r_raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t p_raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    /* 본체 마운팅 기준: BNO055 roll↔body pitch, BNO055 pitch↔body roll, 둘 다 부호 반전 */
    body->yaw   = h_raw / 16.0f;
    body->pitch = -(r_raw / 16.0f);
    body->roll  = -(p_raw / 16.0f);
    return true;
}
