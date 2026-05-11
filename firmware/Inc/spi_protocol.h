#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>

/*
 * SPI 프레임 정의 — Jetson actuator_bridge와 binary-compatible.
 * Full-duplex 248 byte, 50 Hz, SPI Mode 0, 5 MHz.
 * Endianness: little-endian (양쪽 ARM).
 */

#define SPI_FRAME_SIZE          248
#define SPI_NUM_JOINTS          12

#define SPI_MOSI_MAGIC          0xA55A
#define SPI_MISO_MAGIC          0x5AA5

/* Jetson → STM 모드 */
#define SPI_MODE_IDLE           0
#define SPI_MODE_POSITION       1
#define SPI_MODE_CAL            2
#define SPI_MODE_HOLD           3

/* flags 비트 */
#define SPI_FLAG_TORQUE_EN      (1 << 0)
#define SPI_FLAG_E_STOP         (1 << 3)

/* MOSI: Jetson → STM (108 payload + 140 padding = 248) */
typedef struct __attribute__((packed)) {
    uint16_t magic;                             /* 0xA55A */
    uint16_t seq;
    uint32_t timestamp_us;
    uint8_t  mode;
    uint8_t  flags;
    float    target_rad[SPI_NUM_JOINTS];        /* 48 */
    float    max_delta_rad[SPI_NUM_JOINTS];     /* 48 */
    uint16_t crc16;
    uint8_t  _pad[140];
} spi_mosi_frame_t;

/* MISO: STM → Jetson (248 payload, 패딩 없음) */
typedef struct __attribute__((packed)) {
    uint16_t magic;                             /* 0x5AA5 */
    uint16_t seq_echo;
    uint32_t timestamp_us;
    uint8_t  status;
    uint8_t  fault_code;
    float    position_rad[SPI_NUM_JOINTS];      /* 48 */
    float    velocity_rad_s[SPI_NUM_JOINTS];    /* 48 */
    float    load_or_current[SPI_NUM_JOINTS];   /* 48 */
    float    temperature[SPI_NUM_JOINTS];       /* 48 */
    float    gyro_rad_s[3];                     /* 12 */
    float    accel_m_s2[3];                     /* 12 */
    float    quat_wxyz[4];                      /* 16 */
    float    bus_voltage;                       /* 4  */
    uint16_t crc16;                             /* 2  */
} spi_miso_frame_t;

_Static_assert(sizeof(spi_mosi_frame_t) == SPI_FRAME_SIZE, "MOSI frame size mismatch");
_Static_assert(sizeof(spi_miso_frame_t) == SPI_FRAME_SIZE, "MISO frame size mismatch");

#endif /* SPI_PROTOCOL_H */
