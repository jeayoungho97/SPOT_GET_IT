#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_state.h"

/*
 * SPI 프레임 정의 — Jetson actuator_bridge와 binary-compatible.
 * Full-duplex 261 byte, 50 Hz, SPI Mode 0, 5 MHz.
 * Endianness: little-endian (양쪽 ARM).
 */

#define SPI_NUM_JOINTS          12

#define SPI_MOSI_MAGIC          0xA55A
#define SPI_MISO_MAGIC          0x5AA5

/* MOSI payload size (magic ~ crc16, padding 제외) */
#define MOSI_PAYLOAD_SIZE       116

/* MISO payload size = SPI_FRAME_SIZE (패딩 없음) */
#define MISO_PAYLOAD_SIZE       261

/* SPI full-duplex: 양쪽 중 큰 쪽(MISO)에 맞춤 */
#define SPI_FRAME_SIZE          MISO_PAYLOAD_SIZE

/* Jetson -> STM wire mode
 * STM 의 실제 행동은 두 가지뿐 — torque off / target 따라가기.
 * 의미적 모드 (STAND/RL/CROUCH 등 robot_interfaces/JointTarget.msg) 는
 * Jetson 측 bridge 가 OPERATE 로 매핑하고 자세 차이는 target_rad 로 결정.
 * E_STOP 은 별도 wire mode 가 아니라 SPI_FLAG_E_STOP 비트로 전달.
 */
#define SPI_MODE_DISABLE        0   /* torque off — safe state */
#define SPI_MODE_OPERATE        1   /* torque on + apply target_rad */

/* STM -> Jetson motion_state (feedback)
 * STM 은 RL 이 보낸 target 만 따라가므로 motion direction 은 모름.
 * 현재 STM 이 set 할 수 있는 두 값만 정의. 나머지 WALK_FORWARD/TURN_LEFT/...
 * 등은 robot_interfaces/msg/StmMotion.msg 가 정의하지만 RL/high-level 노드가
 * 별도 publish 해야 함 (STM 영역 아님).
 */
#define MOTION_STATE_STOP       0
#define MOTION_STATE_UNKNOWN    8

/* flags 비트 */
#define SPI_FLAG_TORQUE_EN      (1 << 0)
#define SPI_FLAG_E_STOP         (1 << 3)

/* MOSI: Jetson -> STM (116 payload + 145 padding = 261) */
typedef struct __attribute__((packed)) {
    uint16_t magic;                             /* 0xA55A */
    uint16_t seq;
    uint32_t timestamp_us;
    uint8_t  mode;
    uint8_t  flags;
    float    target_rad[SPI_NUM_JOINTS];        /* 48 */
    float    max_delta_rad[SPI_NUM_JOINTS];     /* 48 */
    float    gait_phase;                        /* 4  */
    uint32_t gait_cycle_count;                  /* 4  */
    uint16_t crc16;
    uint8_t  _pad[145];
} spi_mosi_frame_t;

/* MISO: STM -> Jetson (261 byte, 패딩 없음) */
typedef struct __attribute__((packed)) {
    uint16_t magic;                             /* 0x5AA5 */
    uint16_t seq_echo;
    uint32_t timestamp_us;
    uint8_t  status;
    uint8_t  fault_code;
    uint8_t  motion_state;                      /* 1  */
    float    gait_phase;                        /* 4  — echo from MOSI */
    uint32_t gait_cycle_count;                  /* 4  — echo from MOSI */
    float    imu_yaw_rad;                       /* 4  */
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

/* === Decode 결과 === */
typedef enum {
    DECODE_OK = 0,
    DECODE_BAD_MAGIC,
    DECODE_BAD_CRC,
} decode_result_t;

/* CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF) */
uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t length);

/* MOSI 수신 buffer -> g_robot_state 갱신 */
decode_result_t spi_decode_command(const uint8_t *rx_buffer);

/* g_robot_state -> MISO 송신 buffer 인코딩 */
void spi_encode_feedback(uint8_t *tx_buffer);

#endif /* SPI_PROTOCOL_H */
