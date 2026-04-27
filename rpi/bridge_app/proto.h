/**
 * proto.h — Jetson ↔ BridgeDaemon UDP 와이어 프로토콜
 *
 * 패킷 구조:
 *   [ PktHeader (24B) ][ payload ]
 *
 * Odom/Cmd/Heartbeat는 단일 패킷으로 전송.
 * Image/LiDAR는 MTU(1400B) 단위로 분할 전송.
 */

#pragma once

#include <stdint.h>

/* ─── 데이터 타입 ────────────────────────────────────────────── */
#define PKT_TYPE_IMAGE   0x01
#define PKT_TYPE_LIDAR   0x02
#define PKT_TYPE_ODOM    0x03
#define PKT_TYPE_CMD     0x04   /* RPi5 → Jetson 커맨드 */

/* ─── 커맨드 타입 ────────────────────────────────────────────── */
#define CMD_TYPE_ESTOP      0x01   /* 비상 정지 (즉시) */
#define CMD_TYPE_STOP       0x02   /* 일반 정지 */
#define CMD_TYPE_MOVE       0x03   /* 속도 지령 */
#define CMD_TYPE_HEARTBEAT  0x04   /* RPi → Jetson 생존 확인 */

/* ─── 포트 ───────────────────────────────────────────────────── */
#define BRIDGE_PORT      9000   /* Jetson → RPi5 수신 포트 */
#define JETSON_CMD_PORT  9001   /* RPi5 → Jetson 커맨드 포트 */
#define PC_LINK_PORT     9002   /* PC ↔ RPi5 통신 포트 */

/* ─── Unix Domain Socket 경로 (Qt → RPi5) ───────────────────── */
#define BRIDGE_CMD_SOCK  "/tmp/bridge_cmd.sock"

/* ─── 크기 제한 ─────────────────────────────────────────────── */
#define PROTO_MTU        1400
#define PROTO_PKT_MAX    (sizeof(PktHeader) + PROTO_MTU)

/* ─── 공통 헤더 (모든 패킷, 24B) ────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  type;            /* PKT_TYPE_* */
    uint8_t  robot_id;        /* 로봇 식별자 (0 ~ MAX_ROBOTS-1) */
    uint16_t frag_idx;
    uint16_t frag_total;
    uint16_t payload_len;
    uint32_t frame_id;
    uint32_t payload_offset;
    uint64_t timestamp_us;
} PktHeader;

/* ─── Odom 페이로드 ─────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
} OdomPayload;

/* ─── 커맨드 페이로드 (RPi5 → Jetson) ──────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;
    float    vx;
    float    vy;
    float    omega;
    uint32_t seq;
} CmdPayload;

/* ─── 인입 커맨드 패킷 (Qt Unix Socket / PC UDP:9002 공용) ───── */
typedef struct __attribute__((packed)) {
    uint8_t  robot_id;
    uint8_t  cmd_type;
    float    vx;
    float    vy;
    float    omega;
    uint32_t seq;
} CmdPacket;

/* ─── RPi5 → PC 상태 패킷 (UDP, 주기 송신) ─────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  robot_id;
    uint8_t  connected;      /* 0 or 1 */
    float    x;
    float    y;
    float    theta;
    uint32_t odom_seq;
    uint32_t img_drop;
    uint32_t lidar_drop;
    uint64_t timestamp_us;
} PcStatusPacket;
