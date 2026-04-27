/**
 * jetson_rx.c — Jetson UDP 수신 스레드
 *
 * 역할:
 *   1. UDP 소켓 단 1개에서 모든 로봇의 패킷을 수신
 *   2. 헤더의 robot_id로 대상 로봇 판별 (O(1) 배열 인덱스)
 *   3. 최초 수신 시 Jetson IP를 addr_table에 등록 (jetson_tx가 참조)
 *   4. 타입별 dispatch:
 *      - ODOM  → shm_arr[robot_id] 에 직접 기록
 *      - IMAGE / LIDAR → fq_arr[robot_id] 에 push
 *
 * 우선순위: ★★★★ (커널 수신 버퍼 overflow 방지)
 */

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "proto.h"
#include "shm_def.h"
#include "frag_queue.h"
#include "bridge_ctx.h"

/* ─── 소켓 생성 ─────────────────────────────────────────────── */
static int create_jetson_socket(void) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("[jetson_rx] socket"); return -1; }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* 수신 버퍼 8MB: 로봇 10대 버스트 대비 */
    int rcvbuf = 8 * 1024 * 1024;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    /* recv 타임아웃 100ms: stop 체크 가능 */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(BRIDGE_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[jetson_rx] bind"); close(fd); return -1;
    }
    return fd;
}

/* ─── Jetson IP 학습 ─────────────────────────────────────────── */
static void learn_addr(JetsonAddrTable *tbl, uint8_t rid,
                       const struct sockaddr_in *src) {
    pthread_mutex_lock(&tbl->mu);
    if (!tbl->set[rid]) {
        tbl->addr[rid]          = *src;
        tbl->addr[rid].sin_port = htons(JETSON_CMD_PORT);
        tbl->set[rid]           = 1;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &src->sin_addr, ip, sizeof(ip));
        fprintf(stderr, "[jetson_rx] robot=%u 주소 등록: %s\n", rid, ip);
    }
    pthread_mutex_unlock(&tbl->mu);
}

/* ─── Odom 직접 SHM 기록 ────────────────────────────────────── */
static void handle_odom(SharedData *shm, const PktHeader *hdr,
                        const uint8_t *payload, int plen) {
    if (plen < (int)sizeof(OdomPayload)) return;
    const OdomPayload *o = (const OdomPayload *)payload;

    pthread_rwlock_wrlock(&shm->odom_lock);
    shm->odom_x            = o->x;
    shm->odom_y            = o->y;
    shm->odom_theta        = o->theta;
    shm->odom_vx           = o->vx;
    shm->odom_vy           = o->vy;
    shm->odom_omega        = o->omega;
    shm->odom_timestamp_us = hdr->timestamp_us;
    shm->odom_seq          = hdr->frame_id;
    pthread_rwlock_unlock(&shm->odom_lock);
}

/* ─── 수신 스레드 메인 ──────────────────────────────────────── */
void *jetson_rx_thread(void *arg) {
    JetsonRxCtx *ctx = (JetsonRxCtx *)arg;

    int fd = create_jetson_socket();
    if (fd < 0) {
        fprintf(stderr, "[jetson_rx] 소켓 생성 실패\n");
        return NULL;
    }
    fprintf(stderr, "[jetson_rx] 포트 %d 수신 대기 (최대 %d대)\n",
            BRIDGE_PORT, ctx->num_robots);

    /* 단일 스레드 전용 버퍼 (1424B — 스택 할당 안전) */
    uint8_t pkt[PROTO_PKT_MAX];

    while (!ctx->stop) {
        struct sockaddr_in src_addr;
        socklen_t addrlen = sizeof(src_addr);

        /* recvfrom: 패킷 수신 + 송신자 IP 획득 */
        ssize_t n = recvfrom(fd, pkt, sizeof(pkt), 0,
                             (struct sockaddr *)&src_addr, &addrlen);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (errno == EINTR) continue;
            perror("[jetson_rx] recvfrom");
            continue;
        }
        if (n == 0) continue;
        if (n < (ssize_t)sizeof(PktHeader)) continue;

        const PktHeader *hdr     = (const PktHeader *)pkt;
        const uint8_t   *payload = pkt + sizeof(PktHeader);
        int              plen    = (int)n - (int)sizeof(PktHeader);

        /* ── robot_id 검증 ──────────────────────────────────── */
        uint8_t rid = hdr->robot_id;
        if (rid >= (uint8_t)ctx->num_robots) {
            fprintf(stderr, "[jetson_rx] 범위 밖 robot_id=%u (max=%d)\n",
                    rid, ctx->num_robots);
            continue;
        }

        /* ── Jetson IP 학습 (최초 1회) ──────────────────────── */
        learn_addr(ctx->addr_table, rid, &src_addr);

        SharedData *shm = ctx->shm_arr[rid];
        FragQueue  *fq  = ctx->fq_arr[rid];

        /* ── Watchdog: 연결 상태 갱신 (lock-free) ───────────── */
        atomic_store(&shm->meta.jetson_connected, 1);
        atomic_fetch_add(&shm->meta.pkt_count, 1);  /* watchdog 연결 감지용 */

        /* ── 타입별 dispatch ────────────────────────────────── */
        switch (hdr->type) {
        case PKT_TYPE_ODOM:
            handle_odom(shm, hdr, payload, plen);
            break;
        case PKT_TYPE_IMAGE:
        case PKT_TYPE_LIDAR:
            frag_queue_push(fq, pkt, (int)n);
            break;
        default:
            fprintf(stderr, "[jetson_rx] 알 수 없는 타입: 0x%02x (robot=%u)\n",
                    hdr->type, rid);
            break;
        }
    }

    /* 종료 시 모든 FragQueue에 stop 전파 */
    for (int i = 0; i < ctx->num_robots; i++)
        frag_queue_stop(ctx->fq_arr[i]);

    close(fd);
    fprintf(stderr, "[jetson_rx] 종료\n");
    return NULL;
}
