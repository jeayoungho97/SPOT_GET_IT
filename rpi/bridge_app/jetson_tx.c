/**
 * jetson_tx.c — Jetson 커맨드 송신 스레드
 *
 * 역할:
 *   1. Qt가 SharedData.cmd_queue에 넣은 CmdPacket을 읽음
 *   2. send_cmd_to_jetson()으로 Jetson에 UDP 전달
 *
 * 흐름:
 *   Qt ──SHM cmd_queue──▶ jetson_tx ──UDP:9001──▶ Jetson
 *
 * 우선순위: ★★★★★
 *   정지/비상 명령 지연은 허용 불가.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "proto.h"
#include "shm_def.h"
#include "bridge_ctx.h"
#include "cmd_dispatch.h"
#include "bridge_api.h"
#include "shm_cmd_queue.h"

/* ─── UDP 소켓 생성 (송신 전용) ─────────────────────────────── */
static int create_udp_sock(void) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("[jetson_tx] UDP socket"); return -1; }
    return fd;
}

static void send_shm_cmd_entry(JetsonTxCtx *ctx, int udp_fd,
                               const ShmCmdEntry *entry) {
    uint8_t priority = shm_cmd_entry_effective_priority(entry);
    uint8_t flags = entry->flags;
    send_cmd_to_jetson(&entry->cmd, ctx->api, udp_fd,
                       priority, flags, "jetson_tx");
}

/* ─── 송신 스레드 메인 ──────────────────────────────────────── */
void *jetson_tx_thread(void *arg) {
    JetsonTxCtx *ctx = (JetsonTxCtx *)arg;

    int udp_fd = create_udp_sock();
    if (udp_fd < 0) return NULL;

    fprintf(stderr, "[jetson_tx] SHM command queue 대기\n");

    int next_robot = 0;

    while (!atomic_load_explicit(ctx->stop, memory_order_acquire)) {
        int handled = 0;
        bridge_api_poll_timeouts(ctx->api, udp_fd, "jetson_tx");

        for (int offset = 0; offset < ctx->num_robots; offset++) {
            int rid = (next_robot + offset) % ctx->num_robots;
            ShmCmdQueue *q = &ctx->shm_arr[rid]->cmd_queue;
            ShmCmdEntry entry;
            if (shm_cmd_queue_pop_highest(q, &entry)) {
                send_shm_cmd_entry(ctx, udp_fd, &entry);
                next_robot = (rid + 1) % ctx->num_robots;
                handled = 1;
                break;
            }
        }

        if (!handled)
            usleep(5000);
    }

    close(udp_fd);
    fprintf(stderr, "[jetson_tx] 종료\n");
    return NULL;
}
