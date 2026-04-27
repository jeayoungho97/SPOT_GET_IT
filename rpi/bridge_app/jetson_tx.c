/**
 * jetson_tx.c — Jetson 커맨드 송신 스레드
 *
 * 역할:
 *   1. Unix Domain Socket 서버로 Qt 앱의 연결을 수락
 *   2. Qt에서 CmdPacket 수신
 *   3. send_cmd_to_jetson()으로 Jetson에 UDP 전달
 *
 * 흐름:
 *   Qt ──Unix Socket──▶ jetson_tx ──UDP:9001──▶ Jetson
 *
 * 우선순위: ★★★★★
 *   정지/비상 명령 지연은 허용 불가.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "proto.h"
#include "shm_def.h"
#include "bridge_ctx.h"
#include "cmd_dispatch.h"

/* ─── UDP 소켓 생성 (송신 전용) ─────────────────────────────── */
static int create_udp_sock(void) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("[jetson_tx] UDP socket"); return -1; }
    return fd;
}

/* ─── Unix Domain Socket 서버 생성 ──────────────────────────── */
static int create_unix_server(void) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("[jetson_tx] unix socket"); return -1; }

    /* 이전 소켓 파일 정리 */
    unlink(BRIDGE_CMD_SOCK);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, BRIDGE_CMD_SOCK, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[jetson_tx] unix bind"); close(fd); return -1;
    }
    if (listen(fd, 1) < 0) {
        perror("[jetson_tx] unix listen"); close(fd); return -1;
    }
    return fd;
}

/* ─── Qt 연결 처리 루프 ─────────────────────────────────────── */
static void handle_qt_connection(JetsonTxCtx *ctx, int cli_fd, int udp_fd) {
    /* 100ms 수신 타임아웃: stop 체크 가능 */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(cli_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    fprintf(stderr, "[jetson_tx] Qt 연결 수락\n");

    CmdPacket cmd;
    while (!ctx->stop) {
        /* MSG_WAITALL: sizeof(CmdPacket) 전부 올 때까지 대기 */
        ssize_t n = recv(cli_fd, &cmd, sizeof(cmd), MSG_WAITALL);
        if (n == 0) {
            fprintf(stderr, "[jetson_tx] Qt 연결 종료\n");
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue; /* timeout */
            if (errno == EINTR) continue;
            perror("[jetson_tx] recv");
            break;
        }
        if (n != (ssize_t)sizeof(cmd)) continue; /* 짧은 패킷 버림 */

        send_cmd_to_jetson(&cmd, ctx->addr_table, udp_fd,
                           ctx->num_robots, "jetson_tx");
    }
}

/* ─── 송신 스레드 메인 ──────────────────────────────────────── */
void *jetson_tx_thread(void *arg) {
    JetsonTxCtx *ctx = (JetsonTxCtx *)arg;

    int udp_fd = create_udp_sock();
    if (udp_fd < 0) return NULL;

    int srv_fd = create_unix_server();
    if (srv_fd < 0) { close(udp_fd); return NULL; }

    fprintf(stderr, "[jetson_tx] Unix 소켓 대기: %s\n", BRIDGE_CMD_SOCK);

    while (!ctx->stop) {
        /* select로 100ms마다 stop 체크 */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv_fd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        int r = select(srv_fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("[jetson_tx] select");
            break;
        }
        if (r == 0) continue; /* timeout: stop 재확인 */

        int cli_fd = accept(srv_fd, NULL, NULL);
        if (cli_fd < 0) {
            if (errno == EINTR) continue;
            perror("[jetson_tx] accept");
            continue;
        }

        handle_qt_connection(ctx, cli_fd, udp_fd);
        close(cli_fd);
    }

    close(srv_fd);
    close(udp_fd);
    unlink(BRIDGE_CMD_SOCK);
    fprintf(stderr, "[jetson_tx] 종료\n");
    return NULL;
}
