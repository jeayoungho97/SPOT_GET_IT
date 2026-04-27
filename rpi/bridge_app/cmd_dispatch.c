/**
 * cmd_dispatch.c — Jetson 커맨드 패킷 송신 공통 구현
 */

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include "cmd_dispatch.h"
#include "utils.h"

void send_cmd_to_jetson(const CmdPacket    *c,
                        JetsonAddrTable    *tbl,
                        int                 udp_fd,
                        int                 num_robots,
                        const char         *tag) {
    uint8_t rid = c->robot_id;
    if (rid >= (uint8_t)num_robots) {
        fprintf(stderr, "[%s] 범위 밖 robot_id=%u\n", tag, rid);
        return;
    }

    /* Jetson 주소 조회 (lock 구간 최소화: sendto는 lock 밖에서) */
    pthread_mutex_lock(&tbl->mu);
    int                has_addr = tbl->set[rid];
    struct sockaddr_in dst      = tbl->addr[rid];
    pthread_mutex_unlock(&tbl->mu);

    if (!has_addr) {
        fprintf(stderr, "[%s] robot=%u 주소 미등록 — "
                "Jetson으로부터 패킷을 아직 받지 못함\n", tag, rid);
        return;
    }

    /* PktHeader + CmdPayload 구성 */
    uint8_t    buf[sizeof(PktHeader) + sizeof(CmdPayload)];
    PktHeader  *hdr = (PktHeader *)buf;
    CmdPayload *cmd = (CmdPayload *)(buf + sizeof(PktHeader));

    hdr->type           = PKT_TYPE_CMD;
    hdr->robot_id       = rid;
    hdr->frag_idx       = 0;
    hdr->frag_total     = 1;
    hdr->payload_len    = (uint16_t)sizeof(CmdPayload);
    hdr->frame_id       = c->seq;
    hdr->payload_offset = 0;
    hdr->timestamp_us   = now_us();

    cmd->cmd_type = c->cmd_type;
    cmd->vx       = c->vx;
    cmd->vy       = c->vy;
    cmd->omega    = c->omega;
    cmd->seq      = c->seq;

    ssize_t sent = sendto(udp_fd, buf, sizeof(buf), 0,
                          (struct sockaddr *)&dst, sizeof(dst));
    if (sent < 0)
        perror("sendto");
    else
        fprintf(stderr, "[%s] robot=%u cmd=0x%02x vx=%.2f vy=%.2f ω=%.2f\n",
                tag, rid, c->cmd_type, c->vx, c->vy, c->omega);
}
