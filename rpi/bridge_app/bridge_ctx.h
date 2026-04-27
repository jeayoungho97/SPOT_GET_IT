/**
 * bridge_ctx.h — 스레드 컨텍스트 구조체 공유 헤더
 */

#pragma once

#include <arpa/inet.h>
#include <pthread.h>
#include "shm_def.h"
#include "frag_queue.h"

/* ─── Jetson 주소 테이블 ─────────────────────────────────────── */
typedef struct {
    pthread_mutex_t    mu;
    struct sockaddr_in addr[MAX_ROBOTS];
    int                set[MAX_ROBOTS];
} JetsonAddrTable;

/* ─── jetson_rx ─────────────────────────────────────────────── */
typedef struct {
    SharedData      *shm_arr[MAX_ROBOTS];
    FragQueue       *fq_arr[MAX_ROBOTS];
    JetsonAddrTable *addr_table;
    int              num_robots;
    volatile int     stop;
} JetsonRxCtx;

/* ─── jetson_tx ─────────────────────────────────────────────── */
typedef struct {
    JetsonAddrTable *addr_table;
    int              num_robots;
    volatile int     stop;
} JetsonTxCtx;

/* ─── protocol_timer ────────────────────────────────────────── */
typedef struct {
    JetsonAddrTable *addr_table;
    int              num_robots;
    volatile int     stop;
} ProtoTimerCtx;

/* ─── pc_link ───────────────────────────────────────────────── */
typedef struct {
    JetsonAddrTable *addr_table;
    SharedData      *shm_arr[MAX_ROBOTS];
    int              num_robots;
    volatile int     stop;
} PcLinkCtx;

/* ─── reassembly_shm ────────────────────────────────────────── */
typedef struct {
    SharedData *shm;
    FragQueue  *fq;
} ReasmCtx;
