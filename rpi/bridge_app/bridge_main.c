/**
 * bridge_main.c — BridgeDaemon 메인
 *
 * 스레드 구성 (로봇 N대 기준):
 *   main/watchdog      ★★     (1개)
 *   jetson_rx          ★★★★   (1개, 코어2 고정)
 *   reassembly_shm     ★★★    (N개, OS 스케줄)
 *   jetson_tx          ★★★★★  (1개, OS 스케줄)
 *   protocol_timer     ★★★★★  (1개, OS 스케줄)
 *   pc_link            ★★★    (1개, OS 스케줄)
 *
 * 실행 인자:
 *   ./bridge_app [num_robots]   기본값: 1, 최대: MAX_ROBOTS
 */

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>

#include "shm_def.h"
#include "frag_queue.h"
#include "proto.h"
#include "bridge_ctx.h"

/* ─── 외부 스레드 함수 선언 ─────────────────────────────────── */
extern void *jetson_rx_thread(void *arg);
extern void *jetson_tx_thread(void *arg);
extern void *reassembly_shm_thread(void *arg);
extern void *protocol_timer_thread(void *arg);
extern void *pc_link_thread(void *arg);

/* ─── 전역 컨텍스트 ─────────────────────────────────────────── */
typedef struct {
    int              num_robots;
    SharedData      *shm[MAX_ROBOTS];
    FragQueue        fq[MAX_ROBOTS];
    volatile int     stop;

    JetsonAddrTable  addr_table;
    JetsonRxCtx      rx_ctx;
    JetsonTxCtx      tx_ctx;
    ProtoTimerCtx    timer_ctx;
    PcLinkCtx        pc_ctx;
    ReasmCtx         reasm_ctx[MAX_ROBOTS];

    pthread_t        rx_tid;
    pthread_t        tx_tid;
    pthread_t        timer_tid;
    pthread_t        pc_tid;
    pthread_t        reasm_tid[MAX_ROBOTS];
} BridgeCtx;

static BridgeCtx g_ctx;

/* ─── SHM 초기화 (로봇 1대분) ──────────────────────────────── */
static SharedData *shm_init_one(int robot_id) {
    char name[32];
    snprintf(name, sizeof(name), SHM_NAME_FMT, robot_id);
    // SHM_NAME_FMT( /robot_bridge_%d" shm_def.h에 정의됨)에 %d 부분에 robot_id를 넣어서 shm 파일 이름("name") 생성
    

    shm_unlink(name);

    int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return NULL; }

    if (ftruncate(fd, sizeof(SharedData)) < 0) {
        perror("ftruncate"); close(fd); shm_unlink(name); return NULL;
    }

    SharedData *shm = mmap(NULL, sizeof(SharedData),
                           PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (shm == MAP_FAILED) {
        perror("mmap"); shm_unlink(name); return NULL;
    }

    memset(shm, 0, sizeof(SharedData));
    atomic_store(&shm->img_ready_idx,   -1);
    atomic_store(&shm->lidar_ready_idx, -1);

    if (sem_init(&shm->img_sem, 1, 0) < 0) {
        perror("sem_init img");
        munmap(shm, sizeof(SharedData)); shm_unlink(name); return NULL;
    }
    if (sem_init(&shm->lidar_sem, 1, 0) < 0) {
        perror("sem_init lidar");
        sem_destroy(&shm->img_sem);
        munmap(shm, sizeof(SharedData)); shm_unlink(name); return NULL;
    }

    pthread_rwlockattr_t rw_attr;
    pthread_rwlockattr_init(&rw_attr);
    pthread_rwlockattr_setpshared(&rw_attr, PTHREAD_PROCESS_SHARED);
    pthread_rwlock_init(&shm->odom_lock, &rw_attr);
    pthread_rwlock_init(&shm->meta_lock, &rw_attr);
    pthread_rwlockattr_destroy(&rw_attr);

    fprintf(stderr, "[main] SHM 초기화: %s (%zu KB)\n",
            name, sizeof(SharedData) / 1024);
    return shm;
}

/* ─── SHM 정리 (로봇 1대분) ─────────────────────────────────── */
static void shm_cleanup_one(SharedData *shm, int robot_id) {
    if (!shm) return;
    sem_destroy(&shm->img_sem);
    sem_destroy(&shm->lidar_sem);
    pthread_rwlock_destroy(&shm->odom_lock);
    pthread_rwlock_destroy(&shm->meta_lock);
    munmap(shm, sizeof(SharedData));
    char name[32];
    snprintf(name, sizeof(name), SHM_NAME_FMT, robot_id);
    shm_unlink(name);
}

/* ─── 전체 자원 정리 ────────────────────────────────────────── */
static void cleanup_all(int num_robots) {
    for (int i = 0; i < num_robots; i++) {
        frag_queue_destroy(&g_ctx.fq[i]);
        shm_cleanup_one(g_ctx.shm[i], i);
    }
    pthread_mutex_destroy(&g_ctx.addr_table.mu);
}

/* ─── 스레드 생성 헬퍼 ──────────────────────────────────────── */
static pthread_t create_thread(void *(*fn)(void *), void *arg,
                                int priority, int core) {
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    if (core >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
    }
    if (priority > 0) {
        struct sched_param param = { .sched_priority = priority };
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &param);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    }

    pthread_t tid;
    if (pthread_create(&tid, &attr, fn, arg) != 0) {
        perror("pthread_create"); tid = 0;
    }
    pthread_attr_destroy(&attr);
    return tid;
}

/* ─── 시그널 핸들러 ─────────────────────────────────────────── */
static void sig_handler(int sig) {
    (void)sig;
    fprintf(stderr, "\n[main] 종료 신호 수신\n");
    g_ctx.stop             = 1;
    g_ctx.rx_ctx.stop      = 1;
    g_ctx.tx_ctx.stop      = 1;
    g_ctx.timer_ctx.stop   = 1;
    g_ctx.pc_ctx.stop      = 1;
    for (int i = 0; i < g_ctx.num_robots; i++)
        frag_queue_stop(&g_ctx.fq[i]);
}

/* ─── watchdog ──────────────────────────────────────────────── */
static void watchdog_loop(void) {
    int prev_pkt_count[MAX_ROBOTS] = {0};
    int no_pkt_sec[MAX_ROBOTS]     = {0};

    while (!g_ctx.stop) {
        sleep(1);
        for (int i = 0; i < g_ctx.num_robots; i++) {
            SharedData *shm = g_ctx.shm[i];

            uint32_t img_drop, lidar_drop;
            pthread_rwlock_rdlock(&shm->meta_lock);
            img_drop   = shm->meta.img_drop_count;
            lidar_drop = shm->meta.lidar_drop_count;
            pthread_rwlock_unlock(&shm->meta_lock);

            uint8_t connected = atomic_load(&shm->meta.jetson_connected);

            /* odom뿐 아니라 모든 패킷 수신 여부로 연결 판단 */
            int cur_pkt = atomic_load(&shm->meta.pkt_count);
            if (cur_pkt == prev_pkt_count[i]) {
                if (++no_pkt_sec[i] >= 3)
                    atomic_store(&shm->meta.jetson_connected, 0);
            } else {
                no_pkt_sec[i] = 0;
            }
            prev_pkt_count[i] = cur_pkt;

            fprintf(stderr,
                "[watchdog] robot=%d connected=%d img_drop=%u lidar_drop=%u\n",
                i, (int)connected, img_drop, lidar_drop);
        }
    }
}

/* ─── main ───────────────────────────────────────────────────── */
int main(int argc, char *argv[]) {
    umask(0);
    int num_robots = (argc >= 2) ? atoi(argv[1]) : 1;
    if (num_robots < 1 || num_robots > MAX_ROBOTS) {
        fprintf(stderr, "[main] 유효하지 않은 로봇 수: %d (1~%d)\n",
                num_robots, MAX_ROBOTS);
        return 1;
    }
    g_ctx.num_robots = num_robots;

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* addr_table 초기화 */
    memset(&g_ctx.addr_table, 0, sizeof(g_ctx.addr_table));
    pthread_mutex_init(&g_ctx.addr_table.mu, NULL);

    /* SHM + FragQueue 초기화 */
    for (int i = 0; i < num_robots; i++) {
        g_ctx.shm[i] = shm_init_one(i);
        if (!g_ctx.shm[i]) {
            for (int j = 0; j < i; j++) {
                frag_queue_destroy(&g_ctx.fq[j]);
                shm_cleanup_one(g_ctx.shm[j], j);
            }
            pthread_mutex_destroy(&g_ctx.addr_table.mu);
            return 1;
        }
        frag_queue_init(&g_ctx.fq[i]);
    }

    /* 컨텍스트 구성 */
    g_ctx.rx_ctx.num_robots = num_robots;
    g_ctx.rx_ctx.stop       = 0;
    g_ctx.rx_ctx.addr_table = &g_ctx.addr_table;
    for (int i = 0; i < num_robots; i++) {
        g_ctx.rx_ctx.shm_arr[i] = g_ctx.shm[i];
        g_ctx.rx_ctx.fq_arr[i]  = &g_ctx.fq[i];
    }

    g_ctx.tx_ctx.addr_table = &g_ctx.addr_table;
    g_ctx.tx_ctx.num_robots = num_robots;
    g_ctx.tx_ctx.stop       = 0;

    g_ctx.timer_ctx.addr_table = &g_ctx.addr_table;
    g_ctx.timer_ctx.num_robots = num_robots;
    g_ctx.timer_ctx.stop       = 0;

    g_ctx.pc_ctx.addr_table = &g_ctx.addr_table;
    g_ctx.pc_ctx.num_robots = num_robots;
    g_ctx.pc_ctx.stop       = 0;
    for (int i = 0; i < num_robots; i++)
        g_ctx.pc_ctx.shm_arr[i] = g_ctx.shm[i];

    for (int i = 0; i < num_robots; i++) {
        g_ctx.reasm_ctx[i].shm = g_ctx.shm[i];
        g_ctx.reasm_ctx[i].fq  = &g_ctx.fq[i];
    }

    /* 스레드 생성 */
    g_ctx.rx_tid = create_thread(jetson_rx_thread, &g_ctx.rx_ctx, 60, 2);
    if (!g_ctx.rx_tid) { cleanup_all(num_robots); return 1; }

    g_ctx.tx_tid = create_thread(jetson_tx_thread, &g_ctx.tx_ctx, 80, -1);
    if (!g_ctx.tx_tid) {
        g_ctx.rx_ctx.stop = 1;
        for (int i = 0; i < num_robots; i++) frag_queue_stop(&g_ctx.fq[i]);
        pthread_join(g_ctx.rx_tid, NULL);
        cleanup_all(num_robots); return 1;
    }

    g_ctx.timer_tid = create_thread(protocol_timer_thread,
                                    &g_ctx.timer_ctx, 80, -1);
    if (!g_ctx.timer_tid) {
        g_ctx.rx_ctx.stop = g_ctx.tx_ctx.stop = 1;
        for (int i = 0; i < num_robots; i++) frag_queue_stop(&g_ctx.fq[i]);
        pthread_join(g_ctx.rx_tid, NULL);
        pthread_join(g_ctx.tx_tid, NULL);
        cleanup_all(num_robots); return 1;
    }

    g_ctx.pc_tid = create_thread(pc_link_thread, &g_ctx.pc_ctx, 40, -1);
    if (!g_ctx.pc_tid) {
        g_ctx.rx_ctx.stop = g_ctx.tx_ctx.stop = g_ctx.timer_ctx.stop = 1;
        for (int i = 0; i < num_robots; i++) frag_queue_stop(&g_ctx.fq[i]);
        pthread_join(g_ctx.rx_tid, NULL);
        pthread_join(g_ctx.tx_tid, NULL);
        pthread_join(g_ctx.timer_tid, NULL);
        cleanup_all(num_robots); return 1;
    }

    for (int i = 0; i < num_robots; i++) {
        g_ctx.reasm_tid[i] = create_thread(reassembly_shm_thread,
                                           &g_ctx.reasm_ctx[i], 0, -1);
        if (!g_ctx.reasm_tid[i]) {
            g_ctx.stop = g_ctx.rx_ctx.stop = g_ctx.tx_ctx.stop = 1;
            g_ctx.timer_ctx.stop = g_ctx.pc_ctx.stop = 1;
            for (int j = 0; j < num_robots; j++) frag_queue_stop(&g_ctx.fq[j]);
            pthread_join(g_ctx.rx_tid, NULL);
            pthread_join(g_ctx.tx_tid, NULL);
            pthread_join(g_ctx.timer_tid, NULL);
            pthread_join(g_ctx.pc_tid, NULL);
            for (int j = 0; j < i; j++) pthread_join(g_ctx.reasm_tid[j], NULL);
            cleanup_all(num_robots); return 1;
        }
    }

    fprintf(stderr,
        "[main] BridgeDaemon 시작 (PID=%d, robots=%d)\n"
        "[main] 수신 포트: %d | 커맨드 포트: %d | PC 포트: %d\n"
        "[main] Qt 소켓: %s\n",
        getpid(), num_robots,
        BRIDGE_PORT, JETSON_CMD_PORT, PC_LINK_PORT,
        BRIDGE_CMD_SOCK);

    watchdog_loop();

    pthread_join(g_ctx.rx_tid,    NULL);
    pthread_join(g_ctx.tx_tid,    NULL);
    pthread_join(g_ctx.timer_tid, NULL);
    pthread_join(g_ctx.pc_tid,    NULL);
    for (int i = 0; i < num_robots; i++)
        pthread_join(g_ctx.reasm_tid[i], NULL);

    cleanup_all(num_robots);
    fprintf(stderr, "[main] BridgeDaemon 완전 종료\n");
    return 0;
}
