/**
 * reassembly_shm.c — fragment 재조립 → SHM write 스레드
 *
 * 역할:
 *   1. fragment_queue에서 패킷 pop
 *   2. 타입별 재조립 버퍼에 fragment 쌓기
 *   3. 프레임 완성 시:
 *      - 이미지: Triple Buffer 빈 슬롯에 write → atomic index swap → sem_post
 *      - 라이다: Double Buffer 빈 슬롯에 write → atomic index swap → sem_post
 *
 * 슬롯 고갈 방지:
 *   fragment 유실로 완성되지 못한 재조립 버퍼는 타임아웃(100ms)으로 강제 해제
 *   (타임아웃 내 미도착 fragment는 UDP 소실 — 기다려도 오지 않으므로 빠른 슬롯 회수가 최선)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>

#include "proto.h"
#include "shm_def.h"
#include "frag_queue.h"
#include "bridge_ctx.h"
#include "utils.h"

/* ─── 재조립 슬롯 ────────────────────────────────────────────── */
#define REASM_SLOTS      6          /* 동시 재조립 가능한 최대 프레임 수
                                     * 100ms 타임아웃 기준 최대 점유: 이미지 3 + 라이다 1 = 4,
                                     * +2 여유 슬롯 (burst 겹침 대비) */
#define REASM_BUF_MAX (512 * 1024)  /* 3D LiDAR 최대 페이로드 ~128KB, 이미지 ~200KB */
#define REASM_TIMEOUT_US  100000ULL /* 100ms: 로컬 LAN 기준 정상 수신 <30ms,
                                     * fragment 소실 시 빠르게 슬롯 회수 */

typedef struct {
    int      in_use;
    uint8_t  type;
    uint32_t frame_id;
    uint16_t frags_recv;
    uint16_t frags_total;
    uint64_t timestamp_us;   /* 송신 측 타임스탬프 */
    uint64_t first_recv_us;  /* 첫 fragment 수신 시각 (타임아웃 기준) */
    uint32_t total_size;     /* 전체 데이터 크기 */
    uint8_t  buf[REASM_BUF_MAX];
    uint8_t  frag_received[512]; /* 중복 수신 방지 (512: 3D LiDAR ~94 frags + 여유) */
} ReasmSlot;

/* ─── 타임아웃 슬롯 강제 해제 ───────────────────────────────── */
static void expire_stale(ReasmSlot *slots, SharedData *shm) {
    uint64_t now = now_us();
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (slots[i].in_use &&
            (now - slots[i].first_recv_us) > REASM_TIMEOUT_US) {
            fprintf(stderr,
                "[reasm] 타임아웃: type=%d frame_id=%u frags=%u/%u\n",
                slots[i].type, slots[i].frame_id,
                slots[i].frags_recv, slots[i].frags_total);
            slots[i].in_use = 0;

            /* 드롭 카운트 증가 */
            pthread_rwlock_wrlock(&shm->meta_lock);
            if (slots[i].type == PKT_TYPE_IMAGE) shm->meta.img_drop_count++;
            else if (slots[i].type == PKT_TYPE_LIDAR) shm->meta.lidar_drop_count++;
            pthread_rwlock_unlock(&shm->meta_lock);
        }
    }
}

/* ─── 재조립 슬롯 탐색/할당 ─────────────────────────────────── */
static ReasmSlot *find_slot(ReasmSlot *slots, uint8_t type, uint32_t frame_id) {
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (slots[i].in_use &&
            slots[i].type == type &&
            slots[i].frame_id == frame_id)
            return &slots[i];
    }
    return NULL;
}

static ReasmSlot *alloc_slot(ReasmSlot *slots) {
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (!slots[i].in_use) return &slots[i];
    }
    return NULL;
}

/* ─── 이미지 완성 → Triple Buffer write ─────────────────────── */
static void commit_image(SharedData *shm, ReasmSlot *rs) {
    /* 빈 슬롯 찾기 (현재 ready 슬롯 다음 슬롯으로 빙글빙글 순환) */
    int ready = atomic_load(&shm->img_ready_idx);
    int slot = 0;
    if (ready >= 0) {
        slot = (ready + 1) % IMG_SLOTS;
    }

    /* write */
    uint32_t size = rs->total_size;
    if (size > IMG_SLOT_SIZE) {
        fprintf(stderr, "[reasm] 이미지 너무 큼: %u > %d\n",
                size, IMG_SLOT_SIZE);
        return;
    }
    ImgSlot *s = &shm->img_slots[slot];
    memcpy(s->data, rs->buf, size);
    s->size         = size;
    s->frame_id     = rs->frame_id;
    s->timestamp_us = rs->timestamp_us;

    /* atomic index swap → Qt가 atomic_load로 최신 슬롯 확인 */
    atomic_store(&shm->img_ready_idx, slot);

    /* Qt 깨우기 */
    sem_post(&shm->img_sem);
}

/* ─── LiDAR 완성 → Triple Buffer write ─────────────────────── */
static void commit_lidar(SharedData *shm, ReasmSlot *rs) {
    int ready = atomic_load(&shm->lidar_ready_idx);
    int slot  = (ready < 0) ? 0 : (ready + 1) % LIDAR_SLOTS;

    LidarSlot *s = &shm->lidar_slots[slot];

    /*
     * LiDAR 페이로드 형식 (3D):
     *   [ uint32_t count ]
     *   [ float x0 ][ float y0 ][ float z0 ][ float intensity0 ]
     *   [ float x1 ][ float y1 ][ float z1 ][ float intensity1 ]
     *   ...
     *   포인트당 4 floats = 16 bytes
     */
    if (rs->total_size < sizeof(uint32_t)) return;

    uint32_t count;
    memcpy(&count, rs->buf, sizeof(uint32_t));
    if (count > LIDAR_MAX_PTS) {
        fprintf(stderr, "[reasm] LiDAR count 클리핑: %u → %d\n", count, LIDAR_MAX_PTS);
        count = LIDAR_MAX_PTS;
    }

    uint32_t needed = (uint32_t)sizeof(uint32_t) + count * 4 * (uint32_t)sizeof(float);
    if (rs->total_size < needed) {
        fprintf(stderr, "[reasm] LiDAR 버퍼 오버리드 방지: total_size=%u, needed=%u, count=%u\n",
                rs->total_size, needed, count);
        return;
    }

    s->count        = count;
    s->frame_id     = rs->frame_id;
    s->timestamp_us = rs->timestamp_us;

    const float *pts = (const float *)(rs->buf + sizeof(uint32_t));
    for (uint32_t i = 0; i < count; i++) {
        s->x[i]         = pts[i * 4];
        s->y[i]         = pts[i * 4 + 1];
        s->z[i]         = pts[i * 4 + 2];
        s->intensity[i] = pts[i * 4 + 3];
    }

    atomic_store(&shm->lidar_ready_idx, slot);
    sem_post(&shm->lidar_sem);
}

/* ─── 재조립 스레드 메인 ────────────────────────────────────────── */
void *reassembly_shm_thread(void *arg) {
    ReasmCtx   *ctx = (ReasmCtx *)arg;
    SharedData *shm = ctx->shm;
    FragQueue  *fq  = ctx->fq;

    /* static 제거: 다중 스레드에서 공유되면 데이터 오염 발생.
     * ReasmSlot 1개가 ~200KB이므로 스택 할당 대신 힙 할당 사용. */
    ReasmSlot *slots = calloc(REASM_SLOTS, sizeof(ReasmSlot));
    if (!slots) {
        fprintf(stderr, "[reasm] slots 메모리 할당 실패\n");
        return NULL;
    }

    uint8_t *pkt = malloc(PROTO_PKT_MAX);
    if (!pkt) {
        fprintf(stderr, "[reasm] pkt 메모리 할당 실패\n");
        free(slots);
        return NULL;
    }

    fprintf(stderr, "[reasm] 시작\n");

    while (1) {
        // 1. 패킷 큐에서 하나의 조각(Fragment)을 꺼냅니다. 데이터가 없으면 대기(Blocking)합니다.
        int n = frag_queue_pop(fq, pkt, PROTO_PKT_MAX);
        if (n < 0) break;  /* 종료 신호가 들어왔으면 반복문을 빠져나갑니다. */
        if (n < (int)sizeof(PktHeader)) continue;

        const PktHeader *hdr     = (const PktHeader *)pkt;
        const uint8_t   *payload = pkt + sizeof(PktHeader);
        int              plen    = n - (int)sizeof(PktHeader);

        // 2. 가비지 컬렉션: 수신한 지 너무 오래된(100ms) 미완성 조립 프레임들을 폐기합니다.
        expire_stale(slots, shm);

        // 3. 조립 슬롯 탐색: 현재 수신한 조각과 같은 frame_id를 조립 중인 슬롯을 찾습니다.
        ReasmSlot *rs = find_slot(slots, hdr->type, hdr->frame_id);
        if (!rs) {
            // (첫 조각 누락 방어) 해당 frame의 인덱스가 0(첫 조각)이 아니라면, 앞 조각이 유실된 것이므로 이 프레임은 포기합니다.
            if (hdr->frag_idx != 0) continue; 

            // 새 프레임이므로 빈 조립 슬롯을 할당합니다.
            rs = alloc_slot(slots);
            if (!rs) {
                fprintf(stderr, "[reasm] 슬롯 고갈 (frame_id=%u)\n", hdr->frame_id);
                continue; // 빈 슬롯이 4개 다 차있다면 포기합니다.
            }
            
            rs->in_use        = 1;
            rs->type          = hdr->type;
            rs->frame_id      = hdr->frame_id;
            rs->frags_recv    = 0;
            rs->frags_total   = hdr->frag_total;
            rs->timestamp_us  = hdr->timestamp_us;
            rs->first_recv_us = now_us(); // 타임아웃 감시용 시간 저장
            rs->total_size    = 0;
            memset(rs->frag_received, 0, sizeof(rs->frag_received));
            /* rs->buf는 초기화 불필요: offset+plen OOB 검증(아래)으로 안전 보장 */
        }

        // 4. 중복 및 비정상 프래그먼트 방어 로직
        // 전체 조각 수가 0이거나 단일 인덱스가 허용치를 넘었는지 사전에 체크합니다.
        if (hdr->frag_total == 0 || hdr->frag_idx >= hdr->frag_total || hdr->frag_idx >= 512) {
            fprintf(stderr, "[reasm] 잘못된 인덱스 또는 total: idx=%u, total=%u\n", hdr->frag_idx, hdr->frag_total);
            rs->in_use = 0;   /* 방금 alloc한 슬롯이면 반드시 해제, 기존 슬롯이면 무해 */
            continue;
        }
        // UDP 특성상 패킷이 중복으로 왔을 경우를 막아줍니다. 이미 받은 인덱스는 무시!
        if (rs->frag_received[hdr->frag_idx]) {
            continue;
        }
        rs->frag_received[hdr->frag_idx] = 1;

        // 5. 메모리 조립 (페이로드 복사) 및 버퍼 취약점 방어 
        uint32_t offset = hdr->payload_offset;
        
        // 메모리 계산시 오버플로우 공격이나 버그로 인해 엉뚱한 메모리를 침범(OOB)하는 것을 원천 차단합니다.
        if (offset >= REASM_BUF_MAX || (uint32_t)plen > REASM_BUF_MAX - offset) {
            fprintf(stderr, "[reasm] 버퍼 오버플로/언더플로 차단: offset=%u plen=%d\n", offset, plen);
            rs->in_use = 0; // 위험하므로 즉시 해당 슬롯을 폐기합니다.
            continue;
        }
        
        uint32_t end = offset + (uint32_t)plen;
        memcpy(rs->buf + offset, payload, (size_t)plen); // 안전한 오프셋에 패킷 붙여넣기
        
        if (end > rs->total_size) rs->total_size = end;
        rs->frags_recv++;

        // 6. 데이터가 모두 모였는지 확인 (마지막 프래그먼트)
        if (rs->frags_recv == rs->frags_total) {
            // 조립 완료: 해당 타입에 맞는 커밋 함수를 호출하여 Qt 앱(SHM)으로 보냅니다.
            switch (rs->type) {
            case PKT_TYPE_IMAGE:
                commit_image(shm, rs);
                break;
            case PKT_TYPE_LIDAR:
                commit_lidar(shm, rs);
                break;
            }
            // 완료되었으므로 슬롯을 다시 빈 상태로 만듭니다.
            rs->in_use = 0;
        }
    }

    free(slots);
    free(pkt);
    fprintf(stderr, "[reasm] 종료\n");
    return NULL;
}
