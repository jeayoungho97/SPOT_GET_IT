/*
 * spi_trot_test.cpp — Jetson 단독 SPI trot 보행 테스트 (ROS2 불필요)
 * v2: 진단 출력 강화 — Phase 2 동안 target/feedback/fault 비교 출력
 *
 * 빌드 (Jetson):
 *   g++ -O2 -o spi_trot_test spi_trot_test.cpp -lm
 *
 * 실행:
 *   sudo ./spi_trot_test
 *   sudo ./spi_trot_test --cycles 5 --stride 50 --lift 10
 *
 * Ctrl+C → IDLE (torque OFF) 전송 후 종료
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/* ─── 프로토콜 상수 ─── */
static constexpr uint16_t COMMAND_MAGIC  = 0xA55A;
static constexpr uint16_t FEEDBACK_MAGIC = 0x5AA5;
static constexpr int      NUM_JOINTS     = 12;
static constexpr int      SPI_FRAME_SIZE = 261;
static constexpr int      MOSI_PAYLOAD   = 116;

static constexpr uint8_t MODE_IDLE     = 0;
static constexpr uint8_t MODE_POSITION = 1;
static constexpr uint8_t MODE_HOLD     = 3;
static constexpr uint8_t FLAG_TORQUE_EN = 0x01;

/* ─── 로봇 파라미터 (firmware/Inc/config.h) ─── */
static constexpr float L1_MM = 105.0f;
static constexpr float L2_MM = 130.0f;
static constexpr float BODY_HEIGHT_MM = 170.0f;
static constexpr float DEFAULT_FOOT_X = -10.0f;
static constexpr float DEFAULT_FOOT_Z = -BODY_HEIGHT_MM;

/* Trot phase offset: FL, FR, RL, RR */
static const float TROT_PHASE_OFFSET[4] = {0.5f, 0.0f, 0.0f, 0.5f};

/* ─── Fault 코드 이름 (디버그용) ─── */
static const char* fault_name(uint8_t f) {
    switch (f) {
        case 0: return "OK";
        case 1: return "IMU_FAIL";
        case 2: return "SERVO_TMO";
        case 3: return "CRC_ERR";
        case 4: return "STALE";
        case 5: return "TEMP_HIGH";
        case 6: return "VOLT_LOW";
        case 7: return "SAFETY";
        case 8: return "NAN_TGT";
        default: return "???";
    }
}

/* ─── 글로벌 ─── */
static volatile sig_atomic_t g_stop = 0;
static int spi_fd = -1;

static void signal_handler(int) { g_stop = 1; }

/* ═══════════════════════════════════
   시간 유틸
   ═══════════════════════════════════ */

static double now_sec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void sleep_until(double target)
{
    double remain = target - now_sec();
    if (remain > 0) {
        struct timespec ts;
        ts.tv_sec  = (time_t)remain;
        ts.tv_nsec = (long)((remain - ts.tv_sec) * 1e9);
        nanosleep(&ts, nullptr);
    }
}

static uint32_t timestamp_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint32_t)((ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL) & 0xFFFFFFFFULL);
}

/* ═══════════════════════════════════
   CRC-16 / CCITT-FALSE
   ═══════════════════════════════════ */

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* ═══════════════════════════════════
   SPI 초기화 / 전송
   ═══════════════════════════════════ */

static int spi_open(int bus, int dev, uint32_t speed_hz)
{
    char path[64];
    snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, dev);
    int fd = open(path, O_RDWR);
    if (fd < 0) { perror(path); return -1; }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);

    printf("SPI: %s @ %u Hz, mode %u\n", path, speed_hz, mode);
    return fd;
}

static void spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct spi_ioc_transfer tr{};
    tr.tx_buf = (__u64)(uintptr_t)tx;
    tr.rx_buf = (__u64)(uintptr_t)rx;
    tr.len    = (uint32_t)len;
    tr.speed_hz = 5000000;
    tr.bits_per_word = 8;
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) perror("SPI_IOC_MESSAGE");
}

/* ═══════════════════════════════════
   패킷 빌드 / 파싱
   ═══════════════════════════════════ */

struct FeedbackPacket {
    uint16_t seq_echo;
    uint8_t  status;
    uint8_t  fault;
    uint8_t  motion_state;
    float    pos[12];
    float    gyro[3];
    float    quat[4];
    float    bus_voltage;
    float    gait_phase;
    uint32_t gait_cycle;
    bool     valid;
};

static void write_le16(uint8_t *p, uint16_t v) { p[0]=v&0xFF; p[1]=(v>>8)&0xFF; }
static void write_le32(uint8_t *p, uint32_t v) {
    p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF;
}
static void write_f32(uint8_t *p, float f) { uint32_t v; memcpy(&v,&f,4); write_le32(p,v); }
static uint16_t read_le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1]<<8); }
static float read_f32(const uint8_t *p) {
    uint32_t v = p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
    float f; memcpy(&f,&v,4); return f;
}

static void build_mosi(uint8_t *buf, uint16_t seq, uint8_t mode, uint8_t flags,
                       const float target[12], const float delta[12],
                       float gait_phase, uint32_t gait_cycle)
{
    memset(buf, 0, SPI_FRAME_SIZE);
    int off = 0;
    write_le16(buf+off, COMMAND_MAGIC); off += 2;
    write_le16(buf+off, seq);           off += 2;
    write_le32(buf+off, timestamp_us());off += 4;
    buf[off++] = mode;
    buf[off++] = flags;
    for (int i = 0; i < 12; i++) { write_f32(buf+off, target[i]); off += 4; }
    for (int i = 0; i < 12; i++) { write_f32(buf+off, delta[i]);  off += 4; }
    write_f32(buf+off, gait_phase);     off += 4;
    write_le32(buf+off, gait_cycle);    off += 4;
    /* off == 114 */
    uint16_t crc = crc16_ccitt(buf, 114);
    write_le16(buf+114, crc);
}

static FeedbackPacket parse_miso(const uint8_t *buf)
{
    FeedbackPacket fb{};
    fb.valid = false;

    if (read_le16(buf) != FEEDBACK_MAGIC) return fb;
    uint16_t calc = crc16_ccitt(buf, 259);
    if (calc != read_le16(buf+259)) return fb;

    fb.valid = true;
    int off = 2;
    fb.seq_echo = read_le16(buf+off); off += 2;
    off += 4; /* timestamp */
    fb.status       = buf[off++];
    fb.fault        = buf[off++];
    fb.motion_state = buf[off++];
    fb.gait_phase = read_f32(buf+off); off += 4;
    fb.gait_cycle = read_le16(buf+off) | ((uint32_t)read_le16(buf+off+2)<<16); off += 4;
    off += 4; /* imu_yaw */
    for (int i = 0; i < 12; i++) { fb.pos[i]  = read_f32(buf+off); off += 4; }
    off += 48; /* velocity */
    off += 48; /* load */
    off += 48; /* temperature */
    for (int i = 0; i < 3; i++)  { fb.gyro[i] = read_f32(buf+off); off += 4; }
    off += 12; /* accel */
    for (int i = 0; i < 4; i++)  { fb.quat[i] = read_f32(buf+off); off += 4; }
    fb.bus_voltage = read_f32(buf+off);
    return fb;
}

/* ═══════════════════════════════════
   IK — firmware/Src/leg_ik.c 포팅
   ═══════════════════════════════════ */

static void ik_2link(float foot_x, float foot_z,
                     float &theta_t, float &theta_k)
{
    float r2 = foot_x * foot_x + foot_z * foot_z;
    float cos_k = (r2 - L1_MM*L1_MM - L2_MM*L2_MM) / (2.0f * L1_MM * L2_MM);
    if (cos_k >  1.0f) cos_k =  1.0f;
    if (cos_k < -1.0f) cos_k = -1.0f;
    float sin_k = sqrtf(1.0f - cos_k * cos_k);
    theta_k = atan2f(sin_k, cos_k);

    float A = L1_MM + L2_MM * cos_k;
    float B = L2_MM * sin_k;
    float det = A*A + B*B;
    float fz_neg = -foot_z;
    float sin_t = (A * foot_x - B * fz_neg) / det;
    float cos_t = (B * foot_x + A * fz_neg) / det;
    theta_t = atan2f(sin_t, cos_t);
}

/* ═══════════════════════════════════
   Gait — firmware/Src/gait.c 포팅
   ═══════════════════════════════════ */

static float smoothstep(float t) { return t * t * (3.0f - 2.0f * t); }

static void compute_trot_offset(float leg_phase, float stride_x, float lift_z,
                                float duty, float &fx, float &fz)
{
    if (leg_phase < duty) {
        float s = leg_phase / duty;
        fx = stride_x * 0.5f - s * stride_x;
        fz = 0.0f;
        return;
    }
    float s = (leg_phase - duty) / (1.0f - duty);
    s = smoothstep(s);
    float oms = 1.0f - s;
    float B0 = oms*oms*oms, B1 = 3*oms*oms*s, B2 = 3*oms*s*s, B3 = s*s*s;
    float z_ctrl = (4.0f / 3.0f) * lift_z;
    fx = B0 * (-stride_x*0.5f) + B3 * (stride_x*0.5f);
    fz = B1 * z_ctrl + B2 * z_ctrl;
}

static void compute_targets(float phase, float stride_x, float lift_z,
                            float duty, float target[12])
{
    for (int leg = 0; leg < 4; leg++) {
        float lp = phase + TROT_PHASE_OFFSET[leg];
        if (lp >= 1.0f) lp -= 1.0f;

        float fx_off, fz_off;
        compute_trot_offset(lp, stride_x, lift_z, duty, fx_off, fz_off);

        float foot_x = DEFAULT_FOOT_X + fx_off;
        float foot_z = DEFAULT_FOOT_Z + fz_off;

        float theta_t, theta_k;
        ik_2link(foot_x, foot_z, theta_t, theta_k);

        target[leg*3 + 0] = 0.0f;     /* shoulder */
        target[leg*3 + 1] = theta_t;  /* thigh */
        target[leg*3 + 2] = theta_k;  /* knee */
    }
}

/* ═══════════════════════════════════
   메인
   ═══════════════════════════════════ */

int main(int argc, char **argv)
{
    /* 기본값 */
    int   n_cycles  = 3;
    float stride_x  = 70.0f;
    float lift_z    = 13.0f;
    float period_s  = 1.5f;
    float duty      = 0.55f;
    int   spi_bus   = 0;
    int   spi_dev   = 0;

    /* 인자 파싱 */
    static struct option long_opts[] = {
        {"cycles",  required_argument, 0, 'c'},
        {"stride",  required_argument, 0, 's'},
        {"lift",    required_argument, 0, 'l'},
        {"period",  required_argument, 0, 'p'},
        {"duty",    required_argument, 0, 'd'},
        {"bus",     required_argument, 0, 'b'},
        {"dev",     required_argument, 0, 'D'},
        {"help",    no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "c:s:l:p:d:b:D:h", long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'c': n_cycles = atoi(optarg); break;
            case 's': stride_x = strtof(optarg, nullptr); break;
            case 'l': lift_z   = strtof(optarg, nullptr); break;
            case 'p': period_s = strtof(optarg, nullptr); break;
            case 'd': duty     = strtof(optarg, nullptr); break;
            case 'b': spi_bus  = atoi(optarg); break;
            case 'D': spi_dev  = atoi(optarg); break;
            case 'h':
                printf("Usage: sudo %s [--cycles N] [--stride MM] [--lift MM] "
                       "[--period S] [--duty F] [--bus N] [--dev N]\n", argv[0]);
                return 0;
        }
    }

    /* 시그널 핸들러 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* SPI 열기 */
    spi_fd = spi_open(spi_bus, spi_dev, 5000000);
    if (spi_fd < 0) return 1;

    /* standing pose 계산 */
    float stand_target[12];
    compute_targets(0.0f, 0.0f, 0.0f, duty, stand_target);

    printf("\n=== SPI Trot Walking Test (C++ v2 — diag) ===\n");
    printf("  Cycles: %d  Period: %.1fs  Duty: %.2f\n", n_cycles, period_s, duty);
    printf("  Stride: %.0fmm  Lift: %.0fmm\n", stride_x, lift_z);
    printf("  Standing (thigh, knee): (%.3f, %.3f) rad\n",
           stand_target[1], stand_target[2]);
    printf("  All 12 standing targets: [");
    for (int i = 0; i < 12; i++) printf("%.3f%s", stand_target[i], i<11?", ":"");
    printf("]\n\n");

    float delta_standup[12], delta_walk[12], delta_settle[12];
    for (int i = 0; i < 12; i++) {
        delta_standup[i] = 0.02f;
        delta_walk[i]    = 0.08f;
        delta_settle[i]  = 0.03f;
    }

    uint8_t tx[SPI_FRAME_SIZE], rx[SPI_FRAME_SIZE];
    float   target[12] = {};
    uint16_t seq = 0;
    const double TICK = 0.02;  /* 50Hz */

    auto send = [&](uint8_t mode, uint8_t flags, const float *tgt,
                    const float *dlt, float gph = 0, uint32_t gcyc = 0) -> FeedbackPacket {
        build_mosi(tx, seq, mode, flags, tgt, dlt, gph, gcyc);
        spi_transfer(spi_fd, tx, rx, SPI_FRAME_SIZE);
        seq = (seq + 1) & 0xFFFF;
        return parse_miso(rx);
    };

    auto send_idle = [&]() {
        float z[12] = {};
        return send(MODE_IDLE, 0, z, z);
    };

    /* ── SPI 진단 ── */
    {
        printf("[Diag] SPI check... ");
        float zt[12] = {}, zd[12] = {};
        build_mosi(tx, 0xFFFF, MODE_IDLE, 0, zt, zd, 0, 0);
        spi_transfer(spi_fd, tx, rx, SPI_FRAME_SIZE);
        printf("TX[0:4]=%02X %02X %02X %02X  RX[0:4]=%02X %02X %02X %02X\n",
               tx[0], tx[1], tx[2], tx[3],
               rx[0], rx[1], rx[2], rx[3]);
        FeedbackPacket diag = parse_miso(rx);
        if (diag.valid)
            printf("[Diag] MISO OK — seq=%u fault=%s(%u) Vbus=%.1fV\n",
                   diag.seq_echo, fault_name(diag.fault), diag.fault,
                   diag.bus_voltage);
        else
            printf("[Diag] MISO invalid (magic=0x%02X%02X) — STM32 연결 확인!\n",
                   rx[1], rx[0]);
    }
    printf("\n");

    /* ── Phase 0: 카운트다운 ── */
    printf(">>> 3초 후 시작 — 로봇을 잡을 준비! <<<\n");
    for (int i = 3; i > 0 && !g_stop; i--) {
        printf("  %d...\n", i);
        double t_end = now_sec() + 1.0;
        while (now_sec() < t_end && !g_stop) {
            send_idle();
            sleep_until(now_sec() + TICK);
        }
    }
    printf("\n");

    /* ── Phase 1: Stand-up (2초) ── */
    if (!g_stop) {
        printf("[Phase 1] Standing up... (2s)\n");
        double t_end = now_sec() + 2.0;
        double t_next = now_sec();
        while (now_sec() < t_end && !g_stop) {
            send(MODE_POSITION, FLAG_TORQUE_EN, stand_target, delta_standup);
            t_next += TICK;
            sleep_until(t_next);
        }
        FeedbackPacket fb = send(MODE_POSITION, FLAG_TORQUE_EN,
                                 stand_target, delta_standup);
        if (fb.valid) {
            printf("  pos[FL]: [%.3f, %.3f, %.3f]  Vbus=%.1fV  fault=%s(%u)  mst=%u\n",
                   fb.pos[0], fb.pos[1], fb.pos[2],
                   fb.bus_voltage, fault_name(fb.fault), fb.fault, fb.motion_state);
            printf("  pos[FR]: [%.3f, %.3f, %.3f]\n", fb.pos[3], fb.pos[4], fb.pos[5]);
            printf("  pos[RL]: [%.3f, %.3f, %.3f]\n", fb.pos[6], fb.pos[7], fb.pos[8]);
            printf("  pos[RR]: [%.3f, %.3f, %.3f]\n", fb.pos[9], fb.pos[10], fb.pos[11]);
        }
        printf("\n");
    }

    /* ── Phase 2: Trot walking (진단 출력 강화) ── */
    if (!g_stop) {
        double total_walk = n_cycles * period_s;
        printf("[Phase 2] Trot walking — %d cycles (%.1fs)\n", n_cycles, total_walk);
        printf("  (진단: 0.2초마다 FL target vs feedback, fault 변화 추적)\n\n");

        double t_walk_start = now_sec();
        double t_next = t_walk_start;
        int tick_count = 0;
        uint8_t last_fault = 255;   /* 불가능 값 → 첫 출력 강제 */
        int fault_change_count = 0;
        int safety_count = 0;       /* fault=7 (SAFETY) 횟수 */
        int idle_count = 0;         /* motion_state=0 (IDLE) 횟수 */
        int total_ticks = 0;

        while (!g_stop) {
            double elapsed = now_sec() - t_walk_start;
            if (elapsed >= total_walk) break;

            float phase = fmodf((float)elapsed, period_s) / period_s;
            int cycle = (int)(elapsed / period_s);

            compute_targets(phase, stride_x, lift_z, duty, target);

            /* FL 다리 진단용 foot position 계산 */
            float fl_lp = phase + TROT_PHASE_OFFSET[0];
            if (fl_lp >= 1.0f) fl_lp -= 1.0f;
            float fl_fx, fl_fz;
            compute_trot_offset(fl_lp, stride_x, lift_z, duty, fl_fx, fl_fz);

            FeedbackPacket fb = send(MODE_POSITION, FLAG_TORQUE_EN,
                                     target, delta_walk, phase, (uint32_t)cycle);

            if (fb.valid) {
                total_ticks++;

                /* fault 변화 감지 → 즉시 출력 */
                if (fb.fault != last_fault) {
                    printf("  *** FAULT: %s(%u) -> %s(%u)  [tick=%d ph=%.2f mst=%u] ***\n",
                           fault_name(last_fault), last_fault,
                           fault_name(fb.fault), fb.fault,
                           tick_count, phase, fb.motion_state);
                    last_fault = fb.fault;
                    fault_change_count++;
                }

                /* SAFETY / IDLE 카운트 */
                if (fb.fault == 7) safety_count++;
                if (fb.motion_state == 0) idle_count++;

                /* 10 tick (0.2s) 마다 상세 진단 */
                if (tick_count % 10 == 0) {
                    printf("  [c%d/%d ph=%.2f] "
                           "FL(lp=%.2f foot=%+.0f,%+.0f) "
                           "tgt=(%+.3f,%+.3f,%+.3f) "
                           "fb=(%+.3f,%+.3f,%+.3f) "
                           "f=%s mst=%u\n",
                           cycle+1, n_cycles, phase,
                           fl_lp,
                           DEFAULT_FOOT_X + fl_fx,
                           DEFAULT_FOOT_Z + fl_fz,
                           target[0], target[1], target[2],
                           fb.pos[0], fb.pos[1], fb.pos[2],
                           fault_name(fb.fault), fb.motion_state);
                }
            }

            tick_count++;
            t_next = t_walk_start + tick_count * TICK;
            sleep_until(t_next);
        }

        printf("\n  ─── Phase 2 요약 ───\n");
        printf("  총 tick: %d (valid: %d)\n", tick_count, total_ticks);
        printf("  fault 변화 횟수: %d\n", fault_change_count);
        printf("  SAFETY(7) fault tick: %d / %d (%.0f%%)\n",
               safety_count, total_ticks,
               total_ticks > 0 ? 100.0*safety_count/total_ticks : 0.0);
        printf("  IDLE motion_state tick: %d / %d (%.0f%%)\n",
               idle_count, total_ticks,
               total_ticks > 0 ? 100.0*idle_count/total_ticks : 0.0);
        if (safety_count > 0) {
            printf("\n  *** SAFETY_LIMIT 발생! ***\n");
            printf("  원인: IN_HAND_MODE=0 → MAX_PITCH=35 / MAX_ROLL=40\n");
            printf("  해결: firmware config.h에서 IN_HAND_MODE=1로 변경 후 빌드/플래시\n");
        }
        printf("\n");
    }

    /* ── Phase 3: Settle (2초) ── */
    if (!g_stop) {
        printf("[Phase 3] Settling to stand... (2s)\n");
        double t_end = now_sec() + 2.0;
        double t_next = now_sec();
        while (now_sec() < t_end && !g_stop) {
            send(MODE_POSITION, FLAG_TORQUE_EN, stand_target, delta_settle);
            t_next += TICK;
            sleep_until(t_next);
        }
        printf("\n");
    }

    /* ── Phase 4: HOLD → IDLE ── */
    if (!g_stop) {
        printf("[Phase 4] HOLD (1s) -> IDLE\n");
        double t_end = now_sec() + 1.0;
        double t_next = now_sec();
        while (now_sec() < t_end && !g_stop) {
            send(MODE_HOLD, FLAG_TORQUE_EN, stand_target, delta_settle);
            t_next += TICK;
            sleep_until(t_next);
        }
    }

    /* IDLE 전송 */
    printf("Sending IDLE (torque OFF)...\n");
    for (int i = 0; i < 25; i++) {
        send_idle();
        sleep_until(now_sec() + TICK);
    }

    close(spi_fd);
    printf("\n=== %s ===\n", g_stop ? "Ctrl+C: 긴급 정지 완료" : "Trot test complete!");
    return 0;
}
