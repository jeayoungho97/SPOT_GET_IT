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

    /*
     * delta_smooth: standup/settle용 (smoothstep 보간 사용 → slew limit 사실상 무력화)
     * delta_walk:   trot용 (안전상 slew limit 유지)
     */
    float delta_smooth[12], delta_walk[12];
    for (int i = 0; i < 12; i++) {
        delta_smooth[i] = 10.0f;   /* effectively no limit */
        delta_walk[i]   = 0.08f;
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

    /*
     * 진단 데이터 버퍼 (제어 루프 안에서 printf 절대 금지)
     * Phase 2 끝나고 settle 끝나고 한 번에 출력
     */
    struct DiagSnap {
        int      tick, cycle;
        float    phase, fl_lp, fl_fx, fl_fz;
        float    tgt[3], fb_pos[3];
        uint8_t  fault, mst;
    };
    struct FaultEvent {
        int      tick;
        float    phase;
        uint8_t  from, to;
    };
    static DiagSnap   snaps[64];   int snap_count = 0;
    static FaultEvent fevents[32]; int fevent_count = 0;
    int total_ticks = 0;
    int valid_ticks = 0;
    int safety_count = 0;
    int idle_count = 0;

    /* ── SPI 진단 (Phase 진입 전, 한 번만) ── */
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

    /* ── Phase 0: 카운트다운 + 초기 자세 read ──
     *   IDLE 모드로 (torque OFF 유지) feedback 받아 현재 다리 자세 캡처.
     *   smooth standup의 from-pose로 사용.
     */
    printf(">>> 3초 후 시작 — 로봇을 잡을 준비! <<<\n");
    float initial_pose[12] = {0};
    bool got_initial = false;
    for (int i = 3; i > 0 && !g_stop; i--) {
        printf("  %d...\n", i);
        double t_end = now_sec() + 1.0;
        while (now_sec() < t_end && !g_stop) {
            FeedbackPacket fb = send_idle();
            if (fb.valid) {
                memcpy(initial_pose, fb.pos, sizeof(initial_pose));
                got_initial = true;
            }
            sleep_until(now_sec() + TICK);
        }
    }
    printf("\n");
    if (got_initial) {
        printf("[Init pose] FL=(%+.2f,%+.2f,%+.2f) FR=(%+.2f,%+.2f,%+.2f) "
               "RL=(%+.2f,%+.2f,%+.2f) RR=(%+.2f,%+.2f,%+.2f)\n\n",
               initial_pose[0], initial_pose[1], initial_pose[2],
               initial_pose[3], initial_pose[4], initial_pose[5],
               initial_pose[6], initial_pose[7], initial_pose[8],
               initial_pose[9], initial_pose[10], initial_pose[11]);
    } else {
        memcpy(initial_pose, stand_target, sizeof(initial_pose));
        printf("[Init pose] feedback 못 받음 — standing pose로 가정\n\n");
    }

    /* ── Phase 1: Smooth standup (smoothstep 보간) ──
     *   STM의 robot_transition과 동일한 방식.
     *   adaptive duration: max delta * 1초/rad, [1, 3]초 클램프
     *   loop 안에서 printf 없음 — 50Hz 정확히 유지
     */
    if (!g_stop) {
        float max_delta = 0.0f;
        for (int i = 0; i < 12; i++) {
            float d = fabsf(stand_target[i] - initial_pose[i]);
            if (d > max_delta) max_delta = d;
        }
        double dur = (double)max_delta;       /* 1 rad/s */
        if (dur < 1.0) dur = 1.0;
        if (dur > 3.0) dur = 3.0;
        int n_steps = (int)(dur / TICK);

        printf("[Phase 1] Smooth standup (%.1fs, max delta %.2f rad)\n", dur, max_delta);

        for (int t = 0; t <= n_steps && !g_stop; t++) {
            float r = (float)t / (float)n_steps;
            float s = r * r * (3.0f - 2.0f * r);   /* smoothstep */
            float interp[12];
            for (int i = 0; i < 12; i++) {
                interp[i] = initial_pose[i]
                          + (stand_target[i] - initial_pose[i]) * s;
            }
            send(MODE_POSITION, FLAG_TORQUE_EN, interp, delta_smooth);
            sleep_until(now_sec() + TICK);
        }
    }

    /* ── Phase 2: Trot walking ──
     *   loop 안 printf 절대 없음. 진단 데이터는 메모리 버퍼에만 저장.
     *   Phase 1 → 2 사이도 printf 없으므로 STM32 stale 안 일어남.
     */
    if (!g_stop) {
        double total_walk = n_cycles * period_s;
        printf("[Phase 2] Trot walking — %d cycles (%.1fs)\n", n_cycles, total_walk);

        double t_walk_start = now_sec();
        int tick_count = 0;
        uint8_t last_fault = 255;

        while (!g_stop) {
            double elapsed = now_sec() - t_walk_start;
            if (elapsed >= total_walk) break;

            float phase = fmodf((float)elapsed, period_s) / period_s;
            int cycle = (int)(elapsed / period_s);

            compute_targets(phase, stride_x, lift_z, duty, target);

            FeedbackPacket fb = send(MODE_POSITION, FLAG_TORQUE_EN,
                                     target, delta_walk, phase, (uint32_t)cycle);

            if (fb.valid) {
                valid_ticks++;
                if (fb.fault == 7) safety_count++;
                if (fb.motion_state == 0) idle_count++;

                if (fb.fault != last_fault && fevent_count < 32) {
                    fevents[fevent_count++] = {tick_count, phase, last_fault, fb.fault};
                    last_fault = fb.fault;
                }

                if (tick_count % 10 == 0 && snap_count < 64) {
                    float fl_lp = phase + TROT_PHASE_OFFSET[0];
                    if (fl_lp >= 1.0f) fl_lp -= 1.0f;
                    float fl_fx, fl_fz;
                    compute_trot_offset(fl_lp, stride_x, lift_z, duty, fl_fx, fl_fz);

                    DiagSnap& s = snaps[snap_count++];
                    s.tick = tick_count;
                    s.cycle = cycle + 1;
                    s.phase = phase;
                    s.fl_lp = fl_lp;
                    s.fl_fx = DEFAULT_FOOT_X + fl_fx;
                    s.fl_fz = DEFAULT_FOOT_Z + fl_fz;
                    s.tgt[0] = target[0]; s.tgt[1] = target[1]; s.tgt[2] = target[2];
                    s.fb_pos[0] = fb.pos[0]; s.fb_pos[1] = fb.pos[1]; s.fb_pos[2] = fb.pos[2];
                    s.fault = fb.fault;
                    s.mst   = fb.motion_state;
                }
            }

            tick_count++;
            sleep_until(t_walk_start + tick_count * TICK);
        }
        total_ticks = tick_count;
    }

    /* save last trot target → settle interp의 from-pose */
    float last_target[12];
    memcpy(last_target, target, sizeof(last_target));

    /* ── Phase 3: Smooth settle back to standing ──
     *   trot 마지막 자세에서 standing으로 부드럽게 보간.
     *   loop 안 printf 없음.
     */
    if (!g_stop) {
        float max_delta = 0.0f;
        for (int i = 0; i < 12; i++) {
            float d = fabsf(stand_target[i] - last_target[i]);
            if (d > max_delta) max_delta = d;
        }
        double dur = (double)max_delta;
        if (dur < 1.0) dur = 1.0;
        if (dur > 3.0) dur = 3.0;
        int n_steps = (int)(dur / TICK);

        printf("[Phase 3] Smooth settle to standing (%.1fs)\n", dur);

        for (int t = 0; t <= n_steps && !g_stop; t++) {
            float r = (float)t / (float)n_steps;
            float s = r * r * (3.0f - 2.0f * r);
            float interp[12];
            for (int i = 0; i < 12; i++) {
                interp[i] = last_target[i]
                          + (stand_target[i] - last_target[i]) * s;
            }
            send(MODE_POSITION, FLAG_TORQUE_EN, interp, delta_smooth);
            sleep_until(now_sec() + TICK);
        }
    }

    /* ── Phase 4: Smooth return to initial pose ──
     *   Standing → 처음 시작했던 자세 (Phase 0에서 read한 initial_pose)
     *   "올라간 만큼 다시 천천히 내려간다" — STM 자체 trot 마지막 흐름과 동일.
     */
    if (!g_stop && got_initial) {
        float max_delta = 0.0f;
        for (int i = 0; i < 12; i++) {
            float d = fabsf(initial_pose[i] - stand_target[i]);
            if (d > max_delta) max_delta = d;
        }
        double dur = (double)max_delta;
        if (dur < 1.0) dur = 1.0;
        if (dur > 3.0) dur = 3.0;
        int n_steps = (int)(dur / TICK);

        printf("[Phase 4] Smooth return to initial pose (%.1fs)\n", dur);

        for (int t = 0; t <= n_steps && !g_stop; t++) {
            float r = (float)t / (float)n_steps;
            float s = r * r * (3.0f - 2.0f * r);
            float interp[12];
            for (int i = 0; i < 12; i++) {
                interp[i] = stand_target[i]
                          + (initial_pose[i] - stand_target[i]) * s;
            }
            send(MODE_POSITION, FLAG_TORQUE_EN, interp, delta_smooth);
            sleep_until(now_sec() + TICK);
        }
    }

    /* ── 진단 출력 (제어 루프 모두 끝난 후) ── */
    printf("\n══ Phase 2 진단 결과 ══\n");
    printf("  총 tick: %d (valid: %d, %.0f%% loss)\n",
           total_ticks, valid_ticks,
           total_ticks > 0 ? 100.0*(total_ticks-valid_ticks)/total_ticks : 0.0);
    printf("  SAFETY(7) tick: %d / %d (%.0f%%)\n",
           safety_count, valid_ticks,
           valid_ticks > 0 ? 100.0*safety_count/valid_ticks : 0.0);
    printf("  IDLE mst tick: %d / %d (%.0f%%)\n",
           idle_count, valid_ticks,
           valid_ticks > 0 ? 100.0*idle_count/valid_ticks : 0.0);
    if (fevent_count > 0) {
        printf("  fault transitions:\n");
        for (int i = 0; i < fevent_count; i++) {
            printf("    [tick=%d ph=%.2f] %s(%u) -> %s(%u)\n",
                   fevents[i].tick, fevents[i].phase,
                   fault_name(fevents[i].from), fevents[i].from,
                   fault_name(fevents[i].to), fevents[i].to);
        }
    }
    if (snap_count > 0) {
        printf("  snapshots (every 10 tick):\n");
        for (int i = 0; i < snap_count; i++) {
            DiagSnap& s = snaps[i];
            printf("    [c%d ph=%.2f] FL(lp=%.2f foot=%+.0f,%+.0f) "
                   "tgt=(%+.3f,%+.3f,%+.3f) fb=(%+.3f,%+.3f,%+.3f) f=%s mst=%u\n",
                   s.cycle, s.phase, s.fl_lp, s.fl_fx, s.fl_fz,
                   s.tgt[0], s.tgt[1], s.tgt[2],
                   s.fb_pos[0], s.fb_pos[1], s.fb_pos[2],
                   fault_name(s.fault), s.mst);
        }
    }
    printf("\n");

    /* ── Phase 5: HOLD at initial pose until Ctrl+C ──
     *   처음 자세 유지 (이미 낮게 내려와 있어서 토크 풀어도 위험 없음).
     *   자동으로 토크 OFF 안 함 — 사용자가 결정.
     *   진단 출력 동안 STM32가 잠시 stale → HOLD가 돼도, 어차피 같은 자세 유지라 무관.
     */
    if (!g_stop) {
        printf("[Phase 5] Holding initial pose. Press Ctrl+C to release torque.\n");
        while (!g_stop) {
            send(MODE_HOLD, FLAG_TORQUE_EN, initial_pose, delta_smooth);
            sleep_until(now_sec() + TICK);
        }
    }

    /* ── Ctrl+C 받음 → IDLE (안전한 토크 OFF) ── */
    printf("\n토크 OFF (IDLE) 전송 중...\n");
    for (int i = 0; i < 25; i++) {
        send_idle();
        sleep_until(now_sec() + TICK);
    }

    close(spi_fd);
    printf("\n=== %s ===\n", g_stop ? "Ctrl+C: 종료" : "Trot test complete!");
    return 0;
}
