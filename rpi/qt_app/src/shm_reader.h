#ifndef SHM_READER_H
#define SHM_READER_H

#include <QThread>
#include <QImage>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QMetaType>

#ifdef __cplusplus
#include <atomic>
using std::atomic_int;
#endif

extern "C" {
#include "shm_def.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
}

/* ─── Qt 전달용 데이터 구조체 ───────────────────────────────── */
struct OdomData {
    float x, y, theta;
    float vx, vy, omega;
    uint32_t seq;
};

struct MetaData {
    bool     jetson_connected;
    uint32_t img_drop_count;
    uint32_t lidar_drop_count;
    float    avg_img_latency_us;
};

struct LidarPoint {
    float x, y, z, intensity;
};

struct LidarData {
    QVector<LidarPoint> points;
    uint32_t            frame_id;
    uint64_t            timestamp_us;
    /* 수신 시점의 odom (좌표 변환용) */
    float               robot_x;
    float               robot_y;
    float               robot_theta;
};

/* ─── ShmReader ──────────────────────────────────────────────── */
class ShmReader : public QThread
{
    Q_OBJECT
public:
    explicit ShmReader(QObject *parent = nullptr);
    ~ShmReader() override;

    bool init();
    void stop();

signals:
    void newImageReceived(const QImage &image,
                          uint32_t frame_id, uint64_t timestamp_us);
    void newLidarReceived(const LidarData &lidar);
    void newOdomReceived(const OdomData &odom);
    void newMetaReceived(const MetaData &meta);
    void newLogMessage(const QString &msg);

protected:
    void run() override;   /* 이미지 sem_wait 루프 */

private slots:
    void pollRwlockData(); /* 타이머: Odom / Meta / LiDAR polling */

private:
    std::atomic<bool> m_running;
    int          m_shm_fd;
    SharedData  *m_shared_data;
    QTimer      *m_poll_timer;

    /* 최신 odom (lidar와 타임스탬프 연동용) */
    float m_cur_x     = 0.f;
    float m_cur_y     = 0.f;
    float m_cur_theta = 0.f;
};

Q_DECLARE_METATYPE(OdomData)
Q_DECLARE_METATYPE(MetaData)
Q_DECLARE_METATYPE(LidarData)

#endif // SHM_READER_H
