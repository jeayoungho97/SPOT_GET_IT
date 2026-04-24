#ifndef SHM_READER_H
#define SHM_READER_H

#include <QThread>
#include <QImage>
#include <QString>
#include <QTimer>

// Ensure C++ atomic compatibility with C stdatomic
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

// Odom 데이터 구조체 (Qt UI 전달용)
struct OdomData {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
    uint32_t seq;
};

// Meta 데이터 
struct MetaData {
    bool jetson_connected;
    uint32_t img_drop_count;
    uint32_t lidar_drop_count;
    float avg_img_latency_us;
};

class ShmReader : public QThread
{
    Q_OBJECT

public:
    explicit ShmReader(QObject *parent = nullptr);
    ~ShmReader() override;

    bool init(const QString& shm_name = "/robot_bridge_0");     // 메모리 매핑 초기화
    void stop();     // 스레드 종료

signals:
    void newImageReceived(const QImage &image, uint32_t frame_id, uint64_t timestamp_us);
    void newOdomReceived(const OdomData &odom);
    void newMetaReceived(const MetaData &meta);
    void newLogMessage(const QString &msg);

protected:
    void run() override; // 스레드 루프 (세마포어 대기 등)

private slots:
    void pollRwlockData(); // 타이머를 이용해 Odom, Meta 등의 rwlock 데이터를 주기적으로 읽음

private:
    bool m_running;
    int m_shm_fd;
    SharedData* m_shared_data;
    QTimer* m_poll_timer;
};

#endif // SHM_READER_H
