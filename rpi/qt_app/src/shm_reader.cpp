#include "shm_reader.h"
#include <QDebug>
#include <cmath>

ShmReader::ShmReader(QObject *parent)
    : QThread(parent)
    , m_running(false)
    , m_shm_fd(-1)
    , m_shared_data(nullptr)
    , m_poll_timer(nullptr)
{}

ShmReader::~ShmReader()
{
    stop();
    wait();
    if (m_shared_data && m_shared_data != MAP_FAILED)
        munmap(m_shared_data, sizeof(SharedData));
    if (m_shm_fd >= 0)
        close(m_shm_fd);
}

bool ShmReader::init()
{
    const char *shm_name = "/robot_bridge_0";
    m_shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if (m_shm_fd < 0) {
        qWarning() << "ShmReader: shm_open failed:" << shm_name;
        return false;
    }

    m_shared_data = (SharedData *)mmap(nullptr, sizeof(SharedData),
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, m_shm_fd, 0);
    if (m_shared_data == MAP_FAILED) {
        qWarning() << "ShmReader: mmap failed";
        close(m_shm_fd);
        m_shm_fd = -1;
        return false;
    }

    qDebug() << "ShmReader: shared memory mapped.";

    /* 타이머: Odom / Meta / LiDAR polling (50ms = ~20Hz) */
    m_poll_timer = new QTimer(this);
    connect(m_poll_timer, &QTimer::timeout, this, &ShmReader::pollRwlockData);
    m_poll_timer->start(50);

    return true;
}

void ShmReader::stop()
{
    m_running = false;
    if (m_poll_timer) m_poll_timer->stop();
}

/* ─── 이미지 스레드: sem_wait 기반 ─────────────────────────── */
void ShmReader::run()
{
    if (!m_shared_data || m_shared_data == MAP_FAILED) {
        emit newLogMessage("ShmReader: not initialized");
        return;
    }

    m_running = true;
    emit newLogMessage("ShmReader: image thread started.");

    while (m_running) {
        if (sem_wait(&m_shared_data->img_sem) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (!m_running) break;

        int idx = m_shared_data->img_ready_idx.load(std::memory_order_acquire);
        if (idx < 0 || idx >= IMG_SLOTS) continue;

        ImgSlot *slot = &m_shared_data->img_slots[idx];
        uint32_t size = slot->size;
        if (size == 0 || size > IMG_SLOT_SIZE) continue;

        QImage img = QImage::fromData(slot->data, size, "JPEG");
        if (!img.isNull())
            emit newImageReceived(img, slot->frame_id, slot->timestamp_us);
    }
}

/* ─── 타이머 polling: Odom / Meta / LiDAR ──────────────────── */
void ShmReader::pollRwlockData()
{
    if (!m_shared_data || m_shared_data == MAP_FAILED) return;

    /* Odom */
    if (pthread_rwlock_rdlock(&m_shared_data->odom_lock) == 0) {
        OdomData odom;
        odom.x     = m_shared_data->odom_x;
        odom.y     = m_shared_data->odom_y;
        odom.theta = m_shared_data->odom_theta;
        odom.vx    = m_shared_data->odom_vx;
        odom.vy    = m_shared_data->odom_vy;
        odom.omega = m_shared_data->odom_omega;
        odom.seq   = m_shared_data->odom_seq;
        pthread_rwlock_unlock(&m_shared_data->odom_lock);

        /* 최신 odom 캐시 (lidar 좌표변환용) */
        m_cur_x     = odom.x;
        m_cur_y     = odom.y;
        m_cur_theta = odom.theta;

        emit newOdomReceived(odom);
    }

    /* Meta */
    if (pthread_rwlock_rdlock(&m_shared_data->meta_lock) == 0) {
        MetaData meta;
        meta.jetson_connected  = (m_shared_data->meta.jetson_connected == 1);
        meta.img_drop_count    = m_shared_data->meta.img_drop_count;
        meta.lidar_drop_count  = m_shared_data->meta.lidar_drop_count;
        meta.avg_img_latency_us= m_shared_data->meta.avg_img_latency_us;
        pthread_rwlock_unlock(&m_shared_data->meta_lock);
        emit newMetaReceived(meta);
    }

    /* LiDAR — sem_trywait (non-blocking): 새 프레임만 처리 */
    if (sem_trywait(&m_shared_data->lidar_sem) == 0) {
        int idx = m_shared_data->lidar_ready_idx.load(std::memory_order_acquire);
        if (idx >= 0 && idx < LIDAR_SLOTS) {
            const LidarSlot *slot = &m_shared_data->lidar_slots[idx];
            uint32_t cnt = slot->count;
            if (cnt > 0 && cnt <= LIDAR_MAX_PTS) {
                LidarData ld;
                ld.frame_id     = slot->frame_id;
                ld.timestamp_us = slot->timestamp_us;
                ld.robot_x      = m_cur_x;
                ld.robot_y      = m_cur_y;
                ld.robot_theta  = m_cur_theta;
                ld.points.resize((int)cnt);
                for (uint32_t i = 0; i < cnt; i++) {
                    ld.points[i] = { slot->x[i], slot->y[i],
                                     slot->z[i], slot->intensity[i] };
                }
                emit newLidarReceived(ld);
            }
        }
    }
}
