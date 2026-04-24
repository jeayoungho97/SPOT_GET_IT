#include "shm_reader.h"
#include <QDebug>
#include <iostream>

ShmReader::ShmReader(QObject *parent)
    : QThread(parent)
    , m_running(false)
    , m_shm_fd(-1)
    , m_shared_data(nullptr)
    , m_poll_timer(nullptr)
{
}

ShmReader::~ShmReader()
{
    stop();
    wait();

    if (m_shared_data && m_shared_data != MAP_FAILED) {
        munmap(m_shared_data, sizeof(SharedData));
    }
    if (m_shm_fd >= 0) {
        close(m_shm_fd);
    }
}

bool ShmReader::init(const QString& shm_name)
{
    // 공유 메모리 파일 오픈 (읽기 전용이 아닌 rwlock 등을 위해 RDWR 권한 오픈)
    QByteArray ba = shm_name.toUtf8();
    const char* c_shm_name = ba.constData();
    m_shm_fd = shm_open(c_shm_name, O_RDWR, 0666);
    if (m_shm_fd < 0) {
        qWarning() << "ShmReader: Failed to open shared memory: " << c_shm_name;
        return false;
    }

    // 파일 사이즈 확인 및 매핑
    m_shared_data = (SharedData*)mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, m_shm_fd, 0);
    if (m_shared_data == MAP_FAILED) {
        qWarning() << "ShmReader: mmap failed";
        close(m_shm_fd);
        m_shm_fd = -1;
        return false;
    }

    qDebug() << "ShmReader: Successfully mapped shared memory.";

    // 폴링 타이머 설정 (현재 QThread가 아닌 MainThread 쪽에서 타이머가 돌게 하기 위해 init에서 세팅하지 않고, 별도 처리하거나 run 내에서 처리 필요함)
    // 안전을 위해 moveToThread 처리된 후 타이머를 돌리거나 편리를 위해 메인 스레드에서 QTimer를 동작시킴
    // 여기서는 간단히 메인 스레드 소속인 채로 타이머를 돕니다.
    m_poll_timer = new QTimer(this);
    connect(m_poll_timer, &QTimer::timeout, this, &ShmReader::pollRwlockData);
    m_poll_timer->start(40); // 대략 25fps

    return true;
}

void ShmReader::stop()
{
    m_running = false;
    if (m_poll_timer) {
        m_poll_timer->stop();
    }
    // sem_wait 에 블로킹 되어있을 수 있으므로 강제로 세마포어 시그널을 줄지 여부 결정
    // 여기서는 BridgeDaemon이 시그널 주기를 기다리거나, 아니면 pthread_cancel 고려
}

void ShmReader::run()
{
    if (!m_shared_data || m_shared_data == MAP_FAILED) {
        emit newLogMessage("ShmReader: Shared data not initialized!");
        return;
    }

    m_running = true;
    emit newLogMessage("ShmReader: Image receive thread started.");

    while (m_running) {
        // 이미지가 수신될 때까지 대기
        if (sem_wait(&m_shared_data->img_sem) == -1) {
            if (errno == EINTR) continue; // 인터럽트인 경우 재시도
            qWarning() << "ShmReader: sem_wait failed";
            break;
        }

        if (!m_running) break;

        // 최신 슬롯 인덱스 확인 (atomic)
        int ready_idx = m_shared_data->img_ready_idx.load(std::memory_order_acquire);
        if (ready_idx >= 0 && ready_idx < IMG_SLOTS) {
            ImgSlot* slot = &m_shared_data->img_slots[ready_idx];
            uint32_t size = slot->size;
            
            if (size > 0 && size <= IMG_SLOT_SIZE) {
                // JPEG 디코딩 (Qt 내부 QImageReader에 의해 처리)
                QImage img = QImage::fromData(slot->data, size, "JPEG");
                
                if (!img.isNull()) {
                    emit newImageReceived(img, slot->frame_id, slot->timestamp_us);
                } else {
                    // qWarning() << "ShmReader: Failed to decode JPEG";
                }
            }
        }
    }
}

void ShmReader::pollRwlockData()
{
    if (!m_shared_data || m_shared_data == MAP_FAILED) return;

    // --- Odom 데이터 읽기 ---
    if (pthread_rwlock_rdlock(&m_shared_data->odom_lock) == 0) {
        OdomData odom;
        odom.x = m_shared_data->odom_x;
        odom.y = m_shared_data->odom_y;
        odom.theta = m_shared_data->odom_theta;
        odom.vx = m_shared_data->odom_vx;
        odom.vy = m_shared_data->odom_vy;
        odom.omega = m_shared_data->odom_omega;
        odom.seq = m_shared_data->odom_seq;
        pthread_rwlock_unlock(&m_shared_data->odom_lock);
        
        emit newOdomReceived(odom);
    }

    // --- Meta 데이터 읽기 ---
    if (pthread_rwlock_rdlock(&m_shared_data->meta_lock) == 0) {
        MetaData meta;
        meta.jetson_connected = (m_shared_data->meta.jetson_connected == 1);
        meta.img_drop_count = m_shared_data->meta.img_drop_count;
        meta.lidar_drop_count = m_shared_data->meta.lidar_drop_count;
        meta.avg_img_latency_us = m_shared_data->meta.avg_img_latency_us;
        pthread_rwlock_unlock(&m_shared_data->meta_lock);

        emit newMetaReceived(meta);
    }
}
