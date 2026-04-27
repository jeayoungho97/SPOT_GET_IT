#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStackedWidget>
#include <QPushButton>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QFrame>
#include <QComboBox>
#include "shm_reader.h"
#include "pointcloudwidget.h"
#include "lidarmap2dwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void goToPageMap();
    void goToPageStream();
    void goToPageSettings();

    /* 스트리밍 페이지 내 뷰 모드 전환 */
    void showCameraView();
    void show3DLidarView();
    void show2DLidarView();

    /* ShmReader 슬롯 */
    void updateStreamView(const QImage &image,
                          uint32_t frame_id, uint64_t timestamp_us);
    void updateLidar(const LidarData &lidar);
    void updateOdom(const OdomData &odom);
    void updateMeta(const MetaData &meta);

private:
    QStackedWidget *stackedWidget;

    /* 하단 네비게이션 */
    QPushButton *btnMap;
    QPushButton *btnStream;
    QPushButton *btnSettings;

    /* [1번 페이지] 지도 */
    QGraphicsView  *mapView;
    QGraphicsScene *mapScene;

    /* [2번 페이지] 스트리밍 */
    QComboBox   *robotSelector;
    QPushButton *btnEmergencyStop;
    QPushButton *btnResumeMission;
    QPushButton *btnManualMode;

    /* 뷰 모드 토글 버튼 */
    QPushButton *btnViewCamera;
    QPushButton *btnView3D;
    QPushButton *btnView2D;

    /* 스트리밍 내부 QStackedWidget (Camera / 3D / 2D) */
    QStackedWidget    *viewStack;
    QLabel            *streamLabel;       /* 인덱스 0: 카메라 */
    PointCloudWidget  *pcWidget;          /* 인덱스 1: 3D LiDAR */
    LidarMap2DWidget  *map2DWidget;       /* 인덱스 2: 2D top-view */

    /* [3번 페이지] 환경 설정 */
    QLineEdit   *ipEdit;
    QLineEdit   *portEdit;
    QPushButton *btnSaveSettings;

    /* 상태 레이블 */
    QLabel *lblConnectionStatus;
    QLabel *lblDropCount;

    /* SHM 리더 */
    ShmReader *m_shmReader;
};

#endif // MAINWINDOW_H
