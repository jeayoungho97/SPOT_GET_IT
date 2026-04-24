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
#include <QVBoxLayout>
#include <vector>
#include "shm_reader.h"
#include "statemachine.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 하단 버튼 클릭 시 실행될 화면 전환 슬롯
    void goToPageMap();
    void goToPageStream();
    void goToPageSettings();

    // ShmReader 슬롯
    void updateStreamView(const QImage &image, uint32_t frame_id, uint64_t timestamp_us);
    void updateOdom(const OdomData &odom);
    void updateMeta(const MetaData &meta);

    // StateMachine 연동 슬롯
    void onRobotAdded(uint16_t robot_id, QString name);
    void onRobotStateChanged(uint16_t robot_id);
    void onEventAdded(QString msg);
    void onControlRobotChanged(uint16_t robot_id);
    
    // UI 이벤트 슬롯
    void onBtnAddRobotClicked();
    void onRobotSelectorIndexChanged(int index);

private:
    QStackedWidget *stackedWidget;

    // 하단 네비게이션 버튼
    QPushButton *btnMap;
    QPushButton *btnStream;
    QPushButton *btnSettings;

    // [1번 페이지] 전체 지도
    QGraphicsView *mapView;
    QGraphicsScene *mapScene;

    // [2번 페이지] 스트리밍 영상 (공유 메모리 연동용)
    QComboBox *robotSelector;
    QPushButton *btnEmergencyStop;
    QPushButton *btnResumeMission;
    QPushButton *btnManualMode;
    QLabel *streamLabel;

    // [3번 페이지] 환경 설정 (로봇 추가용으로 변경)
    QLineEdit *robotNameEdit;
    QPushButton *btnAddRobot;
    
    // 상태 머신
    StateMachine *m_stateMachine;

    // 공유메모리 리더 스레드 리스트 (로봇마다 1개씩)
    std::vector<ShmReader*> m_shmReaders;
    
    // 상태 및 통계 레이블들
    QLabel *lblRobotCount; // 3/3 대신 쓸 라벨
    QLabel *lblConnectionStatus;
    QLabel *lblDropCount;
    QVBoxLayout *robotStatusLayout; // ROBOT STATUS 동적 추가용
    QVBoxLayout *eventsLayout;      // EVENTS 동적 추가용
};



#endif // MAINWINDOW_H