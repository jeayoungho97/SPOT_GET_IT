#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QWidget>

/* ─── 뷰 모드 버튼 스타일 헬퍼 ──────────────────────────────── */
static void setToggleActive(QPushButton *btn, bool active)
{
    btn->setStyleSheet(active
        ? "QPushButton { background-color: #4a7c59; color: white; border-radius: 6px;"
          " font-weight: bold; font-size: 14px; padding: 6px 16px; border: 1px solid #6abf7e; }"
        : "QPushButton { background-color: #2c2c2c; color: #aaaaaa; border-radius: 6px;"
          " font-size: 14px; padding: 6px 16px; border: 1px solid #444; }"
          " QPushButton:hover { background-color: #3c3c3c; color: white; }"
    );
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    this->resize(1600, 960);

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(20);

    /* ── QStackedWidget (페이지) ── */
    stackedWidget = new QStackedWidget(this);
    mainLayout->addWidget(stackedWidget);

    /* =====================================================
     * [0번 페이지] 지도
     * ===================================================== */
    QWidget *pageMap = new QWidget();
    QVBoxLayout *mapPageLayout = new QVBoxLayout(pageMap);
    mapPageLayout->setContentsMargins(0, 0, 0, 0);
    mapPageLayout->setSpacing(15);

    auto createTopStatBox = [](const QString &title, const QString &value,
                                const QString &tc, const QString &vc,
                                QLabel **out = nullptr) -> QFrame* {
        QFrame *f = new QFrame();
        f->setStyleSheet("QFrame{background:#1a1c23;border-radius:8px;border:1px solid #333;}");
        QVBoxLayout *l = new QVBoxLayout(f);
        l->setContentsMargins(15,10,15,10); l->setSpacing(5);
        QLabel *t = new QLabel(title);
        t->setStyleSheet(QString("color:%1;font-weight:bold;font-size:20px;border:none;").arg(tc));
        QLabel *v = new QLabel(value);
        v->setStyleSheet(QString("color:%1;font-weight:bold;font-size:20px;border:none;").arg(vc));
        v->setAlignment(Qt::AlignCenter);
        l->addWidget(t); l->addWidget(v);
        if (out) *out = v;
        return f;
    };

    QHBoxLayout *topInfoLayout = new QHBoxLayout();
    topInfoLayout->setSpacing(15);
    topInfoLayout->addWidget(createTopStatBox("MISSION IN PROGRESS","00:12:37","#88cc88","#ffffff"),2);
    topInfoLayout->addWidget(createTopStatBox(" ROBOTS","3/3","#888888","#ffffff"),1);
    topInfoLayout->addWidget(createTopStatBox(" AREA","67%","#888888","#ffffff"),1);
    topInfoLayout->addWidget(createTopStatBox(" PACKET DROPS","0","#888888","#ffffff",&lblDropCount),1);
    topInfoLayout->addWidget(createTopStatBox(" SYSTEM","WAIT...","#888888","#88cc88",&lblConnectionStatus),1);
    mapPageLayout->addLayout(topInfoLayout);

    QHBoxLayout *contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(15);

    mapScene = new QGraphicsScene(this);
    mapView  = new QGraphicsView(mapScene);
    mapView->setStyleSheet("background:#0b0c10;border-radius:8px;border:1px solid #333;");
    contentLayout->addWidget(mapView, 3);

    auto createSidebarPanel = [](const QString &title, QWidget *cw) -> QFrame* {
        QFrame *f = new QFrame();
        f->setStyleSheet("QFrame{background:#1a1c23;border-radius:8px;border:1px solid #333;}");
        QVBoxLayout *l = new QVBoxLayout(f);
        l->setContentsMargins(15,15,15,15);
        QLabel *lbl = new QLabel(title);
        lbl->setStyleSheet("color:#fff;font-weight:bold;font-size:30px;border:none;");
        l->addWidget(lbl);
        cw->setStyleSheet("border:none;background:transparent;color:#bbb;font-size:20px;");
        l->addWidget(cw);
        return f;
    };

    QVBoxLayout *sidebarLayout = new QVBoxLayout();
    sidebarLayout->setSpacing(15);

    QWidget *rsw = new QWidget(); QVBoxLayout *rsl = new QVBoxLayout(rsw);
    rsl->setContentsMargins(0,0,0,0);
    rsl->addWidget(new QLabel("micro-spot1       [ 99% ]"));
    rsl->addWidget(new QLabel("micro-spot2       [ 52% ]"));
    sidebarLayout->addWidget(createSidebarPanel("ROBOT STATUS", rsw));

    QWidget *evw = new QWidget(); QVBoxLayout *evl = new QVBoxLayout(evw);
    evl->setContentsMargins(0,0,0,0);
    evl->addWidget(new QLabel("10:11:58 micro-spot1 인원 식별"));
    evl->addWidget(new QLabel("10:10:58 구역 수색 시작"));
    evl->addWidget(new QLabel("10:10:42 micro-spot2 50% 도달"));
    sidebarLayout->addWidget(createSidebarPanel("EVENTS", evw));

    QWidget *mw = new QWidget(); QVBoxLayout *ml = new QVBoxLayout(mw);
    ml->setContentsMargins(0,0,0,0);
    ml->addWidget(new QLabel("단계: 수색 중"));
    ml->addWidget(new QLabel("달성률: 67%"));
    ml->addWidget(new QLabel("할당: 1 / 3 구역"));
    sidebarLayout->addWidget(createSidebarPanel("MISSION", mw));

    contentLayout->addLayout(sidebarLayout, 1);
    mapPageLayout->addLayout(contentLayout);
    stackedWidget->addWidget(pageMap); // 인덱스 0

    /* =====================================================
     * [1번 페이지] 스트리밍 (Camera / 3D / 2D 토글)
     * ===================================================== */
    QWidget *pageStream = new QWidget();
    QVBoxLayout *streamLayout = new QVBoxLayout(pageStream);
    streamLayout->setContentsMargins(0, 0, 0, 0);
    streamLayout->setSpacing(10);

    /* 상단 컨트롤 바 */
    QHBoxLayout *streamTopLayout = new QHBoxLayout();
    streamTopLayout->setSpacing(15);

    QLabel *lblRobotSelect = new QLabel("로봇 선택:");
    lblRobotSelect->setStyleSheet("color:#fff;font-size:16px;font-weight:bold;");
    robotSelector = new QComboBox();
    robotSelector->addItems({"micro-spot1","micro-spot2","micro-spot3"});
    robotSelector->setStyleSheet("QComboBox{background:#2c2c2c;color:#fff;font-size:16px;"
                                 "padding:8px 12px;border-radius:4px;border:1px solid #444;}");
    robotSelector->setMinimumHeight(45);

    btnEmergencyStop = new QPushButton("  긴급 정지  ");
    btnResumeMission = new QPushButton(" 임무 재가동  ");
    btnManualMode    = new QPushButton("  수동 모드  ");
    for (auto *b : {btnEmergencyStop, btnResumeMission, btnManualMode})
        b->setStyleSheet("QPushButton{background:#77777777;color:white;border-radius:6px;"
                         "font-weight:bold;font-size:16px;padding:10px 20px;}");

    streamTopLayout->addWidget(lblRobotSelect);
    streamTopLayout->addWidget(robotSelector);
    streamTopLayout->addStretch();
    streamTopLayout->addWidget(btnEmergencyStop);
    streamTopLayout->addWidget(btnResumeMission);
    streamTopLayout->addWidget(btnManualMode);
    streamLayout->addLayout(streamTopLayout);

    /* ── 뷰 모드 토글 버튼 ── */
    QHBoxLayout *viewToggleLayout = new QHBoxLayout();
    viewToggleLayout->setSpacing(8);
    viewToggleLayout->setContentsMargins(0, 0, 0, 0);

    btnViewCamera = new QPushButton("  카메라");
    btnView3D     = new QPushButton("  3D LiDAR");
    btnView2D     = new QPushButton("  2D 맵");

    for (auto *b : {btnViewCamera, btnView3D, btnView2D}) {
        b->setMinimumHeight(36);
        b->setMaximumWidth(160);
    }
    setToggleActive(btnViewCamera, true);
    setToggleActive(btnView3D, false);
    setToggleActive(btnView2D, false);

    viewToggleLayout->addWidget(btnViewCamera);
    viewToggleLayout->addWidget(btnView3D);
    viewToggleLayout->addWidget(btnView2D);
    viewToggleLayout->addStretch();
    streamLayout->addLayout(viewToggleLayout);

    /* ── 내부 QStackedWidget ── */
    viewStack = new QStackedWidget();

    /* 인덱스 0: 카메라 */
    streamLabel = new QLabel("공유 메모리 스트리밍 대기 중...");
    streamLabel->setObjectName("streamLabel");
    streamLabel->setAlignment(Qt::AlignCenter);
    streamLabel->setStyleSheet("background:#000;color:#888;font-size:16px;");
    viewStack->addWidget(streamLabel);

    /* 인덱스 1: 3D LiDAR */
    pcWidget = new PointCloudWidget();
    pcWidget->setMaxAccumFrames(15);
    viewStack->addWidget(pcWidget);

    /* 인덱스 2: 2D top-view */
    map2DWidget = new LidarMap2DWidget();
    /* 정밀지도 PNG 경로 — 필요 시 수정 */
    // map2DWidget->loadMapImage(":/maps/precision_map.png");
    viewStack->addWidget(map2DWidget);

    streamLayout->addWidget(viewStack, 1);
    stackedWidget->addWidget(pageStream); // 인덱스 1

    /* =====================================================
     * [2번 페이지] 환경 설정
     * ===================================================== */
    QWidget *pageSettings = new QWidget();
    QFormLayout *formLayout = new QFormLayout(pageSettings);
    ipEdit = new QLineEdit("127.0.0.1");
    portEdit = new QLineEdit("5000");
    btnSaveSettings = new QPushButton("설정 저장");
    btnSaveSettings->setObjectName("btnSaveSettings");
    btnSaveSettings->setMinimumHeight(45);
    formLayout->addRow("로봇 IP:",    ipEdit);
    formLayout->addRow("통신 포트:",  portEdit);
    formLayout->addRow("",            btnSaveSettings);
    stackedWidget->addWidget(pageSettings); // 인덱스 2

    /* ── 하단 네비게이션 ── */
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(20);
    btnMap      = new QPushButton("  지도  ");
    btnStream   = new QPushButton("  영상  ");
    btnSettings = new QPushButton(" 환경 설정");
    for (auto *b : {btnMap, btnStream, btnSettings})
        b->setMinimumHeight(60);
    bottomLayout->addWidget(btnMap);
    bottomLayout->addWidget(btnStream);
    bottomLayout->addWidget(btnSettings);
    mainLayout->addLayout(bottomLayout);

    /* ── 시그널-슬롯 ── */
    connect(btnMap,      &QPushButton::clicked, this, &MainWindow::goToPageMap);
    connect(btnStream,   &QPushButton::clicked, this, &MainWindow::goToPageStream);
    connect(btnSettings, &QPushButton::clicked, this, &MainWindow::goToPageSettings);

    connect(btnViewCamera, &QPushButton::clicked, this, &MainWindow::showCameraView);
    connect(btnView3D,     &QPushButton::clicked, this, &MainWindow::show3DLidarView);
    connect(btnView2D,     &QPushButton::clicked, this, &MainWindow::show2DLidarView);

    /* ── ShmReader 초기화 ── */
    m_shmReader = new ShmReader(this);
    if (m_shmReader->init()) {
        connect(m_shmReader, &ShmReader::newImageReceived,
                this, &MainWindow::updateStreamView);
        connect(m_shmReader, &ShmReader::newLidarReceived,
                this, &MainWindow::updateLidar);
        connect(m_shmReader, &ShmReader::newOdomReceived,
                this, &MainWindow::updateOdom);
        connect(m_shmReader, &ShmReader::newMetaReceived,
                this, &MainWindow::updateMeta);
        m_shmReader->start();
    }
}

MainWindow::~MainWindow() {}

/* ─── 페이지 전환 ────────────────────────────────────────────── */
void MainWindow::goToPageMap()    { stackedWidget->setCurrentIndex(0); }
void MainWindow::goToPageStream() { stackedWidget->setCurrentIndex(1); }
void MainWindow::goToPageSettings(){ stackedWidget->setCurrentIndex(2); }

/* ─── 뷰 모드 토글 ───────────────────────────────────────────── */
void MainWindow::showCameraView()
{
    viewStack->setCurrentIndex(0);
    setToggleActive(btnViewCamera, true);
    setToggleActive(btnView3D, false);
    setToggleActive(btnView2D, false);
}

void MainWindow::show3DLidarView()
{
    viewStack->setCurrentIndex(1);
    setToggleActive(btnViewCamera, false);
    setToggleActive(btnView3D, true);
    setToggleActive(btnView2D, false);
}

void MainWindow::show2DLidarView()
{
    viewStack->setCurrentIndex(2);
    setToggleActive(btnViewCamera, false);
    setToggleActive(btnView3D, false);
    setToggleActive(btnView2D, true);
}

/* ─── ShmReader 슬롯 ─────────────────────────────────────────── */
void MainWindow::updateStreamView(const QImage &image,
                                  uint32_t, uint64_t)
{
    if (stackedWidget->currentIndex() == 1 &&
        viewStack->currentIndex() == 0) {
        streamLabel->setPixmap(
            QPixmap::fromImage(image).scaled(
                streamLabel->size(),
                Qt::KeepAspectRatio,
                Qt::SmoothTransformation));
    }
}

void MainWindow::updateLidar(const LidarData &lidar)
{
    /* 3D 뷰어 — 항상 업데이트 (백그라운드 누적) */
    pcWidget->updateLidar(lidar);
    /* 2D 맵 */
    map2DWidget->updateLidar(lidar);
}

void MainWindow::updateOdom(const OdomData &odom)
{
    map2DWidget->updateOdom(odom);
}

void MainWindow::updateMeta(const MetaData &meta)
{
    if (meta.jetson_connected) {
        lblConnectionStatus->setText("CONNECTED");
        lblConnectionStatus->setStyleSheet(
            "color:#88cc88;font-weight:bold;font-size:20px;border:none;");
    } else {
        lblConnectionStatus->setText("DISCONNECTED");
        lblConnectionStatus->setStyleSheet(
            "color:#cc8888;font-weight:bold;font-size:20px;border:none;");
    }
    lblDropCount->setText(
        QString::number(meta.img_drop_count + meta.lidar_drop_count));
}
