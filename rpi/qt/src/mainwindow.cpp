#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QWidget>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 창의 기본 해상도 설정
    this->resize(1600, 960); 

    // 상태 머신 초기화
    m_stateMachine = new StateMachine(this);
    connect(m_stateMachine, &StateMachine::robotAdded, this, &MainWindow::onRobotAdded);
    connect(m_stateMachine, &StateMachine::robotStateChanged, this, &MainWindow::onRobotStateChanged);
    connect(m_stateMachine, &StateMachine::controlRobotChanged, this, &MainWindow::onControlRobotChanged);
    connect(m_stateMachine, &StateMachine::eventAdded, this, &MainWindow::onEventAdded);

    // 메인 중앙 위젯 및 전체 세로 레이아웃 설정
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->setContentsMargins(20, 20, 20, 20); // 레이아웃 주변 여백 20px
    mainLayout->setSpacing(20);                     // 위젯 간 간격 20px

    // ==========================================
    // 상단 영역: QStackedWidget (페이지들이 겹쳐지는 곳)
    // ==========================================
    stackedWidget = new QStackedWidget(this);
    mainLayout->addWidget(stackedWidget);

    // --- [1번 페이지] 전체 지도 (대시보드 형태) ---
    QWidget *pageMap = new QWidget();
    QVBoxLayout *mapPageLayout = new QVBoxLayout(pageMap);
    mapPageLayout->setContentsMargins(0, 0, 0, 0);
    mapPageLayout->setSpacing(15);

    // 1. 상단 정보 표시 바 (Top Bar)
    QHBoxLayout *topInfoLayout = new QHBoxLayout();
    topInfoLayout->setSpacing(15);
    
    // QFrame을 생성해서 스타일을 먹이는 헬퍼 람다
    auto createTopStatBox = [](const QString& title, const QString& value, const QString& titleColor, const QString& valueColor, QLabel** outLabel = nullptr) -> QFrame* {
        QFrame *frame = new QFrame();
        frame->setStyleSheet("QFrame { background-color: #1a1c23; border-radius: 8px; border: 1px solid #333333; }");
        QVBoxLayout *l = new QVBoxLayout(frame);
        l->setContentsMargins(15, 10, 15, 10);
        l->setSpacing(5);
        QLabel *lblTitle = new QLabel(title);
        lblTitle->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 20px; border: none;").arg(titleColor));
        QLabel *lblValue = new QLabel(value);
        lblValue->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 20px; border: none;").arg(valueColor));
        lblValue->setAlignment(Qt::AlignCenter);
        l->addWidget(lblTitle);
        l->addWidget(lblValue);
        
        if (outLabel) {
            *outLabel = lblValue;
        }
        
        return frame;
    };

    topInfoLayout->addWidget(createTopStatBox("MISSION IN PROGRESS", "00:00:00", "#88cc88", "#ffffff"), 2);
    topInfoLayout->addWidget(createTopStatBox(" ROBOTS", "0", "#888888", "#ffffff", &lblRobotCount), 1);
    topInfoLayout->addWidget(createTopStatBox(" AREA", "0%", "#888888", "#ffffff"), 1);
    topInfoLayout->addWidget(createTopStatBox(" PACKET DROPS", "0", "#888888", "#ffffff", &lblDropCount), 1);
    topInfoLayout->addWidget(createTopStatBox(" SYSTEM", "WAIT...", "#888888", "#88cc88", &lblConnectionStatus), 1);

    mapPageLayout->addLayout(topInfoLayout);

    // 2. 메인 컨텐츠 영역 (좌측 맵, 우측 사이드바)
    QHBoxLayout *contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(15);

    // 2-1. 좌측 맵
    mapScene = new QGraphicsScene(this);
    mapView = new QGraphicsView(mapScene);
    mapView->setStyleSheet("background-color: #0b0c10; border-radius: 8px; border: 1px solid #333333;");
    contentLayout->addWidget(mapView, 3); // 가로 비율 3

    // 2-2. 우측 사이드바
    QVBoxLayout *sidebarLayout = new QVBoxLayout();
    sidebarLayout->setSpacing(15);

    // 헬퍼 람다: 사이드바 패널 생성
    auto createSidebarPanel = [](const QString& title, QWidget* contentWidget) -> QFrame* {
        QFrame *frame = new QFrame();
        frame->setStyleSheet("QFrame { background-color: #1a1c23; border-radius: 8px; border: 1px solid #333333; }");
        QVBoxLayout *l = new QVBoxLayout(frame);
        l->setContentsMargins(15, 15, 15, 15);
        QLabel *lblTitle = new QLabel(title);
        lblTitle->setStyleSheet("color: #ffffff; font-weight: bold; font-size: 30px; border: none; text-transform: uppercase;");
        l->addWidget(lblTitle);
        contentWidget->setStyleSheet("border: none; background: transparent; color: #bbbbbb; font-size: 20px;");
        l->addWidget(contentWidget);
        return frame;
    };

    // 패널 1: ROBOT STATUS (동적)
    QWidget *robotStatusWidget = new QWidget();
    robotStatusLayout = new QVBoxLayout(robotStatusWidget);
    robotStatusLayout->setContentsMargins(0, 0, 0, 0);
    robotStatusLayout->setAlignment(Qt::AlignTop);
    sidebarLayout->addWidget(createSidebarPanel("ROBOT STATUS", robotStatusWidget));

    // 패널 2: EVENTS (동적)
    QWidget *eventsWidget = new QWidget();
    eventsLayout = new QVBoxLayout(eventsWidget);
    eventsLayout->setContentsMargins(0, 0, 0, 0);
    eventsLayout->setAlignment(Qt::AlignTop);
    sidebarLayout->addWidget(createSidebarPanel("EVENTS", eventsWidget));

    // 패널 3: MISSION
    QWidget *missionWidget = new QWidget();
    QVBoxLayout *mlayout = new QVBoxLayout(missionWidget);
    mlayout->setContentsMargins(0, 0, 0, 0);
    mlayout->addWidget(new QLabel("단계: 대기 중"));
    mlayout->addWidget(new QLabel("달성률: 0%"));
    sidebarLayout->addWidget(createSidebarPanel("MISSION", missionWidget));

    contentLayout->addLayout(sidebarLayout, 1); // 가로 비율 1
    mapPageLayout->addLayout(contentLayout);

    stackedWidget->addWidget(pageMap); // 인덱스 0

    // --- [2번 페이지] 개별 스트리밍 영상 ---
    QWidget *pageStream = new QWidget();
    QVBoxLayout *streamLayout = new QVBoxLayout(pageStream);

    // 스트리밍 상단 컨트롤 바
    QHBoxLayout *streamTopLayout = new QHBoxLayout();
    streamTopLayout->setSpacing(15);
    
    QLabel *lblRobotSelect = new QLabel("로봇 선택:");
    lblRobotSelect->setStyleSheet("color: #FFFFFF; font-size: 16px; font-weight: bold;");
    
    robotSelector = new QComboBox();
    robotSelector->setStyleSheet("QComboBox { background-color: #2C2C2C; color: #FFFFFF; font-size: 16px; padding: 8px 12px; border-radius: 4px; border: 1px solid #444; }");
    robotSelector->setMinimumHeight(45);
    connect(robotSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onRobotSelectorIndexChanged);
    
    btnEmergencyStop = new QPushButton("  긴급 정지  ");
    btnEmergencyStop->setStyleSheet("QPushButton { background-color: #77777777; color: white; border-radius: 6px; font-weight: bold; font-size: 16px; padding: 10px 20px;} QPushButton:hover { background-color: #77777777; }");
    
    btnResumeMission = new QPushButton(" 임무 재가동  ");
    btnResumeMission->setStyleSheet("QPushButton { background-color: #77777777; color: white; border-radius: 6px; font-weight: bold; font-size: 16px; padding: 10px 20px;} QPushButton:hover { background-color: #77777777; }");
    
    btnManualMode = new QPushButton("  수동 모드  ");
    btnManualMode->setStyleSheet("QPushButton { background-color: #77777777; color: white; border-radius: 6px; font-weight: bold; font-size: 16px; padding: 10px 20px;} QPushButton:hover { background-color: #77777777; }");

    streamTopLayout->addWidget(lblRobotSelect);
    streamTopLayout->addWidget(robotSelector);
    streamTopLayout->addStretch();
    streamTopLayout->addWidget(btnEmergencyStop);
    streamTopLayout->addWidget(btnResumeMission);
    streamTopLayout->addWidget(btnManualMode);

    streamLayout->addLayout(streamTopLayout);

    streamLabel = new QLabel("로봇을 추가하고 선택해주세요.");
    streamLabel->setObjectName("streamLabel");
    streamLabel->setAlignment(Qt::AlignCenter);
    streamLayout->addWidget(streamLabel);
    stackedWidget->addWidget(pageStream); // 인덱스 1

    // --- [3번 페이지] 환경 설정 (로봇 추가 폼) ---
    QWidget *pageSettings = new QWidget();
    QFormLayout *formLayout = new QFormLayout(pageSettings);
    robotNameEdit = new QLineEdit("micro-spot1");
    btnAddRobot = new QPushButton("로봇 추가");
    btnAddRobot->setObjectName("btnAddRobot");
    btnAddRobot->setMinimumHeight(45);
    connect(btnAddRobot, &QPushButton::clicked, this, &MainWindow::onBtnAddRobotClicked);
    
    formLayout->addRow("로봇 이름(또는 ID):", robotNameEdit);
    formLayout->addRow("", btnAddRobot);
    stackedWidget->addWidget(pageSettings); // 인덱스 2

    // ==========================================
    // 하단 영역: 네비게이션 버튼
    // ==========================================
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(20);

    btnMap = new QPushButton("  지도  ");
    btnMap->setIconSize(QSize(28, 28));

    btnStream = new QPushButton("  영상  ");
    btnStream->setIconSize(QSize(28, 28));

    btnSettings = new QPushButton(" 환경 설정");
    btnSettings->setIconSize(QSize(28, 28));

    btnMap->setMinimumHeight(60);
    btnStream->setMinimumHeight(60);
    btnSettings->setMinimumHeight(60);

    bottomLayout->addWidget(btnMap);
    bottomLayout->addWidget(btnStream);
    bottomLayout->addWidget(btnSettings);
    
    mainLayout->addLayout(bottomLayout);

    // ==========================================
    // 시그널 - 슬롯 연결 (화면 전환)
    // ==========================================
    connect(btnMap, &QPushButton::clicked, this, &MainWindow::goToPageMap);
    connect(btnStream, &QPushButton::clicked, this, &MainWindow::goToPageStream);
    connect(btnSettings, &QPushButton::clicked, this, &MainWindow::goToPageSettings);
}

MainWindow::~MainWindow()
{
    for (auto reader : m_shmReaders) {
        if (reader) {
            reader->stop();
            reader->wait();
            delete reader;
        }
    }
}

void MainWindow::goToPageMap() { stackedWidget->setCurrentIndex(0); }
void MainWindow::goToPageStream() { stackedWidget->setCurrentIndex(1); }
void MainWindow::goToPageSettings() { stackedWidget->setCurrentIndex(2); }

// ==========================================
// 신규 슬롯: 로봇 관리
// ==========================================
void MainWindow::onBtnAddRobotClicked()
{
    QString name = robotNameEdit->text();
    if (name.isEmpty()) return;
    
    RobotState newState;
    newState.robot_name = name.toStdString();
    newState.connected = true; 
    newState.pos_x = 0;
    newState.pos_y = 0;
    newState.mission_progress = 0;
    newState.battery = 100;
    newState.robot_shm_data = nullptr;

    m_stateMachine->add_robot(newState, nullptr);
    
    static int nextIdx = 2;
    robotNameEdit->setText(QString("micro-spot%1").arg(nextIdx++));
    QMessageBox::information(this, "성공", name + " 로봇이 추가되었습니다.");
}

void MainWindow::onRobotAdded(uint16_t robot_id, QString name)
{
    // 1. ShmReader 동적 생성
    ShmReader *newReader = new ShmReader(this);
    QString shmName = QString("/robot_bridge_%1").arg(robot_id);
    
    if (newReader->init(shmName)) {
        // 람다를 사용해 어떤 로봇으로부터 온 데이터인지 캡처 가능
        connect(newReader, &ShmReader::newImageReceived, this, [this, robot_id](const QImage &img, uint32_t f, uint64_t t) {
            if (m_stateMachine->get_current_control_robot() == robot_id) {
                this->updateStreamView(img, f, t);
            }
        });
        connect(newReader, &ShmReader::newOdomReceived, this, &MainWindow::updateOdom);
        connect(newReader, &ShmReader::newMetaReceived, this, &MainWindow::updateMeta);
        newReader->start();
    }
    m_shmReaders.push_back(newReader);

    // 2. 콤보박스에 로봇 추가
    robotSelector->addItem(name, QVariant(robot_id));
    
    // 3. 상태바 UI 갱신
    lblRobotCount->setText(QString::number(m_stateMachine->get_robot_num()));
    
    // 4. 사이드바 ROBOT STATUS 항목 동적 추가
    QLabel *newStatusLabel = new QLabel(QString("%1       [ 100% ]").arg(name));
    newStatusLabel->setProperty("robot_id", robot_id);
    robotStatusLayout->addWidget(newStatusLabel);
}

void MainWindow::onRobotStateChanged(uint16_t robot_id)
{
    // 추후 UI 갱신 구현
    Q_UNUSED(robot_id);
}

void MainWindow::onEventAdded(QString msg)
{
    QLabel *eventLabel = new QLabel(msg);
    eventsLayout->addWidget(eventLabel);
}

void MainWindow::onControlRobotChanged(uint16_t robot_id)
{
    qDebug() << "Now controlling robot:" << robot_id;
    // 이전 영상 잔상을 지우기
    streamLabel->clear();
    streamLabel->setText("수신 대기 중...");
}

void MainWindow::onRobotSelectorIndexChanged(int index)
{
    if (index < 0) return;
    uint16_t robot_id = robotSelector->itemData(index).toUInt();
    m_stateMachine->set_current_control_robot(robot_id);
}

// ==========================================
// 공유 메모리 데이터 수신 슬롯들
// ==========================================
void MainWindow::updateStreamView(const QImage &image, uint32_t frame_id, uint64_t timestamp_us)
{
    Q_UNUSED(frame_id);
    Q_UNUSED(timestamp_us);
    
    if (stackedWidget->currentIndex() == 1) { // 스트리밍 탭
        streamLabel->setPixmap(QPixmap::fromImage(image).scaled(streamLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::updateOdom(const OdomData &odom)
{
    Q_UNUSED(odom);
}

void MainWindow::updateMeta(const MetaData &meta)
{
    if (meta.jetson_connected) {
        lblConnectionStatus->setText("CONNECTED");
        lblConnectionStatus->setStyleSheet("color: #88cc88; font-weight: bold; font-size: 20px; border: none;");
    } else {
        lblConnectionStatus->setText("DISCONNECTED");
        lblConnectionStatus->setStyleSheet("color: #cc8888; font-weight: bold; font-size: 20px; border: none;");
    }
    lblDropCount->setText(QString::number(meta.img_drop_count + meta.lidar_drop_count));
}