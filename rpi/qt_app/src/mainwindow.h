#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "dashboardwidgets.h"
#include "shmmonitor.h"

#include <QGridLayout>
#include <QComboBox>
#include <QDateTime>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QStackedWidget>
#include <QVector>

class QDateEdit;
class QLineEdit;
class QTableWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void updateSnapshots(const QVector<RobotSnapshot> &snapshots);
    void appendEvent(const UiEvent &event);
    void selectRobot(int robotId);
    void sendMove();
    void sendExplore();
    void sendStandby();
    void sendStop();
    void sendEstop();
    void showPage(int index);
    void showMap2D();
    void showMap3D();
    void showMapFullscreen();
    void leaveMapFullscreen();
    void showCameraFullscreen(int robotId);
    void leaveCameraFullscreen();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void buildUi();
    void applyStyle();
    QPushButton *makeNavButton(const QString &text, bool active = false);
    QPushButton *makeCommandButton(const QString &text, const QString &objectName = QString());
    QFrame *makeHeaderMetricCard(const QString &title, QLabel **valueLabel,
                                 const QString &accentColor,
                                 QProgressBar **progressBar = nullptr);
    QString formatMissionTime() const;
    void updateMissionSummary();
    void updatePacketLogPanel();
    void refreshRobotSelector();
    void refreshRobotList();
    void sendCommand(uint8_t commandType, float vx = 0.0f, float vy = 0.0f, float omega = 0.0f);
    bool isVictimDetectedEvent(const UiEvent &event) const;
    void showVictimAlert(const UiEvent &event);
    void positionVictimAlert();
    void restoreMapToDashboard();
    void appendLogTableRow(const QString &severityCode,
                           const QString &timestamp,
                           const QString &source,
                           const QString &category,
                           const QString &message,
                           bool prepend = false);
    void applyLogFilters();
    void setLogSeverityFilter(const QString &severityCode);
    void setLogPage(int page);
    bool logRowMatchesFilters(int row) const;
    void updateLogSummary();

    ShmMonitor m_monitor;
    QVector<RobotSnapshot> m_snapshots;
    int m_selectedRobot = 0;

    QLabel *m_system = nullptr;
    QLabel *m_clock = nullptr;
    QLabel *m_crumb = nullptr;
    QLabel *m_selected = nullptr;
    QLabel *m_missionInfo = nullptr;
    QLabel *m_robotInfo = nullptr;
    QLabel *m_metricProgress = nullptr;
    QLabel *m_metricMissionTime = nullptr;
    QLabel *m_metricConnected = nullptr;
    QLabel *m_metricEvents = nullptr;
    QProgressBar *m_metricProgressBar = nullptr;
    MapWidget *m_map = nullptr;
    QVBoxLayout *m_mapLayout = nullptr;
    QVBoxLayout *m_mapFullscreenContentLayout = nullptr;
    QWidget *m_mapFullscreenPage = nullptr;
    QListWidget *m_eventList = nullptr;
    QListWidget *m_logPageList = nullptr;
    QListWidget *m_packetStatsList = nullptr;
    QTableWidget *m_logTable = nullptr;
    QDateEdit *m_logStartDate = nullptr;
    QDateEdit *m_logEndDate = nullptr;
    QLineEdit *m_logSearch = nullptr;
    QLabel *m_logTotalValue = nullptr;
    QLabel *m_logActiveValue = nullptr;
    QLabel *m_logCriticalValue = nullptr;
    QLabel *m_logSeverityRows = nullptr;
    QLabel *m_logFooterRange = nullptr;
    QWidget *m_logSeverityChart = nullptr;
    QVector<QPushButton *> m_logFilterButtons;
    QVector<QPushButton *> m_logPageButtons;
    QPushButton *m_logFirstPageButton = nullptr;
    QPushButton *m_logPrevPageButton = nullptr;
    QPushButton *m_logNextPageButton = nullptr;
    QPushButton *m_logLastPageButton = nullptr;
    QString m_logSeverityFilter;
    int m_logCurrentPage = 0;
    int m_logRowsPerPage = 25;
    QStackedWidget *m_contentStack = nullptr;
    QGridLayout *m_videoGrid = nullptr;
    QVBoxLayout *m_robotStatusLayout = nullptr;
    QVector<VideoTile *> m_videoTiles;
    VideoTile *m_expandedVideoTile = nullptr;
    QWidget *m_cameraPage = nullptr;
    QLabel *m_cameraTitle = nullptr;
    QPushButton *m_victimAlertButton = nullptr;
    QLabel *m_victimAlertTitle = nullptr;
    QLabel *m_victimAlertRobot = nullptr;
    QLabel *m_victimAlertAction = nullptr;
    int m_expandedRobotId = -1;
    int m_victimAlertRobotId = -1;
    bool m_hasCameraReturnTarget = false;
    int m_cameraReturnIndex = 0;
    QWidget *m_cameraReturnWidget = nullptr;
    QString m_cameraReturnCrumb;
    QVector<StatusRow *> m_statusRows;
    QVector<RobotStatusCard *> m_robotStatusCards;
    QVector<QPushButton *> m_navButtons;
    QComboBox *m_robotSelector = nullptr;
    QVector<int> m_availableRobotIds;
    QDateTime m_missionStartedAt;
    bool m_missionTimerActive = false;
    bool m_updatingRobotSelector = false;
    QPushButton *m_btn2d = nullptr;
    QPushButton *m_btn3d = nullptr;
    QPushButton *m_fullscreenBtn2d = nullptr;
    QPushButton *m_fullscreenBtn3d = nullptr;
};

#endif
