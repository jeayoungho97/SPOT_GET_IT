#ifndef DASHBOARDWIDGETS_H
#define DASHBOARDWIDGETS_H

#include "mapdata.h"
#include "shmmonitor.h"

#include <QFrame>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QElapsedTimer>
#include <QWidget>

class LidarMap2DView;
class PointCloud3DView;
class QPaintEvent;
class QMouseEvent;
class QResizeEvent;
class QProgressBar;
class QStackedWidget;

class VideoTile : public QFrame
{
    Q_OBJECT

public:
    explicit VideoTile(int robotId, QWidget *parent = nullptr);
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
    void setRobotId(int robotId);
    void setSnapshot(const RobotSnapshot &snapshot);

signals:
    void clicked(int robotId);

protected:
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    int m_robotId = 0;
    QLabel *m_title = nullptr;
    QLabel *m_image = nullptr;
    QLabel *m_badge = nullptr;
};

class StatusRow : public QFrame
{
    Q_OBJECT

public:
    explicit StatusRow(int robotId, QWidget *parent = nullptr);
    void setSnapshot(const RobotSnapshot &snapshot);

private:
    int m_robotId = 0;
    QLabel *m_name = nullptr;
    QLabel *m_batteryText = nullptr;
    QProgressBar *m_batteryBar = nullptr;
    QLabel *m_mission = nullptr;
    QWidget *m_signal = nullptr;
};

class RobotStatusCard : public QFrame
{
    Q_OBJECT

public:
    explicit RobotStatusCard(int robotId, QWidget *parent = nullptr);
    void setSnapshot(const RobotSnapshot &snapshot);

private:
    QLabel *makeValueLabel(const QString &objectName = QStringLiteral("robotCardValue"));
    QFrame *makeInfoCell(const QString &title, QLabel *value, const QString &detail = QString());
    QLabel *makeSensorChip(const QString &text);
    QLabel *makeStatusChip(const QString &text, const QString &objectName);
    QString missionText(int mode) const;
    QString statusText(const RobotSnapshot &snapshot) const;
    void restyleState(const QString &stateName);
    void randomizeFakeTelemetry();
    void updateFakeTelemetryLabels(const QString &textColor);
    bool partStreamInactiveForCard(const RobotSnapshot &snapshot, int partIndex) const;
    bool displayedPartsAreNormalForCard(const RobotSnapshot &snapshot) const;
    int displayedOfflinePartCountForCard(const RobotSnapshot &snapshot) const;

    int m_robotId = 0;
    QLabel *m_name = nullptr;
    QLabel *m_statusBadge = nullptr;
    QLabel *m_target = nullptr;
    QLabel *m_battery = nullptr;
    QProgressBar *m_batteryBar = nullptr;
    QLabel *m_positionX = nullptr;
    QLabel *m_positionY = nullptr;
    QLabel *m_rtt = nullptr;
    QLabel *m_packetLoss = nullptr;
    QLabel *m_linkQuality = nullptr;
    QLabel *m_speed = nullptr;
    QLabel *m_temp = nullptr;
    QLabel *m_camera = nullptr;
    QWidget *m_robotView = nullptr;
    QVector<QFrame *> m_sensorCards;
    QVector<QLabel *> m_sensorChips;
    RobotSnapshot m_snapshot;
    float m_fakeSpeedMps = 0.0f;
    float m_fakeTempC = 0.0f;
    bool m_fakeTelemetryValid = false;
    bool m_lidarEverSeen = false;
    float m_lastPoseX = 0.0f;
    float m_lastPoseY = 0.0f;
    float m_lastPoseTheta = 0.0f;
    bool m_haveLastPose = false;
    QElapsedTimer m_lastPoseChangeTimer;
};

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = nullptr);
    bool loadMapConfig(const QString &path);
    void setSnapshots(const QVector<RobotSnapshot> &snapshots);
    void setSelectedRobot(int robotId);
    void setViewMode3D(bool enabled);
    void fitToAvailableSize();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QVector<RobotSnapshot> m_snapshots;
    int m_selectedRobot = 0;
    bool m_view3d = false;
    QStackedWidget *m_stack = nullptr;
    LidarMap2DView *m_map2d = nullptr;
    PointCloud3DView *m_map3d = nullptr;
    QPushButton *m_resetViewButton = nullptr;
};

QFrame *makePanel(const QString &title, QWidget *body);
QLabel *makeMetric(const QString &title, const QString &value, const QString &color);

#endif
