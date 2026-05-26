#include "mainwindow.h"

#include <QAbstractItemView>
#include <QByteArray>
#include <QDateTime>
#include <QCoreApplication>
#include <QDateEdit>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QInputMethod>
#include <QLineEdit>
#include <QListView>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QResizeEvent>
#include <QScreen>
#include <QScrollArea>
#include <QShortcut>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

extern "C" {
#include "proto.h"
}

static QString robotName(int id)
{
    return QString("SPOT-%1").arg(id + 1, 2, 10, QLatin1Char('0'));
}

static constexpr int kAllRobotsSelection = -1;
static constexpr int kDisplaySwapA = 0;
static constexpr int kDisplaySwapB = 4;

static int swappedRobotId(int robotId)
{
    if (robotId == kDisplaySwapA) {
        return kDisplaySwapB;
    }
    if (robotId == kDisplaySwapB) {
        return kDisplaySwapA;
    }
    return robotId;
}

static int displayToPhysicalRobotId(int displayRobotId)
{
    return swappedRobotId(displayRobotId);
}

static int physicalToDisplayRobotId(int physicalRobotId)
{
    return swappedRobotId(physicalRobotId);
}

static bool isHardcodedDisplayRobot(int displayRobotId)
{
    Q_UNUSED(displayRobotId);
    return false;
}

static int featuredVideoRobotForSelection(int selectedRobot, int robotCount)
{
    const int clamped = qBound(1, robotCount, kMaxRobots);
    if (selectedRobot == kAllRobotsSelection) {
        return clamped == 5 ? 4 : 0;
    }
    return (selectedRobot >= 0 && selectedRobot < clamped) ? selectedRobot : 0;
}

static RobotSnapshot hardcodedDisplaySnapshot(int displayId)
{
    RobotSnapshot snapshot;
    snapshot.id = displayId;
    return snapshot;
}

static QVector<RobotSnapshot> remapSnapshotsForDisplay(const QVector<RobotSnapshot> &snapshots)
{
    int displaySize = snapshots.size();
    for (const RobotSnapshot &snapshot : snapshots) {
        displaySize = qMax(displaySize, snapshot.id + 1);
    }
    displaySize = qMax(displaySize, qMax(kDisplaySwapA, kDisplaySwapB) + 1);
    QVector<RobotSnapshot> display(displaySize);
    for (int i = 0; i < display.size(); ++i) {
        display[i] = hardcodedDisplaySnapshot(i);
    }

    for (const RobotSnapshot &snapshot : snapshots) {
        const int displayId = physicalToDisplayRobotId(snapshot.id);

        if (displayId >= 0 && displayId < display.size()) {
            RobotSnapshot remapped = snapshot;
            remapped.id = displayId;
            display[displayId] = remapped;
        }
    }
    return display;
}

static QVector<RobotSnapshot> snapshotsForDeployedRobots(const QVector<RobotSnapshot> &snapshots, int robotCount)
{
    QVector<RobotSnapshot> deployed;
    const int count = qBound(0, robotCount, snapshots.size());
    deployed.reserve(count);
    for (int i = 0; i < count; ++i) {
        deployed.append(snapshots[i]);
    }
    return deployed;
}

static bool routeEligibleSnapshot(const RobotSnapshot &snapshot)
{
    return !isHardcodedDisplayRobot(snapshot.id)
        && snapshot.shmOpen && snapshot.connected && snapshot.faultLevel < 3;
}

static QVector<int> deployedRobotIds(const QVector<RobotSnapshot> &snapshots, int robotCount)
{
    QVector<int> ids;
    const QVector<RobotSnapshot> deployedSnapshots = snapshotsForDeployedRobots(snapshots, robotCount);
    ids.reserve(deployedSnapshots.size());
    for (const RobotSnapshot &snapshot : deployedSnapshots) {
        if (snapshot.id >= 0 && snapshot.id < kMaxRobots
            && !isHardcodedDisplayRobot(snapshot.id)) {
            ids.append(snapshot.id);
        }
    }
    return ids;
}

static int displayBatteryPercent(int robotId, float snapshotBattery)
{
    if (robotId == 0) {
        return 90;
    }
    if (robotId == 1) {
        return 70;
    }
    if (robotId == 2) {
        return 89;
    }
    if (robotId == 3) {
        return 91;
    }
    if (robotId == 4) {
        return 77;
    }
    return qBound(0, qRound(snapshotBattery), 100);
}

static constexpr int kDefaultRobotCount = 4;
static constexpr int kRobotCardGridColumns = 2;
static constexpr int kVideoLayoutBaseRows = 4;
static constexpr int kVideoLayoutBaseColumns = 4;
static constexpr int kVideoLayoutScale = 4;
static constexpr int kVideoLayoutRows = kVideoLayoutBaseRows * kVideoLayoutScale;
static constexpr int kVideoLayoutColumns = kVideoLayoutBaseColumns * kVideoLayoutScale;

struct GridSpec {
    int columns = 1;
    int rows = 1;
};

struct VideoLayoutSlot {
    int row = 0;
    int column = 0;
    int rowSpan = 1;
    int columnSpan = 1;
};

struct VideoLayoutSpec {
    int rows = 1;
    int columns = 1;
    QVector<int> rowStretch;
    QVector<int> columnStretch;
    QVector<VideoLayoutSlot> placements;
    bool hasFeaturedTile = false;
};

static GridSpec statusGridSpec(int count)
{
    const int clamped = qBound(1, count, kMaxRobots);
    if (clamped <= 5) {
        return {1, clamped};
    }
    return {clamped, 1};
}

static VideoLayoutSpec videoLayoutSpec(int count)
{
    const int clamped = qBound(1, count, kMaxRobots);
    switch (clamped) {
    case 1:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1}, {{0, 0, 4, 4}}, false};
    case 2:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 4, 2}, {0, 2, 4, 2}}, false};
    case 3:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 4, 2}, {0, 2, 2, 2}, {2, 2, 2, 2}}, true};
    case 4:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 2, 2}, {0, 2, 2, 2}, {2, 0, 2, 2}, {2, 2, 2, 2}}, false};
    case 5:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 4, 2}, {0, 2, 2, 1}, {0, 3, 2, 1},
                 {2, 2, 2, 1}, {2, 3, 2, 1}}, true};
    case 6:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 2, 2}, {0, 2, 1, 2}, {2, 0, 2, 1},
                 {2, 1, 1, 1}, {1, 2, 2, 2}, {3, 1, 1, 3}}, true};
    case 7:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 2, 2}, {0, 2, 1, 1}, {0, 3, 1, 1},
                 {1, 2, 1, 1}, {1, 3, 1, 1}, {2, 0, 2, 2},
                 {2, 2, 2, 2}}, true};
    case 8:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 2, 2}, {0, 2, 1, 1}, {0, 3, 1, 1},
                 {1, 2, 1, 1}, {1, 3, 1, 1}, {2, 0, 1, 1},
                 {2, 1, 1, 1}, {2, 2, 2, 2}}, true};
    case 9:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 1, 1}, {0, 1, 1, 1}, {0, 2, 1, 1},
                 {1, 0, 1, 1}, {1, 1, 1, 1}, {1, 2, 1, 1},
                 {2, 0, 1, 1}, {2, 1, 1, 1}, {2, 2, 2, 2}}, false};
    default:
        return {4, 4, {1, 1, 1, 1}, {1, 1, 1, 1},
                {{0, 0, 1, 1}, {0, 1, 1, 1}, {0, 2, 1, 1},
                 {0, 3, 1, 1}, {1, 0, 1, 1}, {1, 1, 1, 1},
                 {1, 2, 1, 1}, {1, 3, 1, 1}, {2, 0, 2, 2},
                 {2, 2, 2, 2}}, false};
    }
}

static bool videoSlotInBounds(const VideoLayoutSlot &slot)
{
    return slot.row >= 0 && slot.column >= 0
        && slot.rowSpan >= 1 && slot.columnSpan >= 1
        && slot.row + slot.rowSpan <= kVideoLayoutRows
        && slot.column + slot.columnSpan <= kVideoLayoutColumns;
}

static int videoSlotArea(const VideoLayoutSlot &slot)
{
    return slot.rowSpan * slot.columnSpan;
}

static VideoLayoutSlot scaledVideoSlot(const VideoLayoutSlot &slot)
{
    return {slot.row * kVideoLayoutScale,
            slot.column * kVideoLayoutScale,
            slot.rowSpan * kVideoLayoutScale,
            slot.columnSpan * kVideoLayoutScale};
}

static void splitVideoRect(const VideoLayoutSlot &rect, int count,
                           QVector<VideoLayoutSlot> *placements)
{
    if (count <= 0 || !videoSlotInBounds(rect) || videoSlotArea(rect) < count) {
        return;
    }
    if (count == 1) {
        placements->append(rect);
        return;
    }

    const bool splitColumns = rect.columnSpan >= rect.rowSpan;
    const int firstCount = count / 2;
    const int secondCount = count - firstCount;

    if (splitColumns && rect.columnSpan >= 2) {
        int firstSpan = qRound(static_cast<double>(rect.columnSpan) * firstCount / count);
        firstSpan = qBound(1, firstSpan, rect.columnSpan - 1);
        while (firstSpan * rect.rowSpan < firstCount && firstSpan < rect.columnSpan - 1) {
            ++firstSpan;
        }
        while ((rect.columnSpan - firstSpan) * rect.rowSpan < secondCount && firstSpan > 1) {
            --firstSpan;
        }
        splitVideoRect({rect.row, rect.column, rect.rowSpan, firstSpan}, firstCount, placements);
        splitVideoRect({rect.row, rect.column + firstSpan, rect.rowSpan,
                        rect.columnSpan - firstSpan}, secondCount, placements);
        return;
    }

    if (rect.rowSpan >= 2) {
        int firstSpan = qRound(static_cast<double>(rect.rowSpan) * firstCount / count);
        firstSpan = qBound(1, firstSpan, rect.rowSpan - 1);
        while (firstSpan * rect.columnSpan < firstCount && firstSpan < rect.rowSpan - 1) {
            ++firstSpan;
        }
        while ((rect.rowSpan - firstSpan) * rect.columnSpan < secondCount && firstSpan > 1) {
            --firstSpan;
        }
        splitVideoRect({rect.row, rect.column, firstSpan, rect.columnSpan}, firstCount, placements);
        splitVideoRect({rect.row + firstSpan, rect.column, rect.rowSpan - firstSpan,
                        rect.columnSpan}, secondCount, placements);
    }
}

static bool videoSlotCellsAvailable(const VideoLayoutSlot &slot,
                                    const bool occupied[kVideoLayoutRows][kVideoLayoutColumns],
                                    const bool covered[kVideoLayoutRows][kVideoLayoutColumns])
{
    if (!videoSlotInBounds(slot)) {
        return false;
    }

    for (int row = slot.row; row < slot.row + slot.rowSpan; ++row) {
        for (int col = slot.column; col < slot.column + slot.columnSpan; ++col) {
            if (occupied[row][col] || covered[row][col]) {
                return false;
            }
        }
    }
    return true;
}

static int remainingFreeVideoCells(const bool occupied[kVideoLayoutRows][kVideoLayoutColumns],
                                   const bool covered[kVideoLayoutRows][kVideoLayoutColumns])
{
    int count = 0;
    for (int row = 0; row < kVideoLayoutRows; ++row) {
        for (int col = 0; col < kVideoLayoutColumns; ++col) {
            if (!occupied[row][col] && !covered[row][col]) {
                ++count;
            }
        }
    }
    return count;
}

static bool partitionVideoCells(const bool occupied[kVideoLayoutRows][kVideoLayoutColumns],
                                bool covered[kVideoLayoutRows][kVideoLayoutColumns],
                                int slotsLeft,
                                QVector<VideoLayoutSlot> *placements)
{
    int firstRow = -1;
    int firstCol = -1;
    for (int row = 0; row < kVideoLayoutRows && firstRow < 0; ++row) {
        for (int col = 0; col < kVideoLayoutColumns; ++col) {
            if (!occupied[row][col] && !covered[row][col]) {
                firstRow = row;
                firstCol = col;
                break;
            }
        }
    }

    if (firstRow < 0) {
        return slotsLeft == 0;
    }
    if (slotsLeft <= 0) {
        return false;
    }

    const int freeCells = remainingFreeVideoCells(occupied, covered);
    if (freeCells < slotsLeft) {
        return false;
    }

    QVector<VideoLayoutSlot> candidates;
    for (int rowSpan = 1; firstRow + rowSpan <= kVideoLayoutRows; ++rowSpan) {
        for (int columnSpan = 1; firstCol + columnSpan <= kVideoLayoutColumns; ++columnSpan) {
            VideoLayoutSlot slot{firstRow, firstCol, rowSpan, columnSpan};
            if (!videoSlotCellsAvailable(slot, occupied, covered)) {
                continue;
            }
            if (freeCells - videoSlotArea(slot) < slotsLeft - 1) {
                continue;
            }
            candidates.append(slot);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const VideoLayoutSlot &a,
                                                       const VideoLayoutSlot &b) {
        const int areaA = videoSlotArea(a);
        const int areaB = videoSlotArea(b);
        if (areaA != areaB) {
            return areaA > areaB;
        }
        const int balanceA = qAbs(a.rowSpan - a.columnSpan);
        const int balanceB = qAbs(b.rowSpan - b.columnSpan);
        if (balanceA != balanceB) {
            return balanceA < balanceB;
        }
        return a.columnSpan > b.columnSpan;
    });

    for (const VideoLayoutSlot &slot : candidates) {
        for (int row = slot.row; row < slot.row + slot.rowSpan; ++row) {
            for (int col = slot.column; col < slot.column + slot.columnSpan; ++col) {
                covered[row][col] = true;
            }
        }
        placements->append(slot);
        if (partitionVideoCells(occupied, covered, slotsLeft - 1, placements)) {
            return true;
        }
        placements->removeLast();
        for (int row = slot.row; row < slot.row + slot.rowSpan; ++row) {
            for (int col = slot.column; col < slot.column + slot.columnSpan; ++col) {
                covered[row][col] = false;
            }
        }
    }

    return false;
}

static bool arrangeRemainingVideoSlots(const VideoLayoutSlot &fixedSlot,
                                       int remainingTileCount,
                                       QVector<VideoLayoutSlot> *placements)
{
    if (!videoSlotInBounds(fixedSlot)) {
        return false;
    }

    const int freeCells = kVideoLayoutRows * kVideoLayoutColumns - videoSlotArea(fixedSlot);
    if (freeCells < remainingTileCount) {
        return false;
    }

    placements->clear();

    struct Region {
        VideoLayoutSlot rect;
        int count = 0;
    };

    QVector<Region> regions;
    if (fixedSlot.row > 0) {
        regions.append({{0, 0, fixedSlot.row, kVideoLayoutColumns}, 0});
    }
    const int fixedBottom = fixedSlot.row + fixedSlot.rowSpan;
    if (fixedBottom < kVideoLayoutRows) {
        regions.append({{fixedBottom, 0, kVideoLayoutRows - fixedBottom, kVideoLayoutColumns}, 0});
    }
    if (fixedSlot.column > 0) {
        regions.append({{fixedSlot.row, 0, fixedSlot.rowSpan, fixedSlot.column}, 0});
    }
    const int fixedRight = fixedSlot.column + fixedSlot.columnSpan;
    if (fixedRight < kVideoLayoutColumns) {
        regions.append({{fixedSlot.row, fixedRight, fixedSlot.rowSpan,
                         kVideoLayoutColumns - fixedRight}, 0});
    }

    if (remainingTileCount == 0) {
        return true;
    }
    if (regions.isEmpty()) {
        return false;
    }

    for (int i = 0; i < remainingTileCount; ++i) {
        int bestIndex = -1;
        double bestScore = -1.0;
        for (int regionIndex = 0; regionIndex < regions.size(); ++regionIndex) {
            const int area = videoSlotArea(regions[regionIndex].rect);
            if (regions[regionIndex].count >= area) {
                continue;
            }
            const double score = static_cast<double>(area) / (regions[regionIndex].count + 1);
            if (score > bestScore) {
                bestScore = score;
                bestIndex = regionIndex;
            }
        }
        if (bestIndex < 0) {
            return false;
        }
        ++regions[bestIndex].count;
    }

    for (const Region &region : regions) {
        splitVideoRect(region.rect, region.count, placements);
    }

    return placements->size() == remainingTileCount;
}

static int demoRobotCountFromEnvironment()
{
    const QByteArray value = qgetenv("DISASTER_QT_DEMO_ROBOTS");
    if (value.isEmpty()) {
        return 0;
    }

    bool ok = false;
    const int count = value.toInt(&ok);
    return ok ? qBound(1, count, kMaxRobots) : 0;
}

static int missionProgressPercent(const RobotSnapshot &snapshot)
{
    const float percent = snapshot.missionProgress > 1.0f
        ? snapshot.missionProgress
        : snapshot.missionProgress * 100.0f;
    return qBound(0, qRound(percent), 100);
}

static bool missionComplete(const RobotSnapshot &snapshot)
{
    return snapshot.goalReached || missionProgressPercent(snapshot) >= 100;
}

static bool robotPoseAvailable(const RobotSnapshot &snapshot)
{
    return snapshot.shmOpen && (snapshot.odomTimestampUs != 0 || snapshot.odomSeq != 0);
}

static bool robotEverConnected(const RobotSnapshot &snapshot)
{
    return snapshot.lastRxUs != 0 || snapshot.rxPackets > 0 || snapshot.odomSeq != 0;
}

static bool robotConnectionLost(const RobotSnapshot &snapshot)
{
    return snapshot.shmOpen && !snapshot.connected && robotEverConnected(snapshot);
}

static bool robotMissionStarted(const RobotSnapshot &snapshot)
{
    if (!robotPoseAvailable(snapshot)) {
        return false;
    }
    const float linearSpeed = std::hypot(snapshot.vx, snapshot.vy);
    return snapshot.commandMoving
        || linearSpeed > 0.02f
        || std::fabs(snapshot.omega) > 0.02f;
}

static QString robotMissionStateText(const RobotSnapshot &snapshot)
{
    if (!snapshot.shmOpen) {
        return QStringLiteral("미연결");
    }
    if (robotConnectionLost(snapshot)) {
        return QStringLiteral("연결 끊김");
    }
    if (!snapshot.connected) {
        return QStringLiteral("연결 안됨");
    }
    if (missionComplete(snapshot)) {
        return QStringLiteral("도착");
    }
    if (robotMissionStarted(snapshot)) {
        return QStringLiteral("탐색 중");
    }
    return QStringLiteral("대기");
}

static bool hasPathProgressData(const RobotSnapshot &snapshot)
{
    return snapshot.pathOk || snapshot.poseOk || snapshot.goalReached
        || snapshot.waypointIdx > 0 || snapshot.missionProgress > 0.0f
        || snapshot.distanceToGoalM > 0.0f;
}

static QString pathDistanceSummary(const RobotSnapshot &snapshot)
{
    if (!hasPathProgressData(snapshot)
        || !std::isfinite(snapshot.distanceToGoalM)
        || snapshot.distanceToGoalM <= 0.0f) {
        return QStringLiteral("--");
    }
    return QString("%1 m").arg(snapshot.distanceToGoalM, 0, 'f', 1);
}

static QString waypointSummary(const RobotSnapshot &snapshot)
{
    if (snapshot.totalWaypoints > 0) {
        return QString("%1/%2").arg(snapshot.waypointIdx).arg(snapshot.totalWaypoints);
    }
    return QString::number(snapshot.waypointIdx);
}

class NavWatermark : public QWidget
{
public:
    explicit NavWatermark(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setFixedHeight(190);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        const QRectF frame = rect().adjusted(2.0, 2.0, -2.0, -2.0);
        p.setPen(QPen(QColor(255, 216, 64), 5.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.setBrush(QColor(8, 19, 31));
        p.drawRoundedRect(frame, 16.0, 16.0);

        p.setClipPath([frame]() {
            QPainterPath path;
            path.addRoundedRect(frame.adjusted(3.0, 3.0, -3.0, -3.0), 12.0, 12.0);
            return path;
        }());

        auto toPoint = [frame](qreal x, qreal y) {
            return QPointF(frame.left() + frame.width() * x,
                           frame.top() + frame.height() * y);
        };
        auto drawStroke = [&p, &toPoint](std::initializer_list<QPointF> points, qreal width = 3.0) {
            if (points.size() < 2) {
                return;
            }
            QPainterPath path;
            auto it = points.begin();
            path.moveTo(toPoint(it->x(), it->y()));
            QPointF prev = *it++;
            for (; it != points.end(); ++it) {
                const QPointF current = *it;
                const QPointF mid((prev.x() + current.x()) * 0.5, (prev.y() + current.y()) * 0.5);
                path.quadTo(toPoint(prev.x(), prev.y()), toPoint(mid.x(), mid.y()));
                prev = current;
            }
            path.lineTo(toPoint(prev.x(), prev.y()));
            p.setPen(QPen(Qt::white, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.setBrush(Qt::NoBrush);
            p.drawPath(path);
        };

        drawStroke({{0.58, 0.06}, {0.36, 0.09}, {0.31, 0.13}, {0.43, 0.16}, {0.50, 0.18},
                    {0.35, 0.22}, {0.24, 0.23}}, 2.8);
        drawStroke({{0.57, 0.18}, {0.52, 0.25}, {0.47, 0.33}, {0.35, 0.41}}, 3.0);
        drawStroke({{0.50, 0.19}, {0.69, 0.20}, {0.70, 0.25}, {0.57, 0.26}}, 3.0);
        drawStroke({{0.43, 0.46}, {0.34, 0.49}, {0.31, 0.54}, {0.34, 0.58}, {0.48, 0.57},
                    {0.55, 0.51}, {0.50, 0.46}}, 3.0);
        drawStroke({{0.23, 0.66}, {0.52, 0.68}, {0.72, 0.69}}, 3.0);
        drawStroke({{0.54, 0.69}, {0.43, 0.74}, {0.35, 0.81}}, 3.0);

        drawStroke({{0.24, 0.91}, {0.45, 0.91}, {0.60, 0.92}, {0.47, 0.98}, {0.30, 1.05},
                    {0.21, 1.00}, {0.18, 0.96}, {0.22, 0.92}, {0.31, 0.88}}, 3.1);
        drawStroke({{0.66, 0.89}, {0.62, 0.99}, {0.83, 0.98}}, 3.1);
        drawStroke({{0.65, 0.89}, {0.82, 0.88}, {0.96, 0.88}}, 3.1);
        drawStroke({{0.64, 0.94}, {0.79, 0.94}}, 3.1);
        drawStroke({{0.78, 0.88}, {1.08, 0.88}}, 3.1);
        drawStroke({{0.94, 0.88}, {0.92, 1.07}}, 3.1);
    }
};

class SideNavButton : public QPushButton
{
public:
    explicit SideNavButton(const QString &text, QWidget *parent = nullptr)
        : QPushButton(text, parent)
    {
        setCursor(Qt::PointingHandCursor);
        setFlat(true);
        setMinimumSize(88, 96);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setRenderHint(QPainter::TextAntialiasing, true);

        const bool active = objectName() == QStringLiteral("navButtonActive");
        const bool hover = underMouse();
        const QColor accent = active ? QColor(226, 184, 57) : QColor(145, 158, 171);
        const QColor textColor = active ? QColor(242, 201, 76) : QColor(164, 176, 188);
        QRectF card = rect().adjusted(2.0, 4.0, -2.0, -4.0);

        if (active) {
            QLinearGradient fill(card.topLeft(), card.bottomLeft());
            fill.setColorAt(0.0, QColor(43, 34, 8, 232));
            fill.setColorAt(0.55, QColor(31, 27, 13, 228));
            fill.setColorAt(1.0, QColor(17, 21, 18, 226));
            p.setPen(QPen(QColor(130, 99, 18, 110), 2.4));
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(card.adjusted(2.0, 2.0, -2.0, -2.0), 8.0, 8.0);
            p.setPen(QPen(QColor(224, 171, 33), 0.9));
            p.setBrush(fill);
            p.drawRoundedRect(card, 7.0, 7.0);
        } else {
            if (hover) {
                p.setPen(QPen(QColor(55, 78, 98), 1.0));
                p.setBrush(QColor(9, 22, 33, 150));
                p.drawRoundedRect(card.adjusted(4.0, 4.0, -4.0, -4.0), 7.0, 7.0);
            }
        }

        const QPointF iconCenter(width() * 0.5, card.top() + 30.0);
        drawIcon(&p, iconCenter, accent);

        QFont font(QStringLiteral("Noto Sans"), active ? 12 : 12, QFont::Black);
        p.setFont(font);
        p.setPen(textColor);
        p.drawText(QRectF(card.left() + 4.0, card.top() + 60.0,
                          card.width() - 8.0, card.height() - 64.0),
                   Qt::AlignHCenter | Qt::AlignTop | Qt::TextWordWrap,
                   text().replace('\n', ' '));
    }

private:
    void drawIcon(QPainter *p, const QPointF &c, const QColor &color)
    {
        const QString label = text().remove('\n');
        QPen glow(QColor(color.red(), color.green(), color.blue(), 44), 3.0,
                  Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QPen pen(color, 1.55, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        p->save();
        p->translate(c);
        p->scale(0.86, 0.86);
        p->translate(-c);
        p->setBrush(Qt::NoBrush);
        if (label == QStringLiteral("관제")) {
            p->setPen(glow);
            p->drawLine(QPointF(c.x(), c.y() - 17.0), QPointF(c.x(), c.y() - 6.0));
            p->drawLine(QPointF(c.x() - 14.0, c.y() + 5.0), QPointF(c.x() - 5.0, c.y() - 3.0));
            p->drawLine(QPointF(c.x() + 14.0, c.y() + 5.0), QPointF(c.x() + 5.0, c.y() - 3.0));
            p->drawLine(QPointF(c.x() - 14.0, c.y() + 5.0), QPointF(c.x() - 14.0, c.y() + 17.0));
            p->drawLine(QPointF(c.x() + 14.0, c.y() + 5.0), QPointF(c.x() + 14.0, c.y() + 17.0));
            p->setPen(pen);
            p->drawLine(QPointF(c.x(), c.y() - 17.0), QPointF(c.x(), c.y() - 6.0));
            p->drawLine(QPointF(c.x() - 14.0, c.y() + 5.0), QPointF(c.x() - 5.0, c.y() - 3.0));
            p->drawLine(QPointF(c.x() + 14.0, c.y() + 5.0), QPointF(c.x() + 5.0, c.y() - 3.0));
            p->drawLine(QPointF(c.x() - 14.0, c.y() + 5.0), QPointF(c.x() - 14.0, c.y() + 17.0));
            p->drawLine(QPointF(c.x() + 14.0, c.y() + 5.0), QPointF(c.x() + 14.0, c.y() + 17.0));
            p->drawRoundedRect(QRectF(c.x() - 7.0, c.y() - 7.0, 14.0, 14.0), 3.0, 3.0);
            p->drawRoundedRect(QRectF(c.x() - 21.0, c.y() + 13.0, 14.0, 14.0), 3.0, 3.0);
            p->drawRoundedRect(QRectF(c.x() + 7.0, c.y() + 13.0, 14.0, 14.0), 3.0, 3.0);
            p->setBrush(color);
            p->drawEllipse(QPointF(c.x(), c.y()), 2.0, 2.0);
            p->drawEllipse(QPointF(c.x() - 14.0, c.y() + 20.0), 2.0, 2.0);
            p->drawEllipse(QPointF(c.x() + 14.0, c.y() + 20.0), 2.0, 2.0);
        } else if (label == QStringLiteral("로봇상태")) {
            const QRectF panel(c.x() - 22.0, c.y() - 17.0, 44.0, 33.0);
            p->setPen(glow);
            p->drawRoundedRect(panel, 3.5, 3.5);
            p->setPen(pen);
            p->drawRoundedRect(panel, 3.5, 3.5);
            p->drawLine(QPointF(c.x() - 14.0, c.y() + 21.0), QPointF(c.x() + 14.0, c.y() + 21.0));
            p->drawLine(QPointF(c.x(), c.y() + 16.0), QPointF(c.x(), c.y() + 21.0));

            QPainterPath pulse;
            pulse.moveTo(c.x() - 15.0, c.y() - 2.0);
            pulse.lineTo(c.x() - 8.0, c.y() - 2.0);
            pulse.lineTo(c.x() - 4.0, c.y() - 9.0);
            pulse.lineTo(c.x() + 1.0, c.y() + 6.0);
            pulse.lineTo(c.x() + 6.0, c.y() - 4.0);
            pulse.lineTo(c.x() + 15.0, c.y() - 4.0);
            p->drawPath(pulse);

            p->drawLine(QPointF(c.x() - 15.0, c.y() + 8.0), QPointF(c.x() + 15.0, c.y() + 8.0));
            p->setBrush(color);
            p->drawRect(QRectF(c.x() - 15.0, c.y() + 10.0, 4.0, 3.2));
            p->drawRect(QRectF(c.x() - 5.0, c.y() + 10.0, 4.0, 3.2));
            p->drawRect(QRectF(c.x() + 5.0, c.y() + 10.0, 4.0, 3.2));
            p->drawEllipse(QPointF(c.x() + 15.0, c.y() + 11.5), 1.7, 1.7);
        } else if (label == QStringLiteral("로그")) {
            QPainterPath doc;
            doc.moveTo(c.x() - 14.0, c.y() - 20.0);
            doc.lineTo(c.x() + 6.0, c.y() - 20.0);
            doc.lineTo(c.x() + 18.0, c.y() - 8.0);
            doc.lineTo(c.x() + 18.0, c.y() + 20.0);
            doc.lineTo(c.x() - 14.0, c.y() + 20.0);
            doc.closeSubpath();
            p->setPen(glow);
            p->drawPath(doc);
            p->setPen(pen);
            p->drawPath(doc);
            p->drawLine(QPointF(c.x() + 6.0, c.y() - 20.0), QPointF(c.x() + 6.0, c.y() - 8.0));
            p->drawLine(QPointF(c.x() + 6.0, c.y() - 8.0), QPointF(c.x() + 18.0, c.y() - 8.0));
            p->drawLine(QPointF(c.x() - 7.0, c.y() - 1.0), QPointF(c.x() + 7.0, c.y() - 1.0));
            p->drawLine(QPointF(c.x() - 7.0, c.y() + 8.0), QPointF(c.x() + 10.0, c.y() + 8.0));
        } else {
            QPainterPath gear;
            for (int i = 0; i < 16; ++i) {
                const qreal a = (-90.0 + i * 22.5) * M_PI / 180.0;
                const qreal r = (i % 2 == 0) ? 20.0 : 15.0;
                const QPointF pt(c.x() + std::cos(a) * r, c.y() + std::sin(a) * r);
                i == 0 ? gear.moveTo(pt) : gear.lineTo(pt);
            }
            gear.closeSubpath();
            p->setPen(glow);
            p->drawPath(gear);
            p->setPen(pen);
            p->drawPath(gear);
            p->drawEllipse(c, 7.5, 7.5);
        }
        p->restore();
    }
};

class BorderlessComboBox : public QComboBox
{
public:
    explicit BorderlessComboBox(QWidget *parent = nullptr)
        : QComboBox(parent)
    {
    }

    void showPopup() override
    {
        ensurePopup();
        rebuildPopupItems();

        const int popupWidth = qMax(width(), 1);
        const int visibleRows = qMin(count(), qMax(1, maxVisibleItems()));
        m_popupList->setVerticalScrollBarPolicy(count() > visibleRows
                                                    ? Qt::ScrollBarAsNeeded
                                                    : Qt::ScrollBarAlwaysOff);
        m_popup->setFixedSize(popupWidth, visibleRows * kPopupRowHeight);

        const QPoint comboTopLeft = mapToGlobal(QPoint(0, 0));
        QPoint popupPos(comboTopLeft.x(), comboTopLeft.y() + height() / 2 - m_popup->height() / 2);
        constexpr int topMargin = 12;
        constexpr int bottomMargin = 44;
        if (QScreen *screen = QGuiApplication::screenAt(popupPos)) {
            const QRect available = screen->availableGeometry();
            const int minTop = available.top() + topMargin;
            const int maxTop = available.bottom() - bottomMargin - m_popup->height();
            popupPos.setY(qBound(minTop, popupPos.y(), qMax(minTop, maxTop)));
        }
        m_popup->move(popupPos);
        m_popup->show();
        m_popup->raise();
        m_popupList->setFocus(Qt::PopupFocusReason);
    }

    void hidePopup() override
    {
        if (m_popup) {
            m_popup->hide();
        }
        QComboBox::hidePopup();
    }

private:
    static constexpr int kPopupRowHeight = 46;

    void ensurePopup()
    {
        if (m_popup) {
            return;
        }

        m_popup = new QFrame(this, Qt::Popup | Qt::FramelessWindowHint | Qt::NoDropShadowWindowHint);
        m_popup->setObjectName("comboPopup");
        m_popup->setFrameShape(QFrame::NoFrame);
        m_popup->setFrameStyle(QFrame::NoFrame);
        m_popup->setLineWidth(0);
        m_popup->setMidLineWidth(0);
        m_popup->setAttribute(Qt::WA_StyledBackground, true);
        m_popup->setAutoFillBackground(true);
        m_popup->setStyleSheet(
            "QFrame#comboPopup { background:#071017; border:0; margin:0; padding:0; }"
            "QListWidget { background:#071017; border:0; margin:0; padding:0; outline:0; }"
            "QListWidget::viewport { background:#071017; border:0; margin:0; padding:0; }"
            "QListWidget::item { background:#071017; color:#dce7f3; min-height:34px; padding:6px 10px; margin:0; border:0; }"
            "QListWidget::item:selected { background:#1b1805; color:#ffd21a; }"
            "QListWidget::item:focus { outline:0; }");

        QVBoxLayout *popupLayout = new QVBoxLayout(m_popup);
        popupLayout->setContentsMargins(0, 0, 0, 0);
        popupLayout->setSpacing(0);

        m_popupList = new QListWidget(m_popup);
        m_popupList->setFrameShape(QFrame::NoFrame);
        m_popupList->setFrameStyle(QFrame::NoFrame);
        m_popupList->setLineWidth(0);
        m_popupList->setMidLineWidth(0);
        m_popupList->setContentsMargins(0, 0, 0, 0);
        m_popupList->setSpacing(0);
        m_popupList->setUniformItemSizes(true);
        m_popupList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_popupList->setSelectionMode(QAbstractItemView::SingleSelection);
        m_popupList->setAttribute(Qt::WA_StyledBackground, true);
        m_popupList->viewport()->setAttribute(Qt::WA_StyledBackground, true);
        popupLayout->addWidget(m_popupList);

        connect(m_popupList, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
            if (!item) {
                return;
            }
            const int comboIndex = item->data(Qt::UserRole).toInt();
            if (comboIndex >= 0 && comboIndex < count()) {
                setCurrentIndex(comboIndex);
            }
            hidePopup();
        });
    }

    void rebuildPopupItems()
    {
        m_popupList->clear();
        for (int i = 0; i < count(); ++i) {
            auto *item = new QListWidgetItem(itemText(i));
            item->setData(Qt::UserRole, i);
            item->setSizeHint(QSize(qMax(width(), 1), kPopupRowHeight));
            m_popupList->addItem(item);
            if (i == currentIndex()) {
                m_popupList->setCurrentItem(item);
            }
        }
    }

    QFrame *m_popup = nullptr;
    QListWidget *m_popupList = nullptr;
};

static QString findMapYaml()
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QString cwd = QDir::currentPath();
    const QStringList candidates = {
        QDir(appDir).absoluteFilePath("../map_data/new_map.yaml"),
        QDir(appDir).absoluteFilePath("../../map_data/new_map.yaml"),
        QDir(appDir).absoluteFilePath("../../../map_data/new_map.yaml"),
        QDir(cwd).absoluteFilePath("map_data/new_map.yaml"),
        QDir(cwd).absoluteFilePath("../map_data/new_map.yaml"),
        QStringLiteral("/home/pi/robot_project/map_data/new_map.yaml"),
        QDir(appDir).absoluteFilePath("../map_data/map.yaml"),
        QDir(appDir).absoluteFilePath("../../map_data/map.yaml"),
        QDir(appDir).absoluteFilePath("../../../map_data/map.yaml"),
        QDir(cwd).absoluteFilePath("map_data/map.yaml"),
        QDir(cwd).absoluteFilePath("../map_data/map.yaml"),
        QStringLiteral("/home/pi/robot_project/map_data/map.yaml")
    };

    for (const QString &path : candidates) {
        QFileInfo info(path);
        if (info.exists() && info.isFile()) {
            return info.canonicalFilePath();
        }
    }
    return QString();
}

static QString logSeverityText(const QString &code)
{
    if (code == "CRITICAL") {
        return "위험";
    }
    if (code == "WARN") {
        return "주의";
    }
    return "정보";
}

static QString commandTypeText(uint8_t commandType)
{
    switch (commandType) {
    case CMD_TYPE_ESTOP:
        return "긴급 정지";
    case CMD_TYPE_STOP:
        return "정지";
    case CMD_TYPE_MOVE:
        return "이동";
    case CMD_TYPE_START_MISSION:
        return "탐색 시작";
    case CMD_TYPE_PAUSE_MISSION:
        return "일시 정지";
    case CMD_TYPE_CANCEL_MISSION:
        return "임무 취소";
    case CMD_TYPE_RETURN_HOME:
        return "복귀";
    case CMD_TYPE_SET_WAYPOINT:
        return "경로 시작점";
    case CMD_TYPE_SET_ROUTE:
        return "경로 생성";
    default:
        return QString("알 수 없는 명령(%1)").arg(commandType);
    }
}

static bool isBridgeCommandSentEvent(const UiEvent &event)
{
    return event.message.startsWith(QStringLiteral("cmd sent:"), Qt::CaseInsensitive);
}

static bool isReassemblyTimeoutEvent(const UiEvent &event)
{
    return event.message.contains(QStringLiteral("reassembly timeout"), Qt::CaseInsensitive)
        || event.message.contains(QStringLiteral("reasm timeout"), Qt::CaseInsensitive);
}

static bool isCommandAckEvent(const UiEvent &event)
{
    return event.type == kEventTypeCmdAck
        || event.message.compare(QStringLiteral("cmd ack"), Qt::CaseInsensitive) == 0;
}

class LogTrendWidget : public QWidget
{
public:
    explicit LogTrendWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(80, 38);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        const QVector<float> values = {0.25f, 0.32f, 0.28f, 0.52f, 0.44f, 0.82f, 0.55f, 0.90f};
        QPolygonF line;
        const QRectF r = rect().adjusted(4, 5, -4, -5);
        for (int i = 0; i < values.size(); ++i) {
            const qreal x = r.left() + r.width() * i / qMax(values.size() - 1, 1);
            const qreal y = r.bottom() - r.height() * values[i];
            line << QPointF(x, y);
        }
        p.setPen(QPen(QColor("#287dff"), 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(line);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(40, 125, 255, 70));
        for (const QPointF &pt : line) {
            p.drawEllipse(pt, 2.6, 2.6);
        }
    }
};

class SeverityDonutWidget : public QWidget
{
public:
    explicit SeverityDonutWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(72, 72);
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        const QRectF r = rect().adjusted(8, 8, -8, -8);
        const int infoCount = property("infoCount").toInt();
        const int warnCount = property("warnCount").toInt();
        const int criticalCount = property("criticalCount").toInt();
        const int total = infoCount + warnCount + criticalCount;
        if (total <= 0) {
            p.setPen(QPen(QColor("#26384b"), 12, Qt::SolidLine, Qt::FlatCap));
            p.drawArc(r, 0, 360 * 16);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor("#071321"));
            p.drawEllipse(r.adjusted(14, 14, -14, -14));
            return;
        }
        const QVector<QPair<QColor, int>> slices = {
            {QColor("#68df78"), infoCount},
            {QColor("#f6bd32"), warnCount},
            {QColor("#ff5b57"), criticalCount},
        };
        int start = 90 * 16;
        for (const auto &slice : slices) {
            if (slice.second <= 0) {
                continue;
            }
            const int span = -qRound(360.0 * slice.second / static_cast<double>(total) * 16.0);
            p.setPen(QPen(slice.first, 12, Qt::SolidLine, Qt::FlatCap));
            p.drawArc(r, start, span);
            start += span;
        }
        p.setPen(Qt::NoPen);
        p.setBrush(QColor("#071321"));
        p.drawEllipse(r.adjusted(14, 14, -14, -14));
    }
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    const int demoRobotCount = demoRobotCountFromEnvironment();
    if (demoRobotCount > 0) {
        m_demoMode = true;
        m_robotCount = demoRobotCount;
    } else {
        m_robotCount = kDefaultRobotCount;
    }

    buildUi();
    applyStyle();

    if (m_demoMode) {
        startDemoMode(m_robotCount);
    } else {
        connect(&m_monitor, &ShmMonitor::snapshotsUpdated, this, &MainWindow::updateSnapshots);
        connect(&m_monitor, &ShmMonitor::eventReceived, this, [this](UiEvent event) {
            if (event.robotId >= 0) {
                event.robotId = physicalToDisplayRobotId(event.robotId);
            }
            appendEvent(event);
        });
        m_monitor.start(kMaxRobots);
    }

    QTimer *clockTimer = new QTimer(this);
    connect(clockTimer, &QTimer::timeout, this, [this] {
        m_clock->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        updateMissionSummary();
    });
    clockTimer->start(1000);
    m_clock->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
}

MainWindow::~MainWindow()
{
}

void MainWindow::buildUi()
{
    resize(1680, 940);
    setWindowTitle("SPOT Get IT - 통합 관제 대시보드");

    QWidget *root = new QWidget(this);
    setCentralWidget(root);
    QHBoxLayout *rootLayout = new QHBoxLayout(root);
    rootLayout->setContentsMargins(0, 2, 12, 2);
    rootLayout->setSpacing(12);

    QFrame *nav = new QFrame;
    nav->setObjectName("nav");
    nav->setFixedWidth(126);
    QVBoxLayout *navLayout = new QVBoxLayout(nav);
    navLayout->setContentsMargins(14, 10, 14, 16);
    navLayout->setSpacing(12);
    auto addNavSeparator = [navLayout]() {
        QFrame *line = new QFrame;
        line->setObjectName("navSeparator");
        line->setFixedSize(56, 1);
        line->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        navLayout->addWidget(line, 0, Qt::AlignHCenter);
    };
    QPushButton *navExplore = makeNavButton("관제", true);
    QPushButton *navRobots = makeNavButton("로봇\n상태");
    QPushButton *navLogs = makeNavButton("로그");
    QPushButton *navSettings = makeNavButton("설정");
    m_navButtons = {navExplore, navRobots, navLogs, navSettings};
    navLayout->addSpacing(24);
    navLayout->addWidget(navExplore);
    addNavSeparator();
    navLayout->addWidget(navRobots);
    addNavSeparator();
    navLayout->addWidget(navLogs);
    addNavSeparator();
    navLayout->addWidget(navSettings);
    navLayout->addStretch(1);
    navLayout->addWidget(new NavWatermark(nav), 0, Qt::AlignHCenter);
    rootLayout->addWidget(nav);

    QVBoxLayout *main = new QVBoxLayout;
    main->setSpacing(12);
    rootLayout->addLayout(main, 1);

    m_victimAlertButton = new QPushButton(root);
    m_victimAlertButton->setObjectName("victimAlert");
    m_victimAlertButton->setCursor(Qt::PointingHandCursor);
    m_victimAlertButton->setText(QString());
    QVBoxLayout *victimAlertLayout = new QVBoxLayout(m_victimAlertButton);
    victimAlertLayout->setContentsMargins(32, 6, 32, 22);
    victimAlertLayout->setSpacing(12);
    m_victimAlertTitle = new QLabel("생존자 탐지", m_victimAlertButton);
    m_victimAlertTitle->setObjectName("victimAlertTitle");
    m_victimAlertTitle->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    m_victimAlertTitle->setAttribute(Qt::WA_TransparentForMouseEvents);
    QWidget *victimAlertFooter = new QWidget(m_victimAlertButton);
    victimAlertFooter->setObjectName("victimAlertFooter");
    victimAlertFooter->setFixedWidth(430);
    victimAlertFooter->setMinimumHeight(64);
    victimAlertFooter->setMaximumHeight(64);
    victimAlertFooter->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    victimAlertFooter->setAttribute(Qt::WA_TransparentForMouseEvents);
    QHBoxLayout *victimAlertFooterLayout = new QHBoxLayout(victimAlertFooter);
    victimAlertFooterLayout->setContentsMargins(0, 9, 0, 9);
    victimAlertFooterLayout->setSpacing(34);
    m_victimAlertRobot = new QLabel("SPOT-01", victimAlertFooter);
    m_victimAlertRobot->setObjectName("victimAlertSubtitle");
    m_victimAlertRobot->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_victimAlertAction = new QLabel("카메라 보기", victimAlertFooter);
    m_victimAlertAction->setObjectName("victimAlertSubtitle");
    m_victimAlertAction->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_victimAlertAction->setAttribute(Qt::WA_TransparentForMouseEvents);
    victimAlertFooterLayout->addStretch(1);
    victimAlertFooterLayout->addWidget(m_victimAlertRobot);
    victimAlertFooterLayout->addWidget(m_victimAlertAction);
    victimAlertFooterLayout->addStretch(1);
    victimAlertLayout->addStretch(1);
    victimAlertLayout->addWidget(m_victimAlertTitle, 1);
    victimAlertLayout->addWidget(victimAlertFooter, 0, Qt::AlignHCenter);
    victimAlertLayout->addSpacing(18);
    m_victimAlertButton->hide();
    connect(m_victimAlertButton, &QPushButton::clicked, this, [this]() {
        const int robotId = m_victimAlertRobotId;
        m_victimAlertButton->hide();
        if (robotId >= 0) {
            showCameraFullscreen(robotId);
        }
    });

    QFrame *top = new QFrame;
    top->setObjectName("topBar");
    QHBoxLayout *topLayout = new QHBoxLayout(top);
    topLayout->setContentsMargins(18, 10, 18, 10);
    QLabel *title = new QLabel("통합 관제 대시보드");
    title->setObjectName("title");
    m_crumb = new QLabel("  >  재난 탐색");
    m_crumb->setObjectName("accentText");
    m_system = new QLabel("● 시스템 대기");
    m_system->setObjectName("systemText");
    m_clock = new QLabel;
    m_clock->setObjectName("clockText");
    topLayout->addWidget(title);
    topLayout->addWidget(m_crumb);
    topLayout->addStretch();
    topLayout->addWidget(m_system);
    topLayout->addSpacing(18);
    topLayout->addWidget(m_clock);
    main->addWidget(top);

    m_contentStack = new QStackedWidget;
    main->addWidget(m_contentStack, 1);

    QWidget *dashboardPage = new QWidget;
    QVBoxLayout *dashboard = new QVBoxLayout(dashboardPage);
    dashboard->setContentsMargins(0, 0, 0, 0);
    dashboard->setSpacing(12);

    QHBoxLayout *metrics = new QHBoxLayout;
    metrics->setSpacing(12);
    metrics->addWidget(makeHeaderMetricCard("탐색 진행률", &m_metricProgress, "#18d878", &m_metricProgressBar), 3);
    metrics->addWidget(makeHeaderMetricCard("임무 수행 시간", &m_metricMissionTime, "#ffffff"), 2);
    metrics->addWidget(makeHeaderMetricCard("연결 로봇 갯수", &m_metricConnected, "#ffffff"), 2);
    metrics->addWidget(makeHeaderMetricCard("주요 이벤트", &m_metricEvents, "#ffffff"), 2);
    dashboard->addLayout(metrics);

    QHBoxLayout *upper = new QHBoxLayout;
    upper->setSpacing(12);
    QWidget *videoBody = new QWidget;
    m_videoGrid = new QGridLayout(videoBody);
    m_videoGrid->setContentsMargins(0, 0, 0, 0);
    m_videoGrid->setSpacing(8);
    m_videoGrid->setColumnStretch(0, 1);
    m_videoGrid->setColumnStretch(1, 1);
    m_videoScroll = new QScrollArea;
    m_videoScroll->setObjectName("panelScrollArea");
    m_videoScroll->setWidgetResizable(true);
    m_videoScroll->setFrameShape(QFrame::NoFrame);
    m_videoScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_videoScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_videoScroll->setWidget(videoBody);
    syncVideoTiles(m_robotCount);
    upper->addWidget(makePanel("실시간 영상 스트리밍", m_videoScroll), 5);

    QWidget *mapBody = new QWidget;
    m_mapLayout = new QVBoxLayout(mapBody);
    m_mapLayout->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout *mapToggle = new QHBoxLayout;
    m_btn2d = new QPushButton("2D");
    m_btn2d->setObjectName("smallActive");
    m_btn3d = new QPushButton("3D");
    m_btn3d->setObjectName("smallButton");
    QPushButton *mapMaximize = new QPushButton("최대화");
    mapMaximize->setObjectName("smallButton");
    mapToggle->addStretch();
    mapToggle->addWidget(m_btn2d);
    mapToggle->addWidget(m_btn3d);
    mapToggle->addWidget(mapMaximize);
    m_map = new MapWidget;
    const QString mapYaml = findMapYaml();
    if (!mapYaml.isEmpty()) {
        m_map->loadMapConfig(mapYaml);
    } else {
        qWarning() << "map_data/new_map.yaml or map_data/map.yaml not found; map overlay disabled";
    }
    connect(m_map, &MapWidget::routeGenerationRequested,
            this, &MainWindow::handleRouteGenerationRequested);
    m_mapLayout->addLayout(mapToggle);
    m_mapLayout->addWidget(m_map, 1);
    upper->addWidget(makePanel("탐색 지도", mapBody), 5);
    dashboard->addLayout(upper, 5);

    QHBoxLayout *lower = new QHBoxLayout;
    lower->setSpacing(12);

    QWidget *robotBody = new QWidget;
    m_robotStatusGrid = new QGridLayout(robotBody);
    m_robotStatusGrid->setContentsMargins(0, 0, 0, 0);
    m_robotStatusGrid->setSpacing(6);
    m_robotStatusScroll = new QScrollArea;
    m_robotStatusScroll->setObjectName("panelScrollArea");
    m_robotStatusScroll->setWidgetResizable(true);
    m_robotStatusScroll->setFrameShape(QFrame::NoFrame);
    m_robotStatusScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_robotStatusScroll->setWidget(robotBody);
    syncStatusRows(m_robotCount);
    lower->addWidget(makePanel("로봇 상태", m_robotStatusScroll), 4);

    m_eventList = new QListWidget;
    m_eventList->setObjectName("eventList");
    lower->addWidget(makePanel("이벤트 목록", m_eventList), 3);

    QWidget *controlBody = new QWidget;
    QVBoxLayout *control = new QVBoxLayout(controlBody);
    control->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout *robots = new QHBoxLayout;
    QLabel *robotSelectLabel = new QLabel("제어 로봇");
    robotSelectLabel->setObjectName("infoLine");
    m_robotSelector = new BorderlessComboBox;
    m_robotSelector->setMinimumHeight(44);
    m_robotSelector->setMaxVisibleItems(qMin(m_robotCount + 1, kMaxRobots + 1));
    QListView *selectorView = new QListView(m_robotSelector);
    selectorView->setFrameShape(QFrame::NoFrame);
    selectorView->setFrameStyle(QFrame::NoFrame);
    selectorView->setLineWidth(0);
    selectorView->setMidLineWidth(0);
    selectorView->setContentsMargins(0, 0, 0, 0);
    selectorView->setSpacing(0);
    selectorView->setUniformItemSizes(true);
    selectorView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    selectorView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    selectorView->viewport()->setContentsMargins(0, 0, 0, 0);
    selectorView->setAttribute(Qt::WA_StyledBackground, true);
    selectorView->viewport()->setAttribute(Qt::WA_StyledBackground, true);
    m_robotSelector->setView(selectorView);
    if (m_robotSelector->view()) {
        m_robotSelector->view()->setAutoFillBackground(true);
        m_robotSelector->view()->viewport()->setAutoFillBackground(true);
        m_robotSelector->view()->setStyleSheet(
            "QListView, QAbstractItemView { background:#071017; border:0; padding:0; margin:0; outline:0; }"
            "QListView::viewport, QAbstractScrollArea::viewport { background:#071017; border:0; margin:0; padding:0; }"
            "QListView::item, QAbstractItemView::item { background:#071017; color:#dce7f3; min-height:34px; padding:6px 10px; margin:0; border:0; }"
            "QListView::item:selected, QAbstractItemView::item:selected { background:#1b1805; color:#ffd21a; }"
            "QListView::item:disabled, QAbstractItemView::item:disabled { color:#64717d; background:#071017; }");
        QPalette popupPalette = m_robotSelector->view()->palette();
        popupPalette.setColor(QPalette::Base, QColor("#071017"));
        popupPalette.setColor(QPalette::Window, QColor("#071017"));
        popupPalette.setColor(QPalette::AlternateBase, QColor("#071017"));
        popupPalette.setColor(QPalette::Button, QColor("#071017"));
        popupPalette.setColor(QPalette::Light, QColor("#071017"));
        popupPalette.setColor(QPalette::Midlight, QColor("#071017"));
        popupPalette.setColor(QPalette::Mid, QColor("#071017"));
        popupPalette.setColor(QPalette::Dark, QColor("#071017"));
        popupPalette.setColor(QPalette::Shadow, QColor("#071017"));
        m_robotSelector->view()->setPalette(popupPalette);
        m_robotSelector->view()->viewport()->setPalette(popupPalette);
    }
    connect(m_robotSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (m_updatingRobotSelector || index < 0) {
            return;
        }
        const int robotId = m_robotSelector->itemData(index).toInt();
        selectRobot(robotId);
    });
    robots->addWidget(robotSelectLabel);
    robots->addWidget(m_robotSelector, 1);
    control->addLayout(robots);

    QHBoxLayout *commands = new QHBoxLayout;
    commands->setContentsMargins(0, 0, 0, 0);
    commands->setSpacing(8);
    QPushButton *move = makeCommandButton("이동", "yellowCommand");
    QPushButton *manual = makeCommandButton("수동 제어", "controlCommand");
    QPushButton *addRobot = makeCommandButton("로봇 투입", "controlCommand");
    QPushButton *estop = makeCommandButton("긴급 정지", "redCommand");
    connect(move, &QPushButton::clicked, this, &MainWindow::sendMove);
    connect(manual, &QPushButton::clicked, this, [this]() {
        appendEvent(UiEvent{m_selectedRobot,
                            2,
                            0,
                            static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL,
                            "수동 제어 모드 선택"});
    });
    connect(addRobot, &QPushButton::clicked, this, [this]() {
        const int available = kMaxRobots - m_robotCount;
        if (available <= 0) {
            QMessageBox::information(this,
                                     "로봇 투입",
                                     QString("최대 %1대까지 투입할 수 있습니다.").arg(kMaxRobots));
            return;
        }

        QDialog countDialog(this);
        countDialog.setWindowTitle("로봇 투입");
        countDialog.setAttribute(Qt::WA_InputMethodEnabled, false);
        countDialog.setFocusPolicy(Qt::NoFocus);

        QVBoxLayout *countLayout = new QVBoxLayout(&countDialog);
        countLayout->setContentsMargins(24, 20, 24, 18);
        countLayout->setSpacing(14);

        QLabel *countLabel = new QLabel("투입할 로봇 수를 선택하세요", &countDialog);
        countLabel->setWordWrap(true);
        countLabel->setFocusPolicy(Qt::NoFocus);
        countLabel->setAttribute(Qt::WA_InputMethodEnabled, false);
        countLabel->setStyleSheet("QLabel { font-size: 16px; font-weight: 700; }");
        countLayout->addWidget(countLabel);

        int selectedAddCount = 1;
        QHBoxLayout *stepperLayout = new QHBoxLayout;
        stepperLayout->setContentsMargins(0, 4, 0, 4);
        stepperLayout->setSpacing(12);

        QPushButton *minusButton = new QPushButton("-", &countDialog);
        QPushButton *plusButton = new QPushButton("+", &countDialog);
        for (QPushButton *button : {minusButton, plusButton}) {
            button->setFocusPolicy(Qt::NoFocus);
            button->setAttribute(Qt::WA_InputMethodEnabled, false);
            button->setMinimumSize(84, 84);
            button->setCursor(Qt::PointingHandCursor);
            button->setStyleSheet("QPushButton { font-size: 34px; font-weight: 800; border-radius: 8px; padding: 0; }");
        }

        QLabel *countValue = new QLabel(&countDialog);
        countValue->setAlignment(Qt::AlignCenter);
        countValue->setMinimumSize(180, 84);
        countValue->setFocusPolicy(Qt::NoFocus);
        countValue->setAttribute(Qt::WA_InputMethodEnabled, false);
        countValue->setStyleSheet("QLabel { font-size: 42px; font-weight: 800; border: 1px solid rgba(255,255,255,70); border-radius: 8px; padding: 8px 18px; }");

        auto updateCountValue = [&]() {
            countValue->setText(QString("%1대").arg(selectedAddCount));
            minusButton->setEnabled(selectedAddCount > 1);
            plusButton->setEnabled(selectedAddCount < available);
        };
        connect(minusButton, &QPushButton::clicked, &countDialog, [&]() {
            selectedAddCount = qMax(1, selectedAddCount - 1);
            updateCountValue();
        });
        connect(plusButton, &QPushButton::clicked, &countDialog, [&]() {
            selectedAddCount = qMin(available, selectedAddCount + 1);
            updateCountValue();
        });
        updateCountValue();

        stepperLayout->addWidget(minusButton);
        stepperLayout->addWidget(countValue, 1);
        stepperLayout->addWidget(plusButton);
        countLayout->addLayout(stepperLayout);

        QDialogButtonBox *countButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                                              Qt::Horizontal,
                                                              &countDialog);
        auto makeDeployDialogIcon = [](bool deploy) {
            constexpr int side = 28;
            QPixmap pix(side, side);
            pix.fill(Qt::transparent);

            QPainter p(&pix);
            p.setRenderHint(QPainter::Antialiasing, true);

            const QColor stroke = deploy ? QColor(138, 210, 255) : QColor(255, 118, 118);
            const QColor glow = deploy ? QColor(72, 176, 255, 65) : QColor(255, 84, 84, 60);
            const QColor fill = deploy ? QColor(25, 73, 108, 120) : QColor(88, 37, 45, 118);
            QPen glowPen(glow, 4.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            QPen linePen(stroke, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

            QRectF body(6.0, 10.0, 12.0, 8.0);
            p.setPen(glowPen);
            p.drawRoundedRect(body, 3.0, 3.0);
            p.setPen(linePen);
            p.setBrush(fill);
            p.drawRoundedRect(body, 3.0, 3.0);
            p.setBrush(stroke);
            p.drawEllipse(QPointF(10.0, 14.0), 1.1, 1.1);
            p.drawEllipse(QPointF(14.5, 14.0), 1.1, 1.1);
            p.setBrush(Qt::NoBrush);
            p.drawLine(QPointF(7.5, 18.0), QPointF(5.5, 21.5));
            p.drawLine(QPointF(16.5, 18.0), QPointF(18.5, 21.5));

            if (deploy) {
                p.drawLine(QPointF(19.0, 14.0), QPointF(24.0, 14.0));
                p.drawLine(QPointF(21.5, 10.8), QPointF(24.2, 14.0));
                p.drawLine(QPointF(21.5, 17.2), QPointF(24.2, 14.0));
            } else {
                p.drawLine(QPointF(19.5, 9.0), QPointF(24.0, 18.8));
                p.drawLine(QPointF(24.0, 9.0), QPointF(19.5, 18.8));
            }

            return QIcon(pix);
        };
        QPushButton *deployButton = countButtons->button(QDialogButtonBox::Ok);
        QPushButton *cancelButton = countButtons->button(QDialogButtonBox::Cancel);
        deployButton->setText("투입");
        deployButton->setIcon(makeDeployDialogIcon(true));
        deployButton->setIconSize(QSize(28, 28));
        deployButton->setFocusPolicy(Qt::NoFocus);
        deployButton->setAttribute(Qt::WA_InputMethodEnabled, false);
        cancelButton->setText("취소");
        cancelButton->setIcon(makeDeployDialogIcon(false));
        cancelButton->setIconSize(QSize(28, 28));
        cancelButton->setFocusPolicy(Qt::NoFocus);
        cancelButton->setAttribute(Qt::WA_InputMethodEnabled, false);
        connect(countButtons, &QDialogButtonBox::accepted, &countDialog, &QDialog::accept);
        connect(countButtons, &QDialogButtonBox::rejected, &countDialog, &QDialog::reject);
        countLayout->addWidget(countButtons);

        QTimer::singleShot(0, &countDialog, [&countDialog]() {
            countDialog.setFocus();
            if (QGuiApplication::inputMethod()) {
                QGuiApplication::inputMethod()->hide();
            }
        });

        if (countDialog.exec() != QDialog::Accepted) {
            return;
        }

        const int addCount = selectedAddCount;
        if (addCount <= 0) {
            return;
        }

        const int previousCount = m_robotCount;
        setRobotCount(m_robotCount + addCount);
        selectRobot(m_robotCount - 1);
        appendEvent(UiEvent{-1,
                            1,
                            0,
                            static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL,
                            QString("로봇 투입: %1대 -> %2대").arg(previousCount).arg(m_robotCount)});
    });
    connect(estop, &QPushButton::clicked, this, &MainWindow::sendEstop);
    for (QPushButton *button : {move, manual, addRobot, estop}) {
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        commands->addWidget(button);
    }
    control->addLayout(commands, 1);
    lower->addWidget(makePanel("운용 제어", controlBody), 5);

    dashboard->addLayout(lower, 3);
    m_contentStack->addWidget(dashboardPage);

    QWidget *robotPage = new QWidget;
    robotPage->setObjectName("robotOverviewPage");
    QVBoxLayout *robotPageLayout = new QVBoxLayout(robotPage);
    robotPageLayout->setContentsMargins(0, 0, 0, 0);
    robotPageLayout->setSpacing(10);

    QLabel *robotTitle = new QLabel("SPOT 로봇 상태");
    robotTitle->setObjectName("robotPageTitle");
    robotPageLayout->addWidget(robotTitle);

    QWidget *robotGridBody = new QWidget;
    m_robotCardGrid = new QGridLayout(robotGridBody);
    m_robotCardGrid->setContentsMargins(0, 0, 0, 0);
    m_robotCardGrid->setSpacing(10);
    m_robotCardGrid->setColumnStretch(0, 1);
    m_robotCardGrid->setColumnStretch(1, 1);
    m_robotCardScroll = new QScrollArea;
    m_robotCardScroll->setObjectName("panelScrollArea");
    m_robotCardScroll->setWidgetResizable(true);
    m_robotCardScroll->setFrameShape(QFrame::NoFrame);
    m_robotCardScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_robotCardScroll->setWidget(robotGridBody);
    syncRobotStatusCards(m_robotCount);
    robotPageLayout->addWidget(m_robotCardScroll, 1);

    QFrame *robotFooter = new QFrame;
    robotFooter->setObjectName("robotFooter");
    QHBoxLayout *footerLayout = new QHBoxLayout(robotFooter);
    footerLayout->setContentsMargins(18, 8, 18, 8);
    footerLayout->setSpacing(22);
    auto addLegendItem = [footerLayout](const QString &text, const QString &color) {
        QWidget *item = new QWidget;
        item->setObjectName("robotLegendItem");
        QHBoxLayout *itemLayout = new QHBoxLayout(item);
        itemLayout->setContentsMargins(0, 0, 0, 0);
        itemLayout->setSpacing(6);
        QFrame *dot = new QFrame;
        dot->setObjectName("robotLegendDot");
        dot->setFixedSize(9, 9);
        dot->setStyleSheet(QString("#robotLegendDot { background:%1; border:0; border-radius:4px; }").arg(color));
        QLabel *label = new QLabel(text);
        label->setObjectName("robotLegendLabel");
        itemLayout->addWidget(dot, 0, Qt::AlignVCenter);
        itemLayout->addWidget(label, 0, Qt::AlignVCenter);
        footerLayout->addWidget(item, 0, Qt::AlignVCenter);
    };
    addLegendItem("정상", "#20e878");
    addLegendItem("이동/작업", "#72b9ff");
    addLegendItem("주의", "#ffd447");
    addLegendItem("경고", "#ff5b57");
    footerLayout->addSpacing(14);
    m_robotTotalLabel = new QLabel(QString("총 로봇: %1").arg(m_robotCount));
    footerLayout->addWidget(m_robotTotalLabel);
    footerLayout->addStretch();
    footerLayout->addWidget(new QLabel("마지막 업데이트: 실시간"));
    footerLayout->addWidget(new QLabel("자동 갱신  ●"));
    robotPageLayout->addWidget(robotFooter);
    m_contentStack->addWidget(robotPage);

    QWidget *logPage = new QWidget;
    logPage->setObjectName("logPage");
    QVBoxLayout *logLayout = new QVBoxLayout(logPage);
    logLayout->setContentsMargins(0, 0, 0, 0);
    logLayout->setSpacing(12);

    QHBoxLayout *logHeader = new QHBoxLayout;
    logHeader->setSpacing(12);
    QVBoxLayout *logTitleBlock = new QVBoxLayout;
    logTitleBlock->setSpacing(0);
    QLabel *logTitle = new QLabel("시스템 로그");
    logTitle->setObjectName("logHeroTitle");
    logTitle->setAlignment(Qt::AlignCenter);
    logTitleBlock->addStretch();
    logTitleBlock->addWidget(logTitle);
    logTitleBlock->addStretch();
    logHeader->addLayout(logTitleBlock, 1);

    auto makeLogMetric = [](const QString &title, const QString &value, const QString &delta,
                            const QString &accent, QLabel **valueOut = nullptr, QWidget *extra = nullptr) {
        QFrame *card = new QFrame;
        card->setObjectName("logMetricCard");
        QVBoxLayout *cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(12, 10, 12, 10);
        cardLayout->setSpacing(5);
        QLabel *titleLabel = new QLabel(title);
        titleLabel->setObjectName("logMetricTitle");
        QLabel *valueLabel = new QLabel(value);
        valueLabel->setObjectName("logMetricValue");
        valueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        if (valueOut) {
            *valueOut = valueLabel;
        }
        QLabel *deltaLabel = new QLabel(delta);
        deltaLabel->setObjectName("logMetricDelta");
        deltaLabel->setStyleSheet(QString("color:%1").arg(accent));
        cardLayout->addWidget(titleLabel);
        cardLayout->addWidget(valueLabel);
        if (extra) {
            cardLayout->addWidget(extra);
        }
        cardLayout->addStretch();
        cardLayout->addWidget(deltaLabel);
        return card;
    };

    logHeader->addWidget(makeLogMetric("전체 로그", "0", "실시간 집계", "#58df78", &m_logTotalValue, new LogTrendWidget), 1);
    logHeader->addWidget(makeLogMetric("활성 경보", "0", "주의 + 위험 로그", "#f6bd32", &m_logActiveValue), 1);
    logHeader->addWidget(makeLogMetric("위험 이벤트", "0", "위험 로그", "#ff453a", &m_logCriticalValue), 1);

    QFrame *severityCard = new QFrame;
    severityCard->setObjectName("logMetricCard");
    QVBoxLayout *severityLayout = new QVBoxLayout(severityCard);
    severityLayout->setContentsMargins(12, 10, 12, 10);
    severityLayout->setSpacing(6);
    QLabel *severityTitle = new QLabel("심각도별 로그");
    severityTitle->setObjectName("logMetricTitle");
    QHBoxLayout *severityBody = new QHBoxLayout;
    severityBody->setContentsMargins(0, 0, 0, 0);
    severityBody->setSpacing(10);
    severityBody->addStretch(1);
    m_logSeverityChart = new SeverityDonutWidget;
    severityBody->addWidget(m_logSeverityChart);
    m_logSeverityRows = new QLabel;
    m_logSeverityRows->setObjectName("logSeverityRows");
    m_logSeverityRows->setTextFormat(Qt::RichText);
    severityBody->addWidget(m_logSeverityRows);
    severityBody->addStretch(1);
    severityLayout->addWidget(severityTitle);
    severityLayout->addLayout(severityBody, 1);
    logHeader->addWidget(severityCard, 1);
    logLayout->addLayout(logHeader);

    QHBoxLayout *logToolbar = new QHBoxLayout;
    logToolbar->setSpacing(10);
    m_logSearch = new QLineEdit;
    m_logSearch->setObjectName("logSearch");
    m_logSearch->setPlaceholderText("로그 검색...");
    const QDate logEndDefault = QDate::currentDate();
    const QDate logStartDefault = qMin(QDate(2025, 5, 18), logEndDefault);
    m_logStartDate = new QDateEdit(logStartDefault);
    m_logStartDate->setObjectName("logDateEdit");
    m_logStartDate->setCalendarPopup(true);
    m_logStartDate->setDisplayFormat("yyyy-MM-dd");
    m_logEndDate = new QDateEdit(logEndDefault);
    m_logEndDate->setObjectName("logDateEdit");
    m_logEndDate->setCalendarPopup(true);
    m_logEndDate->setDisplayFormat("yyyy-MM-dd");
    QLabel *logStartLabel = new QLabel("시작");
    logStartLabel->setObjectName("logDateLabel");
    QLabel *logEndLabel = new QLabel("종료");
    logEndLabel->setObjectName("logDateLabel");
    QPushButton *filterAll = new QPushButton("전체");
    filterAll->setObjectName("logFilterActive");
    filterAll->setProperty("severityCode", QString());
    QPushButton *filterInfo = new QPushButton("정보");
    filterInfo->setObjectName("logFilterButton");
    filterInfo->setProperty("severityCode", "INFO");
    QPushButton *filterWarn = new QPushButton("주의");
    filterWarn->setObjectName("logFilterButton");
    filterWarn->setProperty("severityCode", "WARN");
    QPushButton *filterCritical = new QPushButton("위험");
    filterCritical->setObjectName("logFilterButton");
    filterCritical->setProperty("severityCode", "CRITICAL");
    m_logFilterButtons = {filterAll, filterInfo, filterWarn, filterCritical};
    connect(m_logSearch, &QLineEdit::textChanged, this, [this]() {
        m_logCurrentPage = 0;
        applyLogFilters();
    });
    connect(m_logStartDate, &QDateEdit::dateChanged, this, [this](const QDate &date) {
        if (m_logEndDate && date > m_logEndDate->date()) {
            m_logEndDate->setDate(date);
        }
        m_logCurrentPage = 0;
        applyLogFilters();
    });
    connect(m_logEndDate, &QDateEdit::dateChanged, this, [this](const QDate &date) {
        if (m_logStartDate && date < m_logStartDate->date()) {
            m_logStartDate->setDate(date);
        }
        m_logCurrentPage = 0;
        applyLogFilters();
    });
    for (QPushButton *button : m_logFilterButtons) {
        connect(button, &QPushButton::clicked, this, [this, button]() {
            setLogSeverityFilter(button->property("severityCode").toString());
        });
    }
    logToolbar->addWidget(m_logSearch, 3);
    logToolbar->addWidget(logStartLabel);
    logToolbar->addWidget(m_logStartDate, 1);
    logToolbar->addWidget(logEndLabel);
    logToolbar->addWidget(m_logEndDate, 1);
    logToolbar->addStretch();
    logToolbar->addWidget(filterAll);
    logToolbar->addWidget(filterInfo);
    logToolbar->addWidget(filterWarn);
    logToolbar->addWidget(filterCritical);
    logLayout->addLayout(logToolbar);

    m_logTable = new QTableWidget;
    m_logTable->setObjectName("logTable");
    m_logTable->setColumnCount(6);
    m_logTable->setHorizontalHeaderLabels({"", "심각도", "시간", "출처", "분류", "메시지"});
    m_logTable->verticalHeader()->hide();
    m_logTable->setShowGrid(false);
    m_logTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_logTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_logTable->setFocusPolicy(Qt::NoFocus);
    m_logTable->setAlternatingRowColors(false);
    m_logTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    m_logTable->horizontalHeader()->resizeSection(0, 42);
    m_logTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    m_logTable->horizontalHeader()->resizeSection(1, 126);
    m_logTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
    m_logTable->horizontalHeader()->resizeSection(2, 220);
    m_logTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    m_logTable->horizontalHeader()->resizeSection(3, 140);
    m_logTable->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
    m_logTable->horizontalHeader()->resizeSection(4, 150);
    m_logTable->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
    m_logTable->verticalHeader()->setDefaultSectionSize(48);

    appendLogTableRow("INFO", "2025-05-25 14:32:18.452", "GNSS-01", "항법", "GPS 고정 완료 (위성 12개)");
    appendLogTableRow("INFO", "2025-05-25 14:32:17.991", "NET-01", "연결", "LTE 재연결 성공 (RSSI -78 dBm)");
    appendLogTableRow("WARN", "2025-05-25 14:31:52.311", "COLL-01", "안전", "충돌 경고: 2.1 m 전방 물체 감지");
    appendLogTableRow("CRITICAL", "2025-05-25 14:31:51.884", "SRS-01", "안전", "운용자 긴급 정지 명령 수신");
    appendLogTableRow("WARN", "2025-05-25 14:31:45.203", "PWR-01", "전원", "배터리 전압 낮음: 11.8 V");
    appendLogTableRow("WARN", "2025-05-25 14:31:32.774", "DRIVE-01", "구동계", "좌측 후륜 미끄러짐 감지");
    appendLogTableRow("INFO", "2025-05-25 14:31:15.661", "ODOM-01", "위치추정", "오도메트리 보정 적용");
    appendLogTableRow("INFO", "2025-05-25 14:30:44.719", "SENS-01", "인지", "센서 융합 상태 정상");
    appendLogTableRow("INFO", "2025-05-25 14:30:33.508", "LOG-01", "시스템", "로그 업로드 완료 (2.1 MB)");
    appendLogTableRow("INFO", "2025-05-25 14:30:12.247", "DIAG-01", "진단", "시스템 진단 통과");
    updateLogSummary();
    logLayout->addWidget(m_logTable, 1);

    QFrame *logFooter = new QFrame;
    logFooter->setObjectName("logFooter");
    QHBoxLayout *logFooterLayout = new QHBoxLayout(logFooter);
    logFooterLayout->setContentsMargins(14, 6, 14, 6);
    logFooterLayout->addWidget(new QLabel("페이지당 행 수: 25"));
    logFooterLayout->addStretch();
    m_logFooterRange = new QLabel;
    logFooterLayout->addWidget(m_logFooterRange);
    logFooterLayout->addStretch();
    m_logFirstPageButton = new QPushButton("|<");
    m_logPrevPageButton = new QPushButton("<");
    m_logNextPageButton = new QPushButton(">");
    m_logLastPageButton = new QPushButton(">|");
    for (QPushButton *button : {m_logFirstPageButton, m_logPrevPageButton, m_logNextPageButton, m_logLastPageButton}) {
        button->setObjectName("logPageButton");
        button->setMinimumWidth(34);
    }
    connect(m_logFirstPageButton, &QPushButton::clicked, this, [this]() { setLogPage(0); });
    connect(m_logPrevPageButton, &QPushButton::clicked, this, [this]() { setLogPage(m_logCurrentPage - 1); });
    connect(m_logNextPageButton, &QPushButton::clicked, this, [this]() { setLogPage(m_logCurrentPage + 1); });
    connect(m_logLastPageButton, &QPushButton::clicked, this, [this]() {
        int filteredCount = 0;
        for (int row = 0; row < m_logTable->rowCount(); ++row) {
            if (logRowMatchesFilters(row)) {
                ++filteredCount;
            }
        }
        setLogPage(qMax(0, (filteredCount - 1) / m_logRowsPerPage));
    });
    logFooterLayout->addWidget(m_logFirstPageButton);
    logFooterLayout->addWidget(m_logPrevPageButton);
    for (int i = 0; i < 5; ++i) {
        QPushButton *pageButton = new QPushButton(QString::number(i + 1));
        pageButton->setObjectName(i == 0 ? "logPageActive" : "logPageButton");
        pageButton->setMinimumWidth(34);
        pageButton->setProperty("pageIndex", i);
        m_logPageButtons.append(pageButton);
        connect(pageButton, &QPushButton::clicked, this, [this, pageButton]() {
            setLogPage(pageButton->property("pageIndex").toInt());
        });
        logFooterLayout->addWidget(pageButton);
    }
    logFooterLayout->addWidget(m_logNextPageButton);
    logFooterLayout->addWidget(m_logLastPageButton);
    logLayout->addWidget(logFooter);
    updateLogSummary();
    m_contentStack->addWidget(logPage);

    QWidget *settingsPage = new QWidget;
    QVBoxLayout *settingsLayout = new QVBoxLayout(settingsPage);
    settingsLayout->setContentsMargins(0, 0, 0, 0);
    QWidget *settingsBody = new QWidget;
    QFormLayout *form = new QFormLayout(settingsBody);
    form->setContentsMargins(8, 4, 8, 4);
    QSpinBox *maxRobots = new QSpinBox;
    maxRobots->setRange(1, kMaxRobots);
    maxRobots->setValue(m_robotCount);
    QLineEdit *cmdIpc = new QLineEdit("SHM cmd_queue (/robot_bridge_N)");
    cmdIpc->setReadOnly(true);
    QLineEdit *shmPattern = new QLineEdit("/robot_bridge_%d");
    shmPattern->setReadOnly(true);
    QPushButton *apply = new QPushButton("공유메모리 재스캔");
    connect(apply, &QPushButton::clicked, this, [this, maxRobots] {
        setRobotCount(maxRobots->value());
        showPage(0);
    });
    form->addRow("감시 로봇 수", maxRobots);
    form->addRow("명령 IPC", cmdIpc);
    form->addRow("SHM 이름", shmPattern);
    form->addRow("", apply);
    settingsLayout->addWidget(makePanel("설정", settingsBody));
    m_contentStack->addWidget(settingsPage);

    m_mapFullscreenPage = new QWidget;
    QVBoxLayout *mapFullscreenLayout = new QVBoxLayout(m_mapFullscreenPage);
    mapFullscreenLayout->setContentsMargins(0, 0, 0, 0);
    mapFullscreenLayout->setSpacing(10);
    QHBoxLayout *mapFullscreenHeader = new QHBoxLayout;
    QLabel *mapFullscreenTitle = new QLabel("탐색 지도");
    mapFullscreenTitle->setObjectName("panelTitle");
    m_fullscreenBtn2d = new QPushButton("2D");
    m_fullscreenBtn2d->setObjectName("smallActive");
    m_fullscreenBtn3d = new QPushButton("3D");
    m_fullscreenBtn3d->setObjectName("smallButton");
    QPushButton *mapManualControlButton = new QPushButton("수동 제어");
    mapManualControlButton->setObjectName("manualControlButton");
    mapManualControlButton->setMinimumSize(168, 42);
    connect(mapManualControlButton, &QPushButton::clicked, this, [this] {
        appendEvent(UiEvent{m_selectedRobot,
                            2,
                            0,
                            static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL,
                            "수동 제어 모드 선택"});
    });
    QPushButton *mapBackButton = new QPushButton("뒤로가기");
    mapBackButton->setObjectName("backButton");
    mapBackButton->setMinimumWidth(104);
    connect(mapBackButton, &QPushButton::clicked, this, &MainWindow::leaveMapFullscreen);
    mapFullscreenHeader->addWidget(mapFullscreenTitle);
    mapFullscreenHeader->addStretch();
    mapFullscreenHeader->addWidget(mapManualControlButton);
    mapFullscreenHeader->addStretch();
    mapFullscreenHeader->addWidget(m_fullscreenBtn2d);
    mapFullscreenHeader->addWidget(m_fullscreenBtn3d);
    mapFullscreenHeader->addSpacing(8);
    mapFullscreenHeader->addWidget(mapBackButton);
    mapFullscreenLayout->addLayout(mapFullscreenHeader);
    m_mapFullscreenContentLayout = new QVBoxLayout;
    m_mapFullscreenContentLayout->setContentsMargins(0, 0, 0, 0);
    m_mapFullscreenContentLayout->setSpacing(0);
    mapFullscreenLayout->addLayout(m_mapFullscreenContentLayout, 1);
    m_contentStack->addWidget(m_mapFullscreenPage);

    m_cameraPage = new QWidget;
    QVBoxLayout *cameraPageLayout = new QVBoxLayout(m_cameraPage);
    cameraPageLayout->setContentsMargins(0, 0, 0, 0);
    cameraPageLayout->setSpacing(10);
    QHBoxLayout *cameraHeader = new QHBoxLayout;
    QPushButton *backButton = new QPushButton("뒤로가기");
    backButton->setObjectName("backButton");
    backButton->setMinimumWidth(104);
    connect(backButton, &QPushButton::clicked, this, &MainWindow::leaveCameraFullscreen);
    QPushButton *manualControlButton = new QPushButton("수동 제어");
    manualControlButton->setObjectName("manualControlButton");
    manualControlButton->setMinimumSize(168, 42);
    connect(manualControlButton, &QPushButton::clicked, this, [this] {
        appendEvent(UiEvent{m_expandedRobotId,
                            2,
                            0,
                            static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL,
                            "수동 제어 모드 선택"});
    });
    m_cameraTitle = new QLabel("카메라 스트리밍");
    m_cameraTitle->setObjectName("panelTitle");
    cameraHeader->addWidget(m_cameraTitle);
    cameraHeader->addStretch();
    cameraHeader->addWidget(manualControlButton);
    cameraHeader->addStretch();
    cameraHeader->addWidget(backButton);
    cameraPageLayout->addLayout(cameraHeader);
    m_expandedVideoTile = new VideoTile(0);
    m_expandedVideoTile->setCursor(Qt::ArrowCursor);
    cameraPageLayout->addWidget(m_expandedVideoTile, 1);
    m_contentStack->addWidget(m_cameraPage);

    connect(navExplore, &QPushButton::clicked, this, [this] { showPage(0); });
    connect(navRobots, &QPushButton::clicked, this, [this] { showPage(1); });
    connect(navLogs, &QPushButton::clicked, this, [this] { showPage(2); });
    connect(navSettings, &QPushButton::clicked, this, [this] { showPage(3); });
    connect(m_btn2d, &QPushButton::clicked, this, &MainWindow::showMap2D);
    connect(m_btn3d, &QPushButton::clicked, this, &MainWindow::showMap3D);
    connect(m_fullscreenBtn2d, &QPushButton::clicked, this, &MainWindow::showMap2D);
    connect(m_fullscreenBtn3d, &QPushButton::clicked, this, &MainWindow::showMap3D);
    connect(mapMaximize, &QPushButton::clicked, this, &MainWindow::showMapFullscreen);
    QShortcut *victimTestShortcut = new QShortcut(QKeySequence("Ctrl+Shift+V"), this);
    connect(victimTestShortcut, &QShortcut::activated, this, [this] {
        appendEvent(UiEvent{m_selectedRobot,
                            4,
                            kEventTypeVictimDetected,
                            static_cast<quint64>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL,
                            "생존자 탐지 테스트"});
    });
    selectRobot(kAllRobotsSelection);
}

void MainWindow::applyStyle()
{
    setStyleSheet(R"(
        QMainWindow, QWidget { background:#03080d; color:#dce7f3; }
        #nav {
            background:#07131d;
            border:1px solid #263c4e;
            border-radius:22px;
        }
        #navSeparator {
            background:rgba(210, 222, 235, 64);
            border:0;
            min-height:1px;
            max-height:1px;
        }
        #topBar, #panel, #metric {
            background:#060d14;
            border:1px solid #1f3544;
            border-radius:8px;
        }
        #title { font-size:26px; font-weight:800; color:#ffffff; }
        #accentText { font-size:18px; font-weight:700; color:#ffd21a; }
        #systemText { font-size:15px; font-weight:700; color:#00e66b; }
        #clockText { font-size:18px; color:#c7d0db; }
        #panelTitle { font-size:20px; font-weight:800; color:#ffffff; background:transparent; border:0; }
        QScrollArea#panelScrollArea {
            background:transparent;
            border:0;
        }
        QScrollArea#panelScrollArea > QWidget > QWidget {
            background:transparent;
        }
        QScrollBar:vertical {
            background:#06111a;
            width:8px;
            margin:0;
            border:0;
        }
        QScrollBar::handle:vertical {
            background:#2f4b5e;
            min-height:24px;
            border-radius:4px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height:0;
            border:0;
        }
        #metric { padding:10px 14px; font-size:15px; line-height:1.35; }
        #metric b { font-size:24px; }
        #headerMetric {
            background:qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                        stop:0 #071017, stop:0.58 #06121b, stop:1 #02070c);
            border:1px solid #0089c9;
            border-radius:7px;
        }
        #headerMetricTitle {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:21px;
            font-weight:900;
        }
        #headerMetricValue {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:38px;
            font-weight:900;
        }
        #headerProgressValue {
            color:#18d878;
            background:transparent;
            border:0;
            font-size:38px;
            font-weight:900;
        }
        QProgressBar#headerProgressBar {
            background:#26333a;
            border:0;
            border-radius:7px;
            min-height:14px;
            max-height:14px;
            text-align:center;
            color:transparent;
        }
        QProgressBar#headerProgressBar::chunk {
            background:#18d878;
            border-radius:7px;
        }
        #videoTile, #statusRow {
            background:#0b141c;
            border:1px solid #263947;
            border-radius:7px;
        }
        #statusHeader {
            background:#121d27;
            border:0;
            border-radius:5px;
            min-height:28px;
            max-height:28px;
        }
        #statusHeaderLabel {
            background:transparent;
            border:0;
            color:#aeb8c8;
            font-size:12px;
            font-weight:800;
        }
        #statusRow {
            background:#071017;
            border:0;
            border-bottom:1px solid #1d3140;
            border-radius:0;
        }
        #statusRobotName {
            background:transparent;
            border:0;
            color:#dce7f3;
            font-size:13px;
            font-weight:800;
        }
        #statusRobotName[state="offline"] { color:#76828e; }
        #statusRobotName[state="warning"] { color:#ffd166; }
        #statusRobotName[state="danger"] { color:#ff736b; }
        #statusBatteryCell {
            background:transparent;
            border:0;
        }
        #statusBatteryText {
            background:transparent;
            border:0;
            color:#16d968;
            font-size:12px;
            font-weight:900;
        }
        #statusBatteryText[state="offline"] { color:#6f7a84; }
        #statusBatteryText[state="warning"] { color:#ffd166; }
        #statusBatteryText[state="danger"] { color:#ff736b; }
        QProgressBar#statusBatteryBar {
            background:#22333d;
            border:0;
            border-radius:5px;
            min-height:10px;
            max-height:10px;
        }
        QProgressBar#statusBatteryBar::chunk {
            background:#16d968;
            border-radius:5px;
        }
        QProgressBar#statusBatteryBar[state="moving"]::chunk { background:#16d968; }
        QProgressBar#statusBatteryBar[state="warning"]::chunk { background:#ffd166; }
        QProgressBar#statusBatteryBar[state="danger"]::chunk { background:#ff736b; }
        QProgressBar#statusBatteryBar[state="offline"]::chunk { background:#55616b; }
        #statusMissionChip {
            background:#10261a;
            border:1px solid #1f7b42;
            border-radius:5px;
            color:#15e36d;
            font-size:12px;
            font-weight:900;
            padding:2px 8px;
        }
        #statusMissionChip[state="moving"] {
            background:#10261a;
            border-color:#1f7b42;
            color:#15e36d;
        }
        #statusMissionChip[state="danger"] {
            background:#321515;
            border-color:#b24a44;
            color:#ff938c;
        }
        #statusMissionChip[state="warning"] {
            background:#2b2410;
            border-color:#c9952d;
            color:#ffd166;
        }
        #statusMissionChip[state="offline"] {
            background:#101820;
            border-color:#384956;
            color:#aeb8c8;
        }
        #videoImage { background:#020507; color:#61707f; font-size:18px; font-weight:800; border-radius:7px; }
        #videoTitle { color:#ffffff; font-size:17px; font-weight:800; background:rgba(0,0,0,130); padding:2px 6px; }
        #videoBadge { color:#00e66b; font-size:11px; font-weight:800; background:rgba(0,0,0,150); padding:2px 6px; }
        #videoBadge[state="wait"] { color:#9aa7b2; }
        #videoBadge[state="nosignal"] { color:#ff453a; }
        #victimAlert {
            background:rgba(176, 0, 18, 185);
            border:3px solid #ff3b30;
            border-radius:10px;
            color:#ffffff;
            font-weight:900;
            padding:0;
        }
        #victimAlert:hover {
            background:rgba(208, 0, 24, 205);
            border-color:#ffdedb;
        }
        #victimAlertTitle {
            background:transparent;
            border:0;
            color:#ffffff;
            font-size:78px;
            font-weight:900;
        }
        #victimAlertFooter {
            background:rgba(0,0,0,80);
            border:0;
            min-height:64px;
            max-height:64px;
        }
        #victimAlertSubtitle {
            background:transparent;
            border:0;
            color:#ffffff;
            font-size:28px;
            font-weight:900;
            padding:0;
        }
        QListWidget#eventList {
            background:#071017;
            border:0;
            outline:0;
            font-size:14px;
        }
        QListWidget#packetStatsList {
            background:#071017;
            border:0;
            outline:0;
            font-size:14px;
        }
        #logPage { background:#03080d; }
        #logHeroTitle {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:28px;
            font-weight:900;
        }
        #logMetricCard {
            background:qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                        stop:0 #081321, stop:0.55 #0a1724, stop:1 #07101a);
            border:1px solid #2b4058;
            border-radius:7px;
        }
        #logMetricTitle {
            color:#aeb8c8;
            background:transparent;
            border:0;
            font-size:12px;
            font-weight:700;
        }
        #logMetricValue {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:24px;
            font-weight:900;
        }
        #logMetricDelta, #logSeverityRows {
            background:transparent;
            border:0;
            font-size:11px;
            font-weight:800;
        }
        QLineEdit#logSearch, #logToolbarButton {
            background:#071321;
            border:1px solid #2b4058;
            border-radius:7px;
            color:#dce7f3;
            min-height:38px;
            padding:4px 12px;
            font-size:13px;
            font-weight:700;
        }
        QLabel#logDateLabel {
            background:transparent;
            border:0;
            color:#94a4b8;
            font-size:12px;
            font-weight:800;
        }
        #logFilterButton, #logFilterActive, #logPageButton, #logPageActive {
            min-height:30px;
            min-width:34px;
            padding:3px 10px;
            border-radius:6px;
            font-size:12px;
            font-weight:800;
        }
        #logFilterButton, #logFilterActive { min-width:86px; }
        #logFilterActive, #logPageActive {
            background:#102f5f;
            border:1px solid #2279ff;
            color:#8ec1ff;
        }
        #logFilterButton, #logPageButton {
            background:#071321;
            border:1px solid #26384b;
            color:#c5cfdb;
        }
        #logPageButton:disabled {
            color:#556273;
            border-color:#1b2a3a;
        }
        QDateEdit#logDateEdit {
            background:#071321;
            border:1px solid #2b4058;
            border-radius:7px;
            color:#dce7f3;
            min-height:38px;
            padding:4px 10px;
            font-size:13px;
            font-weight:700;
        }
        QTableWidget#logTable {
            background:#06111c;
            alternate-background-color:#06111c;
            border:1px solid #2b4058;
            border-radius:8px;
            gridline-color:#1d3144;
            color:#dce7f3;
            font-size:14px;
            outline:0;
        }
        QTableWidget#logTable::item {
            border-bottom:1px solid #1b2d3f;
            padding:7px 8px;
        }
        QHeaderView::section {
            background:#081725;
            color:#b9c7d8;
            border:0;
            border-bottom:1px solid #26384b;
            padding:9px 8px;
            font-size:13px;
            font-weight:900;
        }
        #severityInfo, #severityWarn, #severityCritical {
            border-radius:5px;
            min-width:74px;
            max-width:74px;
            min-height:24px;
            max-height:24px;
            font-size:11px;
            font-weight:900;
        }
        #severityInfo {
            background:#10361e;
            border:1px solid #2a7d3b;
            color:#d9ffe4;
        }
        #severityWarn {
            background:#3c2a04;
            border:1px solid #b98000;
            color:#fff4c7;
        }
        #severityCritical {
            background:#3a1012;
            border:1px solid #c94444;
            color:#ffe3e3;
        }
        #logFooter {
            background:#071321;
            border:1px solid #2b4058;
            border-radius:7px;
            color:#aeb8c8;
            font-size:12px;
            font-weight:800;
        }
        QListWidget::item { padding:8px; border-bottom:1px solid #162633; }
        QPushButton {
            background:#0c1720;
            border:1px solid #2b4050;
            border-radius:7px;
            color:#dce7f3;
            min-height:44px;
            font-size:15px;
            font-weight:700;
            padding:6px 12px;
        }
        QComboBox {
            background:#0c1720;
            border:1px solid #2b4050;
            border-radius:7px;
            color:#dce7f3;
            min-height:44px;
            font-size:15px;
            font-weight:700;
            padding:6px 12px;
        }
        QComboBox::drop-down { border:0; width:28px; }
        QComboBox QAbstractItemView {
            background:#071017;
            color:#dce7f3;
            border:0;
            padding:0;
            margin:0;
            outline:0;
            selection-background-color:#1b1805;
            selection-color:#ffd21a;
        }
        QComboBox QAbstractItemView::item {
            background:#071017;
            color:#dce7f3;
            min-height:34px;
            padding:6px 10px;
            margin:0;
            border:0;
        }
        QComboBox QAbstractItemView::item:selected {
            background:#1b1805;
            color:#ffd21a;
        }
        QPushButton:hover { border-color:#ffd21a; }
        #backButton {
            min-height:34px;
            padding:4px 12px;
            background:#0c1720;
            border:1px solid #2b4050;
            border-radius:6px;
            color:#dce7f3;
            font-size:14px;
            font-weight:800;
        }
        #backButton:hover { border-color:#ffd21a; color:#ffd21a; }
        #manualControlButton {
            min-width:168px;
            min-height:42px;
            padding:4px 20px;
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #3a3005, stop:0.52 #211b04, stop:1 #0f160d);
            border:2px solid #ffd21a;
            border-radius:8px;
            color:#ffd21a;
            font-size:19px;
            font-weight:900;
        }
        #manualControlButton:hover {
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #5a4807, stop:0.52 #2b2405, stop:1 #132010);
            border-color:#fff1a6;
            color:#fff1a6;
        }
        #manualControlButton:pressed {
            background:#4a3a03;
            border-color:#ffffff;
        }
        QPushButton:checked, #smallActive {
            background:#1b1805;
            border-color:#ffd21a;
            color:#ffd21a;
        }
        #navButtonActive {
            background:transparent;
            border-color:transparent;
            color:#f2c94c;
            min-height:96px;
            padding:0;
            font-size:12px;
            line-height:16px;
        }
        #navButton {
            background:transparent;
            border-color:transparent;
            min-height:96px;
            color:#a4b0bc;
            padding:0;
            font-size:12px;
            line-height:16px;
        }
        #yellowCommand {
            background:#241e02;
            border-color:#ffd21a;
            color:#ffd21a;
            min-height:150px;
            font-size:24px;
        }
        #yellowCommand:hover {
            background:#3a3105;
            border-color:#fff1a6;
            color:#fff1a6;
        }
        #redCommand {
            background:#2b0507;
            border-color:#ff2222;
            color:#ffffff;
            min-height:150px;
            font-size:22px;
        }
        #redCommand:hover {
            background:#4a080c;
            border-color:#ff6b6b;
            color:#ffffff;
        }
        #controlCommand {
            font-size:17px;
            font-weight:800;
        }
        #smallButton, #smallActive { min-height:30px; min-width:70px; }
        #infoLine { color:#b9c5d0; background:#071017; border:0; padding:6px 2px; font-size:14px; }
        #robotOverviewPage { background:#03080d; }
        #robotPageTitle {
            color:#ffffff;
            font-size:24px;
            font-weight:900;
            background:transparent;
            border:0;
            padding:4px 2px 8px 2px;
        }
        #robotStatusCard {
            background:qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                        stop:0 #08111b, stop:0.55 #050c13, stop:1 #03080d);
            border:1px solid #263646;
            border-radius:8px;
        }
        #robotStatusCard[state="normal"], #robotStatusCard[state="moving"] {
            border-color:#2a3d50;
        }
        #robotStatusCard[state="warning"] { border-color:#9a7625; }
        #robotStatusCard[state="danger"] { border-color:#a94730; }
        #robotStatusCard[state="offline"] {
            background:#060a0f;
            border-color:#303a42;
        }
        #robotVisualPanel {
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #07111a, stop:1 #02070c);
            border:1px solid #29394a;
            border-radius:8px;
        }
        #robotIdLabel {
            color:#7d8794;
            background:transparent;
            border:0;
            font-size:12px;
            font-weight:900;
        }
        #robotAccentLine {
            background:#6bb7ff;
            border:0;
        }
        #robotDetailPanel { background:transparent; border:0; }
        #robotCardName {
            font-size:28px;
            font-weight:900;
            background:transparent;
            letter-spacing:0;
        }
        #robotStatusBadge {
            background:#101b27;
            border:1px solid #314154;
            border-radius:8px;
            color:#d9e5ef;
            font-size:14px;
            font-weight:900;
            padding:5px 16px;
        }
        #robotStatusBadge[state="normal"] {
            border-color:#2f5b3e;
            color:#74e985;
        }
        #robotStatusBadge[state="moving"] {
            border-color:#3270ad;
            color:#72b9ff;
        }
        #robotStatusBadge[state="danger"] {
            border-color:#b53a3f;
            color:#ff736b;
        }
        #robotStatusBadge[state="warning"] {
            border-color:#9a7625;
            color:#ffd166;
        }
        #robotStatusBadge[state="offline"] {
            background:#0a0f14;
            border-color:#303a42;
            color:#747f89;
        }
        #robotBatteryPanel {
            background:qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                        stop:0 #081421, stop:1 #09121b);
            border:1px solid #28394a;
            border-radius:8px;
        }
        #robotSectionIcon {
            background:transparent;
            border:0;
        }
        #robotBatteryIconBody {
            background:#0f2030;
            border:2px solid #70b9ff;
            border-radius:3px;
        }
        #robotBatteryIconNub {
            background:#70b9ff;
            border:0;
            border-radius:2px;
        }
        #robotBatteryTitle {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:18px;
            font-weight:900;
        }
        #robotBatteryValue {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:18px;
            font-weight:900;
            min-width:42px;
        }
        QProgressBar#robotBatteryBar {
            background:#142131;
            border:0;
            border-radius:3px;
            min-height:20px;
            max-height:20px;
        }
        QProgressBar#robotBatteryBar::chunk {
            background:#72b9ff;
            border-radius:3px;
        }
        QProgressBar#robotBatteryBar[state="moving"]::chunk { background:#72b9ff; }
        QProgressBar#robotBatteryBar[state="warning"]::chunk { background:#ffd166; }
        QProgressBar#robotBatteryBar[state="danger"]::chunk { background:#ff736b; }
        QProgressBar#robotBatteryBar[state="offline"]::chunk { background:#555f68; }
        #robotInfoCell {
            background:#08111a;
            border:1px solid #253645;
            border-radius:8px;
        }
        #robotInfoTitle {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:16px;
            font-weight:900;
        }
        #robotInfoIcon {
            background:transparent;
            border:0;
        }
        #robotRowDot {
            color:#68b4ff;
            background:transparent;
            border:0;
            font-size:12px;
            font-weight:900;
        }
        #robotRowLabel, #robotPartLabel {
            color:#f0f4f8;
            background:transparent;
            border:0;
            font-size:13px;
            font-weight:900;
        }
        #robotCardValue {
            color:#f3f7fc;
            background:transparent;
            border:0;
            font-size:15px;
            font-weight:900;
        }
        #robotCardSubValue {
            color:#f3f7fc;
            background:transparent;
            border:0;
            font-size:14px;
            font-weight:900;
            min-width:64px;
        }
        #robotMiniValue {
            color:#f3f7fc;
            background:transparent;
            border:0;
            font-size:14px;
            font-weight:900;
            min-width:58px;
        }
        #robotPartsPanel {
            background:qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                        stop:0 #07111c, stop:0.52 #081625, stop:1 #07101b);
            border:1px solid #253645;
            border-radius:8px;
        }
        #robotPartsTitleBlock {
            background:transparent;
            border:0;
        }
        #robotPartsIcon {
            background:rgba(30, 104, 180, 34);
            border:1px solid rgba(102, 179, 255, 72);
            border-radius:17px;
        }
        #robotPartsTitle {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:16px;
            font-weight:900;
            line-height:18px;
        }
        #robotPartsDivider {
            background:#254267;
            border:0;
        }
        #robotPartCard {
            background:#071321;
            border:1px solid #25456a;
            border-radius:6px;
        }
        #robotPartCard[state="normal"] {
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #0d2c20, stop:1 #081b17);
            border:1px solid #28c76f;
        }
        #robotPartCard[state="danger"] {
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #421b20, stop:1 #241015);
            border:1px solid #ff5b57;
        }
        #robotPartCard[state="offline"] {
            background:qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #101c26, stop:1 #071017);
            border:1px solid #3f566a;
        }
        #robotPartCardLabel {
            color:#ffffff;
            background:transparent;
            border:0;
            font-size:13px;
            font-weight:1000;
            qproperty-alignment: AlignCenter;
        }
        QLabel#robotStatusChip, QLabel#robotStatusChipMoving, QLabel#robotStatusChipDanger, QLabel#robotStatusChipOffline {
            border-radius:4px;
            min-height:26px;
            max-height:30px;
            min-width:66px;
            font-size:13px;
            font-weight:1000;
            padding:0 2px;
            qproperty-alignment: AlignCenter;
        }
        QLabel#robotStatusChip, QLabel#robotStatusChipMoving {
            background:rgba(12, 64, 38, 90);
            border:1px solid rgba(85, 255, 160, 120);
            color:#d8ffe5;
        }
        QLabel#robotStatusChipDanger {
            background:rgba(74, 18, 26, 95);
            border:1px solid rgba(255, 122, 126, 145);
            color:#ffd6d8;
        }
        QLabel#robotStatusChipOffline {
            background:transparent;
            border:0;
            color:#ffffff;
            padding:0 4px;
            qproperty-alignment: AlignCenter;
        }
        #robotFooter {
            background:#071017;
            border:1px solid #152838;
            border-radius:7px;
            color:#c8d2dd;
            font-size:14px;
            font-weight:800;
        }
        #robotLegendItem {
            background:transparent;
            border:0;
        }
        #robotLegendLabel {
            background:transparent;
            border:0;
            color:#c8d2dd;
            font-size:14px;
            font-weight:800;
        }
    )");
}

QPushButton *MainWindow::makeNavButton(const QString &text, bool active)
{
    QPushButton *button = new SideNavButton(text);
    button->setObjectName(active ? "navButtonActive" : "navButton");
    button->setMinimumWidth(88);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return button;
}

QPushButton *MainWindow::makeCommandButton(const QString &text, const QString &objectName)
{
    QPushButton *button = new QPushButton(text);
    if (!objectName.isEmpty()) {
        button->setObjectName(objectName);
    }
    button->setMinimumHeight(150);
    return button;
}

QFrame *MainWindow::makeHeaderMetricCard(const QString &title, QLabel **valueLabel,
                                         const QString &accentColor,
                                         QProgressBar **progressBar)
{
    QFrame *card = new QFrame;
    card->setObjectName("headerMetric");
    card->setMinimumHeight(112);
    QVBoxLayout *layout = new QVBoxLayout(card);
    layout->setContentsMargins(24, 13, 24, 12);
    layout->setSpacing(progressBar ? 4 : 6);

    QLabel *titleLabel = new QLabel(title);
    titleLabel->setObjectName("headerMetricTitle");
    QLabel *value = new QLabel(progressBar ? "0%" : "0");
    value->setObjectName(progressBar ? "headerProgressValue" : "headerMetricValue");
    value->setStyleSheet(QString("color:%1").arg(accentColor));
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    *valueLabel = value;

    if (progressBar) {
        QHBoxLayout *headerRow = new QHBoxLayout;
        headerRow->setContentsMargins(0, 0, 0, 0);
        headerRow->setSpacing(10);
        headerRow->addWidget(titleLabel);
        headerRow->addStretch(1);
        headerRow->addWidget(value);
        layout->addLayout(headerRow);
        layout->addStretch(1);

        QProgressBar *bar = new QProgressBar;
        bar->setObjectName("headerProgressBar");
        bar->setRange(0, 100);
        bar->setValue(0);
        bar->setTextVisible(false);
        *progressBar = bar;
        layout->addSpacing(2);
        layout->addWidget(bar);
    } else {
        layout->addWidget(titleLabel);
        layout->addWidget(value);
        QFrame *line = new QFrame;
        line->setFrameShape(QFrame::HLine);
        line->setObjectName("headerMetricLine");
        line->setStyleSheet("background:#24313a; border:0; max-height:1px;");
        layout->addStretch(1);
        layout->addWidget(line);
    }
    return card;
}

QString MainWindow::formatMissionTime() const
{
    if (!m_missionTimerActive || !m_missionStartedAt.isValid()) {
        return "00:00";
    }
    const qint64 seconds = m_missionStartedAt.secsTo(QDateTime::currentDateTime());
    return QString("%1:%2")
        .arg(seconds / 60, 2, 10, QLatin1Char('0'))
        .arg(seconds % 60, 2, 10, QLatin1Char('0'));
}

void MainWindow::startDemoMode(int count)
{
    m_robotCount = qBound(1, count, kMaxRobots);
    m_demoTimer = new QTimer(this);
    connect(m_demoTimer, &QTimer::timeout, this, [this]() {
        updateSnapshots(makeDemoSnapshots(m_robotCount));
    });
    updateSnapshots(makeDemoSnapshots(m_robotCount));
    appendEvent(UiEvent{-1, 1, 0, 0,
                        QString("UI 테스트 모드 시작: %1대").arg(m_robotCount)});
    m_demoTimer->start(1000);
}

QVector<RobotSnapshot> MainWindow::makeDemoSnapshots(int count) const
{
    QVector<RobotSnapshot> snapshots;
    const int clamped = qBound(1, count, kMaxRobots);
    snapshots.reserve(clamped);

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const float phase = static_cast<float>((nowMs / 100) % 360) * 0.017453292f;
    const QVector<QColor> colors = {
        QColor("#18d878"), QColor("#00d4ff"), QColor("#ffd21a"), QColor("#ff5b57"),
        QColor("#a78bfa"), QColor("#f97316"), QColor("#14b8a6"), QColor("#e879f9"),
        QColor("#84cc16"), QColor("#60a5fa")
    };

    for (int i = 0; i < clamped; ++i) {
        RobotSnapshot snapshot;
        snapshot.id = i;
        snapshot.shmOpen = true;
        snapshot.connected = true;
        snapshot.mode = i % 3;
        snapshot.faultLevel = (i == 2) ? 4 : 0;
        snapshot.faultCode = (i == 2) ? 0x10u : 0u;
        snapshot.faultText = (i == 2) ? QStringLiteral("drive fault") : QString();
        snapshot.x = 2.0f + static_cast<float>(i % 5) * 1.7f + std::sin(phase + i) * 0.12f;
        snapshot.y = 1.1f + static_cast<float>(i / 5) * 2.25f + std::cos(phase + i) * 0.12f;
        snapshot.theta = phase + static_cast<float>(i) * 0.25f;
        snapshot.vx = 0.15f + static_cast<float>(i % 4) * 0.08f;
        snapshot.vy = 0.0f;
        snapshot.omega = 0.02f;
        snapshot.battery = static_cast<float>(qMax(18, 96 - i * 7));
        snapshot.linkRttMs = 17.0f + static_cast<float>(i * 2);
        snapshot.imageFps = (i == 4 || i == 9) ? 0.0f : 24.0f;
        snapshot.lidarFps = 12.0f;
        snapshot.dropRate = i == 2 ? 8.0f : 0.2f * static_cast<float>(i);
        snapshot.odomSeq = static_cast<uint32_t>(nowMs / 100 + i);
        snapshot.odomTimestampUs = static_cast<quint64>(nowMs) * 1000ULL;
        snapshot.missionId = 100 + i;
        snapshot.waypointIdx = static_cast<uint32_t>((nowMs / 1000 + i) % 8);
        snapshot.totalWaypoints = 8;
        snapshot.missionProgress = static_cast<float>((i * 9 + (nowMs / 500) % 30) % 100);
        snapshot.pathOk = i != 7;
        snapshot.poseOk = true;
        snapshot.goalReached = snapshot.missionProgress >= 95.0f;
        snapshot.distanceToGoalM = qMax(0.0f, 12.0f - snapshot.missionProgress * 0.11f);
        snapshot.rxPackets = static_cast<int>(nowMs / 100 + i * 11);
        snapshot.txCommands = i;
        snapshot.ackPackets = qMax(0, i - 1);
        snapshot.lastRxUs = static_cast<quint64>(nowMs) * 1000ULL;

        if (i != 4 && i != 9) {
            QImage image(640, 360, QImage::Format_RGB32);
            const QColor accent = colors[i % colors.size()];
            image.fill(QColor("#071017"));
            QPainter painter(&image);
            QLinearGradient bg(0, 0, 640, 360);
            bg.setColorAt(0.0, accent.darker(260));
            bg.setColorAt(1.0, QColor("#03080d"));
            painter.fillRect(image.rect(), bg);
            painter.setPen(QPen(accent, 4));
            painter.drawRect(image.rect().adjusted(18, 18, -18, -18));
            painter.setPen(Qt::white);
            painter.setFont(QFont("Noto Sans", 40, QFont::Black));
            painter.drawText(image.rect(), Qt::AlignCenter, robotName(i));
            painter.setFont(QFont("Noto Sans", 18, QFont::Bold));
            painter.setPen(QColor("#c8d7e4"));
            painter.drawText(QRect(0, 232, 640, 42), Qt::AlignCenter,
                             QString("DEMO FEED  %1 fps").arg(snapshot.imageFps, 0, 'f', 0));
            snapshot.image = image;
        }

        snapshot.globalPath = {
            {snapshot.x, snapshot.y, 0.0f, 0.0f},
            {qMin(snapshot.x + 1.2f, 11.0f), qMin(snapshot.y + 0.8f, 7.0f), 0.0f, 0.0f},
            {qMin(snapshot.x + 2.2f, 11.0f), qMin(snapshot.y + 1.4f, 7.0f), 0.0f, 0.0f}
        };

        snapshots.append(snapshot);
    }

    return snapshots;
}

void MainWindow::setRobotCount(int count)
{
    m_robotCount = qBound(1, count, kMaxRobots);
    syncRobotUi(m_robotCount);
    clampSelectedRobot();

    if (m_victimAlertRobotId >= m_robotCount) {
        m_victimAlertRobotId = -1;
        if (m_victimAlertButton) {
            m_victimAlertButton->hide();
        }
    }
    if (m_expandedRobotId >= m_robotCount) {
        m_expandedRobotId = -1;
        if (m_contentStack && m_cameraPage && m_contentStack->currentWidget() == m_cameraPage) {
            leaveCameraFullscreen();
        }
    }

    if (m_demoMode) {
        updateSnapshots(makeDemoSnapshots(m_robotCount));
        return;
    }

    m_monitor.start(kMaxRobots);
    refreshRobotSelector();
    refreshRobotList();
}

void MainWindow::syncRobotUi(int count)
{
    const int clamped = qBound(1, count, kMaxRobots);
    syncVideoTiles(clamped);
    syncStatusRows(clamped);
    syncRobotStatusCards(clamped);

    if (m_robotSelector) {
        m_robotSelector->setMaxVisibleItems(qMin(clamped + 1, kMaxRobots + 1));
    }
    if (m_robotTotalLabel) {
        m_robotTotalLabel->setText(QString("총 로봇: %1").arg(clamped));
    }
}

void MainWindow::syncVideoTiles(int count)
{
    if (!m_videoGrid) {
        return;
    }

    const bool resetLayout = m_videoPlacements.size() != count;
    while (m_videoTiles.size() > count) {
        VideoTile *tile = m_videoTiles.takeLast();
        m_videoGrid->removeWidget(tile);
        tile->deleteLater();
    }
    while (m_videoTiles.size() < count) {
        const int robotId = m_videoTiles.size();
        VideoTile *tile = new VideoTile(robotId);
        m_videoTiles.append(tile);
        connect(tile, &VideoTile::clicked, this, &MainWindow::showCameraFullscreen);
        connect(tile, &VideoTile::moveRequested, this, &MainWindow::moveVideoTile);
        connect(tile, &VideoTile::resizeRequested, this, &MainWindow::resizeVideoTile);
        connect(tile, &VideoTile::shrinkRequested, this, &MainWindow::shrinkVideoTile);
        if (robotId < m_snapshots.size()) {
            tile->setSnapshot(m_snapshots[robotId]);
        }
    }
    if (resetLayout) {
        resetVideoLayout(count);
    }
    relayoutVideoTiles();
}

void MainWindow::syncStatusRows(int count)
{
    if (!m_robotStatusGrid) {
        return;
    }

    while (m_statusRows.size() > count) {
        StatusRow *row = m_statusRows.takeLast();
        m_robotStatusGrid->removeWidget(row);
        row->deleteLater();
    }
    while (m_statusRows.size() < count) {
        const int robotId = m_statusRows.size();
        StatusRow *row = new StatusRow(robotId);
        m_statusRows.append(row);
        if (robotId < m_snapshots.size()) {
            row->setSnapshot(m_snapshots[robotId]);
        }
    }
    const GridSpec spec = statusGridSpec(count);
    for (StatusRow *row : m_statusRows) {
        m_robotStatusGrid->removeWidget(row);
    }
    for (int row = 0; row < kMaxRobots; ++row) {
        m_robotStatusGrid->setRowStretch(row, 0);
    }
    for (int col = 0; col < kMaxRobots; ++col) {
        m_robotStatusGrid->setColumnStretch(col, 0);
    }
    for (int i = 0; i < m_statusRows.size(); ++i) {
        const int row = i / spec.columns;
        const int col = i % spec.columns;
        m_robotStatusGrid->addWidget(m_statusRows[i], row, col);
        m_robotStatusGrid->setRowStretch(row, count >= 6 ? 0 : 1);
        m_robotStatusGrid->setColumnStretch(col, 1);
    }
    if (QWidget *body = m_robotStatusGrid->parentWidget()) {
        if (count >= 6) {
            body->setMinimumHeight(m_statusRows.size() * 36 + qMax(0, m_statusRows.size() - 1) * m_robotStatusGrid->spacing());
            body->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        } else {
            body->setMinimumHeight(0);
            body->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        }
    }
}

void MainWindow::syncRobotStatusCards(int count)
{
    if (!m_robotCardGrid) {
        return;
    }

    while (m_robotStatusCards.size() > count) {
        RobotStatusCard *card = m_robotStatusCards.takeLast();
        m_robotCardGrid->removeWidget(card);
        card->deleteLater();
    }
    while (m_robotStatusCards.size() < count) {
        const int robotId = m_robotStatusCards.size();
        RobotStatusCard *card = new RobotStatusCard(robotId);
        m_robotStatusCards.append(card);
        if (robotId < m_snapshots.size()) {
            card->setSnapshot(m_snapshots[robotId]);
        }
    }
    relayoutRobotStatusCards();
}

void MainWindow::resetVideoLayout(int count)
{
    const int clamped = qBound(1, count, kMaxRobots);
    const VideoLayoutSpec spec = videoLayoutSpec(clamped);
    m_videoPlacements.clear();
    m_videoPlacements.resize(clamped);

    QVector<int> ordered;
    ordered.reserve(clamped);
    const int primary = featuredVideoRobotForSelection(m_selectedRobot, clamped);
    if (spec.hasFeaturedTile) {
        ordered.append(primary);
    }
    for (int i = 0; i < clamped; ++i) {
        if (!spec.hasFeaturedTile || i != primary) {
            ordered.append(i);
        }
    }

    for (int i = 0; i < ordered.size() && i < spec.placements.size(); ++i) {
        const int robotId = ordered[i];
        const VideoLayoutSlot &slot = spec.placements[i];
        if (robotId < 0 || robotId >= m_videoPlacements.size()) {
            continue;
        }
        const VideoLayoutSlot scaledSlot = scaledVideoSlot(slot);
        m_videoPlacements[robotId] = {scaledSlot.row, scaledSlot.column,
                                      scaledSlot.rowSpan, scaledSlot.columnSpan};
    }
    updateVideoLayoutBodySize();
}

bool MainWindow::videoPlacementAvailable(int robotId, int row, int column,
                                         int rowSpan, int columnSpan) const
{
    if (row < 0 || column < 0 || rowSpan < 1 || columnSpan < 1
        || row + rowSpan > kVideoLayoutRows
        || column + columnSpan > kVideoLayoutColumns) {
        return false;
    }

    const QRect candidate(column, row, columnSpan, rowSpan);
    for (int i = 0; i < m_videoPlacements.size(); ++i) {
        if (i == robotId) {
            continue;
        }
        const VideoPlacement &placement = m_videoPlacements[i];
        const QRect occupied(placement.column, placement.row,
                             placement.columnSpan, placement.rowSpan);
        if (candidate.intersects(occupied)) {
            return false;
        }
    }
    return true;
}

bool MainWindow::setVideoPlacementIfFree(int robotId, int row, int column,
                                         int rowSpan, int columnSpan)
{
    if (robotId < 0 || robotId >= m_videoPlacements.size()) {
        return false;
    }

    if (!videoPlacementAvailable(robotId, row, column, rowSpan, columnSpan)) {
        return false;
    }

    m_videoPlacements[robotId] = {row, column, rowSpan, columnSpan};
    relayoutVideoTiles();
    return true;
}

bool MainWindow::autoArrangeVideoLayout(int fixedRobotId, int row, int column,
                                        int rowSpan, int columnSpan)
{
    if (fixedRobotId < 0 || fixedRobotId >= m_videoPlacements.size()) {
        return false;
    }

    const VideoLayoutSlot fixedSlot{row, column, rowSpan, columnSpan};
    QVector<VideoLayoutSlot> remainingSlots;
    if (!arrangeRemainingVideoSlots(fixedSlot,
                                    m_videoPlacements.size() - 1,
                                    &remainingSlots)) {
        return false;
    }

    QVector<VideoPlacement> nextPlacements;
    nextPlacements.resize(m_videoPlacements.size());
    nextPlacements[fixedRobotId] = {fixedSlot.row,
                                    fixedSlot.column,
                                    fixedSlot.rowSpan,
                                    fixedSlot.columnSpan};

    int slotIndex = 0;
    for (int robotId = 0; robotId < nextPlacements.size(); ++robotId) {
        if (robotId == fixedRobotId) {
            continue;
        }
        if (slotIndex >= remainingSlots.size()) {
            return false;
        }
        const VideoLayoutSlot &slot = remainingSlots[slotIndex++];
        nextPlacements[robotId] = {slot.row, slot.column, slot.rowSpan, slot.columnSpan};
    }

    m_videoPlacements = nextPlacements;
    relayoutVideoTiles();
    return true;
}

void MainWindow::updateVideoLayoutBodySize()
{
    if (!m_videoGrid || !m_videoGrid->parentWidget()) {
        return;
    }

    QWidget *body = m_videoGrid->parentWidget();
    body->setMinimumSize(0, 0);
    body->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
}

QPoint MainWindow::videoGridCellAt(const QPoint &globalPos) const
{
    if (!m_videoGrid || !m_videoGrid->parentWidget()) {
        return QPoint(-1, -1);
    }

    QWidget *body = m_videoGrid->parentWidget();
    const QPoint local = body->mapFromGlobal(globalPos);
    if (local.x() < 0 || local.y() < 0) {
        return QPoint(-1, -1);
    }

    const int column = qBound(0, local.x() * kVideoLayoutColumns / qMax(1, body->width()),
                              kVideoLayoutColumns - 1);
    const int row = qBound(0, local.y() * kVideoLayoutRows / qMax(1, body->height()),
                           kVideoLayoutRows - 1);
    return QPoint(column, row);
}

void MainWindow::moveVideoTile(int robotId, const QPoint &globalPos)
{
    if (robotId < 0 || robotId >= m_videoPlacements.size()) {
        return;
    }

    const QPoint cell = videoGridCellAt(globalPos);
    if (cell.x() < 0 || cell.y() < 0) {
        return;
    }

    const VideoPlacement placement = m_videoPlacements[robotId];
    autoArrangeVideoLayout(robotId, cell.y(), cell.x(),
                           placement.rowSpan, placement.columnSpan);
}

void MainWindow::resizeVideoTile(int robotId, int edgeMask, const QPoint &globalPos)
{
    if (robotId < 0 || robotId >= m_videoPlacements.size() || edgeMask == VideoTile::NoEdge) {
        return;
    }

    const QPoint cell = videoGridCellAt(globalPos);
    if (cell.x() < 0 || cell.y() < 0) {
        return;
    }

    const VideoPlacement placement = m_videoPlacements[robotId];
    int row = placement.row;
    int column = placement.column;
    int rowSpan = placement.rowSpan;
    int columnSpan = placement.columnSpan;
    const int right = placement.column + placement.columnSpan - 1;
    const int bottom = placement.row + placement.rowSpan - 1;

    if (edgeMask & VideoTile::LeftEdge) {
        column = qBound(0, cell.x(), right);
        columnSpan = right - column + 1;
    } else if (edgeMask & VideoTile::RightEdge) {
        const int newRight = qBound(column, cell.x(), kVideoLayoutColumns - 1);
        columnSpan = newRight - column + 1;
    }

    if (edgeMask & VideoTile::TopEdge) {
        row = qBound(0, cell.y(), bottom);
        rowSpan = bottom - row + 1;
    } else if (edgeMask & VideoTile::BottomEdge) {
        const int newBottom = qBound(row, cell.y(), kVideoLayoutRows - 1);
        rowSpan = newBottom - row + 1;
    }

    autoArrangeVideoLayout(robotId, row, column, rowSpan, columnSpan);
}

void MainWindow::shrinkVideoTile(int robotId, int edgeMask)
{
    if (robotId < 0 || robotId >= m_videoPlacements.size() || edgeMask == VideoTile::NoEdge) {
        return;
    }

    VideoPlacement placement = m_videoPlacements[robotId];
    if ((edgeMask & VideoTile::LeftEdge) && placement.columnSpan > 1) {
        ++placement.column;
        --placement.columnSpan;
    } else if ((edgeMask & VideoTile::RightEdge) && placement.columnSpan > 1) {
        --placement.columnSpan;
    }

    if ((edgeMask & VideoTile::TopEdge) && placement.rowSpan > 1) {
        ++placement.row;
        --placement.rowSpan;
    } else if ((edgeMask & VideoTile::BottomEdge) && placement.rowSpan > 1) {
        --placement.rowSpan;
    }

    autoArrangeVideoLayout(robotId, placement.row, placement.column,
                           placement.rowSpan, placement.columnSpan);
}

void MainWindow::relayoutVideoTiles()
{
    if (!m_videoGrid) {
        return;
    }
    if (m_videoPlacements.size() != m_videoTiles.size()) {
        resetVideoLayout(m_videoTiles.size());
    }
    for (VideoTile *tile : m_videoTiles) {
        m_videoGrid->removeWidget(tile);
    }
    for (int row = 0; row < kVideoLayoutRows; ++row) {
        m_videoGrid->setRowStretch(row, 0);
    }
    for (int col = 0; col < kVideoLayoutColumns; ++col) {
        m_videoGrid->setColumnStretch(col, 0);
    }

    for (int i = 0; i < m_videoTiles.size() && i < m_videoPlacements.size(); ++i) {
        const VideoPlacement &placement = m_videoPlacements[i];
        if (!videoPlacementAvailable(i, placement.row, placement.column,
                                     placement.rowSpan, placement.columnSpan)) {
            continue;
        }
        m_videoGrid->addWidget(m_videoTiles[i], placement.row, placement.column,
                               placement.rowSpan, placement.columnSpan);
    }
    for (int row = 0; row < kVideoLayoutRows; ++row) {
        m_videoGrid->setRowStretch(row, 1);
    }
    for (int col = 0; col < kVideoLayoutColumns; ++col) {
        m_videoGrid->setColumnStretch(col, 1);
    }

    updateVideoLayoutBodySize();
}

void MainWindow::relayoutRobotStatusCards()
{
    if (!m_robotCardGrid) {
        return;
    }
    const bool scrollLayout = m_robotStatusCards.size() >= 6;
    for (RobotStatusCard *card : m_robotStatusCards) {
        m_robotCardGrid->removeWidget(card);
    }
    for (int row = 0; row <= kMaxRobots / kRobotCardGridColumns; ++row) {
        m_robotCardGrid->setRowStretch(row, 0);
    }
    for (int i = 0; i < m_robotStatusCards.size(); ++i) {
        m_robotCardGrid->addWidget(m_robotStatusCards[i], i / kRobotCardGridColumns, i % kRobotCardGridColumns);
        m_robotCardGrid->setRowStretch(i / kRobotCardGridColumns, scrollLayout ? 0 : 1);
    }
    if (scrollLayout) {
        m_robotCardGrid->setRowStretch((m_robotStatusCards.size() + kRobotCardGridColumns - 1) / kRobotCardGridColumns, 1);
    }
    if (QWidget *body = m_robotCardGrid->parentWidget()) {
        const int rows = qMax(1, (m_robotStatusCards.size() + kRobotCardGridColumns - 1) / kRobotCardGridColumns);
        if (scrollLayout) {
            body->setMinimumWidth(kRobotCardGridColumns * 620 + (kRobotCardGridColumns - 1) * m_robotCardGrid->spacing());
            body->setMinimumHeight(rows * 350 + (rows - 1) * m_robotCardGrid->spacing());
            body->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        } else {
            body->setMinimumSize(kRobotCardGridColumns * 620 + (kRobotCardGridColumns - 1) * m_robotCardGrid->spacing(),
                                 rows * 350 + (rows - 1) * m_robotCardGrid->spacing());
            body->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        }
    }
}

void MainWindow::clampSelectedRobot()
{
    if (m_selectedRobot == kAllRobotsSelection) {
        if (m_map) {
            m_map->setSelectedRobot(m_selectedRobot);
        }
        return;
    }
    const int upper = qMax(0, m_robotCount - 1);
    const int clamped = qBound(0, m_selectedRobot, upper);
    if (clamped == m_selectedRobot) {
        return;
    }

    m_selectedRobot = clamped;
    if (m_map) {
        m_map->setSelectedRobot(m_selectedRobot);
    }
}

void MainWindow::updateMissionSummary()
{
    const QVector<RobotSnapshot> deployedSnapshots = snapshotsForDeployedRobots(m_snapshots, m_robotCount);
    int connected = 0;
    float progressTotal = 0.0f;
    int progressCount = 0;
    for (const RobotSnapshot &snapshot : deployedSnapshots) {
        if (snapshot.connected) {
            ++connected;
            progressTotal += snapshot.missionProgress > 1.0f
                ? snapshot.missionProgress
                : snapshot.missionProgress * 100.0f;
            ++progressCount;
        }
    }

    if (connected > 0 && !m_missionTimerActive) {
        m_missionStartedAt = QDateTime::currentDateTime();
        m_missionTimerActive = true;
    }

    const int progress = progressCount > 0
        ? qBound(0, qRound(progressTotal / progressCount), 100)
        : 0;
    if (m_metricProgress) {
        m_metricProgress->setText(QString("%1%").arg(progress));
    }
    if (m_metricProgressBar) {
        m_metricProgressBar->setValue(progress);
    }
    if (m_metricMissionTime) {
        m_metricMissionTime->setText(formatMissionTime());
    }
    if (m_metricConnected) {
        m_metricConnected->setText(QString::number(connected));
    }
    if (m_metricEvents) {
        m_metricEvents->setText(QString::number(m_eventList ? m_eventList->count() : 0));
    }
}

void MainWindow::updatePacketLogPanel()
{
    if (!m_packetStatsList) {
        return;
    }
    m_packetStatsList->clear();
    const QVector<RobotSnapshot> deployedSnapshots = snapshotsForDeployedRobots(m_snapshots, m_robotCount);
    int totalRx = 0;
    int totalTx = 0;
    int totalAck = 0;
    int totalDrops = 0;
    for (const RobotSnapshot &snapshot : deployedSnapshots) {
        const int drops = static_cast<int>(snapshot.imgDropCount + snapshot.lidarDropCount);
        totalRx += snapshot.rxPackets;
        totalTx += snapshot.txCommands;
        totalAck += snapshot.ackPackets;
        totalDrops += drops;
        QListWidgetItem *item = new QListWidgetItem(
            QString("%1  RX %2  TX %3  ACK %4  DROP %5")
                .arg(robotName(snapshot.id))
                .arg(snapshot.rxPackets)
                .arg(snapshot.txCommands)
                .arg(snapshot.ackPackets)
                .arg(drops));
        item->setForeground(snapshot.connected ? QColor("#dce7f3") : QColor("#8a96a3"));
        m_packetStatsList->addItem(item);
    }
    QListWidgetItem *total = new QListWidgetItem(
        QString("TOTAL  RX %1  TX %2  ACK %3  DROP %4")
            .arg(totalRx)
            .arg(totalTx)
            .arg(totalAck)
            .arg(totalDrops));
    total->setForeground(QColor("#00d4ff"));
    m_packetStatsList->insertItem(0, total);
}

void MainWindow::updateSnapshots(const QVector<RobotSnapshot> &snapshots)
{
    QVector<RobotSnapshot> displaySnapshots = remapSnapshotsForDisplay(snapshots);
    for (RobotSnapshot &snapshot : displaySnapshots) {
        snapshot.commandMoving = snapshot.connected && m_commandMovingRobotIds.contains(snapshot.id);
    }
    const QVector<RobotSnapshot> deployedSnapshots = snapshotsForDeployedRobots(displaySnapshots, m_robotCount);
    m_snapshots = displaySnapshots;
    int connected = 0;
    for (const RobotSnapshot &snapshot : deployedSnapshots) {
        if (snapshot.connected) {
            connected++;
        }
        if (snapshot.id >= 0 && snapshot.id < m_videoTiles.size()) {
            m_videoTiles[snapshot.id]->setSnapshot(snapshot);
        }
        if (m_expandedVideoTile && snapshot.id == m_expandedRobotId) {
            m_expandedVideoTile->setSnapshot(snapshot);
        }
        if (snapshot.id >= 0 && snapshot.id < m_statusRows.size()) {
            m_statusRows[snapshot.id]->setSnapshot(snapshot);
        }
        if (snapshot.id >= 0 && snapshot.id < m_robotStatusCards.size()) {
            m_robotStatusCards[snapshot.id]->setSnapshot(snapshot);
        }
    }

    updateMissionSummary();
    updatePacketLogPanel();
    m_system->setText(connected > 0 ? "● 시스템 정상" : "● 브릿지 대기");
    m_system->setStyleSheet(connected > 0 ? "color:#00e66b" : "color:#ffd21a");
    refreshRobotSelector();
    m_map->setSnapshots(deployedSnapshots);
    refreshRobotList();
}

void MainWindow::appendLogTableRow(const QString &severityCode,
                                   const QString &timestamp,
                                   const QString &source,
                                   const QString &category,
                                   const QString &message,
                                   bool prepend)
{
    if (!m_logTable) {
        return;
    }

    auto makeCell = [](const QString &text) {
        QTableWidgetItem *item = new QTableWidgetItem(text);
        item->setForeground(QColor("#dce7f3"));
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        return item;
    };

    QLabel *badge = new QLabel(logSeverityText(severityCode));
    badge->setAlignment(Qt::AlignCenter);
    badge->setObjectName(severityCode == "CRITICAL" ? "severityCritical"
                         : (severityCode == "WARN" ? "severityWarn" : "severityInfo"));

    const int row = prepend ? 0 : m_logTable->rowCount();
    m_logTable->insertRow(row);
    m_logTable->setItem(row, 0, makeCell(""));
    QTableWidgetItem *severityItem = makeCell(logSeverityText(severityCode));
    severityItem->setData(Qt::UserRole, severityCode);
    m_logTable->setItem(row, 1, severityItem);
    m_logTable->setCellWidget(row, 1, badge);
    m_logTable->setItem(row, 2, makeCell(timestamp));
    m_logTable->setItem(row, 3, makeCell(source));
    m_logTable->setItem(row, 4, makeCell(category));
    m_logTable->setItem(row, 5, makeCell(message));
    applyLogFilters();
    updateLogSummary();
}

void MainWindow::applyLogFilters()
{
    if (!m_logTable) {
        return;
    }
    int filteredIndex = 0;
    for (int row = 0; row < m_logTable->rowCount(); ++row) {
        const bool matches = logRowMatchesFilters(row);
        const bool inPage = filteredIndex >= m_logCurrentPage * m_logRowsPerPage
            && filteredIndex < (m_logCurrentPage + 1) * m_logRowsPerPage;
        m_logTable->setRowHidden(row, !(matches && inPage));
        if (matches) {
            ++filteredIndex;
        }
    }
    updateLogSummary();
}

void MainWindow::setLogSeverityFilter(const QString &severityCode)
{
    m_logSeverityFilter = severityCode;
    m_logCurrentPage = 0;
    for (QPushButton *button : m_logFilterButtons) {
        const bool active = button->property("severityCode").toString() == severityCode;
        button->setObjectName(active ? "logFilterActive" : "logFilterButton");
        button->style()->unpolish(button);
        button->style()->polish(button);
    }
    applyLogFilters();
    updateLogSummary();
}

void MainWindow::setLogPage(int page)
{
    if (!m_logTable) {
        return;
    }
    int filteredCount = 0;
    for (int row = 0; row < m_logTable->rowCount(); ++row) {
        if (logRowMatchesFilters(row)) {
            ++filteredCount;
        }
    }
    const int maxPage = qMax(0, (filteredCount - 1) / m_logRowsPerPage);
    m_logCurrentPage = qBound(0, page, maxPage);
    applyLogFilters();
}

bool MainWindow::logRowMatchesFilters(int row) const
{
    if (!m_logTable) {
        return false;
    }

    QTableWidgetItem *severityItem = m_logTable->item(row, 1);
    const QString severityCode = severityItem ? severityItem->data(Qt::UserRole).toString() : QString();
    if (!m_logSeverityFilter.isEmpty() && severityCode != m_logSeverityFilter) {
        return false;
    }

    QTableWidgetItem *timeItem = m_logTable->item(row, 2);
    const QDateTime rowTime = timeItem
        ? QDateTime::fromString(timeItem->text(), "yyyy-MM-dd HH:mm:ss.zzz")
        : QDateTime();
    if (rowTime.isValid()) {
        if (m_logStartDate && rowTime.date() < m_logStartDate->date()) {
            return false;
        }
        if (m_logEndDate && rowTime.date() > m_logEndDate->date()) {
            return false;
        }
    }

    const QString query = m_logSearch ? m_logSearch->text().trimmed() : QString();
    if (query.isEmpty()) {
        return true;
    }

    QString rowText;
    for (int col = 1; col < m_logTable->columnCount(); ++col) {
        if (QTableWidgetItem *item = m_logTable->item(row, col)) {
            rowText += item->text() + QLatin1Char(' ');
        }
    }
    return rowText.contains(query, Qt::CaseInsensitive);
}

void MainWindow::updateLogSummary()
{
    if (!m_logTable) {
        return;
    }

    int infoCount = 0;
    int warnCount = 0;
    int criticalCount = 0;
    int filteredCount = 0;
    for (int row = 0; row < m_logTable->rowCount(); ++row) {
        QTableWidgetItem *severityItem = m_logTable->item(row, 1);
        const QString severityCode = severityItem ? severityItem->data(Qt::UserRole).toString() : QString();
        if (severityCode == "CRITICAL") {
            ++criticalCount;
        } else if (severityCode == "WARN") {
            ++warnCount;
        } else {
            ++infoCount;
        }
        if (logRowMatchesFilters(row)) {
            ++filteredCount;
        }
    }

    const int total = infoCount + warnCount + criticalCount;
    const int maxPage = qMax(0, (filteredCount - 1) / m_logRowsPerPage);
    if (m_logCurrentPage > maxPage) {
        m_logCurrentPage = maxPage;
        applyLogFilters();
        return;
    }
    const int active = warnCount + criticalCount;
    if (m_logTotalValue) {
        m_logTotalValue->setText(QString::number(total));
    }
    if (m_logActiveValue) {
        m_logActiveValue->setText(QString::number(active));
    }
    if (m_logCriticalValue) {
        m_logCriticalValue->setText(QString::number(criticalCount));
    }

    auto pct = [total](int count) {
        return total > 0 ? qRound(count * 100.0 / total) : 0;
    };
    if (m_logSeverityRows) {
        m_logSeverityRows->setText(
            QString("<span style='color:#58df78'>정보</span>  %1 (%2%)<br>"
                    "<span style='color:#f6bd32'>주의</span>  %3 (%4%)<br>"
                    "<span style='color:#ff5b57'>위험</span>  %5 (%6%)")
                .arg(infoCount)
                .arg(pct(infoCount))
                .arg(warnCount)
                .arg(pct(warnCount))
                .arg(criticalCount)
                .arg(pct(criticalCount)));
    }
    if (m_logSeverityChart) {
        m_logSeverityChart->setProperty("infoCount", infoCount);
        m_logSeverityChart->setProperty("warnCount", warnCount);
        m_logSeverityChart->setProperty("criticalCount", criticalCount);
        m_logSeverityChart->update();
    }
    if (m_logFooterRange) {
        const int first = filteredCount > 0 ? m_logCurrentPage * m_logRowsPerPage + 1 : 0;
        const int last = filteredCount > 0 ? qMin(filteredCount, (m_logCurrentPage + 1) * m_logRowsPerPage) : 0;
        m_logFooterRange->setText(QString("%1 - %2 / 필터 결과 %3, 전체 %4 로그")
                                      .arg(first)
                                      .arg(last)
                                      .arg(filteredCount)
                                      .arg(total));
    }
    const bool hasPrev = m_logCurrentPage > 0;
    const bool hasNext = m_logCurrentPage < maxPage;
    if (m_logFirstPageButton) {
        m_logFirstPageButton->setEnabled(hasPrev);
    }
    if (m_logPrevPageButton) {
        m_logPrevPageButton->setEnabled(hasPrev);
    }
    if (m_logNextPageButton) {
        m_logNextPageButton->setEnabled(hasNext);
    }
    if (m_logLastPageButton) {
        m_logLastPageButton->setEnabled(hasNext);
    }
    for (int i = 0; i < m_logPageButtons.size(); ++i) {
        QPushButton *button = m_logPageButtons[i];
        const int pageBase = (m_logCurrentPage / qMax(1, m_logPageButtons.size())) * m_logPageButtons.size();
        const int pageIndex = pageBase + i;
        button->setProperty("pageIndex", pageIndex);
        button->setText(QString::number(pageIndex + 1));
        button->setEnabled(pageIndex <= maxPage);
        button->setObjectName(pageIndex == m_logCurrentPage ? "logPageActive" : "logPageButton");
        button->style()->unpolish(button);
        button->style()->polish(button);
    }
}

void MainWindow::appendEvent(const UiEvent &event)
{
    if (isBridgeCommandSentEvent(event) || isReassemblyTimeoutEvent(event) || isCommandAckEvent(event)) {
        return;
    }

    QString severity = "정보";
    QString color = "#8a96a3";
    if (event.severity >= 4) {
        severity = "위험";
        color = "#ff3838";
    } else if (event.severity == 3) {
        severity = "경고";
        color = "#ff3838";
    } else if (event.severity == 2) {
        severity = "주의";
        color = "#ffd21a";
    }

    const QString source = event.robotId >= 0 ? robotName(event.robotId) : QString("시스템");
    QListWidgetItem *item = new QListWidgetItem(QString("[%1] %2  %3")
                                                .arg(severity, source, event.message));
    item->setForeground(QColor(color));
    m_eventList->insertItem(0, item);
    if (isVictimDetectedEvent(event)) {
        showVictimAlert(event);
    }
    if (m_logTable) {
        const QString tableSeverity = event.severity >= 3 ? "CRITICAL" : (event.severity == 2 ? "WARN" : "INFO");
        const QDateTime eventTime = event.timestampUs > 0
            ? QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(event.timestampUs / 1000))
            : QDateTime::currentDateTime();
        appendLogTableRow(tableSeverity,
                          eventTime.toString("yyyy-MM-dd HH:mm:ss.zzz"),
                          source,
                          isVictimDetectedEvent(event) ? "안전" : "시스템",
                          event.message,
                          true);
        while (m_logTable->rowCount() > 200) {
            m_logTable->removeRow(m_logTable->rowCount() - 1);
        }
        updateLogSummary();
    } else if (m_logPageList) {
        QListWidgetItem *copy = new QListWidgetItem(item->text());
        copy->setForeground(QColor(color));
        m_logPageList->insertItem(0, copy);
        while (m_logPageList->count() > 200) {
            delete m_logPageList->takeItem(m_logPageList->count() - 1);
        }
    }
    while (m_eventList->count() > 80) {
        delete m_eventList->takeItem(m_eventList->count() - 1);
    }
    updateMissionSummary();
}

bool MainWindow::isVictimDetectedEvent(const UiEvent &event) const
{
    const bool messageLooksLikeVictim =
        event.message.contains("구조자", Qt::CaseInsensitive)
        || event.message.contains("요구조자", Qt::CaseInsensitive)
        || event.message.contains("생존자", Qt::CaseInsensitive)
        || event.message.contains("victim", Qt::CaseInsensitive)
        || event.message.contains("survivor", Qt::CaseInsensitive);

    if (event.type == kEventTypeVictimDetected) {
        if (event.code == 0) {
            return false;
        }
        return event.code > 0 || messageLooksLikeVictim;
    }

    return messageLooksLikeVictim;
}

void MainWindow::showVictimAlert(const UiEvent &event)
{
    if (!m_victimAlertButton || event.robotId < 0) {
        return;
    }
    m_victimAlertRobotId = event.robotId;
    const QString source = robotName(event.robotId);
    if (m_victimAlertTitle) {
        m_victimAlertTitle->setText("생존자 탐지");
    }
    if (m_victimAlertRobot) {
        m_victimAlertRobot->setText(source);
    }
    if (m_victimAlertAction) {
        m_victimAlertAction->setText("카메라 보기");
    }
    m_victimAlertButton->adjustSize();
    positionVictimAlert();
    m_victimAlertButton->raise();
    m_victimAlertButton->show();
}

void MainWindow::positionVictimAlert()
{
    if (!m_victimAlertButton || !centralWidget()) {
        return;
    }
    const QSize hint = m_victimAlertButton->sizeHint();
    const int w = qMax(700, hint.width());
    const int h = qMax(300, hint.height());
    const QRect r = centralWidget()->rect();
    m_victimAlertButton->setGeometry(r.center().x() - w / 2,
                                     r.center().y() - h / 2,
                                     w,
                                     h);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    positionVictimAlert();
}

void MainWindow::showPage(int index)
{
    if (!m_contentStack || index < 0 || index >= m_contentStack->count()) {
        return;
    }
    restoreMapToDashboard();
    m_contentStack->setCurrentIndex(index);
    static const QStringList crumbs = {
        "  >  재난 탐색",
        "  >  로봇 상태",
        "  >  로그",
        "  >  설정"
    };
    if (m_crumb && index < crumbs.size()) {
        m_crumb->setText(crumbs[index]);
    }
    for (int i = 0; i < m_navButtons.size(); ++i) {
        m_navButtons[i]->setObjectName(i == index ? "navButtonActive" : "navButton");
        m_navButtons[i]->style()->unpolish(m_navButtons[i]);
        m_navButtons[i]->style()->polish(m_navButtons[i]);
    }
    if (index == 0 && m_map) {
        QTimer::singleShot(0, this, [this]() {
            if (m_map) {
                m_map->fitToAvailableSize();
            }
        });
    }
}

void MainWindow::showCameraFullscreen(int robotId)
{
    if (!m_contentStack || !m_cameraPage || !m_expandedVideoTile ||
        robotId < 0 || robotId >= m_robotCount) {
        return;
    }
    if (m_contentStack->currentWidget() != m_cameraPage) {
        m_hasCameraReturnTarget = true;
        m_cameraReturnIndex = m_contentStack->currentIndex();
        m_cameraReturnWidget = m_contentStack->currentWidget();
        m_cameraReturnCrumb = m_crumb ? m_crumb->text() : QString();
    }
    m_expandedRobotId = robotId;
    m_expandedVideoTile->setRobotId(robotId);
    if (m_cameraTitle) {
        m_cameraTitle->setText(QString("%1 카메라").arg(robotName(robotId)));
    }
    if (m_crumb) {
        m_crumb->setText("  >  카메라 스트리밍");
    }
    if (robotId < m_snapshots.size()) {
        m_expandedVideoTile->setSnapshot(m_snapshots[robotId]);
    }
    m_contentStack->setCurrentWidget(m_cameraPage);
}

void MainWindow::leaveCameraFullscreen()
{
    if (!m_contentStack) {
        return;
    }
    m_expandedRobotId = -1;
    if (m_hasCameraReturnTarget) {
        const int returnIndex = m_cameraReturnIndex;
        QWidget *returnWidget = m_cameraReturnWidget;
        const QString returnCrumb = m_cameraReturnCrumb;
        m_hasCameraReturnTarget = false;
        m_cameraReturnIndex = 0;
        m_cameraReturnWidget = nullptr;
        m_cameraReturnCrumb.clear();

        if (returnIndex >= 0 && returnIndex < 4 &&
            returnWidget == m_contentStack->widget(returnIndex)) {
            showPage(returnIndex);
            return;
        }
        if (returnWidget && m_contentStack->indexOf(returnWidget) >= 0) {
            m_contentStack->setCurrentWidget(returnWidget);
            if (m_crumb) {
                m_crumb->setText(returnCrumb);
            }
            return;
        }
    }
    showPage(0);
}

void MainWindow::showMapFullscreen()
{
    if (!m_contentStack || !m_mapFullscreenPage || !m_mapFullscreenContentLayout || !m_map) {
        return;
    }
    m_mapFullscreenContentLayout->addWidget(m_map, 1);
    if (m_crumb) {
        m_crumb->setText("  >  탐색 지도");
    }
    m_contentStack->setCurrentWidget(m_mapFullscreenPage);
    QTimer::singleShot(0, this, [this]() {
        if (m_map) {
            m_map->fitToAvailableSize(2.2f);
        }
    });
}

void MainWindow::restoreMapToDashboard()
{
    if (!m_mapLayout || !m_map) {
        return;
    }
    if (m_mapLayout->indexOf(m_map) >= 0) {
        return;
    }
    m_mapLayout->addWidget(m_map, 1);
}

void MainWindow::leaveMapFullscreen()
{
    if (!m_contentStack || !m_mapLayout || !m_map) {
        return;
    }
    restoreMapToDashboard();
    showPage(0);
}

void MainWindow::showMap2D()
{
    m_map->setViewMode3D(false);
    if (m_contentStack && m_contentStack->currentWidget() == m_mapFullscreenPage) {
        QTimer::singleShot(0, this, [this]() {
            if (m_map) {
                m_map->fitToAvailableSize(2.2f);
            }
        });
    }
    m_btn2d->setObjectName("smallActive");
    m_btn3d->setObjectName("smallButton");
    m_fullscreenBtn2d->setObjectName("smallActive");
    m_fullscreenBtn3d->setObjectName("smallButton");
    for (QPushButton *button : {m_btn2d, m_btn3d, m_fullscreenBtn2d, m_fullscreenBtn3d}) {
        button->style()->unpolish(button);
        button->style()->polish(button);
    }
}

void MainWindow::showMap3D()
{
    m_map->setViewMode3D(true);
    m_btn2d->setObjectName("smallButton");
    m_btn3d->setObjectName("smallActive");
    m_fullscreenBtn2d->setObjectName("smallButton");
    m_fullscreenBtn3d->setObjectName("smallActive");
    for (QPushButton *button : {m_btn2d, m_btn3d, m_fullscreenBtn2d, m_fullscreenBtn3d}) {
        button->style()->unpolish(button);
        button->style()->polish(button);
    }
}

void MainWindow::selectRobot(int robotId)
{
    if (robotId != kAllRobotsSelection && (robotId < 0 || robotId >= m_robotCount)) {
        return;
    }
    m_selectedRobot = robotId;
    if (m_robotSelector) {
        const int idx = m_robotSelector->findData(robotId);
        if (idx >= 0 && m_robotSelector->currentIndex() != idx) {
            m_updatingRobotSelector = true;
            m_robotSelector->setCurrentIndex(idx);
            m_updatingRobotSelector = false;
        }
    }
    m_map->setSelectedRobot(robotId);

    const int layoutRobotId = featuredVideoRobotForSelection(robotId, m_videoPlacements.size());
    if (layoutRobotId >= 0 && layoutRobotId < m_videoPlacements.size()) {
        int largestRobotId = layoutRobotId;
        int largestArea = 0;
        for (int i = 0; i < m_videoPlacements.size(); ++i) {
            const VideoPlacement &placement = m_videoPlacements[i];
            const int area = placement.rowSpan * placement.columnSpan;
            if (area > largestArea) {
                largestArea = area;
                largestRobotId = i;
            }
        }
        if (largestRobotId != layoutRobotId) {
            std::swap(m_videoPlacements[layoutRobotId], m_videoPlacements[largestRobotId]);
        }
    }

    relayoutVideoTiles();
    refreshRobotList();
}

void MainWindow::refreshRobotSelector()
{
    if (!m_robotSelector) {
        return;
    }

    auto fitPopupHeightToItems = [this]() {
        QAbstractItemView *view = m_robotSelector->view();
        if (!view || m_robotSelector->count() <= 0) {
            return;
        }
        const int rowHeight = qMax(1, view->sizeHintForRow(0));
        view->setFrameShape(QFrame::NoFrame);
        view->setLineWidth(0);
        view->setMidLineWidth(0);
        view->setFixedHeight(rowHeight * m_robotSelector->count());
    };

    QVector<int> availableIds;
    availableIds.append(kAllRobotsSelection);
    const int count = qBound(0, m_robotCount, kMaxRobots);
    for (int i = 0; i < count; ++i) {
        availableIds.append(i);
    }

    if (availableIds == m_availableRobotIds) {
        const int idx = m_robotSelector->findData(m_selectedRobot);
        if (idx >= 0 && m_robotSelector->currentIndex() != idx) {
            m_updatingRobotSelector = true;
            m_robotSelector->setCurrentIndex(idx);
            m_updatingRobotSelector = false;
        }
        fitPopupHeightToItems();
        return;
    }
    m_availableRobotIds = availableIds;

    m_updatingRobotSelector = true;
    m_robotSelector->clear();
    m_robotSelector->setEnabled(true);
    for (int robotId : availableIds) {
        m_robotSelector->addItem(robotId == kAllRobotsSelection ? QStringLiteral("전체")
                                                                : robotName(robotId),
                                 robotId);
    }

    const int selectedIndex = m_robotSelector->findData(m_selectedRobot);
    if (selectedIndex >= 0) {
        m_robotSelector->setCurrentIndex(selectedIndex);
    } else if (m_robotSelector->count() > 0) {
        m_robotSelector->setCurrentIndex(0);
        m_selectedRobot = m_robotSelector->itemData(0).toInt();
    }
    m_updatingRobotSelector = false;
    fitPopupHeightToItems();
}

void MainWindow::refreshRobotList()
{
    if (m_selectedRobot == kAllRobotsSelection) {
        if (!m_robotInfo || !m_missionInfo || !m_selected) {
            return;
        }
        const QVector<RobotSnapshot> deployedSnapshots = snapshotsForDeployedRobots(m_snapshots, m_robotCount);
        int connected = 0;
        float progressTotal = 0.0f;
        int progressCount = 0;
        for (const RobotSnapshot &snapshot : deployedSnapshots) {
            if (snapshot.connected) {
                ++connected;
                progressTotal += snapshot.missionProgress > 1.0f
                    ? snapshot.missionProgress
                    : snapshot.missionProgress * 100.0f;
                ++progressCount;
            }
        }
        const int progress = progressCount > 0
            ? qBound(0, qRound(progressTotal / progressCount), 100)
            : 0;
        m_robotInfo->setText(QString("로봇 상태  |  전체 선택  |  온라인 %1/%2")
                             .arg(connected)
                             .arg(deployedSnapshots.size()));
        m_missionInfo->setText(QString("임무 정보  |  전체 로봇 제어     평균 진행률: %1%")
                               .arg(progress));
        m_selected->setText(QStringLiteral("좌표  |  전체 선택  |  경로 생성 시 모든 투입 로봇 기준"));
        return;
    }
    if (m_selectedRobot < 0 || m_selectedRobot >= m_snapshots.size()) {
        return;
    }
    if (!m_robotInfo || !m_missionInfo || !m_selected) {
        return;
    }
    const RobotSnapshot &s = m_snapshots[m_selectedRobot];
    m_robotInfo->setText(QString("로봇 상태  |  %1  %2  |  배터리 %3%  |  RTT %4 ms")
                         .arg(robotName(s.id),
                              s.connected ? "온라인" : (robotConnectionLost(s) ? "연결 끊김" : "연결 안됨"))
                         .arg(displayBatteryPercent(s.id, s.battery))
                         .arg(s.linkRttMs, 0, 'f', 1));
    m_missionInfo->setText(QString("임무 정보  |  현재 임무: %1     목표거리: %2     진행률: %3%     Waypoint: %4")
                           .arg(robotMissionStateText(s),
                                pathDistanceSummary(s))
                           .arg(missionProgressPercent(s))
                           .arg(waypointSummary(s)));
    m_selected->setText(QString("좌표  |  x %1  y %2  θ %3  |  IMG %4 fps  LiDAR %5 fps")
                        .arg(s.x, 0, 'f', 2)
                        .arg(s.y, 0, 'f', 2)
                        .arg(s.theta, 0, 'f', 2)
                        .arg(s.imageFps, 0, 'f', 1)
                        .arg(s.lidarFps, 0, 'f', 1));
}

void MainWindow::sendCommand(uint8_t commandType, float vx, float vy, float omega)
{
    const int severity = commandType == CMD_TYPE_ESTOP ? 4
        : (commandType == CMD_TYPE_MOVE ? 2
           : (commandType == CMD_TYPE_STOP ? 3 : 1));
    auto noteCommandState = [this, commandType](int displayRobotId) {
        if (displayRobotId < 0) {
            return;
        }
        if (commandType == CMD_TYPE_MOVE) {
            m_commandMovingRobotIds.insert(displayRobotId);
        } else if (commandType == CMD_TYPE_STOP || commandType == CMD_TYPE_ESTOP ||
                   commandType == CMD_TYPE_PAUSE_MISSION ||
                   commandType == CMD_TYPE_CANCEL_MISSION) {
            m_commandMovingRobotIds.remove(displayRobotId);
        }
    };
    auto showMoveIndicators = [this, commandType](const QVector<int> &robotIds) {
        if (commandType == CMD_TYPE_MOVE && m_map && !robotIds.isEmpty()) {
            m_map->showMoveCommandIndicators(robotIds);
        }
    };
    int targetRobot = m_selectedRobot;
    if (m_robotSelector && m_robotSelector->currentIndex() >= 0) {
        targetRobot = m_robotSelector->currentData().toInt();
        m_selectedRobot = targetRobot;
    }

    if (m_demoMode) {
        Q_UNUSED(vx);
        Q_UNUSED(vy);
        Q_UNUSED(omega);
        QVector<int> demoMoveRobotIds;
        if (targetRobot == kAllRobotsSelection) {
            for (int robotId = 0; robotId < m_robotCount; ++robotId) {
                demoMoveRobotIds.append(robotId);
            }
        } else if (targetRobot >= 0) {
            demoMoveRobotIds.append(targetRobot);
        }
        showMoveIndicators(demoMoveRobotIds);
        appendEvent(UiEvent{targetRobot, severity, commandType, 0,
                            QString("테스트 명령: %1").arg(commandTypeText(commandType))});
        return;
    }

    if (targetRobot == kAllRobotsSelection) {
        QString failed;
        int sent = 0;
        QVector<int> sentRobotIds;
        for (int robotId = 0; robotId < m_robotCount; ++robotId) {
            if (isHardcodedDisplayRobot(robotId)) {
                continue;
            }
            const int commandRobotId = displayToPhysicalRobotId(robotId);
            QString errorMessage;
            if (!m_monitor.sendCommand(commandRobotId, commandType, vx, vy, omega, &errorMessage)) {
                failed += QString("%1: %2\n").arg(robotName(robotId), errorMessage);
                continue;
            }
            noteCommandState(robotId);
            sentRobotIds.append(robotId);
            ++sent;
        }
        showMoveIndicators(sentRobotIds);
        if (!failed.isEmpty()) {
            QMessageBox::warning(this, "명령 전송 실패",
                                 QString("일부 로봇의 브릿지 명령 큐에 쓰지 못했습니다.\n%1").arg(failed));
        }
        if (sent > 0) {
            appendEvent(UiEvent{kAllRobotsSelection, severity, commandType, 0,
                                QString("전체 명령 전송: %1 (%2대)")
                                    .arg(commandTypeText(commandType))
                                    .arg(sent)});
        }
        return;
    }

    QString errorMessage;
    if (isHardcodedDisplayRobot(targetRobot)) {
        QMessageBox::warning(this, "명령 전송 실패",
                             QString("%1은 고정 표시 로봇이라 실제 명령 대상이 아닙니다.")
                                 .arg(robotName(targetRobot)));
        return;
    }
    const int physicalTargetRobot = displayToPhysicalRobotId(targetRobot);
    if (!m_monitor.sendCommand(physicalTargetRobot, commandType, vx, vy, omega, &errorMessage)) {
        QMessageBox::warning(this, "명령 전송 실패",
                             QString("브릿지 명령 큐에 쓰지 못했습니다.\n%1").arg(errorMessage));
        return;
    }
    noteCommandState(targetRobot);
    showMoveIndicators(QVector<int>{targetRobot});
    appendEvent(UiEvent{targetRobot, severity, commandType, 0,
                        QString("명령 전송: %1").arg(commandTypeText(commandType))});
}

void MainWindow::handleRouteGenerationRequested(int robotId, const QPointF &end)
{
    if (robotId != kAllRobotsSelection && (robotId < 0 || robotId >= m_robotCount)) {
        QMessageBox::warning(this, "경로 생성 실패",
                             QString("로봇 ID가 올바르지 않습니다: %1").arg(robotId));
        return;
    }
    if (isHardcodedDisplayRobot(robotId)) {
        QMessageBox::warning(this, "경로 생성 실패",
                             QString("%1은 고정 표시 로봇이라 경로 생성 대상이 아닙니다.")
                                 .arg(robotName(robotId)));
        return;
    }

    if (m_demoMode) {
        QVector<int> visiblePathIds;
        if (robotId == kAllRobotsSelection) {
            visiblePathIds = deployedRobotIds(m_snapshots, m_robotCount);
        } else {
            const QVector<RobotSnapshot> deployedSnapshots =
                snapshotsForDeployedRobots(m_snapshots, m_robotCount);
            auto it = std::find_if(deployedSnapshots.cbegin(), deployedSnapshots.cend(),
                                   [robotId](const RobotSnapshot &snapshot) {
                                       return snapshot.id == robotId;
                                   });
            if (it != deployedSnapshots.cend() && routeEligibleSnapshot(*it)) {
                visiblePathIds.append(robotId);
            }
        }
        if (visiblePathIds.isEmpty()) {
            QMessageBox::warning(this, "경로 생성 실패",
                                 "경로를 생성할 수 있는 정상 로봇이 없습니다.");
            return;
        }
        appendEvent(UiEvent{robotId, 1, CMD_TYPE_SET_ROUTE, 0,
                            QString("테스트 경로 생성: 목적지(%1, %2)")
                                .arg(end.x(), 0, 'f', 2)
                                .arg(end.y(), 0, 'f', 2)});
        if (m_map) {
            m_map->setGlobalPathRobotIds(visiblePathIds);
            m_map->setGlobalPathsVisible(true);
        }
        return;
    }

    QString errorMessage;
    if (robotId == kAllRobotsSelection) {
        QString failed;
        int sent = 0;
        QVector<int> sentIds;
        const QVector<RobotSnapshot> deployedSnapshots =
            snapshotsForDeployedRobots(m_snapshots, m_robotCount);
        for (const RobotSnapshot &snapshot : deployedSnapshots) {
            if (isHardcodedDisplayRobot(snapshot.id)) {
                continue;
            }
            if (!routeEligibleSnapshot(snapshot)) {
                continue;
            }
            QString perRobotError;
            const int physicalRobotId = displayToPhysicalRobotId(snapshot.id);
            if (!m_monitor.sendCommand(physicalRobotId, CMD_TYPE_SET_ROUTE,
                                       static_cast<float>(end.x()),
                                       static_cast<float>(end.y()),
                                       0.0f,
                                       &perRobotError)) {
                failed += QString("%1: %2\n").arg(robotName(snapshot.id), perRobotError);
                continue;
            }
            ++sent;
            sentIds.append(snapshot.id);
        }
        if (sentIds.isEmpty()) {
            QMessageBox::warning(this, "경로 생성 실패",
                                 "경로를 생성할 수 있는 정상 로봇이 없습니다.");
            return;
        }
        if (!failed.isEmpty()) {
            QMessageBox::warning(this, "경로 생성 실패",
                                 QString("일부 로봇의 목적지 좌표를 브릿지 명령 큐에 쓰지 못했습니다.\n%1")
                                     .arg(failed));
        }
        if (sent > 0) {
            if (m_map) {
                m_map->setGlobalPathRobotIds(deployedRobotIds(m_snapshots, m_robotCount));
                m_map->setGlobalPathsVisible(true);
            }
            appendEvent(UiEvent{kAllRobotsSelection, 1, CMD_TYPE_SET_ROUTE, 0,
                                QString("전체 경로 생성 요청: 목적지(%1, %2), %3대")
                                    .arg(end.x(), 0, 'f', 2)
                                    .arg(end.y(), 0, 'f', 2)
                                    .arg(sent)});
        }
        return;
    }

    const QVector<RobotSnapshot> deployedSnapshots =
        snapshotsForDeployedRobots(m_snapshots, m_robotCount);
    auto selectedIt = std::find_if(deployedSnapshots.cbegin(), deployedSnapshots.cend(),
                                   [robotId](const RobotSnapshot &snapshot) {
                                       return snapshot.id == robotId;
                                   });
    if (selectedIt == deployedSnapshots.cend() || !routeEligibleSnapshot(*selectedIt)) {
        QMessageBox::warning(this, "경로 생성 실패",
                             QString("%1은 현재 고장/미연결 상태라 경로를 생성하지 않았습니다.")
                                 .arg(robotName(robotId)));
        return;
    }

    const int physicalRobotId = displayToPhysicalRobotId(robotId);
    if (!m_monitor.sendCommand(physicalRobotId, CMD_TYPE_SET_ROUTE,
                               static_cast<float>(end.x()),
                               static_cast<float>(end.y()),
                               0.0f,
                               &errorMessage)) {
        QMessageBox::warning(this, "경로 생성 실패",
                             QString("목적지 좌표를 브릿지 명령 큐에 쓰지 못했습니다.\n%1")
                                 .arg(errorMessage));
        return;
    }

    appendEvent(UiEvent{robotId, 1, CMD_TYPE_SET_ROUTE, 0,
                        QString("경로 생성 요청: 목적지(%1, %2)")
                            .arg(end.x(), 0, 'f', 2)
                            .arg(end.y(), 0, 'f', 2)});
    if (m_map) {
        m_map->addGlobalPathRobotIds(QVector<int>{robotId});
        m_map->setGlobalPathsVisible(true);
    }
}

void MainWindow::sendMove()
{
    sendCommand(CMD_TYPE_MOVE, 0.25f, 0.0f, 0.0f);
}

void MainWindow::sendExplore()
{
    sendCommand(CMD_TYPE_START_MISSION);
}

void MainWindow::sendStandby()
{
    sendCommand(CMD_TYPE_PAUSE_MISSION);
}

void MainWindow::sendStop()
{
    sendCommand(CMD_TYPE_STOP);
}

void MainWindow::sendEstop()
{
    sendCommand(CMD_TYPE_ESTOP);
}
