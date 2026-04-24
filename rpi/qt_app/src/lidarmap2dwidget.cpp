#include "lidarmap2dwidget.h"
#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include <cmath>

LidarMap2DWidget::LidarMap2DWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(400, 300);
    setAttribute(Qt::WA_OpaquePaintEvent);
    m_panOffset = QPoint(0, 0);
}

void LidarMap2DWidget::loadMapImage(const QString &path)
{
    if (!m_mapImage.load(path))
        qWarning("LidarMap2DWidget: map image not found: %s",
                 qPrintable(path));
    update();
}

/* ─── LiDAR 수신: 월드 좌표로 변환 후 누적 ─────────────────── */
void LidarMap2DWidget::updateLidar(const LidarData &lidar)
{
    float cos_t = cosf(lidar.robot_theta);
    float sin_t = sinf(lidar.robot_theta);

    for (const auto &p : lidar.points) {
        /* z 필터 */
        if (p.z < m_zFloor || p.z > m_zCeil) continue;

        Pt2D pt;
        pt.x = lidar.robot_x + p.x * cos_t - p.y * sin_t;
        pt.y = lidar.robot_y + p.x * sin_t + p.y * cos_t;
        pt.z = p.z;
        m_accumPts.append(pt);
    }

    /* 너무 많으면 오래된 것 삭제 */
    if (m_accumPts.size() > MAX_ACCUM)
        m_accumPts.remove(0, m_accumPts.size() - MAX_ACCUM);

    update();
}

void LidarMap2DWidget::updateOdom(const OdomData &odom)
{
    m_robotX     = odom.x;
    m_robotY     = odom.y;
    m_robotTheta = odom.theta;
    update();
}

/* ─── 좌표 변환 ─────────────────────────────────────────────── */
QPointF LidarMap2DWidget::worldToWidget(float wx, float wy) const
{
    /* 월드(m) → 픽셀 → 줌/팬 적용 */
    float px = (wx - m_originX) / m_mPerPixel;
    float py = (wy - m_originY) / m_mPerPixel;
    return QPointF(
        px * m_scale + width()  / 2.f + m_panOffset.x(),
       -py * m_scale + height() / 2.f + m_panOffset.y()
    );
}

QColor LidarMap2DWidget::zToColor(float z) const
{
    float t = (z - m_zFloor) / qMax(m_zCeil - m_zFloor, 0.001f);
    t = qBound(0.f, t, 1.f);
    /* 파랑 → 초록 → 빨강 */
    int r = (int)(255 * qMin(t * 2.f, 1.f));
    int g = (int)(255 * (t < 0.5f ? t * 2.f : (1.f - t) * 2.f));
    int b = (int)(255 * qMax(1.f - t * 2.f, 0.f));
    return QColor(r, g, b, 200);
}

/* ─── 그리기 ────────────────────────────────────────────────── */
void LidarMap2DWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, false);

    /* 배경 */
    p.fillRect(rect(), QColor(10, 11, 15));

    /* 2D 정밀지도 (배경) */
    if (!m_mapImage.isNull()) {
        QRectF mapRect(
            width() / 2.f + m_panOffset.x() - m_mapImage.width()  * m_scale / 2.f,
            height()/ 2.f + m_panOffset.y() - m_mapImage.height() * m_scale / 2.f,
            m_mapImage.width()  * m_scale,
            m_mapImage.height() * m_scale
        );
        p.setOpacity(0.5);
        p.drawImage(mapRect, m_mapImage);
        p.setOpacity(1.0);
    }

    /* 그리드 */
    p.setPen(QPen(QColor(40, 45, 55), 1));
    int gridStep = qMax(20, (int)(1.f / m_mPerPixel * m_scale)); /* 1m 간격 */
    QPointF origin = worldToWidget(0, 0);
    for (float x = origin.x(); x < width();  x += gridStep) p.drawLine((int)x,0,(int)x,height());
    for (float x = origin.x(); x > 0;        x -= gridStep) p.drawLine((int)x,0,(int)x,height());
    for (float y = origin.y(); y < height(); y += gridStep) p.drawLine(0,(int)y,width(),(int)y);
    for (float y = origin.y(); y > 0;        y -= gridStep) p.drawLine(0,(int)y,width(),(int)y);

    /* 누적 LiDAR 포인트 */
    for (const auto &pt : m_accumPts) {
        QPointF wp = worldToWidget(pt.x, pt.y);
        if (wp.x() < -5 || wp.x() > width()+5) continue;
        if (wp.y() < -5 || wp.y() > height()+5) continue;
        p.setPen(QPen(zToColor(pt.z), 2));
        p.drawPoint(wp);
    }

    /* 로봇 아이콘 */
    QPointF rp = worldToWidget(m_robotX, m_robotY);
    float sz = 12.f;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(100, 200, 100, 220));
    p.drawEllipse(rp, sz, sz);

    /* 로봇 방향 화살표 */
    p.setPen(QPen(Qt::white, 2));
    float dx = cosf(m_robotTheta) * sz * 1.5f;
    float dy = -sinf(m_robotTheta) * sz * 1.5f;
    p.drawLine(rp, QPointF(rp.x() + dx, rp.y() + dy));

    /* 범례 */
    p.setPen(Qt::white);
    p.setFont(QFont("monospace", 10));
    p.drawText(10, 20, QString("포인트: %1  스케일: %2x")
               .arg(m_accumPts.size()).arg(m_scale, 0, 'f', 1));
    p.drawText(10, 36, QString("로봇: (%1, %2)  θ: %3°")
               .arg(m_robotX, 0, 'f', 2)
               .arg(m_robotY, 0, 'f', 2)
               .arg(qRadiansToDegrees(m_robotTheta), 0, 'f', 1));
}

/* ─── 줌/팬 ─────────────────────────────────────────────────── */
void LidarMap2DWidget::wheelEvent(QWheelEvent *e)
{
    m_scale *= (e->angleDelta().y() > 0) ? 1.15f : 0.87f;
    m_scale  = qBound(0.1f, m_scale, 50.f);
    update();
}

void LidarMap2DWidget::mousePressEvent(QMouseEvent *e)
{
    m_lastMouse = e->pos();
}

void LidarMap2DWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() & Qt::LeftButton) {
        m_panOffset += e->pos() - m_lastMouse;
        m_lastMouse  = e->pos();
        update();
    }
}
