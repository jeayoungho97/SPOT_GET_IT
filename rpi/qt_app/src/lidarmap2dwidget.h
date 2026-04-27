#ifndef LIDARMAP2DWIDGET_H
#define LIDARMAP2DWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QImage>
#include <QVector>
#include "shm_reader.h"

class LidarMap2DWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LidarMap2DWidget(QWidget *parent = nullptr);

    /* 2D 정밀지도 PNG 로드 */
    void loadMapImage(const QString &path);

    /* 지도 해상도: 1픽셀 = N미터 (기본 0.05m) */
    void setMapResolution(float mPerPixel) { m_mPerPixel = mPerPixel; }

    /* 지도 원점 오프셋 (미터 → 픽셀 변환 기준점) */
    void setMapOrigin(float ox, float oy) { m_originX = ox; m_originY = oy; }

public slots:
    void updateLidar(const LidarData &lidar);
    void updateOdom(const OdomData &odom);

protected:
    void paintEvent(QPaintEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;

private:
    QPointF worldToWidget(float wx, float wy) const;
    QColor  zToColor(float z) const;

    QImage  m_mapImage;     /* 2D 정밀지도 */
    float   m_mPerPixel = 0.05f;
    float   m_originX   = 0.f;
    float   m_originY   = 0.f;

    /* 누적 포인트 */
    struct Pt2D { float x, y, z; };
    QVector<Pt2D> m_accumPts;
    static constexpr int MAX_ACCUM = 60000;

    /* 로봇 위치 */
    float m_robotX     = 0.f;
    float m_robotY     = 0.f;
    float m_robotTheta = 0.f;

    /* 뷰 변환 (줌/팬) */
    float  m_scale    = 1.f;
    QPoint m_panOffset;
    QPoint m_lastMouse;

    /* z 필터 (장애물만) */
    float m_zFloor = 0.1f;   /* 지면 이하 제거 */
    float m_zCeil  = 2.5f;   /* 천장 이상 제거 */
};

#endif // LIDARMAP2DWIDGET_H
