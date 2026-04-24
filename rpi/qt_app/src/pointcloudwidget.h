#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QVector>
#include "shm_reader.h"

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget() override;

    /* 누적 스캔 수 (0 = 누적 없이 최신 1프레임만) */
    void setMaxAccumFrames(int n) { m_maxAccumFrames = n; }

public slots:
    void updateLidar(const LidarData &lidar);
    void clearAccum();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;

private:
    /* GPU 정점 포맷: xyz + z값(색상용) */
    struct GpuPoint { float x, y, z, zval; };

    void buildWireframe();
    void uploadToGpu();
    QVector3D zToColor(float t) const;

    /* GL 자원 */
    QOpenGLShaderProgram  m_prog;
    QOpenGLBuffer         m_pointVBO { QOpenGLBuffer::VertexBuffer };
    QOpenGLBuffer         m_lineVBO  { QOpenGLBuffer::VertexBuffer };
    int                   m_pointCount = 0;
    int                   m_linePointCount = 0;

    /* 누적 포인트 (로봇 기준 → 월드 좌표계 변환 후 저장) */
    struct AccumFrame {
        QVector<GpuPoint> pts;
    };
    QVector<AccumFrame>   m_accumFrames;
    int                   m_maxAccumFrames = 10;

    /* 렌더링에 올라갈 최종 포인트 버퍼 */
    QVector<GpuPoint>     m_renderPoints;
    QVector<GpuPoint>     m_linePoints;   /* GL_LINES용 (start,end 쌍) */

    /* 카메라 상태 */
    float   m_yaw      = 30.f;
    float   m_pitch    = 45.f;
    float   m_distance = 20.f;
    QPoint  m_lastMouse;

    /* z 범위 (색상 매핑) */
    float m_zMin = -0.5f;
    float m_zMax =  2.5f;

    /* 연결선 거리 임계값 (m) */
    float m_connThresh = 0.5f;

    bool m_glReady = false;

    /* 유니폼 위치 */
    int m_locMvp    = -1;
    int m_locZMin   = -1;
    int m_locZMax   = -1;
    int m_locIsLine = -1;
};

#endif // POINTCLOUDWIDGET_H
