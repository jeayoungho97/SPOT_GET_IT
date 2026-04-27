#include "pointcloudwidget.h"
#include <cmath>
#include <algorithm>
#include <QHash>

/* ─── GLSL 셰이더 ────────────────────────────────────────────── */
static const char *VERT_SRC = R"(
attribute vec4 aPos;       /* xyz + zval */
uniform   mat4 uMvp;
uniform   float uZMin;
uniform   float uZMax;
uniform   int   uIsLine;
varying   vec3  vColor;

vec3 zColor(float t) {
    t = clamp(t, 0.0, 1.0);
    vec3 c;
    if      (t < 0.25) c = mix(vec3(0.0,0.0,1.0), vec3(0.0,1.0,1.0), t*4.0);
    else if (t < 0.50) c = mix(vec3(0.0,1.0,1.0), vec3(0.0,1.0,0.0), (t-0.25)*4.0);
    else if (t < 0.75) c = mix(vec3(0.0,1.0,0.0), vec3(1.0,1.0,0.0), (t-0.50)*4.0);
    else               c = mix(vec3(1.0,1.0,0.0), vec3(1.0,0.0,0.0), (t-0.75)*4.0);
    return c;
}

void main() {
    gl_Position  = uMvp * vec4(aPos.xyz, 1.0);
    gl_PointSize = 3.0;
    float t = (aPos.w - uZMin) / max(uZMax - uZMin, 0.001);
    vColor = zColor(t);
}
)";

static const char *FRAG_SRC = R"(
varying vec3 vColor;
uniform int  uIsLine;
void main() {
    float alpha = (uIsLine == 1) ? 0.35 : 0.9;
    gl_FragColor = vec4(vColor, alpha);
}
)";

/* ─── 생성/소멸 ─────────────────────────────────────────────── */
PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(400, 300);
    QSurfaceFormat fmt;
    fmt.setSamples(4);
    setFormat(fmt);
}

PointCloudWidget::~PointCloudWidget()
{
    makeCurrent();
    m_pointVBO.destroy();
    m_lineVBO.destroy();
    doneCurrent();
}

/* ─── OpenGL 초기화 ─────────────────────────────────────────── */
void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.05f, 0.05f, 0.08f, 1.f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);

    m_prog.addShaderFromSourceCode(QOpenGLShader::Vertex,   VERT_SRC);
    m_prog.addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SRC);
    m_prog.link();

    m_locMvp    = m_prog.uniformLocation("uMvp");
    m_locZMin   = m_prog.uniformLocation("uZMin");
    m_locZMax   = m_prog.uniformLocation("uZMax");
    m_locIsLine = m_prog.uniformLocation("uIsLine");

    m_pointVBO.create();
    m_lineVBO.create();

    m_glReady = true;
}

void PointCloudWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

/* ─── 렌더링 ────────────────────────────────────────────────── */
void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (m_renderPoints.isEmpty()) return;

    /* MVP 행렬 */
    QMatrix4x4 proj, view, model;
    float aspect = width() / float(height());
    proj.perspective(45.f, aspect, 0.1f, 500.f);

    float yawR   = qDegreesToRadians(m_yaw);
    float pitchR = qDegreesToRadians(m_pitch);
    QVector3D eye(
        m_distance * cosf(pitchR) * sinf(yawR),
        m_distance * cosf(pitchR) * cosf(yawR),
        m_distance * sinf(pitchR)
    );
    view.lookAt(eye, {0,0,0}, {0,0,1});
    QMatrix4x4 mvp = proj * view * model;

    m_prog.bind();
    m_prog.setUniformValue(m_locMvp,  mvp);
    m_prog.setUniformValue(m_locZMin, m_zMin);
    m_prog.setUniformValue(m_locZMax, m_zMax);

    int posLoc = m_prog.attributeLocation("aPos");

    /* ── 포인트 렌더링 ── */
    m_pointVBO.bind();
    m_prog.setUniformValue(m_locIsLine, 0);
    m_prog.enableAttributeArray(posLoc);
    m_prog.setAttributeBuffer(posLoc, GL_FLOAT, 0, 4, sizeof(GpuPoint));
    glDrawArrays(GL_POINTS, 0, m_pointCount);

    m_pointVBO.release();

    /* ── 연결선 렌더링 ── */
    if (!m_linePoints.isEmpty()) {
        m_lineVBO.bind();
        m_prog.setUniformValue(m_locIsLine, 1);
        m_prog.setAttributeBuffer(posLoc, GL_FLOAT, 0, 4, sizeof(GpuPoint));
        glDrawArrays(GL_LINES, 0, m_linePointCount);
        m_lineVBO.release();
    }

    m_prog.disableAttributeArray(posLoc);
    m_prog.release();
}

/* ─── LiDAR 데이터 수신 ─────────────────────────────────────── */
void PointCloudWidget::updateLidar(const LidarData &lidar)
{
    if (lidar.points.isEmpty()) return;

    /* 로봇 기준 → 월드 좌표계 변환 */
    float cos_t = cosf(lidar.robot_theta);
    float sin_t = sinf(lidar.robot_theta);

    AccumFrame frame;
    frame.pts.reserve(lidar.points.size());
    for (const auto &p : lidar.points) {
        GpuPoint gp;
        gp.x    = lidar.robot_x + p.x * cos_t - p.y * sin_t;
        gp.y    = lidar.robot_y + p.x * sin_t + p.y * cos_t;
        gp.z    = p.z;
        gp.zval = p.z;
        frame.pts.append(gp);
    }

    m_accumFrames.append(frame);
    if (m_accumFrames.size() > m_maxAccumFrames)
        m_accumFrames.removeFirst();

    /* 누적 포인트 합치기 */
    m_renderPoints.clear();
    for (const auto &f : m_accumFrames)
        m_renderPoints.append(f.pts);

    buildWireframe();
    uploadToGpu();
    update();
}

void PointCloudWidget::clearAccum()
{
    m_accumFrames.clear();
    m_renderPoints.clear();
    m_linePoints.clear();
    update();
}

/* ─── 와이어프레임 연결선 생성 ──────────────────────────────── */
void PointCloudWidget::buildWireframe()
{
    m_linePoints.clear();
    if (m_renderPoints.size() < 2) return;

    const float THRESH = m_connThresh;
    const float THRESH2 = THRESH * THRESH;

    /* 방위각 기준 360개 빈으로 분류 */
    const int N_AZ = 360;
    QVector<QVector<int>> bins(N_AZ);

    for (int i = 0; i < m_renderPoints.size(); i++) {
        const auto &p = m_renderPoints[i];
        float az = atan2f(p.y, p.x) * (180.f / M_PI);
        if (az < 0) az += 360.f;
        int bin = qBound(0, (int)az, N_AZ - 1);
        bins[bin].append(i);
    }

    /* 각 빈을 z 순으로 정렬 */
    for (auto &bin : bins)
        std::sort(bin.begin(), bin.end(), [&](int a, int b) {
            return m_renderPoints[a].z < m_renderPoints[b].z;
        });

    /* 수직 연결: 같은 빈 내 인접 z */
    for (const auto &bin : bins) {
        for (int k = 1; k < bin.size(); k++) {
            const auto &a = m_renderPoints[bin[k-1]];
            const auto &b = m_renderPoints[bin[k]];
            float dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
            if (dx*dx + dy*dy + dz*dz < THRESH2 * 4.f) {
                m_linePoints.append(a);
                m_linePoints.append(b);
            }
        }
    }

    /* 수평 연결: 인접 빈 사이 가장 가까운 포인트 */
    for (int az = 0; az < N_AZ; az++) {
        int az2 = (az + 1) % N_AZ;
        for (int i : bins[az]) {
            const auto &pa = m_renderPoints[i];
            float bestD2 = THRESH2;
            int   bestJ  = -1;
            for (int j : bins[az2]) {
                const auto &pb = m_renderPoints[j];
                float dx = pa.x-pb.x, dy = pa.y-pb.y, dz = pa.z-pb.z;
                float d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < bestD2) { bestD2 = d2; bestJ = j; }
            }
            if (bestJ >= 0) {
                m_linePoints.append(pa);
                m_linePoints.append(m_renderPoints[bestJ]);
            }
        }
    }
}

/* ─── GPU 업로드 ─────────────────────────────────────────────── */
void PointCloudWidget::uploadToGpu()
{
    if (!m_glReady) return;
    makeCurrent();

    m_pointVBO.bind();
    m_pointVBO.allocate(m_renderPoints.constData(),
                        m_renderPoints.size() * sizeof(GpuPoint));
    m_pointVBO.release();
    m_pointCount = m_renderPoints.size();

    if (!m_linePoints.isEmpty()) {
        m_lineVBO.bind();
        m_lineVBO.allocate(m_linePoints.constData(),
                           m_linePoints.size() * sizeof(GpuPoint));
        m_lineVBO.release();
        m_linePointCount = m_linePoints.size();
    }

    doneCurrent();
}

/* ─── 마우스/휠 ─────────────────────────────────────────────── */
void PointCloudWidget::mousePressEvent(QMouseEvent *e)
{
    m_lastMouse = e->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *e)
{
    QPoint delta = e->pos() - m_lastMouse;
    m_yaw   += delta.x() * 0.5f;
    m_pitch += delta.y() * 0.5f;
    m_pitch  = qBound(-89.f, m_pitch, 89.f);
    m_lastMouse = e->pos();
    update();
}

void PointCloudWidget::wheelEvent(QWheelEvent *e)
{
    m_distance -= e->angleDelta().y() * 0.02f;
    m_distance  = qBound(1.f, m_distance, 200.f);
    update();
}

void PointCloudWidget::mouseDoubleClickEvent(QMouseEvent *)
{
    /* 더블클릭: 카메라 리셋 */
    m_yaw = 30.f; m_pitch = 45.f; m_distance = 20.f;
    update();
}
