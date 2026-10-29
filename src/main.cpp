/**
 * @file main.cpp
 * @brief 命令列入口、Qt/OpenGL 點雲視窗及互動事件處理。
 *
 * 無圖形的檢查／匯出先完成，確定需要視窗後才建立 QApplication。
 * 視窗使用 OpenGL 2.1 compatibility immediate mode；所有 reader 與 UI
 * 操作在主執行緒執行。 累計地圖、Odometry 與視窗之間使用非擁有參考，生命週期由
 * main() 的區域變數維持。
 *
 * @par English
 * Command-line entry point, Qt/OpenGL cloud viewer, and interaction handling. Complete
 * headless inspection/export before creating QApplication when a window is needed. Use OpenGL
 * 2.1 compatibility immediate mode; all reader and UI work runs on the main thread. Map,
 * odometry, and window references are non-owning, with lifetimes maintained by main() locals.
 */
#include "accumulation.hpp"
#include "bag.hpp"
#include "intensity.hpp"
#include "odometry.hpp"
#include "playback.hpp"
#include <GL/gl.h>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QLabel>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMessageBox>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QProgressDialog>
#include <QPushButton>
#include <QQuaternion>
#include <QSignalBlocker>
#include <QSlider>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
/** @brief 播放選單與排程共用的倍率，避免標籤與實際除數不一致。
 *
 * @par English
 * Playback rates shared by the selector and scheduler to keep labels and actual timing
 * divisors consistent.
 */
constexpr std::array<double, 7> playbackRates{.25, .5, 1, 2, 4, 8, 16};
/**
 * @class View
 * @brief 顯示單幀或累計樣本、map 原點、光達位姿及歷史軌跡的 OpenGL widget。
 *
 * 此類別不負責讀檔或累計；控制層更新 cloud 與 accumulated 後呼叫 update()。
 * 相機以 map 的 +Z 為 up，預設由 −X 朝 +X 看，讓 +Y 位於畫面左側。
 *
 * @par English
 * An OpenGL widget displaying a single frame or accumulated sample, the map origin, LiDAR
 * pose, and trajectory. This class does not read bags or accumulate clouds; the controller
 * updates cloud and accumulated, then calls update(). The camera uses map +Z up and initially
 * looks along +X from the -X side, placing +Y to the left.
 */
/**
 * @var bool View::showPose
 * @brief 是否繪製目前光達位置、局部座標軸與標籤。
 *
 * @par English
 * Whether to draw the current LiDAR position, local axes, and label.
 */
/**
 * @var float View::radius
 * @brief 由顯示樣本估算的包圍球半徑。
 *
 * @par English
 * Bounding-sphere radius estimated from display points.
 */
/**
 * @var float View::distance
 * @brief 相機 eye 到觀察中心的距離。
 *
 * @par English
 * Distance from the camera eye to its observation center.
 */
/**
 * @var float View::yaw
 * @brief 繞 map Z 的 orbit 方位角，單位度。
 *
 * @par English
 * Orbit azimuth around map Z, in degrees.
 */
class View : public QOpenGLWidget {
public:
  bag::Cloud
      cloud; ///< 目前幀的資料與時間，即使顯示累計地圖仍保留此時間作配對。
      ///< EN: Current frame data and timestamp, retained for pose matching even when displaying the
      ///< accumulated map.
  const bag::OdometryTrack *track =
      nullptr; ///< 非擁有軌跡指標；指向 main() 中存活至視窗關閉的物件。
      ///< EN: Non-owning track pointer; its main() object remains alive until the window closes.
  std::optional<size_t>
      poseIndex; ///< track 中對應目前點雲的索引，無合格配對時為空。
      ///< EN: Matching track index for the current cloud, empty when no acceptable pose exists.
  bool showPose = true,
       showTrajectory = true; ///< 獨立控制目前位置／朝向與歷史軌跡 overlay。
       ///< EN: Independent visibility controls for pose/orientation and trajectory overlays.
  /**
   * @brief 依目前 cloud 的 Header 時間與 frame 更新光達配對索引。
   *
   * 必須在每次更換實際顯示幀後呼叫；配對失敗時清除索引，不沿用上一幀位置。
   *
   * @par English
   * Update the LiDAR match using the current cloud Header timestamp and frame. Call after every
   * actual frame change. Clear the index on failure instead of reusing the preceding frame's
   * position.
   */
  void syncPose() {
    poseIndex = track ? track->match(cloud.stamp, cloud.frame)
                      : std::optional<size_t>{};
  }
  const std::vector<bag::Point> *accumulated =
      nullptr; ///< 非擁有顯示樣本指標；空指標表示原始單幀模式。
      ///< EN: Non-owning accumulated display sample pointer; null selects the original single-frame
      ///< mode.
  /**
   * @brief 取得本次繪圖要使用的點陣列。
   *
   * @return accumulated 非空時使用累計樣本，否則使用單幀 cloud.points。
   *
   * @par English
   * Get points for the next draw. Returns the accumulated sample when accumulated is non-null,
   * otherwise cloud.points.
   */
  const std::vector<bag::Point> &points() const {
    return accumulated ? *accumulated : cloud.points;
  }
  QVector3D center; ///< orbit camera 的世界座標觀察中心。
  ///< EN: Orbit camera observation center in world coordinates.
  float radius = 1, distance = 3, yaw = 0,
        pitch = 0; ///< 擬合半徑、眼到中心距離，以及以度表示的水平／俯仰角。
        ///< EN: Fitted radius, eye-to-center distance, and azimuth/pitch angles in degrees.
  QPoint last;     ///< 前一次滑鼠事件的 widget 像素位置。
  ///< EN: Widget pixel position of the previous mouse event.
  bag::IntensityScale intensityScale; ///< 顯示用強度統計，不改寫原始資料。
  ///< EN: Display-only intensity statistics; source values remain unchanged.
  int colorMode = 0; ///< 0 高度色、1 單色、2 反射強度彩色。
  ///< EN: Color modes: 0 height, 1 single color, 2 reflectance color.
  bool playbackActive =
      false; ///< 播放時降低顯示取樣量以控制繪圖成本，暫停恢復完整顯示上限。
      ///< EN: Reduce display sampling while playing to bound drawing cost; restore the full display
      ///< limit when paused.
  uint64_t paintCount = 0; ///< 實際 OpenGL 重繪次數，用於播放量測。
  ///< EN: Actual OpenGL repaint count used by playback measurements.
  size_t maxPoints = 1000000; ///< 畫面點數上限；原始單幀超過時增加 stride。
  ///< EN: Display point limit; increase stride when an original frame exceeds it.
  /**
   * @brief 建立具有最小可用尺寸的點雲 widget。
   *
   * @param parent Qt 父物件，依 Qt 物件樹管理 widget 所有權。
   *
   * @par English
   * Create a cloud widget with a usable minimum size. Parameter parent participates in Qt
   * parent-child ownership of the widget.
   */
  explicit View(QWidget *parent = nullptr) : QOpenGLWidget(parent) {
    setMinimumSize(640, 480);
  }
  /**
   * @brief 以目前顯示點的 AABB 重設觀察中心、距離與預設方向。
   *
   * 取包圍盒對角線一半為 radius，最小半徑 0.1；距離設為 radius 的 2.8 倍。
   * 對累計模式而言 AABB 根據顯示樣本估算，而非重新遍歷完整佔用地圖。
   * 空點集時保持既有相機設定。
   *
   * @par English
   * Fit the observation center, distance, and initial orientation to the current points'
   * axis-aligned bounding box. Use half the box diagonal as radius with a minimum of 0.1, and
   * set distance to 2.8 times radius. Accumulated mode estimates bounds from display samples
   * rather than traversing the complete voxel map. Leave the camera unchanged for empty input.
   */
  void resetCamera() {
    if (points().empty())
      return;
    QVector3D low(points()[0].x, points()[0].y, points()[0].z), high = low;
    for (auto p : points()) {
      low.setX(std::min(low.x(), p.x));
      low.setY(std::min(low.y(), p.y));
      low.setZ(std::min(low.z(), p.z));
      high.setX(std::max(high.x(), p.x));
      high.setY(std::max(high.y(), p.y));
      high.setZ(std::max(high.z(), p.z));
    }
    center = low * .5f + high * .5f;
    radius = std::max((high - low).length() * .5f, .1f);
    distance = radius * 2.8f;
    yaw = 0;
    pitch = 0;
    update();
  }
  /**
   * @brief 計算固定 Z-up 的 orbit camera 世界到相機矩陣。
   *
   * eye = center + distance * (-cos(pitch)cos(yaw), -cos(pitch)sin(yaw),
   * sin(pitch))。 yaw=pitch=0 時 +X 指向視線前方、+Y 投影到左邊、+Z
   * 投影到上邊。 俯仰由滑鼠處理器限制在 ±89 度，避免 lookAt 的 up 與視線共線。
   *
   * @return 可直接交給 OpenGL model-view 的 Qt column-major 矩陣。
   *
   * @par English
   * Compute a world-to-camera orbit matrix with fixed Z up. The eye offset is distance *
   * (-cos(pitch)cos(yaw), -cos(pitch)sin(yaw), sin(pitch)). At zero yaw and pitch, +X points
   * forward, +Y projects left, and +Z projects up. Mouse handling clamps pitch to +/-89 degrees
   * to keep the view direction away from the up vector. Returns a Qt column-major matrix
   * suitable for the OpenGL model-view matrix.
   */
  QMatrix4x4 viewMatrix() const {
    constexpr float radians = 3.14159265358979323846f / 180.f;
    float azimuth = yaw * radians, elevation = pitch * radians;
    // At yaw=pitch=0: eye on -X, looking along +X, with +Z up and +Y left.
    // 中文：yaw 與 pitch 都為零時，相機位於 -X 側、朝 +X 看，+Z 向上且 +Y 向左。
    QVector3D offset(-std::cos(elevation) * std::cos(azimuth),
                     -std::cos(elevation) * std::sin(azimuth),
                     std::sin(elevation));
    QMatrix4x4 view;
    view.lookAt(center + offset * distance, center, QVector3D(0, 0, 1));
    return view;
  }
  /**
   * @brief 在 Qt 建立有效 OpenGL context 後設定深度測試與背景色。
   *
   * 由 Qt 在需要初始化 context 時呼叫，不應由外部手動呼叫。
   *
   * @par English
   * Enable depth testing and set the background color after Qt creates a valid OpenGL context.
   * Qt invokes this when context initialization is needed; callers should not invoke it
   * manually.
   */
  void initializeGL() override {
    glEnable(GL_DEPTH_TEST);
    glClearColor(.035f, .05f, .075f, 1);
  }
  /**
   * @brief 繪製點雲、固定 map 軸、光達位姿／軌跡與文字。
   *
   * 依序建立透視與觀察矩陣、繪製 XYZ 點、繪製 map RGB 軸，再以 overlay 顯示
   * 光達位置和截至配對樣本的軌跡。光達 overlay
   * 暫停深度測試，讓位置不被密集點雲遮住。 QPainter 最後處理文字；所有 OpenGL
   * 呼叫均在 Qt 的有效 context 內執行。
   *
   * @par English
   * Draw cloud points, fixed map axes, LiDAR pose/trajectory, and text. Build projection and
   * view matrices, draw XYZ points and map RGB axes, then overlay the current pose and
   * trajectory through the matched sample. Temporarily disable depth testing for LiDAR overlays
   * so dense clouds do not hide them. QPainter draws text last. All OpenGL calls execute inside
   * Qt's valid context.
   */
  void paintGL() override {
    ++paintCount;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    QMatrix4x4 projection;
    projection.perspective(45, float(width()) / std::max(1, height()),
                           std::max(distance / 10000, .001f),
                           std::max(distance + radius * 4, 10.f));
    QMatrix4x4 model = viewMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection.constData());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(model.constData());
    glPointSize(2);
    const size_t displayLimit =
        playbackActive ? std::min(maxPoints, size_t(200000)) : maxPoints;
    size_t stride = std::max(size_t(1), (points().size() + displayLimit - 1) /
                                            displayLimit);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points().size(); i += stride) {
      auto p = points()[i];
      float t = bag::heightColorScale.value(p.z);
      if (colorMode == 1)
        glColor3f(.4f, .85f, 1.f);
      else if (colorMode == 2) {
        const auto rgb = bag::intensityColor(
            std::isfinite(p.intensity) && intensityScale.finiteCount
                ? intensityScale.value(p.intensity)
                : std::numeric_limits<float>::quiet_NaN());
        glColor3f(rgb[0], rgb[1], rgb[2]);
      } else
        glColor3f(t, .25f + .65f * (1 - std::abs(2 * t - 1)), 1 - t);
      glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    glBegin(GL_LINES);
    float len = radius * .25f;
    glColor3f(1, .2f, .2f);
    glVertex3f(0, 0, 0);
    glVertex3f(len, 0, 0);
    glColor3f(.2f, 1, .2f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, len, 0);
    glColor3f(.3f, .5f, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, len);
    glEnd();
    // Overlay trajectory and pose so they stay visible within dense accumulated
    // clouds.
    // 中文：將軌跡與位姿疊在點雲上，使它們在密集累計點雲中仍然 可見。
    if (track && poseIndex) {
      const auto &pose = track->poses()[*poseIndex];
      glDisable(GL_DEPTH_TEST);
      if (showTrajectory) {
        glLineWidth(2);
        glColor3f(1, .7f, .1f);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i <= *poseIndex; ++i) {
          const auto &p = track->poses()[i];
          if (p.frame != cloud.frame || p.childFrame != pose.childFrame) {
            glEnd();
            glBegin(GL_LINE_STRIP);
            continue;
          }
          glVertex3dv(p.position.data());
        }
        glEnd();
        glLineWidth(1);
      }
      if (showPose) {
        glPointSize(12);
        glColor3f(1, .15f, .8f);
        glBegin(GL_POINTS);
        glVertex3dv(pose.position.data());
        glEnd();
        QMatrix4x4 transform;
        transform.translate(float(pose.position[0]), float(pose.position[1]),
                            float(pose.position[2]));
        auto q = pose.orientation;
        transform.rotate(
            QQuaternion(float(q[3]), float(q[0]), float(q[1]), float(q[2])));
        glPushMatrix();
        glMultMatrixf(transform.constData());
        glLineWidth(3);
        float length = std::clamp(radius * .025f, .3f, 3.f);
        glBegin(GL_LINES);
        glColor3f(1, .2f, .2f);
        glVertex3f(0, 0, 0);
        glVertex3f(length, 0, 0);
        glColor3f(.2f, 1, .2f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, length, 0);
        glColor3f(.3f, .5f, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, length);
        // Arrow head for the sensor's +X direction.
        // 中文：感測器 +X 方向的箭頭頭部。
        glColor3f(1, .2f, .2f);
        glVertex3f(length, 0, 0);
        glVertex3f(length * .75f, length * .12f, 0);
        glVertex3f(length, 0, 0);
        glVertex3f(length * .75f, -length * .12f, 0);
        glEnd();
        glLineWidth(1);
        glPopMatrix();
      }
      glEnable(GL_DEPTH_TEST);
    }
    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.drawText(
        16, 24,
        "Left drag: orbit | Right drag: pan | Wheel: zoom | XYZ axes: RGB");
    painter.drawText(16, 44,
                     QString("%1 valid points | display stride %2 | %3")
                         .arg(qulonglong(points().size()))
                         .arg(qulonglong(stride))
                         .arg(QString::fromStdString(cloud.frame)));
    if (track && poseIndex && showPose) {
      const auto &p = track->poses()[*poseIndex];
      auto clip = projection * model *
                  QVector4D(float(p.position[0]), float(p.position[1]),
                            float(p.position[2]), 1);
      if (clip.w() > 0) {
        auto ndc = clip.toVector3DAffine();
        if (std::abs(ndc.x()) < 1 && std::abs(ndc.y()) < 1 &&
            std::abs(ndc.z()) <= 1) {
          painter.setPen(QColor(255, 100, 220));
          painter.drawText(QPointF((ndc.x() + 1) * width() * .5 + 12,
                                   (1 - ndc.y()) * height() * .5 - 10),
                           "LiDAR: " + QString::fromStdString(p.childFrame));
        }
      }
    }
    painter.setPen(QColor(255, 190, 65));
    painter.drawText(
        16, 64, "LiDAR: magenta marker + local XYZ axes | Trajectory: gold");
    if (colorMode == 2) {
      painter.setPen(Qt::white);
      painter.drawText(16, 84,
                       intensityScale.finiteCount
                           ? QString("Intensity: blue (low) -> teal -> yellow -> red (high) | "
                                     "P1-P99: %1 .. %2 | missing = gray")
                                 .arg(intensityScale.low)
                                 .arg(intensityScale.high)
                           : QString("Intensity unavailable: no finite "
                                     "intensity values; showing gray"));
    }
  }
  /**
   * @brief 記錄拖曳起點以計算下一次滑鼠移動量。
   *
   * @param e Qt 提供的滑鼠事件，不移轉所有權。
   *
   * @par English
   * Remember the drag origin for the next mouse delta. Parameter e is a Qt-owned mouse event;
   * ownership is not transferred.
   */
  void mousePressEvent(QMouseEvent *e) override { last = e->pos(); }
  /**
   * @brief 左鍵 orbit、右鍵依畫面平面平移相機觀察中心。
   *
   * 右鍵平移先將相機空間位移以 viewMatrix() 的逆矩陣 mapVector 回世界空間，
   * 因此平移方向會跟隨目前視角，且不受矩陣平移分量影響。
   *
   * @param e 包含按鍵狀態及目前位置的 Qt 事件。
   *
   * @par English
   * Orbit with the left button and pan the observation center in the screen plane with the right
   * button. Convert camera-space motion to world space using the inverse viewMatrix() with
   * mapVector, so panning follows the view orientation without including matrix translation.
   * Parameter e contains button state and current position.
   */
  void mouseMoveEvent(QMouseEvent *e) override {
    auto d = e->pos() - last;
    last = e->pos();
    if (e->buttons() & Qt::LeftButton) {
      yaw += d.x() * .4f;
      pitch = std::clamp(pitch + d.y() * .4f, -89.f, 89.f);
    }
    if (e->buttons() & Qt::RightButton) {
      auto v = viewMatrix().inverted().mapVector(QVector3D(-d.x(), d.y(), 0) *
                                                 distance /
                                                 float(std::max(1, height())));
      center += v;
    }
    update();
  }
  /**
   * @brief 依滾輪增量以指數比例縮放觀察距離。
   *
   * 距離限制為 0.01 到 1e8，避免零距離與無限放大。
   *
   * @param e Qt 滾輪事件；使用 angleDelta().y()。
   *
   * @par English
   * Zoom the observation distance exponentially from wheel input. Clamp distance to 0.01 through
   * 1e8 to prevent zero distance or unbounded zoom. Parameter e is the Qt wheel event; use
   * angleDelta().y().
   */
  void wheelEvent(QWheelEvent *e) override {
    distance = std::clamp(
        distance * float(std::exp(-e->angleDelta().y() * .001)), .01f, 1e8f);
    update();
  }
};
/**
 * @struct Options
 * @brief 命令列解析後的完整執行設定。
 *
 * 所有幀索引從零開始；空輸出路徑表示不執行該種匯出。
 * info、validate 或任一匯出通常走無視窗流程；指定 screenshot 時仍建立視窗。
 *
 * @par English
 * Parsed command-line execution settings. Frame indices are zero-based and empty output paths
 * disable their exports. info, validate, and exports normally select headless operation; a
 * screenshot request still creates a window.
 */
/**
 * @var std::string Options::topic
 * @brief 點雲 topic，預設 /cloud_registered。
 *
 * @par English
 * Point cloud topic, defaulting to /cloud_registered.
 */
/**
 * @var std::string Options::ply
 * @brief 單幀 ASCII PLY 目的路徑，空字串表示不匯出。
 *
 * @par English
 * Single-frame ASCII PLY path; empty disables this export.
 */
/**
 * @var std::string Options::screenshot
 * @brief 初始 OpenGL 畫面的 PNG 路徑。
 *
 * @par English
 * PNG path for the initial OpenGL view.
 */
/**
 * @var std::string Options::exportAll
 * @brief 完整 5 公分體素 binary PLY 路徑。
 *
 * @par English
 * Binary PLY destination for the complete 5 cm voxel map.
 */
/**
 * @var std::string Options::exportRaw
 * @brief 不降取樣的全部原始有效 XYZ binary PLY 路徑。
 *
 * @par English
 * Binary PLY destination for all original valid XYZ points without downsampling, including
 * intensity.
 */
/**
 * @var std::string Options::odometryTopic
 * @brief nav_msgs/Odometry topic，預設 /Odometry。
 *
 * @par English
 * nav_msgs/Odometry topic, defaulting to /Odometry.
 */
/**
 * @var size_t Options::maxPoints
 * @brief 顯示點數上限，必須大於零；不影響完整匯出。
 *
 * @par English
 * Positive display point limit; complete export is unaffected.
 */
/**
 * @var bool Options::validate
 * @brief 是否解碼全部點雲並檢查 Odometry 配對統計。
 *
 * @par English
 * Whether to decode every cloud and collect odometry matching statistics.
 */
/**
 * @var bool Options::accumulate
 * @brief 初始是否顯示累計 5 公分體素地圖。
 *
 * @par English
 * Whether to start in accumulated 5 cm voxel map mode.
 */
/**
 * @var bool Options::all
 * @brief 是否將初始幀設為最後一幀並顯示完整累計地圖。
 *
 * @par English
 * Whether to select the last frame initially and display the whole accumulated map.
 */
/**
 * @var bool Options::odometry
 * @brief 是否建立並載入 Odometry 索引與軌跡。
 *
 * @par English
 * Whether to index and load odometry and its trajectory.
 */
struct Options {
  /**
   * @brief 輸入／輸出路徑及兩種 topic 名稱。
   *
   * path 為 bag；topic 為點雲；ply 為單幀 ASCII 匯出；screenshot 為 PNG；
   * exportAll 為完整體素 PLY；exportRaw 為全部原始 XYZ；odometryTopic
   * 為位姿來源。
   *
   * @par English
   * Input/output paths and the two topic names. path is the bag, topic is the cloud source, ply
   * is single-frame ASCII export, screenshot is PNG output, exportAll is the complete voxel PLY,
   * exportRaw is all original XYZ with intensity, and odometryTopic is the pose source.
   */
  std::string path, topic = "/cloud_registered", ply, screenshot, exportAll,
                    exportRaw, odometryTopic = "/Odometry";
  /**
   * @brief 初始幀與顯示點數上限。
   *
   * frame 是零起始索引；maxPoints
   * 必須大於零，只限制顯示、不限制完整地圖或匯出。
   *
   * @par English
   * Initial frame and display point limit. frame is zero-based; maxPoints must be positive and
   * affects neither the complete map nor exports.
   */
  size_t frame = 0, maxPoints = 1000000;
  double initialRate = 1;      ///< 初始播放倍率。
  ///< EN: Initial playback rate.
  double benchmarkSeconds = 0; ///< 播放量測秒數，零表示不自動退出。
  ///< EN: Playback measurement duration in seconds; zero disables automatic exit.
  bool autoPlay = false; ///< 是否在初始畫面完成後自動播放。
  ///< EN: Whether to begin playback after presenting the initial view.
  int color = 0; ///< 初始上色：0 height、1 single、2 intensity。
  ///< EN: Initial color mode: 0 height, 1 single, 2 intensity.
  /**
   * @brief 無視窗操作與視窗初始模式的旗標。
   *
   * info 顯示資訊、validate 完整驗證、accumulate 啟用累計、all 選最後一幀，
   * odometry 決定是否載入位置與軌跡。
   *
   * @par English
   * Flags for headless actions and initial viewer modes. info prints metadata, validate performs
   * full validation, accumulate enables the voxel map, all selects the last frame, and odometry
   * controls pose/trajectory loading.
   */
  bool info = false, validate = false, accumulate = true, all = false,
       odometry = true;
};
/**
 * @brief 解析命令列的非負整數，拒絕負號與未消耗的尾端字元。
 *
 * @param s 要傳給 std::stoull 的字串。
 * @return 轉為 size_t 的整數值。
 * @throws std::runtime_error 空字串、負數或尾端有其他字元。
 * @throws std::invalid_argument 無法轉為整數。
 * @throws std::out_of_range 超出 unsigned long long 範圍。
 *
 * @par English
 * Parse a nonnegative command-line integer, rejecting a minus sign and unconsumed trailing
 * characters. Parameter s is passed to std::stoull. Returns the value converted to size_t.
 * Throws std::runtime_error for empty/negative input or trailing characters,
 * std::invalid_argument when conversion is impossible, or std::out_of_range beyond unsigned
 * long long.
 */
size_t number(const std::string &s) {
  if (s.empty() || s[0] == '-')
    throw std::runtime_error("Expected nonnegative integer");
  size_t used = 0;
  auto n = std::stoull(s, &used);
  if (used != s.size())
    throw std::runtime_error("Invalid integer: " + s);
  return n;
}
/**
 * @brief 解析 CLI、執行無視窗操作，或建立並執行互動式視窗。
 *
 * 流程為：解析參數、建立 Reader 與 OdometryTrack、印出／驗證／匯出，最後才建立
 * Qt。播放以單調實際時間為錨點，解碼與繪圖耗時不再額外加進每一幀等待。
 * 過期的中間畫面合併更新，累計仍讀入所有中間幀。長時間累計與原始匯出使用 modal
 * 進度視窗支援取消。 各 signal callback 以參考捕捉區域物件，生命週期涵蓋
 * app.exec()；所有例外由 適當的 UI 邊界或最外層 catch 顯示並轉為非零結束碼。
 *
 * @param argc 命令列參數數量。
 * @param argv 命令列參數字串陣列。
 * @return 成功或 --help 為 0；解析、讀寫或視覺化驗證失敗為非零。
 *
 * @par English
 * Parse the CLI, execute headless actions, or create and run the interactive window. Parse
 * arguments, construct Reader and OdometryTrack, inspect/validate/export, then create Qt.
 * Monotonic wall time anchors playback, preventing decoding/drawing cost from adding to each
 * frame interval. Combine late display updates while still accumulating every intermediate
 * frame. Long accumulation and original exports use modal progress dialogs with cancellation.
 * Signal callbacks capture locals by reference; those objects outlive app.exec(). UI
 * boundaries or the outer catch report exceptions and return failure. Parameters: argc is the
 * argument count; argv is the argument array. Returns zero for success/help, or nonzero for
 * parsing, I/O, or visualization failure.
 */
int main(int argc, char **argv) {
  try {
    Options o;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      /**
       * @brief 取出目前選項的下一個 argv 值，同時推進參數索引。
       *
       * 若參數尾端缺少值則立即拋出錯誤，避免使用越界 argv。
       *
       * @par English
       * Fetch the argument after the current option and advance the argument index. Throw
       * immediately for a missing value rather than indexing beyond argv.
       */
      auto value = [&]() {
        if (++i >= argc)
          throw std::runtime_error("Missing value for " + a);
        return std::string(argv[i]);
      };
      if (a == "--help" || a == "-h") {
        std::cout
            << "Usage: bag_cloud_viewer FILE.bag [options]\n"
               "  --topic NAME          Topic (default /cloud_registered)\n"
               "  --odom-topic NAME     Odometry topic (default /Odometry)\n"
               "  --no-odometry         Disable pose and trajectory loading\n"
               "  --color MODE          height, single, intensity (default "
               "height)\n"
               "  --playback-rate RATE  Select speed and start playback "
               "(0.25..16)\n"
               "  --benchmark-seconds N Exit after N seconds and report actual "
               "speed\n"
               "  --frame N             Frame index, starting at zero\n"
               "  --all                 Display accumulation of every frame\n"
               "  --single-frame        Display only the selected frame\n"
               "  --info                Inspect without a display\n"
               "  --validate            Decode every frame without a display\n"
               "  --export FILE.ply     Export selected original frame (ASCII "
               "XYZ + intensity)\n"
               "  --export-all FILE.ply Export all frames, 5 cm voxel "
               "centers (binary XYZ + intensity)\n"
               "  --export-all-raw FILE.ply  Stream all original valid XYZ "
               "points + intensity (binary)\n"
               "  --max-points N        Maximum displayed points (default "
               "1000000)\n"
               "  --screenshot FILE.png Save initial 3D view and exit\n"
               "Default view: accumulated frames 0..N, downsampled to 5 cm "
               "voxels.\n";
        return 0;
      } else if (a == "--topic")
        o.topic = value();
      else if (a == "--odom-topic")
        o.odometryTopic = value();
      else if (a == "--no-odometry")
        o.odometry = false;
      else if (a == "--color") {
        auto mode = value();
        if (mode == "height")
          o.color = 0;
        else if (mode == "single")
          o.color = 1;
        else if (mode == "intensity")
          o.color = 2;
        else
          throw std::runtime_error("Unknown color mode: " + mode);
      } else if (a == "--playback-rate" || a == "--benchmark-seconds") {
        auto input = value();
        size_t used = 0;
        double n = std::stod(input, &used);
        if (used != input.size() || !std::isfinite(n) || n <= 0)
          throw std::runtime_error("Invalid value for " + a);
        if (a == "--playback-rate") {
          if (std::find(playbackRates.begin(), playbackRates.end(), n) ==
              playbackRates.end())
            throw std::runtime_error("Unsupported playback rate");
          o.initialRate = n;
          o.autoPlay = true;
        } else {
          if (n > 3600)
            throw std::runtime_error("Benchmark duration must be <= 3600 s");
          o.benchmarkSeconds = n;
        }
      } else if (a == "--frame")
        o.frame = number(value());
      else if (a == "--max-points")
        o.maxPoints = number(value());
      else if (a == "--export")
        o.ply = value();
      else if (a == "--export-all")
        o.exportAll = value();
      else if (a == "--export-all-raw")
        o.exportRaw = value();
      else if (a == "--all")
        o.all = true;
      else if (a == "--single-frame")
        o.accumulate = false;
      else if (a == "--screenshot")
        o.screenshot = value();
      else if (a == "--info")
        o.info = true;
      else if (a == "--validate")
        o.validate = true;
      else if (!a.empty() && a[0] == '-')
        throw std::runtime_error("Unknown option: " + a);
      else if (o.path.empty())
        o.path = a;
      else
        throw std::runtime_error("Unexpected argument: " + a);
    }
    if (o.path.empty() || !o.maxPoints)
      throw std::runtime_error(
          "Provide a bag file and positive --max-points; see --help");
    if (o.benchmarkSeconds > 0 && !o.autoPlay)
      throw std::runtime_error("--benchmark-seconds requires --playback-rate");
    bag::Reader reader(o.path, o.topic, o.odometry ? o.odometryTopic : "");
    if (o.all) {
      if (!o.accumulate)
        throw std::runtime_error("--all requires accumulated view");
      o.frame = reader.frames.size() - 1;
    }
    /**
     * @brief 拒絕輸出到輸入 bag 本身，包含既存硬連結或符號連結別名。
     *
     * 僅在目標已存在時以 filesystem::equivalent 比對，讓新檔案可以正常建立。
     *
     * @par English
     * Reject output that is the input bag itself, including existing hard-link or symbolic-link
     * aliases. Check filesystem::equivalent only for existing destinations so new files remain
     * allowed.
     */
    auto protectInput = [&](const std::string &output) {
      if (!output.empty() && std::filesystem::exists(output) &&
          std::filesystem::equivalent(o.path, output))
        throw std::runtime_error("Output must not overwrite the input bag");
    };
    for (const auto &path : {o.ply, o.exportAll, o.exportRaw, o.screenshot})
      protectInput(path);
    bag::Accumulation accumulation(reader, 0.05, o.maxPoints);
    auto initial = reader.read(o.frame);
    bag::OdometryTrack track(reader);
    auto initialPose = track.match(initial.stamp, initial.frame);
    std::cout << std::setprecision(16) << "Topic: " << o.topic
              << "\nFrames: " << reader.frames.size() << "\nBag time span: "
              << reader.frames.back().time - reader.frames.front().time
              << " s\nFrame: " << o.frame << "\nStamp: " << initial.stamp
              << "\nFrame ID: " << initial.frame
              << "\nDimensions: " << initial.width << " x " << initial.height
              << "\nPoint step: " << initial.pointStep
              << "\nValid points: " << initial.points.size() << '\n';
    bag::IntensityScale initialIntensity;
    initialIntensity.update(initial.points);
    std::cout << "Intensity field: "
              << (initial.hasIntensity ? "present" : "absent")
              << "\nFinite intensities: " << initialIntensity.finiteCount
              << "\nIntensity min/max: " << initialIntensity.minimum << " / "
              << initialIntensity.maximum << '\n';
    std::cout << "Odometry messages: " << track.poses().size() << '\n';
    if (initialPose) {
      const auto &p = track.poses()[*initialPose];
      std::cout << "Odometry frames: " << p.frame << " -> " << p.childFrame
                << "\nLiDAR XYZ: " << p.position[0] << ' ' << p.position[1]
                << ' ' << p.position[2]
                << "\nOdometry time delta: " << p.stamp - initial.stamp
                << " s\n";
    } else if (o.odometry)
      std::cerr << "No matching odometry for selected cloud (same frame, "
                   "within 0.2 s required)\n";
    if (o.validate) {
      uint64_t total = 0, matched = 0;
      double maxDelta = 0;
      for (size_t i = 0; i < reader.frames.size(); ++i) {
        try {
          auto c = reader.read(i);
          total += c.points.size();
          auto pose = track.match(c.stamp, c.frame);
          if (pose) {
            ++matched;
            maxDelta = std::max(maxDelta,
                                std::abs(track.poses()[*pose].stamp - c.stamp));
          }
        } catch (const std::exception &e) {
          throw std::runtime_error("Frame " + std::to_string(i) + ": " +
                                   e.what());
        }
      }
      std::cout << "Validated frames: " << reader.frames.size()
                << "\nTotal valid points: " << total << "\n"
                << "Clouds matched to odometry: " << matched
                << "\nMaximum odometry time delta: " << maxDelta << " s\n";
    }
    if (!o.ply.empty())
      bag::writePly(o.ply, initial);
    if (!o.exportAll.empty()) {
      accumulation.seek(reader.frames.size() - 1);
      bag::writeVoxelPly(o.exportAll, accumulation.grid);
      std::cout << "Accumulated input points: "
                << accumulation.grid.inputPoints()
                << "\n5 cm voxels: " << accumulation.grid.size()
                << "\nExported: " << o.exportAll << '\n';
    }
    if (!o.exportRaw.empty()) {
      bag::exportRawPly(reader, o.exportRaw);
      std::cout << "Exported all original valid XYZ + intensity points: "
                << o.exportRaw << '\n';
    }
    if ((o.info || o.validate || !o.ply.empty() || !o.exportAll.empty() ||
         !o.exportRaw.empty()) &&
        o.screenshot.empty())
      return 0;
    if (reader.frames.size() > size_t(std::numeric_limits<int>::max()))
      throw std::runtime_error("Too many frames for GUI slider");
    QSurfaceFormat fmt;
    fmt.setVersion(2, 1);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    fmt.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(fmt);
    // Parse our CLI first; do not pass custom switches to Qt.
    // 中文：先解析本程式的 CLI，避免將自訂選項交給 Qt。
    int qtArgc = 1;
    QApplication app(qtArgc, argv);
    QMainWindow window;
    window.setWindowTitle("ROS-free PointCloud2 bag viewer");
    auto root = new QWidget;
    auto layout = new QVBoxLayout(root);
    auto view = new View;
    view->track = &track;
    view->colorMode = o.color;
    view->maxPoints = o.maxPoints;
    view->cloud = std::move(initial);
    view->syncPose();
    view->resetCamera();
    layout->addWidget(view, 1);
    auto accumulated = new QCheckBox("Accumulate (5 cm)");
    accumulated->setChecked(o.accumulate);
    auto loadAll = new QPushButton("Load all frames");
    auto exportAll = new QPushButton("Export all frames...");
    auto exportMode = new QComboBox;
    exportMode->addItems(
        {"5 cm voxel centers", "All original XYZ + intensity"});
    auto showPose = new QCheckBox("LiDAR pose");
    showPose->setChecked(true);
    auto showTrajectory = new QCheckBox("Trajectory");
    showTrajectory->setChecked(true);
    auto poseRow = new QHBoxLayout;
    poseRow->addWidget(showPose);
    poseRow->addWidget(showTrajectory);
    poseRow->addStretch();
    layout->addLayout(poseRow);
    auto mapRow = new QHBoxLayout;
    for (auto w :
         std::vector<QWidget *>{accumulated, loadAll, exportMode, exportAll})
      mapRow->addWidget(w);
    layout->addLayout(mapRow);
    auto row = new QHBoxLayout;
    auto play = new QPushButton("Play");
    auto loop = new QCheckBox("Loop playback");
    loop->setChecked(true);
    auto prev = new QPushButton("Previous");
    /** @brief 同步模式相關按鈕；累計模式停用 Previous，單幀模式恢復啟用。
     *
     * @par English
     * Synchronize mode-dependent controls: disable Previous for accumulation and enable it for
     * single-frame viewing.
     */
    auto syncModeControls = [&]() { prev->setEnabled(!accumulated->isChecked()); };
    syncModeControls();
    auto next = new QPushButton("Next");
    auto reset = new QPushButton("Fit view");
    auto follow = new QCheckBox("Fit every frame");
    auto color = new QComboBox;
    color->addItems({"Height color", "Single color", "Intensity (color)"});
    color->setCurrentIndex(o.color);
    auto speed = new QComboBox;
    for (double rate : playbackRates)
      speed->addItem(QString::number(rate) + "x");
    speed->setCurrentIndex(int(
        std::find(playbackRates.begin(), playbackRates.end(), o.initialRate) -
        playbackRates.begin()));
    for (auto w :
         std::vector<QWidget *>{play, prev, next, loop, reset, follow, color, speed})
      row->addWidget(w);
    layout->addLayout(row);
    auto slider = new QSlider(Qt::Horizontal);
    slider->setTracking(false);
    slider->setRange(0, int(reader.frames.size() - 1));
    slider->setValue(int(o.frame));
    layout->addWidget(slider);
    auto status = new QLabel;
    status->setWordWrap(true);
    layout->addWidget(status);
    window.setCentralWidget(root);
    window.resize(1100, 800);
    QTimer timer;
    timer.setSingleShot(true);
    timer.setTimerType(Qt::PreciseTimer);
    QElapsedTimer monotonic;
    monotonic.start();
    auto wallSeconds = [&]() { return monotonic.nsecsElapsed() * 1e-9; };
    bag::PlaybackClock playbackClock;
    double lastPresentation = 0, lastIntensityUpdate = -1e20, rateWallBase = 0,
           rateBagBase = 0;
    double loopBagAdvance = 0; ///< 循環跨越的 bag 秒數，供效能量測使用。
    ///< EN: Bag seconds crossed by loops, used by benchmark measurements.
    bool playing = false;
    bool failed = false;
    /**
     * @brief 僅強度模式需要百分位數；播放時限量取樣並最多每 0.2 秒更新。
     *
     * 暫停或切換模式可強制重算完整顯示資料，避免高度上色仍花時間掃描強度。
     *
     * @par English
     * Only intensity mode needs percentile statistics; bound samples during playback and refresh
     * at most every 0.2 seconds. Pausing or changing mode may force a complete display-statistics
     * refresh, avoiding intensity scans while coloring by height.
     */
    auto refreshIntensity = [&](bool force) {
      double now = wallSeconds();
      if (view->colorMode == 2 && (force || now - lastIntensityUpdate >= .2)) {
        view->intensityScale.update(view->points(),
                                    playing ? size_t(65536) : 0);
        lastIntensityUpdate = now;
      }
    };
    /**
     * @brief 更新幀號、相對 bag 時間、訊息時間、顯示量與光達座標。
     *
     * 體素總數與取樣顯示點數分別呈現，避免將畫面上限誤認為匯出上限。
     *
     * @par English
     * Update the frame index, relative bag time, message time, display counts, and LiDAR
     * coordinates. Show the complete voxel count separately from sampled display points so display
     * limits are not mistaken for export limits.
     */
    auto showStatus = [&]() {
      size_t i = slider->value();
      status->setText(
          QString("Frame %1 / %2 | t = %3 s | stamp = %4 | %5 displayed-source "
                  "points")
              .arg(qulonglong(i))
              .arg(qulonglong(reader.frames.size() - 1))
              .arg(reader.frames[i].time - reader.frames.front().time, 0, 'f',
                   3)
              .arg(view->cloud.stamp, 0, 'f', 6)
              .arg(qulonglong(view->points().size())) +
          (accumulated->isChecked()
               ? QString(" | %1 total voxels (5 cm) | %2 input points")
                     .arg(qulonglong(accumulation.grid.size()))
                     .arg(qulonglong(accumulation.grid.inputPoints()))
               : QString(" | single frame")));
      if (playing) {
        double now = wallSeconds(), wall = now - rateWallBase;
        double actual =
            wall > 0 ? (reader.frames[i].time - rateBagBase) / wall : 0;
        double lag =
            std::max(0., playbackClock.targetTime(now) - reader.frames[i].time);
        status->setText(status->text() +
                        QString(" | speed %1x / %2x | lag %3 s")
                            .arg(actual, 0, 'f', 2)
                            .arg(playbackClock.rate())
                            .arg(lag, 0, 'f', 3));
      }
      if (view->poseIndex) {
        const auto &p = track.poses()[*view->poseIndex];
        status->setText(status->text() +
                        QString(" | LiDAR XYZ: (%1, %2, %3) m | dt %4 s")
                            .arg(p.position[0], 0, 'f', 3)
                            .arg(p.position[1], 0, 'f', 3)
                            .arg(p.position[2], 0, 'f', 3)
                            .arg(p.stamp - view->cloud.stamp, 0, 'f', 4));
      } else
        status->setText(status->text() +
                        " | LiDAR pose unavailable (frame/time mismatch or "
                        "missing odometry)");
    };
    /**
     * @brief 停止播放旗標與 timer，並將播放按鈕回復為 Play。
     *
     * @par English
     * Clear the playing flag and timer, and restore the Play label.
     */
    auto stop = [&]() {
      playing = false;
      timer.stop();
      play->setText("Play");
      view->playbackActive = false;
      refreshIntensity(true);
      view->update();
    };
    /**
     * @brief 依絕對播放時間與 30 Hz 畫面上限排程，將已消耗處理時間扣除。
     *
     * 最後一幀依 checkbox 決定停止或延後循環，避免單幀資料造成忙迴圈。
     *
     * @par English
     * Schedule against absolute playback time and a 30 Hz display cap, deducting time already
     * spent processing. At the final frame, stop or defer looping according to the checkbox;
     * deferred callbacks also prevent a busy loop for single-frame recordings.
     */
    auto schedule = [&]() {
      if (!playing)
        return;
      size_t i = size_t(slider->value());
      if (i + 1 >= reader.frames.size()) {
        if (loop->isChecked())
          timer.start(34);
        else
          stop();
        return;
      }
      double now = wallSeconds();
      double dataWait = std::max(
          0., (reader.frames[i + 1].time - playbackClock.targetTime(now)) /
                  playbackClock.rate());
      double displayWait = std::max(0., 1. / 30 - (now - lastPresentation));
      double milliseconds = std::ceil(std::max(dataWait, displayWait) * 1000);
      timer.start(int(std::clamp(milliseconds, 1.,
                                 double(std::numeric_limits<int>::max()))));
    };
    /**
     * @brief 將累計地圖載入至指定幀，並對齊滑桿、cloud 與位姿。
     *
     * 取消後仍以 loadedFrames()-1 作為實際幀；使用 QSignalBlocker 更新滑桿，
     * 避免程式內部校正再次觸發重入的讀取流程。進度每十幀或最後一幀更新一次。
     *
     * @par English
     * Load the accumulated map through a target and synchronize the slider, cloud, and pose. After
     * cancellation, use loadedFrames()-1 as the actual frame. Block slider signals during
     * correction to avoid recursive loading. Interactive progress updates every ten frames or on
     * the final frame.
     */
    auto loadAccumulation = [&](size_t target, bool interactive = true) {
      syncModeControls();
      bool complete = true;
      if (!interactive) {
        // A time-budget yield is not user cancellation: continue on the next
        // timer tick. This bounds UI stalls if rendering or a suspended window
        // creates a large backlog.
        // 中文：因時間預算而讓出執行權不是使用者取消：應在下一次 timer tick 繼續。這能限制繪圖或視窗暫停造成大量積欠時 的 UI 卡頓。
        double started = wallSeconds();
        accumulation.seek(target, [&](size_t done, size_t total) {
          return done == total || wallSeconds() - started < .05;
        });
      } else if (accumulation.loadedFrames() != target + 1) {
        QProgressDialog progress("Accumulating point clouds (5 cm voxels)...",
                                 "Cancel", 0, int(target + 1), &window);
        progress.setWindowModality(Qt::ApplicationModal);
        progress.setMinimumDuration(300);
        complete = accumulation.seek(target, [&](size_t done, size_t total) {
          if (done % 10 == 0 || done == total)
            progress.setValue(int(done));
          return !progress.wasCanceled();
        });
      }
      view->accumulated = &accumulation.grid.points();
      auto actual = accumulation.loadedFrames() - 1;
      {
        QSignalBlocker block(slider);
        slider->setValue(int(actual));
      }
      view->cloud = reader.read(actual);
      view->syncPose();
      refreshIntensity(!playing);
      return complete;
    };
    /**
     * @brief 集中處理單幀與累計模式的讀取、相機更新與錯誤回報。
     *
     * 取消累計會停止播放；例外則保留可用結果、修正滑桿並設定最終失敗狀態。
     *
     * @par English
     * Handle reading, camera refresh, and errors for both single-frame and accumulated modes.
     * Canceling accumulation stops playback; exceptions retain usable results, correct the slider,
     * and set the final failure status.
     */
    auto loadFrame = [&](size_t i, bool interactive = true) {
      syncModeControls();
      try {
        if (accumulated->isChecked()) {
          if (!loadAccumulation(i, interactive))
            stop();
        } else {
          view->accumulated = nullptr;
          view->cloud = reader.read(i);
          view->syncPose();
          refreshIntensity(!playing);
        }
        if (follow->isChecked())
          view->resetCamera();
        view->update();
        showStatus();
      } catch (const std::exception &e) {
        stop();
        // A failed/cancelled seek must not leave a misleading slider position.
        // 中文：定位失敗或取消後，不可讓滑桿停留在與實際結果不同的位置。
        if (accumulated->isChecked() && accumulation.loadedFrames()) {
          QSignalBlocker block(slider);
          slider->setValue(int(accumulation.loadedFrames() - 1));
          view->accumulated = &accumulation.grid.points();
          view->update();
        }
        status->setText(QString("Read error: ") + e.what());
        failed = true;
      }
    };
    /**
     * @brief 滑桿變更先停用既有 timer，讀取完成後才重新排程，避免 modal
     * 進度造成重入。
     *
     * @par English
     * Stop the existing timer before a slider load and reschedule afterward to avoid reentrant
     * processing through modal progress updates.
     */
    QObject::connect(slider, &QSlider::valueChanged, [&](int i) {
      timer.stop();
      loadFrame(size_t(i));
      if (playing) {
        double now = wallSeconds();
        playbackClock.reset(now, reader.frames[size_t(slider->value())].time,
                            playbackRates[size_t(speed->currentIndex())]);
        rateWallBase = lastPresentation = now;
        rateBagBase = reader.frames[size_t(slider->value())].time;
      }
      schedule();
    });
    /**
     * @brief 切換累計／單幀模式時停止播放、重讀目前幀並重新擬合。
     *
     * @par English
     * Stop playback, reload the current frame, and refit when switching accumulation/single-frame
     * mode.
     */
    QObject::connect(accumulated, &QCheckBox::toggled, [&](bool) {
      stop();
      loadFrame(size_t(slider->value()));
      view->resetCamera();
    });
    /**
     * @brief 切到累計模式並載入最後一幀；封鎖內部 UI 校正造成的重複 signals。
     *
     * @par English
     * Enable accumulation and load the final frame, blocking duplicate signals from internal UI
     * adjustments.
     */
    QObject::connect(loadAll, &QPushButton::clicked, [&]() {
      stop();
      {
        QSignalBlocker block(accumulated);
        accumulated->setChecked(true);
      }
      {
        QSignalBlocker block(slider);
        slider->setValue(slider->maximum());
      }
      loadFrame(reader.frames.size() - 1);
      view->resetCamera();
    });
    /**
     * @brief 依匯出模式選擇完整體素或原始 XYZ，並保護輸入檔。
     *
     * 完整體素匯出共用既有地圖以避免雙份累計；原始匯出則逐幀寫入，取消不提交目的檔。
     *
     * @par English
     * Export either the complete voxel map or original XYZ with intensity, protecting the input
     * file. Voxel export reuses the existing map to avoid a second copy; original export streams
     * frame by frame and does not commit on cancellation.
     */
    QObject::connect(exportAll, &QPushButton::clicked, [&]() {
      stop();
      auto path = QFileDialog::getSaveFileName(
          &window, "Export every frame as one PLY",
          exportMode->currentIndex() == 0 ? "all_clouds_5cm.ply"
                                          : "all_clouds_raw.ply",
          "PLY point cloud (*.ply)");
      if (path.isEmpty())
        return;
      try {
        auto output = path.toStdString();
        protectInput(output);
        bool complete;
        if (exportMode->currentIndex() == 0) {
          {
            QSignalBlocker block(accumulated);
            accumulated->setChecked(true);
          }
          complete = loadAccumulation(reader.frames.size() - 1);
          view->resetCamera();
          view->update();
          showStatus();
          if (complete)
            bag::writeVoxelPly(output, accumulation.grid);
        } else {
          QProgressDialog progress("Exporting all original XYZ points...",
                                   "Cancel", 0, int(reader.frames.size()),
                                   &window);
          progress.setWindowModality(Qt::ApplicationModal);
          progress.setMinimumDuration(300);
          complete =
              bag::exportRawPly(reader, output, [&](size_t done, size_t total) {
                if (done % 10 == 0 || done == total)
                  progress.setValue(int(done));
                return !progress.wasCanceled();
              });
        }
        status->setText(complete ? "Exported: " + path
                                 : "Export cancelled; destination unchanged.");
      } catch (const std::exception &e) {
        QMessageBox::critical(&window, "Export failed", e.what());
      }
    });
    /**
     * @brief 切換播放／暫停；在最後一幀按 Play 會先回到第零幀。
     *
     * @par English
     * Toggle Play/Pause. Pressing Play on the final frame first returns to frame zero.
     */
    QObject::connect(play, &QPushButton::clicked, [&]() {
      if (playing)
        stop();
      else {
        if (slider->value() == slider->maximum())
          slider->setValue(0);
        playing = true;
        view->playbackActive = true;
        double now = wallSeconds();
        playbackClock.reset(now, reader.frames[size_t(slider->value())].time,
                            playbackRates[size_t(speed->currentIndex())]);
        rateWallBase = lastPresentation = now;
        rateBagBase = reader.frames[size_t(slider->value())].time;
        play->setText("Pause");
        schedule();
      }
    });
    /**
     * @brief 單次 timer
     * 到期後依實際時間跳到最新到期畫面，累計仍補讀所有中間幀。
     *
     * @par English
     * On timer expiry, select the newest frame due by wall time; accumulation still reads every
     * intermediate frame.
     */
    QObject::connect(&timer, &QTimer::timeout, [&]() {
      if (!playing)
        return;
      double now = wallSeconds();
      size_t current = size_t(slider->value());
      if (current + 1 == reader.frames.size()) {
        if (!loop->isChecked()) {
          stop();
          return;
        }
        // 後退定位會清除上一輪累計；讀取完成才錨定，維持所選倍率。
        // EN: Backward seeking clears the previous map; anchor after loading at the selected rate.
        {
          QSignalBlocker block(slider);
          slider->setValue(0);
        }
        loadFrame(0, false);
        if (!playing)
          return;
        loopBagAdvance += reader.frames.back().time - reader.frames.front().time;
        now = wallSeconds();
        playbackClock.restart(now, reader.frames.front().time);
        rateWallBase = lastPresentation = now;
        rateBagBase = reader.frames.front().time;
        showStatus();
        schedule();
        return;
      }
      size_t target =
          std::max(current, playbackClock.dueFrame(reader.frames, now));
      if (target > current) {
        lastPresentation = now;
        {
          QSignalBlocker block(slider);
          slider->setValue(int(target));
        }
        loadFrame(target, false);
      }
      schedule();
    });
    /**
     * @brief 變更循環選項時重新排程；速度變更保持目標時間連續。
     *
     * @par English
     * Reschedule when looping changes; rate changes re-anchor without a target-time discontinuity.
     */
    QObject::connect(loop, &QCheckBox::toggled, [&](bool) { schedule(); });
    QObject::connect(
        speed, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int) {
          if (playing) {
            double now = wallSeconds();
            playbackClock.setRate(now,
                                  playbackRates[size_t(speed->currentIndex())]);
            rateWallBase = now;
            rateBagBase = reader.frames[size_t(slider->value())].time;
          }
          schedule();
        });
    /**
     * @brief 停止播放並移到前一幀，邊界由 QSlider 處理。
     *
     * @par English
     * Stop playback and select the preceding frame; QSlider handles range limits.
     */
    QObject::connect(prev, &QPushButton::clicked, [&]() {
      stop();
      slider->setValue(slider->value() - 1);
    });
    /**
     * @brief 停止播放並移到下一幀，邊界由 QSlider 處理。
     *
     * @par English
     * Stop playback and select the next frame; QSlider handles range limits.
     */
    QObject::connect(next, &QPushButton::clicked, [&]() {
      stop();
      slider->setValue(slider->value() + 1);
    });
    /**
     * @brief Fit view 回復以 map +Z 向上、+Y 向左、朝 +X 的預設視角。
     *
     * @par English
     * Fit view restores the map-based initial orientation: +Z up, +Y left, looking along +X.
     */
    QObject::connect(reset, &QPushButton::clicked,
                     [&]() { view->resetCamera(); });
    /**
     * @brief 切換高度、單色或反射強度彩色並要求下一次重繪。
     *
     * @par English
     * Select height, single-color, or reflectance color rendering and request a repaint.
     */
    QObject::connect(color, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [&](int i) {
                       view->colorMode = i;
                       refreshIntensity(true);
                   view->update();
                     });
    /**
     * @brief 僅切換光達位置／朝向 overlay，不改變讀檔或時間配對。
     *
     * @par English
     * Toggle only the LiDAR position/orientation overlay, without changing reading or time
     * matching.
     */
    QObject::connect(showPose, &QCheckBox::toggled, [&](bool on) {
      view->showPose = on;
      view->update();
    });
    /**
     * @brief 僅切換截至目前幀的歷史路徑 overlay。
     *
     * @par English
     * Toggle only the trajectory overlay through the current frame.
     */
    QObject::connect(showTrajectory, &QCheckBox::toggled, [&](bool on) {
      view->showTrajectory = on;
      view->update();
    });
    showStatus();
    window.show();
    loadFrame(o.frame);
    view->resetCamera();
    if (o.autoPlay)
      QTimer::singleShot(200, [&]() {
        const uint64_t firstPaint = view->paintCount;
        QElapsedTimer measured;
        measured.start();
        play->click();
        const size_t startFrame = size_t(slider->value());
        const double startLoopAdvance = loopBagAdvance;
        if (o.benchmarkSeconds > 0)
          QTimer::singleShot(
              int(std::ceil(o.benchmarkSeconds * 1000)),
              [&, startFrame, startLoopAdvance, firstPaint, measured]() {
                double wall = measured.nsecsElapsed() * 1e-9;
                double advanced = loopBagAdvance - startLoopAdvance +
                                  reader.frames[size_t(slider->value())].time -
                                  reader.frames[startFrame].time;
                std::cout << std::fixed << std::setprecision(3)
                          << "Playback benchmark: requested=" << o.initialRate
                          << "x, actual=" << advanced / wall
                          << "x, wall=" << wall << " s, bag=" << advanced
                          << " s, start_frame=" << startFrame
                          << ", end_frame=" << slider->value()
                          << ", paints=" << view->paintCount - firstPaint
                          << ", accumulated_frames="
                          << accumulation.loadedFrames() << ", input_points="
                          << accumulation.grid.inputPoints() << '\n';
                stop();
                app.quit();
              });
      });
    if (!o.screenshot.empty())
      /**
       * @brief 等待初始視窗繪製後擷取 OpenGL
       * framebuffer，成功或失敗均退出事件迴圈。
       *
       * 此模式需要可用 OpenGL context；isValid
       * 或圖片儲存失敗會設定非零結束狀態。
       *
       * @par English
       * Wait for the initial window to render, capture the OpenGL framebuffer, and exit the event
       * loop on either success or failure. A valid OpenGL context is required; isValid() or
       * image-save failure sets a nonzero exit status.
       */
      QTimer::singleShot(800, [&]() {
        if (!view->isValid() || !view->grabFramebuffer().save(
                                    QString::fromStdString(o.screenshot))) {
          std::cerr << "Screenshot failed (OpenGL context/display required)\n";
          failed = true;
        }
        app.quit();
      });
    int rc = app.exec();
    return failed ? 1 : rc;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }
}
