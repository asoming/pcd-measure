#include "olx_capture_dialog.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSizePolicy>
#include <QSplitter>
#include <QStandardPaths>
#include <QTextEdit>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QtEndian>

class LiveCloudPreviewWidget final : public QWidget
{
public:
  explicit LiveCloudPreviewWidget(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setObjectName(QStringLiteral("olxLivePreview"));
    setMinimumSize(480, 360);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    setToolTip(QStringLiteral("左键拖动旋转，滚轮缩放，双击恢复视角"));
  }

  void set_cumulative(bool cumulative)
  {
    cumulative_ = cumulative;
    update();
  }

  bool load_file(
    const QString & path,
    std::uint32_t * frame_id,
    std::size_t * point_count,
    bool * changed)
  {
    if (changed) *changed = false;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return false;
    const QByteArray header = file.read(24);
    if (header.size() != 24 || header.left(8) != QByteArrayLiteral("PCPV0001")) return false;
    const std::uint32_t next_frame = qFromLittleEndian<std::uint32_t>(
      reinterpret_cast<const uchar *>(header.constData() + 8));
    const double timestamp = little_double(header.constData() + 12);
    const std::uint32_t count = qFromLittleEndian<std::uint32_t>(
      reinterpret_cast<const uchar *>(header.constData() + 20));
    constexpr std::uint32_t kMaximumPreviewPoints = 200000U;
    if (!std::isfinite(timestamp) || timestamp < 0.0 || count > kMaximumPreviewPoints ||
      file.size() != 24 + static_cast<qint64>(count) * 16)
    {
      return false;
    }
    if (frame_id) *frame_id = next_frame;
    if (point_count) *point_count = count;
    if (have_frame_ && next_frame == frame_id_ && count == points_.size()) return true;

    const QByteArray payload = file.read(static_cast<qint64>(count) * 16);
    if (payload.size() != static_cast<qint64>(count) * 16) return false;
    std::vector<Point> next_points;
    next_points.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index) {
      const char * record = payload.constData() + static_cast<std::size_t>(index) * 16;
      Point point;
      point.x = little_float(record);
      point.y = little_float(record + 4);
      point.z = little_float(record + 8);
      point.r = static_cast<std::uint8_t>(record[12]);
      point.g = static_cast<std::uint8_t>(record[13]);
      point.b = static_cast<std::uint8_t>(record[14]);
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        next_points.push_back(point);
      }
    }
    points_ = std::move(next_points);
    frame_id_ = next_frame;
    have_frame_ = true;
    if (point_count) *point_count = points_.size();
    if (changed) *changed = true;
    update();
    return true;
  }

  void clear_preview()
  {
    points_.clear();
    have_frame_ = false;
    frame_id_ = 0;
    yaw_ = -0.70;
    pitch_ = 0.56;
    zoom_ = 1.0;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), QColor(QStringLiteral("#06141E")));

    painter.setPen(QPen(QColor(26, 57, 71, 150), 1));
    constexpr int kGrid = 42;
    for (int x = 0; x < width(); x += kGrid) painter.drawLine(x, 0, x, height());
    for (int y = 0; y < height(); y += kGrid) painter.drawLine(0, y, width(), y);
    painter.setPen(QPen(QColor(QStringLiteral("#31566A")), 1));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    if (points_.empty()) {
      painter.setPen(QColor(QStringLiteral("#83A0AD")));
      QFont title_font(QStringLiteral("DejaVu Sans Mono"));
      title_font.setBold(true);
      title_font.setPointSize(13);
      painter.setFont(title_font);
      painter.drawText(rect().adjusted(20, 0, -20, -18), Qt::AlignCenter,
        cumulative_ ? QStringLiteral("ACCUMULATED MAP · 等待第一帧") :
        QStringLiteral("CURRENT FRAME · 等待点云帧"));
      QFont hint_font = painter.font();
      hint_font.setBold(false);
      hint_font.setPointSize(10);
      painter.setFont(hint_font);
      painter.setPen(QColor(QStringLiteral("#587583")));
      painter.drawText(rect().adjusted(20, 38, -20, 0), Qt::AlignCenter,
        cumulative_ ? QStringLiteral("开始扫描后，新采到的表面会保留并逐步成形") :
        QStringLiteral("开始扫描后，这里显示传感器当前帧"));
      return;
    }

    struct ProjectedPoint
    {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      QColor color;
    };
    float min_x = points_.front().x;
    float max_x = points_.front().x;
    float min_y = points_.front().y;
    float max_y = points_.front().y;
    float min_z = points_.front().z;
    float max_z = points_.front().z;
    for (const Point & point : points_) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }
    const double center_x = (static_cast<double>(min_x) + max_x) * 0.5;
    const double center_y = (static_cast<double>(min_y) + max_y) * 0.5;
    const double center_z = (static_cast<double>(min_z) + max_z) * 0.5;
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double cos_pitch = std::cos(pitch_);
    const double sin_pitch = std::sin(pitch_);
    std::vector<ProjectedPoint> projected;
    projected.reserve(points_.size());
    double projected_min_x = 0.0;
    double projected_max_x = 0.0;
    double projected_min_y = 0.0;
    double projected_max_y = 0.0;
    bool first = true;
    const double z_span = std::max(1e-6, static_cast<double>(max_z - min_z));
    for (const Point & point : points_) {
      const double dx = point.x - center_x;
      const double dy = point.y - center_y;
      const double dz = point.z - center_z;
      const double rotated_x = cos_yaw * dx - sin_yaw * dy;
      const double rotated_y = sin_yaw * dx + cos_yaw * dy;
      const double screen_y = cos_pitch * dz - sin_pitch * rotated_y;
      const double depth = sin_pitch * dz + cos_pitch * rotated_y;
      QColor color(point.r, point.g, point.b);
      if (point.r == 0 && point.g == 0 && point.b == 0) {
        const double ratio = std::clamp((point.z - min_z) / z_span, 0.0, 1.0);
        color = QColor::fromRgbF(0.12 + ratio * 0.85, 0.82 - ratio * 0.28, 0.82 - ratio * 0.62);
      }
      projected.push_back({rotated_x, screen_y, depth, color});
      if (first) {
        projected_min_x = projected_max_x = rotated_x;
        projected_min_y = projected_max_y = screen_y;
        first = false;
      } else {
        projected_min_x = std::min(projected_min_x, rotated_x);
        projected_max_x = std::max(projected_max_x, rotated_x);
        projected_min_y = std::min(projected_min_y, screen_y);
        projected_max_y = std::max(projected_max_y, screen_y);
      }
    }
    std::sort(projected.begin(), projected.end(), [](const auto & left, const auto & right) {
      return left.z < right.z;
    });
    const double span_x = std::max(1e-6, projected_max_x - projected_min_x);
    const double span_y = std::max(1e-6, projected_max_y - projected_min_y);
    const QRectF viewport = rect().adjusted(34, 42, -34, -34);
    const double scale = std::min(viewport.width() / span_x, viewport.height() / span_y) *
      0.90 * zoom_;
    const double projected_center_x = (projected_min_x + projected_max_x) * 0.5;
    const double projected_center_y = (projected_min_y + projected_max_y) * 0.5;
    const QPointF screen_center = viewport.center();
    QPen point_pen;
    point_pen.setWidthF(points_.size() < 12000 ? 2.0 : 1.2);
    point_pen.setCapStyle(Qt::RoundCap);
    for (const ProjectedPoint & point : projected) {
      point_pen.setColor(point.color);
      painter.setPen(point_pen);
      painter.drawPoint(QPointF(
        screen_center.x() + (point.x - projected_center_x) * scale,
        screen_center.y() - (point.y - projected_center_y) * scale));
    }

    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QColor(QStringLiteral("#79E6E0")));
    QFont overlay_font(QStringLiteral("DejaVu Sans Mono"));
    overlay_font.setBold(true);
    overlay_font.setPointSize(9);
    painter.setFont(overlay_font);
    painter.drawText(QRect(14, 11, width() - 28, 20), Qt::AlignLeft | Qt::AlignVCenter,
      (cumulative_ ? QStringLiteral("ACCUMULATED MAP · FRAME %1 · %2 MAP PTS") :
      QStringLiteral("CURRENT FRAME · FRAME %1 · %2 PTS"))
        .arg(frame_id_).arg(points_.size()));
    painter.setPen(QColor(QStringLiteral("#6E8995")));
    painter.drawText(QRect(14, height() - 27, width() - 28, 18),
      Qt::AlignLeft | Qt::AlignVCenter,
      cumulative_ ? QStringLiteral("空间去重预览 · 左键旋转 · 滚轮缩放 · 双击复位") :
      QStringLiteral("当前帧预览 · 左键旋转 · 滚轮缩放 · 双击复位"));
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton) {
      dragging_ = true;
      drag_position_ = event->pos();
      event->accept();
      return;
    }
    QWidget::mousePressEvent(event);
  }

  void mouseMoveEvent(QMouseEvent * event) override
  {
    if (!dragging_) {
      QWidget::mouseMoveEvent(event);
      return;
    }
    const QPoint delta = event->pos() - drag_position_;
    drag_position_ = event->pos();
    yaw_ += delta.x() * 0.010;
    pitch_ = std::clamp(pitch_ + delta.y() * 0.008, -1.35, 1.35);
    update();
    event->accept();
  }

  void mouseReleaseEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton) {
      dragging_ = false;
      event->accept();
      return;
    }
    QWidget::mouseReleaseEvent(event);
  }

  void mouseDoubleClickEvent(QMouseEvent * event) override
  {
    yaw_ = -0.70;
    pitch_ = 0.56;
    zoom_ = 1.0;
    update();
    event->accept();
  }

  void wheelEvent(QWheelEvent * event) override
  {
    zoom_ = std::clamp(zoom_ * std::pow(1.0015, event->angleDelta().y()), 0.25, 8.0);
    update();
    event->accept();
  }

private:
  struct Point
  {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
  };

  static float little_float(const char * data)
  {
    const std::uint32_t bits = qFromLittleEndian<std::uint32_t>(
      reinterpret_cast<const uchar *>(data));
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  static double little_double(const char * data)
  {
    const std::uint64_t bits = qFromLittleEndian<std::uint64_t>(
      reinterpret_cast<const uchar *>(data));
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::vector<Point> points_;
  QPoint drag_position_;
  std::uint32_t frame_id_ = 0;
  double yaw_ = -0.70;
  double pitch_ = 0.56;
  double zoom_ = 1.0;
  bool have_frame_ = false;
  bool dragging_ = false;
  bool cumulative_ = true;
};

namespace
{

QString shell_quote(QString value)
{
  value.replace(QLatin1Char('\''), QStringLiteral("'\"'\"'"));
  return QLatin1Char('\'') + value + QLatin1Char('\'');
}

QString status_style(bool ok)
{
  return QStringLiteral(
    "border:1px solid %1; border-radius:5px; padding:5px 8px; color:%2; "
    "background:%3; font-family:'DejaVu Sans Mono'; font-weight:700;")
    .arg(ok ? QStringLiteral("#2A7E82") : QStringLiteral("#705A35"))
    .arg(ok ? QStringLiteral("#7CE4E1") : QStringLiteral("#FFCA72"))
    .arg(ok ? QStringLiteral("#0A2932") : QStringLiteral("#292215"));
}

}  // namespace

OlxCaptureDialog::OlxCaptureDialog(QWidget * parent, bool embedded)
: QDialog(parent, embedded ? Qt::WindowFlags(Qt::Widget) : Qt::WindowFlags()),
  embedded_(embedded)
{
  setObjectName(QStringLiteral("olxCaptureDialog"));
  setProperty("embeddedWorkspace", embedded);
  setWindowTitle(QStringLiteral("设备采集与 OLX 录制"));
  setAttribute(Qt::WA_DeleteOnClose, false);
  if (embedded) {
    setWindowFlags(Qt::Widget);
    setMinimumSize(0, 0);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  } else {
    setMinimumSize(1080, 700);
    resize(1320, 820);
  }

  setStyleSheet(QStringLiteral(R"(
    QDialog#olxCaptureDialog { background:#06141E; color:#DCE9ED; }
    QFrame#captureHero { background:#0B202D; border:1px solid #2A4C5E; border-top:3px solid #FFB547; border-radius:8px; }
    QLabel#captureConsoleMark { color:#FFCA72; font:700 10px 'DejaVu Sans Mono'; letter-spacing:1.4px; }
    QLabel#captureTitle { color:#F1FAFC; font-size:22px; font-weight:700; }
    QLabel#captureSubtitle { color:#8FAAB6; }
    QWidget#captureControlSurface { background:#081924; }
    QFrame#capturePreviewPanel { background:#081924; border:1px solid #29485C; border-radius:7px; }
    QLabel#captureSectionMark { color:#7895A2; font:700 10px 'DejaVu Sans Mono'; letter-spacing:1px; }
    QLabel#olxPreviewStatus { color:#79E6E0; font:700 10px 'DejaVu Sans Mono'; }
    QGroupBox { background:#0B1E2A; color:#91AFBB; border:1px solid #294A5B; border-radius:7px; margin-top:14px; padding:12px 9px 9px 9px; font-weight:650; }
    QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 6px; color:#A6C0CA; }
  )"));

  auto * outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(12, 12, 12, 10);
  outer_layout->setSpacing(9);
  auto * hero = new QFrame(this);
  hero->setObjectName(QStringLiteral("captureHero"));
  auto * hero_layout = new QHBoxLayout(hero);
  hero_layout->setContentsMargins(17, 11, 17, 11);
  auto * hero_text = new QVBoxLayout;
  hero_text->setSpacing(1);
  auto * console_mark = new QLabel(QStringLiteral("DEVICE // LIVE ACQUISITION"), hero);
  console_mark->setObjectName(QStringLiteral("captureConsoleMark"));
  auto * title = new QLabel(QStringLiteral("实时点云采集"), hero);
  title->setObjectName(QStringLiteral("captureTitle"));
  auto * subtitle = new QLabel(QStringLiteral(
    "看着场景随扫描逐帧成形，同时把完整 ROS2 点云安全录制为 OLX。"), hero);
  subtitle->setObjectName(QStringLiteral("captureSubtitle"));
  hero_text->addWidget(console_mark);
  hero_text->addWidget(title);
  hero_text->addWidget(subtitle);
  hero_layout->addLayout(hero_text, 1);
  recording_status_label_ = new QLabel(QStringLiteral("● IDLE · 等待录制"), hero);
  recording_status_label_->setObjectName(QStringLiteral("olxRecordingStatus"));
  recording_status_label_->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; background:#081924; border:1px solid #29485C; border-radius:5px; "
    "padding:7px 10px; font-family:'DejaVu Sans Mono'; font-weight:700;"));
  hero_layout->addWidget(recording_status_label_, 0, Qt::AlignVCenter);
  outer_layout->addWidget(hero);

  auto * body_splitter = new QSplitter(Qt::Horizontal, this);
  body_splitter->setObjectName(QStringLiteral("captureBodySplitter"));
  body_splitter->setChildrenCollapsible(false);
  auto * control_scroll = new QScrollArea(body_splitter);
  control_scroll->setWidgetResizable(true);
  control_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  control_scroll->setMinimumWidth(405);
  control_scroll->setMaximumWidth(500);
  auto * control_surface = new QWidget(control_scroll);
  control_surface->setObjectName(QStringLiteral("captureControlSurface"));
  auto * layout = new QVBoxLayout(control_surface);
  layout->setContentsMargins(9, 4, 9, 9);
  layout->setSpacing(8);

  auto * connection_group = new QGroupBox(QStringLiteral("01 · 连接设备"), control_surface);
  auto * connection_layout = new QVBoxLayout(connection_group);
  connection_layout->setSpacing(7);

  auto * status_row = new QGridLayout;
  usb_status_label_ = new QLabel(QStringLiteral("USB · 检查中"));
  topic_status_label_ = new QLabel(QStringLiteral("ROS2 · 检查中"));
  usb_status_label_->setObjectName(QStringLiteral("olxUsbStatus"));
  topic_status_label_->setObjectName(QStringLiteral("olxTopicStatus"));
  usb_status_label_->setStyleSheet(status_style(false));
  topic_status_label_->setStyleSheet(status_style(false));
  status_row->addWidget(usb_status_label_, 0, 0);
  status_row->addWidget(topic_status_label_, 0, 1);
  status_row->setColumnStretch(0, 1);
  status_row->setColumnStretch(1, 1);
  auto * refresh_button = new QPushButton(QStringLiteral("刷新状态"));
  connect(refresh_button, &QPushButton::clicked, this, &OlxCaptureDialog::refresh_status);
  status_row->addWidget(refresh_button, 1, 0, 1, 2, Qt::AlignRight);
  connection_layout->addLayout(status_row);

  auto * form = new QFormLayout;
  workspace_edit_ = new QLineEdit;
  workspace_edit_->setMinimumWidth(0);
  workspace_edit_->setObjectName(QStringLiteral("olxWorkspace"));
  QString default_workspace = QDir::home().filePath(QStringLiteral("Desktop/odin1_mapping_ws"));
  workspace_edit_->setText(QSettings().value(
    QStringLiteral("olxCapture/workspace"), default_workspace).toString());
  workspace_edit_->setCursorPosition(0);
  auto * workspace_row = new QHBoxLayout;
  workspace_row->addWidget(workspace_edit_, 1);
  auto * workspace_button = new QPushButton(QStringLiteral("选择…"));
  connect(workspace_button, &QPushButton::clicked, this, &OlxCaptureDialog::browse_workspace);
  workspace_row->addWidget(workspace_button);
  form->addRow(QStringLiteral("ROS2 工作空间："), workspace_row);
  connection_layout->addLayout(form);
  layout->addWidget(connection_group);

  auto * source_group = new QGroupBox(QStringLiteral("02 · 选择数据与保存位置"), control_surface);
  auto * source_layout = new QFormLayout(source_group);
  output_edit_ = new QLineEdit;
  output_edit_->setMinimumWidth(0);
  output_edit_->setObjectName(QStringLiteral("olxOutput"));
  QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
  if (documents.isEmpty()) documents = QDir::homePath();
  output_edit_->setText(QSettings().value(
    QStringLiteral("olxCapture/output"), QDir(documents).filePath(QStringLiteral("点云录制"))).toString());
  output_edit_->setCursorPosition(0);
  auto * output_row = new QHBoxLayout;
  output_row->addWidget(output_edit_, 1);
  auto * output_button = new QPushButton(QStringLiteral("选择…"));
  connect(output_button, &QPushButton::clicked, this, &OlxCaptureDialog::browse_output);
  output_row->addWidget(output_button);
  source_layout->addRow(QStringLiteral("保存目录："), output_row);

  scan_mode_combo_ = new QComboBox;
  scan_mode_combo_->setObjectName(QStringLiteral("olxScanMode"));
  scan_mode_combo_->addItem(QStringLiteral("SLAM 彩色点云"), QStringLiteral("/odin1/cloud_slam"));
  scan_mode_combo_->addItem(QStringLiteral("渲染彩色点云"), QStringLiteral("/odin1/cloud_render"));
  scan_mode_combo_->addItem(QStringLiteral("DTOF 原始强度点云"), QStringLiteral("/odin1/cloud_raw"));
  source_layout->addRow(QStringLiteral("扫描模式："), scan_mode_combo_);
  preview_mode_combo_ = new QComboBox;
  preview_mode_combo_->setObjectName(QStringLiteral("olxPreviewMode"));
  preview_mode_combo_->addItem(
    QStringLiteral("累计建图（逐帧叠加）"), QStringLiteral("cumulative"));
  preview_mode_combo_->addItem(
    QStringLiteral("当前帧（不叠加）"), QStringLiteral("frame"));
  preview_mode_combo_->setToolTip(QStringLiteral(
    "SLAM 点云适合累计建图；原始 DTOF 在设备移动时应使用当前帧。"));
  source_layout->addRow(QStringLiteral("实时画面："), preview_mode_combo_);
  pose_topic_edit_ = new QLineEdit(QStringLiteral("/odin1/odometry"));
  image_topic_edit_ = new QLineEdit(QStringLiteral("/odin1/image/compressed"));
  pose_topic_edit_->setObjectName(QStringLiteral("olxPoseTopic"));
  image_topic_edit_->setObjectName(QStringLiteral("olxImageTopic"));
  source_layout->addRow(QStringLiteral("位姿话题："), pose_topic_edit_);
  source_layout->addRow(QStringLiteral("相机话题："), image_topic_edit_);
  layout->addWidget(source_group);

  auto * record_group = new QGroupBox(QStringLiteral("03 · 采集控制"), control_surface);
  auto * record_layout = new QVBoxLayout(record_group);
  auto * options = new QGridLayout;
  pose_check_ = new QCheckBox(QStringLiteral("记录位姿"));
  pose_check_->setChecked(true);
  image_check_ = new QCheckBox(QStringLiteral("记录相机（话题存在时）"));
  image_check_->setChecked(true);
  auto_open_check_ = new QCheckBox(QStringLiteral("完成后自动打开"));
  auto_open_check_->setChecked(true);
  options->addWidget(pose_check_, 0, 0);
  options->addWidget(image_check_, 0, 1);
  options->addWidget(auto_open_check_, 1, 0, 1, 2);
  record_layout->addLayout(options);

  auto * actions = new QGridLayout;
  driver_button_ = new QPushButton(QStringLiteral("启动设备驱动"));
  record_button_ = new QPushButton(QStringLiteral("开始扫描并录制"));
  record_button_->setProperty("role", "primary");
  stop_button_ = new QPushButton(QStringLiteral("停止并完成"));
  stop_button_->setEnabled(false);
  open_button_ = new QPushButton(QStringLiteral("打开最近录制"));
  driver_button_->setObjectName(QStringLiteral("olxDriverButton"));
  record_button_->setObjectName(QStringLiteral("olxRecordButton"));
  stop_button_->setObjectName(QStringLiteral("olxStopButton"));
  open_button_->setObjectName(QStringLiteral("olxOpenButton"));
  open_button_->setEnabled(false);
  connect(driver_button_, &QPushButton::clicked, this, &OlxCaptureDialog::toggle_driver);
  connect(record_button_, &QPushButton::clicked, this, &OlxCaptureDialog::start_recording);
  connect(stop_button_, &QPushButton::clicked, this, &OlxCaptureDialog::stop_recording);
  connect(open_button_, &QPushButton::clicked, this, [this]() {
    if (QFileInfo(last_olx_path_).isFile()) emit openOlxRequested(last_olx_path_);
  });
  actions->addWidget(driver_button_, 0, 0);
  actions->addWidget(record_button_, 0, 1);
  actions->addWidget(stop_button_, 1, 0);
  actions->addWidget(open_button_, 1, 1);
  record_layout->addLayout(actions);

  counter_label_ = new QLabel(QStringLiteral("帧 0 · 点 0 · 0.0 s\n位姿 0 · 图像 0"));
  counter_label_->setObjectName(QStringLiteral("olxCounters"));
  counter_label_->setStyleSheet(QStringLiteral("color:#DCE9ED; font-family:'DejaVu Sans Mono';"));
  counter_label_->setWordWrap(true);
  record_layout->addWidget(counter_label_);
  layout->addWidget(record_group);
  auto * safety_note = new QLabel(QStringLiteral(
    "累计画面使用空间去重并限制显示点数；完整点云仍逐帧写入 OLX，不改变颜色或文件精度。"), control_surface);
  safety_note->setWordWrap(true);
  safety_note->setStyleSheet(QStringLiteral(
    "color:#829EAA; background:#0B202C; padding:8px; border-left:3px solid #25D0C8;"));
  layout->addWidget(safety_note);
  layout->addStretch(1);
  control_scroll->setWidget(control_surface);
  body_splitter->addWidget(control_scroll);

  auto * preview_panel = new QFrame(body_splitter);
  preview_panel->setObjectName(QStringLiteral("capturePreviewPanel"));
  auto * preview_layout = new QVBoxLayout(preview_panel);
  preview_layout->setContentsMargins(9, 9, 9, 9);
  preview_layout->setSpacing(7);
  auto * preview_header = new QHBoxLayout;
  preview_mark_label_ = new QLabel(QStringLiteral("MAP BUILD // SPATIAL PREVIEW"), preview_panel);
  preview_mark_label_->setObjectName(QStringLiteral("captureSectionMark"));
  preview_status_label_ = new QLabel(QStringLiteral("WAITING · MAP 0 PTS"), preview_panel);
  preview_status_label_->setObjectName(QStringLiteral("olxPreviewStatus"));
  preview_status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  preview_header->addWidget(preview_mark_label_);
  preview_header->addStretch(1);
  preview_header->addWidget(preview_status_label_);
  preview_layout->addLayout(preview_header);
  live_preview_ = new LiveCloudPreviewWidget(preview_panel);
  preview_layout->addWidget(live_preview_, 1);

  log_edit_ = new QTextEdit;
  log_edit_->setReadOnly(true);
  log_edit_->document()->setMaximumBlockCount(350);
  log_edit_->setPlaceholderText(QStringLiteral("设备驱动与录制日志会显示在这里。"));
  log_edit_->setMaximumHeight(150);
  preview_layout->addWidget(log_edit_);
  body_splitter->addWidget(preview_panel);
  body_splitter->setStretchFactor(0, 0);
  body_splitter->setStretchFactor(1, 1);
  body_splitter->setSizes({440, 820});
  outer_layout->addWidget(body_splitter, 1);

  close_buttons_ = new QDialogButtonBox(QDialogButtonBox::Close, this);
  close_buttons_->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
  connect(close_buttons_, &QDialogButtonBox::rejected, this, &QDialog::close);
  close_buttons_->setVisible(!embedded_);
  outer_layout->addWidget(close_buttons_);

  connect(&topic_probe_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, [this](int, QProcess::ExitStatus) {
      const QString topics = QString::fromLocal8Bit(topic_probe_.readAllStandardOutput());
      const QStringList topic_list = topics.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
      selected_topic_ready_ = topic_list.contains(selected_cloud_topic());
      odin_driver_detected_ = std::any_of(topic_list.cbegin(), topic_list.cend(),
        [](const QString & topic) {return topic.startsWith(QStringLiteral("/odin1/cloud"));});
      topic_status_label_->setText(selected_topic_ready_ ? QStringLiteral("ROS2 · CLOUD READY") :
        QStringLiteral("ROS2 · 未发现点云话题"));
      topic_status_label_->setStyleSheet(status_style(selected_topic_ready_));
      update_recording_controls();
    });
  connect(&driver_process_, &QProcess::readyReadStandardOutput, this, [this]() {
    append_log(QString::fromLocal8Bit(driver_process_.readAllStandardOutput()));
  });
  connect(&driver_process_, &QProcess::readyReadStandardError, this, [this]() {
    append_log(QString::fromLocal8Bit(driver_process_.readAllStandardError()));
  });
  connect(&driver_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, [this](int code, QProcess::ExitStatus) {
      append_log(QStringLiteral("设备驱动已退出（%1）。").arg(code));
      driver_button_->setText(QStringLiteral("启动设备驱动"));
      driver_started_here_ = false;
      odin_driver_detected_ = false;
      selected_topic_ready_ = false;
      refresh_status();
    });
  connect(&recorder_process_, &QProcess::readyReadStandardOutput,
    this, &OlxCaptureDialog::consume_recorder_output);
  connect(&recorder_process_, &QProcess::readyReadStandardError, this, [this]() {
    append_log(QString::fromLocal8Bit(recorder_process_.readAllStandardError()));
  });
  connect(&recorder_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &OlxCaptureDialog::recorder_finished);
  connect(preview_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {
      const bool cumulative = selected_preview_mode() == QStringLiteral("cumulative");
      live_preview_->set_cumulative(cumulative);
      preview_mark_label_->setText(cumulative ? QStringLiteral("MAP BUILD // SPATIAL PREVIEW") :
        QStringLiteral("LIVE VIEW // CURRENT FRAME"));
      if (recorder_process_.state() == QProcess::NotRunning && !have_preview_frame_) {
        preview_status_label_->setText(cumulative ? QStringLiteral("WAITING · MAP 0 PTS") :
          QStringLiteral("WAITING · FRAME 0 PTS"));
      }
    });
  connect(scan_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int index) {
      if (recorder_process_.state() == QProcess::NotRunning) {
        preview_mode_combo_->setCurrentIndex(index == 2 ? 1 : 0);
      }
      refresh_status();
    });
  connect(workspace_edit_, &QLineEdit::editingFinished, this, &OlxCaptureDialog::refresh_status);
  preview_timer_ = new QTimer(this);
  preview_timer_->setInterval(150);
  connect(preview_timer_, &QTimer::timeout, this, &OlxCaptureDialog::refresh_live_preview);
  if (preview_directory_.isValid()) {
    preview_file_path_ = preview_directory_.filePath(QStringLiteral("live-preview.bin"));
  } else {
    preview_status_label_->setText(QStringLiteral("PREVIEW UNAVAILABLE"));
  }
  update_recording_controls();
  if (!embedded_) refresh_status();
}

OlxCaptureDialog::~OlxCaptureDialog()
{
  stop_child_processes();
}

void OlxCaptureDialog::activate_workspace()
{
  refresh_status();
}

void OlxCaptureDialog::reject()
{
  if (embedded_) return;
  QDialog::reject();
}

QString OlxCaptureDialog::project_directory() const
{
  const QString configured = qEnvironmentVariable("POINT_CLOUD_WORKBENCH_PROJECT_DIR").trimmed();
  if (!configured.isEmpty()) return QFileInfo(configured).absoluteFilePath();
  QDir directory(QCoreApplication::applicationDirPath());
  if (directory.dirName() == QStringLiteral("bin")) {
    directory.cdUp();
    if (directory.dirName() == QStringLiteral("build")) directory.cdUp();
  }
  return directory.absolutePath();
}

QString OlxCaptureDialog::selected_cloud_topic() const
{
  return scan_mode_combo_->currentData().toString();
}

QString OlxCaptureDialog::selected_preview_mode() const
{
  return preview_mode_combo_->currentData().toString();
}

void OlxCaptureDialog::browse_workspace()
{
  const QString path = QFileDialog::getExistingDirectory(
    this, QStringLiteral("选择 ROS2 工作空间"), workspace_edit_->text());
  if (!path.isEmpty()) workspace_edit_->setText(path);
}

void OlxCaptureDialog::browse_output()
{
  const QString path = QFileDialog::getExistingDirectory(
    this, QStringLiteral("选择录制保存目录"), output_edit_->text());
  if (!path.isEmpty()) output_edit_->setText(path);
}

bool OlxCaptureDialog::usb_device_present() const
{
  QDirIterator iterator(QStringLiteral("/sys/bus/usb/devices"),
    QDir::Dirs | QDir::NoDotAndDotDot);
  while (iterator.hasNext()) {
    const QString directory = iterator.next();
    QFile vendor(QDir(directory).filePath(QStringLiteral("idVendor")));
    QFile product(QDir(directory).filePath(QStringLiteral("idProduct")));
    if (!vendor.open(QIODevice::ReadOnly) || !product.open(QIODevice::ReadOnly)) continue;
    if (vendor.readAll().trimmed().toLower() == QByteArrayLiteral("2207") &&
      product.readAll().trimmed().toLower() == QByteArrayLiteral("0019")) return true;
  }
  return false;
}

void OlxCaptureDialog::refresh_status()
{
  const bool usb = usb_device_present();
  usb_status_label_->setText(usb ? QStringLiteral("USB · DEVICE READY") :
    QStringLiteral("USB · 未发现 2207:0019"));
  usb_status_label_->setStyleSheet(status_style(usb));
  if (topic_probe_.state() != QProcess::NotRunning) return;
  const QString setup = QDir(workspace_edit_->text()).filePath(QStringLiteral("install/setup.bash"));
  if (!QFileInfo(setup).isFile()) {
    selected_topic_ready_ = false;
    odin_driver_detected_ = false;
    topic_status_label_->setText(QStringLiteral("ROS2 · 工作空间未构建"));
    topic_status_label_->setStyleSheet(status_style(false));
    update_recording_controls();
    return;
  }
  selected_topic_ready_ = false;
  const QString command = QStringLiteral(
    "source /opt/ros/humble/setup.bash && source %1 && timeout 4 ros2 topic list")
    .arg(shell_quote(setup));
  topic_probe_.start(QStringLiteral("/bin/bash"), {QStringLiteral("-lc"), command});
  topic_status_label_->setText(QStringLiteral("ROS2 · 检查中"));
  topic_status_label_->setStyleSheet(status_style(false));
  update_recording_controls();
}

void OlxCaptureDialog::toggle_driver()
{
  if (driver_process_.state() != QProcess::NotRunning) {
    append_log(QStringLiteral("正在停止设备驱动…"));
    driver_button_->setEnabled(false);
    driver_process_.terminate();
    QTimer::singleShot(8000, this, [this]() {
      if (driver_process_.state() != QProcess::NotRunning) {
        append_log(QStringLiteral("设备驱动未在 8 秒内退出，正在强制结束。"));
        driver_process_.kill();
      }
    });
    return;
  }
  const QString workspace = QFileInfo(workspace_edit_->text()).absoluteFilePath();
  const QString script = QDir(workspace).filePath(QStringLiteral("scripts/start_mapping.sh"));
  if (!QFileInfo(script).isFile()) {
    QMessageBox::warning(this, QStringLiteral("无法启动驱动"),
      QStringLiteral("找不到：%1").arg(script));
    return;
  }
  QSettings().setValue(QStringLiteral("olxCapture/workspace"), workspace);
  driver_process_.setWorkingDirectory(workspace);
  driver_process_.start(QStringLiteral("/bin/bash"), {script, QStringLiteral("--headless")});
  if (!driver_process_.waitForStarted(3000)) {
    QMessageBox::critical(this, QStringLiteral("无法启动驱动"), driver_process_.errorString());
    return;
  }
  driver_started_here_ = true;
  driver_button_->setText(QStringLiteral("停止设备驱动"));
  append_log(QStringLiteral("设备驱动已启动，初始化通常需要 2–8 秒。"));
  update_recording_controls();
  QTimer::singleShot(3500, this, &OlxCaptureDialog::refresh_status);
  QTimer::singleShot(7500, this, &OlxCaptureDialog::refresh_status);
}

void OlxCaptureDialog::start_recording()
{
  if (recorder_process_.state() != QProcess::NotRunning) return;
  const QString workspace = QFileInfo(workspace_edit_->text()).absoluteFilePath();
  const QString output = QFileInfo(output_edit_->text()).absoluteFilePath();
  const QString script = QDir(project_directory()).filePath(QStringLiteral("scripts/record_olx.sh"));
  if (!QFileInfo(QDir(workspace).filePath(QStringLiteral("install/setup.bash"))).isFile() ||
    !QFileInfo(script).isFile())
  {
    QMessageBox::warning(this, QStringLiteral("录制环境不完整"),
      QStringLiteral("请确认 ROS2 工作空间已构建，且项目脚本存在。"));
    return;
  }
  if (!selected_topic_ready_) {
    QMessageBox::warning(this, QStringLiteral("点云话题未就绪"),
      QStringLiteral("尚未发现 %1。请先启动设备驱动，等待初始化后点击“刷新状态”。")
        .arg(selected_cloud_topic()));
    return;
  }
  if (!QDir().mkpath(output)) {
    QMessageBox::critical(this, QStringLiteral("无法创建目录"), output);
    return;
  }
  QSettings().setValue(QStringLiteral("olxCapture/workspace"), workspace);
  QSettings().setValue(QStringLiteral("olxCapture/output"), output);
  const QString session = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
  QStringList arguments{script, workspace,
    QStringLiteral("--output-dir"), output,
    QStringLiteral("--session-name"), session,
    QStringLiteral("--cloud-topic"), selected_cloud_topic(),
    QStringLiteral("--pose-topic"), pose_topic_edit_->text().trimmed(),
    QStringLiteral("--image-topic"), image_topic_edit_->text().trimmed()};
  if (!preview_file_path_.isEmpty()) {
    QFile::remove(preview_file_path_);
    arguments << QStringLiteral("--preview-file") << preview_file_path_
      << QStringLiteral("--preview-points") << QStringLiteral("60000")
      << QStringLiteral("--preview-interval") << QStringLiteral("0.20")
      << QStringLiteral("--preview-mode") << selected_preview_mode();
    live_preview_->set_cumulative(selected_preview_mode() == QStringLiteral("cumulative"));
    live_preview_->clear_preview();
    preview_status_label_->setText(selected_preview_mode() == QStringLiteral("cumulative") ?
      QStringLiteral("STARTING · MAP 0 PTS") : QStringLiteral("STARTING · FRAME 0 PTS"));
    have_preview_frame_ = false;
    last_preview_point_count_ = 0;
    last_preview_source_points_ = 0;
    preview_voxel_size_m_ = 0.0;
    preview_rate_hz_ = 0.0;
    preview_rate_timer_.restart();
  }
  const QString calibration = QDir(workspace).filePath(QStringLiteral("runtime/calib.yaml"));
  if (QFileInfo(calibration).isFile()) arguments << QStringLiteral("--calibration") << calibration;
  if (!pose_check_->isChecked()) arguments << QStringLiteral("--no-pose");
  if (!image_check_->isChecked()) arguments << QStringLiteral("--no-image");
  recorder_output_buffer_.clear();
  last_session_path_.clear();
  last_olx_path_.clear();
  last_preview_error_.clear();
  recorder_process_.setWorkingDirectory(project_directory());
  recorder_process_.start(QStringLiteral("/bin/bash"), arguments);
  if (!recorder_process_.waitForStarted(3000)) {
    QMessageBox::critical(this, QStringLiteral("无法启动录制器"), recorder_process_.errorString());
    update_recording_controls();
    return;
  }
  recording_status_label_->setText(QStringLiteral("● MAPPING + RECORDING · %1")
    .arg(selected_cloud_topic()));
  recording_status_label_->setStyleSheet(QStringLiteral(
    "color:#FFD38A; background:#302817; border:1px solid #8A6933; padding:8px; "
    "font-family:'DejaVu Sans Mono'; font-weight:700;"));
  append_log(selected_preview_mode() == QStringLiteral("cumulative") ?
    QStringLiteral("开始扫描会话 %1；累计地图将从空画面逐帧增长。").arg(session) :
    QStringLiteral("开始录制会话 %1；实时画面显示当前帧。").arg(session));
  if (!preview_file_path_.isEmpty()) preview_timer_->start();
  update_recording_controls();
}

void OlxCaptureDialog::stop_recording()
{
  if (recorder_process_.state() == QProcess::NotRunning) return;
  recording_status_label_->setText(QStringLiteral("● FINALIZING · 正在刷新文件"));
  recorder_process_.terminate();
  QTimer::singleShot(8000, this, [this]() {
    if (recorder_process_.state() != QProcess::NotRunning) {
      append_log(QStringLiteral("录制器未在 8 秒内退出，正在强制结束。"));
      recorder_process_.kill();
    }
  });
}

void OlxCaptureDialog::consume_recorder_output()
{
  recorder_output_buffer_.append(recorder_process_.readAllStandardOutput());
  int newline = -1;
  while ((newline = recorder_output_buffer_.indexOf('\n')) >= 0) {
    const QByteArray line = recorder_output_buffer_.left(newline).trimmed();
    recorder_output_buffer_.remove(0, newline + 1);
    if (line.startsWith("SESSION ")) {
      last_session_path_ = QString::fromLocal8Bit(line.mid(8));
      append_log(QStringLiteral("会话目录：%1").arg(last_session_path_));
      continue;
    }
    if (line.startsWith("STATUS ")) {
      const QJsonDocument document = QJsonDocument::fromJson(line.mid(7));
      const QJsonObject status = document.object();
      last_olx_path_ = status.value(QStringLiteral("olx")).toString(last_olx_path_);
      last_preview_source_points_ = status.value(
        QStringLiteral("preview_source_points")).toVariant().toULongLong();
      preview_voxel_size_m_ = status.value(QStringLiteral("preview_voxel_size_m")).toDouble();
      const QString preview_error = status.value(QStringLiteral("preview_error")).toString();
      if (!preview_error.isEmpty() && preview_error != last_preview_error_) {
        last_preview_error_ = preview_error;
        append_log(QStringLiteral("实时预览已停止，但完整录制继续：%1").arg(preview_error));
        preview_status_label_->setText(QStringLiteral("PREVIEW ERROR · 录制继续"));
      }
      const qulonglong map_points = status.value(
        QStringLiteral("preview_points")).toVariant().toULongLong();
      const bool cumulative = status.value(QStringLiteral("preview_mode")).toString(
        selected_preview_mode()) == QStringLiteral("cumulative");
      const QString preview_detail = cumulative ?
        QStringLiteral("地图 %1 · 帧点 %2 · %3 cm")
          .arg(map_points)
          .arg(static_cast<qulonglong>(last_preview_source_points_))
          .arg(preview_voxel_size_m_ * 100.0, 0, 'f', 1) :
        QStringLiteral("当前帧预览 %1 点").arg(map_points);
      counter_label_->setText(QStringLiteral("帧 %1 · 写盘点 %2 · %5 s\n位姿 %3 · 图像 %4\n%6")
        .arg(status.value(QStringLiteral("cloud_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("points")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("pose_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("image_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("elapsed_seconds")).toDouble(), 0, 'f', 1)
        .arg(preview_detail));
      continue;
    }
    if (!line.isEmpty()) append_log(QString::fromLocal8Bit(line));
  }
}

void OlxCaptureDialog::refresh_live_preview()
{
  if (preview_file_path_.isEmpty() || !live_preview_) return;
  std::uint32_t frame_id = 0;
  std::size_t point_count = 0;
  bool changed = false;
  if (!live_preview_->load_file(preview_file_path_, &frame_id, &point_count, &changed)) return;
  if (!changed) return;

  if (have_preview_frame_ && frame_id > last_preview_frame_id_ && preview_rate_timer_.isValid()) {
    const qint64 elapsed_ms = preview_rate_timer_.elapsed();
    if (elapsed_ms > 0) {
      const double instant_rate = static_cast<double>(frame_id - last_preview_frame_id_) *
        1000.0 / static_cast<double>(elapsed_ms);
      preview_rate_hz_ = preview_rate_hz_ <= 0.0 ? instant_rate :
        preview_rate_hz_ * 0.65 + instant_rate * 0.35;
    }
  }
  last_preview_frame_id_ = frame_id;
  last_preview_point_count_ = point_count;
  have_preview_frame_ = true;
  preview_rate_timer_.restart();
  const QString status_format = selected_preview_mode() == QStringLiteral("cumulative") ?
    QStringLiteral("BUILDING · F%1 · MAP %2 PTS · %3 Hz") :
    QStringLiteral("LIVE · F%1 · %2 PTS · %3 Hz");
  preview_status_label_->setText(status_format
    .arg(frame_id)
    .arg(static_cast<qulonglong>(point_count))
    .arg(preview_rate_hz_, 0, 'f', 1));
}

void OlxCaptureDialog::recorder_finished(int exit_code, QProcess::ExitStatus status)
{
  consume_recorder_output();
  refresh_live_preview();
  if (preview_timer_) preview_timer_->stop();
  const bool complete = status == QProcess::NormalExit && exit_code == 0 &&
    QFileInfo(last_olx_path_).isFile() && QFileInfo(last_olx_path_).size() > 16;
  recording_status_label_->setText(complete ? QStringLiteral("● COMPLETE · OLX 已安全完成") :
    QStringLiteral("● STOPPED · 未收到有效点云"));
  recording_status_label_->setStyleSheet(status_style(complete));
  open_button_->setEnabled(complete);
  append_log(complete ? QStringLiteral("录制完成：%1").arg(last_olx_path_) :
    QStringLiteral("录制结束（退出码 %1）；请检查点云话题和日志。").arg(exit_code));
  if (have_preview_frame_) {
    preview_status_label_->setText(selected_preview_mode() == QStringLiteral("cumulative") ?
      QStringLiteral("FINAL MAP · %1 PTS · F%2")
        .arg(static_cast<qulonglong>(last_preview_point_count_)).arg(last_preview_frame_id_) :
      QStringLiteral("FINAL FRAME · %1 PTS · F%2")
        .arg(static_cast<qulonglong>(last_preview_point_count_)).arg(last_preview_frame_id_));
  }
  update_recording_controls();
  if (complete && auto_open_check_->isChecked()) emit openOlxRequested(last_olx_path_);
  if (close_when_finished_) close();
}

void OlxCaptureDialog::append_log(const QString & text)
{
  const QString cleaned = text.trimmed();
  if (!cleaned.isEmpty()) log_edit_->append(cleaned.toHtmlEscaped().replace(QLatin1Char('\n'), QStringLiteral("<br>")));
}

void OlxCaptureDialog::update_recording_controls()
{
  const bool recording = recorder_process_.state() != QProcess::NotRunning;
  const bool workspace_ready = QFileInfo(
    QDir(workspace_edit_->text()).filePath(QStringLiteral("install/setup.bash"))).isFile();
  const bool driver_running_here = driver_process_.state() != QProcess::NotRunning;
  record_button_->setEnabled(!recording && workspace_ready && selected_topic_ready_);
  stop_button_->setEnabled(recording);
  if (driver_running_here) {
    driver_button_->setText(QStringLiteral("停止设备驱动"));
    driver_button_->setEnabled(!recording);
  } else if (odin_driver_detected_) {
    driver_button_->setText(QStringLiteral("驱动已在运行"));
    driver_button_->setEnabled(false);
  } else {
    driver_button_->setText(QStringLiteral("启动设备驱动"));
    driver_button_->setEnabled(!recording && workspace_ready);
  }
  workspace_edit_->setEnabled(!recording);
  output_edit_->setEnabled(!recording);
  scan_mode_combo_->setEnabled(!recording);
  preview_mode_combo_->setEnabled(!recording);
  pose_topic_edit_->setEnabled(!recording);
  image_topic_edit_->setEnabled(!recording);
  pose_check_->setEnabled(!recording);
  image_check_->setEnabled(!recording);
}

void OlxCaptureDialog::stop_child_processes()
{
  if (preview_timer_) preview_timer_->stop();
  if (recorder_process_.state() != QProcess::NotRunning) {
    recorder_process_.terminate();
    if (!recorder_process_.waitForFinished(8000)) {
      recorder_process_.kill();
      recorder_process_.waitForFinished(2000);
    }
  }
  if (driver_started_here_ && driver_process_.state() != QProcess::NotRunning) {
    driver_process_.terminate();
    if (!driver_process_.waitForFinished(8000)) {
      driver_process_.kill();
      driver_process_.waitForFinished(2000);
    }
  }
  if (topic_probe_.state() != QProcess::NotRunning) {
    topic_probe_.terminate();
    if (!topic_probe_.waitForFinished(1000)) {
      topic_probe_.kill();
      topic_probe_.waitForFinished(1000);
    }
  }
}

void OlxCaptureDialog::closeEvent(QCloseEvent * event)
{
  if (recorder_process_.state() != QProcess::NotRunning) {
    const auto answer = QMessageBox::question(this, QStringLiteral("结束录制？"),
      QStringLiteral("录制仍在进行。是否停止并安全完成文件后关闭？"),
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Yes);
    if (answer != QMessageBox::Yes) {
      event->ignore();
      return;
    }
    close_when_finished_ = true;
    stop_recording();
    event->ignore();
    return;
  }
  stop_child_processes();
  event->accept();
}
