#include "main_window.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <set>
#include <utility>

#include <QAction>
#include <QApplication>
#include <QBuffer>
#include <QCheckBox>
#include <QColor>
#include <QColorDialog>
#include <QComboBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QIcon>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QMessageBox>
#include <QMimeData>
#include <QPageLayout>
#include <QPageSize>
#include <QMenu>
#include <QProgressBar>
#include <QProgressDialog>
#include <QPrinter>
#include <QPushButton>
#include <QPixmap>
#include <QRegularExpression>
#include <QSaveFile>
#include <QScrollArea>
#include <QSettings>
#include <QShortcut>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStandardPaths>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QStringList>
#include <QTableWidget>
#include <QTextDocument>
#include <QTextEdit>
#include <QTextStream>
#include <QTemporaryFile>
#include <QToolBar>
#include <QToolButton>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <QtConcurrent/QtConcurrentRun>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/pcd_io.h>

#include "plot_widget.h"
#include "rosbag_dialog.h"
#include "rosbag_tools.h"

namespace
{

constexpr const char * kCloudId = "pcd_cloud";
constexpr const char * kPendingPointId = "pending_measurement_point";
constexpr const char * kCropFirstPointId = "crop_first_corner";
constexpr const char * kAxesId = "coordinate_axes";
constexpr const char * kComparisonCloudId = "comparison_cloud";
constexpr const char * kContextCloudId = "context_cloud";

struct ComparisonJobOutput
{
  CloudLoadResult second;
  CloudComparisonResult comparison;
};

class WheelSafeComboBox final : public QComboBox
{
public:
  using QComboBox::QComboBox;

protected:
  void wheelEvent(QWheelEvent * event) override
  {
    event->ignore();
  }
};

QLabel * make_value_label(bool emphasize = false)
{
  auto * label = new QLabel(QStringLiteral("—"));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  if (emphasize) {
    QFont font = label->font();
    font.setBold(true);
    font.setPointSize(font.pointSize() + 1);
    label->setFont(font);
    label->setStyleSheet(QStringLiteral(
      "color: #7CE4E1; font-family: 'DejaVu Sans Mono'; letter-spacing: 0.2px;"));
  }
  return label;
}

QString count_text(std::size_t value)
{
  return QLocale(QLocale::English).toString(static_cast<qulonglong>(value));
}

QString bytes_text(std::uint64_t bytes)
{
  constexpr double kib = 1024.0;
  constexpr double mib = kib * 1024.0;
  constexpr double gib = mib * 1024.0;
  if (bytes >= static_cast<std::uint64_t>(gib)) {
    return QStringLiteral("%1 GiB").arg(static_cast<double>(bytes) / gib, 0, 'f', 2);
  }
  if (bytes >= static_cast<std::uint64_t>(mib)) {
    return QStringLiteral("%1 MiB").arg(static_cast<double>(bytes) / mib, 0, 'f', 2);
  }
  if (bytes >= static_cast<std::uint64_t>(kib)) {
    return QStringLiteral("%1 KiB").arg(static_cast<double>(bytes) / kib, 0, 'f', 1);
  }
  return QStringLiteral("%1 B").arg(bytes);
}

QString coordinate_text(const pcl::PointXYZ & point, double scale, const QString & unit)
{
  const int precision = scale >= 1000.0 ? 1 : (scale >= 100.0 ? 2 : 4);
  return QStringLiteral("X %1  Y %2  Z %3 %4")
    .arg(point.x * scale, 0, 'f', precision)
    .arg(point.y * scale, 0, 'f', precision)
    .arg(point.z * scale, 0, 'f', precision)
    .arg(unit);
}

QString csv_cell(QString value)
{
  value.replace(QLatin1Char('"'), QStringLiteral("\"\""));
  return QStringLiteral("\"%1\"").arg(value);
}

QJsonArray point_to_json(const pcl::PointXYZ & point)
{
  return QJsonArray{point.x, point.y, point.z};
}

bool json_to_point(const QJsonValue & value, pcl::PointXYZ & point)
{
  const QJsonArray array = value.toArray();
  if (array.size() != 3) return false;
  point.x = static_cast<float>(array.at(0).toDouble());
  point.y = static_cast<float>(array.at(1).toDouble());
  point.z = static_cast<float>(array.at(2).toDouble());
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

QString measurement_prefix(int id)
{
  return QStringLiteral("measurement_%1_").arg(id);
}

MeasurementKind measurement_kind_for_mode(int index)
{
  switch (index) {
    case 1: return MeasurementKind::Polyline;
    case 3: return MeasurementKind::Point;
    case 4: return MeasurementKind::Angle;
    case 5: return MeasurementKind::Area;
    case 6: return MeasurementKind::Orthogonal;
    case 7: return MeasurementKind::PointToPlane;
    case 8: return MeasurementKind::Circle;
    default: return MeasurementKind::Segment;
  }
}

QTableWidgetItem * numeric_item(const QString & text)
{
  auto * item = new QTableWidgetItem(text);
  item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return item;
}

QJsonValue finite_json_value(double value)
{
  return std::isfinite(value) ? QJsonValue(value) : QJsonValue();
}

bool copy_file_atomically(
  const QString & source_path,
  const QString & destination_path,
  const QString & description,
  QString * error)
{
  QFile source(source_path);
  QSaveFile destination(destination_path);
  if (!source.open(QIODevice::ReadOnly) || !destination.open(QIODevice::WriteOnly)) {
    if (error) {
      *error = QStringLiteral("无法写入%1：%2").arg(description, destination_path);
    }
    return false;
  }
  constexpr qint64 chunk_size = 1024 * 1024;
  while (!source.atEnd()) {
    const QByteArray chunk = source.read(chunk_size);
    if (chunk.isEmpty() && source.error() != QFileDevice::NoError) {
      destination.cancelWriting();
      if (error) *error = QStringLiteral("读取临时%1失败。").arg(description);
      return false;
    }
    if (destination.write(chunk) != chunk.size()) {
      destination.cancelWriting();
      if (error) *error = QStringLiteral("写入%1失败：%2").arg(description, destination_path);
      return false;
    }
  }
  if (!destination.commit()) {
    if (error) *error = QStringLiteral("提交%1失败：%2").arg(description, destination_path);
    return false;
  }
  return true;
}

bool write_widget_png_atomically(QWidget * widget, const QString & path, QString * error)
{
  if (!widget) {
    if (error) *error = QStringLiteral("没有可截图的界面。");
    return false;
  }
  const QPixmap snapshot = widget->grab();
  if (snapshot.isNull()) {
    if (error) *error = QStringLiteral("无法获取当前界面。");
    return false;
  }
  QByteArray png;
  QBuffer buffer(&png);
  if (!buffer.open(QIODevice::WriteOnly) || !snapshot.save(&buffer, "PNG") || png.isEmpty()) {
    if (error) *error = QStringLiteral("无法编码 PNG 图片。");
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly) || file.write(png) != png.size() || !file.commit()) {
    if (error) *error = QStringLiteral("无法写入 PNG：%1").arg(path);
    return false;
  }
  return true;
}

template<typename Result, typename Function>
Result run_progress_task(QWidget * parent, const QString & title, Function function)
{
  QProgressDialog progress(QStringLiteral("正在计算，请稍候…"), QString(), 0, 0, parent);
  progress.setWindowTitle(title);
  progress.setWindowModality(Qt::WindowModal);
  progress.setCancelButton(nullptr);
  progress.setMinimumDuration(150);
  progress.setAutoClose(false);

  QFutureWatcher<Result> watcher;
  QObject::connect(&watcher, &QFutureWatcher<Result>::finished, &progress, &QDialog::accept);
  watcher.setFuture(QtConcurrent::run(std::move(function)));
  progress.exec();
  watcher.waitForFinished();
  return watcher.result();
}

}  // namespace

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
{
  setAcceptDrops(true);
  setWindowTitle(QStringLiteral("PCD 点云测量工具"));
  setMinimumSize(1100, 700);
  resize(1540, 920);
  picking_tree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB>>();

  build_interface();
  initialize_viewer();

  connect(&load_watcher_, &QFutureWatcher<CloudLoadResult>::finished,
    this, &MainWindow::load_finished);

  statusBar()->showMessage(QStringLiteral("请选择或拖入一个 PCD 文件"));
  auto_recovery_timer_ = new QTimer(this);
  auto_recovery_timer_->setInterval(60000);
  connect(auto_recovery_timer_, &QTimer::timeout, this, &MainWindow::write_auto_recovery);
  auto_recovery_timer_->start();
  if (!QCoreApplication::applicationName().contains(QStringLiteral("Test"), Qt::CaseInsensitive)) {
    QTimer::singleShot(350, this, &MainWindow::maybe_restore_auto_recovery);
  }
}

MainWindow::~MainWindow()
{
  if (load_watcher_.isRunning()) {
    load_watcher_.waitForFinished();
  }
  viewer_.reset();
  render_window_ = nullptr;
  renderer_ = nullptr;
}

void MainWindow::build_interface()
{
  setStyleSheet(QStringLiteral(R"(
    QMainWindow, QWidget#centralSurface, QWidget#viewerPanel { background: #07131D; font-family: 'Noto Sans CJK SC'; }
    QWidget#sidePanel { background: #0A1A25; }
    QDialog { background: #0A1A25; color: #DCE9ED; font-family: 'Noto Sans CJK SC'; }
    QToolBar { background: #081924; border: none; border-bottom: 2px solid #21D4D1; spacing: 5px; padding: 7px 10px; }
    QToolBar QToolButton { background: #112938; color: #E8F2F5; border: 1px solid #29485C; border-radius: 6px; margin: 1px; padding: 7px 11px; font-weight: 600; }
    QToolBar QToolButton:hover { background: #17394A; border-color: #3B7189; color: #FFFFFF; }
    QToolBar QToolButton:pressed, QToolBar QToolButton:checked { background: #0C202C; border-color: #21D4D1; color: #7CE4E1; }
    QToolBar QToolButton:disabled { background: #0C1C26; border-color: #1B303E; color: #526A77; }
    QToolBar QToolButton[role="primary"] { background: #15515B; border-color: #21D4D1; color: #F3FFFF; }
    QToolBar QToolButton[role="primary"]:hover { background: #1C6872; }
    QToolBar::separator { width: 1px; background: #29485C; margin: 6px 7px; }
    QMenu { background: #102534; color: #DCE9ED; border: 1px solid #31546A; padding: 5px; }
    QMenu::item { padding: 7px 24px 7px 10px; border-radius: 4px; }
    QMenu::item:selected { background: #185064; color: #FFFFFF; }
    QGroupBox { background: #102534; color: #DCE9ED; border: 1px solid #29485C; border-radius: 7px; margin-top: 15px; padding: 14px 10px 10px 10px; font-weight: 650; }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 7px; color: #9DB8C5; background: #0A1A25; }
    QLabel { color: #C8D9DF; }
    QPushButton { background: #153142; color: #E4EFF2; border: 1px solid #31546A; border-radius: 6px; padding: 7px 11px; font-weight: 600; }
    QPushButton:hover { background: #1B4053; border-color: #21D4D1; color: #FFFFFF; }
    QPushButton:pressed { background: #0C2634; border-color: #159D9B; padding-top: 8px; padding-bottom: 6px; }
    QPushButton:disabled { color: #59707C; background: #0D202B; border-color: #203847; }
    QPushButton[role="primary"] { background: #185B64; border-color: #21D4D1; color: #F4FFFF; }
    QPushButton[role="primary"]:disabled { color: #59707C; background: #0D202B; border-color: #203847; }
    QPushButton[role="danger"] { color: #FFCBC4; border-color: #75423F; }
    QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit, QTextEdit { background: #0B1E2A; color: #E0EDF1; border: 1px solid #31546A; border-radius: 6px; padding: 5px 9px; min-height: 23px; selection-background-color: #185064; }
    QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover, QTextEdit:hover { border-color: #4B7890; background: #102A39; }
    QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus, QTextEdit:focus { border: 1px solid #21D4D1; }
    QComboBox::drop-down { width: 25px; border-left: 1px solid #29485C; }
    QComboBox QAbstractItemView { background: #102534; color: #E0EDF1; border: 1px solid #31546A; selection-background-color: #185064; outline: none; }
    QCheckBox { color: #C8D9DF; spacing: 8px; padding: 3px 1px; }
    QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #456A7D; border-radius: 4px; background: #0B1E2A; }
    QCheckBox::indicator:hover { border-color: #21D4D1; }
    QCheckBox::indicator:checked { background: #21A9A7; border-color: #52E1DE; }
    QTableWidget { background: #0B1E2A; alternate-background-color: #0E2431; color: #D6E5E9; border: 1px solid #29485C; border-radius: 5px; gridline-color: #203A49; selection-background-color: #185064; selection-color: #FFFFFF; font-family: 'DejaVu Sans Mono'; }
    QHeaderView::section { background: #153142; color: #AAC1CB; padding: 6px; border: none; border-right: 1px solid #29485C; border-bottom: 1px solid #29485C; font-weight: 650; }
    QScrollArea { background: #0A1A25; border: none; }
    QScrollBar:vertical { background: #0A1A25; width: 10px; margin: 0; }
    QScrollBar::handle:vertical { background: #31546A; border-radius: 5px; min-height: 35px; }
    QScrollBar::handle:vertical:hover { background: #46748A; }
    QScrollBar:horizontal { background: #0A1A25; height: 10px; margin: 0; }
    QScrollBar::handle:horizontal { background: #31546A; border-radius: 5px; min-width: 35px; }
    QScrollBar::handle:horizontal:hover { background: #46748A; }
    QScrollBar::add-line, QScrollBar::sub-line { width: 0px; height: 0px; }
    QSplitter::handle { background: #183040; width: 2px; }
    QStatusBar { background: #081924; color: #BBD0D8; border-top: 1px solid #183848; }
    QProgressBar { background: #0B1E2A; border: 1px solid #31546A; border-radius: 4px; }
    QProgressBar::chunk { background: #21D4D1; border-radius: 3px; }
    QToolTip { background: #142E3D; color: #EAF4F6; border: 1px solid #3A657A; padding: 5px; }
  )"));

  auto * toolbar = addToolBar(QStringLiteral("主工具栏"));
  toolbar->setMovable(false);
  toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar->setIconSize(QSize(17, 17));

  QAction * open_action = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogOpenButton), QStringLiteral("打开 PCD"));
  open_action->setShortcut(QKeySequence::Open);
  open_action->setToolTip(QStringLiteral("选择一个 PCD 点云（Ctrl+O）"));
  connect(open_action, &QAction::triggered, this, &MainWindow::choose_file);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(open_action))) {
    button->setProperty("role", "primary");
  }

  recent_menu_ = new QMenu(this);
  QAction * recent_action = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DirOpenIcon), QStringLiteral("最近文件"));
  recent_action->setMenu(recent_menu_);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(recent_action))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }
  rebuild_recent_menu();

  reload_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_BrowserReload), QStringLiteral("重新加载"));
  reload_action_->setShortcut(QKeySequence(Qt::Key_F5));
  reload_action_->setEnabled(false);
  connect(reload_action_, &QAction::triggered, this, &MainWindow::reload_file);

  toolbar->addSeparator();
  QAction * open_project_action = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogOpenButton), QStringLiteral("打开工程"));
  open_project_action->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+O")));
  connect(open_project_action, &QAction::triggered, this, &MainWindow::open_project);

  recent_project_menu_ = new QMenu(this);
  QAction * recent_project_action = toolbar->addAction(
    style()->standardIcon(QStyle::SP_FileDialogListView), QStringLiteral("最近工程"));
  recent_project_action->setMenu(recent_project_menu_);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(recent_project_action))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }
  rebuild_recent_project_menu();

  save_project_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogSaveButton), QStringLiteral("保存工程"));
  save_project_action_->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+S")));
  save_project_action_->setEnabled(false);
  connect(save_project_action_, &QAction::triggered, this, &MainWindow::save_project);

  toolbar->addSeparator();
  output_tools_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogSaveButton), QStringLiteral("导出与报告"));
  output_tools_action_->setEnabled(false);
  auto * output_menu = new QMenu(this);
  screenshot_action_ = output_menu->addAction(
    style()->standardIcon(QStyle::SP_DesktopIcon), QStringLiteral("保存当前视图 PNG…"));
  screenshot_action_->setEnabled(false);
  connect(screenshot_action_, &QAction::triggered, this, &MainWindow::save_screenshot);

  export_action_ = output_menu->addAction(
    style()->standardIcon(QStyle::SP_DialogSaveButton), QStringLiteral("导出测量 CSV / JSON…"));
  export_action_->setEnabled(false);
  connect(export_action_, &QAction::triggered, this, &MainWindow::export_measurements);

  report_action_ = output_menu->addAction(
    style()->standardIcon(QStyle::SP_FileDialogDetailedView), QStringLiteral("生成 PDF 报告…"));
  report_action_->setEnabled(false);
  connect(report_action_, &QAction::triggered, this, &MainWindow::export_pdf_report);
  output_tools_action_->setMenu(output_menu);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(output_tools_action_))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }

  advanced_tools_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_FileDialogContentsView), QStringLiteral("区域与分析"));
  advanced_tools_action_->setEnabled(false);
  auto * advanced_menu = new QMenu(this);
  QAction * exact_crop_action = advanced_menu->addAction(QStringLiteral("精确三维框裁剪…"));
  QAction * height_filter_action = advanced_menu->addAction(QStringLiteral("按高程范围筛选…"));
  QAction * polygon_crop_action = advanced_menu->addAction(QStringLiteral("按选中面积裁剪"));
  QAction * invert_crop_action = advanced_menu->addAction(QStringLiteral("反选当前区域"));
  QAction * export_region_action = advanced_menu->addAction(QStringLiteral("导出当前区域 PCD…"));
  advanced_menu->addSeparator();
  QAction * region_analysis_action = advanced_menu->addAction(QStringLiteral("区域质量与平面…"));
  QAction * histogram_action = advanced_menu->addAction(QStringLiteral("高程直方图…"));
  QAction * profile_action = advanced_menu->addAction(QStringLiteral("沿选中折线提取剖面…"));
  QAction * volume_action = advanced_menu->addAction(QStringLiteral("按选中面积估算体积…"));
  advanced_menu->addSeparator();
  QAction * outlier_action = advanced_menu->addAction(QStringLiteral("统计离群点清理…"));
  QAction * plane_action = advanced_menu->addAction(QStringLiteral("RANSAC 主平面提取…"));
  QAction * records_action = advanced_menu->addAction(QStringLiteral("分析记录…"));
  connect(exact_crop_action, &QAction::triggered, this, &MainWindow::open_exact_crop_dialog);
  connect(height_filter_action, &QAction::triggered, this, &MainWindow::open_height_filter_dialog);
  connect(polygon_crop_action, &QAction::triggered, this, &MainWindow::crop_to_selected_area);
  connect(invert_crop_action, &QAction::triggered, this, &MainWindow::invert_current_crop);
  connect(export_region_action, &QAction::triggered, this, &MainWindow::export_current_region);
  connect(region_analysis_action, &QAction::triggered, this, &MainWindow::show_region_analysis);
  connect(histogram_action, &QAction::triggered, this, &MainWindow::show_height_histogram);
  connect(profile_action, &QAction::triggered, this, &MainWindow::show_elevation_profile);
  connect(volume_action, &QAction::triggered, this, &MainWindow::show_volume_estimation);
  connect(outlier_action, &QAction::triggered, this, &MainWindow::show_outlier_filter);
  connect(plane_action, &QAction::triggered, this, &MainWindow::show_dominant_plane);
  connect(records_action, &QAction::triggered, this, &MainWindow::show_analysis_records);
  advanced_tools_action_->setMenu(advanced_menu);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(advanced_tools_action_))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }

  comparison_tools_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_ArrowRight), QStringLiteral("双云对比"));
  comparison_tools_action_->setEnabled(false);
  auto * comparison_menu = new QMenu(this);
  QAction * open_comparison_action = comparison_menu->addAction(QStringLiteral("加载第二点云并对比…"));
  QAction * comparison_summary_action = comparison_menu->addAction(QStringLiteral("查看对比摘要…"));
  comparison_visibility_action_ = comparison_menu->addAction(QStringLiteral("显示差异热力图"));
  comparison_visibility_action_->setCheckable(true);
  comparison_visibility_action_->setChecked(true);
  comparison_menu->addSeparator();
  QAction * aligned_export_action = comparison_menu->addAction(QStringLiteral("导出对齐后点云…"));
  QAction * distance_export_action = comparison_menu->addAction(QStringLiteral("导出逐点距离 CSV…"));
  QAction * summary_export_action = comparison_menu->addAction(QStringLiteral("导出对比摘要 JSON…"));
  comparison_menu->addSeparator();
  QAction * clear_comparison_action = comparison_menu->addAction(QStringLiteral("清除当前对比"));
  connect(open_comparison_action, &QAction::triggered, this, &MainWindow::open_cloud_comparison);
  connect(comparison_summary_action, &QAction::triggered, this, &MainWindow::show_comparison_summary);
  connect(comparison_visibility_action_, &QAction::triggered,
    this, &MainWindow::toggle_comparison_visibility);
  connect(aligned_export_action, &QAction::triggered, this, &MainWindow::export_aligned_cloud);
  connect(distance_export_action, &QAction::triggered, this, &MainWindow::export_comparison_distances);
  connect(summary_export_action, &QAction::triggered, this, &MainWindow::export_comparison_summary);
  connect(clear_comparison_action, &QAction::triggered, this, &MainWindow::clear_cloud_comparison);
  comparison_tools_action_->setMenu(comparison_menu);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(comparison_tools_action_))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }

  coordinate_tools_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_CommandLink), QStringLiteral("坐标工具"));
  coordinate_tools_action_->setEnabled(false);
  auto * coordinate_menu = new QMenu(this);
  QAction * origin_action = coordinate_menu->addAction(QStringLiteral("设置显示原点…"));
  QAction * transform_action = coordinate_menu->addAction(QStringLiteral("平移 / 旋转 / 缩放…"));
  undo_transform_action_ = coordinate_menu->addAction(QStringLiteral("撤销上次点云变换"));
  undo_transform_action_->setEnabled(false);
  coordinate_menu->addSeparator();
  QAction * export_transform_cloud_action = coordinate_menu->addAction(QStringLiteral("导出当前坐标点云…"));
  QAction * export_matrix_action = coordinate_menu->addAction(QStringLiteral("导出累计变换矩阵…"));
  connect(origin_action, &QAction::triggered, this, &MainWindow::set_display_origin);
  connect(transform_action, &QAction::triggered, this, &MainWindow::open_cloud_transform);
  connect(undo_transform_action_, &QAction::triggered, this, &MainWindow::undo_cloud_transform);
  connect(export_transform_cloud_action, &QAction::triggered, this, &MainWindow::export_transformed_cloud);
  connect(export_matrix_action, &QAction::triggered, this, &MainWindow::export_transform_matrix);
  coordinate_tools_action_->setMenu(coordinate_menu);
  if (auto * button = qobject_cast<QToolButton *>(toolbar->widgetForAction(coordinate_tools_action_))) {
    button->setPopupMode(QToolButton::InstantPopup);
  }

  toolbar->addSeparator();
  QAction * rosbag_action = toolbar->addAction(
    style()->standardIcon(QStyle::SP_MediaPlay), QStringLiteral("ROS Bag"));
  rosbag_action->setObjectName(QStringLiteral("rosbagStudioAction"));
  rosbag_action->setToolTip(QStringLiteral("回放 ROS1/ROS2 bag 并生成离线故障诊断报告"));
  connect(rosbag_action, &QAction::triggered, this,
    [this]() { open_rosbag_dialog(); });

  auto * central = new QWidget;
  central->setObjectName(QStringLiteral("centralSurface"));
  auto * central_layout = new QVBoxLayout(central);
  central_layout->setContentsMargins(8, 8, 8, 8);
  central_layout->setSpacing(6);

  auto * splitter = new QSplitter(Qt::Horizontal);
  splitter->setChildrenCollapsible(false);
  central_layout->addWidget(splitter);

  auto * viewer_panel = new QWidget;
  viewer_panel->setObjectName(QStringLiteral("viewerPanel"));
  auto * viewer_layout = new QVBoxLayout(viewer_panel);
  viewer_layout->setContentsMargins(0, 0, 0, 0);
  viewer_layout->setSpacing(5);

  auto * mode_strip = new QFrame;
  mode_strip->setObjectName(QStringLiteral("modeStrip"));
  mode_strip->setStyleSheet(QStringLiteral(
    "QFrame#modeStrip { background: #102534; border: 1px solid #29485C; border-left: 4px solid #21D4D1; border-radius: 6px; }"
    "QLabel { color: #DCEBED; }"
    "QComboBox { background: #17394A; color: #F3FFFF; border: 1px solid #347086; font-weight: 650; min-width: 132px; }"
    "QComboBox:hover { border-color: #21D4D1; }"));
  auto * mode_strip_layout = new QHBoxLayout(mode_strip);
  mode_strip_layout->setContentsMargins(12, 6, 10, 6);
  mode_strip_layout->setSpacing(10);
  auto * lab_mark = new QLabel(QStringLiteral("PCD // MEASURE CONSOLE"));
  lab_mark->setStyleSheet(QStringLiteral(
    "font-family: 'DejaVu Sans Mono'; font-size: 10px; font-weight: 700; letter-spacing: 1px; color: #7CE4E1;"));
  mode_strip_layout->addWidget(lab_mark);
  interaction_mode_combo_ = new WheelSafeComboBox;
  interaction_mode_combo_->addItems({
    QStringLiteral("两点测距"),
    QStringLiteral("连续折线"),
    QStringLiteral("区域裁剪"),
    QStringLiteral("单点坐标"),
    QStringLiteral("三点角度"),
    QStringLiteral("多边形面积"),
    QStringLiteral("正交分解"),
    QStringLiteral("点到平面"),
    QStringLiteral("圆与直径")});
  connect(interaction_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MainWindow::interaction_mode_changed);
  mode_strip_layout->addWidget(interaction_mode_combo_);
  viewer_hint_label_ = new QLabel(
    QStringLiteral("打开或拖入 PCD 文件；加载后按 Shift + 左键选择点"));
  viewer_hint_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  mode_strip_layout->addWidget(viewer_hint_label_, 1);
  system_status_label_ = new QLabel(QStringLiteral("● READY"));
  system_status_label_->setStyleSheet(QStringLiteral(
    "font-family: 'DejaVu Sans Mono'; color: #55D69E; font-weight: 700;"));
  mode_strip_layout->addWidget(system_status_label_);
  viewer_layout->addWidget(mode_strip);

  vtk_widget_ = new QVTKOpenGLNativeWidget;
  vtk_widget_->setMinimumSize(600, 480);
  vtk_widget_->setFocusPolicy(Qt::StrongFocus);
  viewer_layout->addWidget(vtk_widget_, 1);
  splitter->addWidget(viewer_panel);

  auto * scroll_area = new QScrollArea;
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setMinimumWidth(365);
  scroll_area->setMaximumWidth(470);
  auto * side_panel = new QWidget;
  side_panel->setObjectName(QStringLiteral("sidePanel"));
  auto * side_layout = new QVBoxLayout(side_panel);
  side_layout->setContentsMargins(7, 0, 7, 8);
  side_layout->setSpacing(7);

  auto * file_group = new QGroupBox(QStringLiteral("文件信息"));
  auto * file_form = new QFormLayout(file_group);
  file_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  file_name_label_ = make_value_label(true);
  file_path_label_ = make_value_label();
  file_path_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  file_size_label_ = make_value_label();
  encoding_label_ = make_value_label();
  fields_label_ = make_value_label();
  file_form->addRow(QStringLiteral("文件："), file_name_label_);
  file_form->addRow(QStringLiteral("路径："), file_path_label_);
  file_form->addRow(QStringLiteral("大小："), file_size_label_);
  file_form->addRow(QStringLiteral("编码："), encoding_label_);
  file_form->addRow(QStringLiteral("字段："), fields_label_);
  side_layout->addWidget(file_group);

  auto * points_group = new QGroupBox(QStringLiteral("点云统计"));
  auto * points_form = new QFormLayout(points_group);
  total_points_label_ = make_value_label(true);
  valid_points_label_ = make_value_label();
  invalid_points_label_ = make_value_label();
  colored_points_label_ = make_value_label();
  displayed_points_label_ = make_value_label();
  centroid_label_ = make_value_label();
  lowest_point_label_ = make_value_label();
  highest_point_label_ = make_value_label();
  raw_height_label_ = make_value_label();
  spacing_label_ = make_value_label();
  coordinate_range_label_ = make_value_label();
  points_form->addRow(QStringLiteral("总点数："), total_points_label_);
  points_form->addRow(QStringLiteral("有效点："), valid_points_label_);
  points_form->addRow(QStringLiteral("无效点："), invalid_points_label_);
  points_form->addRow(QStringLiteral("颜色："), colored_points_label_);
  points_form->addRow(QStringLiteral("显示点："), displayed_points_label_);
  points_form->addRow(QStringLiteral("质心："), centroid_label_);
  points_form->addRow(QStringLiteral("最低点："), lowest_point_label_);
  points_form->addRow(QStringLiteral("最高点："), highest_point_label_);
  points_form->addRow(QStringLiteral("原始高度差："), raw_height_label_);
  points_form->addRow(QStringLiteral("点间距估计："), spacing_label_);
  points_form->addRow(QStringLiteral("坐标范围："), coordinate_range_label_);
  side_layout->addWidget(points_group);

  auto * dimensions_group = new QGroupBox(QStringLiteral("推荐尺寸 · 稳健主方向包围盒"));
  auto * dimensions_form = new QFormLayout(dimensions_group);
  major_size_label_ = make_value_label(true);
  minor_size_label_ = make_value_label(true);
  height_label_ = make_value_label(true);
  horizontal_diagonal_label_ = make_value_label();
  diagonal_3d_label_ = make_value_label(true);
  yaw_label_ = make_value_label();
  outlier_ratio_label_ = make_value_label();
  dimensions_form->addRow(QStringLiteral("主方向长度："), major_size_label_);
  dimensions_form->addRow(QStringLiteral("宽度："), minor_size_label_);
  dimensions_form->addRow(QStringLiteral("高度："), height_label_);
  dimensions_form->addRow(QStringLiteral("水平对角线："), horizontal_diagonal_label_);
  dimensions_form->addRow(QStringLiteral("三维对角线："), diagonal_3d_label_);
  dimensions_form->addRow(QStringLiteral("主方向角："), yaw_label_);
  dimensions_form->addRow(QStringLiteral("离群点规则："), outlier_ratio_label_);
  side_layout->addWidget(dimensions_group);

  auto * raw_dimensions_group = new QGroupBox(QStringLiteral("原始坐标轴包围盒"));
  auto * raw_dimensions_layout = new QVBoxLayout(raw_dimensions_group);
  auto * raw_note = new QLabel(QStringLiteral("保留全部有效点，容易受少量飞点影响，仅用于对照。"));
  raw_note->setWordWrap(true);
  raw_note->setStyleSheet(QStringLiteral(
    "color: #8FA9B5; background: #0B1E2A; border-left: 3px solid #57758A; padding: 7px;"));
  raw_dimensions_layout->addWidget(raw_note);
  auto * raw_dimensions_form = new QFormLayout;
  raw_bounds_label_ = make_value_label();
  raw_diagonal_label_ = make_value_label();
  raw_dimensions_form->addRow(QStringLiteral("X × Y × Z："), raw_bounds_label_);
  raw_dimensions_form->addRow(QStringLiteral("三维对角线："), raw_diagonal_label_);
  raw_dimensions_layout->addLayout(raw_dimensions_form);
  side_layout->addWidget(raw_dimensions_group);

  auto * crop_group = new QGroupBox(QStringLiteral("局部区域"));
  auto * crop_layout = new QVBoxLayout(crop_group);
  crop_status_label_ = new QLabel(QStringLiteral("未裁剪 · 选择“区域裁剪”后点两个对角点"));
  crop_status_label_->setWordWrap(true);
  crop_status_label_->setStyleSheet(QStringLiteral(
    "background: #0B2932; color: #8BE5E1; border-left: 3px solid #21D4D1; padding: 8px;"));
  crop_layout->addWidget(crop_status_label_);
  auto * crop_form = new QFormLayout;
  local_points_label_ = make_value_label(true);
  local_size_label_ = make_value_label();
  local_center_label_ = make_value_label();
  local_quality_label_ = make_value_label();
  crop_form->addRow(QStringLiteral("局部点数："), local_points_label_);
  crop_form->addRow(QStringLiteral("局部长宽高："), local_size_label_);
  crop_form->addRow(QStringLiteral("局部质心："), local_center_label_);
  crop_form->addRow(QStringLiteral("密度与平面："), local_quality_label_);
  crop_layout->addLayout(crop_form);
  reset_crop_button_ = new QPushButton(QStringLiteral("恢复完整点云"));
  reset_crop_button_->setEnabled(false);
  connect(reset_crop_button_, &QPushButton::clicked, this, &MainWindow::reset_crop);
  crop_layout->addWidget(reset_crop_button_);
  side_layout->addWidget(crop_group);

  auto * measure_group = new QGroupBox(QStringLiteral("测量与记录"));
  auto * measure_layout = new QVBoxLayout(measure_group);

  picking_instruction_label_ = new QLabel(
    QStringLiteral("按住 Shift 并左键点击点云：先选择 A，再选择 B。"));
  picking_instruction_label_->setWordWrap(true);
  picking_instruction_label_->setStyleSheet(
    QStringLiteral("background: #302817; color: #FFD38A; border-left: 3px solid #FFB547; padding: 8px;"));
  measure_layout->addWidget(picking_instruction_label_);

  auto * measurement_form = new QFormLayout;
  point_a_label_ = make_value_label();
  point_b_label_ = make_value_label();
  delta_label_ = make_value_label();
  distance_3d_label_ = make_value_label(true);
  horizontal_distance_label_ = make_value_label();
  vertical_distance_label_ = make_value_label();
  slope_label_ = make_value_label();
  measurement_result_label_ = make_value_label(true);
  measurement_extra_label_ = make_value_label();
  measurement_metadata_label_ = make_value_label();
  measurement_form->addRow(QStringLiteral("主要结果："), measurement_result_label_);
  measurement_form->addRow(QStringLiteral("补充参数："), measurement_extra_label_);
  measurement_form->addRow(QStringLiteral("点 A："), point_a_label_);
  measurement_form->addRow(QStringLiteral("点 B："), point_b_label_);
  measurement_form->addRow(QStringLiteral("坐标差："), delta_label_);
  measurement_form->addRow(QStringLiteral("三维距离："), distance_3d_label_);
  measurement_form->addRow(QStringLiteral("水平距离："), horizontal_distance_label_);
  measurement_form->addRow(QStringLiteral("高度差："), vertical_distance_label_);
  measurement_form->addRow(QStringLiteral("坡度："), slope_label_);
  measurement_form->addRow(QStringLiteral("记录信息："), measurement_metadata_label_);
  measure_layout->addLayout(measurement_form);

  auto * measure_buttons = new QGridLayout;
  auto * new_button = new QPushButton(QStringLiteral("新测量"));
  new_button->setProperty("role", "primary");
  finish_polyline_button_ = new QPushButton(QStringLiteral("完成"));
  finish_polyline_button_->setProperty("role", "primary");
  finish_polyline_button_->setToolTip(QStringLiteral("保存当前折线（Ctrl+Enter）"));
  finish_polyline_button_->setEnabled(false);
  auto * undo_button = new QPushButton(QStringLiteral("撤销"));
  auto * redo_button = new QPushButton(QStringLiteral("重做"));
  auto * edit_button = new QPushButton(QStringLiteral("编辑记录"));
  auto * delete_button = new QPushButton(QStringLiteral("删除选中"));
  auto * copy_button = new QPushButton(QStringLiteral("复制选中"));
  auto * import_button = new QPushButton(QStringLiteral("导入 JSON"));
  auto * group_visibility_button = new QPushButton(QStringLiteral("切换选中分组显隐"));
  auto * clear_button = new QPushButton(QStringLiteral("全部清除"));
  clear_button->setProperty("role", "danger");
  connect(new_button, &QPushButton::clicked, this, &MainWindow::start_new_measurement);
  connect(finish_polyline_button_, &QPushButton::clicked, this, &MainWindow::finish_polyline);
  connect(undo_button, &QPushButton::clicked, this, &MainWindow::undo_measurement);
  connect(redo_button, &QPushButton::clicked, this, &MainWindow::redo_measurement);
  connect(edit_button, &QPushButton::clicked, this, &MainWindow::edit_selected_measurement);
  connect(delete_button, &QPushButton::clicked, this, &MainWindow::delete_selected_measurement);
  connect(copy_button, &QPushButton::clicked, this, &MainWindow::copy_selected_measurement);
  connect(import_button, &QPushButton::clicked, this, &MainWindow::import_measurements);
  connect(group_visibility_button, &QPushButton::clicked,
    this, &MainWindow::toggle_selected_group_visibility);
  connect(clear_button, &QPushButton::clicked, this, &MainWindow::clear_measurements);
  measure_buttons->addWidget(new_button, 0, 0);
  measure_buttons->addWidget(finish_polyline_button_, 0, 1);
  measure_buttons->addWidget(undo_button, 1, 0);
  measure_buttons->addWidget(redo_button, 1, 1);
  measure_buttons->addWidget(edit_button, 2, 0);
  measure_buttons->addWidget(delete_button, 2, 1);
  measure_buttons->addWidget(copy_button, 3, 0);
  measure_buttons->addWidget(import_button, 3, 1);
  measure_buttons->addWidget(group_visibility_button, 4, 0, 1, 2);
  measure_buttons->addWidget(clear_button, 5, 0, 1, 2);
  measure_layout->addLayout(measure_buttons);
  auto * finish_shortcut = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Return")), this);
  connect(finish_shortcut, &QShortcut::activated, this, &MainWindow::finish_polyline);
  auto * redo_shortcut = new QShortcut(QKeySequence::Redo, this);
  connect(redo_shortcut, &QShortcut::activated, this, &MainWindow::redo_measurement);

  unit_combo_ = new WheelSafeComboBox;
  unit_combo_->addItem(QStringLiteral("米 (m)"), 1.0);
  unit_combo_->addItem(QStringLiteral("厘米 (cm)"), 100.0);
  unit_combo_->addItem(QStringLiteral("毫米 (mm)"), 1000.0);
  connect(unit_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MainWindow::refresh_measurement_units);
  auto * unit_row = new QHBoxLayout;
  unit_row->addWidget(new QLabel(QStringLiteral("显示单位：")));
  unit_row->addWidget(unit_combo_, 1);
  measure_layout->addLayout(unit_row);

  auto * filter_grid = new QGridLayout;
  measurement_search_edit_ = new QLineEdit;
  measurement_search_edit_->setObjectName(QStringLiteral("measurementSearch"));
  measurement_search_edit_->setPlaceholderText(QStringLiteral("搜索名称、分组或备注…"));
  measurement_search_edit_->setClearButtonEnabled(true);
  measurement_type_filter_ = new WheelSafeComboBox;
  measurement_type_filter_->setObjectName(QStringLiteral("measurementTypeFilter"));
  measurement_type_filter_->addItem(QStringLiteral("全部类型"), QString());
  for (MeasurementKind kind : {MeasurementKind::Point, MeasurementKind::Segment,
      MeasurementKind::Polyline, MeasurementKind::Angle, MeasurementKind::Area,
      MeasurementKind::Orthogonal, MeasurementKind::PointToPlane, MeasurementKind::Circle})
  {
    measurement_type_filter_->addItem(measurement_kind_label(kind), measurement_kind_key(kind));
  }
  measurement_group_filter_ = new WheelSafeComboBox;
  measurement_group_filter_->setObjectName(QStringLiteral("measurementGroupFilter"));
  measurement_group_filter_->addItem(QStringLiteral("全部分组"), QString());
  filter_grid->addWidget(measurement_search_edit_, 0, 0, 1, 2);
  filter_grid->addWidget(measurement_type_filter_, 1, 0);
  filter_grid->addWidget(measurement_group_filter_, 1, 1);
  measure_layout->addLayout(filter_grid);
  connect(measurement_search_edit_, &QLineEdit::textChanged, this,
    [this](const QString &) { rebuild_measurement_table(); });
  connect(measurement_type_filter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) { rebuild_measurement_table(); });
  connect(measurement_group_filter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) { rebuild_measurement_table(); });

  measurement_table_ = new QTableWidget(0, 9);
  measurement_table_->setHorizontalHeaderLabels({
    QStringLiteral("#"), QStringLiteral("类型"), QStringLiteral("节点"),
    QStringLiteral("三维"), QStringLiteral("水平"), QStringLiteral("ΔZ"),
    QStringLiteral("主要结果"), QStringLiteral("名称"), QStringLiteral("分组")});
  measurement_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  measurement_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  measurement_table_->setAlternatingRowColors(true);
  measurement_table_->verticalHeader()->setVisible(false);
  measurement_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);
  measurement_table_->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(8, QHeaderView::ResizeToContents);
  measurement_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  measurement_table_->setMinimumHeight(130);
  measure_layout->addWidget(measurement_table_);
  measurement_total_label_ = make_value_label(true);
  measurement_total_label_->setText(QStringLiteral("累计三维长度：0.0000 m"));
  measure_layout->addWidget(measurement_total_label_);
  connect(measurement_table_, &QTableWidget::cellClicked, this,
    [this](int row, int) {
      if (row >= 0 && measurement_table_->item(row, 0)) {
        const int index = measurement_table_->item(row, 0)->data(Qt::UserRole).toInt();
        if (index >= 0 && index < static_cast<int>(measurements_.size())) {
          update_measurement_details(&measurements_[static_cast<std::size_t>(index)]);
        }
      }
    });
  connect(measurement_table_, &QTableWidget::cellDoubleClicked,
    this, &MainWindow::edit_selected_measurement);
  side_layout->addWidget(measure_group);

  auto * view_group = new QGroupBox(QStringLiteral("显示与视角"));
  auto * view_layout = new QVBoxLayout(view_group);
  auto * view_form = new QFormLayout;
  color_mode_combo_ = new WheelSafeComboBox;
  color_mode_combo_->addItems({
    QStringLiteral("RGB 原始颜色"),
    QStringLiteral("按高度渐变"),
    QStringLiteral("浅蓝单色")});
  connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MainWindow::apply_color_mode);
  point_size_spin_ = new QSpinBox;
  point_size_spin_->setRange(1, 10);
  point_size_spin_->setValue(2);
  connect(point_size_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
    this, &MainWindow::apply_point_size);
  display_limit_spin_ = new QSpinBox;
  display_limit_spin_->setObjectName(QStringLiteral("displayPointLimit"));
  display_limit_spin_->setRange(10000, 10000000);
  display_limit_spin_->setSingleStep(50000);
  display_limit_spin_->setValue(std::clamp(
    QSettings().value(QStringLiteral("maximumDisplayPoints"), 750000).toInt(),
    10000, 10000000));
  display_limit_spin_->setSuffix(QStringLiteral(" 点"));
  connect(display_limit_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
    [this](int value) {
      QSettings().setValue(QStringLiteral("maximumDisplayPoints"), value);
      if (!current_.ok()) return;
      rebuild_current_display_cloud(static_cast<std::size_t>(value));
      if (crop_.active) rebuild_crop_display_cloud(static_cast<std::size_t>(value));
      render_cloud(false);
      update_crop_information();
      statusBar()->showMessage(QStringLiteral("显示点上限已更新；统计仍使用完整点云。"), 5000);
    });
  background_combo_ = new WheelSafeComboBox;
  background_combo_->addItem(QStringLiteral("深空蓝"), QStringLiteral("#07131D"));
  background_combo_->addItem(QStringLiteral("石墨黑"), QStringLiteral("#0B0D10"));
  background_combo_->addItem(QStringLiteral("雾灰"), QStringLiteral("#DCE5EA"));
  background_combo_->addItem(QStringLiteral("自定义颜色…"), QString());
  connect(background_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MainWindow::apply_background_mode);
  projection_combo_ = new WheelSafeComboBox;
  projection_combo_->addItems({
    QStringLiteral("透视相机"),
    QStringLiteral("正交相机")});
  connect(projection_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &MainWindow::apply_projection_mode);
  view_form->addRow(QStringLiteral("颜色模式："), color_mode_combo_);
  view_form->addRow(QStringLiteral("点大小："), point_size_spin_);
  view_form->addRow(QStringLiteral("显示点上限："), display_limit_spin_);
  view_form->addRow(QStringLiteral("背景颜色："), background_combo_);
  view_form->addRow(QStringLiteral("相机投影："), projection_combo_);
  view_layout->addLayout(view_form);

  auto * camera_grid = new QGridLayout;
  const std::array<std::pair<QString, void (MainWindow::*)()>, 5> camera_buttons{{
    {QStringLiteral("适应窗口"), &MainWindow::fit_view},
    {QStringLiteral("俯视 XY"), &MainWindow::top_view},
    {QStringLiteral("正视 XZ"), &MainWindow::front_view},
    {QStringLiteral("左视 YZ"), &MainWindow::left_view},
    {QStringLiteral("等轴视图"), &MainWindow::isometric_view}}};
  for (std::size_t i = 0; i < camera_buttons.size(); ++i) {
    auto * button = new QPushButton(camera_buttons[i].first);
    if (i == 0) {
      button->setProperty("role", "primary");
    }
    connect(button, &QPushButton::clicked, this, camera_buttons[i].second);
    camera_grid->addWidget(button, static_cast<int>(i / 2), static_cast<int>(i % 2));
  }
  view_layout->addLayout(camera_grid);

  axes_check_ = new QCheckBox(QStringLiteral("显示坐标轴"));
  axes_check_->setChecked(true);
  grid_check_ = new QCheckBox(QStringLiteral("显示 1 米网格"));
  bounds_check_ = new QCheckBox(QStringLiteral("显示稳健尺寸框"));
  bounds_check_->setChecked(true);
  context_cloud_check_ = new QCheckBox(QStringLiteral("局部模式显示完整云上下文"));
  measurement_labels_check_ = new QCheckBox(QStringLiteral("显示测量文字标签"));
  measurement_labels_check_->setChecked(true);
  connect(axes_check_, &QCheckBox::toggled, this, &MainWindow::toggle_axes);
  connect(grid_check_, &QCheckBox::toggled, this, &MainWindow::toggle_grid);
  connect(bounds_check_, &QCheckBox::toggled, this, &MainWindow::toggle_bounds);
  connect(context_cloud_check_, &QCheckBox::toggled, this,
    [this](bool) { render_context_cloud(); });
  connect(measurement_labels_check_, &QCheckBox::toggled, this,
    [this](bool) {
      render_all_measurements();
      request_render();
    });
  view_layout->addWidget(axes_check_);
  view_layout->addWidget(grid_check_);
  view_layout->addWidget(bounds_check_);
  view_layout->addWidget(context_cloud_check_);
  view_layout->addWidget(measurement_labels_check_);
  side_layout->addWidget(view_group);
  side_layout->addStretch(1);

  scroll_area->setWidget(side_panel);
  splitter->addWidget(scroll_area);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 0);
  splitter->setSizes({1100, 400});
  setCentralWidget(central);

  progress_bar_ = new QProgressBar;
  progress_bar_->setMaximumWidth(190);
  progress_bar_->setTextVisible(false);
  progress_bar_->hide();
  statusBar()->addPermanentWidget(progress_bar_);
}

void MainWindow::initialize_viewer()
{
  renderer_ = vtkSmartPointer<vtkRenderer>::New();
  render_window_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  render_window_->AddRenderer(renderer_);

  viewer_.reset(new pcl::visualization::PCLVisualizer(
    renderer_, render_window_, "PCD Measure", false));
  vtk_widget_->setRenderWindow(render_window_);
  viewer_->setupInteractor(vtk_widget_->interactor(), vtk_widget_->renderWindow());
  viewer_->setBackgroundColor(7.0 / 255.0, 19.0 / 255.0, 29.0 / 255.0);
  viewer_->initCameraParameters();
  viewer_->registerPointPickingCallback(
    [this](const pcl::visualization::PointPickingEvent & event) {
      handle_point_picking(event);
    });
  request_render();
}

void MainWindow::open_path(const QString & path)
{
  if (detect_rosbag_kind(path) != RosbagKind::Unknown) {
    open_rosbag_dialog(path);
    return;
  }
  pending_project_state_ = QJsonObject{};
  pending_project_path_.clear();
  begin_load(path);
}

void MainWindow::open_rosbag_dialog(const QString & path)
{
  auto * dialog = new RosbagDiagnosticDialog(path, this);
  dialog->setAttribute(Qt::WA_DeleteOnClose, true);
  dialog->show();
  dialog->raise();
  dialog->activateWindow();
}

void MainWindow::choose_file()
{
  QSettings settings;
  QString initial_directory = settings.value(QStringLiteral("lastDirectory")).toString();
  if (initial_directory.isEmpty()) {
    initial_directory = QDir::homePath();
  }

  const QString path = QFileDialog::getOpenFileName(
    this, QStringLiteral("选择 PCD 点云"), initial_directory,
    QStringLiteral("PCD 点云 (*.pcd);;所有文件 (*)"));
  if (!path.isEmpty()) {
    pending_project_state_ = QJsonObject{};
    pending_project_path_.clear();
    begin_load(path);
  }
}

void MainWindow::reload_file()
{
  if (!current_path_.isEmpty()) {
    pending_project_state_ = QJsonObject{};
    pending_project_path_.clear();
    begin_load(current_path_);
  }
}

void MainWindow::add_recent_file(const QString & path)
{
  const QString absolute_path = QFileInfo(path).absoluteFilePath();
  QSettings settings;
  QStringList recent = settings.value(QStringLiteral("recentPcdFiles")).toStringList();
  recent.removeAll(absolute_path);
  recent.prepend(absolute_path);
  while (recent.size() > 8) {
    recent.removeLast();
  }
  settings.setValue(QStringLiteral("recentPcdFiles"), recent);
  rebuild_recent_menu();
}

void MainWindow::rebuild_recent_menu()
{
  if (!recent_menu_) {
    return;
  }
  recent_menu_->clear();
  QSettings settings;
  const QStringList stored = settings.value(QStringLiteral("recentPcdFiles")).toStringList();
  QStringList valid;
  for (const QString & stored_path : stored) {
    const QString path = QFileInfo(stored_path).absoluteFilePath();
    if (!QFileInfo(path).isFile() || valid.contains(path)) {
      continue;
    }
    valid.append(path);
    QAction * action = recent_menu_->addAction(QFileInfo(path).fileName());
    action->setToolTip(path);
    connect(action, &QAction::triggered, this, [this, path]() {
      pending_project_state_ = QJsonObject{};
      pending_project_path_.clear();
      begin_load(path);
    });
  }
  settings.setValue(QStringLiteral("recentPcdFiles"), valid);
  if (valid.isEmpty()) {
    QAction * empty_action = recent_menu_->addAction(QStringLiteral("暂无最近文件"));
    empty_action->setEnabled(false);
  }
}

void MainWindow::add_recent_project(const QString & path)
{
  const QString absolute_path = QFileInfo(path).absoluteFilePath();
  QSettings settings;
  QStringList recent = settings.value(QStringLiteral("recentProjectFiles")).toStringList();
  recent.removeAll(absolute_path);
  recent.prepend(absolute_path);
  while (recent.size() > 8) recent.removeLast();
  settings.setValue(QStringLiteral("recentProjectFiles"), recent);
  rebuild_recent_project_menu();
}

void MainWindow::rebuild_recent_project_menu()
{
  if (!recent_project_menu_) return;
  recent_project_menu_->clear();
  QSettings settings;
  const QStringList stored = settings.value(QStringLiteral("recentProjectFiles")).toStringList();
  QStringList valid;
  for (const QString & stored_path : stored) {
    const QString path = QFileInfo(stored_path).absoluteFilePath();
    if (!QFileInfo(path).isFile() || valid.contains(path)) continue;
    valid.append(path);
    QAction * action = recent_project_menu_->addAction(QFileInfo(path).fileName());
    action->setToolTip(path);
    connect(action, &QAction::triggered, this, [this, path]() { open_project_path(path); });
  }
  settings.setValue(QStringLiteral("recentProjectFiles"), valid);
  if (valid.isEmpty()) {
    QAction * empty = recent_project_menu_->addAction(QStringLiteral("暂无最近工程"));
    empty->setEnabled(false);
  }
}

void MainWindow::begin_load(const QString & path)
{
  if (load_watcher_.isRunning()) {
    statusBar()->showMessage(QStringLiteral("正在加载点云，请稍候。"), 3000);
    return;
  }

  const QFileInfo info(path);
  if (!info.exists() || !info.isFile()) {
    QMessageBox::warning(this, QStringLiteral("文件不存在"),
      QStringLiteral("找不到文件：\n%1").arg(path));
    return;
  }
  if (info.suffix().compare(QStringLiteral("pcd"), Qt::CaseInsensitive) != 0) {
    QMessageBox::warning(this, QStringLiteral("格式不支持"),
      QStringLiteral("请选择扩展名为 .pcd 的点云文件。"));
    return;
  }

  current_path_ = info.absoluteFilePath();
  QSettings().setValue(QStringLiteral("lastDirectory"), info.absolutePath());
  load_timer_.restart();
  file_name_label_->setText(QStringLiteral("%1  ·  加载中").arg(info.fileName()));
  file_path_label_->setText(info.absolutePath());
  setWindowTitle(QStringLiteral("正在加载 %1 — PCD 点云测量工具").arg(info.fileName()));
  set_loading(true, QStringLiteral("正在读取并分析 %1 …").arg(info.fileName()));

  const QString worker_path = current_path_;
  const std::size_t maximum_display_points = display_limit_spin_ ?
    static_cast<std::size_t>(display_limit_spin_->value()) : 750000;
  load_watcher_.setFuture(QtConcurrent::run([worker_path, maximum_display_points]() {
    try {
      return load_pcd_and_analyze(worker_path, maximum_display_points);
    } catch (const std::exception & error) {
      CloudLoadResult result;
      result.path = worker_path;
      result.error = QStringLiteral("读取点云时发生异常：%1").arg(QString::fromLocal8Bit(error.what()));
      return result;
    } catch (...) {
      CloudLoadResult result;
      result.path = worker_path;
      result.error = QStringLiteral("读取点云时发生未知异常。");
      return result;
    }
  }));
}

void MainWindow::load_finished()
{
  const CloudLoadResult result = load_watcher_.result();
  set_loading(false);
  if (!result.ok()) {
    const bool has_cloud = current_.ok();
    pending_project_state_ = QJsonObject{};
    pending_project_path_.clear();
    restoring_auto_recovery_ = false;
    current_path_ = has_cloud ? current_.path : QString();
    reload_action_->setEnabled(has_cloud);
    screenshot_action_->setEnabled(has_cloud);
    export_action_->setEnabled(has_cloud);
    report_action_->setEnabled(has_cloud);
    output_tools_action_->setEnabled(has_cloud);
    advanced_tools_action_->setEnabled(has_cloud);
    comparison_tools_action_->setEnabled(has_cloud);
    coordinate_tools_action_->setEnabled(has_cloud);
    save_project_action_->setEnabled(has_cloud);
    if (has_cloud) {
      fill_information_panel();
      setWindowTitle(QStringLiteral("%1 — PCD 点云测量工具")
        .arg(QFileInfo(current_.path).fileName()));
    }
    QMessageBox::critical(this, QStringLiteral("PCD 加载失败"), result.error);
    system_status_label_->setText(has_cloud ?
      QStringLiteral("● READY · LOAD ERROR") : QStringLiteral("● ERROR"));
    system_status_label_->setStyleSheet(QStringLiteral(
      "font-family: 'DejaVu Sans Mono'; color: %1; font-weight: 700;")
      .arg(has_cloud ? QStringLiteral("#FFB547") : QStringLiteral("#FF8075")));
    statusBar()->showMessage(QStringLiteral("加载失败：%1").arg(result.error), 8000);
    return;
  }

  clear_measurements();
  undo_history_.clear();
  redo_history_.clear();
  analysis_records_.clear();
  comparison_ = CloudComparisonState{};
  comparison_visibility_action_->setChecked(true);
  display_origin_ = {0.0, 0.0, 0.0};
  cumulative_transform_ = Eigen::Matrix4f::Identity();
  transform_preview_display_.reset();
  transform_backup_valid_ = false;
  transform_backup_cloud_ = CloudLoadResult{};
  transform_backup_measurements_.clear();
  undo_transform_action_->setEnabled(false);
  viewer_->removeAllPointClouds();
  viewer_->removeAllShapes();
  viewer_->removeCoordinateSystem(kAxesId);
  grid_shape_ids_.clear();
  bounds_shape_ids_.clear();
  crop_shape_ids_.clear();
  crop_ = CropRegion{};

  current_ = result;
  current_path_ = result.path;
  update_picking_tree();
  color_mode_combo_->blockSignals(true);
  color_mode_combo_->setCurrentIndex(
    (result.metrics.has_rgb || result.metrics.has_rgba) ? 0 : 1);
  color_mode_combo_->blockSignals(false);

  fill_information_panel();
  render_cloud(true);
  toggle_axes(axes_check_->isChecked());
  update_grid_overlay();
  update_bounds_overlay();
  update_crop_information();

  reload_action_->setEnabled(true);
  screenshot_action_->setEnabled(true);
  export_action_->setEnabled(true);
  report_action_->setEnabled(true);
  output_tools_action_->setEnabled(true);
  advanced_tools_action_->setEnabled(true);
  comparison_tools_action_->setEnabled(true);
  coordinate_tools_action_->setEnabled(true);
  save_project_action_->setEnabled(true);
  add_recent_file(result.path);
  interaction_mode_changed(interaction_mode_combo_->currentIndex());
  setWindowTitle(QStringLiteral("%1 — PCD 点云测量工具").arg(QFileInfo(result.path).fileName()));
  system_status_label_->setText(QStringLiteral("● READY"));
  system_status_label_->setStyleSheet(QStringLiteral(
    "font-family: 'DejaVu Sans Mono'; color: #55D69E; font-weight: 700;"));

  const double elapsed_seconds = static_cast<double>(load_timer_.elapsed()) / 1000.0;
  statusBar()->showMessage(
    QStringLiteral("已加载 %1 个有效点，用时 %2 秒。Shift + 左键开始测距。")
      .arg(count_text(result.metrics.finite_points))
      .arg(elapsed_seconds, 0, 'f', 2),
    10000);
  restore_pending_project();
  request_render();
  emit cloudLoaded(true);
}

void MainWindow::fill_information_panel()
{
  const QFileInfo info(current_.path);
  const CloudMetrics & metrics = current_.metrics;
  const Bounds3d & raw = metrics.raw_bounds;
  const OrientedBounds & oriented = metrics.oriented;

  file_name_label_->setText(info.fileName());
  file_path_label_->setText(info.absolutePath());
  file_path_label_->setToolTip(current_.path);
  file_size_label_->setText(bytes_text(current_.file_bytes));
  encoding_label_->setText(current_.encoding);
  fields_label_->setText(current_.fields);

  total_points_label_->setText(count_text(metrics.header_points));
  valid_points_label_->setText(count_text(metrics.finite_points));
  invalid_points_label_->setText(count_text(metrics.invalid_points));

  if (metrics.has_rgb || metrics.has_rgba) {
    const double percentage = metrics.finite_points > 0 ?
      100.0 * static_cast<double>(metrics.non_black_points) /
      static_cast<double>(metrics.finite_points) : 0.0;
    colored_points_label_->setText(
      QStringLiteral("%1；非黑色 %2（%3%）")
        .arg(metrics.has_rgba && !metrics.has_rgb ? QStringLiteral("RGBA") : QStringLiteral("RGB"))
        .arg(count_text(metrics.non_black_points))
        .arg(percentage, 0, 'f', 2));
  } else {
    colored_points_label_->setText(QStringLiteral("无颜色字段（使用高度着色）"));
  }

  displayed_points_label_->setText(
    metrics.display_downsampled ?
    QStringLiteral("%1（仅显示降采样，统计使用全部点）").arg(count_text(metrics.displayed_points)) :
    QStringLiteral("%1（完整显示）").arg(count_text(metrics.displayed_points)));
  centroid_label_->setText(
    QStringLiteral("%1, %2, %3 m")
      .arg(metrics.centroid[0] - display_origin_[0], 0, 'f', 3)
      .arg(metrics.centroid[1] - display_origin_[1], 0, 'f', 3)
      .arg(metrics.centroid[2] - display_origin_[2], 0, 'f', 3));
  lowest_point_label_->setText(
    QStringLiteral("X %1  Y %2  Z %3 m")
      .arg(metrics.lowest_point[0] - display_origin_[0], 0, 'f', 3)
      .arg(metrics.lowest_point[1] - display_origin_[1], 0, 'f', 3)
      .arg(metrics.lowest_point[2] - display_origin_[2], 0, 'f', 3));
  highest_point_label_->setText(
    QStringLiteral("X %1  Y %2  Z %3 m")
      .arg(metrics.highest_point[0] - display_origin_[0], 0, 'f', 3)
      .arg(metrics.highest_point[1] - display_origin_[1], 0, 'f', 3)
      .arg(metrics.highest_point[2] - display_origin_[2], 0, 'f', 3));
  raw_height_label_->setText(QStringLiteral("%1 m").arg(raw.size_z(), 0, 'f', 3));
  spacing_label_->setText(std::isfinite(metrics.estimated_spacing) ?
    QStringLiteral("%1 m（中位最近邻）").arg(metrics.estimated_spacing, 0, 'f', 4) :
    QStringLiteral("无法估算"));
  coordinate_range_label_->setText(
    QStringLiteral("X [%1, %2]\nY [%3, %4]\nZ [%5, %6] m")
      .arg(raw.min_x - display_origin_[0], 0, 'f', 3)
      .arg(raw.max_x - display_origin_[0], 0, 'f', 3)
      .arg(raw.min_y - display_origin_[1], 0, 'f', 3)
      .arg(raw.max_y - display_origin_[1], 0, 'f', 3)
      .arg(raw.min_z - display_origin_[2], 0, 'f', 3)
      .arg(raw.max_z - display_origin_[2], 0, 'f', 3));

  major_size_label_->setText(QStringLiteral("%1 m").arg(oriented.major_size, 0, 'f', 3));
  minor_size_label_->setText(QStringLiteral("%1 m").arg(oriented.minor_size, 0, 'f', 3));
  height_label_->setText(QStringLiteral("%1 m").arg(oriented.height, 0, 'f', 3));
  horizontal_diagonal_label_->setText(
    QStringLiteral("%1 m").arg(oriented.horizontal_diagonal, 0, 'f', 3));
  diagonal_3d_label_->setText(QStringLiteral("%1 m").arg(oriented.diagonal_3d, 0, 'f', 3));
  yaw_label_->setText(QStringLiteral("%1°（相对 X 轴）").arg(oriented.yaw_degrees, 0, 'f', 2));
  const double pca_excluded_percent = metrics.finite_points > 0 ?
    100.0 * static_cast<double>(metrics.finite_points - oriented.points_used) /
      static_cast<double>(metrics.finite_points) : 0.0;
  outlier_ratio_label_->setText(
    QStringLiteral("每方向两端各 0.5%（合计 1.0%/轴）\n"
      "PCA 预清理 %1% · 保留 %2 点")
      .arg(pca_excluded_percent, 0, 'f', 2)
      .arg(count_text(oriented.points_used)));
  raw_bounds_label_->setText(
    QStringLiteral("%1 × %2 × %3 m")
      .arg(raw.size_x(), 0, 'f', 3)
      .arg(raw.size_y(), 0, 'f', 3)
      .arg(raw.size_z(), 0, 'f', 3));
  raw_diagonal_label_->setText(QStringLiteral("%1 m").arg(raw.diagonal(), 0, 'f', 3));
}

void MainWindow::render_cloud(bool reset_camera)
{
  if (!current_.ok() || !current_.display_cloud) {
    return;
  }

  const auto cloud_to_render = active_display_cloud();
  if (!cloud_to_render || cloud_to_render->empty()) {
    return;
  }
  viewer_->removePointCloud(kCloudId);
  bool added = false;
  const int mode = color_mode_combo_->currentIndex();
  if (mode == 0 && (current_.metrics.has_rgb || current_.metrics.has_rgba)) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(
      cloud_to_render);
    added = viewer_->addPointCloud<pcl::PointXYZRGB>(
      cloud_to_render, handler, kCloudId);
  } else if (mode == 1) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> handler(
      cloud_to_render, "z");
    added = viewer_->addPointCloud<pcl::PointXYZRGB>(
      cloud_to_render, handler, kCloudId);
  } else {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> handler(
      cloud_to_render, 115, 190, 238);
    added = viewer_->addPointCloud<pcl::PointXYZRGB>(
      cloud_to_render, handler, kCloudId);
  }

  if (!added) {
    statusBar()->showMessage(QStringLiteral("当前颜色模式不可用，已使用单色显示。"), 5000);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fallback(
      cloud_to_render, 115, 190, 238);
    viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_to_render, fallback, kCloudId);
  }
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    static_cast<double>(point_size_spin_->value()), kCloudId);

  render_context_cloud();
  render_cloud_comparison();

  if (reset_camera) {
    viewer_->resetCamera();
  }
  request_render();
}

void MainWindow::render_context_cloud()
{
  if (!viewer_) return;
  viewer_->removePointCloud(kContextCloudId);
  if (!crop_.active || !context_cloud_check_ || !context_cloud_check_->isChecked() ||
    !current_.display_cloud || current_.display_cloud->empty())
  {
    request_render();
    return;
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> handler(
    current_.display_cloud, 74, 113, 132);
  if (viewer_->addPointCloud<pcl::PointXYZRGB>(current_.display_cloud, handler, kContextCloudId)) {
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      static_cast<double>(std::max(1, point_size_spin_->value() - 1)), kContextCloudId);
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.16, kContextCloudId);
  }
  request_render();
}

void MainWindow::apply_color_mode()
{
  render_cloud(false);
}

void MainWindow::apply_background_mode(int index)
{
  if (!viewer_ || !background_combo_) {
    return;
  }

  QString color_hex;
  if (index == 3) {
    if (sender() == background_combo_) {
      const QColor selected = QColorDialog::getColor(
        QColor(custom_background_hex_), this, QStringLiteral("选择点云背景颜色"));
      if (!selected.isValid()) {
        background_combo_->blockSignals(true);
        background_combo_->setCurrentIndex(last_background_index_);
        background_combo_->blockSignals(false);
        return;
      }
      custom_background_hex_ = selected.name(QColor::HexRgb).toUpper();
      background_combo_->setItemText(3,
        QStringLiteral("自定义 · %1").arg(custom_background_hex_));
    }
    color_hex = custom_background_hex_;
  } else {
    color_hex = background_combo_->itemData(index).toString();
  }

  const QColor color(color_hex);
  if (!color.isValid()) {
    return;
  }
  last_background_index_ = index;
  viewer_->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
  request_render();
}

void MainWindow::apply_projection_mode(int index)
{
  if (!renderer_) {
    return;
  }
  vtkCamera * camera = renderer_->GetActiveCamera();
  if (!camera) {
    return;
  }

  const bool orthographic = index == 1;
  if (orthographic && !camera->GetParallelProjection()) {
    const double half_angle_radians = camera->GetViewAngle() * std::acos(-1.0) / 360.0;
    camera->SetParallelScale(std::max(0.01, camera->GetDistance() * std::tan(half_angle_radians)));
  }
  camera->SetParallelProjection(orthographic ? 1 : 0);
  renderer_->ResetCameraClippingRange();
  statusBar()->showMessage(
    orthographic ? QStringLiteral("已切换为正交相机。") : QStringLiteral("已切换为透视相机。"),
    3000);
  request_render();
}

void MainWindow::apply_point_size(int size)
{
  if (!current_.ok() || !viewer_->contains(kCloudId)) {
    return;
  }
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, static_cast<double>(size), kCloudId);
  if (viewer_->contains(kComparisonCloudId)) {
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      static_cast<double>(size + 1), kComparisonCloudId);
  }
  if (viewer_->contains(kContextCloudId)) {
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      static_cast<double>(std::max(1, size - 1)), kContextCloudId);
  }
  request_render();
}

void MainWindow::update_picking_tree()
{
  if (!picking_tree_) {
    picking_tree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB>>();
  }
  const auto cloud = crop_.active && crop_.full_cloud ? crop_.full_cloud : current_.cloud;
  if (cloud && !cloud->empty()) {
    picking_tree_->setInputCloud(cloud);
  }
}

pcl::PointXYZ MainWindow::snap_to_nearest_full_point(const pcl::PointXYZ & picked) const
{
  const auto cloud = crop_.active && crop_.full_cloud ? crop_.full_cloud : current_.cloud;
  if (!picking_tree_ || !cloud || cloud->empty()) {
    return picked;
  }

  pcl::PointXYZRGB query;
  query.x = picked.x;
  query.y = picked.y;
  query.z = picked.z;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  if (picking_tree_->nearestKSearch(query, 1, indices, squared_distances) != 1 ||
    indices.front() < 0 || static_cast<std::size_t>(indices.front()) >= cloud->size())
  {
    return picked;
  }

  const pcl::PointXYZRGB & nearest = (*cloud)[static_cast<std::size_t>(indices.front())];
  return pcl::PointXYZ(nearest.x, nearest.y, nearest.z);
}

void MainWindow::handle_point_picking(const pcl::visualization::PointPickingEvent & event)
{
  if (!current_.ok()) {
    return;
  }
  if (event.getPointIndex() < 0) {
    statusBar()->showMessage(QStringLiteral("没有选中点，请放大后再点击。"), 3000);
    return;
  }

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  event.getPoint(x, y, z);
  const pcl::PointXYZ point = snap_to_nearest_full_point(pcl::PointXYZ(x, y, z));
  switch (interaction_mode_combo_->currentIndex()) {
    case 1:
      accept_polyline_point(point);
      break;
    case 2:
      accept_crop_point(point);
      break;
    case 0:
      accept_picked_point(point);
      break;
    default:
      accept_generic_measurement_point(point);
      break;
  }
}

void MainWindow::accept_picked_point(const pcl::PointXYZ & point)
{
  if (!pending_point_) {
    pending_point_ = point;
    viewer_->removeShape(kPendingPointId);
    viewer_->addSphere(point, marker_radius(), 1.0, 0.35, 0.15, kPendingPointId);
    measurement_result_label_->setText(QStringLiteral("等待 B 点…"));
    measurement_extra_label_->setText(QStringLiteral("A 点已吸附到最近的真实点云点"));
    measurement_metadata_label_->setText(QStringLiteral("—"));
    point_a_label_->setText(format_coordinate(point));
    point_b_label_->setText(QStringLiteral("等待选择 B 点…"));
    delta_label_->setText(QStringLiteral("—"));
    distance_3d_label_->setText(QStringLiteral("—"));
    horizontal_distance_label_->setText(QStringLiteral("—"));
    vertical_distance_label_->setText(QStringLiteral("—"));
    slope_label_->setText(QStringLiteral("—"));
    picking_instruction_label_->setText(
      QStringLiteral("A 点已选择。继续按 Shift + 左键选择 B 点。"));
    statusBar()->showMessage(QStringLiteral("已选择 A 点，请选择 B 点。"), 5000);
    request_render();
    return;
  }

  complete_measurement(point);
}

void MainWindow::complete_measurement(const pcl::PointXYZ & point_b)
{
  const MeasurementRecord record = make_measurement_record(
    MeasurementKind::Segment, {*pending_point_, point_b}, next_measurement_id_);

  viewer_->removeShape(kPendingPointId);
  pending_point_.reset();
  commit_measurement(record);
  picking_instruction_label_->setText(
    QStringLiteral("测量完成。再次 Shift + 左键可开始下一组测量。"));
  statusBar()->showMessage(
    QStringLiteral("测量 #%1：三维距离 %2").arg(record.id).arg(format_distance(record.distance_3d)),
    8000);
  request_render();
}

MeasurementRecord MainWindow::make_measurement_record(
  MeasurementKind kind,
  const std::vector<pcl::PointXYZ> & vertices,
  int id) const
{
  return calculate_measurement(kind, vertices, id);
}

void MainWindow::accept_polyline_point(const pcl::PointXYZ & point)
{
  accept_generic_measurement_point(point);
}

void MainWindow::accept_generic_measurement_point(const pcl::PointXYZ & point)
{
  const MeasurementKind kind = measurement_kind_for_mode(interaction_mode_combo_->currentIndex());
  active_polyline_points_.push_back(point);
  render_active_polyline();

  const int minimum = measurement_minimum_points(kind);
  const int required = measurement_required_points(kind);
  finish_polyline_button_->setEnabled(
    measurement_requires_manual_finish(kind) &&
    static_cast<int>(active_polyline_points_.size()) >= minimum);

  if (required > 0 && static_cast<int>(active_polyline_points_.size()) >= required) {
    MeasurementRecord record = make_measurement_record(kind, active_polyline_points_, next_measurement_id_);
    if (!record.valid) {
      statusBar()->showMessage(record.error, 6000);
      picking_instruction_label_->setText(record.error + QStringLiteral(" 请撤销选点后重试。"));
      return;
    }
    for (const std::string & id : active_shape_ids_) {
      viewer_->removeShape(id);
    }
    active_shape_ids_.clear();
    active_polyline_points_.clear();
    commit_measurement(record);
    interaction_mode_changed(interaction_mode_combo_->currentIndex());
    return;
  }

  if (static_cast<int>(active_polyline_points_.size()) >= minimum) {
    const MeasurementRecord preview = make_measurement_record(kind, active_polyline_points_, 0);
    if (preview.valid) {
      update_measurement_details(&preview);
    }
  }
  const QString target = required > 0 ? QString::number(required) : QStringLiteral("至少 %1").arg(minimum);
  picking_instruction_label_->setText(
    QStringLiteral("%1：已选 %2 / %3 个点。%4")
      .arg(measurement_kind_label(kind))
      .arg(active_polyline_points_.size())
      .arg(target)
      .arg(measurement_requires_manual_finish(kind) ?
        QStringLiteral("继续选点或点击“完成”。") : QStringLiteral("继续选择下一点。")));
  statusBar()->showMessage(
    QStringLiteral("%1 · 已选 %2 个点")
      .arg(measurement_kind_label(kind)).arg(active_polyline_points_.size()), 5000);
}

void MainWindow::render_active_polyline()
{
  for (const std::string & id : active_shape_ids_) {
    viewer_->removeShape(id);
  }
  active_shape_ids_.clear();
  if (active_polyline_points_.empty()) {
    request_render();
    return;
  }

  const double radius = marker_radius();
  const MeasurementKind kind = measurement_kind_for_mode(interaction_mode_combo_->currentIndex());
  for (std::size_t i = 0; i < active_polyline_points_.size(); ++i) {
    const std::string node_id = "active_poly_node_" + std::to_string(i);
    viewer_->addSphere(active_polyline_points_[i], radius, 0.12, 0.82, 0.78, node_id);
    active_shape_ids_.push_back(node_id);
    if (i > 0) {
      const std::string line_id = "active_poly_line_" + std::to_string(i - 1);
      viewer_->addLine(active_polyline_points_[i - 1], active_polyline_points_[i],
        0.95, 0.66, 0.18, line_id);
      viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, line_id);
      active_shape_ids_.push_back(line_id);
    }
  }

  if (kind == MeasurementKind::Area && active_polyline_points_.size() >= 3) {
    const std::string close_id = "active_poly_close";
    viewer_->addLine(active_polyline_points_.back(), active_polyline_points_.front(),
      0.95, 0.66, 0.18, close_id);
    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, close_id);
    active_shape_ids_.push_back(close_id);
  }

  if (static_cast<int>(active_polyline_points_.size()) >= measurement_minimum_points(kind)) {
    const MeasurementRecord preview = make_measurement_record(
      kind, active_polyline_points_, 0);
    pcl::PointXYZ label_point = active_polyline_points_.back();
    label_point.z += static_cast<float>(radius * 1.6);
    const std::string text_id = "active_poly_text";
    viewer_->addText3D(
      measurement_primary_text(preview).toStdString(),
      label_point, std::clamp(radius * 1.5, 0.045, 0.18), 0.96, 0.76, 0.24, text_id);
    active_shape_ids_.push_back(text_id);
  }
  request_render();
}

void MainWindow::finish_polyline()
{
  const MeasurementKind kind = measurement_kind_for_mode(interaction_mode_combo_->currentIndex());
  if (!measurement_requires_manual_finish(kind)) {
    statusBar()->showMessage(QStringLiteral("当前测量会在选够点后自动完成。"), 4000);
    return;
  }
  const int minimum = measurement_minimum_points(kind);
  if (static_cast<int>(active_polyline_points_.size()) < minimum) {
    statusBar()->showMessage(
      QStringLiteral("%1至少需要 %2 个点。").arg(measurement_kind_label(kind)).arg(minimum), 4000);
    return;
  }

  const MeasurementRecord record = make_measurement_record(
    kind, active_polyline_points_, next_measurement_id_);
  if (!record.valid) {
    statusBar()->showMessage(record.error, 6000);
    return;
  }
  for (const std::string & id : active_shape_ids_) {
    viewer_->removeShape(id);
  }
  active_shape_ids_.clear();
  active_polyline_points_.clear();
  finish_polyline_button_->setEnabled(false);
  commit_measurement(record);
  picking_instruction_label_->setText(
    QStringLiteral("%1已保存。Shift + 左键可开始下一条。")
      .arg(measurement_kind_label(kind)));
  statusBar()->showMessage(
    QStringLiteral("%1 #%2：%3")
      .arg(measurement_kind_label(kind)).arg(record.id).arg(measurement_primary_text(record)), 8000);
  request_render();
}

void MainWindow::commit_measurement(const MeasurementRecord & record)
{
  if (!record.valid) {
    statusBar()->showMessage(record.error, 6000);
    return;
  }
  push_measurement_history();
  measurements_.push_back(record);
  next_measurement_id_ = std::max(next_measurement_id_, record.id + 1);
  render_measurement(record);
  rebuild_measurement_table();
  update_measurement_details(&measurements_.back());
  for (int row = 0; row < measurement_table_->rowCount(); ++row) {
    if (measurement_table_->item(row, 0)->data(Qt::UserRole).toInt() ==
      static_cast<int>(measurements_.size()) - 1)
    {
      measurement_table_->selectRow(row);
      break;
    }
  }
  request_render();
}

void MainWindow::render_measurement(const MeasurementRecord & record)
{
  const std::string prefix = measurement_prefix(record.id).toStdString();
  remove_measurement_shapes(record.id);
  std::vector<std::string> shape_ids;
  measurement_shape_ids_[record.id] = shape_ids;
  if (!record.visible || record.vertices.empty()) {
    return;
  }

  const double radius = marker_radius();
  const QColor color(record.color_hex);
  const double red = color.isValid() ? color.redF() : 1.0;
  const double green = color.isValid() ? color.greenF() : 0.70;
  const double blue = color.isValid() ? color.blueF() : 0.16;
  auto add_sphere = [&](const pcl::PointXYZ & point, const std::string & suffix,
      double r, double g, double b) {
      const std::string id = prefix + suffix;
      viewer_->addSphere(point, radius, r, g, b, id);
      shape_ids.push_back(id);
    };
  auto add_line = [&](const pcl::PointXYZ & first, const pcl::PointXYZ & second,
      const std::string & suffix, double width = 3.0) {
      const std::string id = prefix + suffix;
      viewer_->addLine(first, second, red, green, blue, id);
      viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, width, id);
      shape_ids.push_back(id);
    };

  for (std::size_t i = 0; i < record.vertices.size(); ++i) {
    const bool first = i == 0;
    const bool last = i + 1 == record.vertices.size();
    add_sphere(record.vertices[i], "node_" + std::to_string(i),
      first ? 1.0 : red,
      first ? 0.22 : (last ? 0.92 : green),
      first ? 0.14 : blue);
  }

  switch (record.kind) {
    case MeasurementKind::Point:
      break;
    case MeasurementKind::Area:
      for (std::size_t i = 0; i < record.vertices.size(); ++i) {
        add_line(record.vertices[i], record.vertices[(i + 1) % record.vertices.size()],
          "edge_" + std::to_string(i));
      }
      break;
    case MeasurementKind::Angle:
      add_line(record.vertices[0], record.vertices[1], "angle_first");
      add_line(record.vertices[1], record.vertices[2], "angle_second");
      break;
    case MeasurementKind::Orthogonal: {
      const pcl::PointXYZ & first = record.vertices.front();
      const pcl::PointXYZ & last = record.vertices.back();
      const pcl::PointXYZ x_corner(last.x, first.y, first.z);
      const pcl::PointXYZ xy_corner(last.x, last.y, first.z);
      add_line(first, x_corner, "ortho_x");
      add_line(x_corner, xy_corner, "ortho_y");
      add_line(xy_corner, last, "ortho_z");
      break;
    }
    case MeasurementKind::PointToPlane: {
      add_line(record.vertices[1], record.vertices[2], "plane_0", 2.0);
      add_line(record.vertices[2], record.vertices[3], "plane_1", 2.0);
      add_line(record.vertices[3], record.vertices[1], "plane_2", 2.0);
      const pcl::PointXYZ projection(
        static_cast<float>(record.center[0]),
        static_cast<float>(record.center[1]),
        static_cast<float>(record.center[2]));
      add_sphere(projection, "projection", 0.20, 0.95, 0.95);
      add_line(record.vertices[0], projection, "perpendicular", 4.0);
      break;
    }
    case MeasurementKind::Circle: {
      const Eigen::Vector3d center(record.center[0], record.center[1], record.center[2]);
      Eigen::Vector3d axis_u = Eigen::Vector3d(
        record.vertices[0].x, record.vertices[0].y, record.vertices[0].z) - center;
      if (axis_u.norm() > 1e-12) {
        axis_u.normalize();
        const Eigen::Vector3d normal(record.normal[0], record.normal[1], record.normal[2]);
        const Eigen::Vector3d axis_v = normal.cross(axis_u).normalized();
        constexpr int segments = 64;
        for (int i = 0; i < segments; ++i) {
          const double first_angle = 2.0 * std::acos(-1.0) * i / segments;
          const double second_angle = 2.0 * std::acos(-1.0) * (i + 1) / segments;
          const Eigen::Vector3d first = center + record.radius *
            (std::cos(first_angle) * axis_u + std::sin(first_angle) * axis_v);
          const Eigen::Vector3d second = center + record.radius *
            (std::cos(second_angle) * axis_u + std::sin(second_angle) * axis_v);
          add_line(
            pcl::PointXYZ(first.x(), first.y(), first.z()),
            pcl::PointXYZ(second.x(), second.y(), second.z()),
            "circle_" + std::to_string(i), 2.0);
        }
        const pcl::PointXYZ center_point(center.x(), center.y(), center.z());
        add_sphere(center_point, "center", 0.20, 0.95, 0.95);
        add_line(center_point, record.vertices[0], "radius", 2.0);
      }
      break;
    }
    case MeasurementKind::Segment:
    case MeasurementKind::Polyline:
      for (std::size_t i = 1; i < record.vertices.size(); ++i) {
        add_line(record.vertices[i - 1], record.vertices[i],
          "line_" + std::to_string(i - 1));
      }
      break;
  }

  pcl::PointXYZ label_point = record.point_b;
  if (record.kind == MeasurementKind::Area || record.kind == MeasurementKind::Circle ||
    record.kind == MeasurementKind::PointToPlane)
  {
    label_point = pcl::PointXYZ(
      static_cast<float>(record.center[0]),
      static_cast<float>(record.center[1]),
      static_cast<float>(record.center[2]));
  } else if (record.kind == MeasurementKind::Angle) {
    label_point = record.vertices[1];
  }
  if (!measurement_labels_check_ || measurement_labels_check_->isChecked()) {
    label_point.z += static_cast<float>(radius * 1.6);
    const std::string text_id = prefix + "text";
    viewer_->addText3D(
      QStringLiteral("#%1 %2").arg(record.id).arg(measurement_primary_text(record)).toStdString(),
      label_point, std::clamp(radius * 1.5, 0.045, 0.18), red, green, blue, text_id);
    shape_ids.push_back(text_id);
  }
  measurement_shape_ids_[record.id] = std::move(shape_ids);
}

void MainWindow::remove_measurement_shapes(int id)
{
  const std::string prefix = measurement_prefix(id).toStdString();
  const auto rendered = measurement_shape_ids_.find(id);
  if (rendered != measurement_shape_ids_.end()) {
    for (const std::string & shape_id : rendered->second) {
      viewer_->removeShape(shape_id);
    }
    measurement_shape_ids_.erase(rendered);
  }
  const auto record = std::find_if(measurements_.begin(), measurements_.end(),
    [id](const MeasurementRecord & item) { return item.id == id; });
  const std::size_t vertices = record == measurements_.end() ? 2 : record->vertices.size();
  for (std::size_t i = 0; i < vertices; ++i) {
    viewer_->removeShape(prefix + "node_" + std::to_string(i));
    if (i > 0) {
      viewer_->removeShape(prefix + "line_" + std::to_string(i - 1));
    }
  }
  // Remove IDs used by version 1.0 projects as well.
  viewer_->removeShape(prefix + "a");
  viewer_->removeShape(prefix + "b");
  viewer_->removeShape(prefix + "line");
  viewer_->removeShape(prefix + "text");
}

void MainWindow::render_all_measurements()
{
  std::vector<int> rendered_ids;
  rendered_ids.reserve(measurement_shape_ids_.size());
  for (const auto & item : measurement_shape_ids_) {
    rendered_ids.push_back(item.first);
  }
  for (int id : rendered_ids) {
    remove_measurement_shapes(id);
  }
  measurement_shape_ids_.clear();
  for (const MeasurementRecord & record : measurements_) {
    render_measurement(record);
  }
}

void MainWindow::clear_active_selection()
{
  viewer_->removeShape(kPendingPointId);
  viewer_->removeShape(kCropFirstPointId);
  pending_point_.reset();
  crop_first_corner_.reset();
  active_polyline_points_.clear();
  for (const std::string & id : active_shape_ids_) {
    viewer_->removeShape(id);
  }
  active_shape_ids_.clear();
  finish_polyline_button_->setEnabled(false);
}

void MainWindow::start_new_measurement()
{
  clear_active_selection();
  measurement_result_label_->setText(QStringLiteral("等待选点…"));
  measurement_extra_label_->setText(QStringLiteral("—"));
  measurement_metadata_label_->setText(QStringLiteral("—"));
  point_a_label_->setText(QStringLiteral("等待选择 A 点…"));
  point_b_label_->setText(QStringLiteral("—"));
  delta_label_->setText(QStringLiteral("—"));
  distance_3d_label_->setText(QStringLiteral("—"));
  horizontal_distance_label_->setText(QStringLiteral("—"));
  vertical_distance_label_->setText(QStringLiteral("—"));
  slope_label_->setText(QStringLiteral("—"));
  interaction_mode_changed(interaction_mode_combo_->currentIndex());
  request_render();
}

void MainWindow::undo_measurement()
{
  if (!active_polyline_points_.empty()) {
    active_polyline_points_.pop_back();
    render_active_polyline();
    const MeasurementKind kind = measurement_kind_for_mode(interaction_mode_combo_->currentIndex());
    finish_polyline_button_->setEnabled(
      measurement_requires_manual_finish(kind) &&
      static_cast<int>(active_polyline_points_.size()) >= measurement_minimum_points(kind));
    if (active_polyline_points_.empty()) {
      update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
    } else {
      const MeasurementRecord preview = make_measurement_record(
        kind, active_polyline_points_, 0);
      update_measurement_details(preview.valid ? &preview : nullptr);
    }
    return;
  }
  if (crop_first_corner_) {
    viewer_->removeShape(kCropFirstPointId);
    crop_first_corner_.reset();
    picking_instruction_label_->setText(QStringLiteral("已取消第一个裁剪角点。"));
    request_render();
    return;
  }
  if (pending_point_) {
    viewer_->removeShape(kPendingPointId);
    pending_point_.reset();
    update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
    picking_instruction_label_->setText(QStringLiteral("已取消待选 A 点。"));
    request_render();
    return;
  }

  if (undo_history_.empty()) {
    statusBar()->showMessage(QStringLiteral("没有可撤销的测量。"), 3000);
    return;
  }

  redo_history_.push_back({measurements_, next_measurement_id_});
  const MeasurementHistoryState state = undo_history_.back();
  undo_history_.pop_back();
  restore_measurement_history(state.records, state.next_id);
  statusBar()->showMessage(QStringLiteral("已撤销上一项测量操作。"), 3000);
}

void MainWindow::redo_measurement()
{
  if (!active_polyline_points_.empty() || pending_point_ || crop_first_corner_) {
    clear_active_selection();
  }
  if (redo_history_.empty()) {
    statusBar()->showMessage(QStringLiteral("没有可重做的测量操作。"), 3000);
    return;
  }

  undo_history_.push_back({measurements_, next_measurement_id_});
  const MeasurementHistoryState state = redo_history_.back();
  redo_history_.pop_back();
  restore_measurement_history(state.records, state.next_id);
  statusBar()->showMessage(QStringLiteral("已重做测量操作。"), 3000);
}

void MainWindow::clear_measurements()
{
  if (!viewer_) {
    return;
  }
  clear_active_selection();
  if (!measurements_.empty()) {
    push_measurement_history();
  }
  for (const MeasurementRecord & record : measurements_) {
    remove_measurement_shapes(record.id);
  }
  measurements_.clear();
  next_measurement_id_ = 1;
  rebuild_measurement_table();
  update_measurement_details(nullptr);
  interaction_mode_changed(interaction_mode_combo_->currentIndex());
  request_render();
}

void MainWindow::push_measurement_history()
{
  undo_history_.push_back({measurements_, next_measurement_id_});
  constexpr std::size_t maximum_history = 100;
  if (undo_history_.size() > maximum_history) {
    undo_history_.erase(undo_history_.begin());
  }
  redo_history_.clear();
}

void MainWindow::restore_measurement_history(
  const std::vector<MeasurementRecord> & records,
  int next_id)
{
  clear_active_selection();
  std::vector<int> rendered_ids;
  rendered_ids.reserve(measurement_shape_ids_.size());
  for (const auto & item : measurement_shape_ids_) {
    rendered_ids.push_back(item.first);
  }
  for (int id : rendered_ids) {
    remove_measurement_shapes(id);
  }

  measurements_ = records;
  next_measurement_id_ = std::max(1, next_id);
  render_all_measurements();
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  if (!measurements_.empty()) {
    for (int row = 0; row < measurement_table_->rowCount(); ++row) {
      if (measurement_table_->item(row, 0)->data(Qt::UserRole).toInt() ==
        static_cast<int>(measurements_.size()) - 1)
      {
        measurement_table_->selectRow(row);
        break;
      }
    }
  }
  request_render();
}

int MainWindow::selected_measurement_index() const
{
  if (!measurement_table_) {
    return -1;
  }
  const int row = measurement_table_->currentRow();
  if (row < 0 || !measurement_table_->item(row, 0)) return -1;
  const int index = measurement_table_->item(row, 0)->data(Qt::UserRole).toInt();
  return index >= 0 && index < static_cast<int>(measurements_.size()) ? index : -1;
}

void MainWindow::edit_selected_measurement()
{
  const int index = selected_measurement_index();
  if (index < 0) {
    statusBar()->showMessage(QStringLiteral("请先在记录表中选择一条测量。"), 4000);
    return;
  }

  const MeasurementRecord & original = measurements_[static_cast<std::size_t>(index)];
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("编辑测量记录 #%1").arg(original.id));
  dialog.setMinimumWidth(430);
  auto * layout = new QVBoxLayout(&dialog);
  auto * form = new QFormLayout;
  auto * name_edit = new QLineEdit(original.name, &dialog);
  name_edit->setObjectName(QStringLiteral("measurementNameEdit"));
  auto * group_edit = new QLineEdit(original.group, &dialog);
  group_edit->setObjectName(QStringLiteral("measurementGroupEdit"));
  auto * note_edit = new QTextEdit(&dialog);
  note_edit->setObjectName(QStringLiteral("measurementNoteEdit"));
  note_edit->setPlainText(original.note);
  note_edit->setPlaceholderText(QStringLiteral("可填写位置、用途、复核说明等"));
  note_edit->setMaximumHeight(100);
  auto * points_edit = new QTextEdit(&dialog);
  points_edit->setObjectName(QStringLiteral("measurementPointsEdit"));
  QStringList point_lines;
  for (const pcl::PointXYZ & point : original.vertices) {
    point_lines.append(QStringLiteral("%1, %2, %3")
      .arg(point.x, 0, 'f', 6).arg(point.y, 0, 'f', 6).arg(point.z, 0, 'f', 6));
  }
  points_edit->setPlainText(point_lines.join(QLatin1Char('\n')));
  points_edit->setToolTip(QStringLiteral("每行一个 X, Y, Z 坐标，坐标单位固定为米"));
  points_edit->setMaximumHeight(125);
  auto * color_edit = new QLineEdit(original.color_hex, &dialog);
  color_edit->setObjectName(QStringLiteral("measurementColorEdit"));
  auto * choose_color_button = new QPushButton(QStringLiteral("选择颜色…"), &dialog);
  auto * color_row = new QWidget(&dialog);
  auto * color_layout = new QHBoxLayout(color_row);
  color_layout->setContentsMargins(0, 0, 0, 0);
  color_layout->addWidget(color_edit, 1);
  color_layout->addWidget(choose_color_button);
  auto * visible_check = new QCheckBox(QStringLiteral("在三维视图中显示"), &dialog);
  visible_check->setObjectName(QStringLiteral("measurementVisibleCheck"));
  visible_check->setChecked(original.visible);
  form->addRow(QStringLiteral("类型："), new QLabel(measurement_kind_label(original.kind), &dialog));
  form->addRow(QStringLiteral("名称："), name_edit);
  form->addRow(QStringLiteral("分组："), group_edit);
  form->addRow(QStringLiteral("备注："), note_edit);
  form->addRow(QStringLiteral("测量点 (m)："), points_edit);
  form->addRow(QStringLiteral("标注颜色："), color_row);
  form->addRow(QString(), visible_check);
  layout->addLayout(form);
  auto * buttons = new QDialogButtonBox(
    QDialogButtonBox::Save | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
  buttons->setObjectName(QStringLiteral("measurementEditButtons"));
  layout->addWidget(buttons);

  connect(choose_color_button, &QPushButton::clicked, &dialog, [color_edit, &dialog]() {
    const QColor initial(color_edit->text());
    const QColor chosen = QColorDialog::getColor(
      initial.isValid() ? initial : QColor(QStringLiteral("#FFB547")),
      &dialog, QStringLiteral("选择测量标注颜色"));
    if (chosen.isValid()) {
      color_edit->setText(chosen.name(QColor::HexRgb).toUpper());
    }
  });
  std::optional<MeasurementRecord> edited_geometry;
  connect(buttons, &QDialogButtonBox::accepted, &dialog,
    [&dialog, points_edit, &original, &edited_geometry]() {
      std::vector<pcl::PointXYZ> points;
      const QStringList lines = points_edit->toPlainText().split(
        QRegularExpression(QStringLiteral("[\r\n]+")), Qt::SkipEmptyParts);
      for (const QString & source_line : lines) {
        QString line = source_line;
        line.replace(QLatin1Char(','), QLatin1Char(' '));
        line.replace(QLatin1Char(';'), QLatin1Char(' '));
        const QStringList parts = line.split(
          QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
        if (parts.size() != 3) {
          QMessageBox::warning(&dialog, QStringLiteral("坐标格式错误"),
            QStringLiteral("每行必须包含 X、Y、Z 三个数字，单位为米。"));
          return;
        }
        bool ok_x = false;
        bool ok_y = false;
        bool ok_z = false;
        const double x = parts[0].toDouble(&ok_x);
        const double y = parts[1].toDouble(&ok_y);
        const double z = parts[2].toDouble(&ok_z);
        if (!ok_x || !ok_y || !ok_z || !std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(z))
        {
          QMessageBox::warning(&dialog, QStringLiteral("坐标格式错误"),
            QStringLiteral("坐标只能使用有限数值。"));
          return;
        }
        points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
      }

      const int required = measurement_required_points(original.kind);
      const int minimum = measurement_minimum_points(original.kind);
      if ((required > 0 && static_cast<int>(points.size()) != required) ||
        (required == 0 && static_cast<int>(points.size()) < minimum))
      {
        QMessageBox::warning(&dialog, QStringLiteral("测量点数量不正确"),
          required > 0 ?
            QStringLiteral("%1需要恰好 %2 个点。").arg(measurement_kind_label(original.kind)).arg(required) :
            QStringLiteral("%1至少需要 %2 个点。").arg(measurement_kind_label(original.kind)).arg(minimum));
        return;
      }

      MeasurementRecord calculated = calculate_measurement(original.kind, points, original.id);
      if (!calculated.valid) {
        QMessageBox::warning(&dialog, QStringLiteral("测量几何无效"), calculated.error);
        return;
      }
      calculated.created_at = original.created_at;
      edited_geometry = std::move(calculated);
      dialog.accept();
    });
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  QColor color(color_edit->text().trimmed());
  if (!color.isValid()) {
    color = QColor(QStringLiteral("#FFB547"));
  }
  push_measurement_history();
  MeasurementRecord & record = measurements_[static_cast<std::size_t>(index)];
  record = *edited_geometry;
  record.name = name_edit->text().trimmed();
  if (record.name.isEmpty()) {
    record.name = QStringLiteral("%1 #%2").arg(measurement_kind_label(record.kind)).arg(record.id);
  }
  record.group = group_edit->text().trimmed();
  if (record.group.isEmpty()) {
    record.group = QStringLiteral("默认");
  }
  record.note = note_edit->toPlainText().trimmed();
  record.color_hex = color.name(QColor::HexRgb).toUpper();
  record.visible = visible_check->isChecked();
  render_measurement(record);
  rebuild_measurement_table();
  for (int row = 0; row < measurement_table_->rowCount(); ++row) {
    if (measurement_table_->item(row, 0)->data(Qt::UserRole).toInt() == index) {
      measurement_table_->selectRow(row);
      break;
    }
  }
  update_measurement_details(&record);
  statusBar()->showMessage(QStringLiteral("测量记录 #%1 已更新。").arg(record.id), 4000);
  request_render();
}

void MainWindow::delete_selected_measurement()
{
  const int index = selected_measurement_index();
  if (index < 0) {
    statusBar()->showMessage(QStringLiteral("请先在记录表中选择一条测量。"), 4000);
    return;
  }

  push_measurement_history();
  const int id = measurements_[static_cast<std::size_t>(index)].id;
  remove_measurement_shapes(id);
  measurements_.erase(measurements_.begin() + index);
  rebuild_measurement_table();
  if (!measurements_.empty() && measurement_table_->rowCount() > 0) {
    const int next_row = std::min(index, measurement_table_->rowCount() - 1);
    measurement_table_->selectRow(next_row);
    const int next_index = selected_measurement_index();
    update_measurement_details(next_index >= 0 ?
      &measurements_[static_cast<std::size_t>(next_index)] : nullptr);
  } else {
    update_measurement_details(nullptr);
  }
  statusBar()->showMessage(QStringLiteral("已删除测量记录 #%1，可使用撤销恢复。").arg(id), 4000);
  request_render();
}

void MainWindow::copy_selected_measurement()
{
  const int index = selected_measurement_index();
  if (index < 0) {
    statusBar()->showMessage(QStringLiteral("请先选择要复制的测量记录。"), 4000);
    return;
  }
  push_measurement_history();
  MeasurementRecord copy = measurements_[static_cast<std::size_t>(index)];
  copy.id = next_measurement_id_++;
  copy.name = copy.name.isEmpty() ?
    QStringLiteral("%1 #%2（副本）").arg(measurement_kind_label(copy.kind)).arg(copy.id) :
    copy.name + QStringLiteral("（副本）");
  copy.created_at = QDateTime::currentDateTime().toString(Qt::ISODate);
  measurements_.push_back(copy);
  render_measurement(copy);
  rebuild_measurement_table();
  update_measurement_details(&measurements_.back());
  for (int row = 0; row < measurement_table_->rowCount(); ++row) {
    if (measurement_table_->item(row, 0)->data(Qt::UserRole).toInt() ==
      static_cast<int>(measurements_.size()) - 1)
    {
      measurement_table_->selectRow(row);
      break;
    }
  }
  statusBar()->showMessage(QStringLiteral("已复制为测量记录 #%1。").arg(copy.id), 4000);
  request_render();
}

void MainWindow::toggle_selected_group_visibility()
{
  const int index = selected_measurement_index();
  if (index < 0) {
    statusBar()->showMessage(QStringLiteral("请先选择该分组中的任意一条记录。"), 4000);
    return;
  }
  const QString group = measurements_[static_cast<std::size_t>(index)].group;
  const bool any_visible = std::any_of(measurements_.begin(), measurements_.end(),
    [&group](const MeasurementRecord & record) {
      return record.group == group && record.visible;
    });
  push_measurement_history();
  int changed = 0;
  for (MeasurementRecord & record : measurements_) {
    if (record.group == group) {
      record.visible = !any_visible;
      ++changed;
    }
  }
  render_all_measurements();
  rebuild_measurement_table();
  statusBar()->showMessage(QStringLiteral("分组“%1”的 %2 条记录已%3。")
    .arg(group).arg(changed).arg(any_visible ? QStringLiteral("隐藏") : QStringLiteral("显示")), 5000);
  request_render();
}

bool MainWindow::import_measurements_from_file(
  const QString & path,
  int * imported_count,
  int * skipped_count,
  QString * error)
{
  if (imported_count) *imported_count = 0;
  if (skipped_count) *skipped_count = 0;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法读取：%1").arg(path);
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError) {
    if (error) *error = QStringLiteral("JSON 解析失败：%1").arg(parse_error.errorString());
    return false;
  }
  QJsonArray array;
  if (document.isArray()) {
    array = document.array();
  } else if (document.isObject()) {
    array = document.object().value(QStringLiteral("measurements")).toArray();
  }
  if (array.isEmpty()) {
    if (error) *error = QStringLiteral("JSON 中没有 measurements 数组或数组为空。");
    return false;
  }

  const QStringList valid_kinds{QStringLiteral("point"), QStringLiteral("segment"),
    QStringLiteral("polyline"), QStringLiteral("angle"), QStringLiteral("area"),
    QStringLiteral("orthogonal"), QStringLiteral("point_to_plane"), QStringLiteral("circle")};
  std::vector<MeasurementRecord> imported;
  imported.reserve(array.size());
  std::set<int> used_ids;
  for (const MeasurementRecord & record : measurements_) used_ids.insert(record.id);
  int skipped = 0;
  int candidate_id = std::max(1, next_measurement_id_);
  for (const QJsonValue & value : array) {
    if (!value.isObject()) {
      ++skipped;
      continue;
    }
    const QJsonObject object = value.toObject();
    const QString kind_key = object.value(QStringLiteral("kind")).toString().trimmed().toLower();
    if (!valid_kinds.contains(kind_key)) {
      ++skipped;
      continue;
    }
    MeasurementRecord record;
    if (!measurement_from_json(object, record)) {
      ++skipped;
      continue;
    }
    if (used_ids.count(record.id) != 0) {
      while (used_ids.count(candidate_id) != 0) ++candidate_id;
      record.id = candidate_id++;
      if (record.name.isEmpty()) {
        record.name = QStringLiteral("%1 #%2").arg(measurement_kind_label(record.kind)).arg(record.id);
      }
    }
    if (!QColor(record.color_hex).isValid()) record.color_hex = QStringLiteral("#FFB547");
    used_ids.insert(record.id);
    candidate_id = std::max(candidate_id, record.id + 1);
    imported.push_back(std::move(record));
  }
  if (imported.empty()) {
    if (skipped_count) *skipped_count = skipped;
    if (error) *error = QStringLiteral("没有可导入的有效测量记录；%1 条记录无效。").arg(skipped);
    return false;
  }
  push_measurement_history();
  for (MeasurementRecord & record : imported) {
    next_measurement_id_ = std::max(next_measurement_id_, record.id + 1);
    measurements_.push_back(std::move(record));
  }
  render_all_measurements();
  rebuild_measurement_table();
  update_measurement_details(&measurements_.back());
  if (imported_count) *imported_count = static_cast<int>(imported.size());
  if (skipped_count) *skipped_count = skipped;
  request_render();
  return true;
}

void MainWindow::import_measurements()
{
  const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入测量记录"),
    current_path_.isEmpty() ? QDir::homePath() : QFileInfo(current_path_).absolutePath(),
    QStringLiteral("JSON 文件 (*.json);;所有文件 (*)"));
  if (path.isEmpty()) return;
  int imported = 0;
  int skipped = 0;
  QString error;
  if (!import_measurements_from_file(path, &imported, &skipped, &error)) {
    QMessageBox::warning(this, QStringLiteral("测量导入失败"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("已导入 %1 条测量，跳过 %2 条无效记录。")
    .arg(imported).arg(skipped), 8000);
}

void MainWindow::interaction_mode_changed(int index)
{
  if (!viewer_) {
    return;
  }
  clear_active_selection();
  finish_polyline_button_->setText(QStringLiteral("完成"));
  finish_polyline_button_->setEnabled(false);
  switch (index) {
    case 1:
      finish_polyline_button_->setText(QStringLiteral("完成折线"));
      picking_instruction_label_->setText(
        QStringLiteral("连续折线：Shift + 左键依次添加节点，然后点击“完成折线”。"));
      viewer_hint_label_->setText(
        QStringLiteral("折线模式 · Shift + 左键添加节点 · 完成后保存"));
      break;
    case 2:
      picking_instruction_label_->setText(
        QStringLiteral("区域裁剪：建议使用俯视图，Shift + 左键选择两个 XY 对角点。"));
      viewer_hint_label_->setText(
        QStringLiteral("裁剪模式 · 选两个 XY 对角点 · 按完整高度保留区域"));
      if (current_.ok()) {
        top_view();
      }
      break;
    case 3:
      picking_instruction_label_->setText(
        QStringLiteral("单点坐标：Shift + 左键选择一个真实点，保存坐标记录。"));
      viewer_hint_label_->setText(QStringLiteral("单点模式 · 点击即保存 XYZ 坐标"));
      break;
    case 4:
      picking_instruction_label_->setText(
        QStringLiteral("三点角度：依次选择 A、顶点 B、C，计算 ∠ABC。"));
      viewer_hint_label_->setText(QStringLiteral("角度模式 · 依次选择 A / 顶点 B / C"));
      break;
    case 5:
      finish_polyline_button_->setText(QStringLiteral("完成面积"));
      picking_instruction_label_->setText(
        QStringLiteral("多边形面积：沿边界依次选点，至少 3 点，再点击“完成面积”。"));
      viewer_hint_label_->setText(QStringLiteral("面积模式 · 沿边界选点 · 自动闭合"));
      break;
    case 6:
      picking_instruction_label_->setText(
        QStringLiteral("正交分解：选择 A、B，显示沿 X/Y/Z 的分解尺寸。"));
      viewer_hint_label_->setText(QStringLiteral("正交模式 · 两点生成 X/Y/Z 阶梯尺寸"));
      break;
    case 7:
      picking_instruction_label_->setText(
        QStringLiteral("点到平面：先选被测点 P，再选择平面上的 A、B、C 三点。"));
      viewer_hint_label_->setText(QStringLiteral("点到平面 · P + 三个平面点"));
      break;
    case 8:
      picking_instruction_label_->setText(
        QStringLiteral("圆与直径：在圆周上选择三个不共线点。"));
      viewer_hint_label_->setText(QStringLiteral("圆测量 · 三点拟合圆心、半径和直径"));
      break;
    default:
      picking_instruction_label_->setText(
        QStringLiteral("两点测距：Shift + 左键依次选择 A 点和 B 点；自动吸附到完整点云最近点。"));
      viewer_hint_label_->setText(
        QStringLiteral("两点模式 · Shift + 左键选择 A/B 点 · 完整点云吸附 · 左键旋转 · 中键平移"));
      break;
  }
  request_render();
}

void MainWindow::accept_crop_point(const pcl::PointXYZ & point)
{
  if (!crop_first_corner_) {
    if (crop_.active) {
      reset_crop();
    }
    crop_first_corner_ = point;
    viewer_->removeShape(kCropFirstPointId);
    viewer_->addSphere(point, marker_radius(), 0.10, 0.88, 0.88, kCropFirstPointId);
    picking_instruction_label_->setText(
      QStringLiteral("第一个区域角点已选择，再选择对角点。Z 方向自动使用完整高度。"));
    statusBar()->showMessage(QStringLiteral("已选择第一个裁剪角点。"), 5000);
    request_render();
    return;
  }

  const pcl::PointXYZ first = *crop_first_corner_;
  viewer_->removeShape(kCropFirstPointId);
  crop_first_corner_.reset();
  apply_crop(
    std::min<double>(first.x, point.x), std::max<double>(first.x, point.x),
    std::min<double>(first.y, point.y), std::max<double>(first.y, point.y));
}

void MainWindow::apply_crop(double min_x, double max_x, double min_y, double max_y)
{
  if (!current_.ok()) {
    return;
  }
  if (max_x - min_x < 1e-5 || max_y - min_y < 1e-5) {
    QMessageBox::warning(this, QStringLiteral("区域过小"),
      QStringLiteral("两个角点需要在 X 和 Y 方向形成一个有效区域。"));
    return;
  }
  BoxSelection selection{
    min_x, max_x, min_y, max_y,
    current_.metrics.raw_bounds.min_z,
    current_.metrics.raw_bounds.max_z,
    false};
  if (selection.max_z - selection.min_z <= 1e-9) {
    selection.min_z -= 1e-6;
    selection.max_z += 1e-6;
  }
  apply_box_crop(selection);
}

bool MainWindow::apply_box_crop(const BoxSelection & selection, bool reset_camera)
{
  if (!current_.ok()) return false;
  QString error;
  const auto selected = select_box_cloud(current_.cloud, selection, &error);
  if (!error.isEmpty()) {
    statusBar()->showMessage(error, 7000);
    return false;
  }
  CropRegion next;
  next.type = CropSelectionType::Box;
  next.inverted = selection.inverted;
  next.box = selection;
  next.description = QStringLiteral("三维框%1 · X [%2, %3] · Y [%4, %5] · Z [%6, %7] m")
    .arg(selection.inverted ? QStringLiteral("反选") : QString())
    .arg(selection.min_x, 0, 'f', 2).arg(selection.max_x, 0, 'f', 2)
    .arg(selection.min_y, 0, 'f', 2).arg(selection.max_y, 0, 'f', 2)
    .arg(selection.min_z, 0, 'f', 2).arg(selection.max_z, 0, 'f', 2);
  const CropRegion previous = crop_;
  crop_ = std::move(next);
  if (!apply_local_cloud(selected, CropSelectionType::Box, crop_.description, reset_camera)) {
    crop_ = previous;
    return false;
  }
  return true;
}

bool MainWindow::apply_polygon_crop(const PolygonSelection & selection, bool reset_camera)
{
  if (!current_.ok()) return false;
  QString error;
  const auto selected = select_polygon_cloud(current_.cloud, selection, &error);
  if (!error.isEmpty()) {
    statusBar()->showMessage(error, 7000);
    return false;
  }
  CropRegion next;
  next.type = CropSelectionType::Polygon;
  next.inverted = selection.inverted;
  next.polygon = selection;
  next.description = QStringLiteral("多边形套索%1 · %2 个边界点 · Z [%3, %4] m")
    .arg(selection.inverted ? QStringLiteral("反选") : QString())
    .arg(selection.vertices.size())
    .arg(selection.min_z, 0, 'f', 2).arg(selection.max_z, 0, 'f', 2);
  const CropRegion previous = crop_;
  crop_ = std::move(next);
  if (!apply_local_cloud(selected, CropSelectionType::Polygon, crop_.description, reset_camera)) {
    crop_ = previous;
    return false;
  }
  return true;
}

bool MainWindow::apply_local_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  CropSelectionType type,
  const QString & description,
  bool reset_camera)
{
  if (!cloud || cloud->size() < 3) {
    statusBar()->showMessage(
      QStringLiteral("区域内只有 %1 个点，至少需要 3 个点。").arg(cloud ? cloud->size() : 0),
      7000);
    return false;
  }
  const RegionAnalysisResult analysis = analyze_region(cloud);
  if (!analysis.valid) {
    statusBar()->showMessage(analysis.error, 7000);
    return false;
  }
  crop_.active = true;
  crop_.type = type;
  crop_.description = description;
  crop_.analysis = analysis;
  crop_.min_x = analysis.bounds.min_x;
  crop_.max_x = analysis.bounds.max_x;
  crop_.min_y = analysis.bounds.min_y;
  crop_.max_y = analysis.bounds.max_y;
  crop_.min_z = analysis.bounds.min_z;
  crop_.max_z = analysis.bounds.max_z;
  crop_.major_size = analysis.oriented.major_size;
  crop_.minor_size = analysis.oriented.minor_size;
  crop_.height = analysis.oriented.height;
  crop_.diagonal_3d = analysis.oriented.diagonal_3d;
  crop_.point_count = cloud->size();
  crop_.centroid = analysis.centroid;
  crop_.full_cloud = cloud;

  rebuild_crop_display_cloud(display_limit_spin_ ?
    static_cast<std::size_t>(display_limit_spin_->value()) : 750000);

  update_picking_tree();
  render_cloud(false);
  update_crop_information();
  update_crop_overlay();
  update_bounds_overlay();
  if (reset_camera) {
    const double camera_distance = std::max(3.0, crop_.diagonal_3d * 1.6);
    set_camera(
      crop_.centroid[0], crop_.centroid[1], crop_.centroid[2] + camera_distance,
      crop_.centroid[0], crop_.centroid[1], crop_.centroid[2],
      0.0, 1.0, 0.0);
  }
  picking_instruction_label_->setText(
    QStringLiteral("区域已启用。测量和分析将使用当前区域；可随时恢复完整点云。"));
  statusBar()->showMessage(
    QStringLiteral("局部区域：%1 个点，尺寸 %2 × %3 × %4 m")
      .arg(count_text(crop_.point_count))
      .arg(crop_.major_size, 0, 'f', 3)
      .arg(crop_.minor_size, 0, 'f', 3)
      .arg(crop_.height, 0, 'f', 3), 10000);
  request_render();
  return true;
}

void MainWindow::open_exact_crop_dialog()
{
  if (!current_.ok()) return;
  const Bounds3d & bounds = current_.metrics.raw_bounds;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("精确三维框裁剪"));
  auto * layout = new QVBoxLayout(&dialog);
  auto * form = new QFormLayout;
  std::array<QDoubleSpinBox *, 6> fields;
  const std::array<double, 6> initial{
    bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z};
  const std::array<QString, 6> labels{
    QStringLiteral("X 最小值 (m)："), QStringLiteral("X 最大值 (m)："),
    QStringLiteral("Y 最小值 (m)："), QStringLiteral("Y 最大值 (m)："),
    QStringLiteral("Z 最小值 (m)："), QStringLiteral("Z 最大值 (m)：")};
  for (std::size_t index = 0; index < fields.size(); ++index) {
    fields[index] = new QDoubleSpinBox(&dialog);
    fields[index]->setObjectName(QStringLiteral("exactCropValue%1").arg(index));
    fields[index]->setRange(-1000000000.0, 1000000000.0);
    fields[index]->setDecimals(6);
    fields[index]->setValue(initial[index]);
    form->addRow(labels[index], fields[index]);
  }
  auto * inverted = new QCheckBox(QStringLiteral("反选：保留三维框外的点"), &dialog);
  inverted->setObjectName(QStringLiteral("exactCropInverted"));
  form->addRow(QString(), inverted);
  layout->addLayout(form);
  auto * buttons = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
  buttons->setObjectName(QStringLiteral("exactCropButtons"));
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("应用裁剪"));
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return;
  BoxSelection selection{
    fields[0]->value(), fields[1]->value(), fields[2]->value(),
    fields[3]->value(), fields[4]->value(), fields[5]->value(), inverted->isChecked()};
  QString error;
  if (!valid_box_selection(selection, &error)) {
    QMessageBox::warning(this, QStringLiteral("三维框无效"), error);
    return;
  }
  if (!apply_box_crop(selection)) {
    QMessageBox::warning(this, QStringLiteral("裁剪失败"),
      QStringLiteral("三维框内没有足够点，当前区域未改变。"));
  }
}

void MainWindow::open_height_filter_dialog()
{
  if (!current_.ok()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("按高程范围筛选"));
  auto * layout = new QVBoxLayout(&dialog);
  auto * form = new QFormLayout;
  auto * minimum = new QDoubleSpinBox;
  minimum->setObjectName(QStringLiteral("heightFilterMinimum"));
  minimum->setRange(-1000000000.0, 1000000000.0);
  minimum->setDecimals(6);
  minimum->setValue(current_.metrics.raw_bounds.min_z);
  auto * maximum = new QDoubleSpinBox;
  maximum->setObjectName(QStringLiteral("heightFilterMaximum"));
  maximum->setRange(-1000000000.0, 1000000000.0);
  maximum->setDecimals(6);
  maximum->setValue(current_.metrics.raw_bounds.max_z);
  auto * inverted = new QCheckBox(QStringLiteral("反选：保留范围外的点"));
  inverted->setObjectName(QStringLiteral("heightFilterInverted"));
  form->addRow(QStringLiteral("最低高程 Z (m)："), minimum);
  form->addRow(QStringLiteral("最高高程 Z (m)："), maximum);
  form->addRow(QString(), inverted);
  layout->addLayout(form);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("应用高程筛选"));
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  if (dialog.exec() != QDialog::Accepted) return;
  if (minimum->value() >= maximum->value()) {
    QMessageBox::warning(this, QStringLiteral("高程范围无效"),
      QStringLiteral("最低高程必须小于最高高程。"));
    return;
  }
  const Bounds3d & bounds = current_.metrics.raw_bounds;
  const double x_padding = std::max(1e-6, std::abs(bounds.size_x()) * 1e-6);
  const double y_padding = std::max(1e-6, std::abs(bounds.size_y()) * 1e-6);
  BoxSelection selection{bounds.min_x - x_padding, bounds.max_x + x_padding,
    bounds.min_y - y_padding, bounds.max_y + y_padding,
    minimum->value(), maximum->value(), inverted->isChecked()};
  if (apply_box_crop(selection)) {
    crop_.description = QStringLiteral("高程%1 · Z [%2, %3] m")
      .arg(selection.inverted ? QStringLiteral("反选") : QStringLiteral("筛选"))
      .arg(selection.min_z, 0, 'f', 3).arg(selection.max_z, 0, 'f', 3);
    update_crop_information();
  }
}

void MainWindow::crop_to_selected_area()
{
  const int index = selected_measurement_index();
  if (index < 0 || measurements_[static_cast<std::size_t>(index)].kind != MeasurementKind::Area) {
    statusBar()->showMessage(QStringLiteral("请先在测量记录表中选择一条“多边形面积”记录。"), 6000);
    return;
  }
  PolygonSelection selection;
  selection.vertices = measurements_[static_cast<std::size_t>(index)].vertices;
  selection.min_z = current_.metrics.raw_bounds.min_z;
  selection.max_z = current_.metrics.raw_bounds.max_z;
  if (!apply_polygon_crop(selection)) {
    QMessageBox::warning(this, QStringLiteral("套索裁剪失败"),
      QStringLiteral("边界无效或边界内没有足够点。"));
  }
}

void MainWindow::invert_current_crop()
{
  if (!crop_.active) {
    statusBar()->showMessage(QStringLiteral("当前没有可反选的区域。"), 5000);
    return;
  }
  if (crop_.type == CropSelectionType::Box) {
    BoxSelection selection = crop_.box;
    selection.inverted = !selection.inverted;
    apply_box_crop(selection, false);
  } else if (crop_.type == CropSelectionType::Polygon) {
    PolygonSelection selection = crop_.polygon;
    selection.inverted = !selection.inverted;
    apply_polygon_crop(selection, false);
  } else {
    statusBar()->showMessage(QStringLiteral("分析结果区域不能反选。"), 5000);
  }
}

bool MainWindow::write_cloud_file(
  const QString & path,
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  QString * error) const
{
  if (!cloud || cloud->empty()) {
    if (error) *error = QStringLiteral("没有可导出的点。");
    return false;
  }
  const QFileInfo target(path);
  if (path.trimmed().isEmpty() || target.isDir()) {
    if (error) *error = QStringLiteral("PCD 导出目标不是文件：%1").arg(path);
    return false;
  }
  const QDir parent = target.absoluteDir();
  if (!parent.exists()) {
    if (error) *error = QStringLiteral("PCD 导出目录不存在：%1").arg(parent.absolutePath());
    return false;
  }
  QTemporaryFile temporary(parent.filePath(QStringLiteral(".pcd-measure-XXXXXX.pcd")));
  temporary.setAutoRemove(true);
  if (!temporary.open()) {
    if (error) *error = QStringLiteral("无法在目标目录创建 PCD 临时文件。");
    return false;
  }
  const QString temporary_path = temporary.fileName();
  temporary.close();
  if (pcl::io::savePCDFileBinaryCompressed(temporary_path.toStdString(), *cloud) != 0 ||
    QFileInfo(temporary_path).size() <= 0)
  {
    if (error) *error = QStringLiteral("无法编码 PCD：%1").arg(path);
    return false;
  }
  return copy_file_atomically(temporary_path, path, QStringLiteral(" PCD 文件"), error);
}

void MainWindow::export_current_region()
{
  if (!crop_.active || !crop_.full_cloud) {
    statusBar()->showMessage(QStringLiteral("请先建立一个局部区域。"), 5000);
    return;
  }
  const QFileInfo source(current_.path);
  QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出当前区域"),
    source.absolutePath() + QDir::separator() + source.completeBaseName() +
      QStringLiteral("_region.pcd"),
    QStringLiteral("PCD 点云 (*.pcd)"));
  if (path.isEmpty()) return;
  if (!path.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive)) path += QStringLiteral(".pcd");
  QString error;
  if (!write_cloud_file(path, crop_.full_cloud, &error)) {
    QMessageBox::critical(this, QStringLiteral("区域导出失败"), error);
    return;
  }
  statusBar()->showMessage(
    QStringLiteral("已导出 %1 个区域点：%2").arg(count_text(crop_.point_count)).arg(path), 8000);
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MainWindow::active_full_cloud() const
{
  if (crop_.active && crop_.full_cloud) return crop_.full_cloud;
  return current_.cloud;
}

const MeasurementRecord * MainWindow::selected_measurement(MeasurementKind kind) const
{
  const int index = selected_measurement_index();
  if (index < 0 || index >= static_cast<int>(measurements_.size())) return nullptr;
  const MeasurementRecord & record = measurements_[static_cast<std::size_t>(index)];
  return record.kind == kind && record.valid ? &record : nullptr;
}

void MainWindow::add_analysis_record(
  const QString & kind,
  const QString & title,
  const QString & summary,
  const QJsonObject & data)
{
  analysis_records_.push_back(AnalysisRecord{
    kind, title, summary, QDateTime::currentDateTime().toString(Qt::ISODate), data});
  if (analysis_records_.size() > 200) {
    analysis_records_.erase(analysis_records_.begin());
  }
}

void MainWindow::show_region_analysis()
{
  const auto cloud = active_full_cloud();
  if (!cloud || cloud->empty()) return;
  const RegionAnalysisResult analysis = run_progress_task<RegionAnalysisResult>(
    this, QStringLiteral("区域质量与平面"), [cloud]() { return analyze_region(cloud); });
  if (!analysis.valid) {
    QMessageBox::warning(this, QStringLiteral("区域分析失败"), analysis.error);
    return;
  }

  const QString plane_text = analysis.plane.valid ?
    QStringLiteral(
      "法向：[%1, %2, %3]\n坡度：%4°  方位：%5°\nRMS：%6 m  平均绝对残差：%7 m  最大残差：%8 m")
      .arg(analysis.plane.normal[0], 0, 'f', 6)
      .arg(analysis.plane.normal[1], 0, 'f', 6)
      .arg(analysis.plane.normal[2], 0, 'f', 6)
      .arg(analysis.plane.slope_degrees, 0, 'f', 3)
      .arg(analysis.plane.azimuth_degrees, 0, 'f', 3)
      .arg(analysis.plane.rms, 0, 'f', 6)
      .arg(analysis.plane.mean_absolute_residual, 0, 'f', 6)
      .arg(analysis.plane.maximum_absolute_residual, 0, 'f', 6) : analysis.plane.error;
  const QString text = QStringLiteral(
    "点数：%1\n质心：[X %2, Y %3, Z %4] m\n"
    "主方向尺寸：%5 × %6 × %7 m\n高程均值 / 标准差：%8 / %9 m\n"
    "估算点间距：%10 m\nXY 密度：%11 点/m²\nXYZ 密度：%12 点/m³\n\n"
    "PCA 拟合平面\n%13")
    .arg(count_text(analysis.point_count))
    .arg(analysis.centroid[0], 0, 'f', 5).arg(analysis.centroid[1], 0, 'f', 5)
    .arg(analysis.centroid[2], 0, 'f', 5)
    .arg(analysis.oriented.major_size, 0, 'f', 4)
    .arg(analysis.oriented.minor_size, 0, 'f', 4)
    .arg(analysis.oriented.height, 0, 'f', 4)
    .arg(analysis.z_mean, 0, 'f', 5).arg(analysis.z_stddev, 0, 'f', 5)
    .arg(analysis.estimated_spacing, 0, 'f', 6)
    .arg(analysis.density_xy, 0, 'f', 2).arg(analysis.density_xyz, 0, 'f', 2)
    .arg(plane_text);

  QJsonObject data;
  data.insert(QStringLiteral("point_count"), static_cast<qint64>(analysis.point_count));
  data.insert(QStringLiteral("centroid_m"), QJsonArray{
    analysis.centroid[0], analysis.centroid[1], analysis.centroid[2]});
  data.insert(QStringLiteral("extent_m"), QJsonArray{
    analysis.oriented.major_size, analysis.oriented.minor_size, analysis.oriented.height});
  data.insert(QStringLiteral("z_mean_m"), analysis.z_mean);
  data.insert(QStringLiteral("z_stddev_m"), analysis.z_stddev);
  data.insert(QStringLiteral("spacing_m"), finite_json_value(analysis.estimated_spacing));
  data.insert(QStringLiteral("density_xy_points_m2"), finite_json_value(analysis.density_xy));
  if (analysis.plane.valid) {
    QJsonObject plane;
    plane.insert(QStringLiteral("normal"), QJsonArray{
      analysis.plane.normal[0], analysis.plane.normal[1], analysis.plane.normal[2]});
    plane.insert(QStringLiteral("d"), analysis.plane.d);
    plane.insert(QStringLiteral("slope_deg"), analysis.plane.slope_degrees);
    plane.insert(QStringLiteral("azimuth_deg"), analysis.plane.azimuth_degrees);
    plane.insert(QStringLiteral("rms_m"), analysis.plane.rms);
    plane.insert(QStringLiteral("max_residual_m"), analysis.plane.maximum_absolute_residual);
    data.insert(QStringLiteral("plane"), plane);
  }
  const QString summary = QStringLiteral("%1 点 · 坡度 %2° · RMS %3 m")
    .arg(count_text(analysis.point_count))
    .arg(analysis.plane.slope_degrees, 0, 'f', 3)
    .arg(analysis.plane.rms, 0, 'f', 6);
  add_analysis_record(QStringLiteral("region"), QStringLiteral("区域质量与平面"), summary, data);

  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("区域质量与平面"));
  dialog.resize(610, 480);
  auto * layout = new QVBoxLayout(&dialog);
  auto * heading = new QLabel(QStringLiteral("REGION // QUALITY INSPECTION"));
  heading->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; font-family:'DejaVu Sans Mono'; font-weight:700; letter-spacing:1px;"));
  layout->addWidget(heading);
  auto * result = new QTextEdit;
  result->setObjectName(QStringLiteral("regionAnalysisText"));
  result->setReadOnly(true);
  result->setPlainText(text);
  layout->addWidget(result, 1);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  dialog.exec();
}

void MainWindow::show_height_histogram()
{
  const auto cloud = active_full_cloud();
  if (!cloud || cloud->empty()) return;
  QDialog options(this);
  options.setWindowTitle(QStringLiteral("高程直方图参数"));
  auto * options_layout = new QVBoxLayout(&options);
  auto * form = new QFormLayout;
  auto * bin_count = new QSpinBox;
  bin_count->setObjectName(QStringLiteral("histogramBinCount"));
  bin_count->setRange(1, 500);
  bin_count->setValue(40);
  form->addRow(QStringLiteral("分箱数量："), bin_count);
  options_layout->addLayout(form);
  auto * option_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  option_buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("生成图表"));
  connect(option_buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
  connect(option_buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
  options_layout->addWidget(option_buttons);
  if (options.exec() != QDialog::Accepted) return;

  const HistogramResult histogram = build_height_histogram(cloud, bin_count->value());
  if (!histogram.valid) {
    QMessageBox::warning(this, QStringLiteral("直方图失败"), histogram.error);
    return;
  }
  QJsonArray bins_json;
  QVector<QPointF> points;
  for (const HistogramBin & bin : histogram.bins) {
    const double center = (bin.lower + bin.upper) * 0.5;
    points.append(QPointF(center, static_cast<double>(bin.count)));
    bins_json.append(QJsonObject{{QStringLiteral("lower_m"), bin.lower},
      {QStringLiteral("upper_m"), bin.upper},
      {QStringLiteral("count"), static_cast<qint64>(bin.count)}});
  }
  QJsonObject data{{QStringLiteral("minimum_m"), histogram.minimum},
    {QStringLiteral("maximum_m"), histogram.maximum},
    {QStringLiteral("point_count"), static_cast<qint64>(histogram.point_count)},
    {QStringLiteral("bins"), bins_json}};
  add_analysis_record(QStringLiteral("histogram"), QStringLiteral("高程直方图"),
    QStringLiteral("%1 点 · %2 箱 · Z [%3, %4] m")
      .arg(count_text(histogram.point_count)).arg(histogram.bins.size())
      .arg(histogram.minimum, 0, 'f', 3).arg(histogram.maximum, 0, 'f', 3), data);

  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("高程直方图"));
  dialog.resize(820, 540);
  auto * layout = new QVBoxLayout(&dialog);
  auto * plot = new PlotWidget;
  plot->setObjectName(QStringLiteral("heightHistogramPlot"));
  plot->set_title(QStringLiteral("高程分布 · %1 个点").arg(count_text(histogram.point_count)));
  plot->set_axis_labels(QStringLiteral("高程 Z (m)"), QStringLiteral("点数"));
  plot->set_series({PlotSeries{QStringLiteral("点数"), QColor(QStringLiteral("#21D4D1")), points, true}});
  layout->addWidget(plot, 1);
  auto * buttons = new QDialogButtonBox;
  auto * csv_button = buttons->addButton(QStringLiteral("导出 CSV"), QDialogButtonBox::ActionRole);
  auto * png_button = buttons->addButton(QStringLiteral("保存图表 PNG"), QDialogButtonBox::ActionRole);
  buttons->addButton(QDialogButtonBox::Close);
  connect(csv_button, &QPushButton::clicked, &dialog, [this, histogram]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出高程直方图"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/height_histogram.csv"),
      QStringLiteral("CSV 文件 (*.csv)"));
    if (path.isEmpty()) return;
    if (!path.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive)) path += QStringLiteral(".csv");
    QString error;
    if (!write_histogram_csv(path, histogram, &error)) {
      QMessageBox::critical(this, QStringLiteral("导出失败"), error);
    }
  });
  connect(png_button, &QPushButton::clicked, &dialog, [this, plot]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("保存高程图表"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/height_histogram.png"),
      QStringLiteral("PNG 图片 (*.png)"));
    if (path.isEmpty()) return;
    if (!path.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)) path += QStringLiteral(".png");
    QString error;
    if (!write_widget_png(plot, path, &error)) {
      QMessageBox::critical(this, QStringLiteral("保存失败"), error);
    }
  });
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  dialog.exec();
}

bool MainWindow::write_histogram_csv(
  const QString & path,
  const HistogramResult & histogram,
  QString * error) const
{
  if (!histogram.valid) {
    if (error) *error = histogram.error.isEmpty() ?
      QStringLiteral("高程直方图结果无效。") : histogram.error;
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入：%1").arg(path);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << "lower_m,upper_m,count\n";
  for (const HistogramBin & bin : histogram.bins) {
    stream << QString::number(bin.lower, 'g', 15) << ','
           << QString::number(bin.upper, 'g', 15) << ',' << bin.count << '\n';
  }
  stream.flush();
  if (stream.status() != QTextStream::Ok || !file.commit()) {
    if (error) *error = QStringLiteral("直方图 CSV 写入失败：%1").arg(path);
    return false;
  }
  return true;
}

bool MainWindow::write_profile_csv(
  const QString & path, const ProfileResult & profile, QString * error) const
{
  if (!profile.valid) {
    if (error) *error = profile.error.isEmpty() ? QStringLiteral("剖面结果无效。") : profile.error;
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入：%1").arg(path);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << "station_start_m,station_end_m,point_count,z_min_m,z_mean_m,z_median_m,z_max_m\n";
  for (const ProfileBin & bin : profile.bins) {
    stream << QString::number(bin.station_start, 'g', 15) << ','
           << QString::number(bin.station_end, 'g', 15) << ',' << bin.point_count << ',';
    if (bin.point_count > 0) {
      stream << QString::number(bin.z_min, 'g', 15) << ','
             << QString::number(bin.z_mean, 'g', 15) << ','
             << QString::number(bin.z_median, 'g', 15) << ','
             << QString::number(bin.z_max, 'g', 15);
    } else {
      stream << ",,,";
    }
    stream << '\n';
  }
  stream.flush();
  if (stream.status() != QTextStream::Ok || !file.commit()) {
    if (error) *error = QStringLiteral("CSV 写入失败：%1").arg(path);
    return false;
  }
  return true;
}

void MainWindow::show_elevation_profile()
{
  const MeasurementRecord * line = selected_measurement(MeasurementKind::Polyline);
  if (!line) {
    statusBar()->showMessage(QStringLiteral("请先在记录表中选择一条“连续折线”。"), 6000);
    return;
  }
  const std::vector<pcl::PointXYZ> path = line->vertices;
  QDialog options(this);
  options.setWindowTitle(QStringLiteral("折线高程剖面参数"));
  auto * layout = new QVBoxLayout(&options);
  auto * form = new QFormLayout;
  auto * width = new QDoubleSpinBox;
  width->setObjectName(QStringLiteral("profileCorridorWidth"));
  width->setRange(0.001, 100000.0);
  width->setDecimals(4);
  width->setValue(std::max(0.1, current_.metrics.estimated_spacing * 10.0));
  auto * bin = new QDoubleSpinBox;
  bin->setObjectName(QStringLiteral("profileBinSize"));
  bin->setRange(0.001, 100000.0);
  bin->setDecimals(4);
  bin->setValue(0.1);
  form->addRow(QStringLiteral("走廊总宽度 (m)："), width);
  form->addRow(QStringLiteral("里程分箱长度 (m)："), bin);
  layout->addLayout(form);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("提取剖面"));
  connect(buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
  layout->addWidget(buttons);
  if (options.exec() != QDialog::Accepted) return;

  const auto cloud = active_full_cloud();
  const double corridor = width->value();
  const double bin_size = bin->value();
  const ProfileResult profile = run_progress_task<ProfileResult>(
    this, QStringLiteral("提取高程剖面"),
    [cloud, path, corridor, bin_size]() {
      return build_elevation_profile(cloud, path, corridor, bin_size);
    });
  if (!profile.valid) {
    QMessageBox::warning(this, QStringLiteral("剖面提取失败"), profile.error);
    return;
  }
  QVector<QPointF> minimum;
  QVector<QPointF> mean;
  QVector<QPointF> maximum;
  QJsonArray bins_json;
  for (const ProfileBin & value : profile.bins) {
    const double station = (value.station_start + value.station_end) * 0.5;
    if (value.point_count > 0) {
      minimum.append(QPointF(station, value.z_min));
      mean.append(QPointF(station, value.z_mean));
      maximum.append(QPointF(station, value.z_max));
    } else {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      minimum.append(QPointF(station, nan));
      mean.append(QPointF(station, nan));
      maximum.append(QPointF(station, nan));
    }
    bins_json.append(QJsonObject{{QStringLiteral("station_start_m"), value.station_start},
      {QStringLiteral("station_end_m"), value.station_end},
      {QStringLiteral("point_count"), static_cast<qint64>(value.point_count)},
      {QStringLiteral("z_min_m"), finite_json_value(value.z_min)},
      {QStringLiteral("z_mean_m"), finite_json_value(value.z_mean)},
      {QStringLiteral("z_median_m"), finite_json_value(value.z_median)},
      {QStringLiteral("z_max_m"), finite_json_value(value.z_max)}});
  }
  QJsonObject data{{QStringLiteral("measurement_id"), line->id},
    {QStringLiteral("path_length_m"), profile.path_length},
    {QStringLiteral("corridor_width_m"), profile.corridor_width},
    {QStringLiteral("bin_size_m"), profile.bin_size},
    {QStringLiteral("selected_points"), static_cast<qint64>(profile.selected_points)},
    {QStringLiteral("bins"), bins_json}};
  add_analysis_record(QStringLiteral("profile"), QStringLiteral("高程剖面"),
    QStringLiteral("折线 #%1 · 长 %2 m · %3 个走廊点")
      .arg(line->id).arg(profile.path_length, 0, 'f', 3)
      .arg(count_text(profile.selected_points)), data);

  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("高程剖面"));
  dialog.resize(860, 560);
  auto * result_layout = new QVBoxLayout(&dialog);
  auto * summary = new QLabel(QStringLiteral(
    "路径 %1 m  ·  走廊宽 %2 m  ·  命中 %3 点  ·  %4 个分箱")
    .arg(profile.path_length, 0, 'f', 3).arg(profile.corridor_width, 0, 'f', 3)
    .arg(count_text(profile.selected_points)).arg(profile.bins.size()));
  summary->setStyleSheet(QStringLiteral("color:#7CE4E1; font-family:'DejaVu Sans Mono';"));
  result_layout->addWidget(summary);
  auto * plot = new PlotWidget;
  plot->setObjectName(QStringLiteral("elevationProfilePlot"));
  plot->set_title(QStringLiteral("沿折线高程剖面"));
  plot->set_axis_labels(QStringLiteral("累计里程 (m)"), QStringLiteral("高程 Z (m)"));
  plot->set_series({
    PlotSeries{QStringLiteral("最低"), QColor(QStringLiteral("#4D9DFF")), minimum, false},
    PlotSeries{QStringLiteral("平均"), QColor(QStringLiteral("#21D4D1")), mean, false},
    PlotSeries{QStringLiteral("最高"), QColor(QStringLiteral("#FFB547")), maximum, false}});
  result_layout->addWidget(plot, 1);
  auto * result_buttons = new QDialogButtonBox;
  auto * export_button = result_buttons->addButton(QStringLiteral("导出剖面 CSV"), QDialogButtonBox::ActionRole);
  result_buttons->addButton(QDialogButtonBox::Close);
  connect(export_button, &QPushButton::clicked, &dialog, [this, profile]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出剖面数据"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/elevation_profile.csv"),
      QStringLiteral("CSV 文件 (*.csv)"));
    if (path.isEmpty()) return;
    QString error;
    if (!write_profile_csv(path, profile, &error)) QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  });
  connect(result_buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  result_layout->addWidget(result_buttons);
  dialog.exec();
}

bool MainWindow::write_volume_csv(
  const QString & path, const VolumeResult & volume, QString * error) const
{
  if (!volume.valid) {
    if (error) *error = volume.error.isEmpty() ? QStringLiteral("体积结果无效。") : volume.error;
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入：%1").arg(path);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << "center_x_m,center_y_m,mean_z_m,point_count,signed_volume_m3\n";
  for (const VolumeCell & cell : volume.cells) {
    stream << QString::number(cell.center_x, 'g', 15) << ','
           << QString::number(cell.center_y, 'g', 15) << ','
           << QString::number(cell.mean_z, 'g', 15) << ',' << cell.point_count << ','
           << QString::number(cell.signed_volume, 'g', 15) << '\n';
  }
  stream.flush();
  if (stream.status() != QTextStream::Ok || !file.commit()) {
    if (error) *error = QStringLiteral("CSV 写入失败：%1").arg(path);
    return false;
  }
  return true;
}

void MainWindow::show_volume_estimation()
{
  const MeasurementRecord * area = selected_measurement(MeasurementKind::Area);
  if (!area) {
    statusBar()->showMessage(QStringLiteral("请先在记录表中选择一条“多边形面积”。"), 6000);
    return;
  }
  const std::vector<pcl::PointXYZ> boundary = area->vertices;
  QDialog options(this);
  options.setWindowTitle(QStringLiteral("栅格体积估算参数"));
  auto * layout = new QVBoxLayout(&options);
  auto * form = new QFormLayout;
  auto * base = new QDoubleSpinBox;
  base->setObjectName(QStringLiteral("volumeBaseZ"));
  base->setRange(-1000000000.0, 1000000000.0);
  base->setDecimals(6);
  base->setValue(area->center[2]);
  auto * cell = new QDoubleSpinBox;
  cell->setObjectName(QStringLiteral("volumeCellSize"));
  cell->setRange(0.001, 100000.0);
  cell->setDecimals(4);
  cell->setValue(std::max(0.05, current_.metrics.estimated_spacing * 5.0));
  form->addRow(QStringLiteral("基准高程 Z (m)："), base);
  form->addRow(QStringLiteral("栅格边长 (m)："), cell);
  layout->addLayout(form);
  auto * note = new QLabel(QStringLiteral("正值为高于基准（填方），负值为低于基准（挖方）。"));
  note->setWordWrap(true);
  layout->addWidget(note);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("开始估算"));
  connect(buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
  layout->addWidget(buttons);
  if (options.exec() != QDialog::Accepted) return;

  const auto cloud = active_full_cloud();
  const double base_z = base->value();
  const double cell_size = cell->value();
  const VolumeResult volume = run_progress_task<VolumeResult>(
    this, QStringLiteral("估算栅格体积"),
    [cloud, boundary, base_z, cell_size]() {
      return estimate_grid_volume(cloud, boundary, base_z, cell_size);
    });
  if (!volume.valid) {
    QMessageBox::warning(this, QStringLiteral("体积估算失败"), volume.error);
    return;
  }
  const double empty_percent = volume.possible_cells > 0 ?
    100.0 * static_cast<double>(volume.possible_cells - volume.occupied_cells) /
      static_cast<double>(volume.possible_cells) : 0.0;
  const QString summary = QStringLiteral(
    "边界面积：%1 m²\n有效覆盖：%2 m²\n占用栅格：%3 / %4（空栅格 %5%）\n\n"
    "高于基准：%6 m³\n低于基准：%7 m³\n净体积：%8 m³")
    .arg(volume.boundary_area, 0, 'f', 4).arg(volume.covered_area, 0, 'f', 4)
    .arg(volume.occupied_cells).arg(volume.possible_cells).arg(empty_percent, 0, 'f', 2)
    .arg(volume.volume_above, 0, 'f', 5).arg(volume.volume_below, 0, 'f', 5)
    .arg(volume.net_volume, 0, 'f', 5);
  QJsonObject data{{QStringLiteral("measurement_id"), area->id},
    {QStringLiteral("base_z_m"), volume.base_z},
    {QStringLiteral("cell_size_m"), volume.cell_size},
    {QStringLiteral("selected_points"), static_cast<qint64>(volume.selected_points)},
    {QStringLiteral("occupied_cells"), static_cast<qint64>(volume.occupied_cells)},
    {QStringLiteral("possible_cells"), static_cast<qint64>(volume.possible_cells)},
    {QStringLiteral("covered_area_m2"), volume.covered_area},
    {QStringLiteral("boundary_area_m2"), volume.boundary_area},
    {QStringLiteral("volume_above_m3"), volume.volume_above},
    {QStringLiteral("volume_below_m3"), volume.volume_below},
    {QStringLiteral("net_volume_m3"), volume.net_volume}};
  add_analysis_record(QStringLiteral("volume"), QStringLiteral("栅格体积"),
    QStringLiteral("面积 #%1 · 净体积 %2 m³ · 覆盖 %3 m²")
      .arg(area->id).arg(volume.net_volume, 0, 'f', 4).arg(volume.covered_area, 0, 'f', 3), data);

  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("栅格体积估算"));
  dialog.resize(520, 390);
  auto * result_layout = new QVBoxLayout(&dialog);
  auto * heading = new QLabel(QStringLiteral("VOLUME // GRID ESTIMATION"));
  heading->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; font-family:'DejaVu Sans Mono'; font-weight:700; letter-spacing:1px;"));
  result_layout->addWidget(heading);
  auto * result = new QTextEdit;
  result->setObjectName(QStringLiteral("volumeResultText"));
  result->setReadOnly(true);
  result->setPlainText(summary);
  result_layout->addWidget(result, 1);
  auto * result_buttons = new QDialogButtonBox;
  auto * export_button = result_buttons->addButton(QStringLiteral("导出栅格 CSV"), QDialogButtonBox::ActionRole);
  result_buttons->addButton(QDialogButtonBox::Close);
  connect(export_button, &QPushButton::clicked, &dialog, [this, volume]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出体积栅格"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/volume_grid.csv"),
      QStringLiteral("CSV 文件 (*.csv)"));
    if (path.isEmpty()) return;
    QString error;
    if (!write_volume_csv(path, volume, &error)) QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  });
  connect(result_buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  result_layout->addWidget(result_buttons);
  dialog.exec();
}

void MainWindow::show_outlier_filter()
{
  const auto cloud = active_full_cloud();
  if (!cloud || cloud->size() < 3) return;
  QDialog options(this);
  options.setWindowTitle(QStringLiteral("统计离群点清理"));
  auto * layout = new QVBoxLayout(&options);
  auto * form = new QFormLayout;
  auto * mean_k = new QSpinBox;
  mean_k->setObjectName(QStringLiteral("outlierMeanK"));
  mean_k->setRange(2, 500);
  mean_k->setValue(std::min(50, static_cast<int>(cloud->size()) - 1));
  auto * multiplier = new QDoubleSpinBox;
  multiplier->setObjectName(QStringLiteral("outlierStddev"));
  multiplier->setRange(0.01, 20.0);
  multiplier->setDecimals(3);
  multiplier->setValue(1.0);
  form->addRow(QStringLiteral("邻域点数 Mean K："), mean_k);
  form->addRow(QStringLiteral("标准差倍数："), multiplier);
  layout->addLayout(form);
  auto * note = new QLabel(QStringLiteral("先预览移除数量，再决定显示或导出；源 PCD 不会被覆盖。"));
  note->setWordWrap(true);
  layout->addWidget(note);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("分析离群点"));
  connect(buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
  layout->addWidget(buttons);
  if (options.exec() != QDialog::Accepted) return;
  const int k = mean_k->value();
  const double sigma = multiplier->value();
  const OutlierFilterResult filtered = run_progress_task<OutlierFilterResult>(
    this, QStringLiteral("统计离群点分析"),
    [cloud, k, sigma]() { return filter_statistical_outliers(cloud, k, sigma); });
  if (!filtered.valid) {
    QMessageBox::warning(this, QStringLiteral("离群点分析失败"), filtered.error);
    return;
  }
  const double removed_percent = cloud->empty() ? 0.0 :
    100.0 * static_cast<double>(filtered.removed->size()) / static_cast<double>(cloud->size());
  QJsonObject data{{QStringLiteral("source_points"), static_cast<qint64>(cloud->size())},
    {QStringLiteral("kept_points"), static_cast<qint64>(filtered.kept->size())},
    {QStringLiteral("removed_points"), static_cast<qint64>(filtered.removed->size())},
    {QStringLiteral("removed_percent"), removed_percent},
    {QStringLiteral("mean_k"), k}, {QStringLiteral("stddev_multiplier"), sigma}};
  add_analysis_record(QStringLiteral("outlier"), QStringLiteral("统计离群点清理"),
    QStringLiteral("保留 %1 · 移除 %2（%3%）")
      .arg(count_text(filtered.kept->size())).arg(count_text(filtered.removed->size()))
      .arg(removed_percent, 0, 'f', 2), data);

  QDialog result_dialog(this);
  result_dialog.setWindowTitle(QStringLiteral("离群点清理预览"));
  auto * result_layout = new QVBoxLayout(&result_dialog);
  auto * text = new QLabel(QStringLiteral(
    "输入：%1 点\n保留：%2 点\n拟移除：%3 点（%4%）")
      .arg(count_text(cloud->size())).arg(count_text(filtered.kept->size()))
      .arg(count_text(filtered.removed->size())).arg(removed_percent, 0, 'f', 3));
  text->setObjectName(QStringLiteral("outlierResultLabel"));
  result_layout->addWidget(text);
  auto * result_buttons = new QDialogButtonBox;
  auto * apply_button = result_buttons->addButton(QStringLiteral("显示清理结果"), QDialogButtonBox::AcceptRole);
  apply_button->setProperty("role", "primary");
  auto * export_button = result_buttons->addButton(QStringLiteral("导出清理后 PCD"), QDialogButtonBox::ActionRole);
  result_buttons->addButton(QDialogButtonBox::Close);
  connect(apply_button, &QPushButton::clicked, &result_dialog, &QDialog::accept);
  connect(export_button, &QPushButton::clicked, &result_dialog, [this, filtered]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出清理后点云"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/cleaned_cloud.pcd"),
      QStringLiteral("PCD 点云 (*.pcd)"));
    if (path.isEmpty()) return;
    QString error;
    if (!write_cloud_file(path, filtered.kept, &error)) QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  });
  connect(result_buttons, &QDialogButtonBox::rejected, &result_dialog, &QDialog::reject);
  result_layout->addWidget(result_buttons);
  if (result_dialog.exec() == QDialog::Accepted) {
    apply_local_cloud(filtered.kept, CropSelectionType::Derived,
      QStringLiteral("统计离群点清理 · MeanK %1 · σ %2 · 移除 %3 点")
        .arg(k).arg(sigma, 0, 'f', 2).arg(count_text(filtered.removed->size())));
  }
}

void MainWindow::show_dominant_plane()
{
  const auto cloud = active_full_cloud();
  if (!cloud || cloud->size() < 3) return;
  QDialog options(this);
  options.setWindowTitle(QStringLiteral("RANSAC 主平面提取"));
  auto * layout = new QVBoxLayout(&options);
  auto * form = new QFormLayout;
  auto * threshold = new QDoubleSpinBox;
  threshold->setObjectName(QStringLiteral("planeDistanceThreshold"));
  threshold->setRange(0.000001, 1000.0);
  threshold->setDecimals(6);
  threshold->setValue(std::max(0.01, current_.metrics.estimated_spacing * 2.0));
  auto * iterations = new QSpinBox;
  iterations->setObjectName(QStringLiteral("planeMaxIterations"));
  iterations->setRange(10, 100000);
  iterations->setValue(1000);
  form->addRow(QStringLiteral("内点距离阈值 (m)："), threshold);
  form->addRow(QStringLiteral("最大迭代次数："), iterations);
  layout->addLayout(form);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("提取主平面"));
  connect(buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
  layout->addWidget(buttons);
  if (options.exec() != QDialog::Accepted) return;
  const double distance = threshold->value();
  const int maximum_iterations = iterations->value();
  const DominantPlaneResult plane = run_progress_task<DominantPlaneResult>(
    this, QStringLiteral("提取主平面"),
    [cloud, distance, maximum_iterations]() {
      return extract_dominant_plane(cloud, distance, maximum_iterations);
    });
  if (!plane.valid) {
    QMessageBox::warning(this, QStringLiteral("主平面提取失败"), plane.error);
    return;
  }
  QJsonObject data{{QStringLiteral("source_points"), static_cast<qint64>(plane.source_points)},
    {QStringLiteral("inlier_points"), static_cast<qint64>(plane.inlier_points)},
    {QStringLiteral("inlier_ratio"), plane.inlier_ratio},
    {QStringLiteral("coefficients"), QJsonArray{
      plane.coefficients[0], plane.coefficients[1], plane.coefficients[2], plane.coefficients[3]}},
    {QStringLiteral("rms_m"), plane.rms},
    {QStringLiteral("distance_threshold_m"), distance},
    {QStringLiteral("maximum_iterations"), maximum_iterations}};
  add_analysis_record(QStringLiteral("plane"), QStringLiteral("RANSAC 主平面"),
    QStringLiteral("内点 %1 / %2（%3%）· RMS %4 m")
      .arg(count_text(plane.inlier_points)).arg(count_text(plane.source_points))
      .arg(plane.inlier_ratio * 100.0, 0, 'f', 2).arg(plane.rms, 0, 'f', 6), data);

  QDialog result_dialog(this);
  result_dialog.setWindowTitle(QStringLiteral("RANSAC 主平面结果"));
  auto * result_layout = new QVBoxLayout(&result_dialog);
  auto * result = new QTextEdit;
  result->setObjectName(QStringLiteral("dominantPlaneResultText"));
  result->setReadOnly(true);
  result->setPlainText(QStringLiteral(
    "平面方程：%1 x + %2 y + %3 z + %4 = 0\n"
    "内点：%5 / %6（%7%）\nRMS 残差：%8 m\n剩余点：%9")
    .arg(plane.coefficients[0], 0, 'f', 7).arg(plane.coefficients[1], 0, 'f', 7)
    .arg(plane.coefficients[2], 0, 'f', 7).arg(plane.coefficients[3], 0, 'f', 7)
    .arg(count_text(plane.inlier_points)).arg(count_text(plane.source_points))
    .arg(plane.inlier_ratio * 100.0, 0, 'f', 3).arg(plane.rms, 0, 'f', 7)
    .arg(count_text(plane.remainder->size())));
  result_layout->addWidget(result);
  auto * result_buttons = new QDialogButtonBox;
  auto * display_button = result_buttons->addButton(QStringLiteral("只显示主平面"), QDialogButtonBox::AcceptRole);
  display_button->setProperty("role", "primary");
  auto * export_button = result_buttons->addButton(QStringLiteral("导出平面内点 PCD"), QDialogButtonBox::ActionRole);
  result_buttons->addButton(QDialogButtonBox::Close);
  connect(display_button, &QPushButton::clicked, &result_dialog, &QDialog::accept);
  connect(export_button, &QPushButton::clicked, &result_dialog, [this, plane]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出主平面内点"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/dominant_plane.pcd"),
      QStringLiteral("PCD 点云 (*.pcd)"));
    if (path.isEmpty()) return;
    QString error;
    if (!write_cloud_file(path, plane.inliers, &error)) QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  });
  connect(result_buttons, &QDialogButtonBox::rejected, &result_dialog, &QDialog::reject);
  result_layout->addWidget(result_buttons);
  if (result_dialog.exec() == QDialog::Accepted) {
    apply_local_cloud(plane.inliers, CropSelectionType::Derived,
      QStringLiteral("RANSAC 主平面 · 内点 %1 · 阈值 %2 m")
        .arg(count_text(plane.inlier_points)).arg(distance, 0, 'f', 4));
  }
}

void MainWindow::show_analysis_records()
{
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("分析记录"));
  dialog.resize(820, 450);
  auto * layout = new QVBoxLayout(&dialog);
  auto * table = new QTableWidget(static_cast<int>(analysis_records_.size()), 4);
  table->setObjectName(QStringLiteral("analysisRecordsTable"));
  table->setHorizontalHeaderLabels({QStringLiteral("#"), QStringLiteral("时间"),
    QStringLiteral("分析类型"), QStringLiteral("结果摘要")});
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->verticalHeader()->setVisible(false);
  for (int row = 0; row < static_cast<int>(analysis_records_.size()); ++row) {
    const AnalysisRecord & record = analysis_records_[static_cast<std::size_t>(row)];
    table->setItem(row, 0, numeric_item(QString::number(row + 1)));
    table->setItem(row, 1, new QTableWidgetItem(record.created_at));
    table->setItem(row, 2, new QTableWidgetItem(record.title));
    table->setItem(row, 3, new QTableWidgetItem(record.summary));
  }
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  layout->addWidget(table, 1);
  auto * buttons = new QDialogButtonBox;
  auto * export_button = buttons->addButton(QStringLiteral("导出分析 JSON"), QDialogButtonBox::ActionRole);
  auto * clear_button = buttons->addButton(QStringLiteral("清空记录"), QDialogButtonBox::DestructiveRole);
  buttons->addButton(QDialogButtonBox::Close);
  connect(export_button, &QPushButton::clicked, &dialog, [this]() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出分析记录"),
      QFileInfo(current_.path).absolutePath() + QStringLiteral("/analysis_records.json"),
      QStringLiteral("JSON 文件 (*.json)"));
    if (path.isEmpty()) return;
    if (!path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) path += QStringLiteral(".json");
    QString error;
    if (!write_analysis_json(path, &error)) {
      QMessageBox::critical(this, QStringLiteral("导出失败"), error);
    }
  });
  connect(clear_button, &QPushButton::clicked, &dialog, [this, table]() {
    analysis_records_.clear();
    table->setRowCount(0);
  });
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  dialog.exec();
}

bool MainWindow::write_analysis_json(const QString & path, QString * error) const
{
  QJsonArray records;
  for (const AnalysisRecord & record : analysis_records_) {
    records.append(QJsonObject{{QStringLiteral("kind"), record.kind},
      {QStringLiteral("title"), record.title}, {QStringLiteral("summary"), record.summary},
      {QStringLiteral("created_at"), record.created_at}, {QStringLiteral("data"), record.data}});
  }
  QSaveFile file(path);
  const QByteArray payload = QJsonDocument(
    QJsonObject{{QStringLiteral("analyses"), records}}).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(payload) != payload.size() || !file.commit()) {
    if (error) *error = QStringLiteral("分析 JSON 写入失败：%1").arg(path);
    return false;
  }
  return true;
}

void MainWindow::open_cloud_comparison()
{
  if (!current_.ok()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("双点云配准与差异"));
  dialog.resize(610, 520);
  auto * layout = new QVBoxLayout(&dialog);
  auto * heading = new QLabel(QStringLiteral("CLOUD // REGISTRATION + DEVIATION"));
  heading->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; font-family:'DejaVu Sans Mono'; font-weight:700; letter-spacing:1px;"));
  layout->addWidget(heading);
  auto * form = new QFormLayout;
  auto * path_row = new QWidget;
  auto * path_layout = new QHBoxLayout(path_row);
  path_layout->setContentsMargins(0, 0, 0, 0);
  auto * path_edit = new QLineEdit;
  path_edit->setObjectName(QStringLiteral("comparisonPathEdit"));
  const QString recent = QSettings().value(QStringLiteral("lastComparisonPcd")).toString();
  if (QFileInfo::exists(recent)) path_edit->setText(recent);
  auto * browse = new QPushButton(QStringLiteral("选择…"));
  path_layout->addWidget(path_edit, 1);
  path_layout->addWidget(browse);
  form->addRow(QStringLiteral("第二点云："), path_row);
  auto * run_icp = new QCheckBox(QStringLiteral("启用 ICP 精配准"));
  run_icp->setObjectName(QStringLiteral("comparisonRunIcp"));
  run_icp->setChecked(true);
  auto * prealign = new QCheckBox(QStringLiteral("先按质心预对齐"));
  prealign->setObjectName(QStringLiteral("comparisonPrealign"));
  prealign->setChecked(true);
  auto * voxel = new QDoubleSpinBox;
  voxel->setObjectName(QStringLiteral("comparisonVoxel"));
  voxel->setRange(0.0, 10000.0);
  voxel->setDecimals(5);
  voxel->setValue(0.05);
  voxel->setSpecialValueText(QStringLiteral("0（不降采样）"));
  auto * iterations = new QSpinBox;
  iterations->setObjectName(QStringLiteral("comparisonIterations"));
  iterations->setRange(1, 100000);
  iterations->setValue(60);
  auto * correspondence = new QDoubleSpinBox;
  correspondence->setObjectName(QStringLiteral("comparisonMaxDistance"));
  correspondence->setRange(0.000001, 100000.0);
  correspondence->setDecimals(6);
  correspondence->setValue(0.5);
  auto * threshold = new QDoubleSpinBox;
  threshold->setObjectName(QStringLiteral("comparisonThreshold"));
  threshold->setRange(0.0, 100000.0);
  threshold->setDecimals(6);
  threshold->setValue(0.05);
  auto * opacity = new QSpinBox;
  opacity->setObjectName(QStringLiteral("comparisonOpacity"));
  opacity->setRange(10, 100);
  opacity->setSuffix(QStringLiteral(" %"));
  opacity->setValue(85);
  form->addRow(QString(), run_icp);
  form->addRow(QString(), prealign);
  form->addRow(QStringLiteral("ICP 体素边长 (m)："), voxel);
  form->addRow(QStringLiteral("最大迭代次数："), iterations);
  form->addRow(QStringLiteral("最大对应距离 (m)："), correspondence);
  form->addRow(QStringLiteral("差异阈值 (m)："), threshold);
  form->addRow(QStringLiteral("热力图不透明度："), opacity);
  layout->addLayout(form);
  auto * explanation = new QLabel(QStringLiteral(
    "蓝色表示接近基准，青/黄表示逐渐偏离，红色表示达到或超过差异阈值。"
    "关闭 ICP 时按原坐标直接比较，变换矩阵保持单位阵。"));
  explanation->setWordWrap(true);
  explanation->setStyleSheet(QStringLiteral(
    "background:#102A39; border-left:3px solid #21D4D1; padding:8px; color:#AAC1CB;"));
  layout->addWidget(explanation);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->setObjectName(QStringLiteral("comparisonButtons"));
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("开始对比"));
  buttons->button(QDialogButtonBox::Ok)->setProperty("role", "primary");
  layout->addWidget(buttons);
  connect(browse, &QPushButton::clicked, &dialog, [this, path_edit]() {
    const QString initial = path_edit->text().isEmpty() ? QFileInfo(current_.path).absolutePath() :
      QFileInfo(path_edit->text()).absolutePath();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("选择第二个 PCD"),
      initial, QStringLiteral("PCD 点云 (*.pcd)"));
    if (!path.isEmpty()) path_edit->setText(path);
  });
  const auto update_icp_controls = [=](bool enabled) {
      prealign->setEnabled(enabled);
      voxel->setEnabled(enabled);
      iterations->setEnabled(enabled);
      correspondence->setEnabled(enabled);
    };
  connect(run_icp, &QCheckBox::toggled, &dialog, update_icp_controls);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return;

  const QString second_path = QFileInfo(path_edit->text().trimmed()).absoluteFilePath();
  if (!QFileInfo::exists(second_path) ||
    QFileInfo(second_path).suffix().compare(QStringLiteral("pcd"), Qt::CaseInsensitive) != 0)
  {
    QMessageBox::warning(this, QStringLiteral("第二点云无效"),
      QStringLiteral("请选择一个存在的 .pcd 文件。"));
    return;
  }
  CloudComparisonOptions options;
  options.run_icp = run_icp->isChecked();
  options.centroid_prealign = prealign->isChecked();
  options.voxel_size = voxel->value();
  options.maximum_iterations = iterations->value();
  options.maximum_correspondence_distance = correspondence->value();
  options.difference_threshold = threshold->value();
  options.maximum_display_points = display_limit_spin_ ?
    static_cast<std::size_t>(display_limit_spin_->value()) : 750000;
  QString option_error;
  if (!valid_comparison_options(options, &option_error)) {
    QMessageBox::warning(this, QStringLiteral("对比参数无效"), option_error);
    return;
  }

  const auto reference = current_.cloud;
  const ComparisonJobOutput output = run_progress_task<ComparisonJobOutput>(
    this, QStringLiteral("双点云配准与差异"), [reference, second_path, options]() {
      ComparisonJobOutput job;
      job.second = load_pcd_and_analyze(second_path, options.maximum_display_points);
      if (job.second.ok()) {
        job.comparison = compare_point_clouds(reference, job.second.cloud, options);
      }
      return job;
    });
  if (!output.second.ok()) {
    QMessageBox::critical(this, QStringLiteral("第二点云加载失败"), output.second.error);
    return;
  }
  if (!output.comparison.valid) {
    QMessageBox::warning(this, QStringLiteral("点云对比失败"), output.comparison.error);
    return;
  }
  QSettings().setValue(QStringLiteral("lastComparisonPcd"), second_path);
  set_cloud_comparison(second_path, options, opacity->value(), output.comparison);
  show_comparison_summary();
}

void MainWindow::set_cloud_comparison(
  const QString & path,
  const CloudComparisonOptions & options,
  int opacity_percent,
  CloudComparisonResult result)
{
  comparison_.active = result.valid;
  comparison_.visible = true;
  comparison_.opacity_percent = std::clamp(opacity_percent, 10, 100);
  comparison_.second_path = QFileInfo(path).absoluteFilePath();
  comparison_.options = options;
  comparison_.result = std::move(result);
  comparison_visibility_action_->setChecked(true);
  render_cloud_comparison();
  const DistanceStatistics & statistics = comparison_.result.statistics;
  statusBar()->showMessage(QStringLiteral(
    "对比完成：RMSE %1 m · P95 %2 m · 超阈值 %3%")
    .arg(statistics.rmse, 0, 'f', 5).arg(statistics.p95, 0, 'f', 5)
    .arg(statistics.over_threshold_ratio * 100.0, 0, 'f', 2), 12000);
}

void MainWindow::render_cloud_comparison()
{
  if (!viewer_) return;
  viewer_->removePointCloud(kComparisonCloudId);
  if (!comparison_.active || !comparison_.visible ||
    !comparison_.result.display_heatmap_cloud || comparison_.result.display_heatmap_cloud->empty())
  {
    request_render();
    return;
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(
    comparison_.result.display_heatmap_cloud);
  if (viewer_->addPointCloud<pcl::PointXYZRGB>(
      comparison_.result.display_heatmap_cloud, handler, kComparisonCloudId))
  {
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      static_cast<double>(point_size_spin_->value() + 1), kComparisonCloudId);
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY,
      static_cast<double>(comparison_.opacity_percent) / 100.0, kComparisonCloudId);
  }
  request_render();
}

void MainWindow::show_comparison_summary()
{
  if (!comparison_.active) {
    statusBar()->showMessage(QStringLiteral("尚未建立双点云对比。"), 5000);
    return;
  }
  const CloudComparisonResult & result = comparison_.result;
  const DistanceStatistics & values = result.statistics;
  QString matrix;
  for (int row = 0; row < 4; ++row) {
    matrix += QStringLiteral("[%1  %2  %3  %4]\n")
      .arg(result.transform(row, 0), 10, 'f', 6)
      .arg(result.transform(row, 1), 10, 'f', 6)
      .arg(result.transform(row, 2), 10, 'f', 6)
      .arg(result.transform(row, 3), 10, 'f', 6);
  }
  const QString text = QStringLiteral(
    "基准：%1\n第二云：%2\n模式：%3\n收敛：%4  Fitness：%5\n\n"
    "逐点最近距离\n最小：%6 m\n均值：%7 m\nRMSE：%8 m\n中位：%9 m\n"
    "P95：%10 m\n最大：%11 m\n超过 %12 m：%13 / %14（%15%）\n\n"
    "第二云 → 基准云 4×4 变换矩阵\n%16")
    .arg(QFileInfo(current_.path).fileName()).arg(QFileInfo(comparison_.second_path).fileName())
    .arg(result.icp_requested ? QStringLiteral("质心预对齐 + ICP") : QStringLiteral("原坐标直接比较"))
    .arg(result.converged ? QStringLiteral("是") : QStringLiteral("否"))
    .arg(result.fitness_score, 0, 'g', 8)
    .arg(values.minimum, 0, 'f', 6).arg(values.mean, 0, 'f', 6)
    .arg(values.rmse, 0, 'f', 6).arg(values.median, 0, 'f', 6)
    .arg(values.p95, 0, 'f', 6).arg(values.maximum, 0, 'f', 6)
    .arg(result.difference_threshold, 0, 'f', 6)
    .arg(count_text(values.over_threshold_count)).arg(count_text(values.point_count))
    .arg(values.over_threshold_ratio * 100.0, 0, 'f', 3).arg(matrix);
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("双点云对比摘要"));
  dialog.resize(700, 590);
  auto * layout = new QVBoxLayout(&dialog);
  auto * heading = new QLabel(QStringLiteral("DEVIATION // INSPECTION SUMMARY"));
  heading->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; font-family:'DejaVu Sans Mono'; font-weight:700; letter-spacing:1px;"));
  layout->addWidget(heading);
  auto * text_view = new QTextEdit;
  text_view->setObjectName(QStringLiteral("comparisonSummaryText"));
  text_view->setReadOnly(true);
  text_view->setPlainText(text);
  layout->addWidget(text_view, 1);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  dialog.exec();
}

void MainWindow::toggle_comparison_visibility()
{
  if (!comparison_.active) {
    comparison_visibility_action_->setChecked(false);
    return;
  }
  comparison_.visible = comparison_visibility_action_->isChecked();
  render_cloud_comparison();
}

void MainWindow::export_aligned_cloud()
{
  if (!comparison_.active || !comparison_.result.aligned_cloud) return;
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出对齐后第二点云"),
    QFileInfo(comparison_.second_path).absolutePath() + QDir::separator() +
      QFileInfo(comparison_.second_path).completeBaseName() + QStringLiteral("_aligned.pcd"),
    QStringLiteral("PCD 点云 (*.pcd)"));
  if (path.isEmpty()) return;
  QString error;
  if (!write_cloud_file(path, comparison_.result.aligned_cloud, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  }
}

void MainWindow::export_comparison_distances()
{
  if (!comparison_.active) return;
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出逐点差异"),
    QFileInfo(comparison_.second_path).absolutePath() + QStringLiteral("/cloud_distances.csv"),
    QStringLiteral("CSV 文件 (*.csv)"));
  if (path.isEmpty()) return;
  QString error;
  if (!write_comparison_distances_csv(path, comparison_.result, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  }
}

void MainWindow::export_comparison_summary()
{
  if (!comparison_.active) return;
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出对比摘要"),
    QFileInfo(comparison_.second_path).absolutePath() + QStringLiteral("/cloud_comparison.json"),
    QStringLiteral("JSON 文件 (*.json)"));
  if (path.isEmpty()) return;
  QString error;
  if (!write_comparison_summary_json(path, comparison_.result,
      current_.path, comparison_.second_path, &error))
  {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  }
}

void MainWindow::clear_cloud_comparison()
{
  if (viewer_) viewer_->removePointCloud(kComparisonCloudId);
  comparison_ = CloudComparisonState{};
  if (comparison_visibility_action_) comparison_visibility_action_->setChecked(true);
  statusBar()->showMessage(QStringLiteral("已清除第二点云和差异热力图。"), 5000);
  request_render();
}

QString MainWindow::format_coordinate(const pcl::PointXYZ & point) const
{
  pcl::PointXYZ displayed(
    static_cast<float>(static_cast<double>(point.x) - display_origin_[0]),
    static_cast<float>(static_cast<double>(point.y) - display_origin_[1]),
    static_cast<float>(static_cast<double>(point.z) - display_origin_[2]));
  return coordinate_text(displayed, unit_scale(), unit_suffix());
}

void MainWindow::set_display_origin()
{
  if (!current_.ok()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("设置显示原点"));
  auto * layout = new QVBoxLayout(&dialog);
  auto * form = new QFormLayout;
  std::array<QDoubleSpinBox *, 3> fields;
  const std::array<QString, 3> labels{
    QStringLiteral("原点 X (m)："), QStringLiteral("原点 Y (m)："), QStringLiteral("原点 Z (m)：")};
  for (std::size_t index = 0; index < fields.size(); ++index) {
    fields[index] = new QDoubleSpinBox;
    fields[index]->setObjectName(QStringLiteral("displayOrigin%1").arg(index));
    fields[index]->setRange(-1000000000.0, 1000000000.0);
    fields[index]->setDecimals(7);
    fields[index]->setValue(display_origin_[index]);
    form->addRow(labels[index], fields[index]);
  }
  layout->addLayout(form);
  auto * note = new QLabel(QStringLiteral(
    "这里只改变界面中坐标的读数和坐标轴位置；源点、距离、面积与导出 PCD 均不改变。"));
  note->setWordWrap(true);
  layout->addWidget(note);
  auto * controls = new QHBoxLayout;
  auto * centroid_button = new QPushButton(QStringLiteral("使用点云质心"));
  auto * reset_button = new QPushButton(QStringLiteral("恢复世界原点"));
  controls->addWidget(centroid_button);
  controls->addWidget(reset_button);
  layout->addLayout(controls);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("应用显示原点"));
  layout->addWidget(buttons);
  connect(centroid_button, &QPushButton::clicked, &dialog, [this, fields]() {
    for (std::size_t index = 0; index < fields.size(); ++index) {
      fields[index]->setValue(current_.metrics.centroid[index]);
    }
  });
  connect(reset_button, &QPushButton::clicked, &dialog, [fields]() {
    for (QDoubleSpinBox * field : fields) field->setValue(0.0);
  });
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return;
  for (std::size_t index = 0; index < fields.size(); ++index) {
    display_origin_[index] = fields[index]->value();
  }
  fill_information_panel();
  update_crop_information();
  update_measurement_details(selected_measurement_index() >= 0 ?
    &measurements_[static_cast<std::size_t>(selected_measurement_index())] : nullptr);
  rebuild_measurement_table();
  toggle_axes(axes_check_->isChecked());
  statusBar()->showMessage(QStringLiteral("显示原点已更新；几何数据与测量距离未改变。"), 6000);
}

void MainWindow::rebuild_current_display_cloud(std::size_t maximum_display_points)
{
  if (!current_.cloud || current_.cloud->empty() || maximum_display_points == 0) {
    current_.display_cloud.reset();
    current_.metrics.displayed_points = 0;
    current_.metrics.display_downsampled = false;
    return;
  }
  if (current_.cloud->size() <= maximum_display_points) {
    current_.display_cloud = current_.cloud;
    current_.metrics.display_downsampled = false;
  } else {
    current_.display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
      static_cast<double>(current_.cloud->size()) / maximum_display_points));
    current_.display_cloud->reserve(maximum_display_points);
    for (std::size_t index = 0; index < current_.cloud->size(); index += stride) {
      current_.display_cloud->push_back((*current_.cloud)[index]);
    }
    current_.display_cloud->width = static_cast<std::uint32_t>(current_.display_cloud->size());
    current_.display_cloud->height = 1;
    current_.display_cloud->is_dense = true;
    current_.metrics.display_downsampled = true;
  }
  current_.metrics.displayed_points = current_.display_cloud->size();
}

void MainWindow::rebuild_crop_display_cloud(std::size_t maximum_display_points)
{
  if (!crop_.active || !crop_.full_cloud || crop_.full_cloud->empty() ||
    maximum_display_points == 0)
  {
    crop_.display_cloud.reset();
    return;
  }
  if (crop_.full_cloud->size() <= maximum_display_points) {
    crop_.display_cloud = crop_.full_cloud;
    return;
  }
  crop_.display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  const std::size_t stride = static_cast<std::size_t>(std::ceil(
    static_cast<double>(crop_.full_cloud->size()) / maximum_display_points));
  crop_.display_cloud->reserve(maximum_display_points);
  for (std::size_t index = 0; index < crop_.full_cloud->size(); index += stride) {
    crop_.display_cloud->push_back((*crop_.full_cloud)[index]);
  }
  crop_.display_cloud->width = static_cast<std::uint32_t>(crop_.display_cloud->size());
  crop_.display_cloud->height = 1;
  crop_.display_cloud->is_dense = true;
}

void MainWindow::update_current_metrics_from_cloud()
{
  if (!current_.cloud || current_.cloud->size() < 3) return;
  const RegionAnalysisResult analysis = analyze_region(current_.cloud);
  if (!analysis.valid) return;
  CloudMetrics & metrics = current_.metrics;
  metrics.finite_points = current_.cloud->size();
  metrics.raw_bounds = analysis.bounds;
  metrics.robust_axis_bounds = analysis.bounds;
  metrics.oriented = analysis.oriented;
  metrics.centroid = analysis.centroid;
  metrics.estimated_spacing = analysis.estimated_spacing;
  bool first = true;
  for (const pcl::PointXYZRGB & point : *current_.cloud) {
    if (first || point.z < metrics.lowest_point[2]) {
      metrics.lowest_point = {point.x, point.y, point.z};
    }
    if (first || point.z > metrics.highest_point[2]) {
      metrics.highest_point = {point.x, point.y, point.z};
    }
    first = false;
  }
}

bool MainWindow::apply_cloud_transform(
  const CloudTransformParameters & parameters,
  QString * error)
{
  if (!current_.ok()) {
    if (error) *error = QStringLiteral("尚未加载点云。");
    return false;
  }
  if (!valid_cloud_transform(parameters, error)) return false;
  const Eigen::Matrix4f matrix = cloud_transform_matrix(parameters);
  const auto transformed = transform_point_cloud(current_.cloud, matrix);
  if (!transformed || transformed->size() < 3) {
    if (error) *error = QStringLiteral("变换后没有足够的有效点。");
    return false;
  }

  if (crop_.active) reset_crop();
  clear_cloud_comparison();
  clear_active_selection();
  transform_backup_cloud_ = current_;
  transform_backup_measurements_ = measurements_;
  transform_backup_matrix_ = cumulative_transform_;
  transform_backup_valid_ = true;

  current_.cloud = transformed;
  current_.metrics.header_points = transformed->size();
  current_.metrics.invalid_points = 0;
  update_current_metrics_from_cloud();
  rebuild_current_display_cloud(display_limit_spin_ ?
    static_cast<std::size_t>(display_limit_spin_->value()) : 750000);
  std::vector<MeasurementRecord> transformed_measurements;
  transformed_measurements.reserve(measurements_.size());
  for (const MeasurementRecord & record : measurements_) {
    MeasurementRecord transformed_record = transform_measurement_record(record, matrix);
    if (transformed_record.valid) transformed_measurements.push_back(std::move(transformed_record));
  }
  measurements_ = std::move(transformed_measurements);
  cumulative_transform_ = matrix * cumulative_transform_;
  last_transform_parameters_ = parameters;
  undo_history_.clear();
  redo_history_.clear();
  analysis_records_.clear();
  QJsonArray matrix_json;
  for (int row = 0; row < 4; ++row) {
    QJsonArray values;
    for (int column = 0; column < 4; ++column) values.append(matrix(row, column));
    matrix_json.append(values);
  }
  add_analysis_record(QStringLiteral("transform"), QStringLiteral("点云坐标变换"),
    QStringLiteral("平移 [%1, %2, %3] m · 旋转 %4° · 尺度 %5")
      .arg(parameters.translation_x, 0, 'f', 3).arg(parameters.translation_y, 0, 'f', 3)
      .arg(parameters.translation_z, 0, 'f', 3)
      .arg(normalized_degrees(parameters.rotation_z_degrees), 0, 'f', 3)
      .arg(parameters.uniform_scale, 0, 'g', 8),
    QJsonObject{{QStringLiteral("matrix"), matrix_json}});
  undo_transform_action_->setEnabled(true);
  update_picking_tree();
  fill_information_panel();
  render_cloud(true);
  render_all_measurements();
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  toggle_axes(axes_check_->isChecked());
  update_grid_overlay();
  update_bounds_overlay();
  request_render();
  return true;
}

void MainWindow::open_cloud_transform()
{
  if (!current_.ok()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("点云坐标变换"));
  dialog.resize(540, 470);
  auto * layout = new QVBoxLayout(&dialog);
  auto * heading = new QLabel(QStringLiteral("COORDINATE // TRANSFORM"));
  heading->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; font-family:'DejaVu Sans Mono'; font-weight:700; letter-spacing:1px;"));
  layout->addWidget(heading);
  auto * form = new QFormLayout;
  std::array<QDoubleSpinBox *, 5> fields;
  const std::array<QString, 5> labels{
    QStringLiteral("平移 X (m)："), QStringLiteral("平移 Y (m)："),
    QStringLiteral("平移 Z (m)："), QStringLiteral("绕 Z 旋转 (°)："),
    QStringLiteral("统一尺度：")};
  for (std::size_t index = 0; index < fields.size(); ++index) {
    fields[index] = new QDoubleSpinBox;
    fields[index]->setObjectName(QStringLiteral("cloudTransform%1").arg(index));
    fields[index]->setDecimals(index == 4 ? 8 : 6);
    fields[index]->setRange(index == 4 ? 0.00000001 : -1000000000.0,
      index == 4 ? 1000000.0 : 1000000000.0);
    fields[index]->setValue(index == 4 ? 1.0 : 0.0);
    form->addRow(labels[index], fields[index]);
  }
  layout->addLayout(form);
  auto * matrix_label = new QLabel;
  matrix_label->setObjectName(QStringLiteral("cloudTransformMatrix"));
  matrix_label->setStyleSheet(QStringLiteral(
    "background:#081721; border:1px solid #29485C; padding:8px; color:#9EC5D3; "
    "font-family:'DejaVu Sans Mono';"));
  matrix_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(matrix_label);
  const auto parameters_from_fields = [fields]() {
      return CloudTransformParameters{fields[0]->value(), fields[1]->value(), fields[2]->value(),
        fields[3]->value(), fields[4]->value()};
    };
  const auto update_matrix = [matrix_label, parameters_from_fields]() {
      const Eigen::Matrix4f matrix = cloud_transform_matrix(parameters_from_fields());
      QString text;
      for (int row = 0; row < 4; ++row) {
        text += QStringLiteral("%1  %2  %3  %4\n")
          .arg(matrix(row, 0), 10, 'f', 5).arg(matrix(row, 1), 10, 'f', 5)
          .arg(matrix(row, 2), 10, 'f', 5).arg(matrix(row, 3), 10, 'f', 5);
      }
      matrix_label->setText(text.trimmed());
    };
  for (QDoubleSpinBox * field : fields) {
    connect(field, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dialog,
      [update_matrix](double) { update_matrix(); });
  }
  update_matrix();
  auto * note = new QLabel(QStringLiteral(
    "预览只变换当前显示样本。应用后将对完整点云和全部测量记录使用同一矩阵；"
    "当前裁剪、分析和双云对比会清除，源 PCD 文件不会被改写。"));
  note->setWordWrap(true);
  layout->addWidget(note);
  auto * buttons = new QDialogButtonBox;
  auto * preview_button = buttons->addButton(QStringLiteral("预览"), QDialogButtonBox::ActionRole);
  preview_button->setObjectName(QStringLiteral("previewCloudTransform"));
  auto * apply_button = buttons->addButton(QStringLiteral("应用变换"), QDialogButtonBox::AcceptRole);
  apply_button->setObjectName(QStringLiteral("applyCloudTransform"));
  apply_button->setProperty("role", "primary");
  buttons->addButton(QDialogButtonBox::Cancel);
  layout->addWidget(buttons);
  connect(preview_button, &QPushButton::clicked, &dialog, [this, parameters_from_fields]() {
    const CloudTransformParameters parameters = parameters_from_fields();
    QString error;
    if (!valid_cloud_transform(parameters, &error)) {
      QMessageBox::warning(this, QStringLiteral("变换参数无效"), error);
      return;
    }
    transform_preview_display_ = transform_point_cloud(
      current_.display_cloud, cloud_transform_matrix(parameters));
    render_cloud(true);
    statusBar()->showMessage(QStringLiteral("正在预览显示样本；关闭窗口可恢复。"), 5000);
  });
  connect(apply_button, &QPushButton::clicked, &dialog, [this, &dialog, parameters_from_fields]() {
    transform_preview_display_.reset();
    QString error;
    if (!apply_cloud_transform(parameters_from_fields(), &error)) {
      QMessageBox::warning(this, QStringLiteral("点云变换失败"), error);
      return;
    }
    dialog.accept();
  });
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  dialog.exec();
  if (transform_preview_display_) {
    transform_preview_display_.reset();
    render_cloud(true);
  }
}

void MainWindow::undo_cloud_transform()
{
  if (!transform_backup_valid_) {
    statusBar()->showMessage(QStringLiteral("没有可撤销的点云变换。"), 5000);
    return;
  }
  if (crop_.active) reset_crop();
  clear_cloud_comparison();
  current_ = transform_backup_cloud_;
  measurements_ = transform_backup_measurements_;
  cumulative_transform_ = transform_backup_matrix_;
  transform_backup_cloud_ = CloudLoadResult{};
  transform_backup_measurements_.clear();
  transform_backup_valid_ = false;
  undo_transform_action_->setEnabled(false);
  analysis_records_.clear();
  undo_history_.clear();
  redo_history_.clear();
  update_picking_tree();
  fill_information_panel();
  render_cloud(true);
  render_all_measurements();
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  toggle_axes(axes_check_->isChecked());
  update_grid_overlay();
  update_bounds_overlay();
  statusBar()->showMessage(QStringLiteral("已撤销上次点云坐标变换。"), 6000);
}

void MainWindow::export_transformed_cloud()
{
  if (!current_.ok()) return;
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出当前坐标点云"),
    QFileInfo(current_.path).absolutePath() + QDir::separator() +
      QFileInfo(current_.path).completeBaseName() + QStringLiteral("_transformed.pcd"),
    QStringLiteral("PCD 点云 (*.pcd)"));
  if (path.isEmpty()) return;
  QString error;
  if (!write_cloud_file(path, current_.cloud, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  }
}

void MainWindow::export_transform_matrix()
{
  if (!current_.ok()) return;
  QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出累计变换矩阵"),
    QFileInfo(current_.path).absolutePath() + QStringLiteral("/transform_matrix.json"),
    QStringLiteral("JSON 文件 (*.json)"));
  if (path.isEmpty()) return;
  QString error;
  if (!write_transform_matrix_json(path, cumulative_transform_, last_transform_parameters_, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
  }
}

void MainWindow::reset_crop()
{
  if (!viewer_) {
    return;
  }
  for (const std::string & id : crop_shape_ids_) {
    viewer_->removeShape(id);
  }
  crop_shape_ids_.clear();
  viewer_->removeShape(kCropFirstPointId);
  crop_first_corner_.reset();
  const bool was_active = crop_.active;
  crop_ = CropRegion{};
  if (current_.ok()) {
    update_picking_tree();
  }
  update_crop_information();
  if (was_active && current_.ok()) {
    render_cloud(true);
    update_bounds_overlay();
    statusBar()->showMessage(QStringLiteral("已恢复完整点云。"), 5000);
  }
  request_render();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MainWindow::active_display_cloud() const
{
  if (transform_preview_display_) {
    return transform_preview_display_;
  }
  if (crop_.active && crop_.display_cloud) {
    return crop_.display_cloud;
  }
  return current_.display_cloud;
}

void MainWindow::update_crop_information()
{
  if (!crop_.active) {
    crop_status_label_->setText(QStringLiteral("未裁剪 · 选择“区域裁剪”后点两个对角点"));
    local_points_label_->setText(QStringLiteral("—"));
    local_size_label_->setText(QStringLiteral("—"));
    local_center_label_->setText(QStringLiteral("—"));
    local_quality_label_->setText(QStringLiteral("—"));
    reset_crop_button_->setEnabled(false);
    if (current_.ok()) {
      displayed_points_label_->setText(
        current_.metrics.display_downsampled ?
        QStringLiteral("%1（显示降采样，统计用全量）")
          .arg(count_text(current_.metrics.displayed_points)) :
        QStringLiteral("%1（完整显示）").arg(count_text(current_.metrics.displayed_points)));
    }
    return;
  }

  crop_status_label_->setText(QStringLiteral("已启用 · %1").arg(crop_.description));
  local_points_label_->setText(count_text(crop_.point_count));
  local_size_label_->setText(
    QStringLiteral("%1 × %2 × %3 m\n对角线 %4 m")
      .arg(crop_.major_size, 0, 'f', 3)
      .arg(crop_.minor_size, 0, 'f', 3)
      .arg(crop_.height, 0, 'f', 3)
      .arg(crop_.diagonal_3d, 0, 'f', 3));
  local_center_label_->setText(
    QStringLiteral("%1, %2, %3 m")
      .arg(crop_.centroid[0] - display_origin_[0], 0, 'f', 3)
      .arg(crop_.centroid[1] - display_origin_[1], 0, 'f', 3)
      .arg(crop_.centroid[2] - display_origin_[2], 0, 'f', 3));
  const QString spacing = std::isfinite(crop_.analysis.estimated_spacing) ?
    QStringLiteral("%1 m").arg(crop_.analysis.estimated_spacing, 0, 'f', 4) :
    QStringLiteral("不可估算");
  QString quality = QStringLiteral("%1 点/m² · 间距 %2")
    .arg(std::isfinite(crop_.analysis.density_xy) ?
      QString::number(crop_.analysis.density_xy, 'f', 1) : QStringLiteral("—"))
    .arg(spacing);
  if (crop_.analysis.plane.valid) {
    quality += QStringLiteral("\n平面 RMS %1 m · 坡度 %2°")
      .arg(crop_.analysis.plane.rms, 0, 'f', 4)
      .arg(crop_.analysis.plane.slope_degrees, 0, 'f', 2);
  }
  local_quality_label_->setText(quality);
  displayed_points_label_->setText(
    QStringLiteral("%1（当前局部区域）")
      .arg(count_text(crop_.display_cloud ? crop_.display_cloud->size() : 0)));
  reset_crop_button_->setEnabled(true);
}

void MainWindow::update_crop_overlay()
{
  if (!viewer_) {
    return;
  }
  for (const std::string & id : crop_shape_ids_) {
    viewer_->removeShape(id);
  }
  crop_shape_ids_.clear();
  if (!crop_.active) {
    return;
  }
  const double red = crop_.inverted ? 0.96 : 0.05;
  const double green = crop_.inverted ? 0.38 : 0.90;
  const double blue = crop_.inverted ? 0.68 : 0.88;
  auto add_edge = [this, red, green, blue](
      const pcl::PointXYZ & first, const pcl::PointXYZ & second, const std::string & id) {
      viewer_->addLine(first, second, red, green, blue, id);
      viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, id);
      crop_shape_ids_.push_back(id);
    };

  if (crop_.type == CropSelectionType::Box) {
    const BoxSelection & box = crop_.box;
    const std::array<pcl::PointXYZ, 8> corners{{
      pcl::PointXYZ(box.min_x, box.min_y, box.min_z),
      pcl::PointXYZ(box.min_x, box.max_y, box.min_z),
      pcl::PointXYZ(box.max_x, box.min_y, box.min_z),
      pcl::PointXYZ(box.max_x, box.max_y, box.min_z),
      pcl::PointXYZ(box.min_x, box.min_y, box.max_z),
      pcl::PointXYZ(box.min_x, box.max_y, box.max_z),
      pcl::PointXYZ(box.max_x, box.min_y, box.max_z),
      pcl::PointXYZ(box.max_x, box.max_y, box.max_z)}};
    const std::array<std::pair<int, int>, 12> edges{{
      {0, 1}, {0, 2}, {1, 3}, {2, 3},
      {4, 5}, {4, 6}, {5, 7}, {6, 7},
      {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
    for (std::size_t index = 0; index < edges.size(); ++index) {
      add_edge(corners[edges[index].first], corners[edges[index].second],
        "crop_box_" + std::to_string(index));
    }
  } else if (crop_.type == CropSelectionType::Polygon) {
    const PolygonSelection & polygon = crop_.polygon;
    for (std::size_t index = 0; index < polygon.vertices.size(); ++index) {
      const pcl::PointXYZ & first = polygon.vertices[index];
      const pcl::PointXYZ & second = polygon.vertices[(index + 1) % polygon.vertices.size()];
      const pcl::PointXYZ bottom_first(first.x, first.y, polygon.min_z);
      const pcl::PointXYZ bottom_second(second.x, second.y, polygon.min_z);
      const pcl::PointXYZ top_first(first.x, first.y, polygon.max_z);
      const pcl::PointXYZ top_second(second.x, second.y, polygon.max_z);
      add_edge(bottom_first, bottom_second, "crop_polygon_bottom_" + std::to_string(index));
      add_edge(top_first, top_second, "crop_polygon_top_" + std::to_string(index));
      add_edge(bottom_first, top_first, "crop_polygon_vertical_" + std::to_string(index));
    }
  }
}

void MainWindow::update_measurement_details(const MeasurementRecord * record)
{
  if (!record) {
    measurement_result_label_->setText(QStringLiteral("—"));
    measurement_extra_label_->setText(QStringLiteral("—"));
    measurement_metadata_label_->setText(QStringLiteral("—"));
    point_a_label_->setText(QStringLiteral("—"));
    point_b_label_->setText(QStringLiteral("—"));
    delta_label_->setText(QStringLiteral("—"));
    distance_3d_label_->setText(QStringLiteral("—"));
    horizontal_distance_label_->setText(QStringLiteral("—"));
    vertical_distance_label_->setText(QStringLiteral("—"));
    slope_label_->setText(QStringLiteral("—"));
    return;
  }

  measurement_result_label_->setText(measurement_primary_text(*record));
  measurement_extra_label_->setText(measurement_extra_text(*record));
  QString metadata = QStringLiteral("%1 · %2")
    .arg(record->name.isEmpty() ? measurement_kind_label(record->kind) : record->name)
    .arg(record->group.isEmpty() ? QStringLiteral("默认") : record->group);
  if (!record->created_at.isEmpty()) {
    metadata += QStringLiteral("\n%1").arg(record->created_at);
  }
  if (!record->note.isEmpty()) {
    metadata += QStringLiteral("\n备注：%1").arg(record->note);
  }
  if (!record->visible) {
    metadata += QStringLiteral("\n当前已隐藏");
  }
  measurement_metadata_label_->setText(metadata);

  point_a_label_->setText(format_coordinate(record->point_a));
  point_b_label_->setText(record->kind == MeasurementKind::Point ? QStringLiteral("—") :
    format_coordinate(record->point_b));
  delta_label_->setText(
    QStringLiteral("ΔX %1  ΔY %2  ΔZ %3 %4")
      .arg(record->dx * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(record->dy * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(record->dz * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(unit_suffix()));
  const bool linear = record->kind == MeasurementKind::Segment ||
    record->kind == MeasurementKind::Polyline || record->kind == MeasurementKind::Orthogonal;
  if (record->kind == MeasurementKind::Point) {
    delta_label_->setText(QStringLiteral("—"));
    distance_3d_label_->setText(QStringLiteral("—"));
    horizontal_distance_label_->setText(QStringLiteral("—"));
    vertical_distance_label_->setText(QStringLiteral("—"));
  } else if (record->kind == MeasurementKind::Area) {
    distance_3d_label_->setText(QStringLiteral("周长 %1").arg(format_distance(record->perimeter_3d)));
    horizontal_distance_label_->setText(
      QStringLiteral("水平周长 %1").arg(format_distance(record->perimeter_horizontal)));
    vertical_distance_label_->setText(QStringLiteral("拟合 RMS %1").arg(format_distance(record->fit_rms)));
  } else if (record->kind == MeasurementKind::Circle) {
    distance_3d_label_->setText(QStringLiteral("周长 %1").arg(format_distance(record->circumference)));
    horizontal_distance_label_->setText(QStringLiteral("半径 %1").arg(format_distance(record->radius)));
    vertical_distance_label_->setText(QStringLiteral("—"));
  } else if (record->kind == MeasurementKind::Angle) {
    distance_3d_label_->setText(QStringLiteral("两边合计 %1").arg(format_distance(record->perimeter_3d)));
    horizontal_distance_label_->setText(
      QStringLiteral("水平合计 %1").arg(format_distance(record->perimeter_horizontal)));
    vertical_distance_label_->setText(QStringLiteral("—"));
  } else {
    distance_3d_label_->setText(
      QStringLiteral("%1%2")
        .arg(format_distance(record->distance_3d))
        .arg(record->kind == MeasurementKind::Polyline ? QStringLiteral("（折线累计）") : QString()));
    horizontal_distance_label_->setText(
      QStringLiteral("%1%2")
        .arg(format_distance(record->horizontal))
        .arg(record->kind == MeasurementKind::Polyline ? QStringLiteral("（分段累计）") : QString()));
    vertical_distance_label_->setText(
      QStringLiteral("%1（带方向：%2）")
        .arg(format_distance(std::abs(record->dz)))
        .arg(format_distance(record->dz)));
  }
  if (linear && std::isfinite(record->slope_percent)) {
    slope_label_->setText(
      QStringLiteral("%1%  ·  %2°")
        .arg(record->slope_percent, 0, 'f', 2)
        .arg(record->angle_degrees, 0, 'f', 2));
  } else if (linear) {
    slope_label_->setText(QStringLiteral("垂直 · 90.00°"));
  } else {
    slope_label_->setText(QStringLiteral("—"));
  }
}

void MainWindow::rebuild_measurement_table()
{
  const QString selected_group = measurement_group_filter_ ?
    measurement_group_filter_->currentData().toString() : QString();
  if (measurement_group_filter_) {
    QStringList groups;
    for (const MeasurementRecord & record : measurements_) {
      const QString group = record.group.isEmpty() ? QStringLiteral("默认") : record.group;
      if (!groups.contains(group)) groups.append(group);
    }
    groups.sort(Qt::CaseInsensitive);
    measurement_group_filter_->blockSignals(true);
    measurement_group_filter_->clear();
    measurement_group_filter_->addItem(QStringLiteral("全部分组"), QString());
    for (const QString & group : groups) measurement_group_filter_->addItem(group, group);
    const int restored_group = measurement_group_filter_->findData(selected_group);
    measurement_group_filter_->setCurrentIndex(std::max(0, restored_group));
    measurement_group_filter_->blockSignals(false);
  }

  const QString search = measurement_search_edit_ ? measurement_search_edit_->text().trimmed() : QString();
  const QString type_filter = measurement_type_filter_ ?
    measurement_type_filter_->currentData().toString() : QString();
  const QString group_filter = measurement_group_filter_ ?
    measurement_group_filter_->currentData().toString() : QString();
  std::vector<int> visible_indices;
  visible_indices.reserve(measurements_.size());
  for (int index = 0; index < static_cast<int>(measurements_.size()); ++index) {
    const MeasurementRecord & record = measurements_[static_cast<std::size_t>(index)];
    const bool type_matches = type_filter.isEmpty() || measurement_kind_key(record.kind) == type_filter;
    const QString group = record.group.isEmpty() ? QStringLiteral("默认") : record.group;
    const bool group_matches = group_filter.isEmpty() || group == group_filter;
    const bool search_matches = search.isEmpty() ||
      record.name.contains(search, Qt::CaseInsensitive) ||
      group.contains(search, Qt::CaseInsensitive) ||
      record.note.contains(search, Qt::CaseInsensitive) ||
      measurement_kind_label(record.kind).contains(search, Qt::CaseInsensitive);
    if (type_matches && group_matches && search_matches) visible_indices.push_back(index);
  }

  measurement_table_->setRowCount(static_cast<int>(visible_indices.size()));
  double total_distance = 0.0;
  for (const MeasurementRecord & record : measurements_) {
    if (record.kind == MeasurementKind::Segment || record.kind == MeasurementKind::Polyline) {
      total_distance += record.distance_3d;
    }
  }
  for (int row = 0; row < static_cast<int>(visible_indices.size()); ++row) {
    const int record_index = visible_indices[static_cast<std::size_t>(row)];
    const MeasurementRecord & record = measurements_[static_cast<std::size_t>(record_index)];
    auto * id_item = numeric_item(QString::number(record.id));
    id_item->setData(Qt::UserRole, record_index);
    measurement_table_->setItem(row, 0, id_item);
    auto * type_item = new QTableWidgetItem(measurement_kind_label(record.kind));
    type_item->setForeground(QColor(record.color_hex));
    measurement_table_->setItem(row, 1, type_item);
    measurement_table_->setItem(row, 2, numeric_item(QString::number(record.vertices.size())));
    measurement_table_->setItem(row, 3, numeric_item(
      record.kind == MeasurementKind::Point ? QStringLiteral("—") : format_distance(record.distance_3d)));
    measurement_table_->setItem(row, 4, numeric_item(
      record.kind == MeasurementKind::Point ? QStringLiteral("—") : format_distance(record.horizontal)));
    measurement_table_->setItem(row, 5, numeric_item(
      record.kind == MeasurementKind::Point ? QStringLiteral("—") : format_distance(record.dz)));
    measurement_table_->setItem(row, 6, new QTableWidgetItem(measurement_primary_text(record)));
    measurement_table_->setItem(row, 7, new QTableWidgetItem(record.name));
    measurement_table_->setItem(row, 8, new QTableWidgetItem(record.group));
    if (!record.visible) {
      for (int column = 0; column < measurement_table_->columnCount(); ++column) {
        measurement_table_->item(row, column)->setForeground(QColor(QStringLiteral("#718793")));
      }
    }
  }
  measurement_total_label_->setText(
    QStringLiteral("线性测量累计：%1 · 显示 %2 / 共 %3 条记录")
      .arg(format_distance(total_distance)).arg(visible_indices.size()).arg(measurements_.size()));
}

void MainWindow::refresh_measurement_units()
{
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  if (viewer_) {
    render_all_measurements();
    request_render();
  }
}

void MainWindow::fit_view()
{
  if (!current_.ok()) {
    return;
  }
  viewer_->resetCamera();
  request_render();
}

void MainWindow::top_view()
{
  if (!current_.ok()) {
    return;
  }
  const OrientedBounds & box = current_.metrics.oriented;
  const double distance = std::max(5.0, box.diagonal_3d * 1.5);
  set_camera(box.center_x, box.center_y, box.center_z + distance,
    box.center_x, box.center_y, box.center_z, 0.0, 1.0, 0.0);
}

void MainWindow::front_view()
{
  if (!current_.ok()) {
    return;
  }
  const OrientedBounds & box = current_.metrics.oriented;
  const double distance = std::max(5.0, box.diagonal_3d * 1.5);
  set_camera(box.center_x, box.center_y - distance, box.center_z,
    box.center_x, box.center_y, box.center_z, 0.0, 0.0, 1.0);
}

void MainWindow::left_view()
{
  if (!current_.ok()) {
    return;
  }
  const OrientedBounds & box = current_.metrics.oriented;
  const double distance = std::max(5.0, box.diagonal_3d * 1.5);
  set_camera(box.center_x - distance, box.center_y, box.center_z,
    box.center_x, box.center_y, box.center_z, 0.0, 0.0, 1.0);
}

void MainWindow::isometric_view()
{
  if (!current_.ok()) {
    return;
  }
  const OrientedBounds & box = current_.metrics.oriented;
  const double distance = std::max(5.0, box.diagonal_3d);
  set_camera(box.center_x + distance, box.center_y - distance, box.center_z + distance * 0.8,
    box.center_x, box.center_y, box.center_z, 0.0, 0.0, 1.0);
}

void MainWindow::set_camera(
  double px, double py, double pz,
  double fx, double fy, double fz,
  double ux, double uy, double uz)
{
  viewer_->setCameraPosition(px, py, pz, fx, fy, fz, ux, uy, uz);
  renderer_->ResetCameraClippingRange();
  request_render();
}

void MainWindow::toggle_axes(bool enabled)
{
  if (!viewer_) {
    return;
  }
  viewer_->removeCoordinateSystem(kAxesId);
  if (enabled && current_.ok()) {
    const double scale = std::clamp(current_.metrics.oriented.diagonal_3d * 0.06, 0.5, 2.0);
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translation() = Eigen::Vector3f(
      static_cast<float>(display_origin_[0]), static_cast<float>(display_origin_[1]),
      static_cast<float>(display_origin_[2]));
    viewer_->addCoordinateSystem(scale, pose, kAxesId);
  }
  request_render();
}

void MainWindow::toggle_grid(bool)
{
  update_grid_overlay();
  request_render();
}

void MainWindow::toggle_bounds(bool)
{
  update_bounds_overlay();
  request_render();
}

void MainWindow::update_grid_overlay()
{
  if (!viewer_) {
    return;
  }
  for (const std::string & id : grid_shape_ids_) {
    viewer_->removeShape(id);
  }
  grid_shape_ids_.clear();
  if (!grid_check_->isChecked() || !current_.ok()) {
    return;
  }

  const Bounds3d & bounds = current_.metrics.robust_axis_bounds;
  const double largest_span = std::max(bounds.size_x(), bounds.size_y());
  double step = 1.0;
  if (largest_span > 100.0) {
    step = 10.0;
  } else if (largest_span > 50.0) {
    step = 5.0;
  } else if (largest_span > 25.0) {
    step = 2.0;
  }

  const double min_x = std::floor(bounds.min_x / step) * step;
  const double max_x = std::ceil(bounds.max_x / step) * step;
  const double min_y = std::floor(bounds.min_y / step) * step;
  const double max_y = std::ceil(bounds.max_y / step) * step;
  const float z = static_cast<float>(current_.metrics.oriented.z_min);
  int index = 0;

  for (double x = min_x; x <= max_x + step * 0.1; x += step) {
    pcl::PointXYZ a(static_cast<float>(x), static_cast<float>(min_y), z);
    pcl::PointXYZ b(static_cast<float>(x), static_cast<float>(max_y), z);
    const std::string id = "grid_" + std::to_string(index++);
    viewer_->addLine(a, b, 0.30, 0.36, 0.42, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.45, id);
    grid_shape_ids_.push_back(id);
  }
  for (double y = min_y; y <= max_y + step * 0.1; y += step) {
    pcl::PointXYZ a(static_cast<float>(min_x), static_cast<float>(y), z);
    pcl::PointXYZ b(static_cast<float>(max_x), static_cast<float>(y), z);
    const std::string id = "grid_" + std::to_string(index++);
    viewer_->addLine(a, b, 0.30, 0.36, 0.42, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.45, id);
    grid_shape_ids_.push_back(id);
  }
  grid_check_->setText(QStringLiteral("显示 %1 米网格").arg(step, 0, 'g', 4));
}

void MainWindow::update_bounds_overlay()
{
  if (!viewer_) {
    return;
  }
  for (const std::string & id : bounds_shape_ids_) {
    viewer_->removeShape(id);
  }
  bounds_shape_ids_.clear();
  if (!bounds_check_->isChecked() || !current_.ok() || crop_.active) {
    return;
  }

  const OrientedBounds & box = current_.metrics.oriented;
  const double half_major = box.major_size / 2.0;
  const double half_minor = box.minor_size / 2.0;
  std::array<pcl::PointXYZ, 8> corners;
  int corner = 0;
  for (int level = 0; level < 2; ++level) {
    const double z = level == 0 ? box.z_min : box.z_max;
    for (int major_sign : {-1, 1}) {
      for (int minor_sign : {-1, 1}) {
        pcl::PointXYZ point;
        point.x = static_cast<float>(box.center_x + major_sign * half_major * box.major_axis_x +
          minor_sign * half_minor * box.minor_axis_x);
        point.y = static_cast<float>(box.center_y + major_sign * half_major * box.major_axis_y +
          minor_sign * half_minor * box.minor_axis_y);
        point.z = static_cast<float>(z);
        corners[static_cast<std::size_t>(corner++)] = point;
      }
    }
  }

  const std::array<std::pair<int, int>, 12> edges{{
    {0, 1}, {0, 2}, {1, 3}, {2, 3},
    {4, 5}, {4, 6}, {5, 7}, {6, 7},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const std::string id = "robust_box_" + std::to_string(i);
    viewer_->addLine(corners[edges[i].first], corners[edges[i].second], 1.0, 0.48, 0.06, id);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, id);
    bounds_shape_ids_.push_back(id);
  }
}

void MainWindow::open_project()
{
  const QString initial_directory = pending_project_path_.isEmpty() ?
    (current_path_.isEmpty() ? QDir::homePath() : QFileInfo(current_path_).absolutePath()) :
    QFileInfo(pending_project_path_).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
    this, QStringLiteral("打开 PCD 测量工程"), initial_directory,
    QStringLiteral("PCD 测量工程 (*.pcdmeasure *.odinpcd);;JSON 文件 (*.json);;所有文件 (*)"));
  if (path.isEmpty()) {
    return;
  }

  open_project_path(path);
}

void MainWindow::open_project_path(const QString & path)
{
  QJsonObject root;
  QString error;
  if (!read_project_file(path, &root, &error)) {
    QMessageBox::critical(this, QStringLiteral("工程格式错误"), error);
    return;
  }
  const QString pcd_path = resolve_project_cloud_path(path, root, &error);
  if (pcd_path.isEmpty()) {
    QMessageBox::critical(this, QStringLiteral("找不到点云"), error);
    return;
  }

  pending_project_state_ = root;
  pending_project_path_ = QFileInfo(path).absoluteFilePath();
  if (pending_project_path_ != auto_recovery_path()) add_recent_project(pending_project_path_);
  begin_load(pcd_path);
}

bool MainWindow::read_project_file(
  const QString & path,
  QJsonObject * state,
  QString * error) const
{
  if (!state) {
    if (error) *error = QStringLiteral("工程状态接收对象为空。");
    return false;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法读取工程文件：%1").arg(path);
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = QStringLiteral("JSON 解析失败：%1").arg(parse_error.errorString());
    return false;
  }
  const QJsonObject root = document.object();
  const QString format = root.value(QStringLiteral("format")).toString();
  if (!format.isEmpty() && format != QStringLiteral("pcd-measure-project")) {
    if (error) *error = QStringLiteral("不是受支持的 PCD 测量工程：%1").arg(format);
    return false;
  }
  if (root.contains(QStringLiteral("version"))) {
    const QJsonValue version_value = root.value(QStringLiteral("version"));
    const int version = version_value.isDouble() ? version_value.toInt(-1) : -1;
    if (version < 1 || version > 7 ||
      std::abs(version_value.toDouble(-1.0) - static_cast<double>(version)) > 1e-9)
    {
      if (error) {
        *error = QStringLiteral("工程版本 %1 不受支持；当前支持 v1–v7。")
          .arg(version_value.toVariant().toString());
      }
      return false;
    }
  }
  const QJsonObject transform = root.value(QStringLiteral("transform")).toObject();
  if (transform.value(QStringLiteral("active")).toBool(false)) {
    const QJsonArray rows = transform.value(QStringLiteral("matrix")).toArray();
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    bool valid_matrix = rows.size() == 4;
    for (int row = 0; valid_matrix && row < 4; ++row) {
      const QJsonArray values = rows.at(row).toArray();
      valid_matrix = values.size() == 4;
      for (int column = 0; valid_matrix && column < 4; ++column) {
        valid_matrix = values.at(column).isDouble();
        matrix(row, column) = values.at(column).toDouble();
        valid_matrix = valid_matrix && std::isfinite(matrix(row, column));
      }
    }
    const double scale_x = std::hypot(matrix(0, 0), matrix(1, 0));
    const double scale_y = std::hypot(matrix(0, 1), matrix(1, 1));
    const double scale_z = std::abs(matrix(2, 2));
    const double tolerance = std::max(1e-8, scale_x * 1e-6);
    valid_matrix = valid_matrix && scale_x > 1e-9 && scale_x <= 1e6 &&
      std::abs(scale_x - scale_y) <= tolerance && std::abs(scale_x - scale_z) <= tolerance &&
      std::abs(matrix(0, 2)) <= tolerance && std::abs(matrix(1, 2)) <= tolerance &&
      std::abs(matrix(2, 0)) <= tolerance && std::abs(matrix(2, 1)) <= tolerance &&
      std::abs(matrix(0, 0) * matrix(0, 1) + matrix(1, 0) * matrix(1, 1)) <=
        tolerance * std::max(1.0, scale_x) &&
      std::abs(matrix(3, 0)) <= 1e-9 && std::abs(matrix(3, 1)) <= 1e-9 &&
      std::abs(matrix(3, 2)) <= 1e-9 && std::abs(matrix(3, 3) - 1.0) <= 1e-9;
    if (!valid_matrix) {
      if (error) *error = QStringLiteral("工程中的坐标变换矩阵无效或包含不支持的变形。");
      return false;
    }
  }
  if (root.value(QStringLiteral("pcd_path")).toString().trimmed().isEmpty() &&
    root.value(QStringLiteral("pcd_relative_path")).toString().trimmed().isEmpty())
  {
    if (error) *error = QStringLiteral("工程中缺少 PCD 文件路径。");
    return false;
  }
  *state = root;
  return true;
}

QString MainWindow::resolve_project_cloud_path(
  const QString & project_path,
  const QJsonObject & state,
  QString * error) const
{
  const QDir project_directory(QFileInfo(project_path).absolutePath());
  const QString stored_path = state.value(QStringLiteral("pcd_path")).toString().trimmed();
  const QString relative_path = state.value(QStringLiteral("pcd_relative_path"))
    .toString().trimmed();
  QStringList candidates;
  if (!stored_path.isEmpty()) {
    candidates.append(QFileInfo(stored_path).isAbsolute() ? stored_path :
      project_directory.absoluteFilePath(stored_path));
  }
  if (!relative_path.isEmpty()) {
    candidates.append(project_directory.absoluteFilePath(relative_path));
  }
  candidates.removeDuplicates();
  for (const QString & candidate : candidates) {
    const QFileInfo info(candidate);
    if (info.exists() && info.isFile() &&
      info.suffix().compare(QStringLiteral("pcd"), Qt::CaseInsensitive) == 0)
    {
      return info.absoluteFilePath();
    }
  }
  if (error) {
    *error = QStringLiteral("工程引用的 PCD 不存在或不是有效的 .pcd 文件：\n%1")
      .arg(candidates.join(QStringLiteral("\n")));
  }
  return QString();
}

QJsonObject MainWindow::build_project_state(const QString & project_path) const
{
  QJsonObject root;
  root.insert(QStringLiteral("format"), QStringLiteral("pcd-measure-project"));
  root.insert(QStringLiteral("version"), 7);
  root.insert(QStringLiteral("pcd_path"), current_.path);
  root.insert(QStringLiteral("pcd_relative_path"),
    QDir(QFileInfo(project_path).absolutePath()).relativeFilePath(current_.path));
  root.insert(QStringLiteral("saved_at"), QDateTime::currentDateTime().toString(Qt::ISODate));

  QJsonObject view;
  view.insert(QStringLiteral("color_mode"), color_mode_combo_->currentIndex());
  view.insert(QStringLiteral("point_size"), point_size_spin_->value());
  view.insert(QStringLiteral("display_point_limit"), display_limit_spin_->value());
  view.insert(QStringLiteral("background_mode"), background_combo_->currentIndex());
  view.insert(QStringLiteral("custom_background"), custom_background_hex_);
  view.insert(QStringLiteral("projection_mode"), projection_combo_->currentIndex());
  view.insert(QStringLiteral("unit"), unit_combo_->currentIndex());
  view.insert(QStringLiteral("interaction_mode"), interaction_mode_combo_->currentIndex());
  view.insert(QStringLiteral("axes"), axes_check_->isChecked());
  view.insert(QStringLiteral("grid"), grid_check_->isChecked());
  view.insert(QStringLiteral("bounds"), bounds_check_->isChecked());
  view.insert(QStringLiteral("context_cloud"), context_cloud_check_->isChecked());
  view.insert(QStringLiteral("measurement_labels"), measurement_labels_check_->isChecked());
  view.insert(QStringLiteral("display_origin_m"), QJsonArray{
    display_origin_[0], display_origin_[1], display_origin_[2]});
  root.insert(QStringLiteral("view"), view);

  QJsonObject transform;
  transform.insert(QStringLiteral("active"),
    !cumulative_transform_.isApprox(Eigen::Matrix4f::Identity(), 1e-7F));
  QJsonArray transform_rows;
  for (int row = 0; row < 4; ++row) {
    QJsonArray values;
    for (int column = 0; column < 4; ++column) values.append(cumulative_transform_(row, column));
    transform_rows.append(values);
  }
  transform.insert(QStringLiteral("matrix"), transform_rows);
  transform.insert(QStringLiteral("latest_parameters"), QJsonObject{
    {QStringLiteral("translation_m"), QJsonArray{last_transform_parameters_.translation_x,
      last_transform_parameters_.translation_y, last_transform_parameters_.translation_z}},
    {QStringLiteral("rotation_z_degrees"), last_transform_parameters_.rotation_z_degrees},
    {QStringLiteral("uniform_scale"), last_transform_parameters_.uniform_scale}});
  root.insert(QStringLiteral("transform"), transform);

  QJsonArray records;
  for (const MeasurementRecord & record : measurements_) records.append(measurement_to_json(record));
  root.insert(QStringLiteral("measurements"), records);

  QJsonObject crop;
  crop.insert(QStringLiteral("active"), crop_.active);
  if (crop_.active) {
    crop.insert(QStringLiteral("description"), crop_.description);
    crop.insert(QStringLiteral("inverted"), crop_.inverted);
    if (crop_.type == CropSelectionType::Box) {
      crop.insert(QStringLiteral("type"), QStringLiteral("box"));
      crop.insert(QStringLiteral("bounds_m"), QJsonArray{
        crop_.box.min_x, crop_.box.max_x, crop_.box.min_y,
        crop_.box.max_y, crop_.box.min_z, crop_.box.max_z});
    } else if (crop_.type == CropSelectionType::Polygon) {
      crop.insert(QStringLiteral("type"), QStringLiteral("polygon"));
      QJsonArray polygon;
      for (const pcl::PointXYZ & point : crop_.polygon.vertices) polygon.append(point_to_json(point));
      crop.insert(QStringLiteral("polygon_m"), polygon);
      crop.insert(QStringLiteral("z_range_m"), QJsonArray{crop_.polygon.min_z, crop_.polygon.max_z});
    } else {
      crop.insert(QStringLiteral("type"), QStringLiteral("derived"));
      const QString companion = QFileInfo(project_path).fileName() + QStringLiteral(".region.pcd");
      crop.insert(QStringLiteral("derived_pcd_relative_path"), companion);
    }
  }
  root.insert(QStringLiteral("crop"), crop);

  QJsonArray analyses;
  for (const AnalysisRecord & record : analysis_records_) {
    analyses.append(QJsonObject{{QStringLiteral("kind"), record.kind},
      {QStringLiteral("title"), record.title}, {QStringLiteral("summary"), record.summary},
      {QStringLiteral("created_at"), record.created_at}, {QStringLiteral("data"), record.data}});
  }
  root.insert(QStringLiteral("analyses"), analyses);

  QJsonObject comparison;
  comparison.insert(QStringLiteral("active"), comparison_.active);
  if (comparison_.active) {
    comparison.insert(QStringLiteral("second_path"), comparison_.second_path);
    comparison.insert(QStringLiteral("second_relative_path"),
      QDir(QFileInfo(project_path).absolutePath()).relativeFilePath(comparison_.second_path));
    comparison.insert(QStringLiteral("visible"), comparison_.visible);
    comparison.insert(QStringLiteral("opacity_percent"), comparison_.opacity_percent);
    comparison.insert(QStringLiteral("options"), QJsonObject{
      {QStringLiteral("run_icp"), comparison_.options.run_icp},
      {QStringLiteral("centroid_prealign"), comparison_.options.centroid_prealign},
      {QStringLiteral("voxel_size_m"), comparison_.options.voxel_size},
      {QStringLiteral("maximum_iterations"), comparison_.options.maximum_iterations},
      {QStringLiteral("maximum_correspondence_distance_m"),
        comparison_.options.maximum_correspondence_distance},
      {QStringLiteral("difference_threshold_m"), comparison_.options.difference_threshold}});
    const DistanceStatistics & values = comparison_.result.statistics;
    comparison.insert(QStringLiteral("summary"), QJsonObject{
      {QStringLiteral("mean_m"), values.mean}, {QStringLiteral("rmse_m"), values.rmse},
      {QStringLiteral("median_m"), values.median}, {QStringLiteral("p95_m"), values.p95},
      {QStringLiteral("maximum_m"), values.maximum},
      {QStringLiteral("over_threshold_ratio"), values.over_threshold_ratio}});
  }
  root.insert(QStringLiteral("comparison"), comparison);

  pcl::visualization::Camera camera;
  viewer_->getCameraParameters(camera);
  QJsonObject camera_json;
  camera_json.insert(QStringLiteral("position"), QJsonArray{camera.pos[0], camera.pos[1], camera.pos[2]});
  camera_json.insert(QStringLiteral("focal"), QJsonArray{camera.focal[0], camera.focal[1], camera.focal[2]});
  camera_json.insert(QStringLiteral("view_up"), QJsonArray{camera.view[0], camera.view[1], camera.view[2]});
  camera_json.insert(QStringLiteral("clip"), QJsonArray{camera.clip[0], camera.clip[1]});
  camera_json.insert(QStringLiteral("fovy"), camera.fovy);
  vtkCamera * active_camera = renderer_->GetActiveCamera();
  camera_json.insert(QStringLiteral("parallel_projection"),
    active_camera && active_camera->GetParallelProjection() != 0);
  camera_json.insert(QStringLiteral("parallel_scale"),
    active_camera ? active_camera->GetParallelScale() : 1.0);
  root.insert(QStringLiteral("camera"), camera_json);
  return root;
}

bool MainWindow::write_project_file(const QString & path, QString * error) const
{
  if (!current_.ok()) {
    if (error) *error = QStringLiteral("尚未加载点云。");
    return false;
  }
  const QJsonObject root = build_project_state(path);
  QSaveFile file(path);
  const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(payload) != payload.size()) {
    if (error) *error = QStringLiteral("无法写入工程文件：%1").arg(path);
    return false;
  }
  QString companion;
  bool companion_existed = false;
  if (crop_.active && crop_.type == CropSelectionType::Derived && crop_.full_cloud) {
    companion = path + QStringLiteral(".region.pcd");
    companion_existed = QFileInfo::exists(companion);
    if (!write_cloud_file(companion, crop_.full_cloud, error)) {
      file.cancelWriting();
      return false;
    }
  }
  if (!file.commit()) {
    if (!companion.isEmpty() && !companion_existed) QFile::remove(companion);
    if (error) *error = QStringLiteral("无法提交工程文件：%1").arg(path);
    return false;
  }
  return true;
}

void MainWindow::save_project()
{
  if (!current_.ok()) {
    return;
  }
  const QFileInfo cloud_info(current_.path);
  const QString suggested = pending_project_path_.isEmpty() ?
    cloud_info.absolutePath() + QDir::separator() + cloud_info.completeBaseName() +
      QStringLiteral(".pcdmeasure") : pending_project_path_;
  QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("保存 PCD 测量工程"), suggested,
    QStringLiteral("PCD 测量工程 (*.pcdmeasure)"));
  if (path.isEmpty()) {
    return;
  }
  if (!path.endsWith(QStringLiteral(".pcdmeasure"), Qt::CaseInsensitive)) {
    path += QStringLiteral(".pcdmeasure");
  }

  QString error;
  if (!write_project_file(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("工程保存失败"), error);
    return;
  }
  pending_project_path_ = QFileInfo(path).absoluteFilePath();
  add_recent_project(pending_project_path_);
  clear_auto_recovery();
  statusBar()->showMessage(QStringLiteral("工程已保存：%1").arg(path), 8000);
}

QString MainWindow::auto_recovery_path() const
{
  const QString directory = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
  return QDir(directory).filePath(QStringLiteral("session-recovery.pcdmeasure"));
}

void MainWindow::write_auto_recovery()
{
  if (!current_.ok() || load_watcher_.isRunning()) return;
  const bool has_work = !measurements_.empty() || !analysis_records_.empty() || crop_.active ||
    comparison_.active || !cumulative_transform_.isApprox(Eigen::Matrix4f::Identity(), 1e-7F) ||
    display_origin_ != std::array<double, 3>{0.0, 0.0, 0.0};
  if (!has_work) return;
  const QString path = auto_recovery_path();
  if (!QDir().mkpath(QFileInfo(path).absolutePath())) return;
  QString error;
  if (!write_project_file(path, &error)) {
    statusBar()->showMessage(QStringLiteral("自动恢复文件写入失败：%1").arg(error), 6000);
  }
}

void MainWindow::clear_auto_recovery()
{
  const QString path = auto_recovery_path();
  QFile::remove(path);
  QFile::remove(path + QStringLiteral(".region.pcd"));
}

void MainWindow::maybe_restore_auto_recovery()
{
  const QString path = auto_recovery_path();
  if (!QFileInfo::exists(path)) return;
  const QMessageBox::StandardButton choice = QMessageBox::question(
    this, QStringLiteral("发现未完成的工程"),
    QStringLiteral("检测到上次异常中断留下的自动恢复文件。是否恢复测量、视角与分析状态？"),
    QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
  if (choice == QMessageBox::Yes) {
    restoring_auto_recovery_ = true;
    open_project_path(path);
    if (!load_watcher_.isRunning() && pending_project_state_.isEmpty()) {
      restoring_auto_recovery_ = false;
    }
  } else {
    clear_auto_recovery();
  }
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  clear_auto_recovery();
  QMainWindow::closeEvent(event);
}

void MainWindow::restore_pending_project()
{
  if (pending_project_state_.isEmpty()) {
    return;
  }
  const QJsonObject root = pending_project_state_;
  pending_project_state_ = QJsonObject{};
  QStringList restore_warnings;

  const QJsonObject view = root.value(QStringLiteral("view")).toObject();
  color_mode_combo_->blockSignals(true);
  point_size_spin_->blockSignals(true);
  display_limit_spin_->blockSignals(true);
  background_combo_->blockSignals(true);
  projection_combo_->blockSignals(true);
  unit_combo_->blockSignals(true);
  interaction_mode_combo_->blockSignals(true);
  axes_check_->blockSignals(true);
  grid_check_->blockSignals(true);
  bounds_check_->blockSignals(true);
  context_cloud_check_->blockSignals(true);
  measurement_labels_check_->blockSignals(true);
  color_mode_combo_->setCurrentIndex(std::clamp(view.value(QStringLiteral("color_mode")).toInt(0), 0, 2));
  point_size_spin_->setValue(std::clamp(view.value(QStringLiteral("point_size")).toInt(2), 1, 10));
  display_limit_spin_->setValue(std::clamp(
    view.value(QStringLiteral("display_point_limit")).toInt(display_limit_spin_->value()),
    10000, 10000000));
  custom_background_hex_ = view.value(QStringLiteral("custom_background"))
    .toString(QStringLiteral("#07131D"));
  const int background_index = std::clamp(
    view.value(QStringLiteral("background_mode")).toInt(0), 0, 3);
  background_combo_->setCurrentIndex(background_index);
  if (background_index == 3) {
    background_combo_->setItemText(3,
      QStringLiteral("自定义 · %1").arg(custom_background_hex_));
  }
  projection_combo_->setCurrentIndex(std::clamp(
    view.value(QStringLiteral("projection_mode")).toInt(0), 0, 1));
  unit_combo_->setCurrentIndex(std::clamp(view.value(QStringLiteral("unit")).toInt(0), 0, 2));
  interaction_mode_combo_->setCurrentIndex(
    std::clamp(view.value(QStringLiteral("interaction_mode")).toInt(0), 0, 8));
  axes_check_->setChecked(view.value(QStringLiteral("axes")).toBool(true));
  grid_check_->setChecked(view.value(QStringLiteral("grid")).toBool(false));
  bounds_check_->setChecked(view.value(QStringLiteral("bounds")).toBool(true));
  context_cloud_check_->setChecked(view.value(QStringLiteral("context_cloud")).toBool(false));
  measurement_labels_check_->setChecked(
    view.value(QStringLiteral("measurement_labels")).toBool(true));
  const QJsonArray origin = view.value(QStringLiteral("display_origin_m")).toArray();
  if (origin.size() == 3) {
    const std::array<double, 3> restored_origin{
      origin.at(0).toDouble(), origin.at(1).toDouble(), origin.at(2).toDouble()};
    if (std::all_of(restored_origin.begin(), restored_origin.end(),
      [](double value) { return std::isfinite(value); }))
    {
      display_origin_ = restored_origin;
    } else {
      restore_warnings.append(QStringLiteral("显示原点无效，已恢复为世界原点"));
    }
  }
  color_mode_combo_->blockSignals(false);
  point_size_spin_->blockSignals(false);
  display_limit_spin_->blockSignals(false);
  background_combo_->blockSignals(false);
  projection_combo_->blockSignals(false);
  unit_combo_->blockSignals(false);
  interaction_mode_combo_->blockSignals(false);
  axes_check_->blockSignals(false);
  grid_check_->blockSignals(false);
  bounds_check_->blockSignals(false);
  context_cloud_check_->blockSignals(false);
  measurement_labels_check_->blockSignals(false);
  apply_background_mode(background_index);
  rebuild_current_display_cloud(static_cast<std::size_t>(display_limit_spin_->value()));

  const QJsonObject transform = root.value(QStringLiteral("transform")).toObject();
  if (transform.value(QStringLiteral("active")).toBool(false)) {
    const QJsonArray rows = transform.value(QStringLiteral("matrix")).toArray();
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    bool matrix_valid = rows.size() == 4;
    for (int row = 0; matrix_valid && row < 4; ++row) {
      const QJsonArray values = rows.at(row).toArray();
      matrix_valid = values.size() == 4;
      for (int column = 0; matrix_valid && column < 4; ++column) {
        matrix(row, column) = static_cast<float>(values.at(column).toDouble());
        matrix_valid = std::isfinite(matrix(row, column));
      }
    }
    if (matrix_valid) {
      CloudTransformParameters parameters;
      parameters.translation_x = matrix(0, 3);
      parameters.translation_y = matrix(1, 3);
      parameters.translation_z = matrix(2, 3);
      parameters.uniform_scale = std::hypot(matrix(0, 0), matrix(1, 0));
      parameters.rotation_z_degrees = std::atan2(matrix(1, 0), matrix(0, 0)) *
        180.0 / std::acos(-1.0);
      QString transform_error;
      if (!apply_cloud_transform(parameters, &transform_error)) {
        restore_warnings.append(QStringLiteral("坐标变换未恢复：%1").arg(transform_error));
      }
      const QJsonObject latest = transform.value(QStringLiteral("latest_parameters")).toObject();
      const QJsonArray translation = latest.value(QStringLiteral("translation_m")).toArray();
      if (translation.size() == 3) {
        last_transform_parameters_.translation_x = translation.at(0).toDouble();
        last_transform_parameters_.translation_y = translation.at(1).toDouble();
        last_transform_parameters_.translation_z = translation.at(2).toDouble();
      }
      last_transform_parameters_.rotation_z_degrees =
        latest.value(QStringLiteral("rotation_z_degrees")).toDouble(parameters.rotation_z_degrees);
      last_transform_parameters_.uniform_scale =
        latest.value(QStringLiteral("uniform_scale")).toDouble(parameters.uniform_scale);
    } else {
      restore_warnings.append(QStringLiteral("坐标变换矩阵无效，已跳过"));
    }
  }

  measurements_.clear();
  undo_history_.clear();
  redo_history_.clear();
  if (measurement_search_edit_) measurement_search_edit_->clear();
  if (measurement_type_filter_) measurement_type_filter_->setCurrentIndex(0);
  next_measurement_id_ = 1;
  const QJsonArray records = root.value(QStringLiteral("measurements")).toArray();
  int skipped_measurements = 0;
  for (const QJsonValue & value : records) {
    MeasurementRecord record;
    if (!measurement_from_json(value.toObject(), record)) {
      ++skipped_measurements;
      continue;
    }
    measurements_.push_back(record);
    next_measurement_id_ = std::max(next_measurement_id_, record.id + 1);
  }
  if (skipped_measurements > 0) {
    restore_warnings.append(QStringLiteral("跳过 %1 条无效测量").arg(skipped_measurements));
  }
  if (transform_backup_valid_ && cumulative_transform_.allFinite()) {
    transform_backup_measurements_.clear();
    const Eigen::Matrix4f inverse = cumulative_transform_.inverse();
    for (const MeasurementRecord & record : measurements_) {
      const MeasurementRecord restored = transform_measurement_record(record, inverse);
      if (restored.valid) transform_backup_measurements_.push_back(restored);
    }
  }

  const QJsonObject crop = root.value(QStringLiteral("crop")).toObject();
  if (crop.value(QStringLiteral("active")).toBool(false)) {
    const QString type = crop.value(QStringLiteral("type")).toString();
    bool crop_restored = false;
    if (type == QStringLiteral("box")) {
      const QJsonArray values = crop.value(QStringLiteral("bounds_m")).toArray();
      if (values.size() == 6) {
        crop_restored = apply_box_crop(BoxSelection{
          values.at(0).toDouble(), values.at(1).toDouble(),
          values.at(2).toDouble(), values.at(3).toDouble(),
          values.at(4).toDouble(), values.at(5).toDouble(),
          crop.value(QStringLiteral("inverted")).toBool(false)}, false);
      }
    } else if (type == QStringLiteral("polygon")) {
      PolygonSelection selection;
      for (const QJsonValue & value : crop.value(QStringLiteral("polygon_m")).toArray()) {
        pcl::PointXYZ point;
        if (json_to_point(value, point)) selection.vertices.push_back(point);
      }
      const QJsonArray z_range = crop.value(QStringLiteral("z_range_m")).toArray();
      if (z_range.size() == 2) {
        selection.min_z = z_range.at(0).toDouble();
        selection.max_z = z_range.at(1).toDouble();
      }
      selection.inverted = crop.value(QStringLiteral("inverted")).toBool(false);
      crop_restored = apply_polygon_crop(selection, false);
    } else if (type == QStringLiteral("derived")) {
      const QString relative = crop.value(QStringLiteral("derived_pcd_relative_path")).toString();
      const QString path = QDir(QFileInfo(pending_project_path_).absolutePath()).absoluteFilePath(relative);
      if (!relative.isEmpty() && QFileInfo::exists(path)) {
        const CloudLoadResult derived = load_pcd_and_analyze(path);
        if (derived.ok()) {
          crop_restored = apply_local_cloud(derived.cloud, CropSelectionType::Derived,
            crop.value(QStringLiteral("description")).toString(QStringLiteral("恢复的分析结果")), false);
        }
      }
    } else if (crop.contains(QStringLiteral("min_x"))) {
      // v1–v4 stored only an XY rectangle.
      apply_crop(
        crop.value(QStringLiteral("min_x")).toDouble(),
        crop.value(QStringLiteral("max_x")).toDouble(),
        crop.value(QStringLiteral("min_y")).toDouble(),
        crop.value(QStringLiteral("max_y")).toDouble());
      crop_restored = crop_.active;
    } else {
      render_cloud(false);
    }
    if (crop_restored) {
      const QString description = crop.value(QStringLiteral("description")).toString();
      if (!description.isEmpty()) {
        crop_.description = description;
        update_crop_information();
      }
    } else {
      restore_warnings.append(QStringLiteral("局部区域无效或配套 PCD 缺失，已跳过"));
    }
  } else {
    render_cloud(false);
  }

  analysis_records_.clear();
  for (const QJsonValue & value : root.value(QStringLiteral("analyses")).toArray()) {
    const QJsonObject object = value.toObject();
    const QString kind = object.value(QStringLiteral("kind")).toString();
    const QString title = object.value(QStringLiteral("title")).toString();
    if (kind.isEmpty() || title.isEmpty()) continue;
    analysis_records_.push_back(AnalysisRecord{kind, title,
      object.value(QStringLiteral("summary")).toString(),
      object.value(QStringLiteral("created_at")).toString(),
      object.value(QStringLiteral("data")).toObject()});
    if (analysis_records_.size() >= 200) break;
  }

  for (const MeasurementRecord & record : measurements_) {
    render_measurement(record);
  }
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  toggle_axes(axes_check_->isChecked());
  update_grid_overlay();
  update_bounds_overlay();
  update_crop_overlay();
  interaction_mode_changed(interaction_mode_combo_->currentIndex());

  const QJsonObject camera_json = root.value(QStringLiteral("camera")).toObject();
  const QJsonArray position = camera_json.value(QStringLiteral("position")).toArray();
  const QJsonArray focal = camera_json.value(QStringLiteral("focal")).toArray();
  const QJsonArray view_up = camera_json.value(QStringLiteral("view_up")).toArray();
  const auto finite_triplet = [](const QJsonArray & values) {
      return values.size() == 3 && values.at(0).isDouble() && values.at(1).isDouble() &&
        values.at(2).isDouble() && std::isfinite(values.at(0).toDouble()) &&
        std::isfinite(values.at(1).toDouble()) && std::isfinite(values.at(2).toDouble());
    };
  const double camera_distance = finite_triplet(position) && finite_triplet(focal) ?
    std::sqrt(std::pow(position.at(0).toDouble() - focal.at(0).toDouble(), 2.0) +
      std::pow(position.at(1).toDouble() - focal.at(1).toDouble(), 2.0) +
      std::pow(position.at(2).toDouble() - focal.at(2).toDouble(), 2.0)) : 0.0;
  const double view_up_length = finite_triplet(view_up) ?
    std::sqrt(std::pow(view_up.at(0).toDouble(), 2.0) +
      std::pow(view_up.at(1).toDouble(), 2.0) +
      std::pow(view_up.at(2).toDouble(), 2.0)) : 0.0;
  if (camera_distance > 1e-9 && view_up_length > 1e-9) {
    viewer_->setCameraPosition(
      position.at(0).toDouble(), position.at(1).toDouble(), position.at(2).toDouble(),
      focal.at(0).toDouble(), focal.at(1).toDouble(), focal.at(2).toDouble(),
      view_up.at(0).toDouble(), view_up.at(1).toDouble(), view_up.at(2).toDouble());
    const QJsonArray clip = camera_json.value(QStringLiteral("clip")).toArray();
    if (clip.size() == 2) {
      const double near_distance = clip.at(0).toDouble();
      const double far_distance = clip.at(1).toDouble();
      if (std::isfinite(near_distance) && std::isfinite(far_distance) &&
        near_distance > 0.0 && far_distance > near_distance)
      {
        viewer_->setCameraClipDistances(near_distance, far_distance);
      }
    }
    if (camera_json.contains(QStringLiteral("fovy"))) {
      const double field_of_view = camera_json.value(QStringLiteral("fovy")).toDouble();
      if (std::isfinite(field_of_view) && field_of_view > 0.0 && field_of_view < std::acos(-1.0)) {
        viewer_->setCameraFieldOfView(field_of_view);
      }
    }
  } else if (!camera_json.isEmpty()) {
    restore_warnings.append(QStringLiteral("相机参数无效，已使用默认视角"));
  }
  vtkCamera * active_camera = renderer_->GetActiveCamera();
  if (active_camera) {
    const bool parallel = camera_json.contains(QStringLiteral("parallel_projection")) ?
      camera_json.value(QStringLiteral("parallel_projection")).toBool(false) :
      projection_combo_->currentIndex() == 1;
    active_camera->SetParallelProjection(parallel ? 1 : 0);
    if (parallel && camera_json.contains(QStringLiteral("parallel_scale"))) {
      active_camera->SetParallelScale(
        std::max(0.01, camera_json.value(QStringLiteral("parallel_scale")).toDouble(1.0)));
    }
    projection_combo_->blockSignals(true);
    projection_combo_->setCurrentIndex(parallel ? 1 : 0);
    projection_combo_->blockSignals(false);
  }
  renderer_->ResetCameraClippingRange();

  bool comparison_restored = false;
  const QJsonObject comparison = root.value(QStringLiteral("comparison")).toObject();
  if (comparison.value(QStringLiteral("active")).toBool(false)) {
    QString second_path = comparison.value(QStringLiteral("second_path")).toString();
    if (!QFileInfo::exists(second_path)) {
      const QString relative = comparison.value(QStringLiteral("second_relative_path")).toString();
      if (!relative.isEmpty()) {
        second_path = QDir(QFileInfo(pending_project_path_).absolutePath()).absoluteFilePath(relative);
      }
    }
    if (QFileInfo::exists(second_path)) {
      const QJsonObject options_json = comparison.value(QStringLiteral("options")).toObject();
      CloudComparisonOptions options;
      options.run_icp = options_json.value(QStringLiteral("run_icp")).toBool(true);
      options.centroid_prealign = options_json.value(QStringLiteral("centroid_prealign")).toBool(true);
      options.voxel_size = options_json.value(QStringLiteral("voxel_size_m")).toDouble(0.05);
      options.maximum_iterations = options_json.value(QStringLiteral("maximum_iterations")).toInt(60);
      options.maximum_correspondence_distance = options_json
        .value(QStringLiteral("maximum_correspondence_distance_m")).toDouble(0.5);
      options.difference_threshold = options_json
        .value(QStringLiteral("difference_threshold_m")).toDouble(0.05);
      options.maximum_display_points = display_limit_spin_ ?
        static_cast<std::size_t>(display_limit_spin_->value()) : 750000;
      const auto reference = current_.cloud;
      const ComparisonJobOutput output = run_progress_task<ComparisonJobOutput>(
        this, QStringLiteral("恢复双点云对比"), [reference, second_path, options]() {
          ComparisonJobOutput job;
          job.second = load_pcd_and_analyze(second_path, options.maximum_display_points);
          if (job.second.ok()) job.comparison = compare_point_clouds(reference, job.second.cloud, options);
          return job;
        });
      if (output.second.ok() && output.comparison.valid) {
        set_cloud_comparison(second_path, options,
          comparison.value(QStringLiteral("opacity_percent")).toInt(85), output.comparison);
        comparison_.visible = comparison.value(QStringLiteral("visible")).toBool(true);
        comparison_visibility_action_->setChecked(comparison_.visible);
        render_cloud_comparison();
        comparison_restored = true;
      } else {
        restore_warnings.append(QStringLiteral("双云对比重算失败，已跳过"));
      }
    } else {
      restore_warnings.append(QStringLiteral("第二点云缺失，已跳过双云对比"));
    }
  }

  QString restored_message = QStringLiteral("工程已恢复：%1 条测量%2%3")
      .arg(measurements_.size())
      .arg(crop_.active ? QStringLiteral("，包含局部裁剪") : QString())
      .arg(comparison_restored ? QStringLiteral("，双云对比已重算") : QString());
  if (!restore_warnings.isEmpty()) {
    restored_message += QStringLiteral("；") + restore_warnings.join(QStringLiteral("；"));
  }
  statusBar()->showMessage(restored_message, 12000);
  if (restoring_auto_recovery_) {
    clear_auto_recovery();
    pending_project_path_.clear();
    restoring_auto_recovery_ = false;
  }
  request_render();
}

void MainWindow::save_screenshot()
{
  if (!current_.ok()) {
    return;
  }
  const QFileInfo cloud_info(current_.path);
  const QString suggested = cloud_info.absolutePath() + QDir::separator() +
    cloud_info.completeBaseName() + QStringLiteral("_view_%1.png")
      .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
  QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("保存点云截图"), suggested, QStringLiteral("PNG 图片 (*.png)"));
  if (path.isEmpty()) {
    return;
  }
  if (!path.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)) {
    path += QStringLiteral(".png");
  }
  QString error;
  if (!write_view_screenshot(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("截图保存失败"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("截图已保存：%1").arg(path), 6000);
}

bool MainWindow::write_view_screenshot(const QString & path, QString * error) const
{
  if (!current_.ok() || !vtk_widget_) {
    if (error) *error = QStringLiteral("尚未加载可截图的点云。");
    return false;
  }
  return write_widget_png(vtk_widget_, path, error);
}

bool MainWindow::write_widget_png(QWidget * widget, const QString & path, QString * error) const
{
  return write_widget_png_atomically(widget, path, error);
}

void MainWindow::export_measurements()
{
  if (!current_.ok()) {
    return;
  }
  const QFileInfo cloud_info(current_.path);
  const QString suggested = cloud_info.absolutePath() + QDir::separator() +
    cloud_info.completeBaseName() + QStringLiteral("_measurements.csv");
  QString selected_filter;
  QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出测量和点云统计"), suggested,
    QStringLiteral("CSV 表格 (*.csv);;JSON 数据 (*.json)"), &selected_filter);
  if (path.isEmpty()) {
    return;
  }

  const bool json = selected_filter.startsWith(QStringLiteral("JSON")) ||
    path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive);
  if (json && !path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) {
    path += QStringLiteral(".json");
  } else if (!json && !path.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive)) {
    path += QStringLiteral(".csv");
  }

  QString error;
  if (!write_measurement_export(path, json, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("数据已导出：%1").arg(path), 7000);
}

bool MainWindow::write_measurement_export(
  const QString & path,
  bool json,
  QString * error) const
{
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) {
      *error = QStringLiteral("无法写入：%1").arg(path);
    }
    return false;
  }

  bool content_written = true;
  if (json) {
    QJsonObject root = QJsonDocument::fromJson(cloud_result_to_json(current_).toUtf8()).object();
    QJsonArray records;
    for (const MeasurementRecord & record : measurements_) {
      QJsonObject item = measurement_to_json(record);
      item.insert(QStringLiteral("node_count"), static_cast<qint64>(record.vertices.size()));
      records.append(item);
    }
    root.insert(QStringLiteral("measurements"), records);
    QJsonObject local_region;
    local_region.insert(QStringLiteral("active"), crop_.active);
    if (crop_.active) {
      const QString type = crop_.type == CropSelectionType::Box ? QStringLiteral("box") :
        (crop_.type == CropSelectionType::Polygon ? QStringLiteral("polygon") : QStringLiteral("derived"));
      local_region.insert(QStringLiteral("type"), type);
      local_region.insert(QStringLiteral("inverted"), crop_.inverted);
      local_region.insert(QStringLiteral("bounds_m"), QJsonArray{
        crop_.min_x, crop_.max_x, crop_.min_y, crop_.max_y, crop_.min_z, crop_.max_z});
      local_region.insert(QStringLiteral("point_count"), static_cast<qint64>(crop_.point_count));
      local_region.insert(QStringLiteral("extent_major_minor_height_m"),
        QJsonArray{crop_.major_size, crop_.minor_size, crop_.height});
      local_region.insert(QStringLiteral("diagonal_3d_m"), crop_.diagonal_3d);
      local_region.insert(QStringLiteral("density_xy_points_m2"),
        finite_json_value(crop_.analysis.density_xy));
      local_region.insert(QStringLiteral("estimated_spacing_m"),
        finite_json_value(crop_.analysis.estimated_spacing));
      if (crop_.analysis.plane.valid) {
        local_region.insert(QStringLiteral("plane_rms_m"), crop_.analysis.plane.rms);
        local_region.insert(QStringLiteral("plane_normal"), QJsonArray{
          crop_.analysis.plane.normal[0], crop_.analysis.plane.normal[1],
          crop_.analysis.plane.normal[2]});
      }
    }
    root.insert(QStringLiteral("local_region"), local_region);
    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    content_written = file.write(payload) == payload.size();
  } else {
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << QChar(0xFEFF);
    stream << "id,type,name,group,note,visible,created_at,node_count,"
              "a_x_m,a_y_m,a_z_m,b_x_m,b_y_m,b_z_m,dx_m,dy_m,dz_m,"
              "horizontal_m,distance_3d_m,angle_deg,azimuth_deg,slope_percent,"
              "perimeter_3d_m,perimeter_horizontal_m,area_planar_m2,area_horizontal_m2,"
              "signed_distance_m,radius_m,diameter_m,circumference_m,fit_rms_m,vertices_xyz_m\n";
    for (const MeasurementRecord & record : measurements_) {
      QStringList vertices_text;
      for (const pcl::PointXYZ & point : record.vertices) {
        vertices_text.append(QStringLiteral("%1 %2 %3")
          .arg(point.x, 0, 'f', 6).arg(point.y, 0, 'f', 6).arg(point.z, 0, 'f', 6));
      }
      stream << record.id << ','
             << measurement_kind_key(record.kind) << ','
             << csv_cell(record.name) << ','
             << csv_cell(record.group) << ','
             << csv_cell(record.note) << ','
             << (record.visible ? "true" : "false") << ','
             << csv_cell(record.created_at) << ','
             << record.vertices.size() << ','
             << QString::number(record.point_a.x, 'f', 6) << ','
             << QString::number(record.point_a.y, 'f', 6) << ','
             << QString::number(record.point_a.z, 'f', 6) << ','
             << QString::number(record.point_b.x, 'f', 6) << ','
             << QString::number(record.point_b.y, 'f', 6) << ','
             << QString::number(record.point_b.z, 'f', 6) << ','
             << QString::number(record.dx, 'f', 6) << ','
             << QString::number(record.dy, 'f', 6) << ','
             << QString::number(record.dz, 'f', 6) << ','
             << QString::number(record.horizontal, 'f', 6) << ','
             << QString::number(record.distance_3d, 'f', 6) << ','
             << QString::number(record.angle_degrees, 'f', 6) << ','
             << QString::number(record.azimuth_degrees, 'f', 6) << ','
             << (std::isfinite(record.slope_percent) ?
                QString::number(record.slope_percent, 'f', 6) : QStringLiteral("inf"))
             << ',' << QString::number(record.perimeter_3d, 'f', 6)
             << ',' << QString::number(record.perimeter_horizontal, 'f', 6)
             << ',' << QString::number(record.area_planar, 'f', 6)
             << ',' << QString::number(record.area_horizontal, 'f', 6)
             << ',' << QString::number(record.signed_distance, 'f', 6)
             << ',' << QString::number(record.radius, 'f', 6)
             << ',' << QString::number(record.diameter, 'f', 6)
             << ',' << QString::number(record.circumference, 'f', 6)
             << ',' << QString::number(record.fit_rms, 'f', 6)
             << ',' << csv_cell(vertices_text.join('|'))
             << '\n';
    }
    stream.flush();
    content_written = stream.status() == QTextStream::Ok;
  }

  if (!content_written) {
    file.cancelWriting();
    if (error) *error = QStringLiteral("写入导出内容失败：%1").arg(path);
    return false;
  }
  if (!file.commit()) {
    if (error) {
      *error = QStringLiteral("保存文件失败：%1").arg(path);
    }
    return false;
  }
  return true;
}

void MainWindow::set_loading(bool loading, const QString & message)
{
  progress_bar_->setVisible(loading);
  if (loading) {
    system_status_label_->setText(QStringLiteral("● LOADING"));
    system_status_label_->setStyleSheet(QStringLiteral(
      "font-family: 'DejaVu Sans Mono'; color: #FFB547; font-weight: 700;"));
    progress_bar_->setRange(0, 0);
    QApplication::setOverrideCursor(Qt::BusyCursor);
    reload_action_->setEnabled(false);
    screenshot_action_->setEnabled(false);
    export_action_->setEnabled(false);
    report_action_->setEnabled(false);
    output_tools_action_->setEnabled(false);
    advanced_tools_action_->setEnabled(false);
    comparison_tools_action_->setEnabled(false);
    coordinate_tools_action_->setEnabled(false);
    save_project_action_->setEnabled(false);
    statusBar()->showMessage(message);
  } else {
    progress_bar_->setRange(0, 1);
    QApplication::restoreOverrideCursor();
  }
}

double MainWindow::unit_scale() const
{
  return unit_combo_ ? unit_combo_->currentData().toDouble() : 1.0;
}

QString MainWindow::unit_suffix() const
{
  if (!unit_combo_) {
    return QStringLiteral("m");
  }
  switch (unit_combo_->currentIndex()) {
    case 1:
      return QStringLiteral("cm");
    case 2:
      return QStringLiteral("mm");
    default:
      return QStringLiteral("m");
  }
}

QString MainWindow::format_distance(double meters) const
{
  const double value = meters * unit_scale();
  const int precision = unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4);
  return QStringLiteral("%1 %2").arg(value, 0, 'f', precision).arg(unit_suffix());
}

QString MainWindow::format_area(double square_meters) const
{
  const double value = square_meters * unit_scale() * unit_scale();
  const int precision = unit_scale() == 1.0 ? 4 : 1;
  return QStringLiteral("%1 %2").arg(value, 0, 'f', precision).arg(unit_suffix()) +
    QChar(0x00B2);
}

QString MainWindow::measurement_primary_text(const MeasurementRecord & record) const
{
  switch (record.kind) {
    case MeasurementKind::Point:
      return format_coordinate(record.point_a);
    case MeasurementKind::Segment:
      return QStringLiteral("距离 %1").arg(format_distance(record.distance_3d));
    case MeasurementKind::Polyline:
      return QStringLiteral("累计 %1").arg(format_distance(record.distance_3d));
    case MeasurementKind::Angle:
      return QStringLiteral("角度 %1°").arg(record.angle_degrees, 0, 'f', 2);
    case MeasurementKind::Area:
      return QStringLiteral("面积 %1").arg(format_area(record.area_planar));
    case MeasurementKind::Orthogonal:
      return QStringLiteral("直线 %1").arg(format_distance(record.distance_3d));
    case MeasurementKind::PointToPlane:
      return QStringLiteral("垂距 %1").arg(format_distance(record.distance_3d));
    case MeasurementKind::Circle:
      return QStringLiteral("直径 %1").arg(format_distance(record.diameter));
  }
  return QStringLiteral("—");
}

QString MainWindow::measurement_extra_text(const MeasurementRecord & record) const
{
  switch (record.kind) {
    case MeasurementKind::Point:
      return QStringLiteral("已吸附到最近的真实点云点");
    case MeasurementKind::Segment:
      return QStringLiteral("水平 %1 · 高差 %2 · 方位 %3°")
        .arg(format_distance(record.horizontal))
        .arg(format_distance(record.dz))
        .arg(record.azimuth_degrees, 0, 'f', 2);
    case MeasurementKind::Polyline:
      return QStringLiteral("%1 个节点 · 水平累计 %2")
        .arg(record.vertices.size()).arg(format_distance(record.perimeter_horizontal));
    case MeasurementKind::Angle:
      return QStringLiteral("∠ABC · 两边合计 %1").arg(format_distance(record.perimeter_3d));
    case MeasurementKind::Area:
      return QStringLiteral("周长 %1 · 水平投影 %2 · 平面 RMS %3")
        .arg(format_distance(record.perimeter_3d))
        .arg(format_area(record.area_horizontal))
        .arg(format_distance(record.fit_rms));
    case MeasurementKind::Orthogonal:
      return QStringLiteral("ΔX %1 · ΔY %2 · ΔZ %3")
        .arg(format_distance(record.dx))
        .arg(format_distance(record.dy))
        .arg(format_distance(record.dz));
    case MeasurementKind::PointToPlane:
      return QStringLiteral("有向距离 %1 · 法向 [%2, %3, %4]")
        .arg(format_distance(record.signed_distance))
        .arg(record.normal[0], 0, 'f', 3)
        .arg(record.normal[1], 0, 'f', 3)
        .arg(record.normal[2], 0, 'f', 3);
    case MeasurementKind::Circle:
      return QStringLiteral("半径 %1 · 周长 %2")
        .arg(format_distance(record.radius)).arg(format_distance(record.circumference));
  }
  return QStringLiteral("—");
}

QString MainWindow::build_report_html() const
{
  if (!current_.ok()) {
    return QString();
  }
  const QFileInfo info(current_.path);
  const CloudMetrics & metrics = current_.metrics;
  const Bounds3d & raw = metrics.raw_bounds;
  const OrientedBounds & box = metrics.oriented;
  const double color_ratio = metrics.finite_points > 0 ?
    100.0 * static_cast<double>(metrics.non_black_points) /
      static_cast<double>(metrics.finite_points) : 0.0;

  QString measurement_rows;
  for (const MeasurementRecord & record : measurements_) {
    measurement_rows += QStringLiteral(
      "<tr><td>%1</td><td>%2</td><td>%3</td><td>%4</td><td>%5</td><td>%6</td></tr>")
      .arg(record.id)
      .arg(measurement_kind_label(record.kind).toHtmlEscaped())
      .arg(record.name.toHtmlEscaped())
      .arg(record.group.toHtmlEscaped())
      .arg(measurement_primary_text(record).toHtmlEscaped())
      .arg(record.note.toHtmlEscaped().replace(QStringLiteral("\n"), QStringLiteral("<br>")));
  }
  if (measurement_rows.isEmpty()) {
    measurement_rows = QStringLiteral(
      "<tr><td colspan='6' class='empty'>尚未保存测量记录</td></tr>");
  }

  QString crop_section;
  if (crop_.active) {
    crop_section = QStringLiteral(
      "<h2>当前局部区域</h2><div class='hero'>%1<br>点数 %2　主方向尺寸 %3 × %4 × %5 m　"
      "三维对角线 %6 m</div>")
      .arg(crop_.description.toHtmlEscaped()).arg(count_text(crop_.point_count))
      .arg(crop_.major_size, 0, 'f', 3).arg(crop_.minor_size, 0, 'f', 3)
      .arg(crop_.height, 0, 'f', 3).arg(crop_.diagonal_3d, 0, 'f', 3);
  }
  QString analysis_rows;
  for (std::size_t index = 0; index < analysis_records_.size(); ++index) {
    const AnalysisRecord & record = analysis_records_[index];
    analysis_rows += QStringLiteral("<tr><td>%1</td><td>%2</td><td>%3</td><td>%4</td></tr>")
      .arg(index + 1).arg(record.created_at.toHtmlEscaped())
      .arg(record.title.toHtmlEscaped()).arg(record.summary.toHtmlEscaped());
  }
  if (analysis_rows.isEmpty()) {
    analysis_rows = QStringLiteral("<tr><td colspan='4' class='empty'>尚未保存分析记录</td></tr>");
  }
  const QString analysis_section = QStringLiteral(
    "<h2>区域与工程分析</h2><table><tr><th>#</th><th>时间</th><th>类型</th>"
    "<th>结果摘要</th></tr>%1</table>").arg(analysis_rows);

  QString comparison_section;
  if (comparison_.active) {
    const DistanceStatistics & values = comparison_.result.statistics;
    comparison_section = QStringLiteral(
      "<h2>双点云配准与差异</h2><table><tr><th>第二点云</th><th>模式</th><th>RMSE</th>"
      "<th>P95</th><th>最大值</th><th>超阈值</th></tr><tr><td>%1</td><td>%2</td>"
      "<td>%3 m</td><td>%4 m</td><td>%5 m</td><td>%6%</td></tr></table>")
      .arg(QFileInfo(comparison_.second_path).fileName().toHtmlEscaped())
      .arg(comparison_.result.icp_requested ? QStringLiteral("ICP 配准") : QStringLiteral("直接比较"))
      .arg(values.rmse, 0, 'f', 6).arg(values.p95, 0, 'f', 6)
      .arg(values.maximum, 0, 'f', 6).arg(values.over_threshold_ratio * 100.0, 0, 'f', 2);
  }
  QString matrix_text;
  for (int row = 0; row < 4; ++row) {
    matrix_text += QStringLiteral("[%1, %2, %3, %4]%5")
      .arg(cumulative_transform_(row, 0), 0, 'f', 6)
      .arg(cumulative_transform_(row, 1), 0, 'f', 6)
      .arg(cumulative_transform_(row, 2), 0, 'f', 6)
      .arg(cumulative_transform_(row, 3), 0, 'f', 6)
      .arg(row == 3 ? QString() : QStringLiteral("<br>"));
  }
  const QString coordinate_section = QStringLiteral(
    "<h2>坐标状态</h2><table><tr><th>显示原点（世界坐标 m）</th><th>源点云累计变换矩阵</th></tr>"
    "<tr><td>%1, %2, %3<br><span class='muted'>显示偏移不改变源数据与距离</span></td>"
    "<td class='mono'>%4</td></tr></table>")
    .arg(display_origin_[0], 0, 'f', 6).arg(display_origin_[1], 0, 'f', 6)
    .arg(display_origin_[2], 0, 'f', 6).arg(matrix_text);

  return QStringLiteral(R"HTML(
<!doctype html><html><head><meta charset="utf-8"><style>
@page { margin: 14mm; }
body { font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; color: #162a35; font-size: 9.5pt; }
h1 { color: #087f83; font-size: 22pt; margin: 0 0 3mm 0; }
h2 { color: #174b5a; font-size: 13pt; border-bottom: 1px solid #8bc9cb; padding-bottom: 2mm; margin-top: 7mm; }
.meta { color: #526b75; margin-bottom: 5mm; }
.hero { background: #e9f7f7; border-left: 4px solid #10a7a6; padding: 3mm; }
.view { margin: 4mm 0; padding: 0; text-align: center; }
table { border-collapse: collapse; width: 100%; margin-top: 2mm; }
th { background: #123b4a; color: white; text-align: left; padding: 2mm; }
td { border: 1px solid #c4d6dc; padding: 1.8mm; vertical-align: top; }
tr:nth-child(even) td { background: #f3f8fa; }
.mono { font-family: "DejaVu Sans Mono", monospace; font-size: 8pt; }
.muted { color: #70858e; font-size: 8pt; }
tr { page-break-inside: avoid; }
.empty { color: #71838a; text-align: center; }
.foot { color: #667c84; font-size: 8pt; margin-top: 7mm; }
</style></head><body>
<h1>PCD 点云测量报告</h1>
<div class="meta">生成时间：%1<br>点云文件：%2<br>路径：%3</div>
<div class="hero"><b>推荐整体尺寸</b><br>
主方向长度 %4 m　宽度 %5 m　高度 %6 m　三维对角线 %7 m</div>
<p class="view"><img src="pcd-view.png" width="470"></p>
<h2>点云统计</h2>
<table><tr><th>总点数</th><th>有效点</th><th>无效点</th><th>彩色点</th><th>文件大小</th></tr>
<tr><td>%8</td><td>%9</td><td>%10</td><td>%11（%12%）</td><td>%13</td></tr></table>
<table><tr><th>坐标轴范围 X / Y / Z（m）</th><th>中心坐标（m）</th><th>估算点间距</th></tr>
<tr><td>%14 / %15 / %16</td><td>%17, %18, %19</td><td>%20 m</td></tr></table>
<h2>测量记录（当前显示单位：%21）</h2>
<table><tr><th>#</th><th>类型</th><th>名称</th><th>分组</th><th>主要结果</th><th>备注</th></tr>%22</table>
%23
%24
%25
%26
<div class="foot">尺寸采用每轴两端各剔除 0.5% 后的稳健主方向包围盒。测量精度受点云分辨率、标定与建图漂移影响。</div>
</body></html>)HTML")
    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")))
    .arg(info.fileName().toHtmlEscaped())
    .arg(current_.path.toHtmlEscaped())
    .arg(box.major_size, 0, 'f', 3)
    .arg(box.minor_size, 0, 'f', 3)
    .arg(box.height, 0, 'f', 3)
    .arg(box.diagonal_3d, 0, 'f', 3)
    .arg(count_text(metrics.header_points))
    .arg(count_text(metrics.finite_points))
    .arg(count_text(metrics.invalid_points))
    .arg(count_text(metrics.non_black_points))
    .arg(color_ratio, 0, 'f', 1)
    .arg(bytes_text(current_.file_bytes))
    .arg(QStringLiteral("%1…%2").arg(raw.min_x - display_origin_[0], 0, 'f', 3)
      .arg(raw.max_x - display_origin_[0], 0, 'f', 3))
    .arg(QStringLiteral("%1…%2").arg(raw.min_y - display_origin_[1], 0, 'f', 3)
      .arg(raw.max_y - display_origin_[1], 0, 'f', 3))
    .arg(QStringLiteral("%1…%2").arg(raw.min_z - display_origin_[2], 0, 'f', 3)
      .arg(raw.max_z - display_origin_[2], 0, 'f', 3))
    .arg(metrics.centroid[0] - display_origin_[0], 0, 'f', 3)
    .arg(metrics.centroid[1] - display_origin_[1], 0, 'f', 3)
    .arg(metrics.centroid[2] - display_origin_[2], 0, 'f', 3)
    .arg(metrics.estimated_spacing, 0, 'f', 4)
    .arg(unit_suffix().toHtmlEscaped())
    .arg(measurement_rows)
    .arg(crop_section)
    .arg(analysis_section)
    .arg(comparison_section)
    .arg(coordinate_section);
}

bool MainWindow::write_pdf_report(const QString & path, QString * error) const
{
  if (!current_.ok()) {
    if (error) {
      *error = QStringLiteral("尚未加载点云。");
    }
    return false;
  }

  const QFileInfo target(path);
  if (path.trimmed().isEmpty() || target.isDir() || !target.absoluteDir().exists()) {
    if (error) *error = QStringLiteral("PDF 导出目标无效：%1").arg(path);
    return false;
  }
  QTemporaryFile temporary(
    target.absoluteDir().filePath(QStringLiteral(".pcd-measure-report-XXXXXX.pdf")));
  temporary.setAutoRemove(true);
  if (!temporary.open()) {
    if (error) *error = QStringLiteral("无法在目标目录创建 PDF 临时文件。");
    return false;
  }
  const QString temporary_path = temporary.fileName();
  temporary.close();

  QPrinter printer(QPrinter::HighResolution);
  printer.setOutputFormat(QPrinter::PdfFormat);
  printer.setOutputFileName(temporary_path);
  printer.setPageSize(QPageSize(QPageSize::A4));
  printer.setPageMargins(QMarginsF(12.0, 12.0, 12.0, 12.0), QPageLayout::Millimeter);

  QTextDocument document;
  const QPixmap snapshot = vtk_widget_->grab();
  if (!snapshot.isNull()) {
    document.addResource(
      QTextDocument::ImageResource, QUrl(QStringLiteral("pcd-view.png")), snapshot.toImage());
  }
  document.setHtml(build_report_html());
  document.print(&printer);

  const QFileInfo output(temporary_path);
  if (!output.exists() || output.size() <= 0) {
    if (error) {
      *error = QStringLiteral("PDF 文件未能生成，请检查目标目录权限。");
    }
    return false;
  }
  if (!copy_file_atomically(temporary_path, path, QStringLiteral(" PDF 文件"), error)) {
    return false;
  }
  const bool written = QFileInfo(path).exists() && QFileInfo(path).size() > 0;
  if (!written && error) {
    *error = QStringLiteral("PDF 文件未能写入，请检查目标目录权限。");
  }
  return written;
}

void MainWindow::export_pdf_report()
{
  if (!current_.ok()) {
    return;
  }
  const QFileInfo cloud_info(current_.path);
  QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出 PDF 测量报告"),
    cloud_info.absolutePath() + QDir::separator() + cloud_info.completeBaseName() +
      QStringLiteral("_report.pdf"),
    QStringLiteral("PDF 报告 (*.pdf)"));
  if (path.isEmpty()) {
    return;
  }
  if (!path.endsWith(QStringLiteral(".pdf"), Qt::CaseInsensitive)) {
    path += QStringLiteral(".pdf");
  }
  QString error;
  if (!write_pdf_report(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("报告导出失败"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("PDF 报告已导出：%1").arg(path), 7000);
}

double MainWindow::marker_radius() const
{
  if (!current_.ok()) {
    return 0.05;
  }
  return std::clamp(current_.metrics.oriented.diagonal_3d * 0.004, 0.03, 0.22);
}

void MainWindow::request_render()
{
  if (vtk_widget_ && vtk_widget_->renderWindow()) {
    vtk_widget_->renderWindow()->Render();
    vtk_widget_->update();
  }
}

void MainWindow::dragEnterEvent(QDragEnterEvent * event)
{
  if (!event->mimeData()->hasUrls()) {
    return;
  }
  for (const QUrl & url : event->mimeData()->urls()) {
    const QString suffix = QFileInfo(url.toLocalFile()).suffix();
    if (url.isLocalFile() && (detect_rosbag_kind(url.toLocalFile()) != RosbagKind::Unknown ||
      suffix.compare(QStringLiteral("pcd"), Qt::CaseInsensitive) == 0 ||
      suffix.compare(QStringLiteral("pcdmeasure"), Qt::CaseInsensitive) == 0 ||
      suffix.compare(QStringLiteral("odinpcd"), Qt::CaseInsensitive) == 0)) {
      event->acceptProposedAction();
      return;
    }
  }
}

void MainWindow::dropEvent(QDropEvent * event)
{
  for (const QUrl & url : event->mimeData()->urls()) {
    const QString path = url.toLocalFile();
    const QString suffix = QFileInfo(path).suffix();
    if (!path.isEmpty() && detect_rosbag_kind(path) != RosbagKind::Unknown) {
      event->acceptProposedAction();
      open_rosbag_dialog(path);
      return;
    }
    if (!path.isEmpty() && (suffix.compare(QStringLiteral("pcdmeasure"), Qt::CaseInsensitive) == 0 ||
      suffix.compare(QStringLiteral("odinpcd"), Qt::CaseInsensitive) == 0)) {
      event->acceptProposedAction();
      open_project_path(path);
      return;
    }
    if (!path.isEmpty() && suffix.compare(QStringLiteral("pcd"), Qt::CaseInsensitive) == 0) {
      event->acceptProposedAction();
      pending_project_state_ = QJsonObject{};
      pending_project_path_.clear();
      begin_load(path);
      return;
    }
  }
}
