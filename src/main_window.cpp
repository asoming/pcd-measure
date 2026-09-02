#include "main_window.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QColorDialog>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
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
#include <QLocale>
#include <QMessageBox>
#include <QMimeData>
#include <QMenu>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSettings>
#include <QShortcut>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QStringList>
#include <QTableWidget>
#include <QTextStream>
#include <QToolBar>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <QtConcurrent/QtConcurrentRun>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

#include <pcl/visualization/point_cloud_color_handlers.h>

namespace
{

constexpr const char * kCloudId = "pcd_cloud";
constexpr const char * kPendingPointId = "pending_measurement_point";
constexpr const char * kCropFirstPointId = "crop_first_corner";
constexpr const char * kAxesId = "coordinate_axes";

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

QString measurement_prefix(int id)
{
  return QStringLiteral("measurement_%1_").arg(id);
}

QTableWidgetItem * numeric_item(const QString & text)
{
  auto * item = new QTableWidgetItem(text);
  item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return item;
}

double sorted_quantile(const std::vector<float> & values, double probability)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double index = probability * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return static_cast<double>(values[lower]) * (1.0 - fraction) +
    static_cast<double>(values[upper]) * fraction;
}

QJsonArray point_to_json(const pcl::PointXYZ & point)
{
  return QJsonArray{point.x, point.y, point.z};
}

bool json_to_point(const QJsonValue & value, pcl::PointXYZ & point)
{
  const QJsonArray array = value.toArray();
  if (array.size() != 3) {
    return false;
  }
  point.x = static_cast<float>(array.at(0).toDouble());
  point.y = static_cast<float>(array.at(1).toDouble());
  point.z = static_cast<float>(array.at(2).toDouble());
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
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
    QComboBox, QSpinBox { background: #0B1E2A; color: #E0EDF1; border: 1px solid #31546A; border-radius: 6px; padding: 5px 9px; min-height: 23px; selection-background-color: #185064; }
    QComboBox:hover, QSpinBox:hover { border-color: #4B7890; background: #102A39; }
    QComboBox:focus, QSpinBox:focus { border: 1px solid #21D4D1; }
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

  save_project_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogSaveButton), QStringLiteral("保存工程"));
  save_project_action_->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+S")));
  save_project_action_->setEnabled(false);
  connect(save_project_action_, &QAction::triggered, this, &MainWindow::save_project);

  toolbar->addSeparator();
  screenshot_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DesktopIcon), QStringLiteral("保存截图"));
  screenshot_action_->setEnabled(false);
  connect(screenshot_action_, &QAction::triggered, this, &MainWindow::save_screenshot);

  export_action_ = toolbar->addAction(
    style()->standardIcon(QStyle::SP_DialogSaveButton), QStringLiteral("导出测量"));
  export_action_->setEnabled(false);
  connect(export_action_, &QAction::triggered, this, &MainWindow::export_measurements);

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
    QStringLiteral("区域裁剪")});
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
  crop_form->addRow(QStringLiteral("局部点数："), local_points_label_);
  crop_form->addRow(QStringLiteral("局部长宽高："), local_size_label_);
  crop_form->addRow(QStringLiteral("局部质心："), local_center_label_);
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
  measurement_form->addRow(QStringLiteral("点 A："), point_a_label_);
  measurement_form->addRow(QStringLiteral("点 B："), point_b_label_);
  measurement_form->addRow(QStringLiteral("坐标差："), delta_label_);
  measurement_form->addRow(QStringLiteral("三维距离："), distance_3d_label_);
  measurement_form->addRow(QStringLiteral("水平距离："), horizontal_distance_label_);
  measurement_form->addRow(QStringLiteral("高度差："), vertical_distance_label_);
  measurement_form->addRow(QStringLiteral("坡度："), slope_label_);
  measure_layout->addLayout(measurement_form);

  auto * measure_buttons = new QGridLayout;
  auto * new_button = new QPushButton(QStringLiteral("新测量"));
  new_button->setProperty("role", "primary");
  finish_polyline_button_ = new QPushButton(QStringLiteral("完成折线"));
  finish_polyline_button_->setProperty("role", "primary");
  finish_polyline_button_->setToolTip(QStringLiteral("保存当前折线（Ctrl+Enter）"));
  finish_polyline_button_->setEnabled(false);
  auto * undo_button = new QPushButton(QStringLiteral("撤销"));
  auto * clear_button = new QPushButton(QStringLiteral("全部清除"));
  clear_button->setProperty("role", "danger");
  connect(new_button, &QPushButton::clicked, this, &MainWindow::start_new_measurement);
  connect(finish_polyline_button_, &QPushButton::clicked, this, &MainWindow::finish_polyline);
  connect(undo_button, &QPushButton::clicked, this, &MainWindow::undo_measurement);
  connect(clear_button, &QPushButton::clicked, this, &MainWindow::clear_measurements);
  measure_buttons->addWidget(new_button, 0, 0);
  measure_buttons->addWidget(finish_polyline_button_, 0, 1);
  measure_buttons->addWidget(undo_button, 1, 0);
  measure_buttons->addWidget(clear_button, 1, 1);
  measure_layout->addLayout(measure_buttons);
  auto * finish_shortcut = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Return")), this);
  connect(finish_shortcut, &QShortcut::activated, this, &MainWindow::finish_polyline);

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

  measurement_table_ = new QTableWidget(0, 6);
  measurement_table_->setHorizontalHeaderLabels({
    QStringLiteral("#"), QStringLiteral("类型"), QStringLiteral("节点"),
    QStringLiteral("三维"), QStringLiteral("水平"), QStringLiteral("ΔZ")});
  measurement_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  measurement_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  measurement_table_->setAlternatingRowColors(true);
  measurement_table_->verticalHeader()->setVisible(false);
  measurement_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  measurement_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  measurement_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
  measurement_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  measurement_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  measurement_table_->setMinimumHeight(130);
  measure_layout->addWidget(measurement_table_);
  measurement_total_label_ = make_value_label(true);
  measurement_total_label_->setText(QStringLiteral("累计三维长度：0.0000 m"));
  measure_layout->addWidget(measurement_total_label_);
  connect(measurement_table_, &QTableWidget::cellClicked, this,
    [this](int row, int) {
      if (row >= 0 && row < static_cast<int>(measurements_.size())) {
        update_measurement_details(&measurements_[static_cast<std::size_t>(row)]);
      }
    });
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
  connect(axes_check_, &QCheckBox::toggled, this, &MainWindow::toggle_axes);
  connect(grid_check_, &QCheckBox::toggled, this, &MainWindow::toggle_grid);
  connect(bounds_check_, &QCheckBox::toggled, this, &MainWindow::toggle_bounds);
  view_layout->addWidget(axes_check_);
  view_layout->addWidget(grid_check_);
  view_layout->addWidget(bounds_check_);
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
  pending_project_state_ = QJsonObject{};
  pending_project_path_.clear();
  begin_load(path);
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
  for (const QString & path : stored) {
    if (!QFileInfo::exists(path)) {
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
  load_watcher_.setFuture(QtConcurrent::run([worker_path]() {
    try {
      return load_pcd_and_analyze(worker_path);
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
    reload_action_->setEnabled(has_cloud);
    screenshot_action_->setEnabled(has_cloud);
    export_action_->setEnabled(has_cloud);
    save_project_action_->setEnabled(has_cloud);
    QMessageBox::critical(this, QStringLiteral("PCD 加载失败"), result.error);
    system_status_label_->setText(QStringLiteral("● ERROR"));
    system_status_label_->setStyleSheet(QStringLiteral(
      "font-family: 'DejaVu Sans Mono'; color: #FF8075; font-weight: 700;"));
    statusBar()->showMessage(QStringLiteral("加载失败：%1").arg(result.error), 8000);
    return;
  }

  clear_measurements();
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
      .arg(metrics.centroid[0], 0, 'f', 3)
      .arg(metrics.centroid[1], 0, 'f', 3)
      .arg(metrics.centroid[2], 0, 'f', 3));
  lowest_point_label_->setText(
    QStringLiteral("X %1  Y %2  Z %3 m")
      .arg(metrics.lowest_point[0], 0, 'f', 3)
      .arg(metrics.lowest_point[1], 0, 'f', 3)
      .arg(metrics.lowest_point[2], 0, 'f', 3));
  highest_point_label_->setText(
    QStringLiteral("X %1  Y %2  Z %3 m")
      .arg(metrics.highest_point[0], 0, 'f', 3)
      .arg(metrics.highest_point[1], 0, 'f', 3)
      .arg(metrics.highest_point[2], 0, 'f', 3));
  raw_height_label_->setText(QStringLiteral("%1 m").arg(raw.size_z(), 0, 'f', 3));
  spacing_label_->setText(std::isfinite(metrics.estimated_spacing) ?
    QStringLiteral("%1 m（中位最近邻）").arg(metrics.estimated_spacing, 0, 'f', 4) :
    QStringLiteral("无法估算"));
  coordinate_range_label_->setText(
    QStringLiteral("X [%1, %2]\nY [%3, %4]\nZ [%5, %6] m")
      .arg(raw.min_x, 0, 'f', 3).arg(raw.max_x, 0, 'f', 3)
      .arg(raw.min_y, 0, 'f', 3).arg(raw.max_y, 0, 'f', 3)
      .arg(raw.min_z, 0, 'f', 3).arg(raw.max_z, 0, 'f', 3));

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

  if (reset_camera) {
    viewer_->resetCamera();
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
    default:
      accept_picked_point(point);
      break;
  }
}

void MainWindow::accept_picked_point(const pcl::PointXYZ & point)
{
  if (!pending_point_) {
    pending_point_ = point;
    viewer_->removeShape(kPendingPointId);
    viewer_->addSphere(point, marker_radius(), 1.0, 0.35, 0.15, kPendingPointId);
    point_a_label_->setText(coordinate_text(point, unit_scale(), unit_suffix()));
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
    MeasurementKind::Segment, {*pending_point_, point_b}, next_measurement_id_++);

  viewer_->removeShape(kPendingPointId);
  pending_point_.reset();
  measurements_.push_back(record);
  render_measurement(record);
  rebuild_measurement_table();
  update_measurement_details(&measurements_.back());
  measurement_table_->selectRow(measurement_table_->rowCount() - 1);
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
  MeasurementRecord record;
  record.id = id;
  record.kind = kind;
  record.vertices = vertices;
  if (vertices.empty()) {
    return record;
  }

  record.point_a = vertices.front();
  record.point_b = vertices.back();
  record.dx = static_cast<double>(record.point_b.x) - record.point_a.x;
  record.dy = static_cast<double>(record.point_b.y) - record.point_a.y;
  record.dz = static_cast<double>(record.point_b.z) - record.point_a.z;
  for (std::size_t i = 1; i < vertices.size(); ++i) {
    const double dx = static_cast<double>(vertices[i].x) - vertices[i - 1].x;
    const double dy = static_cast<double>(vertices[i].y) - vertices[i - 1].y;
    const double dz = static_cast<double>(vertices[i].z) - vertices[i - 1].z;
    const double horizontal = std::hypot(dx, dy);
    record.horizontal += horizontal;
    record.distance_3d += std::hypot(horizontal, dz);
  }
  const double endpoint_horizontal = std::hypot(record.dx, record.dy);
  record.angle_degrees = std::atan2(std::abs(record.dz), endpoint_horizontal) *
    180.0 / std::acos(-1.0);
  record.slope_percent = endpoint_horizontal > 1e-12 ?
    100.0 * record.dz / endpoint_horizontal :
    std::numeric_limits<double>::infinity();
  return record;
}

void MainWindow::accept_polyline_point(const pcl::PointXYZ & point)
{
  active_polyline_points_.push_back(point);
  render_active_polyline();
  finish_polyline_button_->setEnabled(active_polyline_points_.size() >= 2);
  const MeasurementRecord preview = make_measurement_record(
    MeasurementKind::Polyline, active_polyline_points_, 0);
  update_measurement_details(&preview);
  picking_instruction_label_->setText(
    QStringLiteral("折线已有 %1 个节点，继续选点，完成后点击“完成折线”。")
      .arg(active_polyline_points_.size()));
  statusBar()->showMessage(
    QStringLiteral("折线节点 %1 · 当前累计 %2")
      .arg(active_polyline_points_.size())
      .arg(format_distance(preview.distance_3d)), 5000);
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

  if (active_polyline_points_.size() >= 2) {
    const MeasurementRecord preview = make_measurement_record(
      MeasurementKind::Polyline, active_polyline_points_, 0);
    pcl::PointXYZ label_point = active_polyline_points_.back();
    label_point.z += static_cast<float>(radius * 1.6);
    const std::string text_id = "active_poly_text";
    viewer_->addText3D(
      QStringLiteral("累计 %1").arg(format_distance(preview.distance_3d)).toStdString(),
      label_point, std::max(0.22, radius * 2.4), 0.96, 0.76, 0.24, text_id);
    active_shape_ids_.push_back(text_id);
  }
  request_render();
}

void MainWindow::finish_polyline()
{
  if (active_polyline_points_.size() < 2) {
    statusBar()->showMessage(QStringLiteral("折线至少需要两个节点。"), 4000);
    return;
  }

  const MeasurementRecord record = make_measurement_record(
    MeasurementKind::Polyline, active_polyline_points_, next_measurement_id_++);
  for (const std::string & id : active_shape_ids_) {
    viewer_->removeShape(id);
  }
  active_shape_ids_.clear();
  active_polyline_points_.clear();
  finish_polyline_button_->setEnabled(false);
  measurements_.push_back(record);
  render_measurement(record);
  rebuild_measurement_table();
  update_measurement_details(&measurements_.back());
  measurement_table_->selectRow(measurement_table_->rowCount() - 1);
  picking_instruction_label_->setText(
    QStringLiteral("折线已保存。Shift + 左键可开始下一条折线。"));
  statusBar()->showMessage(
    QStringLiteral("折线 #%1：%2 个节点，累计 %3")
      .arg(record.id).arg(record.vertices.size()).arg(format_distance(record.distance_3d)), 8000);
  request_render();
}

void MainWindow::render_measurement(const MeasurementRecord & record)
{
  const std::string prefix = measurement_prefix(record.id).toStdString();
  const double radius = marker_radius();
  for (std::size_t i = 0; i < record.vertices.size(); ++i) {
    const bool first = i == 0;
    const bool last = i + 1 == record.vertices.size();
    const double red = first ? 1.0 : (last ? 0.15 : 0.12);
    const double green = first ? 0.18 : (last ? 0.90 : 0.72);
    const double blue = first ? 0.12 : (last ? 0.35 : 0.78);
    viewer_->addSphere(record.vertices[i], radius, red, green, blue,
      prefix + "node_" + std::to_string(i));
    if (i > 0) {
      const std::string line_id = prefix + "line_" + std::to_string(i - 1);
      viewer_->addLine(record.vertices[i - 1], record.vertices[i], 1.0, 0.70, 0.16, line_id);
      viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, line_id);
    }
  }

  pcl::PointXYZ label_point = record.point_b;
  label_point.z += static_cast<float>(radius * 1.6);
  viewer_->addText3D(
    QStringLiteral("#%1 %2  %3")
      .arg(record.id)
      .arg(record.kind == MeasurementKind::Polyline ? QStringLiteral("折线") : QStringLiteral("两点"))
      .arg(format_distance(record.distance_3d)).toStdString(),
    label_point, std::max(0.22, radius * 2.4), 1.0, 0.92, 0.28, prefix + "text");
}

void MainWindow::remove_measurement_shapes(int id)
{
  const std::string prefix = measurement_prefix(id).toStdString();
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
    finish_polyline_button_->setEnabled(active_polyline_points_.size() >= 2);
    if (active_polyline_points_.empty()) {
      update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
    } else {
      const MeasurementRecord preview = make_measurement_record(
        MeasurementKind::Polyline, active_polyline_points_, 0);
      update_measurement_details(&preview);
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

  if (measurements_.empty()) {
    statusBar()->showMessage(QStringLiteral("没有可撤销的测量。"), 3000);
    return;
  }
  remove_measurement_shapes(measurements_.back().id);
  measurements_.pop_back();
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  statusBar()->showMessage(QStringLiteral("已撤销上一条测量。"), 3000);
  request_render();
}

void MainWindow::clear_measurements()
{
  if (!viewer_) {
    return;
  }
  clear_active_selection();
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

void MainWindow::interaction_mode_changed(int index)
{
  if (!viewer_) {
    return;
  }
  clear_active_selection();
  if (index == 1) {
    picking_instruction_label_->setText(
      QStringLiteral("连续折线：Shift + 左键依次添加节点，然后点击“完成折线”。"));
    viewer_hint_label_->setText(
      QStringLiteral("折线模式 · Shift + 左键添加节点 · “完成折线”保存"));
  } else if (index == 2) {
    picking_instruction_label_->setText(
      QStringLiteral("区域裁剪：建议使用俯视图，Shift + 左键选择两个 XY 对角点。"));
    viewer_hint_label_->setText(
      QStringLiteral("裁剪模式 · 选两个 XY 对角点 · 按完整高度保留区域"));
    if (current_.ok()) {
      top_view();
    }
  } else {
    picking_instruction_label_->setText(
      QStringLiteral("两点测距：Shift + 左键依次选择 A 点和 B 点；自动吸附到完整点云最近点。"));
    viewer_hint_label_->setText(
      QStringLiteral("两点模式 · Shift + 左键选择 A/B 点 · 完整点云吸附 · 左键旋转 · 中键平移"));
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

  auto local_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  local_cloud->reserve(current_.cloud->size() / 4);
  std::vector<float> major_values;
  std::vector<float> minor_values;
  std::vector<float> z_values;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  const OrientedBounds & global_box = current_.metrics.oriented;

  for (const pcl::PointXYZRGB & point : *current_.cloud) {
    if (point.x < min_x || point.x > max_x || point.y < min_y || point.y > max_y) {
      continue;
    }
    local_cloud->push_back(point);
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    major_values.push_back(static_cast<float>(
      point.x * global_box.major_axis_x + point.y * global_box.major_axis_y));
    minor_values.push_back(static_cast<float>(
      point.x * global_box.minor_axis_x + point.y * global_box.minor_axis_y));
    z_values.push_back(point.z);
  }

  if (local_cloud->size() < 3) {
    QMessageBox::warning(this, QStringLiteral("区域内没有足够点"),
      QStringLiteral("该区域只有 %1 个点，请重新选择更大的范围。")
        .arg(local_cloud->size()));
    return;
  }

  local_cloud->width = static_cast<std::uint32_t>(local_cloud->size());
  local_cloud->height = 1;
  local_cloud->is_dense = true;
  std::sort(major_values.begin(), major_values.end());
  std::sort(minor_values.begin(), minor_values.end());
  std::sort(z_values.begin(), z_values.end());

  crop_ = CropRegion{};
  crop_.active = true;
  crop_.min_x = min_x;
  crop_.max_x = max_x;
  crop_.min_y = min_y;
  crop_.max_y = max_y;
  crop_.min_z = sorted_quantile(z_values, 0.005);
  crop_.max_z = sorted_quantile(z_values, 0.995);
  crop_.major_size = sorted_quantile(major_values, 0.995) -
    sorted_quantile(major_values, 0.005);
  crop_.minor_size = sorted_quantile(minor_values, 0.995) -
    sorted_quantile(minor_values, 0.005);
  crop_.height = crop_.max_z - crop_.min_z;
  crop_.diagonal_3d = std::sqrt(
    crop_.major_size * crop_.major_size + crop_.minor_size * crop_.minor_size +
    crop_.height * crop_.height);
  crop_.point_count = local_cloud->size();
  const double count = static_cast<double>(crop_.point_count);
  crop_.centroid = {sum_x / count, sum_y / count, sum_z / count};
  crop_.full_cloud = local_cloud;

  constexpr std::size_t maximum_display_points = 2500000;
  if (local_cloud->size() <= maximum_display_points) {
    crop_.display_cloud = local_cloud;
  } else {
    crop_.display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
      static_cast<double>(local_cloud->size()) / maximum_display_points));
    crop_.display_cloud->reserve(maximum_display_points);
    for (std::size_t i = 0; i < local_cloud->size(); i += stride) {
      crop_.display_cloud->push_back((*local_cloud)[i]);
    }
    crop_.display_cloud->width = static_cast<std::uint32_t>(crop_.display_cloud->size());
    crop_.display_cloud->height = 1;
    crop_.display_cloud->is_dense = true;
  }

  update_picking_tree();
  render_cloud(false);
  update_crop_information();
  update_crop_overlay();
  update_bounds_overlay();
  const double center_x = (crop_.min_x + crop_.max_x) / 2.0;
  const double center_y = (crop_.min_y + crop_.max_y) / 2.0;
  const double center_z = (crop_.min_z + crop_.max_z) / 2.0;
  const double camera_distance = std::max(3.0, crop_.diagonal_3d * 1.6);
  set_camera(
    center_x, center_y, center_z + camera_distance,
    center_x, center_y, center_z,
    0.0, 1.0, 0.0);
  picking_instruction_label_->setText(
    QStringLiteral("区域裁剪完成。可测量局部点云，或点击“恢复完整点云”。"));
  statusBar()->showMessage(
    QStringLiteral("局部区域：%1 个点，尺寸 %2 × %3 × %4 m")
      .arg(count_text(crop_.point_count))
      .arg(crop_.major_size, 0, 'f', 3)
      .arg(crop_.minor_size, 0, 'f', 3)
      .arg(crop_.height, 0, 'f', 3), 10000);
  request_render();
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

  crop_status_label_->setText(
    QStringLiteral("已启用 · XY [%1, %2] × [%3, %4] m")
      .arg(crop_.min_x, 0, 'f', 2).arg(crop_.max_x, 0, 'f', 2)
      .arg(crop_.min_y, 0, 'f', 2).arg(crop_.max_y, 0, 'f', 2));
  local_points_label_->setText(count_text(crop_.point_count));
  local_size_label_->setText(
    QStringLiteral("%1 × %2 × %3 m\n对角线 %4 m")
      .arg(crop_.major_size, 0, 'f', 3)
      .arg(crop_.minor_size, 0, 'f', 3)
      .arg(crop_.height, 0, 'f', 3)
      .arg(crop_.diagonal_3d, 0, 'f', 3));
  local_center_label_->setText(
    QStringLiteral("%1, %2, %3 m")
      .arg(crop_.centroid[0], 0, 'f', 3)
      .arg(crop_.centroid[1], 0, 'f', 3)
      .arg(crop_.centroid[2], 0, 'f', 3));
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

  const std::array<pcl::PointXYZ, 8> corners{{
    pcl::PointXYZ(static_cast<float>(crop_.min_x), static_cast<float>(crop_.min_y), static_cast<float>(crop_.min_z)),
    pcl::PointXYZ(static_cast<float>(crop_.min_x), static_cast<float>(crop_.max_y), static_cast<float>(crop_.min_z)),
    pcl::PointXYZ(static_cast<float>(crop_.max_x), static_cast<float>(crop_.min_y), static_cast<float>(crop_.min_z)),
    pcl::PointXYZ(static_cast<float>(crop_.max_x), static_cast<float>(crop_.max_y), static_cast<float>(crop_.min_z)),
    pcl::PointXYZ(static_cast<float>(crop_.min_x), static_cast<float>(crop_.min_y), static_cast<float>(crop_.max_z)),
    pcl::PointXYZ(static_cast<float>(crop_.min_x), static_cast<float>(crop_.max_y), static_cast<float>(crop_.max_z)),
    pcl::PointXYZ(static_cast<float>(crop_.max_x), static_cast<float>(crop_.min_y), static_cast<float>(crop_.max_z)),
    pcl::PointXYZ(static_cast<float>(crop_.max_x), static_cast<float>(crop_.max_y), static_cast<float>(crop_.max_z))}};
  const std::array<std::pair<int, int>, 12> edges{{
    {0, 1}, {0, 2}, {1, 3}, {2, 3},
    {4, 5}, {4, 6}, {5, 7}, {6, 7},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const std::string id = "crop_box_" + std::to_string(i);
    viewer_->addLine(corners[edges[i].first], corners[edges[i].second], 0.05, 0.90, 0.88, id);
    viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, id);
    crop_shape_ids_.push_back(id);
  }
}

void MainWindow::update_measurement_details(const MeasurementRecord * record)
{
  if (!record) {
    point_a_label_->setText(QStringLiteral("—"));
    point_b_label_->setText(QStringLiteral("—"));
    delta_label_->setText(QStringLiteral("—"));
    distance_3d_label_->setText(QStringLiteral("—"));
    horizontal_distance_label_->setText(QStringLiteral("—"));
    vertical_distance_label_->setText(QStringLiteral("—"));
    slope_label_->setText(QStringLiteral("—"));
    return;
  }

  point_a_label_->setText(coordinate_text(record->point_a, unit_scale(), unit_suffix()));
  point_b_label_->setText(coordinate_text(record->point_b, unit_scale(), unit_suffix()));
  delta_label_->setText(
    QStringLiteral("ΔX %1  ΔY %2  ΔZ %3 %4")
      .arg(record->dx * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(record->dy * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(record->dz * unit_scale(), 0, 'f', unit_scale() >= 1000.0 ? 1 : (unit_scale() >= 100.0 ? 2 : 4))
      .arg(unit_suffix()));
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
  if (std::isfinite(record->slope_percent)) {
    slope_label_->setText(
      QStringLiteral("%1%  ·  %2°")
        .arg(record->slope_percent, 0, 'f', 2)
        .arg(record->angle_degrees, 0, 'f', 2));
  } else {
    slope_label_->setText(QStringLiteral("垂直 · 90.00°"));
  }
}

void MainWindow::rebuild_measurement_table()
{
  measurement_table_->setRowCount(static_cast<int>(measurements_.size()));
  double total_distance = 0.0;
  for (int row = 0; row < static_cast<int>(measurements_.size()); ++row) {
    const MeasurementRecord & record = measurements_[static_cast<std::size_t>(row)];
    total_distance += record.distance_3d;
    measurement_table_->setItem(row, 0, numeric_item(QString::number(record.id)));
    measurement_table_->setItem(row, 1, numeric_item(
      record.kind == MeasurementKind::Polyline ? QStringLiteral("折线") : QStringLiteral("两点")));
    measurement_table_->setItem(row, 2, numeric_item(QString::number(record.vertices.size())));
    measurement_table_->setItem(row, 3, numeric_item(format_distance(record.distance_3d)));
    measurement_table_->setItem(row, 4, numeric_item(format_distance(record.horizontal)));
    measurement_table_->setItem(row, 5, numeric_item(format_distance(record.dz)));
  }
  measurement_total_label_->setText(
    QStringLiteral("累计三维长度：%1").arg(format_distance(total_distance)));
}

void MainWindow::refresh_measurement_units()
{
  rebuild_measurement_table();
  update_measurement_details(measurements_.empty() ? nullptr : &measurements_.back());
  if (viewer_) {
    for (const MeasurementRecord & record : measurements_) {
      remove_measurement_shapes(record.id);
      render_measurement(record);
    }
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
    viewer_->addCoordinateSystem(scale, kAxesId);
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

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::critical(this, QStringLiteral("工程打开失败"),
      QStringLiteral("无法读取工程文件：\n%1").arg(path));
    return;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    QMessageBox::critical(this, QStringLiteral("工程格式错误"),
      QStringLiteral("JSON 解析失败：%1").arg(parse_error.errorString()));
    return;
  }

  const QJsonObject root = document.object();
  QString pcd_path = root.value(QStringLiteral("pcd_path")).toString();
  if (!QFileInfo::exists(pcd_path)) {
    const QString relative = root.value(QStringLiteral("pcd_relative_path")).toString();
    if (!relative.isEmpty()) {
      pcd_path = QDir(QFileInfo(path).absolutePath()).absoluteFilePath(relative);
    }
  }
  if (!QFileInfo::exists(pcd_path)) {
    QMessageBox::critical(this, QStringLiteral("找不到点云"),
      QStringLiteral("工程引用的 PCD 不存在：\n%1").arg(pcd_path));
    return;
  }

  pending_project_state_ = root;
  pending_project_path_ = QFileInfo(path).absoluteFilePath();
  begin_load(pcd_path);
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

  QJsonObject root;
  root.insert(QStringLiteral("format"), QStringLiteral("pcd-measure-project"));
  root.insert(QStringLiteral("version"), 3);
  root.insert(QStringLiteral("pcd_path"), current_.path);
  root.insert(QStringLiteral("pcd_relative_path"),
    QDir(QFileInfo(path).absolutePath()).relativeFilePath(current_.path));
  root.insert(QStringLiteral("saved_at"),
    QDateTime::currentDateTime().toString(Qt::ISODate));

  QJsonObject view;
  view.insert(QStringLiteral("color_mode"), color_mode_combo_->currentIndex());
  view.insert(QStringLiteral("point_size"), point_size_spin_->value());
  view.insert(QStringLiteral("background_mode"), background_combo_->currentIndex());
  view.insert(QStringLiteral("custom_background"), custom_background_hex_);
  view.insert(QStringLiteral("projection_mode"), projection_combo_->currentIndex());
  view.insert(QStringLiteral("unit"), unit_combo_->currentIndex());
  view.insert(QStringLiteral("interaction_mode"), interaction_mode_combo_->currentIndex());
  view.insert(QStringLiteral("axes"), axes_check_->isChecked());
  view.insert(QStringLiteral("grid"), grid_check_->isChecked());
  view.insert(QStringLiteral("bounds"), bounds_check_->isChecked());
  root.insert(QStringLiteral("view"), view);

  QJsonArray records;
  for (const MeasurementRecord & record : measurements_) {
    QJsonObject item;
    item.insert(QStringLiteral("id"), record.id);
    item.insert(QStringLiteral("kind"),
      record.kind == MeasurementKind::Polyline ? QStringLiteral("polyline") : QStringLiteral("segment"));
    QJsonArray vertices;
    for (const pcl::PointXYZ & point : record.vertices) {
      vertices.append(point_to_json(point));
    }
    item.insert(QStringLiteral("vertices_m"), vertices);
    records.append(item);
  }
  root.insert(QStringLiteral("measurements"), records);

  QJsonObject crop;
  crop.insert(QStringLiteral("active"), crop_.active);
  if (crop_.active) {
    crop.insert(QStringLiteral("min_x"), crop_.min_x);
    crop.insert(QStringLiteral("max_x"), crop_.max_x);
    crop.insert(QStringLiteral("min_y"), crop_.min_y);
    crop.insert(QStringLiteral("max_y"), crop_.max_y);
  }
  root.insert(QStringLiteral("crop"), crop);

  pcl::visualization::Camera camera;
  viewer_->getCameraParameters(camera);
  QJsonObject camera_json;
  camera_json.insert(QStringLiteral("position"),
    QJsonArray{camera.pos[0], camera.pos[1], camera.pos[2]});
  camera_json.insert(QStringLiteral("focal"),
    QJsonArray{camera.focal[0], camera.focal[1], camera.focal[2]});
  camera_json.insert(QStringLiteral("view_up"),
    QJsonArray{camera.view[0], camera.view[1], camera.view[2]});
  camera_json.insert(QStringLiteral("clip"), QJsonArray{camera.clip[0], camera.clip[1]});
  camera_json.insert(QStringLiteral("fovy"), camera.fovy);
  vtkCamera * active_camera = renderer_->GetActiveCamera();
  camera_json.insert(QStringLiteral("parallel_projection"),
    active_camera && active_camera->GetParallelProjection() != 0);
  camera_json.insert(QStringLiteral("parallel_scale"),
    active_camera ? active_camera->GetParallelScale() : 1.0);
  root.insert(QStringLiteral("camera"), camera_json);

  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly) ||
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0 || !file.commit())
  {
    QMessageBox::critical(this, QStringLiteral("工程保存失败"),
      QStringLiteral("无法写入工程文件：\n%1").arg(path));
    return;
  }
  pending_project_path_ = QFileInfo(path).absoluteFilePath();
  statusBar()->showMessage(QStringLiteral("工程已保存：%1").arg(path), 8000);
}

void MainWindow::restore_pending_project()
{
  if (pending_project_state_.isEmpty()) {
    return;
  }
  const QJsonObject root = pending_project_state_;
  pending_project_state_ = QJsonObject{};

  const QJsonObject view = root.value(QStringLiteral("view")).toObject();
  color_mode_combo_->blockSignals(true);
  point_size_spin_->blockSignals(true);
  background_combo_->blockSignals(true);
  projection_combo_->blockSignals(true);
  unit_combo_->blockSignals(true);
  interaction_mode_combo_->blockSignals(true);
  axes_check_->blockSignals(true);
  grid_check_->blockSignals(true);
  bounds_check_->blockSignals(true);
  color_mode_combo_->setCurrentIndex(std::clamp(view.value(QStringLiteral("color_mode")).toInt(0), 0, 2));
  point_size_spin_->setValue(std::clamp(view.value(QStringLiteral("point_size")).toInt(2), 1, 10));
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
    std::clamp(view.value(QStringLiteral("interaction_mode")).toInt(0), 0, 2));
  axes_check_->setChecked(view.value(QStringLiteral("axes")).toBool(true));
  grid_check_->setChecked(view.value(QStringLiteral("grid")).toBool(false));
  bounds_check_->setChecked(view.value(QStringLiteral("bounds")).toBool(true));
  color_mode_combo_->blockSignals(false);
  point_size_spin_->blockSignals(false);
  background_combo_->blockSignals(false);
  projection_combo_->blockSignals(false);
  unit_combo_->blockSignals(false);
  interaction_mode_combo_->blockSignals(false);
  axes_check_->blockSignals(false);
  grid_check_->blockSignals(false);
  bounds_check_->blockSignals(false);
  apply_background_mode(background_index);

  measurements_.clear();
  next_measurement_id_ = 1;
  const QJsonArray records = root.value(QStringLiteral("measurements")).toArray();
  for (const QJsonValue & value : records) {
    const QJsonObject item = value.toObject();
    std::vector<pcl::PointXYZ> vertices;
    for (const QJsonValue & point_value : item.value(QStringLiteral("vertices_m")).toArray()) {
      pcl::PointXYZ point;
      if (json_to_point(point_value, point)) {
        vertices.push_back(point);
      }
    }
    if (vertices.size() < 2) {
      continue;
    }
    const int id = std::max(1, item.value(QStringLiteral("id")).toInt(next_measurement_id_));
    const MeasurementKind kind = item.value(QStringLiteral("kind")).toString() ==
      QStringLiteral("polyline") ? MeasurementKind::Polyline : MeasurementKind::Segment;
    measurements_.push_back(make_measurement_record(kind, vertices, id));
    next_measurement_id_ = std::max(next_measurement_id_, id + 1);
  }

  const QJsonObject crop = root.value(QStringLiteral("crop")).toObject();
  if (crop.value(QStringLiteral("active")).toBool(false)) {
    apply_crop(
      crop.value(QStringLiteral("min_x")).toDouble(),
      crop.value(QStringLiteral("max_x")).toDouble(),
      crop.value(QStringLiteral("min_y")).toDouble(),
      crop.value(QStringLiteral("max_y")).toDouble());
  } else {
    render_cloud(false);
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
  if (position.size() == 3 && focal.size() == 3 && view_up.size() == 3) {
    viewer_->setCameraPosition(
      position.at(0).toDouble(), position.at(1).toDouble(), position.at(2).toDouble(),
      focal.at(0).toDouble(), focal.at(1).toDouble(), focal.at(2).toDouble(),
      view_up.at(0).toDouble(), view_up.at(1).toDouble(), view_up.at(2).toDouble());
    const QJsonArray clip = camera_json.value(QStringLiteral("clip")).toArray();
    if (clip.size() == 2) {
      viewer_->setCameraClipDistances(clip.at(0).toDouble(), clip.at(1).toDouble());
    }
    if (camera_json.contains(QStringLiteral("fovy"))) {
      viewer_->setCameraFieldOfView(camera_json.value(QStringLiteral("fovy")).toDouble());
    }
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

  statusBar()->showMessage(
    QStringLiteral("工程已恢复：%1 条测量%2")
      .arg(measurements_.size())
      .arg(crop_.active ? QStringLiteral("，包含局部裁剪") : QString()), 10000);
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
  const QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("保存点云截图"), suggested, QStringLiteral("PNG 图片 (*.png)"));
  if (path.isEmpty()) {
    return;
  }
  viewer_->saveScreenshot(path.toStdString());
  statusBar()->showMessage(QStringLiteral("截图已保存：%1").arg(path), 6000);
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

  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"),
      QStringLiteral("无法写入：%1").arg(path));
    return;
  }

  if (json) {
    QJsonObject root = QJsonDocument::fromJson(cloud_result_to_json(current_).toUtf8()).object();
    QJsonArray records;
    for (const MeasurementRecord & record : measurements_) {
      QJsonObject item;
      item.insert(QStringLiteral("id"), record.id);
      item.insert(QStringLiteral("kind"),
        record.kind == MeasurementKind::Polyline ? QStringLiteral("polyline") : QStringLiteral("segment"));
      item.insert(QStringLiteral("point_a_m"), point_to_json(record.point_a));
      item.insert(QStringLiteral("point_b_m"), point_to_json(record.point_b));
      QJsonArray vertices;
      for (const pcl::PointXYZ & point : record.vertices) {
        vertices.append(point_to_json(point));
      }
      item.insert(QStringLiteral("vertices_m"), vertices);
      item.insert(QStringLiteral("node_count"), static_cast<qint64>(record.vertices.size()));
      item.insert(QStringLiteral("delta_m"), QJsonArray{record.dx, record.dy, record.dz});
      item.insert(QStringLiteral("horizontal_distance_m"), record.horizontal);
      item.insert(QStringLiteral("distance_3d_m"), record.distance_3d);
      item.insert(QStringLiteral("angle_deg"), record.angle_degrees);
      item.insert(QStringLiteral("slope_percent"),
        std::isfinite(record.slope_percent) ? record.slope_percent : 0.0);
      records.append(item);
    }
    root.insert(QStringLiteral("measurements"), records);
    QJsonObject local_region;
    local_region.insert(QStringLiteral("active"), crop_.active);
    if (crop_.active) {
      local_region.insert(QStringLiteral("xy_bounds_m"),
        QJsonArray{crop_.min_x, crop_.max_x, crop_.min_y, crop_.max_y});
      local_region.insert(QStringLiteral("point_count"), static_cast<qint64>(crop_.point_count));
      local_region.insert(QStringLiteral("extent_major_minor_height_m"),
        QJsonArray{crop_.major_size, crop_.minor_size, crop_.height});
      local_region.insert(QStringLiteral("diagonal_3d_m"), crop_.diagonal_3d);
    }
    root.insert(QStringLiteral("local_region"), local_region);
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  } else {
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << QChar(0xFEFF);
    stream << "id,type,node_count,a_x_m,a_y_m,a_z_m,b_x_m,b_y_m,b_z_m,dx_m,dy_m,dz_m,"
              "horizontal_m,distance_3d_m,angle_deg,slope_percent,vertices_xyz_m\n";
    for (const MeasurementRecord & record : measurements_) {
      QStringList vertices_text;
      for (const pcl::PointXYZ & point : record.vertices) {
        vertices_text.append(QStringLiteral("%1 %2 %3")
          .arg(point.x, 0, 'f', 6).arg(point.y, 0, 'f', 6).arg(point.z, 0, 'f', 6));
      }
      stream << record.id << ','
             << (record.kind == MeasurementKind::Polyline ? "polyline" : "segment") << ','
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
             << (std::isfinite(record.slope_percent) ?
                QString::number(record.slope_percent, 'f', 6) : QStringLiteral("inf"))
             << ',' << '"' << vertices_text.join('|') << '"'
             << '\n';
    }
  }

  if (!file.commit()) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), QStringLiteral("保存文件失败。"));
    return;
  }
  statusBar()->showMessage(QStringLiteral("数据已导出：%1").arg(path), 7000);
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
    if (url.isLocalFile() && (suffix.compare(QStringLiteral("pcd"), Qt::CaseInsensitive) == 0 ||
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
