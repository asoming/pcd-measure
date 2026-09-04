#include "environment_setup_panel.h"

#include <algorithm>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QColor>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include <unistd.h>

namespace
{

const QColor kReadyColor(QStringLiteral("#79E6E0"));
const QColor kMissingColor(QStringLiteral("#FFB35A"));
const QColor kOptionalColor(QStringLiteral("#8FAAB6"));

}  // namespace

EnvironmentSetupPanel::EnvironmentSetupPanel(QWidget * parent)
: QWidget(parent)
{
  setObjectName(QStringLiteral("environmentSetupPanel"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setStyleSheet(QStringLiteral(R"(
    QWidget#environmentSetupPanel { background:#06141E; color:#DCE9ED; }
    QFrame#environmentHero { background:#0B202D; border:1px solid #2A4C5E; border-top:3px solid #25D0C8; border-radius:8px; }
    QLabel#environmentConsoleMark { color:#79E6E0; font:700 10px 'DejaVu Sans Mono'; letter-spacing:1.4px; }
    QLabel#environmentTitle { color:#F1FAFC; font-size:22px; font-weight:700; }
    QLabel#environmentSubtitle { color:#8FAAB6; }
    QLabel#environmentOverall { font:700 10px 'DejaVu Sans Mono'; padding:7px 10px; border-radius:5px; }
    QFrame#environmentChecklist, QFrame#environmentActions { background:#081924; border:1px solid #29485C; border-radius:7px; }
    QLabel#environmentSection { color:#7895A2; font:700 10px 'DejaVu Sans Mono'; letter-spacing:1px; }
    QLabel#environmentSecurity { color:#9AB3BD; background:#0B202C; border-left:3px solid #25D0C8; padding:9px; }
  )"));

  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(12, 12, 12, 10);
  outer->setSpacing(9);

  auto * hero = new QFrame(this);
  hero->setObjectName(QStringLiteral("environmentHero"));
  auto * hero_layout = new QHBoxLayout(hero);
  hero_layout->setContentsMargins(17, 11, 17, 11);
  auto * hero_text = new QVBoxLayout;
  hero_text->setSpacing(1);
  auto * console_mark = new QLabel(QStringLiteral("SYSTEM // READINESS CHECK"), hero);
  console_mark->setObjectName(QStringLiteral("environmentConsoleMark"));
  auto * title = new QLabel(QStringLiteral("环境自检与一键安装"), hero);
  title->setObjectName(QStringLiteral("environmentTitle"));
  auto * subtitle = new QLabel(QStringLiteral(
    "检查 GitHub 源码运行所需环境，只补齐缺失项，不改动设备驱动配置。"), hero);
  subtitle->setObjectName(QStringLiteral("environmentSubtitle"));
  hero_text->addWidget(console_mark);
  hero_text->addWidget(title);
  hero_text->addWidget(subtitle);
  hero_layout->addLayout(hero_text, 1);
  overall_status_label_ = new QLabel(QStringLiteral("CHECKING · 正在检测"), hero);
  overall_status_label_->setObjectName(QStringLiteral("environmentOverall"));
  hero_layout->addWidget(overall_status_label_, 0, Qt::AlignVCenter);
  outer->addWidget(hero);

  auto * content = new QHBoxLayout;
  content->setSpacing(9);
  auto * checklist_frame = new QFrame(this);
  checklist_frame->setObjectName(QStringLiteral("environmentChecklist"));
  auto * checklist_layout = new QVBoxLayout(checklist_frame);
  checklist_layout->setContentsMargins(10, 10, 10, 10);
  auto * checklist_mark = new QLabel(QStringLiteral("DEPENDENCY MATRIX // 本机状态"), checklist_frame);
  checklist_mark->setObjectName(QStringLiteral("environmentSection"));
  checklist_layout->addWidget(checklist_mark);
  environment_table_ = new QTableWidget(6, 3, checklist_frame);
  environment_table_->setObjectName(QStringLiteral("environmentTable"));
  environment_table_->setHorizontalHeaderLabels({
    QStringLiteral("组件"), QStringLiteral("状态"), QStringLiteral("作用与处理")});
  environment_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  environment_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  environment_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  environment_table_->verticalHeader()->setVisible(false);
  environment_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  environment_table_->setSelectionMode(QAbstractItemView::NoSelection);
  environment_table_->setAlternatingRowColors(true);
  environment_table_->setWordWrap(true);
  environment_table_->setShowGrid(false);
  environment_table_->verticalHeader()->setDefaultSectionSize(54);
  checklist_layout->addWidget(environment_table_, 1);
  content->addWidget(checklist_frame, 3);

  auto * actions_frame = new QFrame(this);
  actions_frame->setObjectName(QStringLiteral("environmentActions"));
  actions_frame->setMinimumWidth(350);
  auto * actions_layout = new QVBoxLayout(actions_frame);
  actions_layout->setContentsMargins(12, 10, 12, 10);
  actions_layout->setSpacing(9);
  auto * actions_mark = new QLabel(QStringLiteral("INSTALL QUEUE // 安装范围"), actions_frame);
  actions_mark->setObjectName(QStringLiteral("environmentSection"));
  actions_layout->addWidget(actions_mark);
  auto * required_note = new QLabel(QStringLiteral(
    "系统编译依赖与程序构建为必需项；以下可选能力按勾选结果补齐。"), actions_frame);
  required_note->setWordWrap(true);
  actions_layout->addWidget(required_note);
  rosbag_check_ = new QCheckBox(QStringLiteral("ROS Bag 深度解析环境"), actions_frame);
  map_converter_check_ = new QCheckBox(QStringLiteral("MAP BIN 解码器"), actions_frame);
  desktop_check_ = new QCheckBox(QStringLiteral("桌面与应用菜单入口"), actions_frame);
  rosbag_check_->setChecked(true);
  map_converter_check_->setChecked(true);
  desktop_check_->setChecked(true);
  actions_layout->addWidget(rosbag_check_);
  actions_layout->addWidget(map_converter_check_);
  actions_layout->addWidget(desktop_check_);

  auto * security_note = new QLabel(QStringLiteral(
    "需要 APT 权限时才询问 sudo 密码。密码仅写入 sudo 标准输入，"
    "不会保存、显示在日志中或出现在命令行参数里。"), actions_frame);
  security_note->setObjectName(QStringLiteral("environmentSecurity"));
  security_note->setWordWrap(true);
  actions_layout->addWidget(security_note);

  auto * button_row = new QHBoxLayout;
  check_button_ = new QPushButton(QStringLiteral("重新检测"), actions_frame);
  check_button_->setObjectName(QStringLiteral("environmentCheckButton"));
  install_button_ = new QPushButton(QStringLiteral("一键安装所选缺失项"), actions_frame);
  install_button_->setObjectName(QStringLiteral("environmentInstallButton"));
  install_button_->setProperty("role", "primary");
  button_row->addWidget(check_button_);
  button_row->addWidget(install_button_, 1);
  actions_layout->addLayout(button_row);
  progress_bar_ = new QProgressBar(actions_frame);
  progress_bar_->setObjectName(QStringLiteral("environmentProgress"));
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  progress_bar_->setTextVisible(true);
  actions_layout->addWidget(progress_bar_);
  auto * log_mark = new QLabel(QStringLiteral("INSTALL LOG // 安全输出"), actions_frame);
  log_mark->setObjectName(QStringLiteral("environmentSection"));
  actions_layout->addWidget(log_mark);
  log_edit_ = new QTextEdit(actions_frame);
  log_edit_->setObjectName(QStringLiteral("environmentLog"));
  log_edit_->setReadOnly(true);
  log_edit_->document()->setMaximumBlockCount(500);
  log_edit_->setPlaceholderText(QStringLiteral("检测和安装结果会显示在这里。"));
  actions_layout->addWidget(log_edit_, 1);
  content->addWidget(actions_frame, 2);
  outer->addLayout(content, 1);

  connect(check_button_, &QPushButton::clicked, this, &EnvironmentSetupPanel::check_environment);
  connect(install_button_, &QPushButton::clicked,
    this, &EnvironmentSetupPanel::install_missing_environment);
  for (QCheckBox * option : {rosbag_check_, map_converter_check_, desktop_check_}) {
    connect(option, &QCheckBox::toggled, this, [this]() {update_table();});
  }
  process_.setProcessChannelMode(QProcess::MergedChannels);
  connect(&process_, &QProcess::readyRead, this, &EnvironmentSetupPanel::consume_process_output);
  connect(&process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &EnvironmentSetupPanel::process_finished);
  connect(&process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error != QProcess::FailedToStart || current_task_ == Task::None) return;
    append_log(QStringLiteral("无法启动“%1”：%2").arg(task_name(current_task_), process_.errorString()));
    installation_running_ = false;
    install_steps_.clear();
    current_task_ = Task::None;
    check_button_->setEnabled(true);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    update_table();
    set_overall_status(QStringLiteral("ERROR · 无法启动安装任务"), QStringLiteral("error"));
  });

  update_table();
}

void EnvironmentSetupPanel::activate_workspace()
{
  if (!check_complete_ && process_.state() == QProcess::NotRunning) check_environment();
}

QString EnvironmentSetupPanel::project_directory() const
{
  const QString configured = qEnvironmentVariable("PCD_MEASURE_PROJECT_DIR").trimmed();
  if (!configured.isEmpty()) return QFileInfo(configured).absoluteFilePath();
  QDir directory(QCoreApplication::applicationDirPath());
  if (directory.dirName() == QStringLiteral("bin")) {
    directory.cdUp();
    if (directory.dirName() == QStringLiteral("build")) directory.cdUp();
  }
  return directory.absolutePath();
}

QString EnvironmentSetupPanel::desktop_launcher_path() const
{
  QString desktop = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
  if (desktop.isEmpty()) desktop = QDir::home().filePath(QStringLiteral("Desktop"));
  return QDir(desktop).filePath(QStringLiteral("点云测量工具.desktop"));
}

QString EnvironmentSetupPanel::applications_launcher_path() const
{
  QString data = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation);
  if (data.isEmpty()) data = QDir::home().filePath(QStringLiteral(".local/share"));
  return QDir(data).filePath(QStringLiteral("applications/pcd-measure.desktop"));
}

bool EnvironmentSetupPanel::rosbag_environment_ready() const
{
  const QString python = QDir(project_directory()).filePath(
    QStringLiteral(".rosbag-venv/bin/python"));
  if (!QFileInfo(python).isExecutable()) return false;
  QProcess probe;
  probe.start(python, {QStringLiteral("-c"), QStringLiteral("import rosbags, yaml")});
  if (!probe.waitForStarted(1000) || !probe.waitForFinished(2500)) {
    probe.kill();
    probe.waitForFinished(500);
    return false;
  }
  return probe.exitStatus() == QProcess::NormalExit && probe.exitCode() == 0;
}

bool EnvironmentSetupPanel::map_converter_ready() const
{
  return QFileInfo(QDir(project_directory()).filePath(
    QStringLiteral("tools/map_to_ply"))).isExecutable();
}

bool EnvironmentSetupPanel::application_binary_ready() const
{
  return QFileInfo(QDir(project_directory()).filePath(
    QStringLiteral("build/bin/pcd_measure"))).isExecutable();
}

bool EnvironmentSetupPanel::desktop_launchers_ready() const
{
  return QFileInfo(desktop_launcher_path()).isExecutable() &&
    QFileInfo(applications_launcher_path()).isExecutable();
}

bool EnvironmentSetupPanel::ros2_runtime_ready() const
{
  return QFileInfo(QStringLiteral("/opt/ros/humble/setup.bash")).isFile();
}

void EnvironmentSetupPanel::refresh_non_system_state()
{
  build_ready_ = application_binary_ready();
  rosbag_ready_ = rosbag_environment_ready();
  map_converter_ready_ = map_converter_ready();
  desktop_ready_ = desktop_launchers_ready();
  ros2_ready_ = ros2_runtime_ready();
}

void EnvironmentSetupPanel::check_environment()
{
  if (process_.state() != QProcess::NotRunning || installation_running_) return;
  check_complete_ = false;
  check_button_->setEnabled(false);
  install_button_->setEnabled(false);
  progress_bar_->setRange(0, 0);
  set_overall_status(QStringLiteral("CHECKING · 正在检测"), QStringLiteral("checking"));
  append_log(QStringLiteral("开始检查系统包、程序构建和可选组件。"));
  const QString helper = QDir(project_directory()).filePath(
    QStringLiteral("scripts/system_dependencies.sh"));
  start_process(Task::CheckSystem, helper, {QStringLiteral("missing")});
}

void EnvironmentSetupPanel::install_missing_environment()
{
  if (!check_complete_ || process_.state() != QProcess::NotRunning || installation_running_) return;
  install_steps_.clear();
  if (!system_ready_) install_steps_.append(Task::InstallSystem);
  if (!build_ready_) install_steps_.append(Task::BuildApplication);
  if (rosbag_check_->isChecked() && !rosbag_ready_) install_steps_.append(Task::SetupRosbag);
  if (map_converter_check_->isChecked() && !map_converter_ready_) {
    install_steps_.append(Task::SetupMapConverter);
  }
  if (desktop_check_->isChecked() && !desktop_ready_) install_steps_.append(Task::InstallDesktop);
  if (install_steps_.isEmpty()) {
    set_overall_status(QStringLiteral("READY · 所选环境已完整"), QStringLiteral("ready"));
    append_log(QStringLiteral("没有需要安装的项目。"));
    return;
  }

  installation_running_ = true;
  install_step_count_ = install_steps_.size();
  completed_install_steps_ = 0;
  check_button_->setEnabled(false);
  install_button_->setEnabled(false);
  progress_bar_->setRange(0, install_step_count_);
  progress_bar_->setValue(0);
  update_table();
  start_next_install_step();
}

void EnvironmentSetupPanel::consume_process_output()
{
  const QByteArray chunk = process_.readAll();
  process_output_.append(chunk);
  if (current_task_ != Task::CheckSystem) append_log(QString::fromLocal8Bit(chunk));
}

void EnvironmentSetupPanel::process_finished(int exit_code, QProcess::ExitStatus exit_status)
{
  consume_process_output();
  const Task finished_task = current_task_;
  current_task_ = Task::None;
  const bool success = exit_status == QProcess::NormalExit && exit_code == 0;

  if (finished_task == Task::CheckSystem) {
    if (!success) {
      check_complete_ = false;
      progress_bar_->setRange(0, 100);
      progress_bar_->setValue(0);
      check_button_->setEnabled(true);
      set_overall_status(QStringLiteral("ERROR · 系统依赖检测失败"), QStringLiteral("error"));
      append_log(QStringLiteral("系统依赖检查器退出码：%1").arg(exit_code));
      return;
    }
    missing_system_packages_.clear();
    const QRegularExpression package_pattern(QStringLiteral("^[a-z0-9][a-z0-9+.-]+$"));
    const QStringList lines = QString::fromLocal8Bit(process_output_).split(
      QRegularExpression(QStringLiteral("[\\r\\n]+")), Qt::SkipEmptyParts);
    for (const QString & line : lines) {
      const QString package = line.trimmed();
      if (package_pattern.match(package).hasMatch()) missing_system_packages_.append(package);
    }
    system_ready_ = missing_system_packages_.isEmpty();
    refresh_non_system_state();
    check_complete_ = true;
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(100);
    check_button_->setEnabled(true);
    append_log(system_ready_ ? QStringLiteral("系统编译依赖完整。") :
      QStringLiteral("缺少系统包：%1").arg(missing_system_packages_.join(QStringLiteral(", "))));
    update_table();
    return;
  }

  if (!installation_running_) return;
  if (!success) {
    installation_running_ = false;
    install_steps_.clear();
    check_button_->setEnabled(true);
    append_log(QStringLiteral("任务“%1”退出码：%2。修复上方错误后可重新检测。")
      .arg(task_name(finished_task)).arg(exit_code));
    update_table();
    set_overall_status(QStringLiteral("ERROR · %1失败").arg(task_name(finished_task)),
      QStringLiteral("error"));
    return;
  }

  ++completed_install_steps_;
  progress_bar_->setValue(completed_install_steps_);
  append_log(QStringLiteral("%1完成。").arg(task_name(finished_task)));
  start_next_install_step();
}

void EnvironmentSetupPanel::update_table()
{
  if (!environment_table_) return;
  for (QCheckBox * option : {rosbag_check_, map_converter_check_, desktop_check_}) {
    if (option) option->setEnabled(!installation_running_);
  }
  const bool rosbag_selected = rosbag_check_ && rosbag_check_->isChecked();
  const bool map_selected = map_converter_check_ && map_converter_check_->isChecked();
  const bool desktop_selected = desktop_check_ && desktop_check_->isChecked();
  set_row(0, QStringLiteral("系统编译依赖"), system_ready_ ? QStringLiteral("READY") :
    (check_complete_ ? QStringLiteral("MISSING") : QStringLiteral("CHECK")),
    system_ready_ ? QStringLiteral("Qt 5、PCL、VTK、CMake、Python 等固定软件包已安装") :
    (check_complete_ ? QStringLiteral("缺少：%1").arg(
      missing_system_packages_.join(QStringLiteral(", "))) : QStringLiteral("等待检测")),
    system_ready_ ? kReadyColor : (check_complete_ ? kMissingColor : kOptionalColor));
  set_row(1, QStringLiteral("程序构建"), build_ready_ ? QStringLiteral("READY") :
    QStringLiteral("MISSING"), build_ready_ ? QStringLiteral("可执行文件已经生成") :
    QStringLiteral("安装系统包后自动执行 Release 构建"),
    build_ready_ ? kReadyColor : kMissingColor);
  set_row(2, QStringLiteral("ROS Bag 深度解析"), rosbag_ready_ ? QStringLiteral("READY") :
    (rosbag_selected ? QStringLiteral("MISSING") : QStringLiteral("SKIP")),
    rosbag_ready_ ? QStringLiteral("ROS1、MCAP、压缩 Bag 读取器可导入") :
    (rosbag_selected ? QStringLiteral("将在项目内创建独立 Python 环境") :
      QStringLiteral("本次不安装；基础 SQLite 诊断仍可使用")),
    rosbag_ready_ ? kReadyColor : (rosbag_selected ? kMissingColor : kOptionalColor));
  set_row(3, QStringLiteral("MAP BIN 解码器"), map_converter_ready_ ? QStringLiteral("READY") :
    (map_selected ? QStringLiteral("MISSING") : QStringLiteral("SKIP")),
    map_converter_ready_ ? QStringLiteral("官方转换器存在且可执行") :
    (map_selected ? QStringLiteral("将下载并验证固定 SHA-256") :
      QStringLiteral("本次不安装；PCD、PLY、OLX 不受影响")),
    map_converter_ready_ ? kReadyColor : (map_selected ? kMissingColor : kOptionalColor));
  set_row(4, QStringLiteral("桌面启动入口"), desktop_ready_ ? QStringLiteral("READY") :
    (desktop_selected ? QStringLiteral("MISSING") : QStringLiteral("SKIP")),
    desktop_ready_ ? QStringLiteral("桌面与应用菜单入口均可执行") :
    (desktop_selected ? QStringLiteral("将创建本地用户启动图标") :
      QStringLiteral("本次不创建；仍可通过 run.sh 启动")),
    desktop_ready_ ? kReadyColor : (desktop_selected ? kMissingColor : kOptionalColor));
  set_row(5, QStringLiteral("ROS2 Humble"), ros2_ready_ ? QStringLiteral("READY") :
    QStringLiteral("OPTIONAL"), ros2_ready_ ? QStringLiteral("系统 ROS2 环境存在；设备工作空间单独检测") :
    QStringLiteral("不影响点云查看；设备采集需按硬件文档安装"),
    ros2_ready_ ? kReadyColor : kOptionalColor);

  const bool selected_missing = !system_ready_ || !build_ready_ ||
    (rosbag_selected && !rosbag_ready_) ||
    (map_selected && !map_converter_ready_) ||
    (desktop_selected && !desktop_ready_);
  install_button_->setEnabled(check_complete_ && !installation_running_ && selected_missing);
  if (!check_complete_ || installation_running_) return;
  if (!system_ready_ || !build_ready_) {
    set_overall_status(QStringLiteral("ACTION · 缺少核心环境"), QStringLiteral("warning"));
  } else if (selected_missing) {
    set_overall_status(QStringLiteral("READY · 可补齐可选能力"), QStringLiteral("warning"));
  } else if (!ros2_ready_) {
    set_overall_status(QStringLiteral("CORE READY · ROS2 按需安装"), QStringLiteral("ready"));
  } else {
    set_overall_status(QStringLiteral("READY · 环境完整"), QStringLiteral("ready"));
  }
}

void EnvironmentSetupPanel::set_row(int row, const QString & component, const QString & status,
  const QString & detail, const QColor & color)
{
  const QStringList values{component, status, detail};
  for (int column = 0; column < values.size(); ++column) {
    auto * item = new QTableWidgetItem(values[column]);
    item->setForeground(column == 1 ? color : QColor(QStringLiteral("#D6E5E9")));
    if (column == 1) {
      QFont font(QStringLiteral("DejaVu Sans Mono"));
      font.setBold(true);
      item->setFont(font);
      item->setTextAlignment(Qt::AlignCenter);
    }
    environment_table_->setItem(row, column, item);
  }
}

void EnvironmentSetupPanel::set_overall_status(const QString & text, const QString & tone)
{
  if (!overall_status_label_) return;
  QString foreground = QStringLiteral("#79E6E0");
  QString border = QStringLiteral("#2A7E82");
  QString background = QStringLiteral("#0A2932");
  if (tone == QStringLiteral("warning")) {
    foreground = QStringLiteral("#FFCA72");
    border = QStringLiteral("#705A35");
    background = QStringLiteral("#292215");
  } else if (tone == QStringLiteral("error")) {
    foreground = QStringLiteral("#FFB0A7");
    border = QStringLiteral("#75423F");
    background = QStringLiteral("#2B1B1B");
  } else if (tone == QStringLiteral("checking")) {
    foreground = QStringLiteral("#9CC9DB");
    border = QStringLiteral("#31566A");
    background = QStringLiteral("#102735");
  }
  overall_status_label_->setText(text);
  overall_status_label_->setStyleSheet(QStringLiteral(
    "color:%1; border:1px solid %2; background:%3;").arg(foreground, border, background));
}

void EnvironmentSetupPanel::append_log(const QString & text)
{
  const QString cleaned = text.trimmed();
  if (cleaned.isEmpty() || !log_edit_) return;
  log_edit_->append(cleaned.toHtmlEscaped().replace(QLatin1Char('\n'), QStringLiteral("<br>")));
}

bool EnvironmentSetupPanel::start_process(Task task, const QString & program,
  const QStringList & arguments, const QProcessEnvironment & environment)
{
  process_output_.clear();
  current_task_ = task;
  process_.setWorkingDirectory(project_directory());
  process_.setProcessEnvironment(environment);
  process_.start(program, arguments);
  if (process_.waitForStarted(1500)) return true;
  append_log(QStringLiteral("无法启动“%1”：%2").arg(task_name(task), process_.errorString()));
  current_task_ = Task::None;
  installation_running_ = false;
  install_steps_.clear();
  check_button_->setEnabled(true);
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  update_table();
  set_overall_status(QStringLiteral("ERROR · 无法启动任务"), QStringLiteral("error"));
  return false;
}

bool EnvironmentSetupPanel::start_system_install()
{
  const QString helper = QDir(project_directory()).filePath(
    QStringLiteral("scripts/system_dependencies.sh"));
  if (::geteuid() == 0) {
    return start_process(Task::InstallSystem, helper, {QStringLiteral("install")});
  }
  if (!QFileInfo(sudo_program_).isExecutable()) {
    append_log(QStringLiteral("没有找到可执行的 sudo，无法安装系统软件包。"));
    installation_running_ = false;
    install_steps_.clear();
    update_table();
    set_overall_status(QStringLiteral("ERROR · 缺少 sudo"), QStringLiteral("error"));
    return false;
  }

  if (!force_password_prompt_for_test_ && sudo_credentials_cached()) {
    return start_process(Task::InstallSystem, sudo_program_, {
      QStringLiteral("-n"), QStringLiteral("--"), helper, QStringLiteral("install")});
  }

  bool accepted = false;
  QString password = QInputDialog::getText(this, QStringLiteral("需要 sudo 权限"),
    QStringLiteral("将安装以下固定 Ubuntu 软件包：\n%1\n\n"
      "请输入当前用户的 sudo 密码。密码不会保存或写入日志。")
      .arg(missing_system_packages_.join(QStringLiteral(", "))),
    QLineEdit::Password, QString(), &accepted);
  if (!accepted) {
    password.fill(QChar(u'\0'));
    installation_running_ = false;
    install_steps_.clear();
    append_log(QStringLiteral("用户取消了 sudo 认证，未修改系统环境。"));
    update_table();
    set_overall_status(QStringLiteral("CANCELLED · 未执行系统安装"), QStringLiteral("warning"));
    return false;
  }
  if (password.isEmpty()) {
    installation_running_ = false;
    install_steps_.clear();
    append_log(QStringLiteral("sudo 密码为空，未执行系统安装。"));
    update_table();
    set_overall_status(QStringLiteral("CANCELLED · 密码为空"), QStringLiteral("warning"));
    return false;
  }

  QByteArray password_bytes = password.toUtf8();
  password.fill(QChar(u'\0'));
  const bool started = start_process(Task::InstallSystem, sudo_program_, {
    QStringLiteral("-S"), QStringLiteral("-p"), QString(), QStringLiteral("--"),
    helper, QStringLiteral("install")});
  if (started) {
    process_.write(password_bytes);
    process_.write("\n", 1);
    process_.waitForBytesWritten(1500);
    process_.closeWriteChannel();
  }
  std::fill(password_bytes.begin(), password_bytes.end(), '\0');
  password_bytes.clear();
  return started;
}

bool EnvironmentSetupPanel::sudo_credentials_cached() const
{
  QProcess probe;
  probe.start(sudo_program_, {QStringLiteral("-n"), QStringLiteral("true")});
  if (!probe.waitForStarted(1000) || !probe.waitForFinished(1500)) {
    probe.kill();
    probe.waitForFinished(500);
    return false;
  }
  return probe.exitStatus() == QProcess::NormalExit && probe.exitCode() == 0;
}

void EnvironmentSetupPanel::start_next_install_step()
{
  if (!installation_running_) return;
  if (install_steps_.isEmpty()) {
    installation_running_ = false;
    update_table();
    set_overall_status(QStringLiteral("VERIFYING · 安装完成，重新检测"),
      QStringLiteral("checking"));
    check_environment();
    return;
  }

  const Task task = install_steps_.takeFirst();
  set_overall_status(QStringLiteral("INSTALLING · %1").arg(task_name(task)),
    QStringLiteral("checking"));
  append_log(QStringLiteral("开始：%1").arg(task_name(task)));
  const QString project = project_directory();
  switch (task) {
    case Task::InstallSystem:
      start_system_install();
      return;
    case Task::BuildApplication:
      start_process(task, QDir(project).filePath(QStringLiteral("build.sh")), {});
      return;
    case Task::SetupRosbag:
      start_process(task, QDir(project).filePath(
        QStringLiteral("scripts/setup_rosbag_tools.sh")), {});
      return;
    case Task::SetupMapConverter:
      start_process(task, QDir(project).filePath(
        QStringLiteral("scripts/setup_odin_map_tools.sh")), {});
      return;
    case Task::InstallDesktop: {
      QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
      environment.insert(QStringLiteral("PCD_MEASURE_SKIP_SYSTEM_SETUP"), QStringLiteral("1"));
      environment.insert(QStringLiteral("PCD_MEASURE_SKIP_ROSBAG_SETUP"), QStringLiteral("1"));
      environment.insert(QStringLiteral("PCD_MEASURE_SKIP_ODIN_MAP_SETUP"), QStringLiteral("1"));
      start_process(task, QDir(project).filePath(QStringLiteral("install_desktop.sh")), {},
        environment);
      return;
    }
    case Task::CheckSystem:
    case Task::None:
      break;
  }
  installation_running_ = false;
  update_table();
  set_overall_status(QStringLiteral("ERROR · 未知安装任务"), QStringLiteral("error"));
}

QString EnvironmentSetupPanel::task_name(Task task) const
{
  switch (task) {
    case Task::CheckSystem: return QStringLiteral("环境检测");
    case Task::InstallSystem: return QStringLiteral("系统软件包安装");
    case Task::BuildApplication: return QStringLiteral("程序构建");
    case Task::SetupRosbag: return QStringLiteral("ROS Bag 深度解析环境");
    case Task::SetupMapConverter: return QStringLiteral("MAP BIN 解码器");
    case Task::InstallDesktop: return QStringLiteral("桌面启动入口");
    case Task::None: break;
  }
  return QStringLiteral("未知任务");
}
