#include "olx_capture_dialog.h"

#include <algorithm>

#include <QCheckBox>
#include <QCloseEvent>
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
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QStandardPaths>
#include <QTextEdit>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

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

OlxCaptureDialog::OlxCaptureDialog(QWidget * parent)
: QDialog(parent)
{
  setObjectName(QStringLiteral("olxCaptureDialog"));
  setWindowTitle(QStringLiteral("设备采集与 OLX 录制"));
  resize(760, 650);
  setAttribute(Qt::WA_DeleteOnClose, true);

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(18, 16, 18, 16);
  layout->setSpacing(11);
  auto * title = new QLabel(QStringLiteral("DEVICE ▸ ROS2 ▸ OLX ▸ MEASURE"));
  title->setStyleSheet(QStringLiteral(
    "color:#FFCA72; font-family:'DejaVu Sans Mono'; font-size:15px; "
    "font-weight:800; letter-spacing:1px; border-bottom:2px solid #FFB547; padding:7px;"));
  layout->addWidget(title);
  auto * note = new QLabel(QStringLiteral(
    "从 ROS2 点云话题直接录制兼容会话，不修改设备驱动配置。SLAM/RGB、DTOF/强度均会统一写为 XYZRGBA；"
    "录制完成后可自动回到主窗口播放和测量。"));
  note->setWordWrap(true);
  note->setStyleSheet(QStringLiteral("color:#AFC5CD; background:#0B202C; padding:9px; border-left:3px solid #21D4D1;"));
  layout->addWidget(note);

  auto * status_row = new QHBoxLayout;
  usb_status_label_ = new QLabel(QStringLiteral("USB · 检查中"));
  topic_status_label_ = new QLabel(QStringLiteral("ROS2 · 检查中"));
  usb_status_label_->setObjectName(QStringLiteral("olxUsbStatus"));
  topic_status_label_->setObjectName(QStringLiteral("olxTopicStatus"));
  usb_status_label_->setStyleSheet(status_style(false));
  topic_status_label_->setStyleSheet(status_style(false));
  status_row->addWidget(usb_status_label_);
  status_row->addWidget(topic_status_label_);
  status_row->addStretch(1);
  auto * refresh_button = new QPushButton(QStringLiteral("刷新状态"));
  connect(refresh_button, &QPushButton::clicked, this, &OlxCaptureDialog::refresh_status);
  status_row->addWidget(refresh_button);
  layout->addLayout(status_row);

  auto * form = new QFormLayout;
  workspace_edit_ = new QLineEdit;
  workspace_edit_->setObjectName(QStringLiteral("olxWorkspace"));
  QString default_workspace = QDir::home().filePath(QStringLiteral("Desktop/odin1_mapping_ws"));
  workspace_edit_->setText(QSettings().value(
    QStringLiteral("olxCapture/workspace"), default_workspace).toString());
  auto * workspace_row = new QHBoxLayout;
  workspace_row->addWidget(workspace_edit_, 1);
  auto * workspace_button = new QPushButton(QStringLiteral("选择…"));
  connect(workspace_button, &QPushButton::clicked, this, &OlxCaptureDialog::browse_workspace);
  workspace_row->addWidget(workspace_button);
  form->addRow(QStringLiteral("ROS2 工作空间："), workspace_row);

  output_edit_ = new QLineEdit;
  output_edit_->setObjectName(QStringLiteral("olxOutput"));
  QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
  if (documents.isEmpty()) documents = QDir::homePath();
  output_edit_->setText(QSettings().value(
    QStringLiteral("olxCapture/output"), QDir(documents).filePath(QStringLiteral("点云录制"))).toString());
  auto * output_row = new QHBoxLayout;
  output_row->addWidget(output_edit_, 1);
  auto * output_button = new QPushButton(QStringLiteral("选择…"));
  connect(output_button, &QPushButton::clicked, this, &OlxCaptureDialog::browse_output);
  output_row->addWidget(output_button);
  form->addRow(QStringLiteral("保存目录："), output_row);

  scan_mode_combo_ = new QComboBox;
  scan_mode_combo_->setObjectName(QStringLiteral("olxScanMode"));
  scan_mode_combo_->addItem(QStringLiteral("SLAM 彩色点云"), QStringLiteral("/odin1/cloud_slam"));
  scan_mode_combo_->addItem(QStringLiteral("渲染彩色点云"), QStringLiteral("/odin1/cloud_render"));
  scan_mode_combo_->addItem(QStringLiteral("DTOF 原始强度点云"), QStringLiteral("/odin1/cloud_raw"));
  form->addRow(QStringLiteral("扫描模式："), scan_mode_combo_);
  pose_topic_edit_ = new QLineEdit(QStringLiteral("/odin1/odometry"));
  image_topic_edit_ = new QLineEdit(QStringLiteral("/odin1/image/compressed"));
  pose_topic_edit_->setObjectName(QStringLiteral("olxPoseTopic"));
  image_topic_edit_->setObjectName(QStringLiteral("olxImageTopic"));
  form->addRow(QStringLiteral("位姿话题："), pose_topic_edit_);
  form->addRow(QStringLiteral("相机话题："), image_topic_edit_);
  layout->addLayout(form);

  auto * options = new QHBoxLayout;
  pose_check_ = new QCheckBox(QStringLiteral("记录位姿"));
  pose_check_->setChecked(true);
  image_check_ = new QCheckBox(QStringLiteral("记录相机（话题存在时）"));
  image_check_->setChecked(true);
  auto_open_check_ = new QCheckBox(QStringLiteral("完成后自动打开"));
  auto_open_check_->setChecked(true);
  options->addWidget(pose_check_);
  options->addWidget(image_check_);
  options->addWidget(auto_open_check_);
  options->addStretch(1);
  layout->addLayout(options);

  auto * actions = new QHBoxLayout;
  driver_button_ = new QPushButton(QStringLiteral("启动设备驱动"));
  record_button_ = new QPushButton(QStringLiteral("开始录制"));
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
  actions->addWidget(driver_button_);
  actions->addWidget(record_button_);
  actions->addWidget(stop_button_);
  actions->addWidget(open_button_);
  layout->addLayout(actions);

  recording_status_label_ = new QLabel(QStringLiteral("● IDLE · 等待录制"));
  recording_status_label_->setObjectName(QStringLiteral("olxRecordingStatus"));
  recording_status_label_->setStyleSheet(QStringLiteral(
    "color:#7CE4E1; background:#081924; border:1px solid #29485C; padding:8px; "
    "font-family:'DejaVu Sans Mono'; font-weight:700;"));
  counter_label_ = new QLabel(QStringLiteral("帧 0 · 点 0 · 位姿 0 · 图像 0"));
  counter_label_->setObjectName(QStringLiteral("olxCounters"));
  counter_label_->setStyleSheet(QStringLiteral("color:#DCE9ED; font-family:'DejaVu Sans Mono';"));
  layout->addWidget(recording_status_label_);
  layout->addWidget(counter_label_);

  log_edit_ = new QTextEdit;
  log_edit_->setReadOnly(true);
  log_edit_->document()->setMaximumBlockCount(350);
  log_edit_->setPlaceholderText(QStringLiteral("设备驱动与录制日志会显示在这里。"));
  layout->addWidget(log_edit_, 1);

  auto * close_buttons = new QDialogButtonBox(QDialogButtonBox::Close);
  close_buttons->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
  connect(close_buttons, &QDialogButtonBox::rejected, this, &QDialog::close);
  layout->addWidget(close_buttons);

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
  connect(scan_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {refresh_status();});
  connect(workspace_edit_, &QLineEdit::editingFinished, this, &OlxCaptureDialog::refresh_status);
  update_recording_controls();
  refresh_status();
}

QString OlxCaptureDialog::project_directory() const
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

QString OlxCaptureDialog::selected_cloud_topic() const
{
  return scan_mode_combo_->currentData().toString();
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
  const QString calibration = QDir(workspace).filePath(QStringLiteral("runtime/calib.yaml"));
  if (QFileInfo(calibration).isFile()) arguments << QStringLiteral("--calibration") << calibration;
  if (!pose_check_->isChecked()) arguments << QStringLiteral("--no-pose");
  if (!image_check_->isChecked()) arguments << QStringLiteral("--no-image");
  recorder_output_buffer_.clear();
  last_session_path_.clear();
  last_olx_path_.clear();
  recorder_process_.setWorkingDirectory(project_directory());
  recorder_process_.start(QStringLiteral("/bin/bash"), arguments);
  if (!recorder_process_.waitForStarted(3000)) {
    QMessageBox::critical(this, QStringLiteral("无法启动录制器"), recorder_process_.errorString());
    update_recording_controls();
    return;
  }
  recording_status_label_->setText(QStringLiteral("● RECORDING · %1").arg(selected_cloud_topic()));
  recording_status_label_->setStyleSheet(QStringLiteral(
    "color:#FFD38A; background:#302817; border:1px solid #8A6933; padding:8px; "
    "font-family:'DejaVu Sans Mono'; font-weight:700;"));
  append_log(QStringLiteral("开始录制会话 %1").arg(session));
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
      counter_label_->setText(QStringLiteral("帧 %1 · 点 %2 · 位姿 %3 · 图像 %4 · %5 s")
        .arg(status.value(QStringLiteral("cloud_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("points")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("pose_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("image_frames")).toVariant().toULongLong())
        .arg(status.value(QStringLiteral("elapsed_seconds")).toDouble(), 0, 'f', 1));
      continue;
    }
    if (!line.isEmpty()) append_log(QString::fromLocal8Bit(line));
  }
}

void OlxCaptureDialog::recorder_finished(int exit_code, QProcess::ExitStatus status)
{
  consume_recorder_output();
  const bool complete = status == QProcess::NormalExit && exit_code == 0 &&
    QFileInfo(last_olx_path_).isFile() && QFileInfo(last_olx_path_).size() > 16;
  recording_status_label_->setText(complete ? QStringLiteral("● COMPLETE · OLX 已安全完成") :
    QStringLiteral("● STOPPED · 未收到有效点云"));
  recording_status_label_->setStyleSheet(status_style(complete));
  open_button_->setEnabled(complete);
  append_log(complete ? QStringLiteral("录制完成：%1").arg(last_olx_path_) :
    QStringLiteral("录制结束（退出码 %1）；请检查点云话题和日志。").arg(exit_code));
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
  pose_topic_edit_->setEnabled(!recording);
  image_topic_edit_->setEnabled(!recording);
  pose_check_->setEnabled(!recording);
  image_check_->setEnabled(!recording);
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
  event->accept();
}
