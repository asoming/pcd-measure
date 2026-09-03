#include "rosbag_dialog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QMessageBox>
#include <QMimeData>
#include <QProcessEnvironment>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollBar>
#include <QSettings>
#include <QSpinBox>
#include <QSplitter>
#include <QStandardPaths>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

#include <unistd.h>

namespace
{

QString count_text(qint64 value)
{
  return QLocale(QLocale::English).toString(value);
}

QString json_number(const QJsonValue & value, int precision, const QString & suffix = QString())
{
  if (!value.isDouble()) return QStringLiteral("—");
  const double number = value.toDouble();
  if (!std::isfinite(number)) return QStringLiteral("—");
  if (precision == 0) {
    return count_text(static_cast<qint64>(std::llround(number))) + suffix;
  }
  return QStringLiteral("%1%2").arg(number, 0, 'f', precision).arg(suffix);
}

QString severity_text(const QString & severity)
{
  if (severity == QStringLiteral("critical")) return QStringLiteral("严重");
  if (severity == QStringLiteral("warning")) return QStringLiteral("警告");
  if (severity == QStringLiteral("notice")) return QStringLiteral("提示");
  return QStringLiteral("正常");
}

QString severity_color(const QString & severity)
{
  if (severity == QStringLiteral("critical")) return QStringLiteral("#FF716B");
  if (severity == QStringLiteral("warning")) return QStringLiteral("#FFB85C");
  if (severity == QStringLiteral("notice")) return QStringLiteral("#65AEE8");
  return QStringLiteral("#36D6D0");
}

int severity_rank(const QString & severity)
{
  if (severity == QStringLiteral("critical")) return 3;
  if (severity == QStringLiteral("warning")) return 2;
  if (severity == QStringLiteral("notice")) return 1;
  return 0;
}

QFrame * make_metric_card(
  const QString & title,
  const QString & initial_value,
  QLabel ** value_label,
  QWidget * parent)
{
  auto * card = new QFrame(parent);
  card->setObjectName(QStringLiteral("metricCard"));
  auto * layout = new QVBoxLayout(card);
  layout->setContentsMargins(12, 9, 12, 10);
  layout->setSpacing(5);
  auto * title_label = new QLabel(title, card);
  title_label->setObjectName(QStringLiteral("metricTitle"));
  auto * metric = new QLabel(initial_value, card);
  metric->setObjectName(QStringLiteral("metricValue"));
  metric->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(title_label);
  layout->addWidget(metric);
  *value_label = metric;
  return card;
}

QTableWidget * make_table(const QStringList & headers, QWidget * parent)
{
  auto * table = new QTableWidget(0, headers.size(), parent);
  table->setHorizontalHeaderLabels(headers);
  table->setAlternatingRowColors(true);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->verticalHeader()->setVisible(false);
  table->horizontalHeader()->setStretchLastSection(true);
  table->setSortingEnabled(true);
  return table;
}

void set_item(
  QTableWidget * table,
  int row,
  int column,
  const QString & text,
  const QString & tooltip = QString(),
  Qt::Alignment alignment = Qt::AlignLeft | Qt::AlignVCenter)
{
  auto * item = new QTableWidgetItem(text);
  item->setTextAlignment(alignment);
  if (!tooltip.isEmpty()) item->setToolTip(tooltip);
  table->setItem(row, column, item);
}

void color_severity_item(QTableWidgetItem * item, const QString & severity)
{
  if (!item) return;
  item->setForeground(QColor(severity_color(severity)));
  QFont font = item->font();
  font.setBold(true);
  item->setFont(font);
}

QString preferred_report_directory(const QString & bag_path)
{
  const QFileInfo info(bag_path);
  if (info.exists()) return info.isDir() ? info.absoluteFilePath() : info.absolutePath();
  return QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
}

}

RosbagDiagnosticDialog::RosbagDiagnosticDialog(
  const QString & initial_path,
  QWidget * parent)
: QDialog(parent)
{
  setObjectName(QStringLiteral("rosbagDiagnosticDialog"));
  setAttribute(Qt::WA_DeleteOnClose, false);
  setAcceptDrops(true);
  setWindowTitle(QStringLiteral("ROS Bag 回放与离线诊断"));
  setMinimumSize(1060, 700);
  resize(1440, 880);
  build_interface();

  connect(&diagnostic_process_, &QProcess::readyReadStandardOutput,
    this, &RosbagDiagnosticDialog::read_diagnostic_output);
  connect(&diagnostic_process_, &QProcess::readyReadStandardError,
    this, &RosbagDiagnosticDialog::read_diagnostic_errors);
  connect(&diagnostic_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &RosbagDiagnosticDialog::diagnostic_finished);
  connect(&diagnostic_process_, &QProcess::errorOccurred, this,
    [this](QProcess::ProcessError) {
      if (diagnostic_process_.state() == QProcess::NotRunning && current_report_.isEmpty()) {
        append_log(QStringLiteral("诊断进程错误：%1").arg(diagnostic_process_.errorString()),
          QStringLiteral("ERR"));
        diagnostic_progress_->setFormat(QStringLiteral("诊断进程启动失败"));
        diagnostic_status_label_->setText(QStringLiteral("● FAILED"));
        diagnostic_status_label_->setStyleSheet(
          QStringLiteral("color:#FF716B; font:700 11px 'DejaVu Sans Mono';"));
        set_busy(false);
      }
    });

  connect(&playback_process_, &QProcess::readyReadStandardOutput,
    this, &RosbagDiagnosticDialog::read_playback_output);
  connect(&playback_process_, &QProcess::readyReadStandardError,
    this, &RosbagDiagnosticDialog::read_playback_output);
  connect(&playback_process_, &QProcess::started,
    this, &RosbagDiagnosticDialog::playback_started);
  connect(&playback_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &RosbagDiagnosticDialog::playback_finished);
  connect(&playback_process_, &QProcess::errorOccurred, this,
    [this](QProcess::ProcessError) {
      append_log(QStringLiteral("回放进程错误：%1").arg(playback_process_.errorString()),
        QStringLiteral("PLAY"));
      update_playback_controls();
    });

  if (!initial_path.isEmpty()) {
    set_bag_path(initial_path);
  } else {
    update_input_state();
  }
}

RosbagDiagnosticDialog::~RosbagDiagnosticDialog()
{
  if (diagnostic_process_.state() != QProcess::NotRunning) {
    diagnostic_process_.kill();
    diagnostic_process_.waitForFinished(1000);
  }
  if (playback_process_.state() != QProcess::NotRunning) {
    if (playback_paused_) {
      ::kill(static_cast<pid_t>(playback_process_.processId()), SIGCONT);
    }
    playback_process_.kill();
    playback_process_.waitForFinished(1000);
  }
}

void RosbagDiagnosticDialog::build_interface()
{
  setStyleSheet(QStringLiteral(R"(
    QDialog#rosbagDiagnosticDialog { background: #06121B; color: #DCE9ED; }
    QDialog#rosbagDiagnosticDialog QLabel { color: #BFD1D8; }
    QFrame#diagnosticHero { background: #0B202D; border: 1px solid #2A4C5E; border-top: 3px solid #36D6D0; border-radius: 8px; }
    QLabel#consoleMark { color: #65E2DE; font: 700 10px 'DejaVu Sans Mono'; letter-spacing: 1.5px; }
    QLabel#diagnosticTitle { color: #F1FAFC; font-size: 24px; font-weight: 700; }
    QLabel#diagnosticSubtitle { color: #8FAAB6; }
    QLabel#bagKind { color: #9ED8EC; font: 700 11px 'DejaVu Sans Mono'; border: 1px solid #31566A; border-radius: 5px; padding: 5px 8px; background: #102A38; }
    QLabel#diagnosticStatus { color: #8FAAB6; font: 700 11px 'DejaVu Sans Mono'; }
    QFrame#sourcePanel, QFrame#playbackPanel, QFrame#rosEnvironmentPanel { background: #0B1E2A; border: 1px solid #294A5B; border-radius: 7px; }
    QDialog#rosbagDiagnosticDialog QLineEdit,
    QDialog#rosbagDiagnosticDialog QDoubleSpinBox,
    QDialog#rosbagDiagnosticDialog QSpinBox {
      background: #071721; color: #E8F4F6; border: 1px solid #31566A;
      border-radius: 5px; padding: 5px 7px; selection-background-color: #147F83;
    }
    QDialog#rosbagDiagnosticDialog QLineEdit:focus,
    QDialog#rosbagDiagnosticDialog QDoubleSpinBox:focus,
    QDialog#rosbagDiagnosticDialog QSpinBox:focus { border-color: #36D6D0; }
    QDialog#rosbagDiagnosticDialog QPushButton {
      background: #122C39; color: #DDECEF; border: 1px solid #36596A;
      border-radius: 5px; padding: 6px 12px; min-height: 18px; font-weight: 600;
    }
    QDialog#rosbagDiagnosticDialog QPushButton:hover { background: #193A49; border-color: #4C7A8E; }
    QDialog#rosbagDiagnosticDialog QPushButton:pressed { background: #0A202B; }
    QDialog#rosbagDiagnosticDialog QPushButton:disabled { color: #607985; background: #0B1C25; border-color: #223C48; }
    QDialog#rosbagDiagnosticDialog QPushButton[role="primary"] {
      background: #147F83; color: #F4FFFF; border-color: #36D6D0;
    }
    QDialog#rosbagDiagnosticDialog QPushButton[role="primary"]:hover { background: #19979A; }
    QDialog#rosbagDiagnosticDialog QPushButton[role="danger"] { color: #FF9B96; border-color: #7D4548; }
    QDialog#rosbagDiagnosticDialog QCheckBox { color: #BFD1D8; spacing: 6px; }
    QDialog#rosbagDiagnosticDialog QCheckBox::indicator {
      width: 14px; height: 14px; border: 1px solid #426879; border-radius: 3px; background: #071721;
    }
    QDialog#rosbagDiagnosticDialog QCheckBox::indicator:checked { background: #36D6D0; border-color: #79F2ED; }
    QFrame#metricCard { background: #0D2431; border: 1px solid #294A5B; border-radius: 7px; }
    QLabel#metricTitle { color: #809EAB; font-size: 11px; }
    QLabel#metricValue, QLabel#rosbagScore, QLabel#rosbagDuration, QLabel#rosbagMessages,
    QLabel#rosbagTopics, QLabel#rosbagAlerts, QLabel#rosbagCoverage {
      color: #E9F5F7; font: 700 18px 'DejaVu Sans Mono';
    }
    QFrame#healthRailSegment { background: #294A5B; border: none; border-radius: 2px; min-height: 4px; max-height: 4px; }
    QLabel#railLabel { color: #7F9BA8; font: 700 10px 'DejaVu Sans Mono'; }
    QTabWidget::pane { border: 1px solid #294A5B; background: #091923; border-radius: 6px; top: -1px; }
    QTabBar::tab { background: #102735; color: #8FAAB6; border: 1px solid #294A5B; padding: 8px 16px; margin-right: 3px; border-top-left-radius: 5px; border-top-right-radius: 5px; }
    QTabBar::tab:selected { background: #153747; color: #E9FFFF; border-bottom-color: #153747; border-top: 2px solid #36D6D0; }
    QDialog#rosbagDiagnosticDialog QGroupBox {
      background: #0B1E2A; color: #8FAAB6; border: 1px solid #294A5B;
      border-radius: 6px; margin-top: 8px; padding-top: 6px;
    }
    QDialog#rosbagDiagnosticDialog QGroupBox::title {
      subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #89B8C8;
    }
    QDialog#rosbagDiagnosticDialog QTableWidget,
    QDialog#rosbagDiagnosticDialog QTextEdit {
      background: #071721; alternate-background-color: #0A202B; color: #D9E8EC;
      border: 1px solid #294A5B; gridline-color: #1C3947;
      selection-background-color: #164D60; selection-color: #F4FFFF;
    }
    QDialog#rosbagDiagnosticDialog QTableWidget::item { padding: 4px; }
    QDialog#rosbagDiagnosticDialog QHeaderView::section {
      background: #102A38; color: #A9C3CD; border: none;
      border-right: 1px solid #294A5B; border-bottom: 1px solid #36596A;
      padding: 6px; font-weight: 700;
    }
    QDialog#rosbagDiagnosticDialog QTextEdit { font-family: 'DejaVu Sans Mono'; }
    QDialog#rosbagDiagnosticDialog QProgressBar {
      background: #081B25; color: #E8FFFF; border: 1px solid #31566A;
      border-radius: 4px; text-align: center; min-height: 22px;
    }
    QDialog#rosbagDiagnosticDialog QProgressBar::chunk { background: #168E91; border-radius: 3px; }
  )"));

  auto * outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(14, 14, 14, 12);
  outer_layout->setSpacing(9);

  auto * hero = new QFrame(this);
  hero->setObjectName(QStringLiteral("diagnosticHero"));
  auto * hero_layout = new QHBoxLayout(hero);
  hero_layout->setContentsMargins(18, 13, 18, 13);
  auto * hero_text = new QVBoxLayout;
  hero_text->setSpacing(2);
  auto * console_mark = new QLabel(QStringLiteral("ROS BAG // SIGNAL INTEGRITY CONSOLE"), hero);
  console_mark->setObjectName(QStringLiteral("consoleMark"));
  auto * title = new QLabel(QStringLiteral("回放与离线诊断"), hero);
  title->setObjectName(QStringLiteral("diagnosticTitle"));
  auto * subtitle = new QLabel(
    QStringLiteral("自动检查时序、TF、QoS、传感器与硬件故障模式；原始 bag 保持只读。"), hero);
  subtitle->setObjectName(QStringLiteral("diagnosticSubtitle"));
  hero_text->addWidget(console_mark);
  hero_text->addWidget(title);
  hero_text->addWidget(subtitle);
  hero_layout->addLayout(hero_text, 1);
  auto * hero_status = new QVBoxLayout;
  hero_status->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  bag_kind_label_ = new QLabel(QStringLiteral("未识别"), hero);
  bag_kind_label_->setObjectName(QStringLiteral("bagKind"));
  diagnostic_status_label_ = new QLabel(QStringLiteral("● WAITING"), hero);
  diagnostic_status_label_->setObjectName(QStringLiteral("diagnosticStatus"));
  diagnostic_status_label_->setAlignment(Qt::AlignRight);
  hero_status->addWidget(bag_kind_label_, 0, Qt::AlignRight);
  hero_status->addWidget(diagnostic_status_label_);
  hero_layout->addLayout(hero_status);
  outer_layout->addWidget(hero);

  auto * source_panel = new QFrame(this);
  source_panel->setObjectName(QStringLiteral("sourcePanel"));
  auto * source_layout = new QHBoxLayout(source_panel);
  source_layout->setContentsMargins(10, 9, 10, 9);
  bag_path_edit_ = new QLineEdit(source_panel);
  bag_path_edit_->setObjectName(QStringLiteral("rosbagPath"));
  bag_path_edit_->setPlaceholderText(
    QStringLiteral("选择或拖入 ROS1 .bag、ROS2 bag 目录、.db3 或 .mcap"));
  bag_path_edit_->setClearButtonEnabled(true);
  choose_file_button_ = new QPushButton(QStringLiteral("选择 bag 文件"), source_panel);
  choose_file_button_->setObjectName(QStringLiteral("chooseRosbagFileButton"));
  choose_directory_button_ = new QPushButton(QStringLiteral("选择 ROS2 目录"), source_panel);
  choose_directory_button_->setObjectName(QStringLiteral("chooseRosbagDirectoryButton"));
  diagnose_button_ = new QPushButton(QStringLiteral("自动诊断"), source_panel);
  diagnose_button_->setObjectName(QStringLiteral("diagnoseRosbagButton"));
  diagnose_button_->setProperty("role", "primary");
  source_layout->addWidget(bag_path_edit_, 1);
  source_layout->addWidget(choose_file_button_);
  source_layout->addWidget(choose_directory_button_);
  source_layout->addWidget(diagnose_button_);
  outer_layout->addWidget(source_panel);
  connect(choose_file_button_, &QPushButton::clicked, this,
    &RosbagDiagnosticDialog::choose_bag_file);
  connect(choose_directory_button_, &QPushButton::clicked, this,
    &RosbagDiagnosticDialog::choose_bag_directory);
  connect(diagnose_button_, &QPushButton::clicked, this,
    &RosbagDiagnosticDialog::start_diagnosis);
  connect(bag_path_edit_, &QLineEdit::textChanged, this,
    [this](const QString &) { update_input_state(); });

  auto * environment_panel = new QFrame(this);
  environment_panel->setObjectName(QStringLiteral("rosEnvironmentPanel"));
  auto * environment_layout = new QHBoxLayout(environment_panel);
  environment_layout->setContentsMargins(10, 7, 10, 7);
  environment_layout->addWidget(new QLabel(QStringLiteral("ROS 工作空间"), environment_panel));
  ros_setup_edit_ = new QLineEdit(environment_panel);
  ros_setup_edit_->setObjectName(QStringLiteral("rosSetupPath"));
  ros_setup_edit_->setPlaceholderText(
    QStringLiteral("可选：选择 install/setup.bash，用于自定义消息诊断与回放"));
  ros_setup_edit_->setText(QSettings().value(QStringLiteral("rosbagSetupFile")).toString());
  ros_setup_edit_->setClearButtonEnabled(true);
  choose_setup_button_ = new QPushButton(QStringLiteral("选择 setup.bash"), environment_panel);
  choose_setup_button_->setObjectName(QStringLiteral("chooseRosSetupButton"));
  environment_layout->addWidget(ros_setup_edit_, 1);
  environment_layout->addWidget(choose_setup_button_);
  outer_layout->addWidget(environment_panel);
  connect(choose_setup_button_, &QPushButton::clicked,
    this, &RosbagDiagnosticDialog::choose_ros_setup);
  connect(ros_setup_edit_, &QLineEdit::editingFinished, this, [this]() {
    const QString path = ros_setup_edit_->text().trimmed();
    if (path.isEmpty() || QFileInfo(path).isFile()) {
      QSettings().setValue(QStringLiteral("rosbagSetupFile"), path);
    }
  });

  auto * playback_panel = new QFrame(this);
  playback_panel->setObjectName(QStringLiteral("playbackPanel"));
  auto * playback_layout = new QHBoxLayout(playback_panel);
  playback_layout->setContentsMargins(10, 7, 10, 7);
  playback_layout->addWidget(new QLabel(QStringLiteral("回放倍率"), playback_panel));
  playback_rate_spin_ = new QDoubleSpinBox(playback_panel);
  playback_rate_spin_->setObjectName(QStringLiteral("rosbagPlaybackRate"));
  playback_rate_spin_->setRange(0.05, 20.0);
  playback_rate_spin_->setSingleStep(0.25);
  playback_rate_spin_->setValue(1.0);
  playback_rate_spin_->setSuffix(QStringLiteral(" ×"));
  playback_loop_check_ = new QCheckBox(QStringLiteral("循环"), playback_panel);
  playback_clock_check_ = new QCheckBox(QStringLiteral("发布 /clock"), playback_panel);
  playback_clock_check_->setChecked(true);
  play_button_ = new QPushButton(QStringLiteral("开始回放"), playback_panel);
  play_button_->setObjectName(QStringLiteral("rosbagPlayButton"));
  play_button_->setProperty("role", "primary");
  pause_button_ = new QPushButton(QStringLiteral("暂停"), playback_panel);
  pause_button_->setObjectName(QStringLiteral("rosbagPauseButton"));
  stop_button_ = new QPushButton(QStringLiteral("停止"), playback_panel);
  stop_button_->setObjectName(QStringLiteral("rosbagStopButton"));
  stop_button_->setProperty("role", "danger");
  playback_layout->addWidget(playback_rate_spin_);
  playback_layout->addWidget(playback_loop_check_);
  playback_layout->addWidget(playback_clock_check_);
  playback_layout->addSpacing(8);
  playback_layout->addWidget(play_button_);
  playback_layout->addWidget(pause_button_);
  playback_layout->addWidget(stop_button_);
  playback_layout->addStretch(1);
  auto * read_only_note = new QLabel(QStringLiteral("回放发布到当前 ROS 图；诊断不会启动 ROS 节点"), playback_panel);
  read_only_note->setStyleSheet(QStringLiteral("color:#7896A4;"));
  playback_layout->addWidget(read_only_note);
  outer_layout->addWidget(playback_panel);
  connect(play_button_, &QPushButton::clicked, this, &RosbagDiagnosticDialog::start_playback);
  connect(pause_button_, &QPushButton::clicked, this,
    &RosbagDiagnosticDialog::pause_or_resume_playback);
  connect(stop_button_, &QPushButton::clicked, this, &RosbagDiagnosticDialog::stop_playback);

  auto * rail_layout = new QGridLayout;
  rail_layout->setContentsMargins(2, 1, 2, 0);
  rail_layout->setHorizontalSpacing(7);
  const std::array<QString, 4> rail_titles{
    QStringLiteral("TIMING"), QStringLiteral("TF TREE"),
    QStringLiteral("SENSORS"), QStringLiteral("QOS")};
  std::array<QFrame **, 4> rail_segments{
    &timing_segment_, &tf_segment_, &sensor_segment_, &qos_segment_};
  for (int column = 0; column < 4; ++column) {
    auto * label = new QLabel(rail_titles[static_cast<std::size_t>(column)], this);
    label->setObjectName(QStringLiteral("railLabel"));
    *rail_segments[static_cast<std::size_t>(column)] = new QFrame(this);
    (*rail_segments[static_cast<std::size_t>(column)])->setObjectName(
      QStringLiteral("healthRailSegment"));
    rail_layout->addWidget(label, 0, column);
    rail_layout->addWidget(*rail_segments[static_cast<std::size_t>(column)], 1, column);
  }
  outer_layout->addLayout(rail_layout);

  auto * metric_layout = new QHBoxLayout;
  metric_layout->setSpacing(7);
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("健康评分"), QStringLiteral("—"), &score_label_, this));
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("录制时长"), QStringLiteral("—"), &duration_label_, this));
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("消息总数"), QStringLiteral("—"), &messages_label_, this));
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("话题数量"), QStringLiteral("—"), &topics_label_, this));
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("严重 / 警告"), QStringLiteral("—"), &alerts_label_, this));
  metric_layout->addWidget(make_metric_card(
    QStringLiteral("字段解析覆盖"), QStringLiteral("—"), &coverage_label_, this));
  score_label_->setObjectName(QStringLiteral("rosbagScore"));
  duration_label_->setObjectName(QStringLiteral("rosbagDuration"));
  messages_label_->setObjectName(QStringLiteral("rosbagMessages"));
  topics_label_->setObjectName(QStringLiteral("rosbagTopics"));
  alerts_label_->setObjectName(QStringLiteral("rosbagAlerts"));
  coverage_label_->setObjectName(QStringLiteral("rosbagCoverage"));
  outer_layout->addLayout(metric_layout);

  auto * tabs = new QTabWidget(this);
  tabs->setObjectName(QStringLiteral("rosbagResultTabs"));
  auto * overview_tab = new QWidget(tabs);
  auto * overview_layout = new QHBoxLayout(overview_tab);
  overview_layout->setContentsMargins(7, 7, 7, 7);
  auto * overview_splitter = new QSplitter(Qt::Horizontal, overview_tab);
  issue_table_ = make_table({
    QStringLiteral("级别"), QStringLiteral("分类"), QStringLiteral("话题"),
    QStringLiteral("问题"), QStringLiteral("证据"), QStringLiteral("修复建议")},
    overview_splitter);
  issue_table_->setObjectName(QStringLiteral("rosbagIssueTable"));
  issue_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  issue_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  issue_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  issue_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  issue_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
  issue_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  recommendations_text_ = new QTextEdit(overview_splitter);
  recommendations_text_->setObjectName(QStringLiteral("rosbagRecommendations"));
  recommendations_text_->setReadOnly(true);
  recommendations_text_->setPlaceholderText(QStringLiteral("诊断完成后显示处理顺序和结论边界。"));
  overview_splitter->addWidget(issue_table_);
  overview_splitter->addWidget(recommendations_text_);
  overview_splitter->setStretchFactor(0, 4);
  overview_splitter->setStretchFactor(1, 2);
  overview_layout->addWidget(overview_splitter);
  tabs->addTab(overview_tab, QStringLiteral("诊断总览"));

  topic_table_ = make_table({
    QStringLiteral("话题"), QStringLiteral("类型"), QStringLiteral("消息"),
    QStringLiteral("平均 Hz"), QStringLiteral("中位周期"), QStringLiteral("抖动 CV"),
    QStringLiteral("最大间隙"), QStringLiteral("估算缺失"), QStringLiteral("时基"),
    QStringLiteral("QoS"), QStringLiteral("状态")}, tabs);
  topic_table_->setObjectName(QStringLiteral("rosbagTopicTable"));
  topic_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  for (int column = 2; column < topic_table_->columnCount(); ++column) {
    topic_table_->horizontalHeader()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
  }
  tabs->addTab(topic_table_, QStringLiteral("话题时序 / QoS"));

  auto * tf_sensor_tab = new QWidget(tabs);
  auto * tf_sensor_layout = new QVBoxLayout(tf_sensor_tab);
  tf_sensor_layout->setContentsMargins(7, 7, 7, 7);
  auto * tf_sensor_splitter = new QSplitter(Qt::Vertical, tf_sensor_tab);
  tf_table_ = make_table({
    QStringLiteral("父 → 子"), QStringLiteral("类别"), QStringLiteral("样本"),
    QStringLiteral("净位移"), QStringLiteral("最大平移步长"), QStringLiteral("最大旋转步长"),
    QStringLiteral("最大间隙"), QStringLiteral("最大速度")}, tf_sensor_splitter);
  tf_table_->setObjectName(QStringLiteral("rosbagTfTable"));
  sensor_table_ = make_table({
    QStringLiteral("话题"), QStringLiteral("类型"), QStringLiteral("角色"),
    QStringLiteral("抽样"), QStringLiteral("无效"), QStringLiteral("空帧"),
    QStringLiteral("结构异常"), QStringLiteral("饱和"), QStringLiteral("最长重复"),
    QStringLiteral("状态")}, tf_sensor_splitter);
  sensor_table_->setObjectName(QStringLiteral("rosbagSensorTable"));
  tf_sensor_splitter->addWidget(tf_table_);
  tf_sensor_splitter->addWidget(sensor_table_);
  tf_sensor_splitter->setSizes({260, 260});
  tf_sensor_layout->addWidget(tf_sensor_splitter);
  tabs->addTab(tf_sensor_tab, QStringLiteral("TF / 传感器"));

  log_text_ = new QTextEdit(tabs);
  log_text_->setObjectName(QStringLiteral("rosbagLog"));
  log_text_->setReadOnly(true);
  log_text_->setPlaceholderText(QStringLiteral("诊断进度和 rosbag 回放输出会显示在这里。"));
  tabs->addTab(log_text_, QStringLiteral("运行日志"));
  outer_layout->addWidget(tabs, 1);

  auto * settings_group = new QGroupBox(QStringLiteral("诊断阈值（通用基线，可按设备规格调整）"), this);
  auto * settings_layout = new QHBoxLayout(settings_group);
  gap_factor_spin_ = new QDoubleSpinBox(settings_group);
  gap_factor_spin_->setRange(1.5, 20.0);
  gap_factor_spin_->setValue(3.0);
  gap_factor_spin_->setSingleStep(0.5);
  gap_factor_spin_->setSuffix(QStringLiteral(" × 周期"));
  jitter_warning_spin_ = new QDoubleSpinBox(settings_group);
  jitter_warning_spin_->setRange(1.0, 200.0);
  jitter_warning_spin_->setValue(20.0);
  jitter_warning_spin_->setSuffix(QStringLiteral(" %"));
  control_frequency_spin_ = new QDoubleSpinBox(settings_group);
  control_frequency_spin_->setRange(0.1, 1000.0);
  control_frequency_spin_->setValue(10.0);
  control_frequency_spin_->setSuffix(QStringLiteral(" Hz"));
  tf_jump_spin_ = new QDoubleSpinBox(settings_group);
  tf_jump_spin_->setRange(0.001, 100.0);
  tf_jump_spin_->setDecimals(3);
  tf_jump_spin_->setValue(0.5);
  tf_jump_spin_->setSuffix(QStringLiteral(" m"));
  payload_samples_spin_ = new QSpinBox(settings_group);
  payload_samples_spin_->setRange(100, 50000);
  payload_samples_spin_->setSingleStep(500);
  payload_samples_spin_->setValue(2000);
  payload_samples_spin_->setSuffix(QStringLiteral(" 条/话题"));
  deep_analysis_check_ = new QCheckBox(QStringLiteral("TF/传感器深度解析"), settings_group);
  deep_analysis_check_->setObjectName(QStringLiteral("rosbagDeepAnalysis"));
  deep_analysis_check_->setChecked(true);
  const std::array<std::pair<QString, QWidget *>, 5> settings{{
    {QStringLiteral("长间隙"), gap_factor_spin_},
    {QStringLiteral("抖动警告"), jitter_warning_spin_},
    {QStringLiteral("控制最低"), control_frequency_spin_},
    {QStringLiteral("TF 跳变"), tf_jump_spin_},
    {QStringLiteral("载荷抽样"), payload_samples_spin_}
  }};
  for (const auto & setting : settings) {
    settings_layout->addWidget(new QLabel(setting.first, settings_group));
    settings_layout->addWidget(setting.second);
  }
  settings_layout->addWidget(deep_analysis_check_);
  settings_layout->addStretch(1);
  outer_layout->addWidget(settings_group);

  auto * footer_layout = new QHBoxLayout;
  diagnostic_progress_ = new QProgressBar(this);
  diagnostic_progress_->setObjectName(QStringLiteral("rosbagDiagnosticProgress"));
  diagnostic_progress_->setRange(0, 100);
  diagnostic_progress_->setValue(0);
  diagnostic_progress_->setTextVisible(true);
  diagnostic_progress_->setFormat(QStringLiteral("等待选择 bag"));
  export_json_button_ = new QPushButton(QStringLiteral("导出 JSON"), this);
  export_html_button_ = new QPushButton(QStringLiteral("导出 HTML 报告"), this);
  open_report_button_ = new QPushButton(QStringLiteral("打开完整报告"), this);
  open_report_button_->setProperty("role", "primary");
  footer_layout->addWidget(diagnostic_progress_, 1);
  footer_layout->addWidget(export_json_button_);
  footer_layout->addWidget(export_html_button_);
  footer_layout->addWidget(open_report_button_);
  outer_layout->addLayout(footer_layout);
  connect(export_json_button_, &QPushButton::clicked,
    this, &RosbagDiagnosticDialog::export_json_report);
  connect(export_html_button_, &QPushButton::clicked,
    this, &RosbagDiagnosticDialog::export_html_report);
  connect(open_report_button_, &QPushButton::clicked,
    this, &RosbagDiagnosticDialog::open_html_report);

  reset_report();
  update_playback_controls();
}

void RosbagDiagnosticDialog::set_bag_path(const QString & path)
{
  if (path.trimmed().isEmpty()) return;
  const QString normalized = QFileInfo(path).absoluteFilePath();
  bag_path_edit_->setText(QDir::cleanPath(normalized));
  bag_kind_ = detect_rosbag_kind(normalized);
  bag_kind_label_->setText(rosbag_kind_label(bag_kind_));
  if (bag_kind_ != RosbagKind::Unknown) {
    QSettings().setValue(QStringLiteral("rosbagLastDirectory"),
      QFileInfo(normalized).isDir() ? normalized : QFileInfo(normalized).absolutePath());
    diagnostic_progress_->setFormat(QStringLiteral("已识别 %1").arg(rosbag_kind_label(bag_kind_)));
    append_log(QStringLiteral("已选择：%1").arg(normalized), QStringLiteral("INPUT"));
    if (ros_setup_edit_) {
      const QString detected_setup = guess_ros_setup_file(normalized);
      if (!detected_setup.isEmpty() &&
        QFileInfo(ros_setup_edit_->text().trimmed()).absoluteFilePath() != detected_setup)
      {
        ros_setup_edit_->setText(detected_setup);
        QSettings().setValue(QStringLiteral("rosbagSetupFile"), detected_setup);
        append_log(QStringLiteral("自动匹配 ROS 工作空间：%1").arg(detected_setup),
          QStringLiteral("ENV"));
      }
    }
  } else {
    diagnostic_progress_->setFormat(QStringLiteral("不支持的 bag 路径"));
  }
  reset_report();
  update_input_state();
}

void RosbagDiagnosticDialog::choose_bag_file()
{
  const QString initial = QSettings().value(
    QStringLiteral("rosbagLastDirectory"), QDir::homePath()).toString();
  const QString path = QFileDialog::getOpenFileName(
    this,
    QStringLiteral("选择 ROS bag"),
    initial,
    QStringLiteral("ROS bag (*.bag *.db3 *.db3.zstd *.db3.lz4 *.mcap);;ROS1 bag (*.bag);;ROS2 SQLite (*.db3 *.db3.zstd *.db3.lz4);;MCAP (*.mcap);;所有文件 (*)"));
  if (!path.isEmpty()) set_bag_path(path);
}

void RosbagDiagnosticDialog::choose_bag_directory()
{
  const QString initial = QSettings().value(
    QStringLiteral("rosbagLastDirectory"), QDir::homePath()).toString();
  const QString path = QFileDialog::getExistingDirectory(
    this, QStringLiteral("选择含 metadata.yaml 的 ROS2 bag 目录"), initial);
  if (!path.isEmpty()) set_bag_path(path);
}

void RosbagDiagnosticDialog::choose_ros_setup()
{
  QString initial = ros_setup_edit_->text().trimmed();
  if (initial.isEmpty()) {
    initial = QSettings().value(
      QStringLiteral("rosbagLastDirectory"), QDir::homePath()).toString();
  }
  const QString path = QFileDialog::getOpenFileName(
    this,
    QStringLiteral("选择 ROS 工作空间 setup.bash"),
    initial,
    QStringLiteral("ROS 环境脚本 (setup.bash);;Shell 脚本 (*.bash *.sh);;所有文件 (*)"));
  if (path.isEmpty()) return;
  ros_setup_edit_->setText(QFileInfo(path).absoluteFilePath());
  QSettings().setValue(QStringLiteral("rosbagSetupFile"), ros_setup_edit_->text());
  append_log(QStringLiteral("ROS 工作空间：%1").arg(ros_setup_edit_->text()),
    QStringLiteral("ENV"));
}

void RosbagDiagnosticDialog::start_diagnosis()
{
  const QString path = bag_path_edit_->text().trimmed();
  bag_kind_ = detect_rosbag_kind(path);
  if (bag_kind_ == RosbagKind::Unknown) {
    QMessageBox::warning(this, QStringLiteral("无法诊断"),
      QStringLiteral("请选择 ROS1 .bag、ROS2 bag 目录、.db3 或 .mcap。"));
    return;
  }
  const QString setup_file = ros_setup_edit_->text().trimmed();
  if (!setup_file.isEmpty() && !QFileInfo(setup_file).isFile()) {
    QMessageBox::warning(this, QStringLiteral("ROS 环境无效"),
      QStringLiteral("找不到 setup.bash：%1").arg(setup_file));
    return;
  }
  const QString script = rosbag_diagnostic_script();
  const QString wrapper = rosbag_diagnostic_wrapper_script();
  if (!QFileInfo::exists(script) || !QFileInfo::exists(wrapper)) {
    QMessageBox::critical(this, QStringLiteral("缺少诊断模块"),
      QStringLiteral("找不到：%1").arg(script));
    return;
  }
  if (!report_directory_.isValid()) {
    QMessageBox::critical(this, QStringLiteral("无法诊断"),
      QStringLiteral("无法创建临时报告目录。"));
    return;
  }
  if (diagnostic_process_.state() != QProcess::NotRunning) return;

  reset_report();
  const QString run_id = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
  current_json_path_ = QDir(report_directory_.path()).filePath(
    QStringLiteral("rosbag_diagnostic_%1.json").arg(run_id));
  current_html_path_ = QDir(report_directory_.path()).filePath(
    QStringLiteral("rosbag_diagnostic_%1.html").arg(run_id));
  QStringList arguments{
    script,
    path,
    QStringLiteral("--json"), current_json_path_,
    QStringLiteral("--html"), current_html_path_,
    QStringLiteral("--progress"),
    QStringLiteral("--gap-factor"), QString::number(gap_factor_spin_->value(), 'f', 2),
    QStringLiteral("--jitter-warning"), QString::number(jitter_warning_spin_->value(), 'f', 2),
    QStringLiteral("--control-min-hz"), QString::number(control_frequency_spin_->value(), 'f', 2),
    QStringLiteral("--tf-jump-m"), QString::number(tf_jump_spin_->value(), 'f', 4),
    QStringLiteral("--payload-samples"), QString::number(payload_samples_spin_->value())
  };
  if (!deep_analysis_check_->isChecked()) arguments.append(QStringLiteral("--no-deep"));

  diagnostic_error_buffer_.clear();
  diagnostic_process_.setProgram(QStringLiteral("/bin/bash"));
  diagnostic_process_.setArguments(
    QStringList{wrapper, rosbag_python_executable()} + arguments);
  diagnostic_process_.setWorkingDirectory(rosbag_project_root());
  QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
  environment.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
  if (!setup_file.isEmpty()) {
    environment.insert(QStringLiteral("PCD_MEASURE_ROS_SETUP"), setup_file);
  }
  diagnostic_process_.setProcessEnvironment(environment);
  append_log(QStringLiteral("开始只读诊断：%1").arg(path), QStringLiteral("DIAG"));
  set_busy(true, QStringLiteral("正在建立 bag 时间索引…"));
  diagnostic_process_.start();
}

void RosbagDiagnosticDialog::read_diagnostic_output()
{
  const QString output = QString::fromUtf8(diagnostic_process_.readAllStandardOutput()).trimmed();
  if (!output.isEmpty()) append_log(output, QStringLiteral("DIAG"));
}

void RosbagDiagnosticDialog::read_diagnostic_errors()
{
  diagnostic_error_buffer_.append(diagnostic_process_.readAllStandardError());
  consume_progress_lines();
}

void RosbagDiagnosticDialog::consume_progress_lines()
{
  int newline = -1;
  while ((newline = diagnostic_error_buffer_.indexOf('\n')) >= 0) {
    const QByteArray raw_line = diagnostic_error_buffer_.left(newline);
    diagnostic_error_buffer_.remove(0, newline + 1);
    const QString line = QString::fromUtf8(raw_line).trimmed();
    if (line.startsWith(QStringLiteral("PROGRESS\t"))) {
      const QStringList parts = line.split(QLatin1Char('\t'));
      if (parts.size() >= 3) {
        bool ok = false;
        const int value = parts.at(1).toInt(&ok);
        if (ok) diagnostic_progress_->setValue(std::clamp(value, 0, 100));
        diagnostic_progress_->setFormat(parts.mid(2).join(QStringLiteral(" ")));
        diagnostic_status_label_->setText(QStringLiteral("● ANALYZING %1%").arg(value));
        diagnostic_status_label_->setStyleSheet(
          QStringLiteral("color:#FFB85C; font:700 11px 'DejaVu Sans Mono';"));
      }
    } else if (!line.isEmpty()) {
      append_log(line, QStringLiteral("ERR"));
    }
  }
}

void RosbagDiagnosticDialog::diagnostic_finished(
  int exit_code,
  QProcess::ExitStatus exit_status)
{
  if (!diagnostic_error_buffer_.isEmpty()) {
    const QString remainder = QString::fromUtf8(diagnostic_error_buffer_).trimmed();
    diagnostic_error_buffer_.clear();
    if (!remainder.isEmpty()) append_log(remainder, QStringLiteral("ERR"));
  }
  set_busy(false);
  if (exit_status != QProcess::NormalExit || exit_code != 0) {
    diagnostic_progress_->setFormat(QStringLiteral("诊断失败；查看运行日志"));
    diagnostic_status_label_->setText(QStringLiteral("● FAILED"));
    diagnostic_status_label_->setStyleSheet(
      QStringLiteral("color:#FF716B; font:700 11px 'DejaVu Sans Mono';"));
    append_log(QStringLiteral("诊断结束，退出码 %1。").arg(exit_code), QStringLiteral("ERR"));
    return;
  }
  QString error;
  if (!load_report(current_json_path_, &error)) {
    diagnostic_progress_->setFormat(QStringLiteral("报告读取失败"));
    diagnostic_status_label_->setText(QStringLiteral("● REPORT ERROR"));
    append_log(error, QStringLiteral("ERR"));
    return;
  }
  diagnostic_progress_->setValue(100);
  diagnostic_progress_->setFormat(QStringLiteral("诊断完成；原始 bag 未被修改"));
  const QString report_status = current_report_.value(QStringLiteral("summary"))
    .toObject().value(QStringLiteral("status")).toString();
  diagnostic_status_label_->setText(
    report_status == QStringLiteral("critical") ? QStringLiteral("● CRITICAL") :
    report_status == QStringLiteral("warning") ? QStringLiteral("● REVIEW") :
    QStringLiteral("● HEALTHY"));
  diagnostic_status_label_->setStyleSheet(QStringLiteral(
    "color:%1; font:700 11px 'DejaVu Sans Mono';").arg(severity_color(report_status)));
  append_log(QStringLiteral("诊断完成：%1").arg(current_json_path_), QStringLiteral("DIAG"));
}

bool RosbagDiagnosticDialog::load_report(const QString & path, QString * error)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法打开诊断报告：%1").arg(path);
    return false;
  }
  QJsonParseError parse_error;
  const QByteArray payload = file.readAll();
  const QJsonDocument document = QJsonDocument::fromJson(payload, &parse_error);
  const RosbagReportSummary summary = parse_rosbag_report_summary(payload);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject() || !summary.valid) {
    if (error) *error = summary.error.isEmpty() ? parse_error.errorString() : summary.error;
    return false;
  }
  current_report_ = document.object();
  populate_report(current_report_);
  export_json_button_->setEnabled(true);
  export_html_button_->setEnabled(QFileInfo::exists(current_html_path_));
  open_report_button_->setEnabled(QFileInfo::exists(current_html_path_));
  return true;
}

void RosbagDiagnosticDialog::populate_report(const QJsonObject & report)
{
  const QJsonObject summary = report.value(QStringLiteral("summary")).toObject();
  const QJsonObject bag = report.value(QStringLiteral("bag")).toObject();
  score_label_->setText(QStringLiteral("%1 / 100").arg(summary.value(QStringLiteral("score")).toInt()));
  duration_label_->setText(json_number(bag.value(QStringLiteral("duration_sec")), 2, QStringLiteral(" s")));
  messages_label_->setText(json_number(bag.value(QStringLiteral("message_count")), 0));
  topics_label_->setText(json_number(bag.value(QStringLiteral("topic_count")), 0));
  alerts_label_->setText(QStringLiteral("%1 / %2")
    .arg(summary.value(QStringLiteral("critical_count")).toInt())
    .arg(summary.value(QStringLiteral("warning_count")).toInt()));
  coverage_label_->setText(json_number(
    summary.value(QStringLiteral("deep_analysis_coverage_percent")), 1, QStringLiteral(" %")));
  bag_kind_label_->setText(QStringLiteral("ROS%1 · %2")
    .arg(bag.value(QStringLiteral("ros_version")).toInt())
    .arg(bag.value(QStringLiteral("storage")).toString().toUpper()));
  score_label_->setStyleSheet(QStringLiteral(
    "color:%1; font:700 18px 'DejaVu Sans Mono';").arg(
      severity_color(summary.value(QStringLiteral("status")).toString())));
  populate_issue_table(report);
  populate_topic_table(report);
  populate_tf_table(report);
  populate_sensor_table(report);
  update_health_rail(report);

  QStringList recommendation_lines{QStringLiteral("建议执行顺序")};
  int number = 1;
  for (const QJsonValue & value : report.value(QStringLiteral("recommendations")).toArray()) {
    recommendation_lines.append(QStringLiteral("%1. %2").arg(number++).arg(value.toString()));
  }
  recommendation_lines.append(QString());
  recommendation_lines.append(QStringLiteral("结论边界"));
  for (const QJsonValue & value : report.value(QStringLiteral("limitations")).toArray()) {
    recommendation_lines.append(QStringLiteral("• %1").arg(value.toString()));
  }
  recommendations_text_->setPlainText(recommendation_lines.join(QLatin1Char('\n')));
}

void RosbagDiagnosticDialog::populate_issue_table(const QJsonObject & report)
{
  issue_table_->setSortingEnabled(false);
  const QJsonArray issues = report.value(QStringLiteral("issues")).toArray();
  issue_table_->setRowCount(issues.size());
  for (int row = 0; row < issues.size(); ++row) {
    const QJsonObject issue = issues.at(row).toObject();
    const QString severity = issue.value(QStringLiteral("severity")).toString();
    set_item(issue_table_, row, 0, severity_text(severity));
    color_severity_item(issue_table_->item(row, 0), severity);
    set_item(issue_table_, row, 1, issue.value(QStringLiteral("category")).toString());
    set_item(issue_table_, row, 2,
      issue.value(QStringLiteral("topic")).toString().isEmpty() ? QStringLiteral("全局") :
      issue.value(QStringLiteral("topic")).toString());
    set_item(issue_table_, row, 3, issue.value(QStringLiteral("title")).toString());
    set_item(issue_table_, row, 4, issue.value(QStringLiteral("evidence")).toString());
    set_item(issue_table_, row, 5, issue.value(QStringLiteral("suggestion")).toString());
  }
  issue_table_->setSortingEnabled(true);
}

void RosbagDiagnosticDialog::populate_topic_table(const QJsonObject & report)
{
  topic_table_->setSortingEnabled(false);
  const QJsonArray topics = report.value(QStringLiteral("topics")).toArray();
  topic_table_->setRowCount(topics.size());
  for (int row = 0; row < topics.size(); ++row) {
    const QJsonObject topic = topics.at(row).toObject();
    const QJsonObject qos = topic.value(QStringLiteral("qos")).toObject();
    const QString status = topic.value(QStringLiteral("status")).toString();
    set_item(topic_table_, row, 0, topic.value(QStringLiteral("name")).toString());
    set_item(topic_table_, row, 1, topic.value(QStringLiteral("type")).toString());
    set_item(topic_table_, row, 2, json_number(topic.value(QStringLiteral("count")), 0),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 3, json_number(topic.value(QStringLiteral("mean_hz")), 2),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 4, json_number(topic.value(QStringLiteral("median_period_ms")), 2, QStringLiteral(" ms")),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 5, json_number(topic.value(QStringLiteral("jitter_cv_percent")), 1, QStringLiteral(" %")),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 6, json_number(topic.value(QStringLiteral("max_gap_ms")), 1, QStringLiteral(" ms")),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 7, json_number(topic.value(QStringLiteral("estimated_drops")), 0),
      QString(), Qt::AlignRight | Qt::AlignVCenter);
    set_item(topic_table_, row, 8,
      topic.value(QStringLiteral("time_basis")).toString() == QStringLiteral("header") ?
      QStringLiteral("header") : QStringLiteral("bag 存储"));
    set_item(topic_table_, row, 9, qos.value(QStringLiteral("status")).toString(),
      qos.value(QStringLiteral("risk")).toString());
    set_item(topic_table_, row, 10, severity_text(status));
    color_severity_item(topic_table_->item(row, 10), status);
  }
  topic_table_->setSortingEnabled(true);
}

void RosbagDiagnosticDialog::populate_tf_table(const QJsonObject & report)
{
  tf_table_->setSortingEnabled(false);
  const QJsonArray edges = report.value(QStringLiteral("tf")).toObject()
    .value(QStringLiteral("edges")).toArray();
  tf_table_->setRowCount(edges.size());
  for (int row = 0; row < edges.size(); ++row) {
    const QJsonObject edge = edges.at(row).toObject();
    set_item(tf_table_, row, 0, QStringLiteral("%1 → %2")
      .arg(edge.value(QStringLiteral("parent")).toString())
      .arg(edge.value(QStringLiteral("child")).toString()));
    set_item(tf_table_, row, 1,
      edge.value(QStringLiteral("static")).toBool() ? QStringLiteral("静态") : QStringLiteral("动态"));
    set_item(tf_table_, row, 2, json_number(edge.value(QStringLiteral("samples")), 0));
    set_item(tf_table_, row, 3, json_number(edge.value(QStringLiteral("net_translation_m")), 3, QStringLiteral(" m")));
    set_item(tf_table_, row, 4, json_number(edge.value(QStringLiteral("max_step_m")), 3, QStringLiteral(" m")));
    set_item(tf_table_, row, 5, json_number(edge.value(QStringLiteral("max_rotation_step_deg")), 2, QStringLiteral("°")));
    set_item(tf_table_, row, 6, json_number(edge.value(QStringLiteral("max_gap_ms")), 1, QStringLiteral(" ms")));
    set_item(tf_table_, row, 7, json_number(edge.value(QStringLiteral("max_speed_mps")), 2, QStringLiteral(" m/s")));
  }
  tf_table_->resizeColumnsToContents();
  tf_table_->horizontalHeader()->setStretchLastSection(true);
  tf_table_->setSortingEnabled(true);
}

void RosbagDiagnosticDialog::populate_sensor_table(const QJsonObject & report)
{
  sensor_table_->setSortingEnabled(false);
  const QJsonArray sensors = report.value(QStringLiteral("sensors")).toArray();
  sensor_table_->setRowCount(sensors.size());
  for (int row = 0; row < sensors.size(); ++row) {
    const QJsonObject sensor = sensors.at(row).toObject();
    const QString status = sensor.value(QStringLiteral("status")).toString();
    set_item(sensor_table_, row, 0, sensor.value(QStringLiteral("topic")).toString());
    set_item(sensor_table_, row, 1, sensor.value(QStringLiteral("type")).toString());
    set_item(sensor_table_, row, 2, sensor.value(QStringLiteral("role")).toString());
    const std::array<QString, 6> numeric_keys{
      QStringLiteral("decoded_samples"), QStringLiteral("invalid_samples"),
      QStringLiteral("empty_samples"), QStringLiteral("malformed_samples"),
      QStringLiteral("saturated_samples"), QStringLiteral("maximum_repeat_run")};
    for (int column = 0; column < static_cast<int>(numeric_keys.size()); ++column) {
      set_item(sensor_table_, row, column + 3,
        json_number(sensor.value(numeric_keys[static_cast<std::size_t>(column)]), 0),
        QString(), Qt::AlignRight | Qt::AlignVCenter);
    }
    set_item(sensor_table_, row, 9, severity_text(status));
    color_severity_item(sensor_table_->item(row, 9), status);
  }
  sensor_table_->resizeColumnsToContents();
  sensor_table_->horizontalHeader()->setStretchLastSection(true);
  sensor_table_->setSortingEnabled(true);
}

void RosbagDiagnosticDialog::update_health_rail(const QJsonObject & report)
{
  std::array<int, 4> ranks{0, 0, 0, 0};
  for (const QJsonValue & value : report.value(QStringLiteral("issues")).toArray()) {
    const QJsonObject issue = value.toObject();
    const QString category = issue.value(QStringLiteral("category")).toString();
    const int rank = severity_rank(issue.value(QStringLiteral("severity")).toString());
    int index = 0;
    if (category == QStringLiteral("TF")) index = 1;
    else if (category == QStringLiteral("传感器") || category == QStringLiteral("硬件")) index = 2;
    else if (category == QStringLiteral("QoS")) index = 3;
    ranks[static_cast<std::size_t>(index)] = std::max(ranks[static_cast<std::size_t>(index)], rank);
  }
  const std::array<QFrame *, 4> segments{
    timing_segment_, tf_segment_, sensor_segment_, qos_segment_};
  const std::array<QString, 4> severities{
    QStringLiteral("normal"), QStringLiteral("notice"),
    QStringLiteral("warning"), QStringLiteral("critical")};
  for (int index = 0; index < 4; ++index) {
    segments[static_cast<std::size_t>(index)]->setStyleSheet(QStringLiteral(
      "background:%1; border:none; border-radius:2px; min-height:4px; max-height:4px;")
      .arg(severity_color(severities[static_cast<std::size_t>(ranks[static_cast<std::size_t>(index)])])));
  }
}

void RosbagDiagnosticDialog::reset_report()
{
  current_report_ = QJsonObject{};
  current_json_path_.clear();
  current_html_path_.clear();
  for (QLabel * label : {score_label_, duration_label_, messages_label_, topics_label_, alerts_label_, coverage_label_}) {
    if (label) label->setText(QStringLiteral("—"));
  }
  if (issue_table_) issue_table_->setRowCount(0);
  if (topic_table_) topic_table_->setRowCount(0);
  if (tf_table_) tf_table_->setRowCount(0);
  if (sensor_table_) sensor_table_->setRowCount(0);
  if (recommendations_text_) recommendations_text_->clear();
  if (export_json_button_) export_json_button_->setEnabled(false);
  if (export_html_button_) export_html_button_->setEnabled(false);
  if (open_report_button_) open_report_button_->setEnabled(false);
  for (QFrame * segment : {timing_segment_, tf_segment_, sensor_segment_, qos_segment_}) {
    if (segment) segment->setStyleSheet(QString());
  }
}

void RosbagDiagnosticDialog::set_busy(bool busy, const QString & message)
{
  diagnosis_busy_ = busy;
  const bool playback_running = playback_process_.state() != QProcess::NotRunning;
  const bool source_enabled = !busy && !playback_running;
  diagnose_button_->setEnabled(source_enabled && bag_kind_ != RosbagKind::Unknown);
  bag_path_edit_->setEnabled(source_enabled);
  ros_setup_edit_->setEnabled(source_enabled);
  choose_file_button_->setEnabled(source_enabled);
  choose_directory_button_->setEnabled(source_enabled);
  choose_setup_button_->setEnabled(source_enabled);
  deep_analysis_check_->setEnabled(!busy);
  payload_samples_spin_->setEnabled(!busy);
  gap_factor_spin_->setEnabled(!busy);
  jitter_warning_spin_->setEnabled(!busy);
  control_frequency_spin_->setEnabled(!busy);
  tf_jump_spin_->setEnabled(!busy);
  if (busy) {
    diagnostic_progress_->setRange(0, 100);
    diagnostic_progress_->setValue(0);
    diagnostic_progress_->setFormat(message);
  }
  update_playback_controls();
}

void RosbagDiagnosticDialog::update_input_state()
{
  bag_kind_ = detect_rosbag_kind(bag_path_edit_->text().trimmed());
  bag_kind_label_->setText(rosbag_kind_label(bag_kind_));
  const bool valid = bag_kind_ != RosbagKind::Unknown;
  diagnose_button_->setEnabled(valid && diagnostic_process_.state() == QProcess::NotRunning);
  update_playback_controls();
}

void RosbagDiagnosticDialog::append_log(const QString & text, const QString & channel)
{
  if (text.trimmed().isEmpty()) return;
  const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
  const QString prefix = channel.isEmpty() ? QStringLiteral("LOG") : channel;
  const QStringList lines = text.split(QLatin1Char('\n'));
  for (const QString & line : lines) {
    if (!line.trimmed().isEmpty()) {
      log_text_->append(QStringLiteral("[%1] %2 %3").arg(timestamp, prefix, line));
    }
  }
  if (log_text_->verticalScrollBar()) {
    log_text_->verticalScrollBar()->setValue(log_text_->verticalScrollBar()->maximum());
  }
}

void RosbagDiagnosticDialog::start_playback()
{
  if (playback_process_.state() != QProcess::NotRunning) return;
  bag_kind_ = detect_rosbag_kind(bag_path_edit_->text().trimmed());
  const QString setup_file = ros_setup_edit_->text().trimmed();
  if (!setup_file.isEmpty() && !QFileInfo(setup_file).isFile()) {
    QMessageBox::warning(this, QStringLiteral("ROS 环境无效"),
      QStringLiteral("找不到 setup.bash：%1").arg(setup_file));
    return;
  }
  RosbagPlaybackOptions options;
  options.rate = playback_rate_spin_->value();
  options.loop = playback_loop_check_->isChecked();
  options.publish_clock = playback_clock_check_->isChecked();
  const QStringList playback_arguments = rosbag_playback_arguments(
    bag_kind_, bag_path_edit_->text().trimmed(), options);
  const QString script = rosbag_playback_script();
  if (playback_arguments.isEmpty() || !QFileInfo::exists(script)) {
    QMessageBox::warning(this, QStringLiteral("无法回放"),
      QStringLiteral("bag 路径无效或缺少回放脚本。"));
    return;
  }
  playback_paused_ = false;
  playback_process_.setProgram(QStringLiteral("/bin/bash"));
  playback_process_.setArguments(QStringList{script} + playback_arguments);
  playback_process_.setWorkingDirectory(rosbag_project_root());
  QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
  if (!setup_file.isEmpty()) {
    environment.insert(QStringLiteral("PCD_MEASURE_ROS_SETUP"), setup_file);
  }
  playback_process_.setProcessEnvironment(environment);
  append_log(QStringLiteral("启动 %1 回放，倍率 %2×，/clock %3，循环 %4。")
    .arg(rosbag_kind_label(bag_kind_))
    .arg(options.rate, 0, 'g', 4)
    .arg(options.publish_clock ? QStringLiteral("开") : QStringLiteral("关"))
    .arg(options.loop ? QStringLiteral("开") : QStringLiteral("关")),
    QStringLiteral("PLAY"));
  playback_process_.start();
  update_playback_controls();
}

void RosbagDiagnosticDialog::pause_or_resume_playback()
{
  if (playback_process_.state() != QProcess::Running || playback_process_.processId() <= 0) return;
  const int signal = playback_paused_ ? SIGCONT : SIGSTOP;
  if (::kill(static_cast<pid_t>(playback_process_.processId()), signal) != 0) {
    append_log(QStringLiteral("无法切换回放暂停状态。"), QStringLiteral("ERR"));
    return;
  }
  playback_paused_ = !playback_paused_;
  pause_button_->setText(playback_paused_ ? QStringLiteral("继续") : QStringLiteral("暂停"));
  diagnostic_status_label_->setText(
    playback_paused_ ? QStringLiteral("● PLAY PAUSED") : QStringLiteral("● PLAYING"));
  append_log(playback_paused_ ? QStringLiteral("回放已暂停。") : QStringLiteral("回放已继续。"),
    QStringLiteral("PLAY"));
}

void RosbagDiagnosticDialog::stop_playback()
{
  if (playback_process_.state() == QProcess::NotRunning) return;
  if (playback_paused_) {
    ::kill(static_cast<pid_t>(playback_process_.processId()), SIGCONT);
    playback_paused_ = false;
  }
  append_log(QStringLiteral("正在停止回放…"), QStringLiteral("PLAY"));
  playback_process_.terminate();
  QTimer::singleShot(1800, this, [this]() {
    if (playback_process_.state() != QProcess::NotRunning) playback_process_.kill();
  });
  update_playback_controls();
}

void RosbagDiagnosticDialog::read_playback_output()
{
  const QString standard_output = QString::fromUtf8(
    playback_process_.readAllStandardOutput()).trimmed();
  const QString standard_error = QString::fromUtf8(
    playback_process_.readAllStandardError()).trimmed();
  if (!standard_output.isEmpty()) append_log(standard_output, QStringLiteral("PLAY"));
  if (!standard_error.isEmpty()) append_log(standard_error, QStringLiteral("PLAY"));
}

void RosbagDiagnosticDialog::playback_started()
{
  diagnostic_status_label_->setText(QStringLiteral("● PLAYING"));
  diagnostic_status_label_->setStyleSheet(
    QStringLiteral("color:#36D6D0; font:700 11px 'DejaVu Sans Mono';"));
  append_log(QStringLiteral("回放进程已启动。"), QStringLiteral("PLAY"));
  update_playback_controls();
}

void RosbagDiagnosticDialog::playback_finished(
  int exit_code,
  QProcess::ExitStatus exit_status)
{
  read_playback_output();
  playback_paused_ = false;
  pause_button_->setText(QStringLiteral("暂停"));
  const bool successful = exit_status == QProcess::NormalExit && exit_code == 0;
  diagnostic_status_label_->setText(
    successful ? QStringLiteral("● PLAY COMPLETE") : QStringLiteral("● PLAY STOPPED"));
  diagnostic_status_label_->setStyleSheet(QStringLiteral(
    "color:%1; font:700 11px 'DejaVu Sans Mono';")
    .arg(successful ? QStringLiteral("#36D6D0") : QStringLiteral("#FFB85C")));
  append_log(QStringLiteral("回放结束，退出码 %1。").arg(exit_code), QStringLiteral("PLAY"));
  update_playback_controls();
}

void RosbagDiagnosticDialog::update_playback_controls()
{
  const bool running = playback_process_.state() != QProcess::NotRunning;
  const bool valid = bag_kind_ != RosbagKind::Unknown;
  const bool source_enabled = !running && !diagnosis_busy_;
  play_button_->setEnabled(valid && source_enabled);
  pause_button_->setEnabled(running);
  stop_button_->setEnabled(running);
  playback_rate_spin_->setEnabled(!running);
  playback_loop_check_->setEnabled(!running);
  playback_clock_check_->setEnabled(!running);
  bag_path_edit_->setEnabled(source_enabled);
  ros_setup_edit_->setEnabled(source_enabled);
  choose_file_button_->setEnabled(source_enabled);
  choose_directory_button_->setEnabled(source_enabled);
  choose_setup_button_->setEnabled(source_enabled);
  diagnose_button_->setEnabled(valid && source_enabled);
}

void RosbagDiagnosticDialog::export_json_report()
{
  if (!QFileInfo::exists(current_json_path_)) return;
  const QString suggested = QDir(preferred_report_directory(bag_path_edit_->text())).filePath(
    QStringLiteral("rosbag_diagnostic_%1.json").arg(
      QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
  const QString destination = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出 JSON 诊断报告"), suggested, QStringLiteral("JSON (*.json)"));
  if (destination.isEmpty()) return;
  QString error;
  if (!copy_report_atomically(current_json_path_, destination, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
    return;
  }
  append_log(QStringLiteral("JSON 报告已导出：%1").arg(destination), QStringLiteral("REPORT"));
}

void RosbagDiagnosticDialog::export_html_report()
{
  if (!QFileInfo::exists(current_html_path_)) return;
  const QString suggested = QDir(preferred_report_directory(bag_path_edit_->text())).filePath(
    QStringLiteral("rosbag_diagnostic_%1.html").arg(
      QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
  const QString destination = QFileDialog::getSaveFileName(
    this, QStringLiteral("导出 HTML 诊断报告"), suggested, QStringLiteral("HTML (*.html)"));
  if (destination.isEmpty()) return;
  QString error;
  if (!copy_report_atomically(current_html_path_, destination, &error)) {
    QMessageBox::critical(this, QStringLiteral("导出失败"), error);
    return;
  }
  append_log(QStringLiteral("HTML 报告已导出：%1").arg(destination), QStringLiteral("REPORT"));
}

void RosbagDiagnosticDialog::open_html_report()
{
  if (!QFileInfo::exists(current_html_path_)) return;
  QDesktopServices::openUrl(QUrl::fromLocalFile(current_html_path_));
}

bool RosbagDiagnosticDialog::copy_report_atomically(
  const QString & source,
  const QString & destination,
  QString * error) const
{
  QFile input(source);
  QSaveFile output(destination);
  if (!input.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法读取临时报告：%1").arg(source);
    return false;
  }
  if (!output.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入：%1").arg(destination);
    return false;
  }
  while (!input.atEnd()) {
    const QByteArray block = input.read(1024 * 1024);
    if (block.isEmpty() && input.error() != QFileDevice::NoError) {
      output.cancelWriting();
      if (error) *error = QStringLiteral("读取临时报告失败。");
      return false;
    }
    if (output.write(block) != block.size()) {
      output.cancelWriting();
      if (error) *error = QStringLiteral("写入报告失败：%1").arg(destination);
      return false;
    }
  }
  if (!output.commit()) {
    if (error) *error = QStringLiteral("无法提交报告文件：%1").arg(destination);
    return false;
  }
  return true;
}

void RosbagDiagnosticDialog::dragEnterEvent(QDragEnterEvent * event)
{
  if (!event->mimeData()->hasUrls()) return;
  for (const QUrl & url : event->mimeData()->urls()) {
    if (url.isLocalFile() && detect_rosbag_kind(url.toLocalFile()) != RosbagKind::Unknown) {
      event->acceptProposedAction();
      return;
    }
  }
}

void RosbagDiagnosticDialog::dropEvent(QDropEvent * event)
{
  for (const QUrl & url : event->mimeData()->urls()) {
    if (!url.isLocalFile()) continue;
    const QString path = url.toLocalFile();
    if (detect_rosbag_kind(path) != RosbagKind::Unknown) {
      event->acceptProposedAction();
      set_bag_path(path);
      return;
    }
  }
}

void RosbagDiagnosticDialog::closeEvent(QCloseEvent * event)
{
  if (diagnostic_process_.state() != QProcess::NotRunning) {
    diagnostic_process_.kill();
    diagnostic_process_.waitForFinished(1000);
  }
  stop_playback();
  event->accept();
}
