#pragma once

#include <QByteArray>
#include <QList>
#include <QProcess>
#include <QProcessEnvironment>
#include <QStringList>
#include <QWidget>

class QCheckBox;
class QColor;
class QLabel;
class QProgressBar;
class QPushButton;
class QTableWidget;
class QTextEdit;

class EnvironmentSetupPanel final : public QWidget
{
  Q_OBJECT

  friend class MainWindowTest;

public:
  explicit EnvironmentSetupPanel(QWidget * parent = nullptr);

  void activate_workspace();

private slots:
  void check_environment();
  void install_missing_environment();
  void consume_process_output();
  void process_finished(int exit_code, QProcess::ExitStatus exit_status);

private:
  enum class Task
  {
    None,
    CheckSystem,
    InstallSystem,
    BuildApplication,
    SetupRosbag,
    SetupMapConverter,
    InstallDesktop
  };

  QString project_directory() const;
  QString desktop_launcher_path() const;
  QString applications_launcher_path() const;
  bool rosbag_environment_ready() const;
  bool map_converter_ready() const;
  bool application_binary_ready() const;
  bool desktop_launchers_ready() const;
  bool ros2_runtime_ready() const;
  void refresh_non_system_state();
  void update_table();
  void set_row(int row, const QString & component, const QString & status,
    const QString & detail, const QColor & color);
  void set_overall_status(const QString & text, const QString & tone);
  void append_log(const QString & text);
  bool start_process(Task task, const QString & program, const QStringList & arguments,
    const QProcessEnvironment & environment = QProcessEnvironment::systemEnvironment());
  bool start_system_install();
  bool sudo_credentials_cached() const;
  void start_next_install_step();
  QString task_name(Task task) const;

  QLabel * overall_status_label_ = nullptr;
  QTableWidget * environment_table_ = nullptr;
  QCheckBox * rosbag_check_ = nullptr;
  QCheckBox * map_converter_check_ = nullptr;
  QCheckBox * desktop_check_ = nullptr;
  QPushButton * check_button_ = nullptr;
  QPushButton * install_button_ = nullptr;
  QProgressBar * progress_bar_ = nullptr;
  QTextEdit * log_edit_ = nullptr;

  QProcess process_;
  QByteArray process_output_;
  QStringList missing_system_packages_;
  QList<Task> install_steps_;
  Task current_task_ = Task::None;
  int install_step_count_ = 0;
  int completed_install_steps_ = 0;
  bool check_complete_ = false;
  bool installation_running_ = false;
  bool system_ready_ = false;
  bool build_ready_ = false;
  bool rosbag_ready_ = false;
  bool map_converter_ready_ = false;
  bool desktop_ready_ = false;
  bool ros2_ready_ = false;
  QString sudo_program_ = QStringLiteral("/usr/bin/sudo");
  bool force_password_prompt_for_test_ = false;
};
