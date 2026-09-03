#pragma once

#include <QDialog>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryDir>

#include "rosbag_tools.h"

class QCheckBox;
class QCloseEvent;
class QDoubleSpinBox;
class QDragEnterEvent;
class QDropEvent;
class QFrame;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTextEdit;

class RosbagDiagnosticDialog final : public QDialog
{
  Q_OBJECT

public:
  explicit RosbagDiagnosticDialog(
    const QString & initial_path = QString(),
    QWidget * parent = nullptr,
    bool embedded = false);
  ~RosbagDiagnosticDialog() override;

  void set_bag_path(const QString & path);

public slots:
  void reject() override;

protected:
  void closeEvent(QCloseEvent * event) override;
  void dragEnterEvent(QDragEnterEvent * event) override;
  void dropEvent(QDropEvent * event) override;

private slots:
  void choose_bag_file();
  void choose_bag_directory();
  void choose_ros_setup();
  void start_diagnosis();
  void read_diagnostic_output();
  void read_diagnostic_errors();
  void diagnostic_finished(int exit_code, QProcess::ExitStatus exit_status);
  void start_playback();
  void pause_or_resume_playback();
  void stop_playback();
  void read_playback_output();
  void playback_started();
  void playback_finished(int exit_code, QProcess::ExitStatus exit_status);
  void export_json_report();
  void export_html_report();
  void open_html_report();

private:
  void build_interface();
  void set_busy(bool busy, const QString & message = QString());
  void reset_report();
  bool load_report(const QString & path, QString * error = nullptr);
  void populate_report(const QJsonObject & report);
  void populate_issue_table(const QJsonObject & report);
  void populate_topic_table(const QJsonObject & report);
  void populate_tf_table(const QJsonObject & report);
  void populate_sensor_table(const QJsonObject & report);
  void update_health_rail(const QJsonObject & report);
  void update_input_state();
  void append_log(const QString & text, const QString & channel = QString());
  void consume_progress_lines();
  void update_playback_controls();
  bool copy_report_atomically(
    const QString & source,
    const QString & destination,
    QString * error = nullptr) const;

  QLineEdit * bag_path_edit_ = nullptr;
  QLineEdit * ros_setup_edit_ = nullptr;
  QLabel * bag_kind_label_ = nullptr;
  QLabel * diagnostic_status_label_ = nullptr;
  QLabel * score_label_ = nullptr;
  QLabel * duration_label_ = nullptr;
  QLabel * messages_label_ = nullptr;
  QLabel * topics_label_ = nullptr;
  QLabel * alerts_label_ = nullptr;
  QLabel * coverage_label_ = nullptr;
  QFrame * timing_segment_ = nullptr;
  QFrame * tf_segment_ = nullptr;
  QFrame * sensor_segment_ = nullptr;
  QFrame * qos_segment_ = nullptr;
  QProgressBar * diagnostic_progress_ = nullptr;
  QTableWidget * issue_table_ = nullptr;
  QTableWidget * topic_table_ = nullptr;
  QTableWidget * tf_table_ = nullptr;
  QTableWidget * sensor_table_ = nullptr;
  QTextEdit * recommendations_text_ = nullptr;
  QTextEdit * log_text_ = nullptr;
  QDoubleSpinBox * playback_rate_spin_ = nullptr;
  QDoubleSpinBox * gap_factor_spin_ = nullptr;
  QDoubleSpinBox * jitter_warning_spin_ = nullptr;
  QDoubleSpinBox * control_frequency_spin_ = nullptr;
  QDoubleSpinBox * tf_jump_spin_ = nullptr;
  QSpinBox * payload_samples_spin_ = nullptr;
  QCheckBox * playback_loop_check_ = nullptr;
  QCheckBox * playback_clock_check_ = nullptr;
  QCheckBox * deep_analysis_check_ = nullptr;
  QPushButton * diagnose_button_ = nullptr;
  QPushButton * choose_file_button_ = nullptr;
  QPushButton * choose_directory_button_ = nullptr;
  QPushButton * choose_setup_button_ = nullptr;
  QPushButton * play_button_ = nullptr;
  QPushButton * pause_button_ = nullptr;
  QPushButton * stop_button_ = nullptr;
  QPushButton * export_json_button_ = nullptr;
  QPushButton * export_html_button_ = nullptr;
  QPushButton * open_report_button_ = nullptr;

  QProcess diagnostic_process_;
  QProcess playback_process_;
  QTemporaryDir report_directory_;
  QByteArray diagnostic_error_buffer_;
  QJsonObject current_report_;
  QString current_json_path_;
  QString current_html_path_;
  RosbagKind bag_kind_ = RosbagKind::Unknown;
  bool diagnosis_busy_ = false;
  bool playback_paused_ = false;
};
