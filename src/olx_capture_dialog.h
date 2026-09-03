#pragma once

#include <QByteArray>
#include <QDialog>
#include <QProcess>

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;

class OlxCaptureDialog : public QDialog
{
  Q_OBJECT

public:
  explicit OlxCaptureDialog(QWidget * parent = nullptr);

signals:
  void openOlxRequested(const QString & path);

protected:
  void closeEvent(QCloseEvent * event) override;

private slots:
  void browse_workspace();
  void browse_output();
  void refresh_status();
  void toggle_driver();
  void start_recording();
  void stop_recording();
  void consume_recorder_output();
  void recorder_finished(int exit_code, QProcess::ExitStatus status);

private:
  QString project_directory() const;
  QString selected_cloud_topic() const;
  void append_log(const QString & text);
  void update_recording_controls();
  bool usb_device_present() const;

  QLineEdit * workspace_edit_ = nullptr;
  QLineEdit * output_edit_ = nullptr;
  QComboBox * scan_mode_combo_ = nullptr;
  QLineEdit * pose_topic_edit_ = nullptr;
  QLineEdit * image_topic_edit_ = nullptr;
  QCheckBox * pose_check_ = nullptr;
  QCheckBox * image_check_ = nullptr;
  QCheckBox * auto_open_check_ = nullptr;
  QLabel * usb_status_label_ = nullptr;
  QLabel * topic_status_label_ = nullptr;
  QLabel * recording_status_label_ = nullptr;
  QLabel * counter_label_ = nullptr;
  QPushButton * driver_button_ = nullptr;
  QPushButton * record_button_ = nullptr;
  QPushButton * stop_button_ = nullptr;
  QPushButton * open_button_ = nullptr;
  QTextEdit * log_edit_ = nullptr;

  QProcess topic_probe_;
  QProcess driver_process_;
  QProcess recorder_process_;
  QByteArray recorder_output_buffer_;
  QString last_session_path_;
  QString last_olx_path_;
  bool close_when_finished_ = false;
  bool driver_started_here_ = false;
  bool selected_topic_ready_ = false;
  bool odin_driver_detected_ = false;
};
