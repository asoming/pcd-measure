#pragma once

#include <cstddef>
#include <cstdint>

#include <QByteArray>
#include <QDialog>
#include <QElapsedTimer>
#include <QProcess>
#include <QTemporaryDir>

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QTimer;
class LiveCloudPreviewWidget;

class OlxCaptureDialog : public QDialog
{
  Q_OBJECT

  friend class MainWindowTest;

public:
  explicit OlxCaptureDialog(QWidget * parent = nullptr, bool embedded = false);
  ~OlxCaptureDialog() override;

  void activate_workspace();

signals:
  void openOlxRequested(const QString & path);

public slots:
  void reject() override;

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
  void refresh_live_preview();

private:
  QString project_directory() const;
  QString selected_cloud_topic() const;
  QString selected_preview_mode() const;
  void append_log(const QString & text);
  void update_recording_controls();
  bool usb_device_present() const;
  void stop_child_processes();

  QLineEdit * workspace_edit_ = nullptr;
  QLineEdit * output_edit_ = nullptr;
  QComboBox * scan_mode_combo_ = nullptr;
  QComboBox * preview_mode_combo_ = nullptr;
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
  QDialogButtonBox * close_buttons_ = nullptr;
  LiveCloudPreviewWidget * live_preview_ = nullptr;
  QLabel * preview_mark_label_ = nullptr;
  QLabel * preview_status_label_ = nullptr;
  QTimer * preview_timer_ = nullptr;

  QProcess topic_probe_;
  QProcess driver_process_;
  QProcess recorder_process_;
  QByteArray recorder_output_buffer_;
  QString last_session_path_;
  QString last_olx_path_;
  QString last_preview_error_;
  QString preview_file_path_;
  QTemporaryDir preview_directory_;
  QElapsedTimer preview_rate_timer_;
  std::uint32_t last_preview_frame_id_ = 0;
  std::size_t last_preview_point_count_ = 0;
  std::uint64_t last_preview_source_points_ = 0;
  double preview_voxel_size_m_ = 0.0;
  bool have_preview_frame_ = false;
  double preview_rate_hz_ = 0.0;
  bool embedded_ = false;
  bool close_when_finished_ = false;
  bool driver_started_here_ = false;
  bool selected_topic_ready_ = false;
  bool odin_driver_detected_ = false;
};
