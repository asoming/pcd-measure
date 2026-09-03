#pragma once

#include <QString>
#include <QStringList>

enum class RosbagKind
{
  Unknown,
  Ros1,
  Ros2Sqlite,
  Ros2Mcap
};

struct RosbagPlaybackOptions
{
  double rate = 1.0;
  bool loop = false;
  bool publish_clock = true;
};

struct RosbagReportSummary
{
  bool valid = false;
  int score = 0;
  int ros_version = 0;
  int topic_count = 0;
  qint64 message_count = 0;
  int critical_count = 0;
  int warning_count = 0;
  int notice_count = 0;
  double duration_seconds = 0.0;
  QString status;
  QString storage;
  QString error;
};

RosbagKind detect_rosbag_kind(const QString & path);
QString rosbag_kind_label(RosbagKind kind);
QString rosbag_playback_target(const QString & path, RosbagKind kind);
QString guess_ros_setup_file(const QString & bag_path);
QString rosbag_project_root();
QString rosbag_python_executable();
QString rosbag_diagnostic_script();
QString rosbag_diagnostic_wrapper_script();
QString rosbag_playback_script();
QStringList rosbag_playback_arguments(
  RosbagKind kind,
  const QString & path,
  const RosbagPlaybackOptions & options);
RosbagReportSummary parse_rosbag_report_summary(const QByteArray & json);
