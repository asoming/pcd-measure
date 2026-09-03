#include "rosbag_tools.h"

#include <cmath>

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{

QString first_existing_project_root(const QStringList & candidates)
{
  for (const QString & candidate : candidates) {
    if (candidate.isEmpty()) continue;
    const QDir directory(QDir::cleanPath(candidate));
    if (directory.exists(QStringLiteral("tools/rosbag_diagnose.py")) &&
      directory.exists(QStringLiteral("scripts/rosbag_play.sh")))
    {
      return directory.absolutePath();
    }
  }
  return QString();
}

}

RosbagKind detect_rosbag_kind(const QString & path)
{
  const QFileInfo info(path);
  if (!info.exists()) return RosbagKind::Unknown;
  if (info.isFile()) {
    const QString suffix = info.suffix().toLower();
    const QString lower_name = info.fileName().toLower();
    if (suffix == QStringLiteral("bag")) return RosbagKind::Ros1;
    if (suffix == QStringLiteral("db3") ||
      lower_name.endsWith(QStringLiteral(".db3.zstd")) ||
      lower_name.endsWith(QStringLiteral(".db3.lz4")))
    {
      return RosbagKind::Ros2Sqlite;
    }
    if (suffix == QStringLiteral("mcap")) return RosbagKind::Ros2Mcap;
    return RosbagKind::Unknown;
  }
  if (!info.isDir()) return RosbagKind::Unknown;
  const QDir directory(info.absoluteFilePath());
  if (!directory.exists(QStringLiteral("metadata.yaml"))) return RosbagKind::Unknown;
  if (!directory.entryList(
      {QStringLiteral("*.db3"), QStringLiteral("*.db3.zstd"), QStringLiteral("*.db3.lz4")},
      QDir::Files).isEmpty())
  {
    return RosbagKind::Ros2Sqlite;
  }
  if (!directory.entryList({QStringLiteral("*.mcap")}, QDir::Files).isEmpty()) {
    return RosbagKind::Ros2Mcap;
  }
  return RosbagKind::Unknown;
}

QString rosbag_kind_label(RosbagKind kind)
{
  switch (kind) {
    case RosbagKind::Ros1: return QStringLiteral("ROS1 · BAG");
    case RosbagKind::Ros2Sqlite: return QStringLiteral("ROS2 · SQLite3");
    case RosbagKind::Ros2Mcap: return QStringLiteral("ROS2 · MCAP");
    case RosbagKind::Unknown: break;
  }
  return QStringLiteral("未识别");
}

QString rosbag_playback_target(const QString & path, RosbagKind kind)
{
  const QFileInfo info(path);
  if (kind == RosbagKind::Ros2Sqlite && info.isFile() &&
    QFileInfo::exists(info.absoluteDir().filePath(QStringLiteral("metadata.yaml"))))
  {
    return info.absolutePath();
  }
  return info.absoluteFilePath();
}

QString guess_ros_setup_file(const QString & bag_path)
{
  const QString configured = qEnvironmentVariable("PCD_MEASURE_ROS_SETUP");
  if (QFileInfo(configured).isFile()) return QFileInfo(configured).absoluteFilePath();

  QFileInfo input(bag_path);
  QDir directory(input.isDir() ? input.absoluteFilePath() : input.absolutePath());
  for (int depth = 0; depth < 8; ++depth) {
    const QString candidate = directory.filePath(QStringLiteral("install/setup.bash"));
    if (QFileInfo(candidate).isFile()) return QFileInfo(candidate).absoluteFilePath();
    if (!directory.cdUp()) break;
  }
  return QString();
}

QString rosbag_project_root()
{
  const QString environment_root = qEnvironmentVariable("PCD_MEASURE_PROJECT_DIR");
  const QDir application_directory(QCoreApplication::applicationDirPath());
  const QString detected = first_existing_project_root({
    environment_root,
    application_directory.absoluteFilePath(QStringLiteral("../..")),
    application_directory.absoluteFilePath(QStringLiteral("..")),
    QDir::currentPath()
  });
  return detected.isEmpty() ? QDir::currentPath() : detected;
}

QString rosbag_python_executable()
{
  const QString configured = qEnvironmentVariable("PCD_MEASURE_ROSBAG_PYTHON");
  if (!configured.isEmpty() && QFileInfo(configured).isExecutable()) return configured;
  const QString virtual_environment = QDir(rosbag_project_root()).filePath(
    QStringLiteral(".rosbag-venv/bin/python"));
  if (QFileInfo(virtual_environment).isExecutable()) return virtual_environment;
  if (QFileInfo(QStringLiteral("/usr/bin/python3")).isExecutable()) {
    return QStringLiteral("/usr/bin/python3");
  }
  return QStringLiteral("python3");
}

QString rosbag_diagnostic_script()
{
  return QDir(rosbag_project_root()).filePath(QStringLiteral("tools/rosbag_diagnose.py"));
}

QString rosbag_diagnostic_wrapper_script()
{
  return QDir(rosbag_project_root()).filePath(QStringLiteral("scripts/rosbag_diagnose.sh"));
}

QString rosbag_playback_script()
{
  return QDir(rosbag_project_root()).filePath(QStringLiteral("scripts/rosbag_play.sh"));
}

QStringList rosbag_playback_arguments(
  RosbagKind kind,
  const QString & path,
  const RosbagPlaybackOptions & options)
{
  if (kind == RosbagKind::Unknown || path.isEmpty() ||
    !std::isfinite(options.rate) || options.rate <= 0.0)
  {
    return {};
  }
  const bool ros1 = kind == RosbagKind::Ros1;
  return {
    ros1 ? QStringLiteral("1") : QStringLiteral("2"),
    rosbag_playback_target(path, kind),
    QString::number(options.rate, 'g', 8),
    options.loop ? QStringLiteral("1") : QStringLiteral("0"),
    options.publish_clock ? QStringLiteral("1") : QStringLiteral("0")
  };
}

RosbagReportSummary parse_rosbag_report_summary(const QByteArray & json)
{
  RosbagReportSummary result;
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(json, &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    result.error = QStringLiteral("报告 JSON 无效：%1").arg(parse_error.errorString());
    return result;
  }
  const QJsonObject root = document.object();
  if (root.value(QStringLiteral("schema")).toString() !=
    QStringLiteral("rosbag-diagnostic-report"))
  {
    result.error = QStringLiteral("不是受支持的 ROS bag 诊断报告。");
    return result;
  }
  const QJsonObject summary = root.value(QStringLiteral("summary")).toObject();
  const QJsonObject bag = root.value(QStringLiteral("bag")).toObject();
  if (summary.isEmpty() || bag.isEmpty()) {
    result.error = QStringLiteral("报告缺少 summary 或 bag 字段。");
    return result;
  }
  result.valid = true;
  result.score = summary.value(QStringLiteral("score")).toInt();
  result.status = summary.value(QStringLiteral("status")).toString();
  result.critical_count = summary.value(QStringLiteral("critical_count")).toInt();
  result.warning_count = summary.value(QStringLiteral("warning_count")).toInt();
  result.notice_count = summary.value(QStringLiteral("notice_count")).toInt();
  result.ros_version = bag.value(QStringLiteral("ros_version")).toInt();
  result.storage = bag.value(QStringLiteral("storage")).toString();
  result.topic_count = bag.value(QStringLiteral("topic_count")).toInt();
  result.message_count = static_cast<qint64>(bag.value(QStringLiteral("message_count")).toDouble());
  result.duration_seconds = bag.value(QStringLiteral("duration_sec")).toDouble();
  return result;
}
