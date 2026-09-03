#include <QtTest>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "rosbag_tools.h"

class RosbagToolsTest : public QObject
{
  Q_OBJECT

private slots:
  void detects_supported_inputs();
  void builds_safe_playback_arguments();
  void parses_report_summary();
  void rejects_invalid_report();
};

void RosbagToolsTest::detects_supported_inputs()
{
  QTemporaryDir temporary;
  QVERIFY(temporary.isValid());
  const QString ros1_path = temporary.filePath(QStringLiteral("recording.bag"));
  const QString db3_path = temporary.filePath(QStringLiteral("recording.db3"));
  const QString mcap_path = temporary.filePath(QStringLiteral("recording.mcap"));
  const QString compressed_path = temporary.filePath(QStringLiteral("recording.db3.zstd"));
  for (const QString & path : {ros1_path, db3_path, mcap_path, compressed_path}) {
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write("test");
  }
  QCOMPARE(detect_rosbag_kind(ros1_path), RosbagKind::Ros1);
  QCOMPARE(detect_rosbag_kind(db3_path), RosbagKind::Ros2Sqlite);
  QCOMPARE(detect_rosbag_kind(mcap_path), RosbagKind::Ros2Mcap);
  QCOMPARE(detect_rosbag_kind(compressed_path), RosbagKind::Ros2Sqlite);

  const QString bag_directory = temporary.filePath(QStringLiteral("bag_directory"));
  QVERIFY(QDir().mkpath(bag_directory));
  QFile metadata(QDir(bag_directory).filePath(QStringLiteral("metadata.yaml")));
  QVERIFY(metadata.open(QIODevice::WriteOnly));
  metadata.write("rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n");
  metadata.close();
  QFile database(QDir(bag_directory).filePath(QStringLiteral("part_0.db3")));
  QVERIFY(database.open(QIODevice::WriteOnly));
  database.write("test");
  database.close();
  QCOMPARE(detect_rosbag_kind(bag_directory), RosbagKind::Ros2Sqlite);
  QCOMPARE(rosbag_playback_target(database.fileName(), RosbagKind::Ros2Sqlite), bag_directory);
  const QString workspace = temporary.filePath(QStringLiteral("robot_ws"));
  const QString nested_bag = QDir(workspace).filePath(QStringLiteral("bags/run/sample.db3"));
  QVERIFY(QDir().mkpath(QFileInfo(nested_bag).absolutePath()));
  QVERIFY(QDir().mkpath(QDir(workspace).filePath(QStringLiteral("install"))));
  QFile setup(QDir(workspace).filePath(QStringLiteral("install/setup.bash")));
  QVERIFY(setup.open(QIODevice::WriteOnly));
  setup.write("true\n");
  setup.close();
  QFile nested_database(nested_bag);
  QVERIFY(nested_database.open(QIODevice::WriteOnly));
  nested_database.close();
  QCOMPARE(guess_ros_setup_file(nested_bag), setup.fileName());
  QCOMPARE(detect_rosbag_kind(temporary.filePath(QStringLiteral("missing.bag"))), RosbagKind::Unknown);
}

void RosbagToolsTest::builds_safe_playback_arguments()
{
  QTemporaryDir temporary;
  QVERIFY(temporary.isValid());
  const QString path = temporary.filePath(QStringLiteral("bag with spaces.bag"));
  QFile file(path);
  QVERIFY(file.open(QIODevice::WriteOnly));
  file.write("test");
  file.close();
  RosbagPlaybackOptions options;
  options.rate = 0.5;
  options.loop = true;
  options.publish_clock = false;

  const QStringList arguments = rosbag_playback_arguments(RosbagKind::Ros1, path, options);

  QCOMPARE(arguments.size(), 5);
  QCOMPARE(arguments.at(0), QStringLiteral("1"));
  QCOMPARE(arguments.at(1), path);
  QCOMPARE(arguments.at(2), QStringLiteral("0.5"));
  QCOMPARE(arguments.at(3), QStringLiteral("1"));
  QCOMPARE(arguments.at(4), QStringLiteral("0"));
  QVERIFY(rosbag_playback_arguments(RosbagKind::Unknown, path, options).isEmpty());
  options.rate = 0.0;
  QVERIFY(rosbag_playback_arguments(RosbagKind::Ros1, path, options).isEmpty());
}

void RosbagToolsTest::parses_report_summary()
{
  QJsonObject summary{
    {QStringLiteral("score"), 83},
    {QStringLiteral("status"), QStringLiteral("warning")},
    {QStringLiteral("critical_count"), 0},
    {QStringLiteral("warning_count"), 2},
    {QStringLiteral("notice_count"), 1}
  };
  QJsonObject bag{
    {QStringLiteral("ros_version"), 2},
    {QStringLiteral("storage"), QStringLiteral("sqlite3")},
    {QStringLiteral("topic_count"), 7},
    {QStringLiteral("message_count"), 12345.0},
    {QStringLiteral("duration_sec"), 42.5}
  };
  QJsonObject root{
    {QStringLiteral("schema"), QStringLiteral("rosbag-diagnostic-report")},
    {QStringLiteral("summary"), summary},
    {QStringLiteral("bag"), bag}
  };

  const RosbagReportSummary parsed = parse_rosbag_report_summary(
    QJsonDocument(root).toJson(QJsonDocument::Compact));

  QVERIFY(parsed.valid);
  QCOMPARE(parsed.score, 83);
  QCOMPARE(parsed.ros_version, 2);
  QCOMPARE(parsed.topic_count, 7);
  QCOMPARE(parsed.message_count, qint64(12345));
  QCOMPARE(parsed.warning_count, 2);
  QCOMPARE(parsed.duration_seconds, 42.5);
}

void RosbagToolsTest::rejects_invalid_report()
{
  RosbagReportSummary parsed = parse_rosbag_report_summary(QByteArray("not-json"));
  QVERIFY(!parsed.valid);
  QVERIFY(!parsed.error.isEmpty());

  parsed = parse_rosbag_report_summary(QByteArray("{\"schema\":\"different\"}"));
  QVERIFY(!parsed.valid);
  QVERIFY(parsed.error.contains(QStringLiteral("支持")));
}

QTEST_GUILESS_MAIN(RosbagToolsTest)
#include "rosbag_tools_test.moc"
