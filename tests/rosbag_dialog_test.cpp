#include <QtTest>

#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTemporaryDir>

#include "rosbag_dialog.h"

class RosbagDialogTest : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();
  void controls_follow_input_and_process_state();
  void runs_diagnostic_journey_and_populates_tables();
  void playback_can_pause_resume_and_stop();
  void renders_technology_console_layout();

private:
  static bool create_test_bag(const QString & directory, QString * error = nullptr);
};

void RosbagDialogTest::initTestCase()
{
  QStandardPaths::setTestModeEnabled(true);
  QSettings().clear();
  qputenv("PCD_MEASURE_PROJECT_DIR", QByteArray(PCD_MEASURE_TEST_PROJECT_DIR));
  qputenv("PCD_MEASURE_ROSBAG_PYTHON", QByteArray("/usr/bin/python3"));
}

bool RosbagDialogTest::create_test_bag(const QString & directory, QString * error)
{
  QDir().mkpath(directory);
  const QString database_path = QDir(directory).filePath(QStringLiteral("test_0.db3"));
  const QString program = QStringLiteral(
    "import sqlite3,sys\n"
    "p=sys.argv[1]\n"
    "d=sqlite3.connect(p)\n"
    "d.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT);"
    "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')\n"
    "d.execute(\"INSERT INTO topics VALUES(1,'/cmd_vel','geometry_msgs/msg/Twist','cdr','')\")\n"
    "start=1700000000000000000\n"
    "d.executemany('INSERT INTO messages VALUES(?,?,?,?)',[(i+1,1,start+i*50000000,b'xx') for i in range(40)])\n"
    "d.commit();d.close()\n");
  QProcess process;
  process.start(QStringLiteral("/usr/bin/python3"), {QStringLiteral("-c"), program, database_path});
  if (!process.waitForFinished(5000) || process.exitCode() != 0) {
    if (error) *error = QString::fromUtf8(process.readAllStandardError());
    return false;
  }
  QFile metadata(QDir(directory).filePath(QStringLiteral("metadata.yaml")));
  if (!metadata.open(QIODevice::WriteOnly)) {
    if (error) *error = metadata.errorString();
    return false;
  }
  metadata.write(
    "rosbag2_bagfile_information:\n"
    "  version: 5\n"
    "  storage_identifier: sqlite3\n"
    "  duration:\n"
    "    nanoseconds: 1950000000\n"
    "  starting_time:\n"
    "    nanoseconds_since_epoch: 1700000000000000000\n"
    "  message_count: 40\n"
    "  topics_with_message_count:\n"
    "    - topic_metadata:\n"
    "        name: /cmd_vel\n"
    "        type: geometry_msgs/msg/Twist\n"
    "        serialization_format: cdr\n"
    "        offered_qos_profiles: ''\n"
    "      message_count: 40\n"
    "  compression_format: ''\n"
    "  compression_mode: ''\n"
    "  relative_file_paths:\n"
    "    - test_0.db3\n");
  return true;
}

void RosbagDialogTest::controls_follow_input_and_process_state()
{
  RosbagDiagnosticDialog dialog;
  auto * diagnose = dialog.findChild<QPushButton *>(QStringLiteral("diagnoseRosbagButton"));
  auto * play = dialog.findChild<QPushButton *>(QStringLiteral("rosbagPlayButton"));
  QVERIFY(diagnose);
  QVERIFY(play);
  QVERIFY(!diagnose->isEnabled());
  QVERIFY(!play->isEnabled());

  QTemporaryDir temporary;
  QVERIFY(temporary.isValid());
  const QString bag_path = temporary.filePath(QStringLiteral("sample.bag"));
  QFile bag(bag_path);
  QVERIFY(bag.open(QIODevice::WriteOnly));
  bag.write("placeholder");
  bag.close();
  dialog.set_bag_path(bag_path);
  QVERIFY(diagnose->isEnabled());
  QVERIFY(play->isEnabled());

  dialog.set_bag_path(temporary.filePath(QStringLiteral("missing.bag")));
  QVERIFY(!diagnose->isEnabled());
  QVERIFY(!play->isEnabled());
}

void RosbagDialogTest::runs_diagnostic_journey_and_populates_tables()
{
  QTemporaryDir temporary;
  QVERIFY(temporary.isValid());
  const QString bag_directory = temporary.filePath(QStringLiteral("test_bag"));
  QString error;
  QVERIFY2(create_test_bag(bag_directory, &error), qPrintable(error));

  RosbagDiagnosticDialog dialog(bag_directory);
  dialog.show();
  auto * deep = dialog.findChild<QCheckBox *>(QStringLiteral("rosbagDeepAnalysis"));
  auto * diagnose = dialog.findChild<QPushButton *>(QStringLiteral("diagnoseRosbagButton"));
  auto * progress = dialog.findChild<QProgressBar *>(QStringLiteral("rosbagDiagnosticProgress"));
  auto * topics = dialog.findChild<QTableWidget *>(QStringLiteral("rosbagTopicTable"));
  auto * issues = dialog.findChild<QTableWidget *>(QStringLiteral("rosbagIssueTable"));
  auto * score = dialog.findChild<QLabel *>(QStringLiteral("rosbagScore"));
  QVERIFY(deep);
  QVERIFY(diagnose);
  QVERIFY(progress);
  QVERIFY(topics);
  QVERIFY(issues);
  QVERIFY(score);
  deep->setChecked(false);

  diagnose->click();

  QTRY_COMPARE_WITH_TIMEOUT(topics->rowCount(), 1, 10000);
  QCOMPARE(progress->value(), 100);
  QVERIFY(issues->rowCount() >= 1);
  QVERIFY(score->text().contains(QStringLiteral("/ 100")));
  QVERIFY(diagnose->isEnabled());
  const QString screenshot_path = qEnvironmentVariable("PCD_MEASURE_ROSBAG_RESULT_SCREENSHOT");
  if (!screenshot_path.isEmpty()) {
    QVERIFY2(dialog.grab().save(screenshot_path), qPrintable(screenshot_path));
  }
}

void RosbagDialogTest::playback_can_pause_resume_and_stop()
{
  QTemporaryDir temporary;
  QVERIFY(temporary.isValid());
  const QString bin_directory = temporary.filePath(QStringLiteral("bin"));
  QVERIFY(QDir().mkpath(bin_directory));
  const QString fake_ros2_path = QDir(bin_directory).filePath(QStringLiteral("ros2"));
  QFile fake_ros2(fake_ros2_path);
  QVERIFY(fake_ros2.open(QIODevice::WriteOnly));
  fake_ros2.write(
    "#!/usr/bin/env bash\n"
    "trap 'exit 0' TERM INT\n"
    "while true; do sleep 0.05; done\n");
  fake_ros2.close();
  QVERIFY(fake_ros2.setPermissions(
    QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner));

  const QByteArray original_path = qgetenv("PATH");
  qputenv("PATH", QFile::encodeName(bin_directory) + ':' + original_path);
  const QString bag_path = temporary.filePath(QStringLiteral("playback.db3"));
  QFile bag(bag_path);
  QVERIFY(bag.open(QIODevice::WriteOnly));
  bag.write("placeholder");
  bag.close();

  RosbagDiagnosticDialog dialog(bag_path);
  dialog.show();
  auto * play = dialog.findChild<QPushButton *>(QStringLiteral("rosbagPlayButton"));
  auto * pause = dialog.findChild<QPushButton *>(QStringLiteral("rosbagPauseButton"));
  auto * stop = dialog.findChild<QPushButton *>(QStringLiteral("rosbagStopButton"));
  auto * status = dialog.findChild<QLabel *>(QStringLiteral("diagnosticStatus"));
  QVERIFY(play);
  QVERIFY(pause);
  QVERIFY(stop);
  QVERIFY(status);

  play->click();
  QTRY_COMPARE_WITH_TIMEOUT(status->text(), QStringLiteral("● PLAYING"), 5000);
  QVERIFY(pause->isEnabled());
  QVERIFY(stop->isEnabled());

  pause->click();
  QCOMPARE(pause->text(), QStringLiteral("继续"));
  pause->click();
  QCOMPARE(pause->text(), QStringLiteral("暂停"));

  stop->click();
  QTRY_VERIFY_WITH_TIMEOUT(play->isEnabled(), 5000);
  QVERIFY(!pause->isEnabled());
  qputenv("PATH", original_path);
}

void RosbagDialogTest::renders_technology_console_layout()
{
  RosbagDiagnosticDialog dialog;
  dialog.show();
  QVERIFY(QTest::qWaitForWindowExposed(&dialog));
  QCOMPARE(dialog.objectName(), QStringLiteral("rosbagDiagnosticDialog"));
  QVERIFY(dialog.findChild<QTableWidget *>(QStringLiteral("rosbagTfTable")));
  QVERIFY(dialog.findChild<QTableWidget *>(QStringLiteral("rosbagSensorTable")));
  const QString screenshot_path = qEnvironmentVariable("PCD_MEASURE_ROSBAG_SCREENSHOT");
  if (!screenshot_path.isEmpty()) {
    QVERIFY2(dialog.grab().save(screenshot_path), qPrintable(screenshot_path));
  }
}

QTEST_MAIN(RosbagDialogTest)
#include "rosbag_dialog_test.moc"
