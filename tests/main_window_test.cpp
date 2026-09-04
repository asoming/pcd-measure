#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include <QApplication>
#include <QAction>
#include <QBuffer>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDragEnterEvent>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMimeData>
#include <QMenu>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSettings>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QStatusBar>
#include <QSurfaceFormat>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QTextEdit>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QUrl>
#include <QtEndian>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkObject.h>
#include <vtkRenderer.h>

#include "main_window.h"
#include "environment_setup_panel.h"
#include "olx_capture_dialog.h"
#include "plot_widget.h"

class MainWindowTest : public QObject
{
  Q_OBJECT

private:
  std::unique_ptr<MainWindow> window_;
  QString actual_pcd_;
  QString fixture_directory_ = QStringLiteral(POINT_CLOUD_WORKBENCH_TEST_FIXTURE_DIR);

  bool wait_for_load(const QString & expected_path, int timeout_ms = 60000)
  {
    const QString expected = QFileInfo(expected_path).absoluteFilePath();
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeout_ms) {
      QApplication::processEvents(QEventLoop::AllEvents, 50);
      if (!window_->load_watcher_.isRunning() && window_->current_.ok() &&
        !window_->progress_bar_->isVisible() &&
        !window_->file_name_label_->text().contains(QStringLiteral("加载中")) &&
        window_->pending_project_state_.isEmpty() &&
        QFileInfo(window_->current_.path).absoluteFilePath() == expected)
      {
        return true;
      }
      QTest::qWait(20);
    }
    return false;
  }

  bool load(const QString & path)
  {
    window_->open_path(path);
    return wait_for_load(path);
  }

  static bool close_to(double actual, double expected, double tolerance)
  {
    return std::abs(actual - expected) <= tolerance;
  }

  static void append_u32(QByteArray & bytes, std::uint32_t value)
  {
    const quint32 little = qToLittleEndian<quint32>(value);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(little));
  }

  static void append_float(QByteArray & bytes, float value)
  {
    quint32 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32(bytes, bits);
  }

  static void append_double(QByteArray & bytes, double value)
  {
    quint64 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    const quint64 little = qToLittleEndian<quint64>(bits);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(little));
  }

  static QByteArray sequence_fixture()
  {
    QByteArray bytes;
    for (std::uint32_t frame = 0; frame < 3; ++frame) {
      append_u32(bytes, frame + 20);
      append_double(bytes, 200.0 + frame * 0.1);
      append_u32(bytes, 4);
      for (std::uint32_t point = 0; point < 4; ++point) {
        append_float(bytes, static_cast<float>(frame) + point * 0.2F);
        append_float(bytes, point % 2 == 0 ? 0.0F : 1.0F);
        append_float(bytes, point * 0.1F + frame * 0.05F);
        bytes.append(static_cast<char>(30 + frame * 20));
        bytes.append(static_cast<char>(80 + point * 10));
        bytes.append(static_cast<char>(150 - point * 10));
        bytes.append(static_cast<char>(255));
      }
    }
    return bytes;
  }

  bool drive_modal(
    const std::function<void()> & open,
    const std::function<bool(QDialog *)> & handler)
  {
    bool completed = false;
    QTimer timer;
    timer.setInterval(10);
    connect(&timer, &QTimer::timeout, this, [&]() {
      auto * dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget());
      if (dialog && handler(dialog)) completed = true;
    });
    timer.start();
    open();
    timer.stop();
    return completed;
  }

  bool dismiss_message_after(
    const std::function<void()> & start,
    int timeout_ms = 10000)
  {
    bool dismissed = false;
    QTimer timer;
    timer.setInterval(10);
    connect(&timer, &QTimer::timeout, this, [&dismissed]() {
      auto * message = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
      if (!message) return;
      dismissed = true;
      message->accept();
    });
    timer.start();
    start();
    QElapsedTimer elapsed;
    elapsed.start();
    while (elapsed.elapsed() < timeout_ms &&
      (!dismissed || window_->load_watcher_.isRunning()))
    {
      QApplication::processEvents(QEventLoop::AllEvents, 50);
      QTest::qWait(10);
    }
    timer.stop();
    return dismissed;
  }

private slots:
  void initTestCase()
  {
    QStandardPaths::setTestModeEnabled(true);
    actual_pcd_ = QString::fromLocal8Bit(qgetenv("POINT_CLOUD_WORKBENCH_TEST_PCD"));
    QVERIFY2(QFileInfo::exists(fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd")),
      "XYZ fixture is missing");
    QVERIFY2(QFileInfo::exists(fixture_directory_ + QStringLiteral("/rgba_ascii.pcd")),
      "RGBA fixture is missing");
    window_ = std::make_unique<MainWindow>();
    QCOMPARE(window_->windowTitle(), QStringLiteral("点云工作台"));
    window_->show();
    QTest::qWait(250);
  }

  void fileLoadingAndActualStatistics()
  {
    if (actual_pcd_.isEmpty() || !QFileInfo::exists(actual_pcd_)) {
      QSKIP("POINT_CLOUD_WORKBENCH_TEST_PCD is not set to an existing cloud");
    }
    window_->open_path(actual_pcd_);
    QVERIFY(window_->progress_bar_->isVisible());
    QVERIFY(window_->file_name_label_->text().contains(QFileInfo(actual_pcd_).fileName()));
    QVERIFY(window_->system_status_label_->text().contains(QStringLiteral("LOADING")));
    QVERIFY2(wait_for_load(actual_pcd_), "Timed out loading the reference PCD");

    const CloudMetrics & metrics = window_->current_.metrics;
    QCOMPARE(metrics.header_points, std::size_t(1514634));
    QCOMPARE(metrics.finite_points, std::size_t(1514634));
    QCOMPARE(metrics.invalid_points, std::size_t(0));
    QVERIFY(metrics.has_rgb);
    QVERIFY(!metrics.has_rgba);
    QVERIFY(metrics.non_black_points > std::size_t(1510000));
    QVERIFY(close_to(metrics.oriented.major_size, 19.3464, 0.01));
    QVERIFY(close_to(metrics.oriented.minor_size, 11.3436, 0.01));
    QVERIFY(close_to(metrics.oriented.height, 3.9527, 0.01));
    QVERIFY(close_to(metrics.oriented.diagonal_3d, 22.7724, 0.01));
    QVERIFY(close_to(metrics.lowest_point[2], metrics.raw_bounds.min_z, 1e-5));
    QVERIFY(close_to(metrics.highest_point[2], metrics.raw_bounds.max_z, 1e-5));
    QVERIFY(!window_->progress_bar_->isVisible());
    QCOMPARE(window_->color_mode_combo_->currentIndex(), 0);
    QVERIFY(window_->lowest_point_label_->text().contains(QStringLiteral("Z")));
    QVERIFY(window_->highest_point_label_->text().contains(QStringLiteral("Z")));
    QVERIFY(window_->raw_height_label_->text().contains(QStringLiteral("m")));
    QVERIFY(window_->outlier_ratio_label_->text().contains(QStringLiteral("0.5%")));
    QVERIFY(window_->system_status_label_->text().contains(QStringLiteral("READY")));

    const QStringList recent = QSettings().value(QStringLiteral("recentCloudFiles")).toStringList();
    QVERIFY(recent.contains(QFileInfo(actual_pcd_).absoluteFilePath()));
    const QString screenshot_path = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_ACTUAL_SCREENSHOT"));
    if (!screenshot_path.isEmpty()) {
      QVERIFY2(window_->grab().save(screenshot_path, "PNG"), qPrintable(screenshot_path));
      QVERIFY(QFileInfo(screenshot_path).size() > 1000);
    }

    const QString annotation_path = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_ACTUAL_ANNOTATION_SCREENSHOT"));
    if (!annotation_path.isEmpty()) {
      window_->clear_measurements();
      const float cx = static_cast<float>(metrics.oriented.center_x);
      const float cy = static_cast<float>(metrics.oriented.center_y);
      const float cz = static_cast<float>((metrics.oriented.z_min + metrics.oriented.z_max) * 0.5);
      const std::vector<std::pair<MeasurementKind, std::vector<pcl::PointXYZ>>> examples{
        {MeasurementKind::Point, {pcl::PointXYZ(cx - 6.0F, cy + 3.0F, cz)}},
        {MeasurementKind::Segment, {pcl::PointXYZ(cx - 5.0F, cy - 3.0F, cz),
          pcl::PointXYZ(cx - 1.0F, cy - 3.0F, cz + 0.4F)}},
        {MeasurementKind::Polyline, {pcl::PointXYZ(cx - 5.0F, cy + 1.0F, cz),
          pcl::PointXYZ(cx - 2.0F, cy + 2.0F, cz + 0.2F),
          pcl::PointXYZ(cx + 1.0F, cy + 1.0F, cz + 0.4F)}},
        {MeasurementKind::Angle, {pcl::PointXYZ(cx + 2.0F, cy - 3.0F, cz),
          pcl::PointXYZ(cx + 2.0F, cy - 1.0F, cz),
          pcl::PointXYZ(cx + 4.0F, cy - 1.0F, cz)}},
        {MeasurementKind::Area, {pcl::PointXYZ(cx + 2.0F, cy + 1.0F, cz),
          pcl::PointXYZ(cx + 5.0F, cy + 1.0F, cz),
          pcl::PointXYZ(cx + 5.0F, cy + 3.0F, cz),
          pcl::PointXYZ(cx + 2.0F, cy + 3.0F, cz)}},
        {MeasurementKind::Orthogonal, {pcl::PointXYZ(cx - 1.0F, cy - 1.0F, cz - 0.5F),
          pcl::PointXYZ(cx + 1.0F, cy + 1.0F, cz + 1.0F)}},
        {MeasurementKind::PointToPlane, {pcl::PointXYZ(cx, cy, cz + 1.5F),
          pcl::PointXYZ(cx - 1.0F, cy - 1.0F, cz),
          pcl::PointXYZ(cx + 1.0F, cy - 1.0F, cz),
          pcl::PointXYZ(cx, cy + 1.0F, cz)}},
        {MeasurementKind::Circle, {pcl::PointXYZ(cx + 1.0F, cy + 4.0F, cz),
          pcl::PointXYZ(cx, cy + 5.0F, cz), pcl::PointXYZ(cx - 1.0F, cy + 4.0F, cz)}}};
      int id = 1;
      for (const auto & example : examples) {
        const MeasurementRecord record = calculate_measurement(example.first, example.second, id++);
        QVERIFY2(record.valid, qPrintable(record.error));
        window_->commit_measurement(record);
      }
      QCOMPARE(window_->measurements_.size(), std::size_t(8));
      QVERIFY2(window_->grab().save(annotation_path, "PNG"), qPrintable(annotation_path));
      QVERIFY(QFileInfo(annotation_path).size() > 1000);
      window_->clear_measurements();
    }
  }

  void rgbaNoColorAndInvalidInputs()
  {
    const QString rgba = fixture_directory_ + QStringLiteral("/rgba_ascii.pcd");
    QVERIFY2(load(rgba), "RGBA fixture failed to load");
    QVERIFY(window_->current_.metrics.has_rgba);
    QVERIFY(!window_->current_.metrics.has_rgb);
    QCOMPARE(window_->current_.metrics.non_black_points, std::size_t(12));
    QCOMPARE(window_->color_mode_combo_->currentIndex(), 0);

    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    QVERIFY(!window_->current_.metrics.has_rgb);
    QVERIFY(!window_->current_.metrics.has_rgba);
    QCOMPARE(window_->current_.metrics.non_black_points, std::size_t(0));
    QCOMPARE(window_->color_mode_combo_->currentIndex(), 1);
    QVERIFY(!(window_->color_mode_combo_->model()->flags(
      window_->color_mode_combo_->model()->index(0, 0)) & Qt::ItemIsEnabled));
    QVERIFY(!(window_->color_mode_combo_->model()->flags(
      window_->color_mode_combo_->model()->index(3, 0)) & Qt::ItemIsEnabled));

    const QString invalid = fixture_directory_ + QStringLiteral("/invalid_points_ascii.pcd");
    QVERIFY2(load(invalid), "Invalid-point fixture failed to load");
    QCOMPARE(window_->current_.metrics.header_points, std::size_t(14));
    QCOMPARE(window_->current_.metrics.finite_points, std::size_t(12));
    QCOMPARE(window_->current_.metrics.invalid_points, std::size_t(2));
  }

  void dragAndDropLoading()
  {
    const QString rgba = fixture_directory_ + QStringLiteral("/rgba_ascii.pcd");
    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(rgba)});
    QDragEnterEvent drag_event(
      QPoint(20, 20), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drag_event);
    QVERIFY(drag_event.isAccepted());

    QDropEvent drop_event(
      QPointF(20.0, 20.0), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drop_event);
    QVERIFY(drop_event.isAccepted());
    QVERIFY2(wait_for_load(rgba), "Dropped PCD did not load");
    QCOMPARE(QFileInfo(window_->current_.path).fileName(), QStringLiteral("rgba_ascii.pcd"));
  }

  void olxTimelinePlaybackAndCameraPreview()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString olx_path = directory.filePath(QStringLiteral("MT sequence.olx"));
    QFile olx(olx_path);
    QVERIFY(olx.open(QIODevice::WriteOnly));
    const QByteArray sequence = sequence_fixture();
    QCOMPARE(olx.write(sequence), static_cast<qint64>(sequence.size()));
    olx.close();

    QImage camera_image(8, 6, QImage::Format_RGB32);
    camera_image.fill(QColor(QStringLiteral("#21D4D1")));
    QByteArray png;
    QBuffer png_buffer(&png);
    QVERIFY(png_buffer.open(QIODevice::WriteOnly));
    QVERIFY(camera_image.save(&png_buffer, "PNG"));
    QByteArray image_stream;
    for (std::uint32_t frame = 0; frame < 3; ++frame) {
      append_u32(image_stream, frame + 30);
      append_double(image_stream, 200.0 + frame * 0.1);
      append_u32(image_stream, static_cast<std::uint32_t>(png.size()));
      image_stream.append(png);
    }
    QFile image_file(directory.filePath(QStringLiteral("OdinImage.bin")));
    QVERIFY(image_file.open(QIODevice::WriteOnly));
    QCOMPARE(image_file.write(image_stream), static_cast<qint64>(image_stream.size()));
    image_file.close();

    QVERIFY2(load(olx_path), "OLX sequence fixture failed to load");
    QCOMPARE(window_->current_.source_kind, CloudSourceKind::OdinOlx);
    QCOMPARE(window_->current_.frames.size(), std::size_t(3));
    QCOMPARE(window_->current_.metrics.finite_points, std::size_t(12));
    QVERIFY(window_->sequence_strip_->isVisible());
    QCOMPARE(window_->sequence_slider_->maximum(), 2);
    QCOMPARE(window_->sequence_slider_->value(), 2);
    QCOMPARE(window_->sequence_display_cloud_->size(), std::size_t(12));
    QVERIFY(window_->source_badge_label_->text().contains(QStringLiteral("OLX")));
    QVERIFY(window_->color_mode_combo_->model()->flags(
      window_->color_mode_combo_->model()->index(3, 0)) & Qt::ItemIsEnabled);
    QVERIFY(window_->camera_preview_check_->isEnabled());
    QVERIFY(window_->camera_preview_frame_->isVisible());
    QVERIFY(window_->camera_preview_label_->text().isEmpty());

    window_->sequence_render_combo_->setCurrentIndex(1);
    window_->sequence_slider_->setValue(1);
    QCOMPARE(window_->sequence_display_cloud_->size(), std::size_t(4));
    QVERIFY(window_->sequence_frame_label_->text().contains(QStringLiteral("2/3")));

    window_->color_mode_combo_->setCurrentIndex(3);
    QCOMPARE(window_->sequence_display_cloud_->size(), std::size_t(4));
    QVERIFY(window_->sequence_display_cloud_->front().r != std::uint8_t(50));

    window_->sequence_render_combo_->setCurrentIndex(0);
    window_->sequence_infinite_check_->setChecked(false);
    window_->sequence_max_frames_spin_->setValue(2);
    window_->sequence_slider_->setValue(2);
    QCOMPARE(window_->sequence_display_cloud_->size(), std::size_t(8));

    window_->sequence_slider_->setValue(0);
    window_->sequence_tick();
    QCOMPARE(window_->sequence_slider_->value(), 1);
    window_->stop_sequence_playback();

    const QString olx_screenshot = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_OLX_SCREENSHOT"));
    if (!olx_screenshot.isEmpty()) {
      window_->resize(1540, 920);
      QApplication::processEvents();
      QVERIFY2(window_->grab().save(olx_screenshot, "PNG"), qPrintable(olx_screenshot));
      QVERIFY(QFileInfo(olx_screenshot).size() > 1000);
    }

    const QJsonObject project = window_->build_project_state(
      directory.filePath(QStringLiteral("sequence.pcworkbench")));
    const QJsonObject view = project.value(QStringLiteral("view")).toObject();
    QCOMPARE(view.value(QStringLiteral("sequence_frame")).toInt(), 1);
    QCOMPARE(view.value(QStringLiteral("sequence_render_mode")).toInt(), 0);

    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(olx_path)});
    QDragEnterEvent drag_event(
      QPoint(20, 20), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drag_event);
    QVERIFY(drag_event.isAccepted());
  }

  void embeddedWorkspacesAndDragDrop()
  {
    int cloud_commands = 0;
    for (QAction * toolbar_action : window_->cloud_toolbar_->actions()) {
      if (!toolbar_action->isSeparator()) ++cloud_commands;
    }
    QCOMPARE(cloud_commands, 6);
    auto * capture_action = window_->findChild<QAction *>(QStringLiteral("olxCaptureAction"));
    QVERIFY(capture_action);
    QVERIFY(capture_action->toolTip().contains(QStringLiteral("OLX")));
    capture_action->trigger();
    QTRY_VERIFY(window_->findChild<QDialog *>(QStringLiteral("olxCaptureDialog")));
    auto * capture_dialog = window_->findChild<QDialog *>(QStringLiteral("olxCaptureDialog"));
    QCOMPARE(window_->workspace_stack_->currentIndex(), 1);
    QVERIFY(!capture_dialog->isWindow());
    QVERIFY(window_->capture_workspace_button_->isChecked());
    QVERIFY(!window_->cloud_toolbar_->isVisible());
    QVERIFY(capture_dialog->findChild<QWidget *>(QStringLiteral("olxLivePreview")));
    auto * scan_mode = capture_dialog->findChild<QComboBox *>(QStringLiteral("olxScanMode"));
    auto * preview_mode = capture_dialog->findChild<QComboBox *>(QStringLiteral("olxPreviewMode"));
    QVERIFY(scan_mode);
    QVERIFY(preview_mode);
    QCOMPARE(preview_mode->currentData().toString(), QStringLiteral("cumulative"));
    scan_mode->setCurrentIndex(2);
    QCOMPARE(preview_mode->currentData().toString(), QStringLiteral("frame"));
    scan_mode->setCurrentIndex(0);
    QCOMPARE(preview_mode->currentData().toString(), QStringLiteral("cumulative"));
    QTest::keyClick(capture_dialog, Qt::Key_Escape);
    QVERIFY(capture_dialog->isVisible());
    QCOMPARE(window_->workspace_stack_->currentIndex(), 1);
    auto * preview_status = capture_dialog->findChild<QLabel *>(QStringLiteral("olxPreviewStatus"));
    QVERIFY(preview_status);
    QByteArray preview("PCPV0001", 8);
    append_u32(preview, 42);
    append_double(preview, 12.5);
    constexpr int preview_points = 6000;
    append_u32(preview, preview_points);
    for (int point = 0; point < preview_points; ++point) {
      const int surface = point % 3;
      const int sample = point / 3;
      const float u = static_cast<float>(sample % 80);
      const float v = static_cast<float>((sample / 80) % 25);
      const float x = surface == 2 ? 0.0F : u * 0.10F;
      const float y = surface == 1 ? 0.0F : (surface == 2 ? u * 0.05F : v * 0.15F);
      const float z = surface == 0 ? 0.03F * std::sin(u * 0.2F) : v * 0.08F;
      append_float(preview, x);
      append_float(preview, y);
      append_float(preview, z);
      preview.append(static_cast<char>(surface == 0 ? 70 : 190));
      preview.append(static_cast<char>(surface == 1 ? 170 : 110));
      preview.append(static_cast<char>(surface == 2 ? 225 : 155));
      preview.append(static_cast<char>(255));
    }
    QFile preview_file(window_->capture_panel_->preview_file_path_);
    QVERIFY(preview_file.open(QIODevice::WriteOnly));
    QCOMPARE(preview_file.write(preview), static_cast<qint64>(preview.size()));
    preview_file.close();
    window_->capture_panel_->refresh_live_preview();
    QVERIFY(preview_status->text().contains(QStringLiteral("F42")));
    QVERIFY(preview_status->text().contains(QStringLiteral("MAP 6000 PTS")));
    const QString valid_preview_status = preview_status->text();
    QVERIFY(preview_file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    preview_file.write("broken preview");
    preview_file.close();
    window_->capture_panel_->refresh_live_preview();
    QCOMPARE(preview_status->text(), valid_preview_status);
    auto * workspace = capture_dialog->findChild<QLineEdit *>(QStringLiteral("olxWorkspace"));
    auto * topic_status = capture_dialog->findChild<QLabel *>(QStringLiteral("olxTopicStatus"));
    auto * record_button = capture_dialog->findChild<QPushButton *>(QStringLiteral("olxRecordButton"));
    auto * stop_button = capture_dialog->findChild<QPushButton *>(QStringLiteral("olxStopButton"));
    QVERIFY(workspace);
    QVERIFY(topic_status);
    QVERIFY(record_button);
    QVERIFY(stop_button);
    QVERIFY(!stop_button->isEnabled());
    QTRY_VERIFY_WITH_TIMEOUT(!topic_status->text().contains(QStringLiteral("检查中")), 6000);
    const QString capture_screenshot = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_CAPTURE_SCREENSHOT"));
    if (!capture_screenshot.isEmpty()) {
      auto * recording_status = capture_dialog->findChild<QLabel *>(
        QStringLiteral("olxRecordingStatus"));
      auto * counters = capture_dialog->findChild<QLabel *>(QStringLiteral("olxCounters"));
      QVERIFY(recording_status);
      QVERIFY(counters);
      recording_status->setText(QStringLiteral("● MAPPING + RECORDING · /odin1/cloud_slam"));
      recording_status->setStyleSheet(QStringLiteral(
        "color:#FFD38A; background:#302817; border:1px solid #8A6933; border-radius:5px; "
        "padding:7px 10px; font-family:'DejaVu Sans Mono'; font-weight:700;"));
      counters->setText(QStringLiteral(
        "帧 42 · 写盘点 816,420 · 4.2 s\n位姿 41 · 图像 0\n"
        "地图 6,000 · 帧点 1,938 · 2.0 cm"));
      preview_status->setText(QStringLiteral("BUILDING · F42 · MAP 6,000 PTS · 9.8 Hz"));
      record_button->setEnabled(false);
      stop_button->setEnabled(true);
      QApplication::processEvents();
      QVERIFY2(window_->grab().save(capture_screenshot, "PNG"),
        qPrintable(capture_screenshot));
      QVERIFY(QFileInfo(capture_screenshot).size() > 1000);
    }
    QTemporaryDir invalid_workspace;
    QVERIFY(invalid_workspace.isValid());
    workspace->setText(invalid_workspace.path());
    QVERIFY(QMetaObject::invokeMethod(capture_dialog, "refresh_status", Qt::DirectConnection));
    QVERIFY(topic_status->text().contains(QStringLiteral("未构建")));
    QVERIFY(!record_button->isEnabled());

    auto * action = window_->findChild<QAction *>(QStringLiteral("rosbagStudioAction"));
    QVERIFY(action);
    QVERIFY(action->toolTip().contains(QStringLiteral("ROS1/ROS2")));

    QTemporaryDir temporary;
    QVERIFY(temporary.isValid());
    const QString bag_path = temporary.filePath(QStringLiteral("回放 bag with spaces.bag"));
    QFile bag(bag_path);
    QVERIFY(bag.open(QIODevice::WriteOnly));
    bag.write("placeholder");
    bag.close();

    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(bag_path)});
    QDragEnterEvent drag_event(
      QPoint(20, 20), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drag_event);
    QVERIFY(drag_event.isAccepted());
    QDropEvent drop_event(
      QPointF(20.0, 20.0), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drop_event);
    QVERIFY(drop_event.isAccepted());

    QTRY_VERIFY(window_->findChild<QDialog *>(QStringLiteral("rosbagDiagnosticDialog")));
    auto * dialog = window_->findChild<QDialog *>(QStringLiteral("rosbagDiagnosticDialog"));
    QCOMPARE(window_->workspace_stack_->currentIndex(), 2);
    QVERIFY(!dialog->isWindow());
    QVERIFY(window_->rosbag_workspace_button_->isChecked());
    QVERIFY(!window_->cloud_toolbar_->isVisible());
    QTest::keyClick(dialog, Qt::Key_Escape);
    QVERIFY(dialog->isVisible());
    QCOMPARE(window_->workspace_stack_->currentIndex(), 2);
    auto * path_edit = dialog->findChild<QLineEdit *>(QStringLiteral("rosbagPath"));
    QVERIFY(path_edit);
    QCOMPARE(path_edit->text(), QFileInfo(bag_path).absoluteFilePath());
    const QString embedded_rosbag_screenshot = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_EMBEDDED_ROSBAG_SCREENSHOT"));
    if (!embedded_rosbag_screenshot.isEmpty()) {
      QApplication::processEvents();
      QVERIFY2(window_->grab().save(embedded_rosbag_screenshot, "PNG"),
        qPrintable(embedded_rosbag_screenshot));
      QVERIFY(QFileInfo(embedded_rosbag_screenshot).size() > 1000);
    }
    window_->cloud_workspace_button_->click();
    QCOMPARE(window_->workspace_stack_->currentIndex(), 0);
    QVERIFY(window_->cloud_toolbar_->isVisible());
  }

  void environmentWorkspaceAndSudoPasswordSafety()
  {
    QVERIFY(window_->environment_panel_);
    QVERIFY(window_->environment_workspace_button_);
    window_->environment_workspace_button_->click();
    QCOMPARE(window_->workspace_stack_->currentIndex(), 3);
    QVERIFY(window_->environment_workspace_button_->isChecked());
    QVERIFY(!window_->cloud_toolbar_->isVisible());
    auto * panel = window_->environment_panel_;
    QVERIFY(panel->isVisible());
    QCOMPARE(panel->environment_table_->rowCount(), 6);
    QCOMPARE(panel->environment_table_->columnCount(), 3);
    QVERIFY(panel->check_button_->text().contains(QStringLiteral("检测")));
    QVERIFY(panel->install_button_->text().contains(QStringLiteral("一键安装")));
    QTRY_VERIFY_WITH_TIMEOUT(panel->check_complete_, 8000);
    QCOMPARE(panel->process_.state(), QProcess::NotRunning);
    QVERIFY(panel->system_ready_);
    QCOMPARE(panel->environment_table_->item(0, 1)->text(), QStringLiteral("READY"));
    QVERIFY(panel->log_edit_->toPlainText().contains(QStringLiteral("系统编译依赖完整")));

    const QString screenshot = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_ENVIRONMENT_SCREENSHOT"));
    if (!screenshot.isEmpty()) {
      window_->resize(1540, 920);
      QApplication::processEvents();
      QVERIFY2(window_->grab().save(screenshot, "PNG"), qPrintable(screenshot));
      QVERIFY(QFileInfo(screenshot).size() > 1000);
    }

    QTemporaryDir temporary;
    QVERIFY(temporary.isValid());
    const QString fake_sudo = temporary.filePath(QStringLiteral("fake-sudo.sh"));
    QFile fake(fake_sudo);
    QVERIFY(fake.open(QIODevice::WriteOnly));
    const QByteArray fake_script(
      "#!/usr/bin/env bash\n"
      "if [[ \"${1:-}\" == '-n' ]]; then\n"
      "  echo 'fake sudo used cached authentication'\n"
      "  exit 0\n"
      "fi\n"
      "IFS= read -r credential\n"
      "if [[ -n \"${credential}\" ]]; then\n"
      "  echo 'fake sudo received stdin credential'\n"
      "fi\n"
      "if [[ \"${credential}\" == 'valid-test-password' ]]; then\n"
      "  exit 0\n"
      "fi\n"
      "echo 'fake sudo rejected authentication'\n"
      "exit 23\n");
    QCOMPARE(fake.write(fake_script), static_cast<qint64>(fake_script.size()));
    fake.close();
    QVERIFY(QFile::setPermissions(fake_sudo, QFileDevice::ReadOwner | QFileDevice::WriteOwner |
      QFileDevice::ExeOwner));

    const auto prepare_missing_system_state = [panel, &fake_sudo]() {
      panel->missing_system_packages_ = QStringList{
        QStringLiteral("fake-package-for-test")};
      panel->system_ready_ = false;
      panel->build_ready_ = true;
      panel->rosbag_ready_ = true;
      panel->map_converter_ready_ = true;
      panel->desktop_ready_ = true;
      panel->check_complete_ = true;
      panel->installation_running_ = false;
      panel->rosbag_check_->setChecked(false);
      panel->map_converter_check_->setChecked(false);
      panel->desktop_check_->setChecked(false);
      panel->sudo_program_ = fake_sudo;
      panel->force_password_prompt_for_test_ = true;
      panel->update_table();
    };

    prepare_missing_system_state();
    panel->sudo_program_ = temporary.filePath(QStringLiteral("missing-sudo"));
    panel->install_missing_environment();
    QVERIFY(!panel->installation_running_);
    QCOMPARE(panel->process_.state(), QProcess::NotRunning);
    QVERIFY(panel->overall_status_label_->text().contains(QStringLiteral("缺少 sudo")));

    prepare_missing_system_state();
    bool cancel_prompt_seen = false;
    QVERIFY(drive_modal([panel]() { panel->install_missing_environment(); },
      [&cancel_prompt_seen](QDialog * dialog) {
        auto * input = qobject_cast<QInputDialog *>(dialog);
        if (!input) return false;
        cancel_prompt_seen = true;
        input->reject();
        return true;
      }));
    QVERIFY(cancel_prompt_seen);
    QVERIFY(!panel->installation_running_);
    QCOMPARE(panel->process_.state(), QProcess::NotRunning);
    QVERIFY(panel->overall_status_label_->text().contains(QStringLiteral("CANCELLED")));

    prepare_missing_system_state();
    bool empty_prompt_seen = false;
    QVERIFY(drive_modal([panel]() { panel->install_missing_environment(); },
      [&empty_prompt_seen](QDialog * dialog) {
        auto * input = qobject_cast<QInputDialog *>(dialog);
        if (!input) return false;
        empty_prompt_seen = true;
        input->setTextValue(QString());
        input->accept();
        return true;
      }));
    QVERIFY(empty_prompt_seen);
    QVERIFY(!panel->installation_running_);
    QCOMPARE(panel->process_.state(), QProcess::NotRunning);
    QVERIFY(panel->overall_status_label_->text().contains(QStringLiteral("密码为空")));

    panel->log_edit_->clear();
    prepare_missing_system_state();
    const QString secret = QStringLiteral("do-not-log-this-password");
    bool password_prompt_seen = false;
    bool password_mode = false;
    QVERIFY(drive_modal([panel]() { panel->install_missing_environment(); },
      [&password_prompt_seen, &password_mode, &secret](QDialog * dialog) {
        auto * input = qobject_cast<QInputDialog *>(dialog);
        if (!input) return false;
        auto * editor = input->findChild<QLineEdit *>();
        if (!editor) return false;
        password_prompt_seen = true;
        password_mode = editor->echoMode() == QLineEdit::Password;
        editor->setText(secret);
        input->accept();
        return true;
      }));
    QVERIFY(password_prompt_seen);
    QVERIFY(password_mode);
    QTRY_VERIFY_WITH_TIMEOUT(!panel->installation_running_, 5000);
    QCOMPARE(panel->process_.state(), QProcess::NotRunning);
    QVERIFY(panel->overall_status_label_->text().contains(QStringLiteral("ERROR")));
    QVERIFY(panel->log_edit_->toPlainText().contains(
      QStringLiteral("fake sudo received stdin credential")));
    QVERIFY(!panel->log_edit_->toPlainText().contains(secret));
    QVERIFY(!panel->process_.arguments().join(QLatin1Char(' ')).contains(secret));

    panel->log_edit_->clear();
    prepare_missing_system_state();
    const QString valid_secret = QStringLiteral("valid-test-password");
    QVERIFY(drive_modal([panel]() { panel->install_missing_environment(); },
      [&valid_secret](QDialog * dialog) {
        auto * input = qobject_cast<QInputDialog *>(dialog);
        if (!input) return false;
        auto * editor = input->findChild<QLineEdit *>();
        if (!editor) return false;
        editor->setText(valid_secret);
        input->accept();
        return true;
      }));
    QTRY_VERIFY_WITH_TIMEOUT(panel->check_complete_ && !panel->installation_running_, 8000);
    QVERIFY(panel->system_ready_);
    QVERIFY(panel->log_edit_->toPlainText().contains(
      QStringLiteral("fake sudo received stdin credential")));
    QVERIFY(!panel->log_edit_->toPlainText().contains(valid_secret));
    QVERIFY(!panel->process_.arguments().join(QLatin1Char(' ')).contains(valid_secret));

    panel->log_edit_->clear();
    prepare_missing_system_state();
    panel->force_password_prompt_for_test_ = false;
    panel->install_missing_environment();
    QTRY_VERIFY_WITH_TIMEOUT(panel->check_complete_ && !panel->installation_running_, 8000);
    QVERIFY(panel->system_ready_);
    QVERIFY(panel->log_edit_->toPlainText().contains(
      QStringLiteral("fake sudo used cached authentication")));

    panel->sudo_program_ = QStringLiteral("/usr/bin/sudo");
    panel->force_password_prompt_for_test_ = false;
    panel->rosbag_check_->setChecked(true);
    panel->map_converter_check_->setChecked(true);
    panel->desktop_check_->setChecked(true);
    panel->check_environment();
    QTRY_VERIFY_WITH_TIMEOUT(panel->check_complete_, 8000);
    window_->cloud_workspace_button_->click();
    QCOMPARE(window_->workspace_stack_->currentIndex(), 0);
    QVERIFY(window_->cloud_toolbar_->isVisible());
  }

  void recentMenusAndProjectDragDrop()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    const QString rgba = fixture_directory_ + QStringLiteral("/rgba_ascii.pcd");
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString missing = directory.filePath(QStringLiteral("missing.pcd"));
    QSettings settings;
    settings.setValue(QStringLiteral("recentCloudFiles"), QStringList{missing, rgba, rgba});
    window_->rebuild_recent_menu();
    QCOMPARE(settings.value(QStringLiteral("recentCloudFiles")).toStringList(), QStringList{rgba});
    QCOMPARE(window_->recent_menu_->actions().size(), 1);

    const QString project_path = directory.filePath(QStringLiteral("可拖放工程.odinpcd"));
    QJsonObject root{{QStringLiteral("format"), QStringLiteral("pcd-measure-project")},
      {QStringLiteral("version"), 7}, {QStringLiteral("pcd_path"), xyz}};
    QFile project(project_path);
    QVERIFY(project.open(QIODevice::WriteOnly));
    const QByteArray payload = QJsonDocument(root).toJson();
    QCOMPARE(project.write(payload), static_cast<qint64>(payload.size()));
    project.close();
    settings.setValue(QStringLiteral("recentProjectFiles"),
      QStringList{directory.filePath(QStringLiteral("missing.pcworkbench")), project_path});
    window_->rebuild_recent_project_menu();
    QCOMPARE(settings.value(QStringLiteral("recentProjectFiles")).toStringList(),
      QStringList{project_path});
    QCOMPARE(window_->recent_project_menu_->actions().size(), 1);

    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(project_path)});
    QDragEnterEvent drag_event(
      QPoint(20, 20), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drag_event);
    QVERIFY(drag_event.isAccepted());
    QDropEvent drop_event(
      QPointF(20.0, 20.0), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(window_.get(), &drop_event);
    QVERIFY(drop_event.isAccepted());
    QVERIFY2(wait_for_load(xyz), "Dropped legacy-extension project did not load");
    QCOMPARE(window_->pending_project_path_, QFileInfo(project_path).absoluteFilePath());
  }

  void measurementPolylineAndUnits()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();

    const pcl::PointXYZ a(0.0F, 0.0F, 0.0F);
    const pcl::PointXYZ b(3.0F, 4.0F, 12.0F);
    const MeasurementRecord known = window_->make_measurement_record(
      MeasurementKind::Segment, {a, b}, 1);
    QVERIFY(close_to(known.horizontal, 5.0, 1e-9));
    QVERIFY(close_to(known.distance_3d, 13.0, 1e-9));
    QVERIFY(close_to(known.dz, 12.0, 1e-9));
    QVERIFY(close_to(known.slope_percent, 240.0, 1e-9));

    window_->interaction_mode_combo_->setCurrentIndex(0);
    window_->accept_picked_point(a);
    QVERIFY(window_->pending_point_.has_value());
    window_->accept_picked_point(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QCOMPARE(window_->measurement_table_->rowCount(), 1);
    QVERIFY(window_->distance_3d_label_->text().contains(QStringLiteral("1.0000 m")));
    window_->undo_measurement();
    QVERIFY(window_->measurements_.empty());

    window_->interaction_mode_combo_->setCurrentIndex(1);
    window_->accept_polyline_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_polyline_point(pcl::PointXYZ(3.0F, 0.0F, 0.0F));
    window_->accept_polyline_point(pcl::PointXYZ(3.0F, 4.0F, 0.0F));
    QVERIFY(window_->finish_polyline_button_->isEnabled());
    window_->finish_polyline();
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QCOMPARE(window_->measurements_.front().kind, MeasurementKind::Polyline);
    QCOMPARE(window_->measurements_.front().vertices.size(), std::size_t(3));
    QVERIFY(close_to(window_->measurements_.front().distance_3d, 7.0, 1e-9));

    window_->unit_combo_->setCurrentIndex(1);
    QVERIFY(window_->measurement_table_->item(0, 3)->text().contains(QStringLiteral("cm")));
    QVERIFY(window_->point_a_label_->text().contains(QStringLiteral("cm")));
    window_->unit_combo_->setCurrentIndex(2);
    QVERIFY(window_->delta_label_->text().contains(QStringLiteral("mm")));
    window_->unit_combo_->setCurrentIndex(0);
    window_->clear_measurements();
    QVERIFY(window_->measurements_.empty());
  }

  void advancedMeasurementsHistoryAndReport()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    window_->undo_history_.clear();
    window_->redo_history_.clear();

    window_->interaction_mode_combo_->setCurrentIndex(3);
    window_->accept_generic_measurement_point(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Point);

    window_->interaction_mode_combo_->setCurrentIndex(4);
    window_->accept_generic_measurement_point(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 1.0F, 0.0F));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Angle);
    QVERIFY(close_to(window_->measurements_.back().angle_degrees, 90.0, 1e-6));

    window_->interaction_mode_combo_->setCurrentIndex(5);
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(2.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(2.0F, 3.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 3.0F, 0.0F));
    QVERIFY(window_->finish_polyline_button_->isEnabled());
    window_->finish_polyline();
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Area);
    QVERIFY(close_to(window_->measurements_.back().area_planar, 6.0, 1e-6));

    window_->interaction_mode_combo_->setCurrentIndex(6);
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(3.0F, 4.0F, 12.0F));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Orthogonal);

    window_->interaction_mode_combo_->setCurrentIndex(7);
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.25F, 0.25F, 2.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 1.0F, 0.0F));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::PointToPlane);
    QVERIFY(close_to(window_->measurements_.back().distance_3d, 2.0, 1e-6));

    window_->interaction_mode_combo_->setCurrentIndex(8);
    window_->accept_generic_measurement_point(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(0.0F, 1.0F, 0.0F));
    window_->accept_generic_measurement_point(pcl::PointXYZ(-1.0F, 0.0F, 0.0F));
    QCOMPARE(window_->measurements_.size(), std::size_t(6));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Circle);
    QVERIFY(close_to(window_->measurements_.back().diameter, 2.0, 1e-6));
    QCOMPARE(window_->measurement_table_->rowCount(), 6);
    for (const MeasurementRecord & record : window_->measurements_) {
      const auto shapes = window_->measurement_shape_ids_.find(record.id);
      QVERIFY(shapes != window_->measurement_shape_ids_.end());
      QVERIFY(!shapes->second.empty());
      for (const std::string & shape_id : shapes->second) {
        QVERIFY2(window_->viewer_->contains(shape_id), shape_id.c_str());
      }
    }
    window_->measurement_labels_check_->setChecked(false);
    for (const MeasurementRecord & record : window_->measurements_) {
      const std::string text_id = QStringLiteral("measurement_%1_text").arg(record.id).toStdString();
      QVERIFY(!window_->viewer_->contains(text_id));
    }
    window_->measurement_labels_check_->setChecked(true);
    for (const MeasurementRecord & record : window_->measurements_) {
      const std::string text_id = QStringLiteral("measurement_%1_text").arg(record.id).toStdString();
      QVERIFY(window_->viewer_->contains(text_id));
    }

    window_->undo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(5));
    window_->redo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(6));
    QCOMPARE(window_->measurements_.back().kind, MeasurementKind::Circle);

    window_->measurement_table_->selectRow(1);
    const int deleted_id = window_->measurements_[1].id;
    window_->delete_selected_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(5));
    window_->undo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(6));
    QCOMPARE(window_->measurements_[1].id, deleted_id);

    window_->measurement_table_->selectRow(0);
    bool dialog_handled = false;
    QTimer::singleShot(50, window_.get(), [&dialog_handled]() {
      auto * dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget());
      if (!dialog) {
        return;
      }
      dialog->findChild<QLineEdit *>(QStringLiteral("measurementNameEdit"))
        ->setText(QStringLiteral("入口,控制点"));
      dialog->findChild<QLineEdit *>(QStringLiteral("measurementGroupEdit"))
        ->setText(QStringLiteral("一层控制点"));
      dialog->findChild<QLineEdit *>(QStringLiteral("measurementColorEdit"))
        ->setText(QStringLiteral("#00CCFF"));
      dialog->findChild<QTextEdit *>(QStringLiteral("measurementNoteEdit"))
        ->setPlainText(QStringLiteral("引号\"与逗号,转义"));
      dialog->findChild<QTextEdit *>(QStringLiteral("measurementPointsEdit"))
        ->setPlainText(QStringLiteral("2.0, 3.0, 4.0"));
      dialog_handled = true;
      dialog->findChild<QDialogButtonBox *>(QStringLiteral("measurementEditButtons"))
        ->button(QDialogButtonBox::Save)->click();
    });
    window_->edit_selected_measurement();
    QVERIFY(dialog_handled);
    QCOMPARE(window_->measurements_[0].name, QStringLiteral("入口,控制点"));
    QCOMPARE(window_->measurements_[0].group, QStringLiteral("一层控制点"));
    QCOMPARE(window_->measurements_[0].color_hex, QStringLiteral("#00CCFF"));
    QVERIFY(close_to(window_->measurements_[0].point_a.x, 2.0, 1e-6));
    window_->undo_measurement();
    QVERIFY(close_to(window_->measurements_[0].point_a.x, 1.0, 1e-6));
    window_->redo_measurement();
    QVERIFY(close_to(window_->measurements_[0].point_a.x, 2.0, 1e-6));

    window_->unit_combo_->setCurrentIndex(1);
    QCOMPARE(window_->measurements_[2].kind, MeasurementKind::Area);
    const QString area_text = window_->measurement_primary_text(window_->measurements_[2]);
    QVERIFY2(area_text.contains(QStringLiteral("cm²")), qPrintable(area_text));
    window_->unit_combo_->setCurrentIndex(0);

    QTemporaryDir export_directory;
    QVERIFY(export_directory.isValid());
    QString export_error;
    const QString json_path = export_directory.filePath(QStringLiteral("measurements.json"));
    const QString csv_path = export_directory.filePath(QStringLiteral("measurements.csv"));
    QVERIFY2(window_->write_measurement_export(json_path, true, &export_error),
      qPrintable(export_error));
    QVERIFY2(window_->write_measurement_export(csv_path, false, &export_error),
      qPrintable(export_error));
    QFile json_file(json_path);
    QVERIFY(json_file.open(QIODevice::ReadOnly));
    const QJsonDocument exported_json = QJsonDocument::fromJson(json_file.readAll());
    QCOMPARE(exported_json.object().value(QStringLiteral("measurements")).toArray().size(), 6);
    QCOMPARE(
      exported_json.object().value(QStringLiteral("measurements")).toArray().at(2).toObject()
        .value(QStringLiteral("kind")).toString(),
      QStringLiteral("area"));
    QFile csv_file(csv_path);
    QVERIFY(csv_file.open(QIODevice::ReadOnly));
    const QString csv_text = QString::fromUtf8(csv_file.readAll());
    QVERIFY(csv_text.contains(QStringLiteral("area_planar_m2")));
    QVERIFY(csv_text.contains(QStringLiteral("\"入口,控制点\"")));
    QVERIFY(csv_text.contains(QStringLiteral("\"引号\"\"与逗号,转义\"")));

    const QString html = window_->build_report_html();
    QVERIFY(html.contains(QStringLiteral("点云测量报告")));
    QVERIFY(html.contains(QStringLiteral("多边形面积")));
    QVERIFY(html.contains(QStringLiteral("圆与直径")));
    QVERIFY(!html.contains(QRegularExpression(QStringLiteral("%[0-9]+"))));

    QTemporaryDir report_directory;
    QVERIFY(report_directory.isValid());
    const QString requested_report = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_REPORT_PATH"));
    const QString report_path = requested_report.isEmpty() ?
      report_directory.filePath(QStringLiteral("report.pdf")) : requested_report;
    QString report_error;
    QVERIFY2(window_->write_pdf_report(report_path, &report_error), qPrintable(report_error));
    QVERIFY(QFileInfo(report_path).size() > 1000);

    const QString annotated_screenshot = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_ANNOTATION_SCREENSHOT"));
    if (!annotated_screenshot.isEmpty()) {
      QVERIFY2(window_->grab().save(annotated_screenshot, "PNG"),
        qPrintable(annotated_screenshot));
      QVERIFY(QFileInfo(annotated_screenshot).size() > 1000);
    }

    window_->clear_measurements();
    QVERIFY(window_->measurements_.empty());
    window_->undo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(6));
  }

  void advancedProjectRestoreWithFixture()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString project_path = directory.filePath(QStringLiteral("advanced.pcdmeasure"));

    MeasurementRecord area = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
       pcl::PointXYZ(0.0F, 2.0F, 0.0F)}, 21);
    area.name = QStringLiteral("测试区域");
    area.group = QStringLiteral("回归测试");
    area.note = QStringLiteral("工程版本 4");
    area.color_hex = QStringLiteral("#22CCAA");

    QJsonObject root;
    root.insert(QStringLiteral("format"), QStringLiteral("pcd-measure-project"));
    root.insert(QStringLiteral("version"), 4);
    root.insert(QStringLiteral("pcd_path"), QFileInfo(xyz).absoluteFilePath());
    QJsonObject view;
    view.insert(QStringLiteral("interaction_mode"), 8);
    view.insert(QStringLiteral("color_mode"), 1);
    view.insert(QStringLiteral("point_size"), 2);
    view.insert(QStringLiteral("background_mode"), 0);
    view.insert(QStringLiteral("projection_mode"), 0);
    view.insert(QStringLiteral("unit"), 0);
    view.insert(QStringLiteral("axes"), true);
    view.insert(QStringLiteral("grid"), false);
    view.insert(QStringLiteral("bounds"), true);
    root.insert(QStringLiteral("view"), view);
    root.insert(QStringLiteral("measurements"), QJsonArray{measurement_to_json(area)});

    QFile file(project_path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    QVERIFY(file.write(QJsonDocument(root).toJson()) > 0);
    file.close();

    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(xyz), "Advanced project PCD failed to load");
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QCOMPARE(window_->measurements_.front().kind, MeasurementKind::Area);
    QCOMPARE(window_->measurements_.front().name, QStringLiteral("测试区域"));
    QCOMPARE(window_->measurements_.front().group, QStringLiteral("回归测试"));
    QCOMPARE(window_->interaction_mode_combo_->currentIndex(), 8);
    QCOMPARE(window_->next_measurement_id_, 22);
    QVERIFY(window_->undo_history_.empty());
  }

  void advancedSelectionAndRegionExport()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");

    bool dialog_handled = false;
    QTimer::singleShot(50, window_.get(), [&dialog_handled]() {
      auto * dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget());
      if (!dialog) return;
      const std::array<double, 6> values{-0.1, 1.1, -0.1, 2.1, -0.1, 1.0};
      for (std::size_t index = 0; index < values.size(); ++index) {
        auto * field = dialog->findChild<QDoubleSpinBox *>(
          QStringLiteral("exactCropValue%1").arg(index));
        if (!field) return;
        field->setValue(values[index]);
      }
      auto * buttons = dialog->findChild<QDialogButtonBox *>(QStringLiteral("exactCropButtons"));
      if (!buttons) return;
      dialog_handled = true;
      buttons->button(QDialogButtonBox::Ok)->click();
    });
    window_->open_exact_crop_dialog();
    QVERIFY(dialog_handled);
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.type, CropSelectionType::Box);
    QCOMPARE(window_->crop_.point_count, std::size_t(6));
    QVERIFY(window_->crop_.analysis.valid);
    QVERIFY(window_->local_quality_label_->text().contains(QStringLiteral("点/m²")));
    QCOMPARE(window_->picking_tree_->getInputCloud()->size(), std::size_t(6));

    window_->invert_current_crop();
    QCOMPARE(window_->crop_.point_count, std::size_t(6));
    QVERIFY(window_->crop_.inverted);
    window_->reset_crop();
    QVERIFY(!window_->crop_.active);

    window_->clear_measurements();
    const MeasurementRecord area = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(-0.1F, -0.1F, 0.0F), pcl::PointXYZ(1.1F, -0.1F, 0.0F),
       pcl::PointXYZ(1.1F, 2.1F, 0.0F), pcl::PointXYZ(-0.1F, 2.1F, 0.0F)}, 1);
    window_->commit_measurement(area);
    window_->measurement_table_->selectRow(0);
    window_->crop_to_selected_area();
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.type, CropSelectionType::Polygon);
    QCOMPARE(window_->crop_.point_count, std::size_t(6));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString exported_path = directory.filePath(QStringLiteral("selected.pcd"));
    QString export_error;
    QVERIFY2(window_->write_cloud_file(exported_path, window_->crop_.full_cloud, &export_error),
      qPrintable(export_error));
    const CloudLoadResult exported = load_pcd_and_analyze(exported_path);
    QVERIFY2(exported.ok(), qPrintable(exported.error));
    QCOMPARE(exported.metrics.finite_points, std::size_t(6));
    QVERIFY(exported.metrics.has_rgb);

    window_->invert_current_crop();
    QCOMPARE(window_->crop_.point_count, std::size_t(6));
    window_->reset_crop();
  }

  void analysisExportsAndRecords()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    const auto cloud = window_->active_full_cloud();
    const RegionAnalysisResult region = analyze_region(cloud);
    QVERIFY(region.valid);
    QVERIFY(region.plane.valid);
    const HistogramResult histogram = build_height_histogram(cloud, 6);
    QVERIFY(histogram.valid);
    std::size_t histogram_total = 0;
    for (const HistogramBin & bin : histogram.bins) histogram_total += bin.count;
    QCOMPARE(histogram_total, cloud->size());

    const std::vector<pcl::PointXYZ> path{
      pcl::PointXYZ(-1.0F, 1.0F, 0.0F), pcl::PointXYZ(2.0F, 1.0F, 0.0F)};
    const ProfileResult profile = build_elevation_profile(cloud, path, 2.1, 0.5);
    QVERIFY2(profile.valid, qPrintable(profile.error));
    QVERIFY(profile.selected_points > 0);
    const std::vector<pcl::PointXYZ> boundary{
      pcl::PointXYZ(-1.1F, -0.1F, 0.0F), pcl::PointXYZ(2.1F, -0.1F, 0.0F),
      pcl::PointXYZ(2.1F, 2.1F, 0.0F), pcl::PointXYZ(-1.1F, 2.1F, 0.0F)};
    const VolumeResult volume = estimate_grid_volume(cloud, boundary, 0.0, 0.5);
    QVERIFY2(volume.valid, qPrintable(volume.error));
    QVERIFY(volume.occupied_cells > 0);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QString error;
    const QString histogram_path = directory.filePath(QStringLiteral("histogram.csv"));
    const QString profile_path = directory.filePath(QStringLiteral("profile.csv"));
    const QString volume_path = directory.filePath(QStringLiteral("volume.csv"));
    QVERIFY2(window_->write_histogram_csv(histogram_path, histogram, &error), qPrintable(error));
    QVERIFY2(window_->write_profile_csv(profile_path, profile, &error), qPrintable(error));
    QVERIFY2(window_->write_volume_csv(volume_path, volume, &error), qPrintable(error));
    QFile histogram_file(histogram_path);
    QVERIFY(histogram_file.open(QIODevice::ReadOnly));
    QVERIFY(histogram_file.readLine().contains("lower_m"));
    QCOMPARE(histogram_file.readAll().count('\n'), static_cast<int>(histogram.bins.size()));
    QFile profile_file(profile_path);
    QVERIFY(profile_file.open(QIODevice::ReadOnly));
    QVERIFY(profile_file.readLine().contains("station_start_m"));
    QVERIFY(profile_file.readAll().count('\n') >= static_cast<int>(profile.bins.size()) - 1);
    QFile volume_file(volume_path);
    QVERIFY(volume_file.open(QIODevice::ReadOnly));
    QVERIFY(volume_file.readLine().contains("signed_volume_m3"));
    QVERIFY(volume_file.readAll().count('\n') >= static_cast<int>(volume.cells.size()) - 1);
    ProfileResult invalid_profile;
    QVERIFY(!window_->write_profile_csv(directory.filePath(QStringLiteral("invalid.csv")),
      invalid_profile, &error));
    HistogramResult invalid_histogram;
    QVERIFY(!window_->write_histogram_csv(directory.filePath(QStringLiteral("invalid-hist.csv")),
      invalid_histogram, &error));
    QVERIFY(!window_->write_histogram_csv(directory.path(), histogram, &error));
    QVERIFY(!window_->write_profile_csv(directory.path(), profile, &error));
    QVERIFY(!window_->write_volume_csv(directory.path(), volume, &error));

    window_->add_analysis_record(QStringLiteral("profile"), QStringLiteral("高程剖面"),
      QStringLiteral("测试剖面 · %1 点").arg(profile.selected_points),
      QJsonObject{{QStringLiteral("selected_points"), static_cast<qint64>(profile.selected_points)}});
    window_->add_analysis_record(QStringLiteral("volume"), QStringLiteral("栅格体积"),
      QStringLiteral("净体积 %1 m³").arg(volume.net_volume, 0, 'f', 4), QJsonObject{});
    QCOMPARE(window_->analysis_records_.size(), std::size_t(2));
    const QString analysis_path = directory.filePath(QStringLiteral("analyses.json"));
    QVERIFY2(window_->write_analysis_json(analysis_path, &error), qPrintable(error));
    QFile analysis_file(analysis_path);
    QVERIFY(analysis_file.open(QIODevice::ReadOnly));
    QCOMPARE(QJsonDocument::fromJson(analysis_file.readAll()).object()
      .value(QStringLiteral("analyses")).toArray().size(), 2);
    QVERIFY(!window_->write_analysis_json(directory.path(), &error));
    const QString html = window_->build_report_html();
    QVERIFY(html.contains(QStringLiteral("区域与工程分析")));
    QVERIFY(html.contains(QStringLiteral("测试剖面")));
    QVERIFY(html.contains(QStringLiteral("净体积")));
    QVERIFY(!html.contains(QRegularExpression(QStringLiteral("%[0-9]+"))));
  }

  void measurementFilteringCopyGroupAndImport()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    window_->measurement_search_edit_->clear();
    window_->measurement_type_filter_->setCurrentIndex(0);

    MeasurementRecord first = calculate_measurement(MeasurementKind::Point,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F)}, 1);
    first.name = QStringLiteral("入口控制点");
    first.group = QStringLiteral("控制点");
    first.note = QStringLiteral("北侧");
    MeasurementRecord second = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 2);
    second.name = QStringLiteral("墙宽");
    second.group = QStringLiteral("墙体");
    MeasurementRecord third = calculate_measurement(MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
       pcl::PointXYZ(1.0F, 1.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 3);
    third.name = QStringLiteral("房间面积");
    third.group = QStringLiteral("房间");
    window_->commit_measurement(first);
    window_->commit_measurement(second);
    window_->commit_measurement(third);
    QCOMPARE(window_->measurement_table_->rowCount(), 3);

    window_->measurement_search_edit_->setText(QStringLiteral("入口"));
    QCOMPARE(window_->measurement_table_->rowCount(), 1);
    QCOMPARE(window_->selected_measurement_index(), 0);
    window_->measurement_search_edit_->clear();
    const int area_filter = window_->measurement_type_filter_->findData(QStringLiteral("area"));
    window_->measurement_type_filter_->setCurrentIndex(area_filter);
    QCOMPARE(window_->measurement_table_->rowCount(), 1);
    window_->measurement_type_filter_->setCurrentIndex(0);
    const int wall_group = window_->measurement_group_filter_->findData(QStringLiteral("墙体"));
    QVERIFY(wall_group > 0);
    window_->measurement_group_filter_->setCurrentIndex(wall_group);
    QCOMPARE(window_->measurement_table_->rowCount(), 1);
    window_->measurement_group_filter_->setCurrentIndex(0);

    window_->measurement_table_->selectRow(0);
    window_->copy_selected_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(4));
    QVERIFY(window_->measurements_.back().name.contains(QStringLiteral("副本")));
    QVERIFY(window_->measurements_.back().id != window_->measurements_.front().id);
    window_->measurement_search_edit_->clear();
    window_->measurement_group_filter_->setCurrentIndex(0);
    window_->rebuild_measurement_table();
    window_->measurement_table_->selectRow(0);
    window_->toggle_selected_group_visibility();
    QVERIFY(!window_->measurements_[0].visible);
    QVERIFY(!window_->measurements_[3].visible);
    window_->undo_measurement();
    QVERIFY(window_->measurements_[0].visible);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    MeasurementRecord imported_record = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 2.0F, 0.0F)}, 1);
    imported_record.name = QStringLiteral("重复 ID 导入");
    QJsonObject unknown = measurement_to_json(imported_record);
    unknown.insert(QStringLiteral("kind"), QStringLiteral("unknown"));
    QJsonObject invalid = measurement_to_json(imported_record);
    invalid.insert(QStringLiteral("vertices_m"), QJsonArray{QJsonArray{0.0, 0.0}});
    const QJsonObject import_root{{QStringLiteral("measurements"),
      QJsonArray{measurement_to_json(imported_record), unknown, invalid}}};
    const QString import_path = directory.filePath(QStringLiteral("import.json"));
    QFile import_file(import_path);
    QVERIFY(import_file.open(QIODevice::WriteOnly));
    QVERIFY(import_file.write(QJsonDocument(import_root).toJson()) > 0);
    import_file.close();
    int imported = 0;
    int skipped = 0;
    QString error;
    QVERIFY2(window_->import_measurements_from_file(
      import_path, &imported, &skipped, &error), qPrintable(error));
    QCOMPARE(imported, 1);
    QCOMPARE(skipped, 2);
    QCOMPARE(window_->measurements_.size(), std::size_t(5));
    std::set<int> ids;
    for (const MeasurementRecord & record : window_->measurements_) ids.insert(record.id);
    QCOMPARE(ids.size(), window_->measurements_.size());

    const QString broken_path = directory.filePath(QStringLiteral("broken.json"));
    QFile broken(broken_path);
    QVERIFY(broken.open(QIODevice::WriteOnly));
    broken.write("{not-json");
    broken.close();
    QVERIFY(!window_->import_measurements_from_file(broken_path, nullptr, nullptr, &error));
  }

  void transformOriginProjectV7AndRecovery()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    MeasurementRecord segment = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 1);
    segment.name = QStringLiteral("变换基准线");
    window_->commit_measurement(segment);
    const auto original_cloud = window_->current_.cloud;
    CloudTransformParameters parameters;
    parameters.translation_x = 1.0;
    parameters.translation_y = 2.0;
    parameters.translation_z = 3.0;
    parameters.uniform_scale = 2.0;
    QString error;
    QVERIFY2(window_->apply_cloud_transform(parameters, &error), qPrintable(error));
    QVERIFY(window_->transform_backup_valid_);
    QVERIFY(window_->undo_transform_action_->isEnabled());
    QVERIFY(close_to(window_->current_.cloud->front().x, 1.0, 1e-6));
    QVERIFY(close_to(window_->current_.cloud->front().y, 2.0, 1e-6));
    QVERIFY(close_to(window_->current_.cloud->front().z, 3.0, 1e-6));
    QVERIFY(close_to(window_->measurements_.front().distance_3d, 2.0, 1e-6));
    QCOMPARE(window_->current_.cloud->front().r, original_cloud->front().r);
    window_->display_origin_ = {1.0, 2.0, 3.0};
    QCOMPARE(window_->format_coordinate(pcl::PointXYZ(1.0F, 2.0F, 3.0F)),
      QStringLiteral("X 0.0000  Y 0.0000  Z 0.0000 m"));
    QVERIFY(close_to(window_->measurements_.front().distance_3d, 2.0, 1e-6));

    const Bounds3d & bounds = window_->current_.metrics.raw_bounds;
    QVERIFY(window_->apply_box_crop(BoxSelection{bounds.min_x - 0.1, bounds.max_x + 0.1,
      bounds.min_y - 0.1, bounds.max_y + 0.1, bounds.min_z - 0.1, bounds.max_z + 0.1, false}, false));
    QCOMPARE(window_->crop_.point_count, std::size_t(12));
    window_->add_analysis_record(QStringLiteral("region"), QStringLiteral("工程分析"),
      QStringLiteral("应完整恢复"), QJsonObject{{QStringLiteral("ok"), true}});
    window_->measurement_labels_check_->setChecked(false);

    const CloudLoadResult second = load_pcd_and_analyze(xyz);
    QVERIFY(second.ok());
    CloudComparisonOptions comparison_options;
    comparison_options.run_icp = false;
    comparison_options.difference_threshold = 0.2;
    CloudComparisonResult comparison = compare_point_clouds(
      window_->current_.cloud, second.cloud, comparison_options);
    QVERIFY2(comparison.valid, qPrintable(comparison.error));
    window_->set_cloud_comparison(xyz, comparison_options, 73, comparison);
    QVERIFY(window_->viewer_->contains("comparison_cloud"));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString project_path = directory.filePath(QStringLiteral("complete.pcworkbench"));
    QVERIFY2(window_->write_project_file(project_path, &error), qPrintable(error));
    QFile project_file(project_path);
    QVERIFY(project_file.open(QIODevice::ReadOnly));
    const QJsonObject root = QJsonDocument::fromJson(project_file.readAll()).object();
    QCOMPARE(root.value(QStringLiteral("format")).toString(),
      QStringLiteral("point-cloud-workbench-project"));
    QCOMPARE(root.value(QStringLiteral("version")).toInt(), 8);
    QVERIFY(root.contains(QStringLiteral("cloud_path")));
    QVERIFY(!root.contains(QStringLiteral("pcd_path")));
    QVERIFY(root.value(QStringLiteral("transform")).toObject()
      .value(QStringLiteral("active")).toBool());
    QVERIFY(root.value(QStringLiteral("comparison")).toObject()
      .value(QStringLiteral("active")).toBool());
    QVERIFY(!root.value(QStringLiteral("view")).toObject()
      .value(QStringLiteral("measurement_labels")).toBool(true));
    QCOMPARE(root.value(QStringLiteral("analyses")).toArray().size(), 2);

    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(xyz), "Version 8 project failed to restore");
    QVERIFY(close_to(window_->current_.cloud->front().x, 1.0, 1e-6));
    QVERIFY(close_to(window_->current_.cloud->front().y, 2.0, 1e-6));
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QVERIFY(close_to(window_->measurements_.front().distance_3d, 2.0, 1e-6));
    QCOMPARE(window_->display_origin_[0], 1.0);
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.point_count, std::size_t(12));
    QCOMPARE(window_->analysis_records_.size(), std::size_t(2));
    QVERIFY(window_->comparison_.active);
    QCOMPARE(window_->comparison_.opacity_percent, 73);
    QVERIFY(!window_->measurement_labels_check_->isChecked());

    window_->write_auto_recovery();
    QVERIFY(QFileInfo::exists(window_->auto_recovery_path()));
    window_->clear_auto_recovery();
    QVERIFY(!QFileInfo::exists(window_->auto_recovery_path()));

    window_->undo_cloud_transform();
    QVERIFY(close_to(window_->current_.cloud->front().x, 0.0, 1e-6));
    QVERIFY(close_to(window_->measurements_.front().distance_3d, 1.0, 1e-6));
    window_->measurement_labels_check_->setChecked(true);
  }

  void derivedRegionProjectCompanion()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    auto derived = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (std::size_t index = 0; index < 6; ++index) derived->push_back((*window_->current_.cloud)[index]);
    derived->width = static_cast<std::uint32_t>(derived->size());
    derived->height = 1;
    derived->is_dense = true;
    QVERIFY(window_->apply_local_cloud(derived, CropSelectionType::Derived,
      QStringLiteral("测试派生区域"), false));
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString project_path = directory.filePath(QStringLiteral("derived.pcworkbench"));
    QString error;
    QVERIFY2(window_->write_project_file(project_path, &error), qPrintable(error));
    QVERIFY(QFileInfo::exists(project_path + QStringLiteral(".region.pcd")));
    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(xyz), "Derived project failed to restore");
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.type, CropSelectionType::Derived);
    QCOMPARE(window_->crop_.point_count, std::size_t(6));
    QCOMPARE(window_->crop_.description, QStringLiteral("测试派生区域"));
  }

  void autoRecoveryAcceptRejectAndCorruptFailure()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    window_->clear_auto_recovery();
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    MeasurementRecord record = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 91);
    record.name = QStringLiteral("自动恢复基准");
    window_->commit_measurement(record);
    window_->display_origin_ = {0.25, -0.5, 1.25};

    window_->write_auto_recovery();
    QVERIFY(QFileInfo::exists(window_->auto_recovery_path()));
    bool rejected = false;
    QVERIFY(drive_modal([this]() { window_->maybe_restore_auto_recovery(); },
      [&rejected](QDialog * dialog) {
        auto * message = qobject_cast<QMessageBox *>(dialog);
        if (!message || !message->button(QMessageBox::No)) return false;
        rejected = true;
        message->button(QMessageBox::No)->click();
        return true;
      }));
    QVERIFY(rejected);
    QVERIFY(!QFileInfo::exists(window_->auto_recovery_path()));

    window_->write_auto_recovery();
    QVERIFY(QFileInfo::exists(window_->auto_recovery_path()));
    window_->clear_measurements();
    window_->display_origin_ = {0.0, 0.0, 0.0};
    bool accepted = false;
    QVERIFY(drive_modal([this]() { window_->maybe_restore_auto_recovery(); },
      [&accepted](QDialog * dialog) {
        auto * message = qobject_cast<QMessageBox *>(dialog);
        if (!message || !message->button(QMessageBox::Yes)) return false;
        accepted = true;
        message->button(QMessageBox::Yes)->click();
        return true;
      }));
    QVERIFY(accepted);
    QVERIFY2(wait_for_load(xyz), "Automatic recovery project failed to load");
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QCOMPARE(window_->measurements_.front().name, QStringLiteral("自动恢复基准"));
    QCOMPARE(window_->display_origin_[0], 0.25);
    QCOMPARE(window_->display_origin_[1], -0.5);
    QCOMPARE(window_->display_origin_[2], 1.25);
    QVERIFY(!window_->restoring_auto_recovery_);
    QVERIFY(window_->pending_project_path_.isEmpty());
    QVERIFY(!QFileInfo::exists(window_->auto_recovery_path()));

    QFile corrupt(window_->auto_recovery_path());
    QVERIFY(QDir().mkpath(QFileInfo(corrupt).absolutePath()));
    QVERIFY(corrupt.open(QIODevice::WriteOnly));
    corrupt.write("{broken-recovery");
    corrupt.close();
    int stage = 0;
    QVERIFY(drive_modal([this]() { window_->maybe_restore_auto_recovery(); },
      [&stage](QDialog * dialog) {
        auto * message = qobject_cast<QMessageBox *>(dialog);
        if (!message) return false;
        if (stage == 0 && message->button(QMessageBox::Yes)) {
          ++stage;
          message->button(QMessageBox::Yes)->click();
          return false;
        }
        if (stage == 1) {
          ++stage;
          message->accept();
          return true;
        }
        return false;
      }));
    QCOMPARE(stage, 2);
    QVERIFY(!window_->restoring_auto_recovery_);
    window_->clear_auto_recovery();
  }

  void transformPreviewCancelAndSpatialStateReset()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    const pcl::PointXYZRGB original_full = window_->current_.cloud->front();
    const pcl::PointXYZRGB original_display = window_->current_.display_cloud->front();
    int stage = 0;
    bool preview_state_valid = false;
    QVERIFY(drive_modal([this]() {
      window_->open_cloud_transform();
    }, [this, &stage, &preview_state_valid, original_full, original_display](QDialog * dialog) {
      auto * x = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("cloudTransform0"));
      auto * preview = dialog->findChild<QPushButton *>(QStringLiteral("previewCloudTransform"));
      if (!x || !preview) return false;
      if (stage == 0) {
        x->setValue(0.75);
        preview->click();
        ++stage;
        return false;
      }
      preview_state_valid = window_->transform_preview_display_ &&
        close_to(window_->transform_preview_display_->front().x,
          original_display.x + 0.75, 1e-6) &&
        close_to(window_->current_.cloud->front().x, original_full.x, 1e-6);
      dialog->reject();
      ++stage;
      return true;
    }));
    QCOMPARE(stage, 2);
    QVERIFY(preview_state_valid);
    QVERIFY(!window_->transform_preview_display_);
    QVERIFY(close_to(window_->current_.cloud->front().x, original_full.x, 1e-6));

    const Bounds3d bounds = window_->current_.metrics.raw_bounds;
    QVERIFY(window_->apply_box_crop(BoxSelection{bounds.min_x, bounds.max_x, bounds.min_y,
      bounds.max_y, bounds.min_z, bounds.max_z, false}, false));
    const CloudLoadResult second = load_pcd_and_analyze(xyz);
    QVERIFY(second.ok());
    CloudComparisonOptions options;
    options.run_icp = false;
    const CloudComparisonResult comparison = compare_point_clouds(
      window_->current_.cloud, second.cloud, options);
    QVERIFY(comparison.valid);
    window_->set_cloud_comparison(xyz, options, 80, comparison);
    window_->add_analysis_record(QStringLiteral("test"), QStringLiteral("旧分析"),
      QStringLiteral("应在变换时清除"), QJsonObject{});
    QVERIFY(window_->crop_.active);
    QVERIFY(window_->comparison_.active);

    CloudTransformParameters parameters;
    parameters.translation_x = 0.25;
    QString error;
    QVERIFY2(window_->apply_cloud_transform(parameters, &error), qPrintable(error));
    QVERIFY(!window_->crop_.active);
    QVERIFY(!window_->comparison_.active);
    QVERIFY(!window_->viewer_->contains("comparison_cloud"));
    QCOMPARE(window_->analysis_records_.size(), std::size_t(1));
    QCOMPARE(window_->analysis_records_.front().kind, QStringLiteral("transform"));
    window_->undo_cloud_transform();
    QVERIFY(close_to(window_->current_.cloud->front().x, original_full.x, 1e-6));
  }

  void plotWidgetHandlesEmptyConstantAndNonFiniteSeries()
  {
    PlotWidget plot;
    plot.set_title(QStringLiteral("分析曲线"));
    plot.set_axis_labels(QStringLiteral("里程 (m)"), QStringLiteral("高程 (m)"));
    plot.resize(640, 360);
    plot.show();
    QApplication::processEvents();
    QImage image = plot.grab().toImage();
    QVERIFY(!image.isNull());
    QVERIFY(std::abs(static_cast<double>(image.width()) / image.devicePixelRatio() -
      plot.width()) <= 1.0);
    QVERIFY(std::abs(static_cast<double>(image.height()) / image.devicePixelRatio() -
      plot.height()) <= 1.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    plot.set_series({
      PlotSeries{QStringLiteral("常量折线"), QColor(QStringLiteral("#47E5D6")),
        {QPointF(1.0, 2.0), QPointF(nan, nan), QPointF(1.0, 2.0)}, false},
      PlotSeries{QStringLiteral("柱状"), QColor(QStringLiteral("#FF9F43")),
        {QPointF(1.0, 2.0), QPointF(nan, 3.0)}, true}});
    QApplication::processEvents();
    image = plot.grab().toImage();
    QVERIFY(!image.isNull());
    std::set<QRgb> colors;
    for (int y = 0; y < image.height(); y += 12) {
      for (int x = 0; x < image.width(); x += 12) colors.insert(image.pixel(x, y));
    }
    QVERIFY(colors.size() > std::size_t(4));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QString error;
    const QString png_path = directory.filePath(QStringLiteral("图表.png"));
    QVERIFY2(window_->write_widget_png(&plot, png_path, &error), qPrintable(error));
    QFile png(png_path);
    QVERIFY(png.open(QIODevice::ReadOnly));
    QCOMPARE(png.read(8), QByteArray::fromHex("89504e470d0a1a0a"));
    QVERIFY(!window_->write_widget_png(&plot, directory.path(), &error));
    QVERIFY(!window_->write_widget_png(nullptr, png_path, &error));
  }

  void interactiveAnalysisComparisonAndTransformDialogs()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();

    bool region_text_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_region_analysis(); },
      [&region_text_seen](QDialog * dialog) {
        auto * text = dialog->findChild<QTextEdit *>(QStringLiteral("regionAnalysisText"));
        if (!text) return false;
        region_text_seen = text->toPlainText().contains(QStringLiteral("PCA 拟合平面"));
        dialog->reject();
        return true;
      }));
    QVERIFY(region_text_seen);

    int stage = 0;
    bool histogram_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_height_histogram(); },
      [&stage, &histogram_seen](QDialog * dialog) {
        if (stage == 0) {
          auto * bins = dialog->findChild<QSpinBox *>(QStringLiteral("histogramBinCount"));
          if (!bins) return false;
          bins->setValue(5);
          ++stage;
          dialog->accept();
          return false;
        }
        if (dialog->findChild<QWidget *>(QStringLiteral("heightHistogramPlot"))) {
          histogram_seen = true;
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(histogram_seen);

    MeasurementRecord line = calculate_measurement(MeasurementKind::Polyline,
      {pcl::PointXYZ(-1.0F, 1.0F, 0.0F), pcl::PointXYZ(2.0F, 1.0F, 0.0F)}, 1);
    line.name = QStringLiteral("剖面线");
    window_->commit_measurement(line);
    window_->measurement_table_->selectRow(0);
    stage = 0;
    bool profile_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_elevation_profile(); },
      [&stage, &profile_seen](QDialog * dialog) {
        if (stage == 0) {
          auto * width = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("profileCorridorWidth"));
          auto * bin = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("profileBinSize"));
          if (!width || !bin) return false;
          width->setValue(2.1);
          bin->setValue(0.5);
          ++stage;
          dialog->accept();
          return false;
        }
        if (dialog->findChild<QWidget *>(QStringLiteral("elevationProfilePlot"))) {
          profile_seen = true;
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(profile_seen);

    MeasurementRecord area = calculate_measurement(MeasurementKind::Area,
      {pcl::PointXYZ(-1.1F, -0.1F, 0.0F), pcl::PointXYZ(2.1F, -0.1F, 0.0F),
       pcl::PointXYZ(2.1F, 2.1F, 0.0F), pcl::PointXYZ(-1.1F, 2.1F, 0.0F)}, 2);
    area.name = QStringLiteral("体积边界");
    window_->commit_measurement(area);
    window_->measurement_table_->selectRow(1);
    stage = 0;
    bool volume_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_volume_estimation(); },
      [&stage, &volume_seen](QDialog * dialog) {
        if (stage == 0) {
          auto * base = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("volumeBaseZ"));
          auto * cell = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("volumeCellSize"));
          if (!base || !cell) return false;
          base->setValue(0.0);
          cell->setValue(0.5);
          ++stage;
          dialog->accept();
          return false;
        }
        auto * result = dialog->findChild<QTextEdit *>(QStringLiteral("volumeResultText"));
        if (result) {
          volume_seen = result->toPlainText().contains(QStringLiteral("净体积"));
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(volume_seen);

    stage = 0;
    bool outlier_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_outlier_filter(); },
      [&stage, &outlier_seen](QDialog * dialog) {
        if (stage == 0) {
          auto * mean_k = dialog->findChild<QSpinBox *>(QStringLiteral("outlierMeanK"));
          auto * sigma = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("outlierStddev"));
          if (!mean_k || !sigma) return false;
          mean_k->setValue(4);
          sigma->setValue(1.0);
          ++stage;
          dialog->accept();
          return false;
        }
        auto * result = dialog->findChild<QLabel *>(QStringLiteral("outlierResultLabel"));
        if (result) {
          outlier_seen = result->text().contains(QStringLiteral("拟移除"));
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(outlier_seen);

    stage = 0;
    bool plane_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_dominant_plane(); },
      [&stage, &plane_seen](QDialog * dialog) {
        if (stage == 0) {
          auto * threshold = dialog->findChild<QDoubleSpinBox *>(
            QStringLiteral("planeDistanceThreshold"));
          auto * iterations = dialog->findChild<QSpinBox *>(QStringLiteral("planeMaxIterations"));
          if (!threshold || !iterations) return false;
          threshold->setValue(0.02);
          iterations->setValue(200);
          ++stage;
          dialog->accept();
          return false;
        }
        auto * result = dialog->findChild<QTextEdit *>(QStringLiteral("dominantPlaneResultText"));
        if (result) {
          plane_seen = result->toPlainText().contains(QStringLiteral("平面方程"));
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(plane_seen);

    bool records_seen = false;
    QVERIFY(drive_modal([this]() { window_->show_analysis_records(); },
      [&records_seen](QDialog * dialog) {
        auto * table = dialog->findChild<QTableWidget *>(QStringLiteral("analysisRecordsTable"));
        if (!table) return false;
        records_seen = table->rowCount() >= 6;
        dialog->reject();
        return true;
      }));
    QVERIFY(records_seen);

    bool origin_applied = false;
    QVERIFY(drive_modal([this]() { window_->set_display_origin(); },
      [&origin_applied](QDialog * dialog) {
        auto * x = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("displayOrigin0"));
        auto * y = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("displayOrigin1"));
        auto * z = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("displayOrigin2"));
        if (!x || !y || !z) return false;
        x->setValue(0.1);
        y->setValue(0.2);
        z->setValue(0.3);
        origin_applied = true;
        dialog->accept();
        return true;
      }));
    QVERIFY(origin_applied);
    QCOMPARE(window_->display_origin_[0], 0.1);

    bool transform_applied = false;
    QVERIFY(drive_modal([this]() { window_->open_cloud_transform(); },
      [&transform_applied](QDialog * dialog) {
        auto * x = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("cloudTransform0"));
        auto * apply = dialog->findChild<QPushButton *>(QStringLiteral("applyCloudTransform"));
        if (!x || !apply) return false;
        x->setValue(0.25);
        transform_applied = true;
        apply->click();
        return true;
      }));
    QVERIFY(transform_applied);
    QVERIFY(close_to(window_->current_.cloud->front().x, 0.25, 1e-6));
    window_->undo_cloud_transform();
    QVERIFY(close_to(window_->current_.cloud->front().x, 0.0, 1e-6));

    stage = 0;
    bool comparison_seen = false;
    QVERIFY(drive_modal([this, xyz]() { window_->open_cloud_comparison(); },
      [&stage, &comparison_seen, xyz](QDialog * dialog) {
        if (stage == 0) {
          auto * path = dialog->findChild<QLineEdit *>(QStringLiteral("comparisonPathEdit"));
          auto * icp = dialog->findChild<QCheckBox *>(QStringLiteral("comparisonRunIcp"));
          auto * buttons = dialog->findChild<QDialogButtonBox *>(QStringLiteral("comparisonButtons"));
          if (!path || !icp || !buttons) return false;
          path->setText(xyz);
          icp->setChecked(false);
          ++stage;
          buttons->button(QDialogButtonBox::Ok)->click();
          return false;
        }
        auto * summary = dialog->findChild<QTextEdit *>(QStringLiteral("comparisonSummaryText"));
        if (summary) {
          comparison_seen = summary->toPlainText().contains(QStringLiteral("逐点最近距离"));
          dialog->reject();
          return true;
        }
        return false;
      }));
    QVERIFY(comparison_seen);
    QVERIFY(window_->comparison_.active);
    QVERIFY(window_->viewer_->contains("comparison_cloud"));
    window_->comparison_visibility_action_->setChecked(false);
    window_->toggle_comparison_visibility();
    QVERIFY(!window_->viewer_->contains("comparison_cloud"));
    window_->clear_cloud_comparison();

    bool height_applied = false;
    QVERIFY(drive_modal([this]() { window_->open_height_filter_dialog(); },
      [&height_applied](QDialog * dialog) {
        auto * minimum = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("heightFilterMinimum"));
        auto * maximum = dialog->findChild<QDoubleSpinBox *>(QStringLiteral("heightFilterMaximum"));
        if (!minimum || !maximum) return false;
        minimum->setValue(-0.01);
        maximum->setValue(0.61);
        height_applied = true;
        dialog->accept();
        return true;
      }));
    QVERIFY(height_applied);
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.point_count, std::size_t(12));
    window_->context_cloud_check_->setChecked(true);
    QVERIFY(window_->viewer_->contains("context_cloud"));
    window_->context_cloud_check_->setChecked(false);
    QVERIFY(!window_->viewer_->contains("context_cloud"));
    window_->reset_crop();
  }

  void historyLimitRedoBranchAndPendingCancellation()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    window_->undo_history_.clear();
    window_->redo_history_.clear();

    for (int index = 0; index < 105; ++index) {
      window_->push_measurement_history();
      MeasurementRecord record = calculate_measurement(MeasurementKind::Point,
        {pcl::PointXYZ(index * 0.01F, 0.0F, 0.0F)}, index + 1);
      window_->measurements_.push_back(record);
      window_->next_measurement_id_ = index + 2;
    }
    QCOMPARE(window_->undo_history_.size(), std::size_t(100));
    QVERIFY(window_->redo_history_.empty());
    window_->undo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(104));
    QCOMPARE(window_->redo_history_.size(), std::size_t(1));
    window_->redo_measurement();
    QCOMPARE(window_->measurements_.size(), std::size_t(105));
    window_->undo_measurement();
    MeasurementRecord branch = calculate_measurement(MeasurementKind::Point,
      {pcl::PointXYZ(8.0F, 8.0F, 8.0F)}, window_->next_measurement_id_);
    window_->commit_measurement(branch);
    QVERIFY(window_->redo_history_.empty());

    window_->clear_measurements();
    window_->undo_history_.clear();
    window_->redo_history_.clear();
    window_->interaction_mode_combo_->setCurrentIndex(1);
    window_->accept_polyline_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    window_->accept_polyline_point(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
    QCOMPARE(window_->active_polyline_points_.size(), std::size_t(2));
    window_->undo_measurement();
    QCOMPARE(window_->active_polyline_points_.size(), std::size_t(1));
    QVERIFY(!window_->finish_polyline_button_->isEnabled());
    window_->start_new_measurement();
    QVERIFY(window_->active_polyline_points_.empty());

    window_->interaction_mode_combo_->setCurrentIndex(0);
    window_->accept_picked_point(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
    QVERIFY(window_->pending_point_.has_value());
    window_->undo_measurement();
    QVERIFY(!window_->pending_point_.has_value());
  }

  void projectValidationRelativePathsAndLegacyVersions()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString copied_cloud = directory.filePath(QStringLiteral("子目录/点 云.pcd"));
    QVERIFY(QDir().mkpath(QFileInfo(copied_cloud).absolutePath()));
    QVERIFY(QFile::copy(xyz, copied_cloud));

    const auto write_json = [](const QString & path, const QJsonObject & root) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly)) return false;
        const QByteArray payload = QJsonDocument(root).toJson();
        return file.write(payload) == payload.size();
      };
    QJsonObject state;
    QString error;
    const QString corrupt_path = directory.filePath(QStringLiteral("corrupt.pcworkbench"));
    QFile corrupt(corrupt_path);
    QVERIFY(corrupt.open(QIODevice::WriteOnly));
    corrupt.write("{not-json");
    corrupt.close();
    QVERIFY(!window_->read_project_file(corrupt_path, &state, &error));
    QVERIFY(!error.isEmpty());

    QJsonObject root{{QStringLiteral("format"), QStringLiteral("wrong-format")},
      {QStringLiteral("version"), 8}, {QStringLiteral("cloud_path"), copied_cloud}};
    QString project_path = directory.filePath(QStringLiteral("invalid.pcworkbench"));
    QVERIFY(write_json(project_path, root));
    QVERIFY(!window_->read_project_file(project_path, &state, &error));
    root.insert(QStringLiteral("format"), QStringLiteral("point-cloud-workbench-project"));
    root.insert(QStringLiteral("version"), 9);
    QVERIFY(write_json(project_path, root));
    QVERIFY(!window_->read_project_file(project_path, &state, &error));
    root.insert(QStringLiteral("version"), 8.5);
    QVERIFY(write_json(project_path, root));
    QVERIFY(!window_->read_project_file(project_path, &state, &error));
    root.insert(QStringLiteral("version"), 8);
    root.remove(QStringLiteral("cloud_path"));
    QVERIFY(write_json(project_path, root));
    QVERIFY(!window_->read_project_file(project_path, &state, &error));

    root.insert(QStringLiteral("cloud_path"), QStringLiteral("/missing/absolute/cloud.pcd"));
    root.insert(QStringLiteral("cloud_relative_path"), QStringLiteral("子目录/点 云.pcd"));
    QVERIFY(write_json(project_path, root));
    QVERIFY2(window_->read_project_file(project_path, &state, &error), qPrintable(error));
    QCOMPARE(window_->resolve_project_cloud_path(project_path, state, &error),
      QFileInfo(copied_cloud).absoluteFilePath());

    QJsonObject invalid_transform{{QStringLiteral("active"), true},
      {QStringLiteral("matrix"), QJsonArray{
        QJsonArray{1.0, 0.5, 0.0, 0.0}, QJsonArray{0.0, 1.0, 0.0, 0.0},
        QJsonArray{0.0, 0.0, 1.0, 0.0}, QJsonArray{0.0, 0.0, 0.0, 1.0}}}};
    root.insert(QStringLiteral("transform"), invalid_transform);
    QVERIFY(write_json(project_path, root));
    QVERIFY(!window_->read_project_file(project_path, &state, &error));
    root.remove(QStringLiteral("transform"));

    for (int version : {1, 2}) {
      QJsonObject legacy{{QStringLiteral("version"), version},
        {QStringLiteral("pcd_path"), QStringLiteral("/moved/cloud.pcd")},
        {QStringLiteral("pcd_relative_path"), QStringLiteral("子目录/点 云.pcd")},
        {QStringLiteral("measurements"), QJsonArray{measurement_to_json(calculate_measurement(
          MeasurementKind::Segment,
          {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 4))}}};
      project_path = directory.filePath(QStringLiteral("legacy-v%1.pcdmeasure").arg(version));
      QVERIFY(write_json(project_path, legacy));
      window_->open_project_path(project_path);
      QVERIFY2(wait_for_load(copied_cloud), "Legacy relative-path project failed to load");
      QCOMPARE(window_->measurements_.size(), std::size_t(1));
      QCOMPARE(window_->measurements_.front().id, 4);
    }

    const QString copied_second = directory.filePath(QStringLiteral("子目录/第二点云.pcd"));
    QVERIFY(QFile::copy(xyz, copied_second));
    QJsonObject moved_comparison{
      {QStringLiteral("format"), QStringLiteral("pcd-measure-project")},
      {QStringLiteral("version"), 7},
      {QStringLiteral("pcd_path"), QStringLiteral("/moved/base.pcd")},
      {QStringLiteral("pcd_relative_path"), QStringLiteral("子目录/点 云.pcd")},
      {QStringLiteral("comparison"), QJsonObject{
        {QStringLiteral("active"), true},
        {QStringLiteral("second_path"), QStringLiteral("/moved/second.pcd")},
        {QStringLiteral("second_relative_path"), QStringLiteral("子目录/第二点云.pcd")},
        {QStringLiteral("visible"), false},
        {QStringLiteral("opacity_percent"), 64},
        {QStringLiteral("options"), QJsonObject{
          {QStringLiteral("run_icp"), false},
          {QStringLiteral("difference_threshold_m"), 0.01}}}}}};
    project_path = directory.filePath(QStringLiteral("moved-comparison.pcdmeasure"));
    QVERIFY(write_json(project_path, moved_comparison));
    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(copied_cloud), "Relative comparison paths failed to restore");
    QVERIFY(window_->comparison_.active);
    QCOMPARE(QFileInfo(window_->comparison_.second_path).absoluteFilePath(),
      QFileInfo(copied_second).absoluteFilePath());
    QCOMPARE(window_->comparison_.opacity_percent, 64);
    QVERIFY(!window_->comparison_.visible);
    QVERIFY(!window_->viewer_->contains("comparison_cloud"));

    QJsonObject version_five{{QStringLiteral("format"), QStringLiteral("pcd-measure-project")},
      {QStringLiteral("version"), 5}, {QStringLiteral("pcd_path"), copied_cloud},
      {QStringLiteral("crop"), QJsonObject{{QStringLiteral("active"), true},
        {QStringLiteral("type"), QStringLiteral("box")},
        {QStringLiteral("description"), QStringLiteral("保存的高程筛选说明")},
        {QStringLiteral("bounds_m"), QJsonArray{-0.1, 1.1, -0.1, 2.1, -0.1, 1.0}}}}};
    project_path = directory.filePath(QStringLiteral("version-five.pcdmeasure"));
    QVERIFY(write_json(project_path, version_five));
    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(copied_cloud), "Version 5 project failed to load");
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.type, CropSelectionType::Box);
    QCOMPARE(window_->crop_.description, QStringLiteral("保存的高程筛选说明"));

    QJsonArray polygon{QJsonArray{-0.1, -0.1, 0.0}, QJsonArray{1.1, -0.1, 0.0},
      QJsonArray{1.1, 2.1, 0.0}, QJsonArray{-0.1, 2.1, 0.0}};
    QJsonArray analyses;
    for (int index = 0; index < 205; ++index) {
      analyses.append(QJsonObject{{QStringLiteral("kind"), QStringLiteral("test")},
        {QStringLiteral("title"), QStringLiteral("记录 %1").arg(index)}});
    }
    QJsonObject version_six{{QStringLiteral("format"), QStringLiteral("pcd-measure-project")},
      {QStringLiteral("version"), 6}, {QStringLiteral("pcd_path"), copied_cloud},
      {QStringLiteral("crop"), QJsonObject{{QStringLiteral("active"), true},
        {QStringLiteral("type"), QStringLiteral("polygon")},
        {QStringLiteral("description"), QStringLiteral("保存的套索说明")},
        {QStringLiteral("polygon_m"), polygon},
        {QStringLiteral("z_range_m"), QJsonArray{-0.1, 1.0}}}},
      {QStringLiteral("analyses"), analyses}};
    project_path = directory.filePath(QStringLiteral("version-six.pcdmeasure"));
    QVERIFY(write_json(project_path, version_six));
    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(copied_cloud), "Version 6 project failed to load");
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->crop_.type, CropSelectionType::Polygon);
    QCOMPARE(window_->crop_.description, QStringLiteral("保存的套索说明"));
    QCOMPARE(window_->analysis_records_.size(), std::size_t(200));
  }

  void failedLoadAndMissingProjectPartsDegradeSafely()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    window_->commit_measurement(calculate_measurement(MeasurementKind::Point,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F)}, 1));
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString broken_cloud = directory.filePath(QStringLiteral("broken.pcd"));
    QFile broken(broken_cloud);
    QVERIFY(broken.open(QIODevice::WriteOnly));
    broken.write("not a PCD");
    broken.close();
    QVERIFY(dismiss_message_after([this, broken_cloud]() { window_->open_path(broken_cloud); }));
    QVERIFY(window_->current_.ok());
    QCOMPARE(QFileInfo(window_->current_.path).absoluteFilePath(), QFileInfo(xyz).absoluteFilePath());
    QCOMPARE(window_->current_path_, window_->current_.path);
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QVERIFY(window_->system_status_label_->text().contains(QStringLiteral("READY")));
    QCOMPARE(window_->source_badge_label_->text(), QStringLiteral("LOAD ERROR"));
    QVERIFY(window_->pending_project_state_.isEmpty());

    const QString unsupported = directory.filePath(QStringLiteral("not-a-cloud.txt"));
    QFile text_file(unsupported);
    QVERIFY(text_file.open(QIODevice::WriteOnly));
    text_file.write("text");
    text_file.close();
    bool unsupported_dismissed = false;
    QVERIFY(drive_modal([this, unsupported]() { window_->open_path(unsupported); },
      [&unsupported_dismissed](QDialog * dialog) {
        auto * message = qobject_cast<QMessageBox *>(dialog);
        if (!message) return false;
        unsupported_dismissed = true;
        message->accept();
        return true;
      }));
    QVERIFY(unsupported_dismissed);
    QCOMPARE(QFileInfo(window_->current_.path).absoluteFilePath(), QFileInfo(xyz).absoluteFilePath());

    QJsonObject project{
      {QStringLiteral("format"), QStringLiteral("point-cloud-workbench-project")},
      {QStringLiteral("version"), 8}, {QStringLiteral("cloud_path"), xyz},
      {QStringLiteral("crop"), QJsonObject{{QStringLiteral("active"), true},
        {QStringLiteral("type"), QStringLiteral("derived")},
        {QStringLiteral("derived_cloud_relative_path"), QStringLiteral("missing-region.pcd")}}},
      {QStringLiteral("comparison"), QJsonObject{{QStringLiteral("active"), true},
        {QStringLiteral("second_path"), QStringLiteral("/missing/second.pcd")},
        {QStringLiteral("second_relative_path"), QStringLiteral("missing-second.pcd")}}}};
    const QString project_path = directory.filePath(QStringLiteral("missing-parts.pcworkbench"));
    QFile project_file(project_path);
    QVERIFY(project_file.open(QIODevice::WriteOnly));
    const QByteArray payload = QJsonDocument(project).toJson();
    QCOMPARE(project_file.write(payload), static_cast<qint64>(payload.size()));
    project_file.close();
    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(xyz), "Project with missing optional parts failed to load base PCD");
    QVERIFY(!window_->crop_.active);
    QVERIFY(!window_->comparison_.active);
    QVERIFY(window_->statusBar()->currentMessage().contains(QStringLiteral("配套 PCD 缺失")));
    QVERIFY(window_->statusBar()->currentMessage().contains(QStringLiteral("第二点云缺失")));
  }

  void atomicOutputsScreenshotAndLargeReport()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->clear_measurements();
    window_->commit_measurement(calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 1));
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QString error;

    const QString screenshot = directory.filePath(QStringLiteral("当前视图.png"));
    QVERIFY2(window_->write_view_screenshot(screenshot, &error), qPrintable(error));
    QFile png(screenshot);
    QVERIFY(png.open(QIODevice::ReadOnly));
    QCOMPARE(png.read(8), QByteArray::fromHex("89504e470d0a1a0a"));

    const QString cloud_path = directory.filePath(QStringLiteral("原子导出.pcd"));
    QVERIFY2(window_->write_cloud_file(cloud_path, window_->current_.cloud, &error), qPrintable(error));
    const CloudLoadResult reloaded = load_pcd_and_analyze(cloud_path);
    QVERIFY2(reloaded.ok(), qPrintable(reloaded.error));
    QCOMPARE(reloaded.cloud->size(), window_->current_.cloud->size());

    std::vector<MeasurementRecord> original_records = window_->measurements_;
    window_->measurements_.clear();
    for (int index = 0; index < 140; ++index) {
      MeasurementRecord record = calculate_measurement(MeasurementKind::Segment,
        {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(index * 0.01F + 1.0F, 0.0F, 0.0F)},
        index + 1);
      record.name = QStringLiteral("中文长记录 %1").arg(index + 1);
      record.note = QStringLiteral("包含逗号, 引号\"和换行\n第二行，用于分页与转义检查。");
      window_->measurements_.push_back(record);
    }
    const QString report = directory.filePath(QStringLiteral("large-report.pdf"));
    QVERIFY2(window_->write_pdf_report(report, &error), qPrintable(error));
    QFile pdf(report);
    QVERIFY(pdf.open(QIODevice::ReadOnly));
    QVERIFY(pdf.read(5).startsWith("%PDF"));
    QVERIFY(QFileInfo(report).size() > 5000);
    window_->measurements_ = original_records;
    window_->rebuild_measurement_table();

    const QString invalid_target = directory.path();
    const qint64 directory_size_before = QFileInfo(invalid_target).size();
    QVERIFY(!window_->write_view_screenshot(invalid_target, &error));
    QVERIFY(!window_->write_cloud_file(invalid_target, window_->current_.cloud, &error));
    QVERIFY(!window_->write_measurement_export(invalid_target, true, &error));
    QVERIFY(!window_->write_measurement_export(invalid_target, false, &error));
    QVERIFY(!window_->write_pdf_report(invalid_target, &error));
    QVERIFY(!window_->write_project_file(invalid_target, &error));
    QCOMPARE(QFileInfo(invalid_target).size(), directory_size_before);

    auto derived = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*window_->current_.cloud);
    QVERIFY(window_->apply_local_cloud(derived, CropSelectionType::Derived,
      QStringLiteral("导出失败测试"), false));
    const QString directory_target = directory.filePath(QStringLiteral("existing-directory"));
    QVERIFY(QDir().mkpath(directory_target));
    QVERIFY(!window_->write_project_file(directory_target, &error));
    QVERIFY(!QFileInfo::exists(directory_target + QStringLiteral(".region.pcd")));
    window_->reset_crop();
  }

  void minimumWindowLayoutAndOptionalSnapshot()
  {
    const QString xyz = fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd");
    QVERIFY2(load(xyz), "XYZ fixture failed to load");
    window_->resize(window_->minimumSize());
    QApplication::processEvents();
    auto * scroll = window_->findChild<QScrollArea *>(
      QStringLiteral("cloudInformationScroll"));
    auto * toolbar = window_->findChild<QToolBar *>();
    QVERIFY(scroll);
    QVERIFY(toolbar);
    QVERIFY(scroll->viewport()->width() > 250);
    QVERIFY2(scroll->viewport()->height() > 300,
      qPrintable(QStringLiteral("unexpected scroll viewport %1x%2 (%3)")
        .arg(scroll->viewport()->width()).arg(scroll->viewport()->height(), 0, 10)
        .arg(scroll->objectName())));
    QVERIFY(toolbar->height() > 30);
    QVERIFY(window_->centralWidget()->rect().contains(window_->centralWidget()->rect().center()));
    window_->switch_workspace(1);
    QApplication::processEvents();
    auto * live_preview = window_->findChild<QWidget *>(QStringLiteral("olxLivePreview"));
    QVERIFY(live_preview);
    QVERIFY(live_preview->isVisible());
    QVERIFY(live_preview->width() >= 400);
    QVERIFY(live_preview->height() >= 300);
    window_->switch_workspace(2);
    QApplication::processEvents();
    auto * bag_tabs = window_->findChild<QWidget *>(QStringLiteral("rosbagResultTabs"));
    QVERIFY(bag_tabs);
    QVERIFY(bag_tabs->isVisible());
    QVERIFY(bag_tabs->width() >= 700);
    window_->switch_workspace(3);
    QApplication::processEvents();
    QVERIFY(window_->environment_panel_->isVisible());
    QVERIFY(window_->environment_panel_->environment_table_->isVisible());
    QVERIFY(window_->environment_panel_->environment_table_->width() >= 400);
    window_->switch_workspace(0);
    QVERIFY(window_->cloud_toolbar_->isVisible());
    const QString output = QString::fromLocal8Bit(
      qgetenv("POINT_CLOUD_WORKBENCH_TEST_UI_SCREENSHOT"));
    if (!output.isEmpty()) {
      QVERIFY2(window_->grab().save(output, "PNG"), qPrintable(output));
      QVERIFY(QFileInfo(output).size() > 1000);
    }
    window_->resize(1540, 920);
  }

  void cropFullCloudSnapAndViewControls()
  {
    if (actual_pcd_.isEmpty() || !QFileInfo::exists(actual_pcd_)) {
      QSKIP("POINT_CLOUD_WORKBENCH_TEST_PCD is not set to an existing cloud");
    }
    QVERIFY2(load(actual_pcd_), "Reference PCD failed to reload");

    window_->open_path(actual_pcd_);
    QVERIFY(window_->load_watcher_.isRunning());
    const QString loading_path = window_->current_path_;
    window_->open_path(fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd"));
    QCOMPARE(window_->current_path_, loading_path);
    QVERIFY2(wait_for_load(actual_pcd_), "Busy-load guard did not preserve the first request");

    window_->apply_crop(-12.0, -2.0, -10.0, -2.0);
    QVERIFY(window_->crop_.active);
    QVERIFY(window_->crop_.point_count > std::size_t(100000));
    QVERIFY(window_->crop_.point_count < window_->current_.metrics.finite_points);
    QCOMPARE(window_->picking_tree_->getInputCloud()->size(), window_->crop_.point_count);
    QVERIFY(window_->local_points_label_->text() != QStringLiteral("—"));
    QVERIFY(window_->crop_.major_size > 1.0);
    QVERIFY(window_->crop_.minor_size > 1.0);
    QVERIFY(window_->crop_.height > 1.0);

    const pcl::PointXYZRGB & source = window_->crop_.full_cloud->front();
    const pcl::PointXYZ snapped = window_->snap_to_nearest_full_point(
      pcl::PointXYZ(source.x + 0.0001F, source.y + 0.0001F, source.z + 0.0001F));
    QVERIFY(close_to(snapped.x, source.x, 1e-6));
    QVERIFY(close_to(snapped.y, source.y, 1e-6));
    QVERIFY(close_to(snapped.z, source.z, 1e-6));
    window_->reset_crop();
    QVERIFY(!window_->crop_.active);
    QCOMPARE(window_->picking_tree_->getInputCloud()->size(), window_->current_.cloud->size());

    window_->background_combo_->setCurrentIndex(1);
    const double * background = window_->renderer_->GetBackground();
    QVERIFY(close_to(background[0], 11.0 / 255.0, 1e-4));
    window_->background_combo_->setCurrentIndex(2);
    background = window_->renderer_->GetBackground();
    QVERIFY(background[0] > 0.8);
    window_->custom_background_hex_ = QStringLiteral("#123456");
    window_->background_combo_->blockSignals(true);
    window_->background_combo_->setCurrentIndex(3);
    window_->background_combo_->blockSignals(false);
    window_->apply_background_mode(3);
    background = window_->renderer_->GetBackground();
    QVERIFY(close_to(background[0], 18.0 / 255.0, 1e-4));
    QVERIFY(close_to(background[1], 52.0 / 255.0, 1e-4));
    QVERIFY(close_to(background[2], 86.0 / 255.0, 1e-4));
    window_->background_combo_->setCurrentIndex(0);

    window_->projection_combo_->setCurrentIndex(1);
    QVERIFY(window_->renderer_->GetActiveCamera()->GetParallelProjection() != 0);
    window_->projection_combo_->setCurrentIndex(0);
    QVERIFY(window_->renderer_->GetActiveCamera()->GetParallelProjection() == 0);

    window_->point_size_spin_->setValue(4);
    QCOMPARE(window_->point_size_spin_->value(), 4);
    for (int mode = 0; mode < 3; ++mode) {
      window_->color_mode_combo_->setCurrentIndex(mode);
      QVERIFY(window_->viewer_->contains("pcd_cloud"));
    }
    window_->color_mode_combo_->setCurrentIndex(0);

    window_->grid_check_->setChecked(true);
    QVERIFY(!window_->grid_shape_ids_.empty());
    window_->bounds_check_->setChecked(true);
    QVERIFY(!window_->bounds_shape_ids_.empty());
    window_->top_view();
    window_->front_view();
    window_->left_view();
    window_->isometric_view();
    window_->fit_view();
  }

  void projectRestore()
  {
    if (actual_pcd_.isEmpty() || !QFileInfo::exists(actual_pcd_)) {
      QSKIP("POINT_CLOUD_WORKBENCH_TEST_PCD is not set to an existing cloud");
    }
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString project_path = directory.filePath(QStringLiteral("roundtrip.pcdmeasure"));

    QJsonObject root;
    root.insert(QStringLiteral("format"), QStringLiteral("pcd-measure-project"));
    root.insert(QStringLiteral("version"), 3);
    root.insert(QStringLiteral("pcd_path"), QFileInfo(actual_pcd_).absoluteFilePath());
    QJsonObject view;
    view.insert(QStringLiteral("color_mode"), 0);
    view.insert(QStringLiteral("point_size"), 3);
    view.insert(QStringLiteral("background_mode"), 1);
    view.insert(QStringLiteral("projection_mode"), 1);
    view.insert(QStringLiteral("unit"), 0);
    view.insert(QStringLiteral("interaction_mode"), 0);
    view.insert(QStringLiteral("axes"), true);
    view.insert(QStringLiteral("grid"), false);
    view.insert(QStringLiteral("bounds"), true);
    root.insert(QStringLiteral("view"), view);
    QJsonObject measurement;
    measurement.insert(QStringLiteral("id"), 7);
    measurement.insert(QStringLiteral("kind"), QStringLiteral("segment"));
    measurement.insert(QStringLiteral("vertices_m"), QJsonArray{
      QJsonArray{-8.0, -7.0, 1.0}, QJsonArray{-4.0, -4.0, 1.5}});
    root.insert(QStringLiteral("measurements"), QJsonArray{measurement});
    QJsonObject crop;
    crop.insert(QStringLiteral("active"), true);
    crop.insert(QStringLiteral("min_x"), -12.0);
    crop.insert(QStringLiteral("max_x"), -2.0);
    crop.insert(QStringLiteral("min_y"), -10.0);
    crop.insert(QStringLiteral("max_y"), -2.0);
    root.insert(QStringLiteral("crop"), crop);
    QJsonObject camera;
    camera.insert(QStringLiteral("position"), QJsonArray{-7.0, -6.0, 30.0});
    camera.insert(QStringLiteral("focal"), QJsonArray{-7.0, -6.0, 1.0});
    camera.insert(QStringLiteral("view_up"), QJsonArray{0.0, 1.0, 0.0});
    camera.insert(QStringLiteral("clip"), QJsonArray{0.1, 100.0});
    camera.insert(QStringLiteral("fovy"), 0.5235987756);
    camera.insert(QStringLiteral("parallel_projection"), true);
    camera.insert(QStringLiteral("parallel_scale"), 8.0);
    root.insert(QStringLiteral("camera"), camera);

    QFile file(project_path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    QCOMPARE(file.write(QJsonDocument(root).toJson()), qint64(QJsonDocument(root).toJson().size()));
    file.close();

    window_->open_project_path(project_path);
    QVERIFY2(wait_for_load(actual_pcd_), "Project PCD failed to load");
    QCOMPARE(window_->measurements_.size(), std::size_t(1));
    QCOMPARE(window_->measurements_.front().id, 7);
    QVERIFY(window_->crop_.active);
    QCOMPARE(window_->background_combo_->currentIndex(), 1);
    QCOMPARE(window_->projection_combo_->currentIndex(), 1);
    QVERIFY(window_->renderer_->GetActiveCamera()->GetParallelProjection() != 0);
    QVERIFY(close_to(window_->renderer_->GetActiveCamera()->GetParallelScale(), 8.0, 1e-6));
  }

  void cleanupTestCase()
  {
    window_.reset();
  }
};

int main(int argc, char * argv[])
{
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  vtkObject::GlobalWarningDisplayOff();
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName(QStringLiteral("Point Cloud Workbench Test"));
  QCoreApplication::setOrganizationName(QStringLiteral("Point Cloud Workbench"));
  MainWindowTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "main_window_test.moc"
