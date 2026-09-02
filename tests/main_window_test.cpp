#include <cmath>
#include <memory>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDragEnterEvent>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMimeData>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QSurfaceFormat>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QUrl>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkObject.h>
#include <vtkRenderer.h>

#include "main_window.h"

class MainWindowTest : public QObject
{
  Q_OBJECT

private:
  std::unique_ptr<MainWindow> window_;
  QString actual_pcd_;
  QString fixture_directory_ = QStringLiteral(PCD_MEASURE_TEST_FIXTURE_DIR);

  bool wait_for_load(const QString & expected_path, int timeout_ms = 60000)
  {
    const QString expected = QFileInfo(expected_path).absoluteFilePath();
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeout_ms) {
      QApplication::processEvents(QEventLoop::AllEvents, 50);
      if (!window_->load_watcher_.isRunning() && window_->current_.ok() &&
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

private slots:
  void initTestCase()
  {
    actual_pcd_ = QString::fromLocal8Bit(qgetenv("PCD_MEASURE_TEST_PCD"));
    QVERIFY2(QFileInfo::exists(fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd")),
      "XYZ fixture is missing");
    QVERIFY2(QFileInfo::exists(fixture_directory_ + QStringLiteral("/rgba_ascii.pcd")),
      "RGBA fixture is missing");
    window_ = std::make_unique<MainWindow>();
    window_->show();
    QTest::qWait(250);
  }

  void fileLoadingAndActualStatistics()
  {
    if (actual_pcd_.isEmpty() || !QFileInfo::exists(actual_pcd_)) {
      QSKIP("PCD_MEASURE_TEST_PCD is not set to an existing cloud");
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

    const QStringList recent = QSettings().value(QStringLiteral("recentPcdFiles")).toStringList();
    QVERIFY(recent.contains(QFileInfo(actual_pcd_).absoluteFilePath()));
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

  void cropFullCloudSnapAndViewControls()
  {
    if (actual_pcd_.isEmpty() || !QFileInfo::exists(actual_pcd_)) {
      QSKIP("PCD_MEASURE_TEST_PCD is not set to an existing cloud");
    }
    QVERIFY2(load(actual_pcd_), "Reference PCD failed to reload");

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
      QSKIP("PCD_MEASURE_TEST_PCD is not set to an existing cloud");
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
  QCoreApplication::setApplicationName(QStringLiteral("PCD Measure Test"));
  QCoreApplication::setOrganizationName(QStringLiteral("PCD Tools"));
  MainWindowTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "main_window_test.moc"
