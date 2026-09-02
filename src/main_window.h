#pragma once

#include <optional>
#include <string>
#include <vector>

#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QJsonObject>
#include <QMainWindow>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>

#include "cloud_data.h"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QDropEvent;
class QDragEnterEvent;
class QLabel;
class QMenu;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;

enum class MeasurementKind
{
  Segment,
  Polyline
};

struct MeasurementRecord
{
  int id = 0;
  MeasurementKind kind = MeasurementKind::Segment;
  std::vector<pcl::PointXYZ> vertices;
  pcl::PointXYZ point_a;
  pcl::PointXYZ point_b;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  double horizontal = 0.0;
  double distance_3d = 0.0;
  double angle_degrees = 0.0;
  double slope_percent = 0.0;
};

struct CropRegion
{
  bool active = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  double major_size = 0.0;
  double minor_size = 0.0;
  double height = 0.0;
  double diagonal_3d = 0.0;
  std::array<double, 3> centroid{0.0, 0.0, 0.0};
  std::size_t point_count = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

  friend class MainWindowTest;

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

  void open_path(const QString & path);
  void open_project_path(const QString & path);

protected:
  void dragEnterEvent(QDragEnterEvent * event) override;
  void dropEvent(QDropEvent * event) override;

private slots:
  void choose_file();
  void reload_file();
  void load_finished();
  void apply_color_mode();
  void apply_background_mode(int index);
  void apply_projection_mode(int index);
  void apply_point_size(int size);
  void fit_view();
  void top_view();
  void front_view();
  void left_view();
  void isometric_view();
  void toggle_axes(bool enabled);
  void toggle_grid(bool enabled);
  void toggle_bounds(bool enabled);
  void start_new_measurement();
  void undo_measurement();
  void clear_measurements();
  void finish_polyline();
  void interaction_mode_changed(int index);
  void reset_crop();
  void refresh_measurement_units();
  void save_screenshot();
  void export_measurements();
  void open_project();
  void save_project();

private:
  void build_interface();
  void initialize_viewer();
  void begin_load(const QString & path);
  void render_cloud(bool reset_camera);
  void fill_information_panel();
  void set_loading(bool loading, const QString & message = QString());
  void handle_point_picking(const pcl::visualization::PointPickingEvent & event);
  void accept_picked_point(const pcl::PointXYZ & point);
  void accept_polyline_point(const pcl::PointXYZ & point);
  void accept_crop_point(const pcl::PointXYZ & point);
  void complete_measurement(const pcl::PointXYZ & point_b);
  MeasurementRecord make_measurement_record(
    MeasurementKind kind,
    const std::vector<pcl::PointXYZ> & vertices,
    int id) const;
  void render_measurement(const MeasurementRecord & record);
  void remove_measurement_shapes(int id);
  void clear_active_selection();
  void render_active_polyline();
  void update_measurement_details(const MeasurementRecord * record = nullptr);
  void rebuild_measurement_table();
  void update_grid_overlay();
  void update_bounds_overlay();
  void update_crop_overlay();
  void update_crop_information();
  void apply_crop(double min_x, double max_x, double min_y, double max_y);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr active_display_cloud() const;
  void add_recent_file(const QString & path);
  void rebuild_recent_menu();
  void restore_pending_project();
  void update_picking_tree();
  pcl::PointXYZ snap_to_nearest_full_point(const pcl::PointXYZ & picked) const;
  void request_render();
  void set_camera(
    double px, double py, double pz,
    double fx, double fy, double fz,
    double ux, double uy, double uz);

  double unit_scale() const;
  QString unit_suffix() const;
  QString format_distance(double meters) const;
  double marker_radius() const;

  QVTKOpenGLNativeWidget * vtk_widget_ = nullptr;
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;

  QFutureWatcher<CloudLoadResult> load_watcher_;
  QElapsedTimer load_timer_;
  CloudLoadResult current_;
  QString current_path_;

  QLabel * viewer_hint_label_ = nullptr;
  QLabel * file_name_label_ = nullptr;
  QLabel * file_path_label_ = nullptr;
  QLabel * file_size_label_ = nullptr;
  QLabel * encoding_label_ = nullptr;
  QLabel * fields_label_ = nullptr;
  QLabel * total_points_label_ = nullptr;
  QLabel * valid_points_label_ = nullptr;
  QLabel * invalid_points_label_ = nullptr;
  QLabel * colored_points_label_ = nullptr;
  QLabel * displayed_points_label_ = nullptr;
  QLabel * centroid_label_ = nullptr;
  QLabel * lowest_point_label_ = nullptr;
  QLabel * highest_point_label_ = nullptr;
  QLabel * raw_height_label_ = nullptr;
  QLabel * spacing_label_ = nullptr;
  QLabel * coordinate_range_label_ = nullptr;
  QLabel * major_size_label_ = nullptr;
  QLabel * minor_size_label_ = nullptr;
  QLabel * height_label_ = nullptr;
  QLabel * horizontal_diagonal_label_ = nullptr;
  QLabel * diagonal_3d_label_ = nullptr;
  QLabel * yaw_label_ = nullptr;
  QLabel * outlier_ratio_label_ = nullptr;
  QLabel * raw_bounds_label_ = nullptr;
  QLabel * raw_diagonal_label_ = nullptr;
  QLabel * point_a_label_ = nullptr;
  QLabel * point_b_label_ = nullptr;
  QLabel * delta_label_ = nullptr;
  QLabel * distance_3d_label_ = nullptr;
  QLabel * horizontal_distance_label_ = nullptr;
  QLabel * vertical_distance_label_ = nullptr;
  QLabel * slope_label_ = nullptr;
  QLabel * measurement_total_label_ = nullptr;
  QLabel * picking_instruction_label_ = nullptr;
  QLabel * crop_status_label_ = nullptr;
  QLabel * local_points_label_ = nullptr;
  QLabel * local_size_label_ = nullptr;
  QLabel * local_center_label_ = nullptr;
  QLabel * system_status_label_ = nullptr;

  QComboBox * color_mode_combo_ = nullptr;
  QComboBox * background_combo_ = nullptr;
  QComboBox * projection_combo_ = nullptr;
  QComboBox * unit_combo_ = nullptr;
  QComboBox * interaction_mode_combo_ = nullptr;
  QSpinBox * point_size_spin_ = nullptr;
  QCheckBox * axes_check_ = nullptr;
  QCheckBox * grid_check_ = nullptr;
  QCheckBox * bounds_check_ = nullptr;
  QTableWidget * measurement_table_ = nullptr;
  QProgressBar * progress_bar_ = nullptr;
  QPushButton * finish_polyline_button_ = nullptr;
  QPushButton * reset_crop_button_ = nullptr;
  QMenu * recent_menu_ = nullptr;
  QAction * reload_action_ = nullptr;
  QAction * screenshot_action_ = nullptr;
  QAction * export_action_ = nullptr;
  QAction * save_project_action_ = nullptr;

  std::optional<pcl::PointXYZ> pending_point_;
  std::vector<pcl::PointXYZ> active_polyline_points_;
  std::optional<pcl::PointXYZ> crop_first_corner_;
  std::vector<MeasurementRecord> measurements_;
  std::vector<std::string> grid_shape_ids_;
  std::vector<std::string> bounds_shape_ids_;
  std::vector<std::string> crop_shape_ids_;
  std::vector<std::string> active_shape_ids_;
  CropRegion crop_;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr picking_tree_;
  QJsonObject pending_project_state_;
  QString pending_project_path_;
  QString custom_background_hex_ = QStringLiteral("#07131D");
  int last_background_index_ = 0;
  int next_measurement_id_ = 1;
};
