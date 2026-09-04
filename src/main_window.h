#pragma once

#include <optional>
#include <map>
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
#include "analysis_tools.h"
#include "comparison_tools.h"
#include "measurement.h"
#include "transform_tools.h"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QDropEvent;
class QDragEnterEvent;
class QCloseEvent;
class QLabel;
class QLineEdit;
class QMenu;
class QProgressBar;
class QPushButton;
class QFrame;
class QSlider;
class QSpinBox;
class QTableWidget;
class QTimer;
class QToolButton;
class QToolBar;
class QStackedWidget;
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;
class OlxCaptureDialog;
class RosbagDiagnosticDialog;
class EnvironmentSetupPanel;

enum class CropSelectionType
{
  None,
  Box,
  Polygon,
  Derived
};

struct CropRegion
{
  bool active = false;
  CropSelectionType type = CropSelectionType::None;
  QString description;
  bool inverted = false;
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
  BoxSelection box;
  PolygonSelection polygon;
  RegionAnalysisResult analysis;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud;
};

struct AnalysisRecord
{
  QString kind;
  QString title;
  QString summary;
  QString created_at;
  QJsonObject data;
};

struct CloudComparisonState
{
  bool active = false;
  bool visible = true;
  int opacity_percent = 85;
  QString second_path;
  CloudComparisonOptions options;
  CloudComparisonResult result;
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

signals:
  void cloudLoaded(bool success);

protected:
  void dragEnterEvent(QDragEnterEvent * event) override;
  void dropEvent(QDropEvent * event) override;
  void closeEvent(QCloseEvent * event) override;

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
  void redo_measurement();
  void clear_measurements();
  void finish_polyline();
  void edit_selected_measurement();
  void delete_selected_measurement();
  void copy_selected_measurement();
  void toggle_selected_group_visibility();
  void import_measurements();
  void interaction_mode_changed(int index);
  void reset_crop();
  void open_exact_crop_dialog();
  void open_height_filter_dialog();
  void crop_to_selected_area();
  void invert_current_crop();
  void export_current_region();
  void show_region_analysis();
  void show_height_histogram();
  void show_elevation_profile();
  void show_volume_estimation();
  void show_outlier_filter();
  void show_dominant_plane();
  void show_analysis_records();
  void open_cloud_comparison();
  void show_comparison_summary();
  void toggle_comparison_visibility();
  void export_aligned_cloud();
  void export_comparison_distances();
  void export_comparison_summary();
  void clear_cloud_comparison();
  void set_display_origin();
  void open_cloud_transform();
  void undo_cloud_transform();
  void export_transformed_cloud();
  void export_transform_matrix();
  void refresh_measurement_units();
  void save_screenshot();
  void export_measurements();
  void export_pdf_report();
  void open_project();
  void save_project();
  void write_auto_recovery();
  void maybe_restore_auto_recovery();
  void toggle_sequence_playback();
  void sequence_position_changed(int index);
  void sequence_render_mode_changed();
  void sequence_tick();

private:
  void build_interface();
  void initialize_viewer();
  void open_rosbag_dialog(const QString & path = QString());
  void open_olx_capture_dialog();
  void switch_workspace(int index);
  void begin_load(const QString & path);
  void render_cloud(bool reset_camera);
  void fill_information_panel();
  void set_loading(bool loading, const QString & message = QString());
  void handle_point_picking(const pcl::visualization::PointPickingEvent & event);
  void accept_picked_point(const pcl::PointXYZ & point);
  void accept_polyline_point(const pcl::PointXYZ & point);
  void accept_generic_measurement_point(const pcl::PointXYZ & point);
  void accept_crop_point(const pcl::PointXYZ & point);
  void complete_measurement(const pcl::PointXYZ & point_b);
  MeasurementRecord make_measurement_record(
    MeasurementKind kind,
    const std::vector<pcl::PointXYZ> & vertices,
    int id) const;
  void render_measurement(const MeasurementRecord & record);
  void remove_measurement_shapes(int id);
  void render_all_measurements();
  void commit_measurement(const MeasurementRecord & record);
  void clear_active_selection();
  void render_active_polyline();
  void update_measurement_details(const MeasurementRecord * record = nullptr);
  void rebuild_measurement_table();
  void push_measurement_history();
  void restore_measurement_history(
    const std::vector<MeasurementRecord> & records,
    int next_id);
  int selected_measurement_index() const;
  void update_grid_overlay();
  void update_bounds_overlay();
  void update_crop_overlay();
  void update_crop_information();
  void apply_crop(double min_x, double max_x, double min_y, double max_y);
  bool apply_box_crop(const BoxSelection & selection, bool reset_camera = true);
  bool apply_polygon_crop(const PolygonSelection & selection, bool reset_camera = true);
  bool apply_local_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    CropSelectionType type,
    const QString & description,
    bool reset_camera = true);
  bool write_cloud_file(
    const QString & path,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
    QString * error = nullptr) const;
  bool write_view_screenshot(const QString & path, QString * error = nullptr) const;
  bool write_widget_png(QWidget * widget, const QString & path, QString * error = nullptr) const;
  bool write_histogram_csv(
    const QString & path,
    const HistogramResult & histogram,
    QString * error = nullptr) const;
  bool write_profile_csv(
    const QString & path,
    const ProfileResult & profile,
    QString * error = nullptr) const;
  bool write_volume_csv(
    const QString & path,
    const VolumeResult & volume,
    QString * error = nullptr) const;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr active_full_cloud() const;
  const MeasurementRecord * selected_measurement(MeasurementKind kind) const;
  void add_analysis_record(
    const QString & kind,
    const QString & title,
    const QString & summary,
    const QJsonObject & data);
  bool write_analysis_json(const QString & path, QString * error = nullptr) const;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr active_display_cloud() const;
  void add_recent_file(const QString & path);
  void rebuild_recent_menu();
  void add_recent_project(const QString & path);
  void rebuild_recent_project_menu();
  void restore_pending_project();
  void update_picking_tree();
  pcl::PointXYZ snap_to_nearest_full_point(const pcl::PointXYZ & picked) const;
  void request_render();
  void render_cloud_comparison();
  void set_cloud_comparison(
    const QString & path,
    const CloudComparisonOptions & options,
    int opacity_percent,
    CloudComparisonResult result);
  bool apply_cloud_transform(
    const CloudTransformParameters & parameters,
    QString * error = nullptr);
  void rebuild_current_display_cloud(std::size_t maximum_display_points = 2500000);
  void rebuild_crop_display_cloud(std::size_t maximum_display_points);
  void update_current_metrics_from_cloud();
  QString format_coordinate(const pcl::PointXYZ & point) const;
  void render_context_cloud();
  void set_camera(
    double px, double py, double pz,
    double fx, double fy, double fz,
    double ux, double uy, double uz);
  void configure_sequence_controls();
  void rebuild_sequence_display();
  void update_sequence_image();
  void stop_sequence_playback();

  double unit_scale() const;
  QString unit_suffix() const;
  QString format_distance(double meters) const;
  QString format_area(double square_meters) const;
  QString measurement_primary_text(const MeasurementRecord & record) const;
  QString measurement_extra_text(const MeasurementRecord & record) const;
  bool write_measurement_export(
    const QString & path,
    bool json,
    QString * error = nullptr) const;
  bool import_measurements_from_file(
    const QString & path,
    int * imported_count = nullptr,
    int * skipped_count = nullptr,
    QString * error = nullptr);
  QString build_report_html() const;
  bool write_pdf_report(const QString & path, QString * error = nullptr) const;
  QJsonObject build_project_state(const QString & project_path) const;
  bool read_project_file(
    const QString & path,
    QJsonObject * state,
    QString * error = nullptr) const;
  QString resolve_project_cloud_path(
    const QString & project_path,
    const QJsonObject & state,
    QString * error = nullptr) const;
  bool write_project_file(const QString & path, QString * error = nullptr) const;
  QString auto_recovery_path() const;
  void clear_auto_recovery();
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
  QLabel * source_badge_label_ = nullptr;
  QLabel * source_format_label_ = nullptr;
  QLabel * source_details_label_ = nullptr;
  QLabel * decoder_label_ = nullptr;
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
  QLabel * measurement_result_label_ = nullptr;
  QLabel * measurement_extra_label_ = nullptr;
  QLabel * measurement_metadata_label_ = nullptr;
  QLabel * picking_instruction_label_ = nullptr;
  QLabel * crop_status_label_ = nullptr;
  QLabel * local_points_label_ = nullptr;
  QLabel * local_size_label_ = nullptr;
  QLabel * local_center_label_ = nullptr;
  QLabel * local_quality_label_ = nullptr;
  QLabel * system_status_label_ = nullptr;
  QLabel * sequence_frame_label_ = nullptr;
  QLabel * camera_preview_label_ = nullptr;
  QLabel * camera_preview_title_ = nullptr;

  QComboBox * color_mode_combo_ = nullptr;
  QComboBox * background_combo_ = nullptr;
  QComboBox * projection_combo_ = nullptr;
  QComboBox * unit_combo_ = nullptr;
  QComboBox * interaction_mode_combo_ = nullptr;
  QComboBox * sequence_render_combo_ = nullptr;
  QComboBox * sequence_speed_combo_ = nullptr;
  QSpinBox * point_size_spin_ = nullptr;
  QSpinBox * display_limit_spin_ = nullptr;
  QCheckBox * axes_check_ = nullptr;
  QCheckBox * grid_check_ = nullptr;
  QCheckBox * bounds_check_ = nullptr;
  QCheckBox * context_cloud_check_ = nullptr;
  QCheckBox * measurement_labels_check_ = nullptr;
  QCheckBox * sequence_infinite_check_ = nullptr;
  QCheckBox * camera_preview_check_ = nullptr;
  QFrame * sequence_strip_ = nullptr;
  QFrame * camera_preview_frame_ = nullptr;
  QSlider * sequence_slider_ = nullptr;
  QToolButton * sequence_play_button_ = nullptr;
  QSpinBox * sequence_max_frames_spin_ = nullptr;
  QTableWidget * measurement_table_ = nullptr;
  QLineEdit * measurement_search_edit_ = nullptr;
  QComboBox * measurement_type_filter_ = nullptr;
  QComboBox * measurement_group_filter_ = nullptr;
  QProgressBar * progress_bar_ = nullptr;
  QToolBar * cloud_toolbar_ = nullptr;
  QStackedWidget * workspace_stack_ = nullptr;
  QToolButton * cloud_workspace_button_ = nullptr;
  QToolButton * capture_workspace_button_ = nullptr;
  QToolButton * rosbag_workspace_button_ = nullptr;
  QToolButton * environment_workspace_button_ = nullptr;
  OlxCaptureDialog * capture_panel_ = nullptr;
  RosbagDiagnosticDialog * rosbag_panel_ = nullptr;
  EnvironmentSetupPanel * environment_panel_ = nullptr;
  QPushButton * finish_polyline_button_ = nullptr;
  QPushButton * reset_crop_button_ = nullptr;
  QMenu * recent_menu_ = nullptr;
  QMenu * recent_project_menu_ = nullptr;
  QAction * reload_action_ = nullptr;
  QAction * output_tools_action_ = nullptr;
  QAction * screenshot_action_ = nullptr;
  QAction * export_action_ = nullptr;
  QAction * report_action_ = nullptr;
  QAction * advanced_tools_action_ = nullptr;
  QAction * comparison_tools_action_ = nullptr;
  QAction * comparison_visibility_action_ = nullptr;
  QAction * coordinate_tools_action_ = nullptr;
  QAction * undo_transform_action_ = nullptr;
  QAction * save_project_action_ = nullptr;
  QTimer * auto_recovery_timer_ = nullptr;
  QTimer * sequence_timer_ = nullptr;

  std::optional<pcl::PointXYZ> pending_point_;
  std::vector<pcl::PointXYZ> active_polyline_points_;
  std::optional<pcl::PointXYZ> crop_first_corner_;
  std::vector<MeasurementRecord> measurements_;
  std::vector<AnalysisRecord> analysis_records_;
  CloudComparisonState comparison_;
  std::array<double, 3> display_origin_{0.0, 0.0, 0.0};
  Eigen::Matrix4f cumulative_transform_ = Eigen::Matrix4f::Identity();
  CloudTransformParameters last_transform_parameters_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_preview_display_;
  bool transform_backup_valid_ = false;
  CloudLoadResult transform_backup_cloud_;
  std::vector<MeasurementRecord> transform_backup_measurements_;
  Eigen::Matrix4f transform_backup_matrix_ = Eigen::Matrix4f::Identity();
  struct MeasurementHistoryState
  {
    std::vector<MeasurementRecord> records;
    int next_id = 1;
  };
  std::vector<MeasurementHistoryState> undo_history_;
  std::vector<MeasurementHistoryState> redo_history_;
  std::map<int, std::vector<std::string>> measurement_shape_ids_;
  std::vector<std::string> grid_shape_ids_;
  std::vector<std::string> bounds_shape_ids_;
  std::vector<std::string> crop_shape_ids_;
  std::vector<std::string> active_shape_ids_;
  CropRegion crop_;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr picking_tree_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sequence_display_cloud_;
  QJsonObject pending_project_state_;
  QString pending_project_path_;
  bool restoring_auto_recovery_ = false;
  QString custom_background_hex_ = QStringLiteral("#07131D");
  int last_background_index_ = 0;
  int next_measurement_id_ = 1;
};
