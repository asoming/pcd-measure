#include <QApplication>
#include <QCoreApplication>
#include <QFileInfo>
#include <QSettings>
#include <QSurfaceFormat>
#include <QTimer>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkObject.h>

#include "main_window.h"

namespace
{
void import_legacy_settings()
{
  QSettings current;
  const QString marker = QStringLiteral("settingsMigration/legacyProductNamesImported");
  if (current.value(marker, false).toBool()) return;

  const auto import_namespace = [&current](const QString & organization, const QString & application) {
    QSettings legacy(
      QSettings::NativeFormat, QSettings::UserScope, organization, application);
    for (const QString & key : legacy.allKeys()) {
      if (!current.contains(key)) current.setValue(key, legacy.value(key));
    }
  };
  import_namespace(QStringLiteral("PCD Tools"), QStringLiteral("PCD Measure"));
  import_namespace(QStringLiteral("Local Odin Tools"), QStringLiteral("Odin PCD Measure"));

  if (!current.contains(QStringLiteral("recentCloudFiles")) &&
    current.contains(QStringLiteral("recentPcdFiles")))
  {
    current.setValue(
      QStringLiteral("recentCloudFiles"), current.value(QStringLiteral("recentPcdFiles")));
  }
  if (!current.contains(QStringLiteral("lastComparisonCloud")) &&
    current.contains(QStringLiteral("lastComparisonPcd")))
  {
    current.setValue(
      QStringLiteral("lastComparisonCloud"), current.value(QStringLiteral("lastComparisonPcd")));
  }
  current.remove(QStringLiteral("recentPcdFiles"));
  current.remove(QStringLiteral("lastComparisonPcd"));
  current.setValue(marker, true);
  current.sync();
}

bool is_project_suffix(const QString & suffix)
{
  return suffix.compare(QStringLiteral("pcworkbench"), Qt::CaseInsensitive) == 0 ||
    suffix.compare(QStringLiteral("pcdmeasure"), Qt::CaseInsensitive) == 0 ||
    suffix.compare(QStringLiteral("odinpcd"), Qt::CaseInsensitive) == 0;
}
}  // namespace

int main(int argc, char * argv[])
{
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  vtkObject::GlobalWarningDisplayOff();

  QApplication app(argc, argv);
  QCoreApplication::setApplicationName(QStringLiteral("Point Cloud Workbench"));
  QApplication::setApplicationDisplayName(QStringLiteral("点云工作台"));
  QCoreApplication::setApplicationVersion(QStringLiteral("2.4.0"));
  QCoreApplication::setOrganizationName(QStringLiteral("Point Cloud Workbench"));
  import_legacy_settings();

  MainWindow window;
  window.show();

  if (qEnvironmentVariableIsSet("POINT_CLOUD_WORKBENCH_TEST_EXIT_AFTER_LOAD")) {
    QObject::connect(&window, &MainWindow::cloudLoaded, &app,
      [&app](bool success) { app.exit(success ? 0 : 3); });
  }

  if (app.arguments().size() > 1) {
    const QString initial_path = app.arguments().at(1);
    QTimer::singleShot(0, &window, [&window, initial_path]() {
      const QString suffix = QFileInfo(initial_path).suffix();
      if (is_project_suffix(suffix)) {
        window.open_project_path(initial_path);
      } else {
        window.open_path(initial_path);
      }
    });
  }

  return app.exec();
}
