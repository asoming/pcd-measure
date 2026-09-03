#include <QApplication>
#include <QCoreApplication>
#include <QFileInfo>
#include <QSurfaceFormat>
#include <QTimer>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkObject.h>

#include "main_window.h"

int main(int argc, char * argv[])
{
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  vtkObject::GlobalWarningDisplayOff();

  QApplication app(argc, argv);
  QCoreApplication::setApplicationName(QStringLiteral("PCD Measure"));
  QApplication::setApplicationDisplayName(QStringLiteral("PCD 点云测量工具"));
  QCoreApplication::setApplicationVersion(QStringLiteral("2.0.0"));
  QCoreApplication::setOrganizationName(QStringLiteral("PCD Tools"));

  MainWindow window;
  window.show();

  if (qEnvironmentVariableIsSet("PCD_MEASURE_TEST_EXIT_AFTER_LOAD")) {
    QObject::connect(&window, &MainWindow::cloudLoaded, &app,
      [&app](bool success) { app.exit(success ? 0 : 3); });
  }

  if (app.arguments().size() > 1) {
    const QString initial_path = app.arguments().at(1);
    QTimer::singleShot(0, &window, [&window, initial_path]() {
      const QString suffix = QFileInfo(initial_path).suffix();
      if (suffix.compare(QStringLiteral("pcdmeasure"), Qt::CaseInsensitive) == 0 ||
        suffix.compare(QStringLiteral("odinpcd"), Qt::CaseInsensitive) == 0) {
        window.open_project_path(initial_path);
      } else {
        window.open_path(initial_path);
      }
    });
  }

  return app.exec();
}
