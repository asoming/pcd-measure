#include <iostream>
#include <limits>

#include <QCoreApplication>
#include <QStringList>

#include "cloud_data.h"

int main(int argc, char * argv[])
{
  QCoreApplication app(argc, argv);
  const QStringList arguments = app.arguments();
  QString path;
  std::size_t maximum_display_points = 2500000;
  if (arguments.size() == 2) {
    path = arguments.at(1);
  } else if (arguments.size() == 4 && arguments.at(1) == QStringLiteral("--max-display-points")) {
    bool ok = false;
    const qulonglong parsed = arguments.at(2).toULongLong(&ok);
    if (!ok || parsed == 0 || parsed > std::numeric_limits<std::size_t>::max()) {
      std::cerr << "--max-display-points must be a positive integer\n";
      return 1;
    }
    maximum_display_points = static_cast<std::size_t>(parsed);
    path = arguments.at(3);
  } else {
    std::cerr << "Usage: pcd_analyze [--max-display-points N] <cloud.pcd>\n";
    return 1;
  }

  const CloudLoadResult result = load_pcd_and_analyze(path, maximum_display_points);
  std::cout << cloud_result_to_json(result).toStdString();
  return result.ok() ? 0 : 2;
}
