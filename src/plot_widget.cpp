#include "plot_widget.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>

PlotWidget::PlotWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(620, 320);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PlotWidget::set_title(const QString & title)
{
  title_ = title;
  update();
}

void PlotWidget::set_axis_labels(const QString & horizontal, const QString & vertical)
{
  horizontal_label_ = horizontal;
  vertical_label_ = vertical;
  update();
}

void PlotWidget::set_series(const QVector<PlotSeries> & series)
{
  series_ = series;
  update();
}

void PlotWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(QStringLiteral("#081721")));

  const QRectF plot_rect(72.0, 48.0, std::max(10, width() - 100), std::max(10, height() - 112));
  painter.setPen(QPen(QColor(QStringLiteral("#31546A")), 1.0));
  painter.setBrush(QColor(QStringLiteral("#0B1E2A")));
  painter.drawRoundedRect(plot_rect, 5.0, 5.0);

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const PlotSeries & series : series_) {
    for (const QPointF & point : series.points) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y())) continue;
      min_x = std::min(min_x, point.x());
      max_x = std::max(max_x, point.x());
      min_y = std::min(min_y, point.y());
      max_y = std::max(max_y, point.y());
    }
  }
  if (!std::isfinite(min_x) || !std::isfinite(min_y)) {
    painter.setPen(QColor(QStringLiteral("#7893A0")));
    painter.drawText(plot_rect, Qt::AlignCenter, QStringLiteral("暂无可绘制数据"));
    return;
  }
  if (std::abs(max_x - min_x) < 1e-12) {
    min_x -= 0.5;
    max_x += 0.5;
  }
  if (std::abs(max_y - min_y) < 1e-12) {
    min_y = std::min(0.0, min_y - 0.5);
    max_y += 0.5;
  }
  if (min_y > 0.0) min_y = 0.0;
  const double y_padding = std::max(1e-9, (max_y - min_y) * 0.08);
  max_y += y_padding;

  const auto map_point = [&](const QPointF & point) {
      return QPointF(
        plot_rect.left() + (point.x() - min_x) / (max_x - min_x) * plot_rect.width(),
        plot_rect.bottom() - (point.y() - min_y) / (max_y - min_y) * plot_rect.height());
    };

  QFont grid_font = painter.font();
  grid_font.setPointSize(std::max(8, grid_font.pointSize() - 1));
  painter.setFont(grid_font);
  for (int index = 0; index <= 5; ++index) {
    const double ratio = static_cast<double>(index) / 5.0;
    const double x = plot_rect.left() + ratio * plot_rect.width();
    const double y = plot_rect.bottom() - ratio * plot_rect.height();
    painter.setPen(QPen(QColor(36, 67, 83, 150), 1.0, Qt::DashLine));
    painter.drawLine(QPointF(x, plot_rect.top()), QPointF(x, plot_rect.bottom()));
    painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y));
    painter.setPen(QColor(QStringLiteral("#7893A0")));
    painter.drawText(QRectF(x - 44.0, plot_rect.bottom() + 7.0, 88.0, 18.0),
      Qt::AlignHCenter | Qt::AlignTop,
      QString::number(min_x + ratio * (max_x - min_x), 'g', 5));
    painter.drawText(QRectF(2.0, y - 9.0, 62.0, 18.0),
      Qt::AlignRight | Qt::AlignVCenter,
      QString::number(min_y + ratio * (max_y - min_y), 'g', 5));
  }

  for (const PlotSeries & series : series_) {
    if (series.points.isEmpty()) continue;
    painter.setPen(QPen(series.color, 2.0));
    if (series.bars) {
      const double bar_width = std::max(2.0,
        plot_rect.width() / static_cast<double>(std::max(1, series.points.size())) * 0.82);
      painter.setBrush(QColor(series.color.red(), series.color.green(), series.color.blue(), 150));
      for (const QPointF & value : series.points) {
        if (!std::isfinite(value.x()) || !std::isfinite(value.y())) continue;
        const QPointF top = map_point(value);
        const QPointF zero = map_point(QPointF(value.x(), 0.0));
        painter.drawRect(QRectF(top.x() - bar_width * 0.5, top.y(),
          bar_width, std::max(1.0, zero.y() - top.y())));
      }
    } else {
      QPainterPath path;
      bool started = false;
      for (const QPointF & value : series.points) {
        if (!std::isfinite(value.x()) || !std::isfinite(value.y())) {
          started = false;
          continue;
        }
        const QPointF mapped = map_point(value);
        if (!started) {
          path.moveTo(mapped);
          started = true;
        } else {
          path.lineTo(mapped);
        }
      }
      painter.setBrush(Qt::NoBrush);
      painter.drawPath(path);
    }
  }

  QFont title_font = painter.font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 2);
  painter.setFont(title_font);
  painter.setPen(QColor(QStringLiteral("#DCEBED")));
  painter.drawText(QRectF(12.0, 10.0, width() - 24.0, 26.0), Qt::AlignCenter, title_);

  painter.setFont(grid_font);
  painter.setPen(QColor(QStringLiteral("#9DB8C5")));
  painter.drawText(QRectF(plot_rect.left(), height() - 33.0, plot_rect.width(), 20.0),
    Qt::AlignCenter, horizontal_label_);
  painter.save();
  painter.translate(17.0, plot_rect.center().y());
  painter.rotate(-90.0);
  painter.drawText(QRectF(-plot_rect.height() * 0.5, -10.0, plot_rect.height(), 20.0),
    Qt::AlignCenter, vertical_label_);
  painter.restore();

  double legend_x = plot_rect.left() + 8.0;
  for (const PlotSeries & series : series_) {
    painter.setPen(QPen(series.color, 3.0));
    painter.drawLine(QPointF(legend_x, plot_rect.top() + 12.0),
      QPointF(legend_x + 18.0, plot_rect.top() + 12.0));
    painter.setPen(QColor(QStringLiteral("#C8D9DF")));
    painter.drawText(QPointF(legend_x + 24.0, plot_rect.top() + 16.0), series.name);
    legend_x += 34.0 + painter.fontMetrics().horizontalAdvance(series.name);
  }
}
