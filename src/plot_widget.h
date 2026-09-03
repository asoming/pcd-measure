#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

struct PlotSeries
{
  QString name;
  QColor color;
  QVector<QPointF> points;
  bool bars = false;
};

class PlotWidget final : public QWidget
{
public:
  explicit PlotWidget(QWidget * parent = nullptr);

  void set_title(const QString & title);
  void set_axis_labels(const QString & horizontal, const QString & vertical);
  void set_series(const QVector<PlotSeries> & series);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QString title_;
  QString horizontal_label_;
  QString vertical_label_;
  QVector<PlotSeries> series_;
};
