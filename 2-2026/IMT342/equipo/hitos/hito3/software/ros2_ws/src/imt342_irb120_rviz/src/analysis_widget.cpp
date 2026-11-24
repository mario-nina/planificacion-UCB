#include "analysis_widget.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include <QBuffer>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollBar>
#include <QTabWidget>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <QWheelEvent>

namespace imt342_irb120_rviz
{
namespace
{
constexpr std::size_t history_limit = 12000;
constexpr std::size_t events_limit = 400;
constexpr double degrees_per_rad = 57.29577951308232;
const double missing = std::numeric_limits<double>::quiet_NaN();

QJsonValue valueAt(QJsonValue value, const QString & path)
{
  for (const QString & key : path.split('.')) {
    if (value.isArray()) {
      const auto array = value.toArray();
      bool is_index;
      const int index = key.toInt(&is_index);
      if (!is_index || index < 0 || index >= array.size()) {
        return {};
      }
      value = array[index];
    } else {
      value = value.toObject().value(key);
    }
  }
  return value;
}

QString taskTrialId(const QJsonObject & task)
{
  return task.value("plan_id").toString(task.value("request_id").toString());
}

QString fkTrialId(const QJsonObject & sample)
{
  const auto fk = sample.value("fk_review").toObject();
  return fk.value("trial_id").toString(fk.value("id").toString());
}

bool reportsTrial(const QJsonObject & sample, const QString & id)
{
  return valueAt(sample, "physical_run.id").toString() == id ||
         valueAt(sample, "plan.run_id").toString() == id ||
         valueAt(sample, "individual_trial.trial_id").toString() == id ||
         fkTrialId(sample) == id ||
         taskTrialId(sample.value("task").toObject()) == id ||
         sample.value("pose_plan_id").toString() == id;
}

bool belongsToTrial(const QJsonObject & sample, const QString & id)
{
  return valueAt(sample, "physical_run.id").toString() == id ||
         valueAt(sample, "plan.run_id").toString() == id ||
         sample.value("evidence_operation_id").toString() == id;
}

QString fixedValue(double number, int precision = 5)
{
  return std::isfinite(number) ? QString::number(number, 'g', precision) : QStringLiteral("N/D");
}

QString jsonText(const QJsonValue & value)
{
  if (value.isDouble()) {
    return fixedValue(value.toDouble());
  }
  if (value.isString()) {
    return value.toString();
  }
  if (value.isBool()) {
    return value.toBool() ? QStringLiteral("Sí") : QStringLiteral("No");
  }
  if (value.isArray()) {
    return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  }
  if (value.isObject()) {
    return QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
  }
  return QStringLiteral("N/D");
}

QString matrixText(const QJsonValue & value)
{
  if (!value.isArray()) {
    return QStringLiteral("N/D");
  }
  QStringList rows;
  for (const auto & row : value.toArray()) {
    QStringList cells;
    for (const auto & cell : row.toArray()) {
      cells.append(fixedValue(cell.toDouble(missing), 6).rightJustified(12));
    }
    rows.append(cells.join(" "));
  }
  return rows.join('\n');
}

QString vectorText(const QJsonValue & value, double factor = 1.0)
{
  if (!value.isArray()) {
    return QStringLiteral("N/D");
  }
  QStringList parts;
  for (const auto & component : value.toArray()) {
    parts.append(fixedValue(component.toDouble(missing) * factor));
  }
  return parts.join("  ");
}

QString utcTime(const QJsonObject & object)
{
  const auto exact_stamp = object.value("timestamp_ns_text").toString();
  const auto stamp = object.value("timestamp_ns");
  const qint64 ns = !exact_stamp.isEmpty() ? exact_stamp.toLongLong() :
    (stamp.isString() ? stamp.toString().toLongLong() : static_cast<qint64>(stamp.toDouble()));
  return ns > 0 ? QDateTime::fromMSecsSinceEpoch(ns / 1000000, Qt::UTC).toString(Qt::ISODateWithMs) :
    QStringLiteral("N/D");
}

void setPageText(QPlainTextEdit * view, const QString & text)
{
  if (view->toPlainText() == text) {
    return;
  }
  const int old_scroll = view->verticalScrollBar()->value();
  view->setPlainText(text);
  view->verticalScrollBar()->setValue(old_scroll);
}

enum class SeriesSource
{
  Value, VirtualDerivative, SigmaActual, SigmaReference, TorqueModel,
  VirtualPosition, NormalPosition, TrajectoryPosition, OtherPosition, MockPosition, SentJointGoal, GoalError, FinalError
};

struct Series
{
  QString label;
  QString path;
  QColor color;
  Qt::PenStyle style = Qt::SolidLine;
  double factor = 1.0;
  bool respect_stale = false;
  double width = 1.6;
  SeriesSource source = SeriesSource::Value;
  bool step = false;
  bool quintic_only = false;
  bool points_only = false;
};

struct Sample
{
  double time;
  int segment;
  QJsonObject data;
};

struct Trace
{
  Series series;
  std::vector<QPointF> points;
};

class Plot : public QWidget
{
public:
  explicit Plot(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    setMinimumHeight(145);
    setMinimumWidth(240);
    setMouseTracking(true);
    setAccessibleName(tr("Gráfica de análisis IRB 120"));
    setToolTip(tr("Rueda para ampliar. Arrastre para desplazar. Doble clic para volver al tiempo actual."));
  }

  void data(std::vector<Trace> traces, double left, double right,
    const QString & unit, const QString & axis, double min_span = 0.0, double error_band = 0.0)
  {
    minimum_span_ = min_span;
    error_band_ = error_band;
    traces_ = std::move(traces);
    left_ = left;
    right_ = std::max(right, left + 1e-6);
    unit_ = unit;
    axis_ = axis;
    update();
  }

  std::function<void(double)> zoom;
  std::function<void(double)> pan;
  std::function<void()> reset;

  QImage snapshot()
  {
    QImage image(size(), QImage::Format_ARGB32_Premultiplied);
    image.fill(palette().base().color());
    render(&image);
    return image;
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), palette().base());
    const auto area = bounds();
    const QColor foreground = palette().text().color();
    QColor guides = foreground;
    guides.setAlpha(55);
    painter.setPen(QPen(guides));
    painter.drawRect(area);

    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -minimum;
    for (const auto & trace : traces_) {
      for (const auto & p : trace.points) {
        if (std::isfinite(p.y())) {
          minimum = std::min(minimum, p.y());
          maximum = std::max(maximum, p.y());
        }
      }
    }
    if (!std::isfinite(minimum)) {
      painter.setPen(foreground);
      painter.drawText(area, Qt::AlignCenter, tr("N/D · Sin muestras para las series elegidas"));
      return;
    }
    if (error_band_ > 0.0) {
      minimum = std::min(minimum, -error_band_);
      maximum = std::max(maximum, error_band_);
    }
    // Keep nearly constant joint traces readable without amplifying tiny changes.
    if (minimum_span_ > 0.0 && maximum - minimum < minimum_span_) {
      const double center = (minimum + maximum) / 2.0;
      minimum = center - minimum_span_ / 2.0;
      maximum = center + minimum_span_ / 2.0;
    }
    const double pad = std::max((maximum - minimum) * .07, 1e-8 * (1.0 + std::abs(maximum)));
    minimum -= pad;
    maximum += pad;
    const auto xToPx = [&](double x) {return area.left() + (x - left_) * area.width() / (right_ - left_);};
    const auto yToPx = [&](double y) {
      return area.bottom() - (y - minimum) * area.height() / (maximum - minimum);
    };
    if (error_band_ > 0.0) {
      QColor shade(60, 150, 95, 28);
      painter.fillRect(QRectF(QPointF(area.left(), yToPx(error_band_)),
        QPointF(area.right(), yToPx(-error_band_))), shade);
    }
    const int label_height = painter.fontMetrics().height() + 3;
    const int label_width = painter.fontMetrics().horizontalAdvance(QStringLiteral("-99.9e+99")) + 8;
    const int x_grid = area.width() < 420 ? 4 : 6;
    for (int i = 0; i <= x_grid; ++i) {
      const double fraction = static_cast<double>(i) / x_grid;
      const double x = area.left() + fraction * area.width();
      const double t = left_ + fraction * (right_ - left_);
      painter.setPen(guides);
      painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
      painter.setPen(foreground);
      painter.drawText(QRectF(x - label_width / 2.0, area.bottom() + 3, label_width, label_height),
        Qt::AlignCenter, fixedValue(t, 6));
    }
    for (int i = 0; i <= 3; ++i) {
      const double fraction = i / 3.0;
      const double y = area.bottom() - fraction * area.height();
      painter.setPen(guides);
      painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
      painter.setPen(foreground);
      painter.drawText(QRectF(1, y - label_height / 2.0, area.left() - 12, label_height),
        Qt::AlignRight | Qt::AlignVCenter,
        fixedValue(minimum + fraction * (maximum - minimum), 4));
    }
    painter.drawText(QRectF(6, 2, 160, label_height), Qt::AlignLeft | Qt::AlignVCenter, unit_);
    painter.drawText(QRectF(area.left(), 2, area.width(), label_height),
      Qt::AlignRight | Qt::AlignVCenter, axis_);

    painter.save();
    painter.setClipRect(area.adjusted(-1, -1, 1, 1));
    for (const auto & trace : traces_) {
      painter.setPen(QPen(trace.series.color, trace.series.width, trace.series.style));
      QPainterPath path;
      bool move = true;
      QPointF previous;
      for (const auto & p : trace.points) {
        if (!std::isfinite(p.y())) {
          move = true;
        } else if (trace.series.points_only) {
          painter.drawPoint(QPointF(xToPx(p.x()), yToPx(p.y())));
        } else if (move) {
          const QPointF pixel(xToPx(p.x()), yToPx(p.y()));
          path.moveTo(pixel);
          painter.drawPoint(pixel);
          previous = pixel;
          move = false;
        } else {
          const QPointF pixel(xToPx(p.x()), yToPx(p.y()));
          if (trace.series.step) {
            path.lineTo(pixel.x(), previous.y());
          }
          path.lineTo(pixel);
          previous = pixel;
        }
      }
      painter.drawPath(path);
    }
    painter.restore();

    if (area.contains(cursor_)) {
      const double time = left_ + (cursor_.x() - area.left()) * (right_ - left_) / area.width();
      painter.setPen(QPen(foreground, 1, Qt::DotLine));
      painter.drawLine(QPointF(cursor_.x(), area.top()), QPointF(cursor_.x(), area.bottom()));
      QStringList lines{tr("x = %1").arg(fixedValue(time, 6))};
      for (const auto & trace : traces_) {
        const auto nearest = std::min_element(trace.points.begin(), trace.points.end(),
          [time](const QPointF & a, const QPointF & b) {
            const auto distance = [time](const QPointF & p) {
              return std::isfinite(p.y()) ? std::abs(p.x() - time) :
                     std::numeric_limits<double>::infinity();
            };
            return distance(a) < distance(b);
          });
        if (nearest != trace.points.end() && std::isfinite(nearest->y())) {
          lines.append(QString("%1  %2").arg(trace.series.label, fixedValue(nearest->y())));
        }
      }
      const int line_height = painter.fontMetrics().height();
      const int limit = std::min(lines.size(), std::max(2, static_cast<int>(area.height()) / line_height));
      lines = lines.mid(0, limit);
      int width = 0;
      for (const auto & line : lines) {
        width = std::max(width, painter.fontMetrics().horizontalAdvance(line));
      }
      QPointF origin = cursor_ + QPointF(11, 10);
      if (origin.x() + width + 12 > area.right()) {
        origin.setX(cursor_.x() - width - 18);
      }
      origin.setY(std::min(origin.y(), area.bottom() - limit * line_height - 12));
      const QRectF label(origin, QSizeF(width + 12, limit * line_height + 8));
      painter.fillRect(label, palette().window());
      painter.setPen(foreground);
      painter.drawRect(label);
      painter.drawText(label.adjusted(6, 4, -6, -4), Qt::AlignLeft, lines.join('\n'));
    }
  }

  void mouseMoveEvent(QMouseEvent * event) override
  {
    cursor_ = event->localPos();
    if (event->buttons() & Qt::LeftButton && pan) {
      pan((last_x_ - cursor_.x()) / bounds().width() * (right_ - left_));
    }
    last_x_ = cursor_.x();
    update();
  }

  void mousePressEvent(QMouseEvent * event) override {last_x_ = event->localPos().x();}
  void mouseDoubleClickEvent(QMouseEvent *) override {if (reset) {reset();}}
  void leaveEvent(QEvent *) override {cursor_ = {-1, -1}; update();}
  void wheelEvent(QWheelEvent * event) override
  {
    if (zoom) {
      zoom(std::pow(0.8, event->angleDelta().y() / 120.0));
      event->accept();
    }
  }

private:
  QRectF bounds() const
  {
    const auto metrics = fontMetrics();
    const int labels = metrics.horizontalAdvance(QStringLiteral("-99.9e+99")) + 8;
    const int row = metrics.height() + 3;
    return rect().adjusted(labels + 7, row + 8, -labels / 2 - 4, -row - 8);
  }
  std::vector<Trace> traces_;
  QString unit_;
  QString axis_;
  double left_{};
  double right_{1.0};
  double minimum_span_{0.0};
  double error_band_{0.0};
  double last_x_{};
  QPointF cursor_{-1, -1};
};

QByteArray quoted(QByteArray cell)
{
  cell.replace("\"", "\"\"");
  return "\"" + cell + "\"";
}

void flatten(QMap<QString, QJsonValue> & cells, const QString & path, const QJsonValue & value)
{
  if (value.isArray()) {
    const auto array = value.toArray();
    for (int i = 0; i < array.size(); ++i) {
      flatten(cells, QString("%1.%2").arg(path).arg(i), array[i]);
    }
  } else if (value.isObject()) {
    const auto object = value.toObject();
    for (auto it = object.begin(); it != object.end(); ++it) {
      flatten(cells, path.isEmpty() ? it.key() : path + "." + it.key(), it.value());
    }
  } else {
    cells.insert(path, value);
  }
}

QByteArray csvCell(const QJsonValue & value)
{
  if (value.isNull() || value.isUndefined()) {
    return {};
  }
  if (value.isDouble()) {
    return QByteArray::number(value.toDouble(), 'g', 17);
  }
  if (value.isBool()) {
    return value.toBool() ? "true" : "false";
  }
  auto text = value.toString().toUtf8();
  if (!text.isEmpty() && QByteArray("=+-@").contains(text.at(0))) {
    text.prepend('\'');
  }
  return quoted(text);
}

QString tableField(QString text)
{
  return text.replace('|', "&#124;").replace('\n', " ");
}

bool writeArtifact(const QDir & dir, const QString & name, const QByteArray & bytes, QString * error)
{
  QSaveFile file(dir.filePath(name));
  if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) {
    if (error) {
      *error = QString("%1 · %2").arg(name, file.errorString());
    }
    return false;
  }
  return true;
}

enum Page
{
  graphs_page, pose_page, jacobian_page, singularity_page, profile_page,
  events_page, diagnostics_page, evidence_page
};

enum Group
{
  joint_group, velocity_group, acceleration_group, sigma_group, torque_group, error_group, jerk_group
};

QString pointName(const QString & point)
{
  return point == "flange" ? QStringLiteral("Brida") : QStringLiteral("TCP activo");
}

QString poseBlock(const QString & name, const QJsonValue & value, double angle_factor, const QString & unit)
{
  const auto pose = value.toObject();
  if (pose.isEmpty()) {
    return name + QStringLiteral(" · N/D");
  }
  return QString("%1\nXYZ [m]       %2\nRPY [%3]      %4\nCuaternión xyzw %5\n%6")
         .arg(name, vectorText(pose.value("xyz")), unit, vectorText(pose.value("rpy"), angle_factor),
    vectorText(pose.value("xyzw")), matrixText(pose.value("matrix")));
}

QString routeDetails(const QJsonObject & route, double factor, const QString & unit)
{
  QStringList rows;
  if (!route.value("error").toString().isEmpty()) {
    rows.append(route.value("error").toString());
  }
  const auto branches = route.value("routes").toArray();
  for (const auto & value : branches) {
    const auto branch = value.toObject();
    const bool rejected = !branch.value("reason").toString().isEmpty();
    rows << QString("%1 · %2 · %3\n  J1–J6 [%4]    %5\n  Residuo FK    %6 m · %7 %4\n"
                    "  LSPB          %8 s · costo %9 · %10")
      .arg(jsonText(branch.value("branch")), jsonText(branch.value("source")),
        rejected ? branch.value("reason").toString() : QStringLiteral("Candidata"),
        unit, vectorText(branch.value("q"), factor), jsonText(branch.value("position_error_m")),
        fixedValue(branch.value("orientation_error_rad").toDouble(missing) * factor),
        jsonText(branch.value("duration")), jsonText(branch.value("score")))
      .arg(jsonText(branch.value("collision")));
  }
  return rows.isEmpty() ? QStringLiteral("N/D · Calcule IK y rutas para ver las ramas.") : rows.join("\n\n");
}
}  // namespace

struct AnalysisWidget::Impl
{
  explicit Impl(AnalysisWidget * widget)
  : owner(widget)
  {
    auto * layout = new QVBoxLayout(owner);
    layout->setContentsMargins(5, 5, 5, 5);
    tabs = new QTabWidget(owner);
    layout->addWidget(tabs);
    setupGraphs();
    setupPage(tr("FK e IK"), tr("Poses del modelo y residuos internos de las candidatas Pieper. "
        "RPY ZYX, cuaternión xyzw; no hay referencia independiente cargada."));
    setupPage(tr("Jacobiano"), tr("Jacobiano geométrico en el marco base. "
        "Jv en m/rad y Jω en rad/rad; barrido de diferencias finitas centrales."));
    setupPage(tr("Singularidades"), tr("SVD de Js = [Jv/L; Jω]. "
        "σ y κ adimensionales; la dirección calculada pertenece al espacio de twist escalado."));
    setupPage(tr("Plan / dinámica"), tr("Muestra del plan en el cursor de previsualización. "
        "Dinámica docente 1R con parámetros sintéticos."));
    setupPage(tr("Eventos / errores"), tr("Cambios de estado y errores recibidos, con tiempo ROS. "
        "Incluye incidencias de cálculo y cruces de vigencia."));
    setupPage(tr("Diagnóstico"), tr("Estado de la sesión, procedencia de las muestras y "
        "datos del transporte cuando están disponibles."));
    setupPage(tr("Evidencia"), tr("Guarde el historial adquirido, la gráfica actual y las tablas S14 "
        "con sus referencias y estados de revisión."));
    auto * evidence_form = new QHBoxLayout;
    operator_id = new QLineEdit;
    team = new QLineEdit;
    operator_id->setPlaceholderText(tr("Operador"));
    operator_id->setAccessibleName(tr("Operador del ensayo"));
    team->setPlaceholderText(tr("Equipo"));
    team->setAccessibleName(tr("Equipo del ensayo"));
    export_run = new QComboBox;
    export_run->setAccessibleName(tr("Alcance de la exportación"));
    export_run->addItem(tr("Toda la captura"), QString());
    auto * export_button = new QPushButton(tr("Exportar evidencia…"));
    export_button->setObjectName("exportarEnsayo");
    export_status = new QLabel;
    export_status->setWordWrap(true);
    evidence_form->addWidget(operator_id);
    evidence_form->addWidget(team);
    evidence_form->addWidget(export_button);
    detail_layouts[evidence_page - 1]->insertLayout(1, evidence_form);
    detail_layouts[evidence_page - 1]->insertWidget(2, export_run);
    detail_layouts[evidence_page - 1]->addWidget(export_status);
    connect(export_button, &QPushButton::clicked, owner, [this]() {
      const QString dir = QFileDialog::getExistingDirectory(owner, tr("Carpeta de evidencia"));
      if (dir.isEmpty()) {
        return;
      }
      QString error;
      const bool ok = owner->exportEvidence(dir, &error, export_run->currentData().toString());
      export_status->setText(ok ? tr("Ensayo guardado en %1").arg(exported_path) : error);
    });
    connect(tabs, &QTabWidget::currentChanged, owner, [this]() {refresh();});
    auto * timer = new QTimer(owner);
    connect(timer, &QTimer::timeout, owner, [this]() {
      if (dirty && owner->isVisible()) {
        refresh();
      }
    });
    timer->start(120);
    refresh();
  }

  void setupPage(const QString & title, const QString & description)
  {
    auto * page = new QWidget;
    auto * layout = new QVBoxLayout(page);
    layout->setContentsMargins(3, 3, 3, 3);
    auto * intro = new QLabel(description);
    intro->setWordWrap(true);
    layout->addWidget(intro);
    auto * text = new QPlainTextEdit;
    text->setReadOnly(true);
    text->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    text->setLineWrapMode(QPlainTextEdit::NoWrap);
    text->setAccessibleName(title);
    text->setMinimumHeight(75);
    layout->addWidget(text, 1);
    detail_layouts.push_back(layout);
    details.push_back(text);
    tabs->addTab(page, title);
  }

  void setupGraphs()
  {
    auto * page = new QWidget;
    auto * layout = new QVBoxLayout(page);
    layout->setContentsMargins(3, 3, 3, 3);
    layout->setSpacing(3);
    auto * controls = new QGridLayout;
    group = new QComboBox;
    group->setAccessibleName(tr("Grupo de curvas"));
    group->addItems({tr("Posición q"), tr("Velocidad q̇"), tr("Aceleración q̈"),
      tr("Valores singulares"), tr("Dinámica 1R"), tr("Error al objetivo"), tr("Jerk de referencia")});
    dynamics_axis = new QComboBox;
    dynamics_axis->setAccessibleName(tr("Articulación del modelo 1R"));
    for (int i = 1; i <= 6; ++i) {
      dynamics_axis->addItem(QString("J%1").arg(i));
    }
    point = new QComboBox;
    point->setAccessibleName(tr("Punto de la gráfica"));
    point->addItem(tr("Brida"), "flange");
    point->addItem(tr("TCP activo"), "tcp");
    point->setCurrentIndex(0);
    view_mode = new QComboBox;
    view_mode->setAccessibleName(tr("Vista temporal de la gráfica"));
    view_mode->addItems({tr("Ventana en vivo"), tr("Ensayo completo (en memoria)"),
      tr("Comparar backends")});
    view_mode->setToolTip(tr("Ensayo completo utiliza las muestras conservadas; "
      "el búfer tiene un límite de 12000 muestras."));
    window = new QDoubleSpinBox;
    window->setAccessibleName(tr("Ventana temporal de la gráfica"));
    window->setRange(.5, 120.0);
    window->setDecimals(1);
    window->setSuffix(tr(" s"));
    window->setValue(10.0);
    auto * pause = new QPushButton(tr("Pausar gráfica"));
    pause->setCheckable(true);
    auto * reset = new QPushButton(tr("Centrar"));
    auto * clear = new QPushButton(tr("Limpiar historial"));
    clear->setObjectName("limpiarHistorial");
    clear->setToolTip(tr("Vacía muestras, ensayos, eventos y rutas exportables. "
      "El ensayo visible queda excluido hasta iniciar otro. Los archivos guardados se conservan."));
    // Two rows avoid overflowing the narrow analysis dock on laptop displays.
    controls->addWidget(group, 0, 0, 1, 2);
    controls->addWidget(point, 0, 2);
    controls->addWidget(view_mode, 0, 3);
    controls->addWidget(window, 0, 4);
    controls->addWidget(pause, 1, 0);
    controls->addWidget(reset, 1, 1);
    controls->addWidget(clear, 1, 2);
    controls->addWidget(dynamics_axis, 1, 3);
    controls->setColumnStretch(3, 1);
    layout->addLayout(controls);

    auto * selection = new QHBoxLayout;
    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
      joints[i] = new QCheckBox(QString("J%1").arg(i + 1));
      joints[i]->setChecked(i == 0);
      selection->addWidget(joints[i]);
      connect(joints[i], &QCheckBox::toggled, owner, [this]() {refresh();});
    }
    selection->addStretch();
    layout->addLayout(selection);

    plot = new Plot;
    plot_info = new QLabel;
    plot_info->setTextFormat(Qt::PlainText);
    plot_info->setWordWrap(true);
    legend = new QLabel;
    legend->setObjectName("analysisPlotLegend");
    legend->setWordWrap(true);
    layout->addWidget(plot, 1);
    layout->addWidget(legend);
    layout->addWidget(plot_info);
    tabs->addTab(page, tr("Gráficas"));

    connect(group, qOverload<int>(&QComboBox::currentIndexChanged), owner, [this]() {refresh();});
    connect(point, qOverload<int>(&QComboBox::currentIndexChanged), owner, [this]() {refresh();});
    connect(dynamics_axis, qOverload<int>(&QComboBox::currentIndexChanged), owner, [this]() {refresh();});
    connect(view_mode, qOverload<int>(&QComboBox::currentIndexChanged), owner, [this]() {
      offset = 0.0;
      refresh();
    });
    connect(window, qOverload<double>(&QDoubleSpinBox::valueChanged), owner, [this]() {refresh();});
    connect(reset, &QPushButton::clicked, owner, [this]() {
      offset = 0.0;
      refresh();
    });
    connect(clear, &QPushButton::clicked, owner, [this]() {
      history.clear();
      while (export_run->count() > 1) {
        export_run->removeItem(1);
      }
      events.clear();
      route_history.clear();
      end_markers.clear();
      cleared_run_id = latest.value("plan").toObject().value("run_id").toString();
      cleared_physical_id = valueAt(latest, "physical_run.id").toString();
      ++segment;
      export_status->clear();
      dropped = 0;
      offset = 0.0;
      frozen_time = last_time;
      current_trial = {};
      exported_path.clear();
      refresh();
    });
    connect(pause, &QPushButton::toggled, owner, [this, pause](bool active) {
      paused = active;
      frozen_time = last_time;
      pause->setText(active ? tr("Reanudar gráfica") : tr("Pausar gráfica"));
      refresh();
    });
    plot->zoom = [this](double multiplier) {
        if (view_mode->currentIndex() == 0) {
          window->setValue(window->value() * multiplier);
        }
      };
    plot->pan = [this](double seconds) {
        if (!history.empty() && view_mode->currentIndex() == 0)
        {
          const double available = (paused ? frozen_time : last_time) - history.front().time;
          offset = std::clamp(offset - seconds, 0.0, std::max(0.0, available));
          refresh();
        }
      };
    plot->reset = [this]() {offset = 0.0; refresh();};
  }

  void addSample(QJsonObject sample)
  {
    const auto elapsed = sample.value("elapsed_s");
    if (!elapsed.isDouble() || !std::isfinite(elapsed.toDouble())) {
      return;
    }
    const auto stamp = sample.value("timestamp_ns");
    if (!latest.isEmpty() && stamp == latest.value("timestamp_ns") && elapsed == latest.value("elapsed_s")) {
      return;
    }

    const auto incidents = sample.take("incident_history").toArray();
    const auto resolution = sample.take("incident_resolution").toObject();
    const auto end_marker = sample.take("preview_end").toObject();
    const auto physical_id = valueAt(sample, "physical_run.id").toString();
    const bool closed_duplicate = !physical_id.isEmpty() &&
      (physical_id == last_closed_physical_id || physical_id == cleared_physical_id);
    if (closed_duplicate) {
      sample.remove("physical_run");
    } else if (valueAt(sample, "physical_run.finished").toBool()) {
      last_closed_physical_id = physical_id;
    }
    const auto packet = sample.take("route_result").toObject();
    if (!packet.isEmpty()) {
      addRoutes(packet, stamp);
    }
    const auto trial_stamp = sample.value("timestamp_ns_text").toString();
    const auto record = [&](const QString & type, const QString & id, const QString & result) {
        if (id.isEmpty()) {
          return;
        }
        if (observed_trials.value(type) != id) {
          observed_trials.insert(type, id);
          current_trial.trial_id = id;
          current_trial.trial_type = type;
          current_trial.timestamp_ns = trial_stamp;
        }
        if (current_trial.trial_id == id) {
          current_trial.trial_result = result;
        }
      };
    const auto route_error = packet.value("error").toString();
    record(tr("IK"), sample.value("pose_plan_id").toString(),
      !route_error.isEmpty() ? route_error :
      !packet.value("routes").toArray().isEmpty() ? tr("Ramas disponibles") : QString());
    const auto plan = sample.value("plan").toObject();
    const auto task = sample.value("task").toObject();
    const auto run_id_for_summary = plan.value("run_id").toString();
    const auto task_preview = task.value("preview_run_id").toString();
    if (!run_id_for_summary.isEmpty() && run_id_for_summary != task_preview) {
      const auto phase = end_marker.value("run_id").toString() == run_id_for_summary ?
        end_marker.value("phase").toString() : plan.value("phase").toString();
      record(plan.value("profile_id").toString() == "quintic" ? tr("Ruta quíntica") :
        tr("Ruta LSPB"), run_id_for_summary, phase);
    }
    const auto review = sample.value("fk_review").toObject();
    record(tr("FK"), fkTrialId(sample), review.value("result").toString(tr("Calculada")));
    const auto physical = sample.value("individual_trial").toObject();
    record(physical.value("backend").toString() == "rws_trajectory" ?
      tr("Control físico · RWS v2") : tr("Control físico"),
      physical.value("trial_id").toString(), physical.value("result").toString());
    record(tr("Tarea"), taskTrialId(task), task.value("state").toString());
    if (!physical_id.isEmpty() && !closed_duplicate) {
      sample.insert("evidence_operation_id", physical_id);
    } else if (!current_trial.trial_id.isEmpty() && current_trial.trial_id != physical_id &&
      reportsTrial(sample, current_trial.trial_id))
    {
      sample.insert("evidence_operation_id", current_trial.trial_id);
    }
    const double received_time = elapsed.toDouble();
    if (!latest.isEmpty() && received_time < latest.value("elapsed_s").toDouble()) {
      time_origin = last_time - received_time + .1;
      ++segment;
    }
    // Each preview or time-cursor jump is a discontinuity, not robot motion.
    // Keep this boundary in exported JSON/CSV as well as in the plot.
    if (!latest.isEmpty() &&
      (sample.value("preview_epoch") != latest.value("preview_epoch") ||
      sample.value("preview_run_id") != latest.value("preview_run_id") ||
      valueAt(sample, "physical_run.id") != valueAt(latest, "physical_run.id") ||
      valueAt(sample, "physical_run.reference.sequence") != valueAt(latest, "physical_run.reference.sequence") ||
      sample.value("source") != latest.value("source") ||
      sample.value("mode") != latest.value("mode") ||
      sample.value("backend") != latest.value("backend") ||
      valueAt(sample, "physical_run.reference.origin") != valueAt(latest, "physical_run.reference.origin") ||
      sample.value("model_sha256") != latest.value("model_sha256") ||
      sample.value("tool_offset") != latest.value("tool_offset")))
    {
      ++segment;
    }
    last_time = received_time + time_origin;
    sample.insert("plot_time_s", last_time);
    sample.insert("plot_segment", segment);
    latest = sample;
    const auto run_id = latest.value("plan").toObject().value("run_id").toString();
    if (cleared_run_id.isEmpty() || run_id != cleared_run_id) {
      if (run_id.isEmpty() && !cleared_run_id.isEmpty() &&
        sample.value("preview_run_id").toString() == cleared_run_id)
      {
        sample.remove("preview_run_id");
      }
      history.push_back({last_time, segment, std::move(sample)});
      if (!run_id.isEmpty() && export_run->findData(run_id) < 0) {
        export_run->addItem(run_id, run_id);
      }
      if (!physical_id.isEmpty() && !closed_duplicate && export_run->findData(physical_id) < 0) {
        const auto backend = latest.value("backend").toString();
        export_run->addItem(tr("Orden o lote %1 · %2").arg(
          backend == "rws_trajectory" ? tr("v2") : tr("Normal"), physical_id.left(8)), physical_id);
      }
      if (history.size() > history_limit) {
        const auto dropped_sample = history.front().data;
        history.pop_front();
        ++dropped;
        for (const auto & path : {"plan.run_id", "physical_run.id"}) {
          const auto removed = valueAt(dropped_sample, path);
          if (removed.isString() &&
            std::none_of(history.begin(), history.end(), [&](const Sample & record) {
              return valueAt(record.data, path) == removed;
            }))
          {
            export_run->removeItem(export_run->findData(removed.toString()));
            end_markers.remove(removed.toString());
          }
        }
      }
    }

    const auto ended_run = end_marker.value("run_id").toString();
    if (!ended_run.isEmpty() && export_run->findData(ended_run) > 0) {
      end_markers.insert(ended_run, end_marker);
    }

    const auto operation_id = current_trial.trial_id;
    const bool operation_retained = !operation_id.isEmpty() && belongsToTrial(latest, operation_id);
    const auto recordIncident = [&](const char * key, QJsonObject & previous, const QString & state) {
        auto incident = latest.value(key).toObject();
        if (!incident.isEmpty() && incident != previous) {
          previous = incident;
          incident.insert("estado_incidente", state);
          incident.insert("ensayo_al_observar", run_id);
          if (operation_retained) {
            incident.insert("ensayo_operacion", operation_id);
          }
          events.push_back(incident);
        }
      };
    if (incident_history != incidents) {
      for (const auto & entry : incidents) {
        if (incident_history.contains(entry)) {
          continue;
        }
        auto incident = entry.toObject();
        incident.insert("estado_incidente", incident.value("state").toString() == "resolved" ?
          QStringLiteral("resuelto") : QStringLiteral("reemplazado"));
        events.push_back(incident);
      }
      incident_history = incidents;
    }
    if (!resolution.isEmpty() && latest_resolution != resolution) {
      latest_resolution = resolution;
      if (!incident_history.contains(resolution)) {
        auto incident = resolution;
        incident.insert("estado_incidente", QStringLiteral("resuelto"));
        events.push_back(incident);
      }
    }
    recordIncident("incident", latest_incident, QStringLiteral("vigente"));
    auto diagnostic = latest.value("diagnostic").toObject();
    const auto status = latest.value("status").toString();
    const QString signature = QString("%1|%2|%3|%4|%5").arg(
      jsonText(diagnostic.value("severity")), jsonText(diagnostic.value("code")),
      jsonText(diagnostic.value("detail")), jsonText(latest.value("source")),
      jsonText(latest.value("stale")));
    if (signature != event_signature) {
      diagnostic.insert("timestamp_ns", stamp);
      diagnostic.insert("source", latest.value("source"));
      diagnostic.insert("status", status);
      diagnostic.insert("ensayo_al_observar", run_id);
      if (operation_retained) {
        diagnostic.insert("ensayo_operacion", operation_id);
      }
      events.push_back(diagnostic);
      event_signature = signature;
    }
    while (events.size() > events_limit) {
      events.pop_front();
    }
    dirty = true;
  }

  void addRoutes(const QJsonObject & result, const QJsonValue & received_stamp)
  {
    if (result.isEmpty() || routes == result) {
      return;
    }
    routes = result;
    auto snapshot = result;
    snapshot.insert("observado_ros_ns", received_stamp);
    route_history.push_back(snapshot);
    if (route_history.size() > 100) {
      route_history.pop_front();
    }
    dirty = true;
  }

  std::vector<Series> seriesFor(int selected, const QString & frame) const
  {
    const double angle = radians ? 1.0 : degrees_per_rad;
    const std::array<QColor, 6> colors{
      QColor("#1381bc"), QColor("#d46234"), QColor("#32956c"), QColor("#9358c9"),
      QColor("#ad8518"), QColor("#c64c85")};
    std::vector<Series> output;
    if (selected == torque_group) {
      const int index = dynamics_axis->currentIndex();
      output.push_back({tr("Par ideal 1R · J%1").arg(index + 1), QString::number(index),
          colors[index], Qt::DashLine, 1.0, false, 1.6, SeriesSource::TorqueModel});
      return output;
    }
    for (int i = 0; i < 6; ++i) {
      const QString suffix = "." + QString::number(i);
      if (selected == sigma_group) {
        const QString label = tr("σ%1").arg(i + 1);
        output.push_back({label + tr(" · actual"), frame + suffix, colors[i],
            Qt::SolidLine, 1.0, true, 2.6, SeriesSource::SigmaActual});
        output.push_back({label + tr(" · LSPB"), frame + suffix, colors[i],
            Qt::DashLine, 1.0, false, 1.6, SeriesSource::SigmaReference});
        output.push_back({label + tr(" · quíntico"), frame + suffix, colors[i],
            Qt::DotLine, 1.0, false, 1.6, SeriesSource::SigmaReference, false, true});
        continue;
      }
      if (!joints[i]->isChecked()) {
        continue;
      }
      const auto joint = QStringLiteral("J%1").arg(i + 1);
      if (selected == jerk_group) {
        output.push_back({joint + tr(" · jerk quíntico"), "plan.profile_jerk" + suffix,
            colors[i], Qt::DotLine, angle, false, 1.6, SeriesSource::Value, false, true});
        continue;
      }
      if (selected == joint_group) {
        output.push_back({joint + tr(" · simulada"), "actual" + suffix, colors[i],
            Qt::DashLine, angle, true, 2.6, SeriesSource::VirtualPosition});
        output.push_back({joint + tr(" · medida Normal"), "actual" + suffix, colors[i],
            Qt::SolidLine, angle, true, 2.6, SeriesSource::NormalPosition});
        output.push_back({joint + tr(" · medida v2"), "actual" + suffix, colors[i].darker(145),
            Qt::SolidLine, angle, true, 2.6, SeriesSource::TrajectoryPosition});
        output.push_back({joint + tr(" · muestra externa"), "actual" + suffix, colors[i],
            Qt::SolidLine, angle, true, 3.6, SeriesSource::OtherPosition, false, false, true});
        output.push_back({joint + tr(" · medida mock"), "actual" + suffix, colors[i],
            Qt::SolidLine, angle, true, 3.6, SeriesSource::MockPosition, false, false, true});
        output.push_back({joint + tr(" · objetivo"), "target" + suffix, colors[i].darker(130),
            Qt::DashDotLine, angle, false, 1.3, SeriesSource::Value, true});
        output.push_back({joint + tr(" · consigna RWS confirmada"), "individual_trial.motion.target_rad" + suffix,
            colors[i].darker(165), Qt::SolidLine, angle, false, 4.0, SeriesSource::SentJointGoal,
            false, false, true});
      } else if (selected == velocity_group) {
        output.push_back({joint + tr(" · simulada"), "virtual_motion.qdot" + suffix, colors[i],
            Qt::DashLine, angle, true, 2.6, SeriesSource::VirtualDerivative});
      } else if (selected == acceleration_group) {
        output.push_back({joint + tr(" · simulada"), "virtual_motion.qddot" + suffix, colors[i],
            Qt::DashLine, angle, true, 2.6, SeriesSource::VirtualDerivative, true});
      } else if (selected == error_group) {
        output.push_back({joint + tr(" · error final RWS"), "transmitted_goal.final_error" + suffix,
            colors[i].darker(145), Qt::SolidLine, angle, false, 4.0, SeriesSource::FinalError,
            false, false, true});
        output.push_back({joint + tr(" · consigna/objetivo menos postura"), suffix, colors[i],
            Qt::SolidLine, angle, true, 3.6, SeriesSource::GoalError, false, false, true});
        output.push_back({joint + tr(" · referencia host menos medida"), "physical_run.reference_error_rad" + suffix,
            colors[i].lighter(145), Qt::DashLine, angle, true, 1.6, SeriesSource::Value});
        continue;
      }
      if (selected == velocity_group || selected == acceleration_group) {
        const QString measured = (selected == velocity_group ? "measured_qdot" : "measured_qddot") + suffix;
        output.push_back({joint + tr(" · estimada Normal"), measured, colors[i],
            Qt::SolidLine, angle, true, 2.6, SeriesSource::NormalPosition});
        output.push_back({joint + tr(" · estimada v2"), measured, colors[i].darker(145),
            Qt::SolidLine, angle, true, 2.6, SeriesSource::TrajectoryPosition});
        output.push_back({joint + tr(" · estimada mock"), measured, colors[i],
            Qt::SolidLine, angle, true, 2.6, SeriesSource::MockPosition});
        output.push_back({joint + tr(" · estimada externa"), measured, colors[i],
            Qt::SolidLine, angle, true, 2.6, SeriesSource::OtherPosition});
      }
      const QString reference = selected == joint_group ? "plan.q" : selected == velocity_group ?
        "plan.profile_qdot" : "plan.profile_qddot";
      output.push_back({joint + tr(" · LSPB"), reference + suffix, colors[i].lighter(145), Qt::DashLine, angle,
          false, 1.6, SeriesSource::Value, selected == acceleration_group});
      output.push_back({joint + tr(" · quíntico"), reference + suffix, colors[i].lighter(145), Qt::DotLine, angle,
          false, 1.6, SeriesSource::Value, false, true});
    }
    return output;
  }

  static double sampleValue(const QJsonObject & object, const Series & series)
  {
    if (series.respect_stale && object.value("stale").toBool()) {
      return missing;
    }
    const auto physical = object.value("physical_run").toObject();
    if (!physical.isEmpty() && series.path.startsWith("target.")) {
      return valueAt(physical, "goal_rad." + series.path.section('.', -1)).toDouble(missing) *
             series.factor;
    }
    // Only the mathematical reference series is replaced. Actual values and
    // derivative estimates still come from the original RWS measurement.
    if (!physical.isEmpty() && series.source == SeriesSource::Value &&
      (series.path.startsWith("plan.q.") || series.path.startsWith("plan.profile_qdot.") ||
      series.path.startsWith("plan.profile_qddot.")))
    {
      const auto reference = physical.value("reference").toObject();
      if (reference.isEmpty() || (reference.value("profile_id").toString() == "quintic") != series.quintic_only) {
        return missing;
      }
      return valueAt(reference, series.path.mid(5)).toDouble(missing) * series.factor;
    }
    if ((series.path.startsWith("plan.") && series.source == SeriesSource::Value) ||
      series.source == SeriesSource::SigmaReference)
    {
      const bool quintic = valueAt(object, "plan.profile_id").toString() == "quintic";
      if (quintic != series.quintic_only) {
        return missing;
      }
    }
    const bool offline = object.value("mode").toString() == "offline";
    const auto source = object.value("source").toString();
    const auto rws_goal = object.value("individual_trial").toObject();
    const bool mock = source.contains("mock", Qt::CaseInsensitive);
    switch (series.source) {
      case SeriesSource::VirtualPosition:
      case SeriesSource::VirtualDerivative:
        return offline ? valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::NormalPosition:
        return !offline && source == "rws" ?
          valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::TrajectoryPosition:
        return !offline && source == "rws_trajectory" ?
          valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::OtherPosition:
        return !offline && !mock && (source == "external" || source == "ROS externo") ?
          valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::MockPosition:
        return !offline && mock ?
          valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::SentJointGoal: {
        const auto id = object.value("transmitted_goal").toObject().value("id").toString();
        if (!id.isEmpty()) {
          return !offline && !mock ?
            valueAt(object, "transmitted_goal.q." + series.path.section('.', -1)).toDouble(missing) *
            series.factor : missing;
        }
        return !offline && !mock && !rws_goal.value("trial_id").toString().isEmpty() &&
          rws_goal.value("trial_id") == object.value("evidence_operation_id") &&
          rws_goal.value("backend") == object.value("backend") &&
          valueAt(rws_goal, "motion.submit_confirmed").toBool() ?
          valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      }
      case SeriesSource::FinalError:
        return !offline && !mock ? valueAt(object, series.path).toDouble(missing) * series.factor : missing;
      case SeriesSource::GoalError: {
        if (!physical.isEmpty()) {
          return (valueAt(physical, "goal_rad" + series.path).toDouble(missing) -
            valueAt(object, "actual" + series.path).toDouble(missing)) * series.factor;
        }
        const auto error = valueAt(object, "arrival.error_rad" + series.path);
        if (error.isDouble()) {
          return error.toDouble() * series.factor;
        }
        return (valueAt(object, "target" + series.path).toDouble(missing) -
          valueAt(object, "actual" + series.path).toDouble(missing)) * series.factor;
      }
      default:
        break;
    }
    if (series.source == SeriesSource::Value) {
      if ((series.path.startsWith("measured_qdot.") || series.path.startsWith("measured_qddot.")) &&
        object.value("mode").toString() == "offline")
      {
        return missing;
      }
      auto field = valueAt(object, series.path);
      // Historical captures predate the ideal derivatives of a paused profile.
      if (field.isUndefined() && series.path.startsWith("plan.profile_")) {
        QString legacy = series.path;
        field = valueAt(object, legacy.remove(QStringLiteral("profile_")));
      }
      return field.toDouble(missing) * series.factor;
    }
    if (series.source == SeriesSource::TorqueModel) {
      const auto plan = object.value("plan").toObject();
      const auto dynamics = plan.value("dynamics_1r").toObject();
      const auto q = valueAt(plan, "q." + series.path).toDouble(missing);
      auto qddot = valueAt(plan, "profile_qddot." + series.path);
      if (qddot.isUndefined()) {
        qddot = valueAt(plan, "qddot." + series.path);
      }
      const double inertia = dynamics.value("inertia_kg_m2").toDouble(missing);
      const double mass = dynamics.value("mass_kg").toDouble(missing);
      const double radius = dynamics.value("com_radius_m").toDouble(missing);
      const double gravity = dynamics.value("gravity_m_s2").toDouble(missing);
      if (!(inertia > 0.0 && mass > 0.0 && radius > 0.0 && gravity > 0.0)) {
        return missing;
      }
      return inertia * qddot.toDouble(missing) + mass * gravity * radius * std::sin(q);
    }
    const bool actual = series.source == SeriesSource::SigmaActual;
    const auto jacobian_source = object.value("jacobian_source").toString();
    if ((actual && jacobian_source == "actual") || (!actual && jacobian_source == "plan_preview")) {
      const auto index = series.path.lastIndexOf('.');
      const auto path = series.path.left(index) + ".sigma" + series.path.mid(index);
      return valueAt(object, "jacobian." + path).toDouble(missing);
    }
    return missing;
  }

  QString axisUnit(int selected) const
  {
    const QString angular = radians ? QStringLiteral("rad") : QStringLiteral("°");
    switch (selected) {
      case joint_group: return angular;
      case error_group: return angular;
      case velocity_group: return angular + "/s";
      case acceleration_group: return angular + "/s²";
      case jerk_group: return angular + "/s³";
      case torque_group: return "N·m";
      default: return tr("adimensional");
    }
  }

  void refreshGraph(const std::deque<Sample> * selection = nullptr)
  {
    const bool whole_run = view_mode->currentIndex() == 1;
    std::deque<Sample> physical_selection;
    if (!selection && whole_run && latest.value("mode").toString() == "control") {
      QString id;
      for (auto it = history.rbegin(); it != history.rend(); ++it) {
        if (it->data.value("backend") == latest.value("backend") &&
          it->data.value("session_id") == latest.value("session_id"))
        {
          id = valueAt(it->data, "physical_run.id").toString();
          if (!id.isEmpty()) {
            break;
          }
        }
      }
      if (!id.isEmpty()) {
        for (const auto & sample : history) {
          if (valueAt(sample.data, "physical_run.id").toString() == id) {
            physical_selection.push_back(sample);
            if (valueAt(sample.data, "physical_run.finished").toBool()) {
              break;
            }
          }
        }
        selection = &physical_selection;
      }
    }
    const auto & history = selection ? *selection : this->history;
    const auto & latest = selection ? history.back().data : this->latest;
    const bool paused = !selection && this->paused;
    const auto physical = latest.value("physical_run").toObject();
    const double session_offset = selection && !physical.isEmpty() ?
      history.back().time - latest.value("elapsed_s").toDouble(history.back().time) : 0.0;
    const double graph_origin = selection && !physical.isEmpty() ?
      physical.value("started_s").toDouble() + session_offset : 0.0;
    const int selected = group->currentIndex();
    const QString frame = point->currentData().toString();
    const bool by_joint = selected == joint_group || selected == velocity_group ||
      selected == acceleration_group || selected == error_group || selected == jerk_group;
    for (auto * joint : joints) {
      joint->setVisible(by_joint);
    }
    dynamics_axis->setVisible(selected == torque_group);
    point->setVisible(selected == sigma_group);
    const bool current_context = !selection && view_mode->currentIndex() == 0;
    window->setEnabled(!whole_run);
    double right = std::max(window->value(), (paused ? frozen_time : last_time) - offset);
    double left = right - window->value();
    if (selection) {
      left = history.front().time - .05;
      right = std::max(history.back().time + .05, left + .5);
    } else if (whole_run && !history.empty()) {
      const QString run_id = latest.value("preview_run_id").toString();
      bool found = false;
      double first_time = 0.0;
      double end_time = 0.0;
      for (const auto & record : history) {
        const auto plan_record = record.data.value("plan").toObject();
        if (run_id.isEmpty() || plan_record.value("run_id").toString() != run_id) {
          continue;
        }
        if (!found) {
          first_time = record.time;
          found = true;
        }
        end_time = record.time;
        if (plan_record.value("phase").toString() == QStringLiteral("completed")) {
          break;  // Ignore periodic idle packets after reaching the endpoint.
        }
      }
      if (found) {
        left = first_time - .05;
        right = std::max(end_time + .05, left + .5);
      }
    }
    auto first = std::lower_bound(history.begin(), history.end(), left,
      [](const Sample & sample, double time) {return sample.time < time;});
    const auto end = std::upper_bound(first, history.end(), right,
      [](double time, const Sample & sample) {return time < sample.time;});
    if (first != history.begin()) {
      --first;
    }
    const auto count = std::distance(first, end);
    const auto stride = std::max<decltype(count)>(1, count / std::max(1, 2 * plot->width()));
    const auto specs = seriesFor(selected, frame);
    std::vector<Trace> traces;
    QStringList captions;
    for (const auto & spec : specs) {
      Trace trace{spec, {}};
      int previous_segment = -1;
      QString previous_stamp;
      double previous_measurement = missing;
      bool has_data = false;
      for (auto it = first; it != end;) {
        if (previous_segment >= 0 && it->segment != previous_segment) {
          trace.points.emplace_back(it->time - graph_origin, missing);
          previous_stamp.clear();
          previous_measurement = missing;
        }
        previous_segment = it->segment;
        const bool matching_context = it->data.value("backend") == latest.value("backend") &&
          it->data.value("mode") == latest.value("mode") &&
          it->data.value("session_id") == latest.value("session_id");
        const double value = (!current_context || matching_context) ? sampleValue(it->data, spec) : missing;
        has_data = has_data || std::isfinite(value);
        QString stamp = it->data.value("actual_stamp_ns_text").toString();
        if (spec.source == SeriesSource::SentJointGoal || spec.source == SeriesSource::FinalError) {
          stamp = valueAt(it->data, "transmitted_goal.id").toString(
            it->data.value("evidence_operation_id").toString());
        }
        const bool physical_sample = spec.source == SeriesSource::TrajectoryPosition ||
          spec.source == SeriesSource::NormalPosition;
        const bool reference = spec.source == SeriesSource::Value &&
          (spec.path.startsWith("plan.q.") || spec.path.startsWith("plan.profile_qdot.") ||
           spec.path.startsWith("plan.profile_qddot."));
        if ((!spec.points_only && !physical_sample) || !std::isfinite(value) ||
          stamp.isEmpty() || stamp != previous_stamp) {
          double x = it->time;
          const double packet_offset = it->time - it->data.value("elapsed_s").toDouble(it->time);
          if (physical_sample) {
            auto observed = it->data.value("actual_received_session_s");
            if (!observed.isDouble()) {
              observed = valueAt(it->data, "physical_run.measurement_session_s");
            }
            if (observed.isDouble()) {
              x = observed.toDouble() + packet_offset;
            }
            // Never connect two measured points across an acquisition outage.
            if (std::isfinite(previous_measurement) &&
              (x <= previous_measurement || x - previous_measurement > .5)) {
              trace.points.emplace_back(x - graph_origin, missing);
            }
            previous_measurement = std::isfinite(value) ? x : missing;
          } else if (reference) {
            auto evaluated = valueAt(it->data, "physical_run.reference.plot_session_s");
            if (!evaluated.isDouble()) {
              evaluated = valueAt(it->data, "physical_run.reference.measurement_session_s");
            }
            if (evaluated.isDouble()) {
              x = evaluated.toDouble() + packet_offset;
            }
          } else if (spec.source == SeriesSource::SentJointGoal) {
            const auto sent = valueAt(it->data, "physical_run.reference.submit_session_s");
            if (sent.isDouble()) {
              x = sent.toDouble() + packet_offset;
            }
          }
          trace.points.emplace_back(x - graph_origin, value);
          previous_stamp = std::isfinite(value) ? stamp : QString();
        }
        const auto remaining = std::distance(it, end);
        if (remaining <= 1) {
          break;
        }
        std::advance(it, std::min<decltype(stride)>(stride, remaining - 1));
      }
      if (has_data) {
        const QString line = spec.points_only ? QStringLiteral("•") : spec.style == Qt::DashLine ?
          QStringLiteral("– –") : spec.style == Qt::DotLine ? QStringLiteral("···") : QStringLiteral("━━");
        captions << QString("<span style=\"color:%1\">%2</span>&nbsp;%3")
          .arg(spec.color.name(), line, spec.label.toHtmlEscaped());
        traces.push_back(std::move(trace));
      }
    }
    legend->setText(captions.join(QStringLiteral(" &nbsp; ")));
    const double factor = radians ? 1.0 : degrees_per_rad;
    const double min_span = selected == joint_group || selected == velocity_group ||
      selected == acceleration_group ? (0.2 / degrees_per_rad) * factor : 0.0;
    const double error_band = selected == error_group ?
      latest.value("arrival_tolerance_rad").toDouble(0.0) * factor : 0.0;
    plot->data(std::move(traces), left - graph_origin, right - graph_origin, axisUnit(selected),
      graph_origin != 0.0 ? tr("Tiempo desde solicitud del ensayo [s]") : tr("Tiempo de sesión [s]"),
      min_span, error_band);
    const auto plan = latest.value("plan").toObject();
    const auto plan_time = plan.value("time_s");
    const bool offline = latest.value("mode").toString() == "offline";
    const auto base = latest.value("base_frame").toString();
    const auto source = latest.value("source").toString();
    const auto source_label = offline ? tr("Simulación Offline") :
      source.contains("mock", Qt::CaseInsensitive) ? tr("Telemetría mock") :
      source == "rws_trajectory" ? tr("Telemetría IRC5 · RWS v2") :
      source == "rws" ? tr("Telemetría IRC5") : jsonText(latest.value("source"));
    QString description = QString("%1 · %2 · %3").arg(
      source_label, utcTime(latest), base == "base_link" ? tr("Base del robot") :
      jsonText(latest.value("base_frame")));
    if (latest.value("stale").toBool()) {
      description += tr(" · Datos vencidos");
    }
    if (plan_time.isDouble()) {
      description += tr(" · Cursor del plan %1 / %2 s%3").arg(jsonText(plan_time),
        jsonText(plan.value("duration_s")), plan.value("paused").toBool() ? tr(" · en pausa") : QString());
    }
    if (selected == joint_group) {
      description += tr(" · Continua: posición física RWS (unión de muestras); discontinua: simulación/referencia. Sin unión sobre huecos.");
    } else if (selected == velocity_group) {
      if (!offline && (!latest.value("measured_qdot").isArray() || latest.value("stale").toBool())) {
        description += tr(" · Estimación actual no disponible en la última muestra.");
      }
      description += offline ?
        tr(" · Líneas discontinuas: simulación aplicada y referencia del plan.") :
        tr(" · Línea continua: estimación desde muestras. Discontinua: referencia del plan.");
      const auto filter = latest.value("measured_qdot_filter").toString();
      if (!offline && !filter.isEmpty()) {
        description += " " + filter;
      }
    } else if (selected == acceleration_group) {
      description += offline ?
        tr(" · Líneas discontinuas: simulación aplicada y referencia del plan.") :
        tr(" · Línea continua: aceleración estimada de muestras RWS (no medida por el servo). "
          "Discontinua: referencia del plan.");
    } else if (selected == sigma_group) {
      description += tr(" · σ1 ≥ … ≥ σ6. Valores calculados con el Jacobiano escalado en %1.")
        .arg(pointName(frame));
    } else if (selected == torque_group) {
      description += tr(" · Modelo educativo 1R. Entrada J%1 del perfil planificado. Par ideal, no medido.")
        .arg(dynamics_axis->currentIndex() + 1);
    } else if (selected == error_group) {
      description += tr(" · Error respecto a última consigna RWS o al objetivo seleccionado. "
        "La banda sombreada es de posición, no confirma el final RAPID. "
        "El seguimiento requiere referencia y medida sincronizadas.");
    }
    if (offline && (selected == velocity_group || selected == acceleration_group) &&
      !valueAt(latest, "virtual_motion.applied").toBool())
    {
      description += tr(" · Robot simulado en reposo.");
    }
    if (!physical.isEmpty()) {
      const auto origin = valueAt(physical, "reference.origin").toString();
      const auto timeLabel = [&physical](const QString & path) {
          const auto value = valueAt(physical, path);
          return value.isDouble() ? QString::number(value.toDouble(), 'f', 3) + " s" : tr("N/D");
        };
      description += tr(" · Ensayo %1 · tramo %2 · ref. %3 · solicitud ABB %4 · envío→acuse %5 · "
        "entrada en banda %6 · envío→liberación %7")
        .arg(physical.value("id").toString().left(8)).arg(physical.value("leg_index").toInt() + 1)
        .arg(timeLabel("reference.duration_s"), timeLabel("motion.requested_duration_s"),
          timeLabel("motion.submit_to_terminal_reply_s"), timeLabel("motion.submit_to_first_observed_band_s"),
          timeLabel("motion.submit_to_release_s"));
      const auto reference = physical.value("reference").toObject();
      const auto shape = reference.value("shape").toString();
      if (shape == "triangular" || shape == "trapezoidal") {
        description += tr(" · LSPB %1 · rampa %2 · crucero %3")
          .arg(shape == "triangular" ? tr("triangular") : tr("trapezoidal"),
               timeLabel("reference.ramp_s"), timeLabel("reference.cruise_s"));
      }
      const auto metrics = valueAt(physical, "tracking_metrics.leg").toObject();
      if (metrics.value("samples").toInt() > 0) {
        description += tr(" · RMSE J1–J6 %1° · error máximo J1–J6 %2° · %3 muestras RWS únicas (fase 2, intervalo de referencia)")
          .arg(metrics.value("rmse_rad").toDouble() * 180.0 / 3.141592653589793, 0, 'f', 3)
          .arg(metrics.value("max_error_rad").toDouble() * 180.0 / 3.141592653589793, 0, 'f', 3)
          .arg(metrics.value("samples").toInt());
        const auto errors = metrics.value("max_error_by_joint_rad").toArray();
        if (errors.size() == 6) {
          int worst = 0;
          for (int j = 1; j < 6; ++j) {
            if (errors[j].toDouble() > errors[worst].toDouble()) {worst = j;}
          }
          description += tr(" · mayor error: J%1").arg(worst + 1);
        }
      } else {
        description += tr(" · RMSE N/D: sin muestras correlacionadas dentro del intervalo de referencia.");
      }
      if (physical.value("backend").toString() == "rws_trajectory") {
        const auto revision = valueAt(physical, "motion.module_revision").toString();
        const auto ratio = valueAt(physical, "motion.lspb_acceleration_ratio_requested");
        if (!revision.isEmpty()) {
          description += tr(" · v2 %1").arg(revision == "lspb_candidate" ?
            tr("candidato LSPB") : revision == "continuous_candidate" ?
            tr("candidato continuo") : tr("original"));
        }
        if (ratio.isDouble()) {
          description += tr(" · factor AccSet solicitado %1 (no medido)")
            .arg(ratio.toDouble(), 0, 'f', 2);
        }
        if (valueAt(physical, "motion.controller_timing_s").isDouble()) {
          const auto instruction = valueAt(physical, "motion.motion_instruction").toString() == "MoveL" ?
            tr("MoveL") : tr("MoveAbsJ");
          description += tr(" · %5 reloj RAPID %1 · WaitRob %2 · AccSet base/aplicado %3/%4 %")
            .arg(timeLabel("motion.controller_timing_s"), timeLabel("motion.controller_wait_inpos_s"))
            .arg(valueAt(physical, "motion.controller_base_acc_percent").toDouble(), 0, 'f', 1)
            .arg(valueAt(physical, "motion.controller_applied_acc_percent").toDouble(), 0, 'f', 1)
            .arg(instruction);
        }
        const auto fit = valueAt(physical, "motion.lspb_equivalent_fit").toObject();
        if (fit.value("ramp_s").isDouble()) {
          description += tr(" · ajuste equivalente RWS: rampa %1 s · crucero %2 s · duración %3 s · razón a %4")
            .arg(fit.value("ramp_s").toDouble(), 0, 'f', 3)
            .arg(fit.value("cruise_s").toDouble(), 0, 'f', 3)
            .arg(fit.value("fitted_duration_s").toDouble(), 0, 'f', 3)
            .arg(fit.value("acceleration_ratio").toDouble(), 0, 'f', 2);
        } else {
          description += tr(" · ajuste de forma N/D: requiere muestras suficientes de un tramo físico.");
        }
      }
      const auto requested = valueAt(physical, "motion.requested_duration_s");
      const auto observed = valueAt(physical, "motion.submit_to_terminal_reply_s");
      if (requested.isDouble() && requested.toDouble() > 0 && observed.isDouble()) {
        description += tr(" · envío→acuse / duración solicitada: %1×")
          .arg(observed.toDouble() / requested.toDouble(), 0, 'f', 2);
      }
      if (physical.value("ack_minus_reference_s").isDouble()) {
        description += tr(" · envío→acuse menos referencia %1 s (incluye transporte y espera)")
          .arg(physical.value("ack_minus_reference_s").toDouble(), 0, 'f', 3);
      }
      if (!origin.isEmpty()) {
        description += origin == "rapid_phase_2_observed" ?
          tr(" · Origen de referencia: fase 2 observada en Fedora.") :
          tr(" · Origen de referencia: envío desde Fedora.");
      }
    }
    if (whole_run) {
      if (physical.isEmpty()) {
        description += tr(" · Ensayo %1 · historial en memoria: %2/%3 muestras")
          .arg(jsonText(latest.value("preview_run_id"))).arg(history.size()).arg(history_limit);
      }
      if (dropped) {
        description += tr(" · ADVERTENCIA: datos anteriores descartados (%1)").arg(dropped);
      }
    }
    if (paused) {
      description += tr(" · Gráfica pausada");
    }
    plot_info->setText(description);
  }

  void refresh()
  {
    dirty = false;
    if (tabs->currentIndex() == graphs_page) {
      refreshGraph();
      return;
    }
    const int active = tabs->currentIndex();
    if (active <= 0 || active > static_cast<int>(details.size())) {
      return;
    }

    auto * view = details[active - 1];
    const QString source = jsonText(latest.value("source"));
    const QString base = jsonText(latest.value("base_frame"));
    const QString unit = radians ? QStringLiteral("rad") : QStringLiteral("°");
    const double factor = radians ? 1.0 : degrees_per_rad;
    const QString header = QString("%1   ·   %2   ·   base %3\n\n")
      .arg(source, utcTime(latest), base);
    const auto analysis_point = latest.value("point").toString();
    const QStringList points = analysis_point == "both" ?
      QStringList{"flange", "tcp"} : QStringList{analysis_point == "flange" ? "flange" : "tcp"};
    QStringList contents;
    if (active == pose_page) {
      contents << tr("Postura actual [%1] · %2\nObjetivo solicitado [%1] · %3\n"
          "Fantasma [%1] · %4\nFinal de la trayectoria [%1] · %5")
        .arg(unit, vectorText(latest.value("actual"), factor),
          vectorText(latest.value("target"), factor), vectorText(latest.value("preview"), factor),
          vectorText(valueAt(latest, "plan.final_q"), factor));
      contents << tr("Diferencia objetivo solicitado − actual del modelo [%1] · %2")
        .arg(unit, vectorText(valueAt(latest, "error.joint_rad"), factor));
      contents << tr("Diferencia final de trayectoria − fantasma [%1] · %2")
        .arg(unit, vectorText(valueAt(latest, "plan.remaining_joint_rad"), factor));
      for (const auto & key : points) {
        contents << poseBlock(pointName(key) + tr(" actual"),
          valueAt(latest, "actual_fk." + key), factor, unit)
          << poseBlock(pointName(key) + tr(" objetivo"), valueAt(latest, "target_fk." + key), factor, unit)
          << poseBlock(pointName(key) + tr(" fantasma"), valueAt(latest, "preview_fk." + key), factor, unit);
      }
      contents << poseBlock(tr("Offset brida → TCP · ") + jsonText(latest.value("tool_source")),
        latest.value("tool_offset"), factor, unit)
        << tr("Referencia independiente: N/D · INCOMPARABLE")
        << routeDetails(routes, factor, unit);
    } else if (active == jacobian_page || active == singularity_page) {
      const auto q = latest.value("jacobian_q");
      contents << QString("q [%1]  %2   ·   %3").arg(unit, vectorText(q, factor),
          jsonText(latest.value("jacobian_source")));
      for (const auto & key : points) {
        const auto result = valueAt(latest, "jacobian." + key).toObject();
        if (active == jacobian_page) {
          QStringList rows;
          for (const auto & item : result.value("fd").toArray()) {
            rows << QString("%1   %2   %3").arg(jsonText(valueAt(item, "h")).rightJustified(11),
              jsonText(valueAt(item, "error_jv")).rightJustified(13),
              jsonText(valueAt(item, "error_jw")).rightJustified(13));
          }
          contents << QString("%1\nJv [m/rad]\n%2\n\nJω [rad/rad]\n%3\n\n"
              "    h [rad]      Error Jv      Error Jω\n%4")
            .arg(pointName(key), matrixText(result.value("jv")), matrixText(result.value("jw")),
              rows.isEmpty() ? "N/D" : rows.join('\n'));
        } else {
          contents << QString("%1\nσ = %2\nσmín = %3   κ = %4   rango = %5"
              "\nL = %6 m   umbral de rango = %7\nDirección crítica (twist escalado): %8\nGanancia inversa: %9")
            .arg(pointName(key), vectorText(result.value("sigma")), jsonText(result.value("sigma_min")),
              jsonText(result.value("kappa")), jsonText(result.value("rank")),
              jsonText(result.value("L")), jsonText(result.value("rank_tol")),
              vectorText(result.value("weakest_direction_scaled")), jsonText(result.value("inverse_gain")));
        }
      }
    } else if (active == profile_page) {
      const auto plan = latest.value("plan").toObject();
      if (plan.isEmpty()) {
        const auto end_marker = end_markers.value(latest.value("preview_run_id").toString());
        if (end_marker.isEmpty()) {
          contents << tr("N/D · Inicie la previsualización de una ruta.");
        } else {
          contents << tr("Ensayo %1 · %2 · %3")
            .arg(jsonText(end_marker.value("run_id")), jsonText(end_marker.value("phase")),
              jsonText(end_marker.value("reason")));
        }
      } else {
        contents << QString("%1 · %2 / %3 s · %4\nq [%5]       %6\nqdot [%5/s]   %7\nqddot [%5/s²] %8")
          .arg(jsonText(plan.value("branch")), jsonText(plan.value("time_s")),
            jsonText(plan.value("duration_s")), plan.value("paused").toBool() ? tr("En pausa") :
            (plan.value("running").toBool() ? tr("En curso") : tr("Detenida")),
            unit, vectorText(plan.value("q"), factor), vectorText(plan.value("qdot"), factor),
            vectorText(plan.value("qddot"), factor));
        contents << tr("Ensayo %1 · %2\nFinal de la trayectoria [%3] · %4")
          .arg(jsonText(plan.value("run_id")), jsonText(plan.value("phase")), unit,
            vectorText(plan.value("final_q"), factor));
        contents << QString("DINÁMICA DOCENTE 1R\nτ = %1 N·m\nParámetros: %2\nMezcla [s]: %3")
          .arg(jsonText(plan.value("torque_1r_nm")), jsonText(plan.value("dynamics_1r")),
            jsonText(plan.value("blend_s")));
      }
    } else if (active == events_page) {
      for (auto it = events.rbegin(); it != events.rend(); ++it) {
        const auto state = it->value("estado_incidente").toString();
        QString lifecycle;
        if (!state.isEmpty()) {
          lifecycle = tr("\nEstado al registrar: %1").arg(state);
        }
        if (state == QStringLiteral("resuelto")) {
          const QJsonObject resolved{{"timestamp_ns", it->value("resolved_timestamp_ns")}};
          lifecycle += tr("\nResolución %1 · %2").arg(utcTime(resolved),
            jsonText(it->value("resolution")));
        }
        contents << QString("%1  %2  %3  %4\n%5\nAcción: %6%7")
          .arg(utcTime(*it), jsonText(it->value("severity")),
            it->value("code").toString(tr("sin código")), jsonText(it->value("source")),
            jsonText(it->value("detail")), jsonText(it->value("action")),
            lifecycle);
      }
      if (contents.isEmpty()) {
        contents << tr("Sin eventos registrados.");
      }
    } else if (active == diagnostics_page) {
      const auto diagnostic = latest.value("diagnostic").toObject();
      contents << tr("Modo %1 · backend %2 · %3").arg(jsonText(latest.value("mode")),
        jsonText(latest.value("backend")),
        latest.value("stale").toBool() ? QStringLiteral("STALE") : jsonText(diagnostic.value("severity")));
      contents << tr("Conexión del backend · %1\nHabilitación física · %2\nDisponible para habilitar · %3")
        .arg(latest.value("backend_connected").toBool() ? tr("conectado") : tr("sin conexión"),
          latest.value("physical_armed").toBool() ? tr("activa") : tr("inactiva"),
          latest.value("physical_can_arm").toBool() ? tr("sí") : tr("no"));
      contents << QString("Feedback: edad %1 s · frecuencia %2 Hz\nOrden: %3\nReloj: %4\n"
          "Brida: %5   TCP: %6   Offset: %7")
        .arg(jsonText(latest.value("feedback_age_s")), jsonText(latest.value("feedback_hz")),
          jsonText(latest.value("joint_order")), jsonText(latest.value("clock")),
          jsonText(latest.value("flange_frame")), jsonText(latest.value("tcp_frame")),
          jsonText(latest.value("tool_source")));
      contents << tr("Estado: %1\nCódigo: %2\nCausa: %3\nEfecto: %4\nAcción: %5")
        .arg(jsonText(diagnostic.value("detail")), diagnostic.value("code").toString(tr("sin código")),
          jsonText(diagnostic.value("cause")), jsonText(diagnostic.value("effect")),
          jsonText(diagnostic.value("action")));
      const auto transport = latest.value("transport").toObject();
      QStringList transport_fields;
      for (auto it = transport.begin(); it != transport.end(); ++it) {
        transport_fields << QString("%1 = %2").arg(it.key(), jsonText(it.value()));
      }
      contents << QString("Transporte:\n%1\nComparabilidad: %2")
        .arg(transport_fields.isEmpty() ? "N/D" : transport_fields.join("\n"),
          jsonText(valueAt(latest, "error.comparison")));
      const auto incident = latest.value("incident").toObject();
      if (!incident.isEmpty()) {
        contents << tr("Incidente vigente: %1 · %2 · %3\n%4")
          .arg(utcTime(incident), jsonText(incident.value("code")), jsonText(incident.value("detail")),
            jsonText(incident.value("action")));
      }
    } else {
      contents << tr("Muestras en memoria: %1 / %2 · descartadas por capacidad: %3\nEventos: %4")
        .arg(history.size()).arg(history_limit).arg(dropped).arg(events.size());
      if (!cleared_run_id.isEmpty() &&
        latest.value("plan").toObject().value("run_id").toString() == cleared_run_id)
      {
        contents << tr("Ensayo %1 excluido por la limpieza. Inicie otro para capturarlo desde el comienzo.")
          .arg(cleared_run_id);
      }
      contents << tr("S14 P1–P7 · tolerancias y revisión docente pendientes.\n"
          "G0 · Contexto documental y modelo. G1 · Telemetría real pendiente de este ensayo.\n"
          "G2 · Comprobaciones internas. G3 · Requiere laboratorio. G4 · Repetibilidad física pendiente.");
    }
    setPageText(view, header + contents.join("\n\n"));
  }

  AnalysisWidget * owner;
  QTabWidget * tabs{};
  QComboBox * group{};
  QComboBox * point{};
  QComboBox * dynamics_axis{};
  QComboBox * view_mode{};
  QComboBox * export_run{};
  QDoubleSpinBox * window{};
  Plot * plot{};
  QLabel * legend{};
  QLabel * plot_info{};
  QLabel * export_status{};
  QLineEdit * operator_id{};
  QLineEdit * team{};
  std::array<QCheckBox *, 6> joints{};
  std::vector<QPlainTextEdit *> details;
  std::vector<QVBoxLayout *> detail_layouts;
  std::deque<Sample> history;
  std::deque<QJsonObject> events;
  std::deque<QJsonObject> route_history;
  QJsonObject latest;
  QJsonObject latest_incident;
  QJsonObject latest_resolution;
  QJsonArray incident_history;
  QMap<QString, QJsonObject> end_markers;
  QJsonObject routes;
  QString cleared_run_id, cleared_physical_id, last_closed_physical_id;
  QString event_signature;
  QString exported_path;
  QMap<QString, QString> observed_trials;
  AnalysisWidget::CaptureSummary current_trial;
  double offset{};
  double last_time{};
  double frozen_time{};
  double time_origin{};
  int segment{};
  std::size_t dropped{};
  bool paused{};
  bool dirty{};
  bool radians{};
};

AnalysisWidget::AnalysisWidget(QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(this))
{}

AnalysisWidget::~AnalysisWidget() = default;

void AnalysisWidget::ingest(const QJsonObject & sample)
{
  impl_->addSample(sample);
}

void AnalysisWidget::ingestRoutes(const QJsonObject & routes)
{
  impl_->addRoutes(routes, impl_->latest.value("timestamp_ns"));
}

void AnalysisWidget::setRadians(bool enabled)
{
  if (impl_->radians != enabled) {
    impl_->radians = enabled;
    impl_->refresh();
  }
}

void AnalysisWidget::selectDiagnostics()
{
  impl_->tabs->setCurrentIndex(diagnostics_page);
}

void AnalysisWidget::selectEvidence()
{
  impl_->tabs->setCurrentIndex(evidence_page);
}

AnalysisWidget::CaptureSummary AnalysisWidget::captureSummary() const
{
  auto summary = impl_->current_trial;
  summary.samples = static_cast<int>(impl_->history.size());
  summary.events = static_cast<int>(impl_->events.size());
  summary.exported_directory = impl_->exported_path;
  if (!summary.trial_id.isEmpty() &&
    std::none_of(impl_->history.crbegin(), impl_->history.crend(), [&](const Sample & sample) {
      return belongsToTrial(sample.data, summary.trial_id);
    }))
  {
    summary.trial_id.clear();
    summary.trial_type.clear();
    summary.trial_result.clear();
    summary.timestamp_ns.clear();
  }
  return summary;
}

namespace
{
QString s14Report(const QString & id, const QJsonObject & last, const QJsonObject & routes,
  const QString & team, const QString & operator_id)
{
  QString result;
  const auto * const unavailable = "N/D";
  const QString stamp = utcTime(last);
  const QString sha = jsonText(last.value("model_sha256"));
  const QString order = jsonText(last.value("joint_order"));
  const QString base = jsonText(last.value("base_frame"));
  const QString source = jsonText(last.value("source"));
  const QString q = jsonText(last.value("actual"));
  const QString decision = QStringLiteral("Parcial · solo modelo interno");
  const auto row = [](const QStringList & fields) {
      QStringList escaped;
      for (const auto & field : fields) {
        escaped.append(tableField(field));
      }
      return "| " + escaped.join(" | ") + " |\n";
    };
  result += QString("# S14 · ensayo %1\n\nEquipo: %2 · Operador: %3 · Fecha UTC: %4\n\n"
      "Modelo SHA-256: %5 · Fuente: %6 · Orden: %7 · Unidad canónica: rad\n\n"
      "Referencia independiente: N/D. Tolerancias y revisión docente: pendientes.\n\n")
    .arg(id, team, operator_id, stamp, sha, source, order);

  result += QStringLiteral("## P1 · Cinemática directa\n\n"
    "| q [rad] | Fecha UTC | Marco y punto | p propia [m] | R propia | Referencia | e_p/e_R | Decisión | Evidencia |\n"
    "| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");
  for (const QString & key : {QString("flange"), QString("tcp")}) {
    const auto pose = valueAt(last, "actual_fk." + key).toObject();
    const auto matrix = pose.value("matrix").toArray();
    QJsonArray rotation;
    for (int i = 0; i < 3 && i < matrix.size(); ++i) {
      const auto source_row = matrix[i].toArray();
      QJsonArray destination_row;
      for (int j = 0; j < 3 && j < source_row.size(); ++j) {
        destination_row.append(source_row[j]);
      }
      rotation.append(destination_row);
    }
    const QString frame = base + " → " + jsonText(last.value(key + "_frame"));
    result += row({q, stamp, frame, jsonText(pose.value("xyz")),
      rotation.isEmpty() ? QString(unavailable) : jsonText(rotation), unavailable, unavailable,
      decision, "manifest.json y muestras.json"});
  }

  result += QStringLiteral("\n## P2 · Ramas de cinemática inversa\n\n"
    "| Pose / punto | Método | Referencia | Rama | q [rad] | e_p [m] | e_R [rad] | Límites / estado | Evidencia |\n"
    "| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");
  for (const auto & value : routes.value("routes").toArray()) {
    const auto branch = value.toObject();
    const auto cause = branch.value("reason").toString();
    result += row({jsonText(routes.value("request_id")) + " / " + jsonText(routes.value("point")),
      jsonText(branch.value("source")), unavailable, jsonText(branch.value("branch")),
      jsonText(branch.value("q")), jsonText(branch.value("position_error_m")),
      jsonText(branch.value("orientation_error_rad")),
      cause.isEmpty() ? QStringLiteral("Candidata según URDF") : cause, "rutas.json"});
  }
  if (routes.value("routes").toArray().isEmpty()) {
    result += row({unavailable, unavailable, unavailable, unavailable, unavailable, unavailable,
      unavailable, QStringLiteral("No ejecutado"), "rutas.json"});
  }

  result += QStringLiteral("\n## P3 · Jacobiano y diferencias finitas\n\n"
    "| q [rad] | Base y punto | Jv/Jω propia | Referencia | h [rad] y errores DF | Twist/modelo | Decisión |\n"
    "| --- | --- | --- | --- | --- | --- | --- |\n");
  const auto jacobian_q = last.value("jacobian_q");
  for (const QString & key : {QString("flange"), QString("tcp")}) {
    const auto jacobian = valueAt(last, "jacobian." + key).toObject();
    result += row({jsonText(jacobian_q), base + " → " + jsonText(last.value(key + "_frame")),
      "muestras.json → jacobian." + key, unavailable, jsonText(jacobian.value("fd")),
      jsonText(valueAt(last, "plan.twist." + key)), decision});
  }

  result += QStringLiteral("\n## P4 · Singularidades\n\n"
    "| q [rad] | Punto | L [m] | σmin / σmax | Umbral | Rango | Dirección crítica | κ | Evidencia |\n"
    "| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");
  for (const QString & key : {QString("flange"), QString("tcp")}) {
    const auto jacobian = valueAt(last, "jacobian." + key).toObject();
    result += row({jsonText(jacobian_q), pointName(key), jsonText(jacobian.value("L")),
      jsonText(jacobian.value("sigma_min")) + " / " + jsonText(valueAt(jacobian, "sigma.0")),
      jsonText(jacobian.value("rank_tol")), jsonText(jacobian.value("rank")),
      jsonText(jacobian.value("weakest_direction_scaled")), jsonText(jacobian.value("kappa")), "muestras.json"});
  }

  result += QStringLiteral("\n## P5 · Decisión formal\n\n"
    "| Alcance | Estado | Observación |\n| --- | --- | --- |\n"
    "| G0 · modelo y configuración | Parcial | Manifiesto y procedencia de esta sesión. |\n"
    "| G1 · controlador real | No ejecutado | Sin datos reales verificados en el ensayo. |\n"
    "| G2 · consistencia interna | Parcial | Comparación independiente y tolerancias pendientes. |\n"
    "| G3 · ejecución física | Requiere laboratorio | Sin evidencia física en este ensayo. |\n"
    "| G4 · tablero | Requiere laboratorio | Calibración y repetibilidad pendientes. |\n"
    "\n## P6 · Checklist\n\n"
    "- [x] Identificación de origen y versión disponible por muestra.\n"
    "- [ ] Referencia independiente y compatibilidad de punto, marcos y reloj confirmadas.\n"
    "- [ ] Tolerancias e incertidumbre revisadas.\n"
    "- [ ] Revisión de evidencia física, si aplica.\n"
    "\n## P7 · Revisión\n\n"
    "Responsable: ____________________  ·  Fecha: ____________________\n\n"
    "Revisión docente: pendiente  ·  Alcance aprobado: ____________________\n");
  return result;
}
}  // namespace

bool AnalysisWidget::exportEvidence(
  const QString & directory, QString * error, const QString & run_id)
{
  std::deque<Sample> selection;
  if (!run_id.isEmpty()) {
    for (const auto & sample : impl_->history) {
      if (belongsToTrial(sample.data, run_id)) {
        selection.push_back(sample);
      }
    }
  }
  const auto & history = run_id.isEmpty() ? impl_->history : selection;
  if (history.empty()) {
    if (error) {
      *error = run_id.isEmpty() ?
        tr("N/D · Se necesita al menos una muestra en la captura. "
          "Inicie un ensayo nuevo si el anterior sigue visible.") :
        tr("El ensayo seleccionado ya no tiene muestras en memoria.");
    }
    return false;
  }
  const auto now = QDateTime::currentDateTimeUtc();
  const QString id = QString("IRB120-%1-%2").arg(now.toString("yyyyMMdd-HHmmss"),
    QUuid::createUuid().toString(QUuid::WithoutBraces).left(8));
  const QDir parent(directory);
  if (!parent.exists() || !parent.mkdir(id)) {
    if (error) {
      *error = tr("No se pudo crear el directorio del ensayo en %1.").arg(directory);
    }
    return false;
  }
  const QDir dir(parent.filePath(id));
  const auto & last = history.back().data;
  auto route_snapshot = run_id.isEmpty() && !impl_->route_history.empty() ?
    impl_->routes : QJsonObject{};
  QMap<QString, QJsonValue> columns;
  for (const auto & sample : history) {
    flatten(columns, "", sample.data);
  }

  QSaveFile csv_file(dir.filePath("muestras.csv"));
  QSaveFile json_file(dir.filePath("muestras.json"));
  if (!csv_file.open(QIODevice::WriteOnly) || !json_file.open(QIODevice::WriteOnly)) {
    if (error) {
      *error = tr("No se pudieron abrir los archivos de muestras en %1.").arg(dir.path());
    }
    return false;
  }
  QByteArray header;
  for (const auto & field : columns.keys()) {
    if (!header.isEmpty()) {
      header += ',';
    }
    header += quoted(field.toUtf8());
  }
  header += '\n';
  bool ok = csv_file.write(header) == header.size() && json_file.write("[\n") == 2;
  auto fields = columns.keys();
  for (auto it = history.begin(); it != history.end() && ok; ++it) {
    QMap<QString, QJsonValue> cells;
    flatten(cells, "", it->data);
    QByteArray row;
    for (int i = 0; i < fields.size(); ++i) {
      if (i != 0) {
        row += ',';
      }
      row += csvCell(cells.value(fields[i]));
    }
    row += '\n';
    QByteArray json = QJsonDocument(it->data).toJson(QJsonDocument::Compact);
    json += std::next(it) == history.end() ? "\n" : ",\n";
    ok = csv_file.write(row) == row.size() && json_file.write(json) == json.size();
  }
  if (!ok || json_file.write("]\n") != 2 || !csv_file.commit() || !json_file.commit()) {
    if (error) {
      *error = tr("No se completó la escritura de las muestras en %1.").arg(dir.path());
    }
    return false;
  }

  QJsonArray events;
  QJsonArray route_history;
  for (const auto & route : impl_->route_history) {
    if (run_id.isEmpty() || route.value("request_id").toString() == run_id) {
      route_history.append(route);
      if (!run_id.isEmpty()) {
        route_snapshot = route;
      }
    }
  }
  for (const auto & event : impl_->events) {
    if (run_id.isEmpty() || event.value("ensayo_al_observar").toString() == run_id ||
      event.value("ensayo_operacion").toString() == run_id)
    {
      events.append(event);
    }
  }
  const QString team = impl_->team->text().trimmed().isEmpty() ?
    QStringLiteral("N/D") : impl_->team->text().trimmed();
  const QString operator_id = impl_->operator_id->text().trimmed().isEmpty() ?
    QStringLiteral("N/D") : impl_->operator_id->text().trimmed();
  impl_->refreshGraph(run_id.isEmpty() ? nullptr : &selection);
  QImage picture = impl_->plot->snapshot();
  if (!run_id.isEmpty()) {
    impl_->refreshGraph();
  }
  QByteArray png;
  QBuffer buffer(&png);
  buffer.open(QIODevice::WriteOnly);
  if (!picture.save(&buffer, "PNG")) {
    if (error) {
      *error = tr("No se pudo generar la figura PNG.");
    }
    return false;
  }
  // One summary per retained preview run; never claim physical execution.
  QMap<QString, QJsonObject> runs;
  for (const auto & record : history) {
    const auto plan = record.data.value("plan").toObject();
    const auto run_id = plan.value("run_id").toString();
    if (run_id.isEmpty()) {
      continue;
    }
    auto run = runs.value(run_id);
    if (run.isEmpty()) {
      run.insert("run_id", run_id);
      run.insert("first_ros_ns_text", record.data.value("timestamp_ns_text"));
      run.insert("first_plot_time_s", record.time);
      run.insert("samples", 0);
      run.insert("mode", record.data.value("mode"));
      run.insert("source", record.data.value("source"));
    }
    run.insert("last_ros_ns_text", record.data.value("timestamp_ns_text"));
    run.insert("last_plot_time_s", record.time);
    run.insert("samples", run.value("samples").toInt() + 1);
    run.insert("duration_s", plan.value("duration_s"));
    run.insert("profile_id", plan.value("profile_id"));
    run.insert("profile_source", plan.value("profile_source"));
    run.insert("last_plan_time_s", plan.value("time_s"));
    run.insert("phase", plan.value("phase"));
    run.insert("cycles_total", plan.value("cycles_total"));
    run.insert("cycle_index", plan.value("cycle_index"));
    run.insert("waypoints_per_cycle", plan.value("waypoints_per_cycle"));
    run.insert("waypoint_times_s", plan.value("waypoint_times_s"));
    run.insert("final_q", plan.value("final_q"));
    runs.insert(run_id, run);
  }
  QJsonArray runs_array;
  for (auto it = runs.cbegin(); it != runs.cend(); ++it) {
    auto run = it.value();
    const auto end_marker = impl_->end_markers.value(it.key());
    if (!end_marker.isEmpty()) {
      run.insert("phase", end_marker.value("phase"));
      run.insert("fin", end_marker);
    }
    const auto phase = run.value("phase").toString();
    run.insert("completed", phase == QStringLiteral("completed"));
    if (phase == QStringLiteral("running") || phase == QStringLiteral("paused")) {
      run.insert("estado_captura", tr("Sin estado terminal registrado"));
    }
    runs_array.append(run);
  }
  const QJsonObject manifest{
    {"schema", "imt342.evidence.v1"},
    {"ensayo", id},
    {"equipo", team},
    {"operador", operator_id},
    {"exportado_utc", now.toString(Qt::ISODateWithMs)},
    {"primera_muestra_ros_ns", history.front().data.value("timestamp_ns")},
    {"ultima_muestra_ros_ns", history.back().data.value("timestamp_ns")},
    {"primera_muestra_ros_ns_text", history.front().data.value("timestamp_ns_text")},
    {"ultima_muestra_ros_ns_text", history.back().data.value("timestamp_ns_text")},
    {"muestras", static_cast<int>(history.size())},
    {"muestras_descartadas_por_capacidad", static_cast<int>(impl_->dropped)},
    {"eventos", events.size()},
    {"cálculos_de_ruta", route_history.size()},
    {"ensayos_previsualizados_en_buffer", runs_array.size()},
    {"ensayos_previsualizados", runs_array},
    {"ensayo_visualizado", last.value("preview_run_id")},
    {"alcance", run_id.isEmpty() ?
      tr("Captura desde el arranque o la última limpieza") : tr("Ensayo seleccionado")},
    {"ensayo_filtrado", run_id.isEmpty() ? QJsonValue::Null : QJsonValue(run_id)},
    {"alcance_eventos", run_id.isEmpty() ? tr("Toda la captura") :
      tr("Eventos observados durante el ensayo con asociación registrada")},
    {"alcance_rutas", run_id.isEmpty() ? tr("Toda la captura") :
      tr("Cálculos IK que coinciden con el identificador seleccionado")},
    {"fuentes_de_datos", QJsonObject{
        {"actual", "Ver source y stale en cada muestra; simulación, mock o feedback atribuido al backend"},
        {"target", "Objetivo solicitado"},
        {"preview", "Fantasma calculado"},
        {"plan", "Previsualización calculada; ver profile_id y profile_source"},
        {"individual_trial", "Orden física, referencia calculada, duración solicitada "
          "y observaciones del protocolo cuando estén disponibles"},
        {"transmitted_goal", "Objetivo confirmado por el protocolo y diferencia final medida, si está disponible"},
        {"fk_jacobian", "Cálculo con el modelo"},
        {"error", "Comparación interna según el campo comparison; referencia independiente pendiente"}}},
    {"continuidad", "Cambios de plot_segment delimitan reposicionamientos; no interpolar entre ellos"},
    {"referencia_independiente", QJsonValue::Null},
    {"tolerancias_docentes", "pendientes"},
    {"revision_docente", "pendiente"},
    {"controlador_verificado", QJsonValue::Null},
    {"figura_grupo", impl_->group->currentText()},
    {"figura_punto", impl_->point->currentData().toString()},
    {"figura_unidad_angular", impl_->radians ? "rad" : "deg"},
    {"archivos", QJsonArray{"muestras.csv", "muestras.json", "rutas.json",
        "eventos.json", "ensayos.json", "grafica.png", "S14_P1-P7.md"}},
    {"estado_final", last}};
  if (!writeArtifact(dir, "rutas.json", QJsonDocument(QJsonObject{
      {"vigente", route_snapshot}, {"historial", route_history}}).toJson(), error) ||
    !writeArtifact(dir, "eventos.json", QJsonDocument(events).toJson(), error) ||
    !writeArtifact(dir, "ensayos.json", QJsonDocument(runs_array).toJson(), error) ||
    !writeArtifact(dir, "grafica.png", png, error) ||
    !writeArtifact(dir, "S14_P1-P7.md",
    s14Report(id, last, route_snapshot, team, operator_id).toUtf8(), error) ||
    !writeArtifact(dir, "manifest.json", QJsonDocument(manifest).toJson(), error))
  {
    return false;
  }
  impl_->exported_path = dir.path();
  return true;
}
}  // namespace imt342_irb120_rviz
