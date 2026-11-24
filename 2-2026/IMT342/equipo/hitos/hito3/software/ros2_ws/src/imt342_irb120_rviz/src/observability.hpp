#ifndef IMT342_IRB120_RVIZ__OBSERVABILITY_HPP_
#define IMT342_IRB120_RVIZ__OBSERVABILITY_HPP_

#include <functional>
#include <initializer_list>
#include <utility>

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QJsonArray>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>

#include "analysis_widget.hpp"

namespace imt342_irb120_rviz
{
class Observability
{
public:
  Observability(
    QVBoxLayout * diagnostics, QVBoxLayout * evidence, std::function<void()> prepare,
    std::function<void()> show_evidence, std::function<AnalysisWidget *()> find_analysis,
    std::function<bool()> export_task)
  : find_analysis_(std::move(find_analysis))
  {
    auto * system = section(diagnostics, tr("Sistema ROS 2"));
    coordinator_ = row(system, tr("Coordinador"));
    operation_ = row(system, tr("Operación"));
    backend_ = row(system, tr("Backend"));
    auto * telemetry = section(diagnostics, tr("Robot y telemetría"));
    source_ = row(telemetry, tr("Fuente de J1–J6"));
    freshness_ = row(telemetry, tr("Vigencia"));
    feedback_age_ = row(telemetry, tr("Edad y frecuencia"));
    auto * controller = section(diagnostics, tr("Controlador IRC5"));
    connection_ = row(controller, tr("Conectividad RWS"));
    authentication_ = row(controller, tr("Autenticación"));
    controller_state_ = row(controller, tr("Modo y motores"));
    rapid_state_ = row(controller, tr("RAPID"));
    controller_task_ = row(controller, tr("Tarea y módulo"));
    program_pointer_ = row(controller, tr("Puntero"));
    auto * execution = section(diagnostics, tr("Estado de ejecución"));
    authorization_ = row(execution, tr("Autorización individual"));
    execution_state_ = row(execution, tr("Estado de la orden"));
    last_order_ = row(execution, tr("Última secuencia"));
    auto * rapid = section(diagnostics, tr("Servidor RAPID"));
    preparation_ = row(rapid, tr("Preparación"));
    prepare_ = new QPushButton(tr("Preparar servidor RAPID…"));
    prepare_->setEnabled(false);
    rapid->addRow(prepare_);
    QObject::connect(prepare_, &QPushButton::clicked, prepare_, [this, action = std::move(prepare)]() {
      if (QMessageBox::question(prepare_, tr("Preparar servidor RAPID"),
          tr("Confirme la supervisión de la celda. Esta operación puede posicionar el puntero "
          "en ServeJointOrders e iniciar RAPID. La orden de movimiento se autoriza por separado."),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes &&
        prepare_->isEnabled())
      {
        prepare_->setEnabled(false);
        prepare_request_.start();
        action();
      }
    });

    auto * advice = section(diagnostics, tr("Incidencias"));
    problem_ = row(advice, tr("Estado o causa"));
    recommended_ = row(advice, tr("Acción recomendada"));
    auto * disclose = new QToolButton;
    disclose->setText(tr("Detalle técnico"));
    disclose->setCheckable(true);
    disclose->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    disclose->setArrowType(Qt::RightArrow);
    diagnostics->addWidget(disclose);
    technical_ = new QPlainTextEdit;
    technical_->setReadOnly(true);
    technical_->setAccessibleName(tr("Detalle técnico del diagnóstico"));
    technical_->setMaximumHeight(180);
    technical_->hide();
    diagnostics->addWidget(technical_);
    QObject::connect(disclose, &QToolButton::toggled, technical_, [this, disclose](bool open) {
      disclose->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
      technical_->setVisible(open);
    });

    auto * capture = section(evidence, tr("Captura de la sesión"));
    session_ = row(capture, tr("Procedencia"));
    trial_id_ = row(capture, tr("ID de ensayo"));
    trial_time_ = row(capture, tr("Fecha y hora del ensayo"));
    trial_kind_ = row(capture, tr("Tipo de prueba"));
    trial_result_ = row(capture, tr("Resultado"));
    counts_ = row(capture, tr("Muestras y eventos"));
    evidence_order_ = row(capture, tr("Orden RAPID"));
    exported_ = row(capture, tr("Exportación"));

    auto * analysis = button(evidence, tr("Abrir análisis de evidencia"));
    QObject::connect(analysis, &QPushButton::clicked, analysis, std::move(show_evidence));
    export_session_ = button(evidence, tr("Exportar sesión…"));
    export_trial_ = button(evidence, tr("Exportar último ensayo…"));
    export_task_ = button(evidence, tr("Exportar resultado de tarea…"));
    QObject::connect(export_session_, &QPushButton::clicked, export_session_, [this]() {
      exportCapture(false);
    });
    QObject::connect(export_trial_, &QPushButton::clicked, export_trial_, [this]() {
      exportCapture(true);
    });
    QObject::connect(export_task_, &QPushButton::clicked, export_task_,
      [this, action = std::move(export_task)]() {
        if (action()) {
          export_message_ = tr("Resultado de tarea guardado");
          set(exported_, export_message_);
        }
      });
    folder_ = button(evidence, tr("Ver carpeta de evidencias"));
    QObject::connect(folder_, &QPushButton::clicked, folder_, [this]() {
      const auto * analysis = find_analysis_();
      const auto exported = analysis ? analysis->captureSummary().exported_directory : QString();
      const auto path = QDir::cleanPath(QFileInfo(exported).isDir() ? exported : evidenceDirectory());
      if (!QDesktopServices::openUrl(QUrl::fromLocalFile(path))) {
        export_message_ = tr("Carpeta disponible en %1").arg(path);
        set(exported_, export_message_);
      }
    });
    export_session_->setEnabled(false);
    export_trial_->setEnabled(false);
    export_task_->setEnabled(false);
    folder_->setEnabled(QFileInfo(evidenceDirectory()).isDir());

  }

  void update(const QJsonObject & packet, bool prepare_command_available)
  {
    const auto stamp = packet.value("timestamp_ns_text").toString();
    if (!stamp.isEmpty() && stamp != last_stamp_) {
      last_stamp_ = stamp;
      coordinator_seen_.restart();
    }
    const bool live = coordinator_seen_.isValid() && coordinator_seen_.elapsed() <= 2000;
    const auto mode = packet.value("mode").toString();
    const auto backend = packet.value("backend").toString();
    const bool offline = mode == "offline";
    const bool rws = backend == "rws" && !offline;
    const auto source = packet.value("source").toString();
    const bool stale = packet.value("stale").toBool() || (!live && !packet.isEmpty());
    const bool feedback = !packet.value("actual").toArray().isEmpty();
    const bool connected = live && packet.value("backend_connected").toBool();
    const auto transport = packet.value("transport").toObject();
    const auto config = packet.value("controller_config").toObject();
    const auto status = packet.value("diagnostic").toObject();
    const auto incident = packet.value("incident").toObject();
    const auto preparation = packet.value("rapid_preparation").toObject();
    const auto pointer = transport.value("pointer").toArray();
    const auto task = packet.value("task").toObject();
    const auto reply = transport.value("reply").toArray();
    const auto http = transport.value("http_status");
    const auto sequence = transport.value("order_sequence");
    const auto age = metric(packet.value("feedback_age_s"), tr("s"));
    const auto frequency = metric(packet.value("feedback_hz"), tr("Hz"));
    const auto preparation_state = preparation.value("state").toString();
    const auto rapid = transport.value("execution").toString();
    const auto result = transport.value("result").toString();
    set(coordinator_, live ? tr("Conectado") : tr("Sin comunicación reciente"));
    set(operation_, mode == "offline" ? tr("Offline") : mode == "observe" ? tr("Online · Solo lectura") :
      mode == "control" ? tr("Online · Control físico") : QString());
    set(backend_, backend == "rws" ? tr("RWS-RAPID") : backend == "rws_trajectory" ? tr("RWS/RAPID — Trayectorias v2") :
      backend == "external" ? tr("Telemetría ROS externa") : QString());
    set(source_, offline ? tr("Postura simulada") : source.compare("N/D") == 0 ? QString() : source);
    set(freshness_, stale ? tr("Telemetría vencida") :
      feedback ? (offline ? tr("Simulación activa") : tr("Telemetría reciente")) : QString());
    set(feedback_age_, age.isEmpty() && frequency.isEmpty() ? QString() :
      tr("%1 · %2").arg(available(age), available(frequency)));
    set(connection_, offline ? tr("Offline") : rws ?
      (connected ? tr("Conectado") : tr("Sin conexión")) : QString());
    set(authentication_, !rws ? QString() : http.toInt() == 401 ? tr("Credenciales rechazadas") :
      http.toInt() == 403 ? tr("Acceso denegado") : connected ? tr("Lecturas RWS autenticadas") : QString());
    set(controller_state_, joined({transport.value("mode").toString(), motorState(transport.value("motors").toString())}));
    set(rapid_state_, rapid == "running" ? tr("RAPID en ejecución") :
      rapid == "stopped" ? tr("RAPID detenido") : rapid);
    set(controller_task_, joined({config.value("task").toString(),
      config.value("module").toString(pointer.size() == 2 ? pointer[0].toString() : QString())}));
    set(program_pointer_, pointer.size() == 2 ?
      joined({pointer[0].toString(), pointer[1].toString()}, QStringLiteral(" / ")) : QString());
    set(authorization_, mode != "control" ? tr("Disponible en Control físico") :
      packet.value("physical_armed").toBool() ? tr("Una orden habilitada") : tr("Orden sin habilitar"));
    set(execution_state_, resultLabel(result));
    set(last_order_, sequence.toInt() > 0 ? QString::number(sequence.toInt()) : QString());
    set(preparation_, preparation_state == "already_running" || preparation_state == "started" ?
      tr("Servidor preparado") : preparation_state == "preparing" ? tr("Preparando servidor…") :
      preparation_state == "failed" ? tr("Preparación sin confirmar") : QString());
    prepare_->setEnabled(mode == "observe" && rws && connected && !stale &&
      prepare_command_available && preparation_state != "preparing" &&
      (!prepare_request_.isValid() || prepare_request_.elapsed() > 2000));

    const auto error = incident.value("detail").toString(status.value("detail").toString());
    const auto cause = incident.value("cause").toString(status.value("cause").toString());
    set(problem_, joined({error, cause}));
    set(recommended_, incident.value("action").toString(status.value("action").toString()));
    QStringList detail;
    const QUrl endpoint(config.value("endpoint").toString());
    if (endpoint.isValid() && !endpoint.host().isEmpty() &&
      (endpoint.scheme() == "http" || endpoint.scheme() == "https") && endpoint.userInfo().isEmpty())
    {
      QUrl address;
      address.setScheme(endpoint.scheme());
      address.setHost(endpoint.host());
      address.setPort(endpoint.port());
      detail << tr("Endpoint RWS · %1").arg(address.toDisplayString());
    }
    if (http.isDouble()) {
      detail << tr("HTTP %1").arg(http.toInt());
    }
    const auto prepare_http = preparation.value("http_status");
    if (prepare_http.isDouble()) {
      detail << tr("Preparación · HTTP %1").arg(prepare_http.toInt());
    }
    if (transport.value("abb_code").isDouble()) {
      detail << tr("Código ABB %1").arg(transport.value("abb_code").toInt());
    }
    if (reply.size() == 2) {
      detail << tr("Respuesta RAPID · secuencia %1 · fase %2").arg(reply[0].toInt()).arg(reply[1].toInt());
    }
    if (pointer.size() == 2) {
      detail << tr("Puntero RAPID · %1 / %2").arg(pointer[0].toString(), pointer[1].toString());
    }
    const auto motion = transport.value("motion_evidence").toObject();
    for (const auto & item : {
      std::pair<const char *, QString>{"order_http_s", tr("Tiempo de POST RWS")},
      {"observed_phase_2_to_terminal_s", tr("Fase 2 a terminal observadas")},
      {"controller_timing_s", tr("Duración interna ABB")}})
    {
      if (!motion.isEmpty()) {
        detail << tr("%1 · %2").arg(item.second, available(metric(motion.value(item.first), tr("s"))));
      }
    }
    for (const auto & message : {status.value("code").toString(), preparation.value("detail").toString(),
      packet.value("status").toString()})
    {
      if (!message.isEmpty()) {
        detail << message;
      }
    }
    const auto details = detail.isEmpty() ? tr("Sin datos") : detail.join('\n');
    if (technical_->toPlainText() != details) {
      technical_->setPlainText(details);
    }

    const auto * analysis = find_analysis_();
    auto capture = analysis ? analysis->captureSummary() : AnalysisWidget::CaptureSummary{};
    set(session_, source.contains("mock", Qt::CaseInsensitive) ? tr("Mock") :
      offline ? tr("Offline") : rws && source.compare("rws", Qt::CaseInsensitive) == 0 ? tr("RWS real") :
      source == "rws_trajectory" ? tr("RWS/RAPID — Trayectorias v2") : source == "external" ? tr("Telemetría ROS externa") : QString());
    set(trial_id_, capture.trial_id);
    bool date_ok = false;
    const auto ns = capture.timestamp_ns.toLongLong(&date_ok);
    set(trial_time_, date_ok && ns > 0 ? QDateTime::fromMSecsSinceEpoch(ns / 1000000)
      .toString(QStringLiteral("dd/MM/yyyy HH:mm:ss t")) : QString());
    set(trial_kind_, capture.trial_type);
    set(trial_result_, resultLabel(capture.trial_result));
    set(counts_, capture.samples || capture.events ?
      tr("%1 muestras · %2 eventos").arg(capture.samples).arg(capture.events) : QString());
    set(evidence_order_, sequence.toInt() > 0 ? QString::number(sequence.toInt()) : QString());
    set(exported_, export_message_.isEmpty() && !capture.exported_directory.isEmpty() ?
      tr("Guardado en %1").arg(capture.exported_directory) : export_message_);
    export_session_->setEnabled(capture.samples > 0);
    export_trial_->setEnabled(!capture.trial_id.isEmpty());
    export_task_->setEnabled(!task.value("plan_id").toString().isEmpty());
    folder_->setEnabled(QFileInfo(evidenceDirectory()).isDir());
  }

private:
  static QString tr(const char * text)
  {
    return QObject::tr(text);
  }

  static QString available(const QString & text)
  {
    return text.isEmpty() ? tr("Sin datos") : text;
  }

  static void set(QLabel * label, const QString & text)
  {
    label->setText(available(text));
  }

  static QString joined(
    const std::initializer_list<QString> & fields, const QString & separator = QStringLiteral(" · "))
  {
    QStringList parts;
    for (const auto & field : fields) {
      if (!field.isEmpty()) {
        parts << field;
      }
    }
    return parts.join(separator);
  }

  static QString metric(const QJsonValue & value, const QString & unit)
  {
    return value.isDouble() ? QStringLiteral("%1 %2").arg(value.toDouble(), 0, 'f', 2).arg(unit) : QString();
  }

  static QString motorState(const QString & value)
  {
    if (value == "motoron") {return tr("Motores encendidos");}
    if (value == "motoroff") {return tr("Motores apagados");}
    return value;
  }

  static QString resultLabel(const QString & result)
  {
    if (result == "verified") {return tr("Última ejecución verificada");}
    if (result == "executing") {return tr("Orden ejecutándose");}
    if (result == "accepted") {return tr("Orden aceptada");}
    if (result == "transmitted") {return tr("Orden transmitida");}
    if (result == "unknown") {return tr("Resultado desconocido");}
    if (result == "requested") {return tr("Orden solicitada");}
    if (result == "failed") {return tr("Ejecución fallida");}
    if (result == "cancelled") {return tr("Cancelada");}
    if (result == "completed") {return tr("Completada");}
    if (result == "running") {return tr("En curso");}
    if (result == "simulating") {return tr("Simulación en curso");}
    if (result == "simulated") {return tr("Simulación completada");}
    if (result == "planned") {return tr("Planificada");}
    if (result == "invalid" || result == "stale") {return tr("Requiere revisión");}
    if (result == "idle") {return tr("Sin orden en curso");}
    return result;
  }

  static QFormLayout * section(QVBoxLayout * layout, const QString & name)
  {
    auto * group = new QGroupBox(name);
    auto * form = new QFormLayout(group);
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    layout->addWidget(group);
    return form;
  }

  static QLabel * row(QFormLayout * form, const QString & label)
  {
    auto * value = new QLabel(tr("Sin datos"));
    value->setTextFormat(Qt::PlainText);
    value->setWordWrap(true);
    value->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(label, value);
    return value;
  }

  static QPushButton * button(QVBoxLayout * layout, const QString & label)
  {
    auto * control = new QPushButton(label);
    layout->addWidget(control);
    return control;
  }

  static QString evidenceDirectory()
  {
    return QFileInfo(QStringLiteral("/evidencia")).isDir() ? QStringLiteral("/evidencia") : QDir::homePath();
  }

  void exportCapture(bool latest)
  {
    auto * analysis = find_analysis_();
    if (!analysis) {
      return;
    }
    const auto trial = latest ? analysis->captureSummary().trial_id : QString();
    if (latest && trial.isEmpty()) {
      return;
    }
    const auto directory = QFileDialog::getExistingDirectory(export_session_,
      latest ? tr("Exportar ensayo") : tr("Exportar sesión"), evidenceDirectory());
    if (directory.isEmpty()) {
      return;
    }
    QString error;
    const bool exported = analysis->exportEvidence(directory, &error, trial);
    export_message_ = exported ? QString() : error;
    set(exported_, exported ? tr("Guardado en %1").arg(analysis->captureSummary().exported_directory) : error);
  }

  std::function<AnalysisWidget *()> find_analysis_;
  QElapsedTimer coordinator_seen_;
  QElapsedTimer prepare_request_;
  QString last_stamp_;
  QString export_message_;
  QLabel * coordinator_{};
  QLabel * operation_{};
  QLabel * backend_{};
  QLabel * source_{};
  QLabel * freshness_{};
  QLabel * feedback_age_{};
  QLabel * connection_{};
  QLabel * authentication_{};
  QLabel * controller_state_{};
  QLabel * rapid_state_{};
  QLabel * controller_task_{};
  QLabel * program_pointer_{};
  QLabel * authorization_{};
  QLabel * execution_state_{};
  QLabel * last_order_{};
  QLabel * preparation_{};
  QLabel * problem_{};
  QLabel * recommended_{};
  QLabel * session_{};
  QLabel * trial_id_{};
  QLabel * trial_time_{};
  QLabel * trial_kind_{};
  QLabel * trial_result_{};
  QLabel * counts_{};
  QLabel * evidence_order_{};
  QLabel * exported_{};
  QPlainTextEdit * technical_{};
  QPushButton * prepare_{};
  QPushButton * export_session_{};
  QPushButton * export_trial_{};
  QPushButton * export_task_{};
  QPushButton * folder_{};
};
}  // namespace imt342_irb120_rviz

#endif  // IMT342_IRB120_RVIZ__OBSERVABILITY_HPP_
