#include "../src/analysis_widget.hpp"
#include "../src/observability.hpp"

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QThread>

#include <iostream>

namespace
{
QJsonObject sample(int index, bool stale = false)
{
  const double t = 0.1 * index;
  const QString stamp = QString::number(1700000000000000000LL + 100000000LL * index);
  const auto angles = [](double x) {return QJsonArray{x, 0, 0, 0, 0, 0};};
  return QJsonObject{
    {"elapsed_s", t}, {"timestamp_ns_text", stamp}, {"timestamp_ns", stamp},
    {"source", stale ? "MEDIDO (SIMULADO/MOCK)" : "SIMULACIÓN OFFLINE"},
    {"preview_epoch", index < 20 ? 1 : 2},
    {"preview_run_id", index < 20 ? "preview-test-1" : "preview-test-2"},
    {"mode", stale ? "observe" : "offline"},
    {"base_frame", "base_link"}, {"joint_unit", "rad"},
    {"stale", stale},
    {"diagnostic", QJsonObject{{"severity", stale ? "STALE" : "OFFLINE"}}},
    {"incident", index == 20 ?
      QJsonObject{{"severity", "ERROR"}, {"code", "VALOR_INVÁLIDO"}, {"timestamp_ns_text", stamp}} :
      QJsonObject{}},
    {"actual", stale ? QJsonValue() : QJsonValue(angles(0.2 * t * t))},
    {"target", angles(0.7)}, {"preview", angles(t / 3.0)},
    {"measured_qdot", angles(0.4)},
    {"virtual_motion", QJsonObject{{"applied", true}, {"qdot", angles(t)}, {"qddot", angles(t * t)}}},
    {"jacobian_source", index < 20 ? "actual" : "plan_preview"},
    {"jacobian", QJsonObject{{"tcp", QJsonObject{
          {"sigma", QJsonArray{6.0, 5.0, 4.0, 3.0, 2.0, t / 10.0}}}}}},
    {"plan", QJsonObject{{"run_id", index < 20 ? "preview-test-1" : "preview-test-2"},
        {"phase", "running"}, {"time_s", t}, {"duration_s", 7.0}, {"running", true},
        {"q", QJsonArray{t / 3.0, 0.25, 0, 0, 0, 0}}, {"qdot", angles(t)}, {"qddot", angles(t * t)},
        {"dynamics_1r", QJsonObject{{"inertia_kg_m2", 0.08}, {"mass_kg", 1.2},
            {"com_radius_m", 0.16}, {"gravity_m_s2", 9.81}}}}}
  };
}

void refresh()
{
  QThread::msleep(135);
  QCoreApplication::processEvents();
}

bool check(bool condition, const char * failure)
{
  if (!condition) {
    std::cerr << failure << '\n';
  }
  return condition;
}

QByteArray readFile(const QString & path)
{
  QFile file(path);
  return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};
}
}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  imt342_irb120_rviz::AnalysisWidget widget;
  widget.resize(1050, 370);
  widget.show();
  QTemporaryDir destination;
  if (!check(destination.isValid() && !widget.exportEvidence(destination.path()),
      "Export without a ROS sample must fail"))
  {
    return 1;
  }

  QWidget * plot = nullptr;
  for (auto * child : widget.findChildren<QWidget *>()) {
    if (child->accessibleName() == QStringLiteral("Gráfica de análisis IRB 120")) {
      plot = child;
      break;
    }
  }
  auto * tabs = widget.findChild<QTabWidget *>();
  if (!check(plot && tabs, "Missing accessible plot or tab widget")) {
    return 1;
  }

  for (int index = 0; index < 15; ++index) {
    widget.ingest(sample(index));
  }
  refresh();
  const QImage plot_before = plot->grab().toImage();
  for (int index = 15; index < 30; ++index) {
    auto packet = sample(index);
    if (index == 20) {
      packet.insert("preview_end", QJsonObject{
          {"run_id", "preview-test-1"}, {"phase", "interrupted"}, {"reason", "new_run"}});
    }
    widget.ingest(packet);
  }
  refresh();
  if (!check(plot_before != plot->grab().toImage(), "Articular live plot failed to update")) {
    return 1;
  }
  QComboBox * plot_group = nullptr;
  QComboBox * dynamics_axis = nullptr;
  QComboBox * graph_point = nullptr;
  for (auto * box : widget.findChildren<QComboBox *>()) {
    if (box->accessibleName() == QStringLiteral("Grupo de curvas")) {
      plot_group = box;
    } else if (box->accessibleName() == QStringLiteral("Articulación del modelo 1R")) {
      dynamics_axis = box;
    } else if (box->accessibleName() == QStringLiteral("Punto de la gráfica")) {
      graph_point = box;
    }
  }
  if (!check(plot_group && plot_group->count() == 7 && plot_group->itemText(0) == QStringLiteral("Posición q") &&
      dynamics_axis && dynamics_axis->count() == 6,
      "Expected seven presentation plots and one 1R axis selector"))
  {
    return 1;
  }
  if (!check(graph_point && graph_point->currentData().toString() == "flange",
      "Expected flange as the default graph point")) {
    return 1;
  }
  graph_point->setCurrentIndex(graph_point->findData(QStringLiteral("tcp")));
  auto * legend = widget.findChild<QLabel *>("analysisPlotLegend");
  plot_group->setCurrentText(QStringLiteral("Velocidad q̇"));
  if (!check(legend && legend->text().contains(QStringLiteral("J1 · simulada")) &&
      legend->text().contains(QStringLiteral("J1 · LSPB")),
      "Offline velocity must overlay the simulated state and planned reference"))
  {
    return 1;
  }
  plot_group->setCurrentText(QStringLiteral("Valores singulares"));
  if (!check(legend->text().contains(QStringLiteral("σ1 · actual")) &&
      legend->text().contains(QStringLiteral("σ6 · LSPB")),
      "Singular values must label σ1–σ6 and the source of the joint state"))
  {
    return 1;
  }
  plot_group->setCurrentText(QStringLiteral("Dinámica 1R"));
  const auto torque_first = plot->grab().toImage();
  dynamics_axis->setCurrentIndex(1);
  if (!check(legend->text().contains(QStringLiteral("Par ideal 1R · J2")) &&
      torque_first != plot->grab().toImage(),
      "Changing the 1R joint must use its own q and acceleration without a measured torque"))
  {
    return 1;
  }
  plot_group->setCurrentText(QStringLiteral("Aceleración q̈"));
  refresh();
  const auto accel_before = plot->grab().toImage();
  widget.ingest(sample(30));
  refresh();
  if (!check(accel_before != plot->grab().toImage() &&
      legend->text().contains(QStringLiteral("J1 · simulada")) &&
      legend->text().contains(QStringLiteral("J1 · LSPB")),
      "Offline acceleration must compare the analytic simulated state and its ideal LSPB reference"))
  {
    return 1;
  }

  bool found_full_view = false;
  for (auto * selector : widget.findChildren<QComboBox *>()) {
    if (selector->accessibleName() == QStringLiteral("Vista temporal de la gráfica")) {
      found_full_view = selector->count() == 3;
      selector->setCurrentIndex(1);
      break;
    }
  }
  refresh();
  if (!check(found_full_view, "Missing complete-session view")) {
    return 1;
  }
  const QJsonObject superseded{
    {"state", "superseded"}, {"code", "INCIDENTE_RECUPERADO_DEL_HISTORIAL"}};
  auto stale = sample(31, true);
  stale.insert("incident_history", QJsonArray{superseded});
  widget.ingest(stale);
  widget.ingestRoutes(QJsonObject{{"request_id", "ensayo-1"}, {"routes", QJsonArray{
        QJsonObject{{"branch", "A"}, {"q", QJsonArray{0.7, 0, 0, 0, 0, 0}}}}}});
  widget.selectDiagnostics();
  refresh();
  const auto * diagnostic = tabs->currentWidget()->findChild<QPlainTextEdit *>();
  if (!check(diagnostic && diagnostic->toPlainText().contains("STALE"),
      "The diagnostics page did not report STALE"))
  {
    return 1;
  }

  widget.selectEvidence();
  QString error;
  if (!check(widget.exportEvidence(destination.path(), &error), qPrintable(error))) {
    return 1;
  }
  const QDir parent(destination.path());
  const auto folders = parent.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  if (!check(folders.size() == 1, "Expected one evidence folder")) {
    return 1;
  }
  const QDir evidence(parent.filePath(folders.first()));
  const auto manifest = QJsonDocument::fromJson(readFile(evidence.filePath("manifest.json"))).object();
  const auto samples = QJsonDocument::fromJson(readFile(evidence.filePath("muestras.json"))).array();
  const auto runs = QJsonDocument::fromJson(readFile(evidence.filePath("ensayos.json"))).array();
  const auto events = QJsonDocument::fromJson(readFile(evidence.filePath("eventos.json"))).array();
  const auto routes = QJsonDocument::fromJson(readFile(evidence.filePath("rutas.json"))).object();
  const auto s14 = readFile(evidence.filePath("S14_P1-P7.md"));
  const bool data_ok = manifest.value("muestras").toInt() == 32 && samples.size() == 32 &&
    samples.last().toObject().value("stale").toBool() && samples.last().toObject().value("actual").isNull() &&
    samples.last().toObject().value("timestamp_ns_text").toString() == "1700000003100000000" &&
    samples.at(19).toObject().value("plot_segment").toInt() !=
      samples.at(20).toObject().value("plot_segment").toInt() &&
    samples.at(20).toObject().value("plot_time_s").isDouble() &&
    runs.size() == 2 && manifest.value("ensayos_previsualizados_en_buffer").toInt() == 2 &&
    routes.value("vigente").toObject().value("request_id").toString() == "ensayo-1" &&
    events.size() >= 4 && s14.contains("## P7") &&
    readFile(evidence.filePath("eventos.json")).contains("INCIDENTE_RECUPERADO_DEL_HISTORIAL") &&
    readFile(evidence.filePath("muestras.csv")).count('\n') == 33 &&
    !QImage(evidence.filePath("grafica.png")).isNull();
  if (!check(data_ok, "Evidence contents failed round trip")) {
    return 1;
  }

  QTemporaryDir selected_dir;
  if (!check(widget.exportEvidence(selected_dir.path(), &error, "preview-test-1"), qPrintable(error))) {
    return 1;
  }
  const QDir selected_parent(selected_dir.path());
  const auto selected_folders = selected_parent.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  if (!check(selected_folders.size() == 1, "Expected one selected evidence folder")) {
    return 1;
  }
  const QDir selected(selected_parent.filePath(selected_folders.first()));
  const auto selected_manifest = QJsonDocument::fromJson(
    readFile(selected.filePath("manifest.json"))).object();
  const auto selected_samples = QJsonDocument::fromJson(readFile(selected.filePath("muestras.json"))).array();
  const auto selected_runs = QJsonDocument::fromJson(readFile(selected.filePath("ensayos.json"))).array();
  if (!check(selected_manifest.value("ensayo_filtrado") == "preview-test-1" &&
      selected_samples.size() == 20 && selected_runs.size() == 1 &&
      selected_runs.first().toObject().value("phase") == "interrupted" &&
      selected_runs.first().toObject().value("fin").toObject().value("reason") == "new_run" &&
      selected_manifest.value("estado_final").toObject().value("preview_run_id") == "preview-test-1" &&
      !readFile(selected.filePath("muestras.json")).contains("preview-test-2") &&
      !readFile(selected.filePath("rutas.json")).contains("ensayo-1") &&
      !readFile(selected.filePath("eventos.json")).contains("INCIDENTE_RECUPERADO_DEL_HISTORIAL") &&
      !QImage(selected.filePath("grafica.png")).isNull(),
      "Selected export must scope samples, runs, routes, events, final state and graph"))
  {
    return 1;
  }

  auto * clear = widget.findChild<QPushButton *>("limpiarHistorial");
  if (!check(clear, "Missing clear-history control")) {
    return 1;
  }
  clear->click();
  if (!check(!widget.exportEvidence(selected_dir.path(), &error, "preview-test-1"),
    "Cleared run cannot be selected again"))
  {
    return 1;
  }
  auto previous = sample(32, true);
  previous.insert("incident_history", QJsonArray{superseded});
  widget.ingest(previous);
  if (!check(!widget.exportEvidence(destination.path(), &error),
      "A republished sample from the cleared run must not recreate its evidence"))
  {
    return 1;
  }

  auto next = sample(33);
  next.insert("preview_run_id", "preview-test-3");
  auto plan = next.value("plan").toObject();
  plan.insert("run_id", "preview-test-3");
  next.insert("plan", plan);
  const QJsonObject resolution{
    {"state", "resolved"}, {"code", "VALOR_INVÁLIDO"}, {"timestamp_ns_text", "1700000002000000000"},
    {"resolved_timestamp_ns", "1700000003300000000"}};
  next.insert("incident_resolution", resolution);
  next.insert("incident_history", QJsonArray{superseded, resolution});
  widget.ingest(next);
  QTemporaryDir fresh;
  if (!check(fresh.isValid() && widget.exportEvidence(fresh.path(), &error), qPrintable(error))) {
    return 1;
  }
  const QDir clean_parent(fresh.path());
  const auto clean_folders = clean_parent.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  if (!check(clean_folders.size() == 1, "Expected one evidence folder after reset")) {
    return 1;
  }
  const QDir clean(clean_parent.filePath(clean_folders.first()));
  const auto clean_samples = QJsonDocument::fromJson(readFile(clean.filePath("muestras.json"))).array();
  const auto clean_runs = QJsonDocument::fromJson(readFile(clean.filePath("ensayos.json"))).array();
  const auto clean_routes = QJsonDocument::fromJson(readFile(clean.filePath("rutas.json"))).object();
  const auto clean_events = readFile(clean.filePath("eventos.json"));
  if (!check(clean_samples.size() == 1 && clean_runs.size() == 1 &&
      clean_runs.first().toObject().value("run_id").toString() == "preview-test-3" &&
      clean_routes.value("vigente").toObject().isEmpty() &&
      clean_routes.value("historial").toArray().isEmpty() && clean_events.count("resuelto") == 1 &&
      !clean_events.contains("INCIDENTE_RECUPERADO_DEL_HISTORIAL"),
    "New capture must contain only the new run and post-clear events"))
  {
    return 1;
  }

  auto task = sample(34);
  task.remove("plan");
  task.insert("task", QJsonObject{{"plan_id", "task-test"}, {"state", "simulated"}});
  widget.ingest(task);
  const auto summary = widget.captureSummary();
  QTemporaryDir task_dir;
  if (!check(summary.samples == 2 && summary.events > 0 && summary.trial_id == "task-test" &&
      summary.trial_type == QStringLiteral("Tarea") && summary.trial_result == "simulated" &&
      !summary.timestamp_ns.isEmpty() && widget.exportEvidence(task_dir.path(), &error, summary.trial_id),
      "Task evidence must share the captured history and export by its plan ID"))
  {
    return 1;
  }
  const auto scoped_task = readFile(QDir(widget.captureSummary().exported_directory).filePath("muestras.json"));
  const auto scoped_records = QJsonDocument::fromJson(scoped_task).array();
  if (!check(scoped_records.size() == 1 && scoped_records.first().toObject()
      .value("task").toObject().value("plan_id") == "task-test",
      "Task export must contain only samples with the selected plan"))
  {
    return 1;
  }

  QWidget diagnostics;
  QWidget evidence_tab;
  QVBoxLayout diagnostics_layout(&diagnostics);
  QVBoxLayout evidence_layout(&evidence_tab);
  imt342_irb120_rviz::Observability observability(&diagnostics_layout, &evidence_layout,
    []() {}, []() {}, [&widget]() {return &widget;}, []() {return false;});
  auto packet = sample(35);
  packet.insert("mode", "observe");
  packet.insert("backend", "rws");
  packet.insert("source", "rws");
  packet.insert("backend_connected", true);
  packet.insert("transport", QJsonObject{{"execution", "running"}, {"result", "verified"},
    {"cookies", "synthetic-secret"}, {"pointer", QJsonArray{"Module", "Routine"}}});
  packet.insert("controller_config", QJsonObject{{"endpoint", "http://mock:synthetic-secret@localhost/"}});
  observability.update(packet, true);
  const auto * technical = diagnostics.findChild<QPlainTextEdit *>();
  const auto controls = diagnostics.findChildren<QPushButton *>();
  if (!check(technical && !technical->toPlainText().contains("synthetic-secret") &&
      controls.size() == 1 && controls.first()->isEnabled(),
      "Observe must expose the preparation action and discard private transport fields"))
  {
    return 1;
  }
  packet.insert("stale", true);
  observability.update(packet, true);
  if (!check(!controls.first()->isEnabled(), "Stale telemetry must disable RAPID preparation")) {
    return 1;
  }

  imt342_irb120_rviz::AnalysisWidget gap_panel;
  gap_panel.resize(800, 355);
  gap_panel.show();
  auto * gap_group = [&]() -> QComboBox * {
      for (auto * box : gap_panel.findChildren<QComboBox *>()) {
        if (box->accessibleName() == QStringLiteral("Grupo de curvas")) {
          return box;
        }
      }
      return nullptr;
    }();
  if (!check(gap_group, "Missing graph selector")) {
    return 1;
  }
  for (int i : {0, 1, 5}) {
    gap_panel.ingest(sample(i));
  }
  gap_group->setCurrentText(QStringLiteral("Aceleración q̈"));
  auto * gap_legend = gap_panel.findChild<QLabel *>("analysisPlotLegend");
  if (!check(gap_legend && gap_legend->text().contains(QStringLiteral("J1 · simulada")),
      "Irregular timestamps must not suppress the analytic simulated acceleration"))
  {
    return 1;
  }
  gap_panel.findChild<QPushButton *>("limpiarHistorial")->click();
  // Missing derivatives in older captures and packets from a physical session
  // must not generate a fake simulated acceleration curve.
  auto old_packet = sample(21);
  old_packet.remove("virtual_motion");
  gap_panel.ingest(old_packet);
  auto online_packet = sample(22);
  online_packet.insert("mode", "observe");
  online_packet.insert("source", "rws");
  online_packet.insert("backend", "rws");
  gap_panel.ingest(online_packet);
  refresh();
  if (!check(!gap_legend->text().contains(QStringLiteral("J1 · simulada")) &&
      gap_legend->text().contains(QStringLiteral("J1 · LSPB")),
      "Only Offline packets with analytic derivatives can show simulated acceleration"))
  {
    return 1;
  }
  auto idle_packet = sample(23);
  const QJsonArray zeroes{0, 0, 0, 0, 0, 0};
  idle_packet.insert("virtual_motion", QJsonObject{
      {"applied", false}, {"qdot", zeroes}, {"qddot", zeroes}});
  gap_panel.ingest(idle_packet);
  refresh();
  if (!check(gap_legend->text().contains(QStringLiteral("J1 · simulada")) &&
    gap_legend->text().contains(QStringLiteral("J1 · LSPB")),
    "Published stationary Offline state must remain visible beside the plan"))
  {
    return 1;
  }
  auto v2_measurement = sample(24);
  v2_measurement.insert("mode", "observe");
  v2_measurement.insert("source", "rws_trajectory");
  v2_measurement.insert("backend", "rws_trajectory");
  auto quintic = v2_measurement.value("plan").toObject();
  quintic.insert("profile_id", "quintic");
  v2_measurement.insert("plan", quintic);
  gap_panel.ingest(v2_measurement);
  gap_group->setCurrentText(QStringLiteral("Posición q"));
  refresh();
  if (!check(gap_legend->text().contains(QStringLiteral("J1 · medida v2")) &&
      !gap_legend->text().contains(QStringLiteral("J1 · medida Normal")),
      "The live window must follow the selected RWS context"))
  {
    return 1;
  }
  for (auto * selector : gap_panel.findChildren<QComboBox *>()) {
    if (selector->accessibleName() == QStringLiteral("Vista temporal de la gráfica")) {
      selector->setCurrentText(QStringLiteral("Comparar backends"));
      break;
    }
  }
  refresh();
  if (!check(gap_legend->text().contains(QStringLiteral("J1 · medida Normal")) &&
      gap_legend->text().contains(QStringLiteral("J1 · medida v2")) &&
      gap_legend->text().contains(QStringLiteral("J1 · objetivo")) &&
      gap_legend->text().contains(QStringLiteral("J1 · quíntico")),
      "Comparison must distinguish measurements from both RWS backends, goals and profiles"))
  {
    return 1;
  }
  gap_group->setCurrentText(QStringLiteral("Error al objetivo"));
  if (!check(gap_legend->text().contains(QStringLiteral("J1 · consigna/objetivo menos postura")),
      "Goal error requires an available actual position and target")) {
    return 1;
  }
  // The preview can be cleared before a physical run starts. A complete-run
  // view must still include the later RWS measurement and its host reference.
  for (const auto * backend_name : {"rws", "rws_trajectory"}) {
    const bool v2 = QString::fromUtf8(backend_name) == "rws_trajectory";
    const int first_index = v2 ? 40 : 30;
    for (int i = first_index; i <= first_index + 3; ++i) {
      auto physical = sample(i);
      physical.remove("plan");
      physical.insert("mode", "control");
      physical.insert("source", backend_name);
      physical.insert("backend", backend_name);
      physical.insert("physical_run", QJsonObject{
          {"id", QStringLiteral("physical-") + QString::fromUtf8(backend_name)},
          {"started_s", first_index * .1},
          {"ended_s", i == first_index + 3 ? QJsonValue(i * .1) : QJsonValue()},
          {"finished", i == first_index + 3},
          {"reference", QJsonObject{{"profile_id", "lspb"}, {"q", QJsonArray{.2, 0, 0, 0, 0, 0}},
              {"profile_qdot", QJsonArray{.1, 0, 0, 0, 0, 0}}}}});
      gap_panel.ingest(physical);
    }
    // A later preview must not displace the completed physical run.
    auto newer_preview = sample(first_index + 4);
    newer_preview.insert("mode", "control");
    newer_preview.insert("source", backend_name);
    newer_preview.insert("backend", backend_name);
    auto preview_plan = newer_preview.value("plan").toObject();
    preview_plan.insert("run_id", QStringLiteral("preview-after-physical"));
    preview_plan.insert("profile_id", QStringLiteral("quintic"));
    newer_preview.insert("plan", preview_plan);
    gap_panel.ingest(newer_preview);
    for (auto * selector : gap_panel.findChildren<QComboBox *>()) {
      if (selector->accessibleName() == QStringLiteral("Vista temporal de la gráfica")) {
        selector->setCurrentIndex(1);
      }
    }
    gap_group->setCurrentText(QStringLiteral("Posición q"));
    refresh();
    if (!check(gap_legend->text().contains(v2 ? QStringLiteral("J1 · medida v2") : QStringLiteral("J1 · medida Normal")) &&
        !gap_legend->text().contains(v2 ? QStringLiteral("J1 · medida Normal") : QStringLiteral("J1 · medida v2")) &&
        gap_legend->text().contains(QStringLiteral("J1 · LSPB")) &&
        !gap_legend->text().contains(QStringLiteral("J1 · quíntico")),
        "Complete physical run must display its measurement and reference only")) {
      return 1;
    }
    const auto name = v2 ? QStringLiteral("J1 · medida v2") : QStringLiteral("J1 · medida Normal");
    if (!check(gap_legend->text().contains(QStringLiteral("━━</span>&nbsp;") + name) &&
        gap_legend->text().contains(QStringLiteral("– –</span>&nbsp;J1 · LSPB")),
        "Physical positions must use solid lines and LSPB reference dashed lines")) {
      return 1;
    }
  }
  return 0;
}
