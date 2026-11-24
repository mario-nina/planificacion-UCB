#include "../src/task_editor.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

#include <QApplication>
#include <QFileInfo>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTimer>

namespace
{
void check_at(bool condition, int line)
{
  if (!condition) {
    std::cerr << "Falló una comprobación del editor de tareas en línea " << line << std::endl;
    std::abort();
  }
}

#define check(condition) check_at((condition), __LINE__)

QPushButton * button(QWidget * widget, const char * id)
{
  auto * found = widget->findChild<QPushButton *>(id);
  check(found != nullptr);
  return found;
}

void choose_file(QWidget * widget, const QString & file, const char * id)
{
  QTimer::singleShot(50, [file]() {
    auto * dialog = qobject_cast<QFileDialog *>(QApplication::activeModalWidget());
    check(dialog != nullptr);
    dialog->setDirectory(QFileInfo(file).absolutePath());
    auto * filename = dialog->findChild<QLineEdit *>("fileNameEdit");
    check(filename != nullptr);
    filename->setText(QFileInfo(file).fileName());
    QMetaObject::invokeMethod(dialog, "accept", Qt::DirectConnection);
  });
  button(widget, id)->click();
}
void confirm(QWidget * widget, const char * id, bool accepted = true)
{
  QTimer::singleShot(50, [accepted]() {
    auto * dialog = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
    check(dialog != nullptr);
    dialog->button(accepted ? QMessageBox::Yes : QMessageBox::No)->click();
  });
  button(widget, id)->click();
}
}  // namespace

int main(int argc, char ** argv)
{
  QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
  QApplication app(argc, argv);
  std::vector<QJsonObject> sent;
  QTabWidget tabs;
  tabs.addTab(new QWidget, "FK");
  auto * editor = new imt342_irb120_rviz::TaskEditor(
    [&sent](const QJsonObject & command) {sent.push_back(command);}, nullptr,
    []() {return QStringLiteral("123456789");});
  tabs.addTab(editor, "Tareas");
  tabs.setCurrentIndex(1);
  tabs.show();

  check(sent.empty());
  auto * plan = button(editor, "planTask");
  auto * preview = button(editor, "previewTask");
  auto * simulate = button(editor, "simulateTask");
  auto * cancel = button(editor, "cancelTask");
  auto * preset = editor->findChild<QComboBox *>("exampleTasks");
  check(preset && preset->count() == 5);
  auto * cycles = editor->findChild<QSpinBox *>("taskCycles");
  check(cycles && cycles->isVisible() && cycles->value() == 1);
  check(!plan->isVisible() && !button(editor, "saveTask")->isVisible());
  editor->refresh({}, "offline", true, {}, false, false);
  check(editor->findChild<QDoubleSpinBox *>("J2")->value() < 0);
  check(editor->findChild<QDoubleSpinBox *>("J2")->suffix().contains("°"));
  check(sent.size() == 1 && sent.back().value("action").toString() == "plan");
  const auto request = sent.back().value("request_id");
  const auto task = sent.back().value("task").toObject();
  check(task.value("offline_example").toBool());
  check(task.value("waypoints").toObject().size() == 4);
  check(task.value("waypoints").toObject().value("HOME").toObject().value("q_rad").toArray()[0] != 0);
  check(preview->isEnabled() && !simulate->isEnabled() && cancel->isEnabled());

  preview->click();
  preview->click();
  check(sent.size() == 1 && !preview->isEnabled() && cancel->isEnabled());
  QJsonObject packet{{"request_id", "otro"}, {"plan_id", "plan-1"}, {"state", "planned"}};
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  tabs.setCurrentIndex(0);
  tabs.setCurrentIndex(1);
  check(sent.size() == 1 && !simulate->isEnabled());
  packet.insert("request_id", request);
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  check(sent.size() == 2 && sent.back().value("action").toString() == "preview");
  check(sent.back().value("plan_id").toString() == "plan-1");
  packet.insert("preview_run_id", "preview-1");
  editor->refresh(
    {{"task", packet}, {"plan", QJsonObject{{"run_id", "preview-1"}, {"running", true}}}},
    "offline", true, {}, false);
  check(!simulate->isEnabled() && cancel->isEnabled());
  packet.insert("previewed", true);
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  check(simulate->isEnabled());
  simulate->click();
  check(sent.back().value("action").toString() == "simulate");
  packet.insert("state", "simulating");
  packet.insert("preview_run_id", "preview-2");
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  check(cancel->isEnabled() && !plan->isEnabled());
  cancel->click();
  check(sent.back().value("action").toString() == "cancel");

  packet.insert("state", "cancelled");
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  check(editor->hasResult());
  for (int i = 1; i < preset->count(); ++i) {
    preset->setCurrentIndex(i);
    check(sent.back().value("action").toString() == "plan");
    check(sent.back().value("task").toObject().value("name").toString() == preset->currentText());
  }
  auto size = sent.size();
  editor->refresh({{"task", packet}}, "control", true, {}, true);
  check(!preview->isEnabled() && !simulate->isEnabled() && !plan->isEnabled());
  tabs.setCurrentIndex(0);
  tabs.setCurrentIndex(1);
  check(sent.size() == size);

  const QJsonArray actual{0, 0, 0, 0, 0, 0};
  editor->refresh({{"task", packet}}, "observe", true, actual, false);
  check(!preview->isEnabled());
  editor->refresh({{"task", packet}}, "observe", true, actual, true);
  check(preview->isEnabled() && !simulate->isEnabled() && sent.size() == size);
  preview->click();
  check(sent.back().value("action").toString() == "plan");
  packet = QJsonObject{{"request_id", sent.back().value("request_id")},
    {"plan_id", "observe-plan"}, {"state", "planned"}};
  editor->refresh({{"task", packet}}, "observe", true, actual, true);
  check(sent.back().value("action").toString() == "preview" && !simulate->isEnabled());
  check(sent.back().value("plan_id").toString() == "observe-plan");

  QTemporaryDir temp;
  check(temp.isValid());
  const auto path = temp.filePath("prueba.json");
  editor->refresh({{"task", packet}}, "offline", true, {}, false);
  button(editor, "editTask")->click();
  check(button(editor, "saveTask")->isVisible());
  cycles->setValue(4);
  choose_file(editor, path, "saveTask");
  check(QFile::exists(path));
  cycles->setValue(2);
  choose_file(editor, path, "openTask");
  check(cycles->value() == 4);
  const auto loaded = sent.back().value("task").toObject();
  check(loaded.value("cycles").toInt() == 4);
  check(loaded.value("waypoints").toObject().size() == 4);
  auto * authorize = button(editor, "authorizePhysicalTask");
  auto * execute = button(editor, "executePhysicalTask");
  check(!authorize->isVisible() && !execute->isVisible());

  // Load a custom task before explicitly marking its physical review.
  auto physical_task = loaded;
  physical_task.remove("offline_example");
  QFile physical_file(temp.filePath("physical.json"));
  check(physical_file.open(QIODevice::WriteOnly));
  check(physical_file.write(QJsonDocument(physical_task).toJson()) > 0);
  physical_file.close();
  choose_file(editor, physical_file.fileName(), "openTask");
  const QJsonObject context{{"schema_version", 1}, {"model_sha256", "audited"},
    {"tool_offset", QJsonObject{{"matrix", QJsonArray{1, 0, 0, 1}}}},
    {"controller_config", QJsonObject{{"endpoint", "http://127.0.0.1"},
      {"task", "T_ROB1"}, {"module", "Imt342JointServer"}}},
    {"base_frame", "base_link"}, {"tcp_frame", "tcp_active"}, {"flange_frame", "dh_frame_6"}};
  QJsonObject analysis{{"backend", "rws"}, {"source", "rws"},
    {"physical_can_arm", true}, {"physical_task_context", context}};
  editor->refresh(analysis, "control", true, actual, true);
  check(!authorize->isEnabled() && !execute->isEnabled());
  confirm(editor, "inspectPhysicalTask");
  check(sent.back().value("action").toString() == "plan");
  const auto reviewed = sent.back().value("task").toObject();
  check(reviewed.value("physical_reviewed").toBool() && reviewed.value("physical_context") == context);

  auto * edit_dialog = editor->findChild<QDialog *>();
  check(edit_dialog != nullptr);
  edit_dialog->close();
  packet = QJsonObject{{"request_id", sent.back().value("request_id")}, {"plan_id", "physical-plan"},
    {"state", "planned"}, {"physical_context", context}, {"physical_eligible", true},
    {"previewed", true}, {"authorization_id", "old"}};
  analysis.insert("task", packet);
  editor->refresh(analysis, "control", true, actual, true);
  check(authorize->isEnabled() && !execute->isEnabled());
  confirm(editor, "authorizePhysicalTask");
  check(sent.back().value("action").toString() == "authorize");
  check(sent.back().value("stamp_ns").toString() == "123456789");
  check(!authorize->isEnabled() && !execute->isEnabled() && cancel->isEnabled());
  packet.insert("authorization_id", "echo");
  packet.insert("authorization_stamp_ns", "11111111");
  analysis.insert("task", packet);
  editor->refresh(analysis, "control", true, actual, true);
  check(!execute->isEnabled());
  packet.insert("authorization_stamp_ns", "123456789");
  analysis.insert("task", packet);
  editor->refresh(analysis, "control", true, actual, true);
  check(execute->isEnabled());
  execute->click();
  check(sent.back().value("action").toString() == "execute");
  check(sent.back().value("authorization_id").toString() == "echo");
  size = sent.size();
  execute->click();
  check(sent.size() == size);

  packet.insert("state", "executing");
  packet.insert("active", true);
  analysis.insert("task", packet);
  editor->refresh(analysis, "control", true, actual, true);
  check(cancel->isEnabled() && !execute->isEnabled());
  cancel->click();
  check(sent.back().value("action").toString() == "cancel");

  std::vector<QJsonObject> catalog_commands;
  const auto catalog = temp.filePath("catalogo");
  imt342_irb120_rviz::TaskEditor catalog_editor(
    [&catalog_commands](const QJsonObject & command) {catalog_commands.push_back(command);},
    nullptr, []() {return QStringLiteral("123456789");}, catalog);
  catalog_editor.show();
  QJsonObject live{{"backend", "rws"}, {"source", "rws"},
    {"physical_can_arm", true}, {"physical_task_context", context}};
  const QJsonArray precise{0.123456789, -0.123456789, 0.499999999, 0.34, 0.12, -0.456789123};
  catalog_editor.refresh(live, "observe", true, precise, false);
  auto * physical_presets = catalog_editor.findChild<QComboBox *>("physicalTaskPresets");
  auto * waypoint = catalog_editor.findChild<QComboBox *>("taskWaypoint");
  auto * catalog_cycles = catalog_editor.findChild<QSpinBox *>("taskCycles");
  auto * catalog_preview = button(&catalog_editor, "previewTask");
  auto * catalog_authorize = button(&catalog_editor, "authorizePhysicalTask");
  auto * catalog_execute = button(&catalog_editor, "executePhysicalTask");
  auto * catalog_save = button(&catalog_editor, "saveTaskCatalog");
  auto * catalog_inspect = button(&catalog_editor, "inspectPhysicalTask");
  check(physical_presets && physical_presets->count() == 7 && physical_presets->findData("demo_cycle_live") >= 0 &&
    physical_presets->currentIndex() == -1);
  check(waypoint && catalog_cycles);

  physical_presets->setCurrentIndex(physical_presets->findData("cycle"));
  check(catalog_commands.empty() && !catalog_preview->isEnabled() && !catalog_authorize->isEnabled());
  button(&catalog_editor, "editTask")->click();
  check(catalog_save->isVisible() && catalog_save->isEnabled());
  catalog_save->click();
  QFile cycle_config(QDir(catalog).filePath("cycle.json"));
  check(cycle_config.open(QIODevice::ReadOnly));
  auto cycle_draft = QJsonDocument::fromJson(cycle_config.readAll()).object();
  cycle_config.close();
  const auto draft_points = cycle_draft.value("waypoints").toObject();
  check(draft_points.size() == 4 && draft_points.value("HOME").toObject().isEmpty());
  check(!cycle_draft.value("physical_reviewed").toBool() && catalog_cycles->maximum() == 1);
  auto * catalog_dialog = catalog_editor.findChild<QDialog *>();
  check(catalog_dialog != nullptr);
  catalog_dialog->close();

  // The live task waits for fresh RWS, then prepares a plan without opening the editor.
  physical_presets->setCurrentIndex(physical_presets->findData("joint_test_live"));
  auto * live_axis = catalog_editor.findChild<QComboBox *>("liveJointAxis");
  auto * live_offset = catalog_editor.findChild<QDoubleSpinBox *>("liveJointOffset");
  check(live_axis && live_offset && live_axis->isVisible() && live_offset->isVisible());
  check(live_axis->count() == 6 && live_offset->minimum() < -2 && live_offset->maximum() > 2);
  check(catalog_commands.empty() && !catalog_preview->isEnabled());
  auto other_telemetry = live;
  other_telemetry.insert("source", "external");
  catalog_editor.refresh(other_telemetry, "observe", true, precise, true);
  check(catalog_commands.empty() && !catalog_preview->isEnabled());
  catalog_editor.refresh(live, "observe", true, precise, true);
  check(catalog_commands.size() == 1 && catalog_commands.back().value("action") == "plan");
  const auto first_joint_request = catalog_commands.back().value("request_id");
  auto live_task = catalog_commands.back().value("task").toObject();
  check(live_task.value("kind") == "joint_test_live" && live_task.value("cycles").toInt() == 1);
  check(live_task.value("axis").toInt() == 1 && live_task.value("offset_deg").toDouble() == 0.5);
  check(!catalog_preview->isEnabled() && !catalog_authorize->isVisible());
  catalog_preview->click();
  check(catalog_commands.size() == 1);

  live.insert("task", QJsonObject{{"request_id", first_joint_request}, {"state", "invalid"},
    {"error", "OBJETIVO supera el límite de J1"}});
  catalog_editor.refresh(live, "observe", true, precise, true);
  catalog_editor.refresh(live, "observe", true, precise, true);
  check(catalog_commands.size() == 1 && !catalog_preview->isEnabled());
  live_axis->setCurrentIndex(2);
  live_offset->setValue(-7.5);
  auto * live_duration = catalog_editor.findChild<QDoubleSpinBox *>("liveJointDuration");
  auto * live_speed = catalog_editor.findChild<QSpinBox *>("liveJointSpeed");
  check(live_duration && !live_speed);
  live_duration->setValue(1.75);
  check(catalog_commands.back().value("action") == "plan");
  live_task = catalog_commands.back().value("task").toObject();
  check(live_task.value("axis").toInt() == 3 && live_task.value("offset_deg").toDouble() == -7.5 &&
    live_task.value("duration_s").toDouble() == 1.75 && !live_task.contains("velocity_percent"));
  const auto adjusted_joint_request = catalog_commands.back().value("request_id");
  check(adjusted_joint_request != first_joint_request && !catalog_preview->isEnabled());
  QJsonObject joint_packet{{"request_id", first_joint_request}, {"plan_id", "old-joint-plan"},
    {"state", "planned"}, {"order", QJsonArray{"OBJETIVO", "INICIO"}},
    {"physical_context", context}, {"previewed", false}};
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "observe", true, precise, true);
  check(!catalog_preview->isEnabled());
  joint_packet.insert("request_id", adjusted_joint_request);
  joint_packet.insert("plan_id", "observe-joint-plan");
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "observe", true, precise, true);
  check(catalog_preview->isEnabled() && !catalog_authorize->isVisible());
  catalog_preview->click();
  check(catalog_commands.back().value("action") == "preview" &&
    catalog_commands.back().value("plan_id") == "observe-joint-plan");

  joint_packet.insert("previewed", true);
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "observe", true, precise, true);
  check(!catalog_authorize->isVisible() && !catalog_execute->isVisible());

  auto stale_origin = joint_packet;
  stale_origin.insert("state", "stale");
  stale_origin.insert("previewed", false);
  live.insert("task", stale_origin);
  auto moved = precise;
  moved.replace(0, 0.13);
  catalog_editor.refresh(live, "observe", true, moved, true);
  check(catalog_commands.back().value("action") == "plan");
  const auto recovered_request = catalog_commands.back().value("request_id");
  check(recovered_request != adjusted_joint_request && !catalog_preview->isEnabled());
  size = catalog_commands.size();
  catalog_editor.refresh(live, "observe", true, moved, true);
  check(catalog_commands.size() == size);

  catalog_editor.refresh(live, "control", true, precise, true);
  check(catalog_commands.back().value("action") == "plan");
  check(catalog_commands.back().value("request_id") != recovered_request);
  joint_packet.insert("request_id", catalog_commands.back().value("request_id"));
  joint_packet.insert("plan_id", "control-joint-plan");
  joint_packet.insert("physical_eligible", false);
  joint_packet.insert("previewed", false);
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "control", true, precise, true);
  check(catalog_preview->isEnabled() && !catalog_authorize->isEnabled() && !catalog_execute->isEnabled());
  catalog_preview->click();
  check(catalog_commands.back().value("action") == "preview");
  joint_packet.insert("physical_eligible", true);
  joint_packet.insert("previewed", true);
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "control", true, precise, true);
  check(catalog_authorize->isEnabled() && !catalog_execute->isEnabled());
  size = catalog_commands.size();
  confirm(&catalog_editor, "authorizePhysicalTask", false);
  check(catalog_commands.size() == size && !catalog_execute->isEnabled());
  confirm(&catalog_editor, "authorizePhysicalTask");
  check(catalog_commands.back().value("action") == "authorize");
  check(catalog_commands.back().value("reviewed_live_route").toBool());
  check(!catalog_execute->isEnabled());
  joint_packet.insert("authorization_id", "single-use-joint");
  joint_packet.insert("authorization_stamp_ns", "123456789");
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "control", true, precise, true);
  check(catalog_execute->isEnabled());
  catalog_execute->click();
  check(catalog_commands.back().value("action") == "execute" &&
    catalog_commands.back().value("authorization_id") == "single-use-joint");
  size = catalog_commands.size();
  catalog_execute->click();
  check(catalog_commands.size() == size);

  joint_packet.insert("state", "verified");
  joint_packet.remove("authorization_id");
  live.insert("task", joint_packet);
  catalog_editor.refresh(live, "control", true, precise, true);

  physical_presets->setCurrentIndex(physical_presets->findData("repeat"));
  check(catalog_cycles->minimum() == 2 && !catalog_preview->isEnabled());
  catalog_cycles->setValue(4);
  button(&catalog_editor, "editTask")->click();
  catalog_save->click();
  catalog_dialog->close();

  QJsonObject cycle_points;
  for (const auto * name : {"A", "B", "C", "HOME"}) {
    cycle_points.insert(name, QJsonObject{{"q_rad", precise}});
  }
  cycle_draft.insert("waypoints", cycle_points);
  cycle_draft.insert("physical_reviewed", true);
  cycle_draft.insert("physical_context", context);
  cycle_draft.insert("physical_review_waypoints", cycle_points);
  check(cycle_config.open(QIODevice::WriteOnly | QIODevice::Truncate));
  check(cycle_config.write(QJsonDocument(cycle_draft).toJson()) > 0);
  cycle_config.close();

  physical_presets->setCurrentIndex(physical_presets->findData("cycle"));
  check(catalog_commands.back().value("action") == "plan");
  const auto cycle = catalog_commands.back().value("task").toObject();
  check(cycle.value("kind") == "cycle" && cycle.value("waypoints") == cycle_points &&
    cycle.value("physical_context") == context);
  check(!catalog_preview->isEnabled() && !catalog_dialog->isVisible());
  physical_presets->setCurrentIndex(physical_presets->findData("repeat"));
  check(!catalog_preview->isEnabled() && catalog_cycles->value() == 4);
  check(!catalog_inspect->isEnabled());
  catalog_cycles->setValue(3);
  check(catalog_commands.back().value("action") == "plan");
  const auto repeat = catalog_commands.back().value("task").toObject();
  check(repeat.value("kind").toString() == "repeat" && repeat.value("cycles").toInt() == 3);
  check(repeat.value("waypoints") == cycle_points &&
    repeat.value("physical_review_waypoints") == cycle_points);
  check(repeat.value("physical_context") == context);

  QJsonObject repeat_status{{"request_id", catalog_commands.back().value("request_id")},
    {"plan_id", "catalog-repeat"}, {"state", "planned"}, {"physical_context", context},
    {"previewed", false}, {"physical_eligible", false}, {"repeat_validated", false},
    {"physical_block_reason", "Falta verificar un primer ciclo físico"}};
  auto repeat_analysis = live;
  repeat_analysis.insert("task", repeat_status);
  catalog_editor.refresh(repeat_analysis, "control", true, precise, true);
  check(catalog_preview->isEnabled() && !catalog_authorize->isEnabled() &&
    !catalog_execute->isEnabled());
  catalog_preview->click();
  check(catalog_commands.back().value("action") == "preview");
  repeat_status.insert("previewed", true);
  repeat_analysis.insert("task", repeat_status);
  catalog_editor.refresh(repeat_analysis, "control", true, precise, true);
  check(!catalog_authorize->isEnabled());
  repeat_status.insert("repeat_validated", true);
  repeat_status.remove("physical_block_reason");
  repeat_status.insert("physical_eligible", true);
  repeat_analysis.insert("task", repeat_status);
  catalog_editor.refresh(repeat_analysis, "control", true, precise, true);
  check(catalog_authorize->isEnabled());

  catalog_cycles->setValue(2);
  check(catalog_commands.back().value("action") == "plan" &&
    catalog_commands.back().value("request_id") != repeat_status.value("request_id"));
  check(!catalog_preview->isEnabled() && !catalog_authorize->isEnabled() &&
    !catalog_execute->isEnabled());

  std::vector<QJsonObject> reopened_commands;
  imt342_irb120_rviz::TaskEditor reopened_editor(
    [&reopened_commands](const QJsonObject & command) {reopened_commands.push_back(command);},
    nullptr, []() {return QStringLiteral("123456789");}, catalog);
  auto fresh_session = live;
  fresh_session.remove("task");
  reopened_editor.refresh(fresh_session, "observe", true, precise, true);
  auto * reopened_presets = reopened_editor.findChild<QComboBox *>("physicalTaskPresets");
  check(reopened_presets != nullptr);
  reopened_presets->setCurrentIndex(reopened_presets->findData("repeat"));
  check(reopened_commands.size() == 1 && reopened_commands.back().value("action") == "plan");
  const auto reopened = reopened_commands.back().value("task").toObject();
  check(reopened.value("kind") == "repeat" && reopened.value("cycles").toInt() == 4);
  check(reopened.value("waypoints") == cycle_points && reopened.value("physical_context") == context);

  std::vector<QJsonObject> custom_commands;
  std::vector<QString> backend_selections;
  const auto custom_directory = temp.filePath("personalizadas");
  const auto source = []() {return QJsonArray{.2, 0, .02, 0, 0, 0};};
  imt342_irb120_rviz::TaskEditor custom(
    [&custom_commands](const QJsonObject & command) {custom_commands.push_back(command);},
    nullptr, {}, custom_directory, source,
    [&backend_selections](const QString & backend) {backend_selections.push_back(backend);});
  custom.show();
  custom.refresh(QJsonObject{{"backend", "rws_trajectory"}}, "offline", true, actual, true);
  button(&custom, "editTask")->click();
  button(&custom, "newPhysicalTask")->click();
  auto * name = custom.findChild<QLineEdit *>("customTaskName");
  auto * duration = custom.findChild<QDoubleSpinBox *>("waypointDuration");
  auto * task_profile = custom.findChild<QComboBox *>("taskReferenceProfile");
  auto * waypoint_profile = custom.findChild<QComboBox *>("waypointReferenceProfile");
  auto * repeats = custom.findChild<QSpinBox *>("taskCycles");
  auto * backend = custom.findChild<QComboBox *>("taskBackend");
  check(name && duration && task_profile && waypoint_profile && repeats && backend && backend->count() == 3);
  backend->setCurrentIndex(backend->findData(QStringLiteral("rws_trajectory")));
  QMetaObject::invokeMethod(backend, "activated", Qt::DirectConnection, Q_ARG(int, backend->currentIndex()));
  check(backend_selections == std::vector<QString>{QStringLiteral("rws_trajectory")});
  name->setText("Secuencia de prácticas");
  QMetaObject::invokeMethod(name, "editingFinished", Qt::DirectConnection);
  button(&custom, "copyFkIkTaskWaypoint")->click();
  duration->setValue(1.4);
  waypoint_profile->setCurrentIndex(waypoint_profile->findData(QStringLiteral("quintic")));
  button(&custom, "saveTaskWaypoint")->click();
  button(&custom, "addTaskDestination")->click();
  custom.findChild<QDoubleSpinBox *>("J1")->setValue(-.1);
  button(&custom, "saveTaskWaypoint")->click();
  task_profile->setCurrentIndex(task_profile->findData(QStringLiteral("lspb")));
  repeats->setValue(3);
  button(&custom, "moveTaskDestinationUp")->click();
  check(custom_commands.back().value("action") == "plan");
  const auto multi = custom_commands.back().value("task").toObject();
  check(multi.value("kind") == "custom" && multi.value("name") == "Secuencia de prácticas");
  check(multi.value("order") == QJsonArray({"P2", "P1"}));
  check(multi.value("profile") == "lspb" && multi.value("cycles").toInt() == 3);
  check(multi.value("backend") == "rws_trajectory");
  const auto custom_points = multi.value("waypoints").toObject();
  check(custom_points.value("P1").toObject().value("q_rad") == source());
  check(custom_points.value("P1").toObject().value("duration_s").toDouble() == 1.4);
  check(custom_points.value("P1").toObject().value("profile") == "quintic");
  button(&custom, "saveTaskCatalog")->click();
  const auto filename = QDir(custom_directory).filePath(
    "custom-" + multi.value("catalog_id").toString() + ".json");
  check(QFileInfo::exists(filename));

  imt342_irb120_rviz::TaskEditor custom_reopened(
    [&custom_commands](const QJsonObject & command) {custom_commands.push_back(command);},
    nullptr, {}, custom_directory);
  custom_reopened.show();
  custom_reopened.refresh(QJsonObject{{"backend", "rws"}}, "offline", true, actual, true);
  auto * presets = custom_reopened.findChild<QComboBox *>("physicalTaskPresets");
  const auto item = presets->findText("Secuencia de prácticas");
  check(item >= 3);
  const auto previous_commands = custom_commands.size();
  presets->setCurrentIndex(item);
  check(custom_commands.size() == previous_commands ||
    custom_commands.back().value("action") == "invalidate");
  custom_reopened.refresh(QJsonObject{{"backend", "rws_trajectory"}}, "offline", true, actual, true);
  check(custom_commands.back().value("task").toObject().value("waypoints") == custom_points);
  check(custom_commands.back().value("task").toObject().value("profile") == "lspb");
  check(custom_reopened.findChild<QComboBox *>("taskBackend")->currentData().toString() ==
    QStringLiteral("rws_trajectory"));
  button(&custom_reopened, "editTask")->click();
  QPushButton * duplicate = nullptr;
  for (auto * candidate : custom_reopened.findChildren<QPushButton *>()) {
    if (candidate->text() == QStringLiteral("Duplicar")) {
      duplicate = candidate;
      break;
    }
  }
  check(duplicate && duplicate->isEnabled());
  duplicate->click();
  const auto copy = custom_commands.back().value("task").toObject();
  check(copy.value("name") == QStringLiteral("Secuencia de prácticas · copia"));
  check(copy.value("catalog_id") != multi.value("catalog_id") && copy.value("waypoints") == custom_points);
  button(&custom_reopened, "saveTaskCatalog")->click();
  const auto copy_filename = QDir(custom_directory).filePath(
    "custom-" + copy.value("catalog_id").toString() + ".json");
  check(QFileInfo::exists(copy_filename) && QFileInfo::exists(filename));
  confirm(&custom_reopened, "deleteTaskCatalog");
  check(!QFileInfo::exists(copy_filename) && QFileInfo::exists(filename));
  presets->setCurrentIndex(presets->findText("Secuencia de prácticas"));
  confirm(&custom_reopened, "deleteTaskCatalog", false);
  check(QFileInfo::exists(filename));
  confirm(&custom_reopened, "deleteTaskCatalog");
  check(!QFileInfo::exists(filename) && presets->count() == 7);

  presets->setCurrentIndex(presets->findData(QStringLiteral("cycle")));
  auto * destinations = custom_reopened.findChild<QComboBox *>("taskWaypoint");
  check(destinations && destinations->count() == 4);
  const auto reorder_down = button(&custom_reopened, "moveTaskDestinationDown");
  reorder_down->click();
  check(destinations->itemText(0) == "B" && destinations->itemText(1) == "A");
  destinations->setCurrentIndex(2);
  check(!reorder_down->isEnabled() && destinations->itemText(3) == "HOME");
  destinations->setCurrentIndex(3);
  check(!button(&custom_reopened, "moveTaskDestinationUp")->isEnabled());

  // The factory demo requires no catalog JSON or waypoint edits for either backend.
  for (const auto * backend_name : {"rws", "rws_trajectory"}) {
    std::vector<QJsonObject> demo_commands;
    imt342_irb120_rviz::TaskEditor demo(
      [&demo_commands](const QJsonObject & command) {demo_commands.push_back(command);},
      nullptr, []() {return QStringLiteral("123456789");}, temp.filePath(backend_name));
    demo.show();
    QJsonObject feedback{{"backend", backend_name}, {"source", backend_name},
      {"physical_can_arm", true}, {"physical_task_context", context}};
    demo.refresh(feedback, "control", true, precise, true);
    auto * choices = demo.findChild<QComboBox *>("physicalTaskPresets");
    choices->setCurrentIndex(choices->findData("demo_cycle_live"));
    check(!demo_commands.empty() && demo_commands.back().value("action") == "plan");
    const auto selected = demo_commands.back().value("task").toObject();
    check(selected.value("kind") == "demo_cycle_live" && selected.value("cycles").toInt() == 1);
    check(!selected.contains("waypoints") && !selected.contains("physical_reviewed"));
    check(!button(&demo, "editTask")->isEnabled());
    auto request = demo_commands.back().value("request_id");
    QJsonObject planned{{"request_id", request}, {"plan_id", "demo-plan"}, {"state", "planned"},
      {"physical_context", context}, {"physical_eligible", true}, {"previewed", false}};
    feedback.insert("task", planned);
    demo.refresh(feedback, "control", true, precise, true);
    check(button(&demo, "previewTask")->isEnabled());
    check(!button(&demo, "authorizePhysicalTask")->isEnabled());
    button(&demo, "previewTask")->click();
    check(demo_commands.back().value("action") == "preview");
    planned.insert("previewed", true);
    feedback.insert("task", planned);
    demo.refresh(feedback, "control", true, precise, true);
    check(button(&demo, "authorizePhysicalTask")->isEnabled());
    confirm(&demo, "authorizePhysicalTask");
    check(demo_commands.back().value("action") == "authorize");
    check(demo_commands.back().value("reviewed_live_route").toBool());
    check(!button(&demo, "executePhysicalTask")->isEnabled());
    planned.insert("state", "verified");
    planned.insert("physical_eligible", false);
    planned.insert("active", false);
    planned.insert("steps", QJsonArray{QJsonObject{{"result", "already_satisfied"}}});
    feedback.insert("task", planned);
    demo.refresh(feedback, "control", true, precise, true);
    auto * results = demo.findChild<QTableWidget *>("taskLegResults");
    check(results && results->item(0, 2)->text() == "Destino ya satisfecho");
    check(button(&demo, "previewTask")->isEnabled());
    button(&demo, "previewTask")->click();
    check(demo_commands.back().value("action") == "plan");
    const auto next_request = demo_commands.back().value("request_id");
    check(next_request != request);
    check(button(&demo, "cancelTask")->isEnabled());
    check(!button(&demo, "previewTask")->isEnabled());

    feedback.insert("physical_order_pending", true);
    feedback.insert("task_plan_reply", QJsonObject{{"request_id", next_request},
      {"state", "invalid"}, {"error", "Resuelva primero la orden física anterior"}});
    demo.refresh(feedback, "control", true, precise, true);
    auto * progress = demo.findChild<QLabel *>("taskProgress");
    check(progress && progress->text().contains("Resuelva primero la orden física anterior"));
    check(!button(&demo, "previewTask")->isEnabled());
    feedback.insert("physical_order_pending", false);
    demo.refresh(feedback, "control", true, precise, true);
    check(demo_commands.back().value("action") == "plan");
    check(demo_commands.back().value("request_id") != next_request);

    button(&demo, "cancelTask")->click();
    check(demo_commands.back().value("action") == "invalidate");
    check(progress->text().contains("Planificación cancelada"));
    check(button(&demo, "previewTask")->isEnabled());
  }

  // The cell presets replan from the current posture without IK matrix entry.
  for (const auto * backend : {"rws", "rws_trajectory"}) {
    for (const auto * kind : {"cell_wall", "cell_perimeter", "air_triangle"}) {
      std::vector<QJsonObject> cell_commands;
      imt342_irb120_rviz::TaskEditor cell(
        [&cell_commands](const QJsonObject & command) {cell_commands.push_back(command);},
        nullptr, []() {return QStringLiteral("123456789");}, temp.filePath(kind));
      cell.show();
      QJsonObject feedback{{"backend", backend}, {"source", "SIMULACIÓN OFFLINE"}};
      cell.refresh(feedback, "offline", true, precise, true);
      auto * choices = cell.findChild<QComboBox *>("physicalTaskPresets");
      choices->setCurrentIndex(choices->findData(kind));
      auto * cell_cycles = cell.findChild<QSpinBox *>("taskCycles");
      check(cell_cycles && cell_cycles->minimum() == 1 && cell_cycles->maximum() == 8);
      check(!button(&cell, "editTask")->isVisible());
      cell_cycles->setValue(3);
      auto request = cell_commands.back();
      check(request.value("action") == "plan");
      const auto cell_task = request.value("task").toObject();
      check(cell_task.value("kind") == kind && cell_task.value("cycles").toInt() == 3);
      check(!cell_task.contains("waypoints") && !cell_task.contains("order") &&
        !cell_task.contains("duration_s") && !cell_task.contains("velocity_percent"));
      const QJsonObject offline_context{{"controller_config", QJsonObject{}},
        {"model_sha256", "modelo-offline"}};
      feedback["physical_task_context"] = offline_context;
      QJsonObject plan{{"state", "planned"}, {"plan_id", "cell-plan"}, {"cycles", 3},
        {"request_id", request.value("request_id")}, {"physical_context", offline_context},
        {"order", QJsonArray{"Aproximación", "Muro", "HOME"}},
        {"cell_path", QJsonObject{{"max_tcp_deviation_m", .001},
          {"wall_clearance_min_m", .099}, {"wall_clearance_max_m", .101},
          {"command_segments", 9}}}};
      if (QString::fromUtf8(kind) == "air_triangle") {
        plan["cell_path"] = QJsonObject{{"variant", "air_triangle"}, {"side_m", .1},
          {"max_tcp_deviation_m", .004}, {"command_segments", 7}};
      }
      feedback["task"] = plan;
      cell.refresh(feedback, "offline", true, precise, true);
      check(button(&cell, "previewTask")->isEnabled());
      check(cell.findChild<QLabel *>("taskProgress")->text().contains(
        QString::fromUtf8(kind) == "air_triangle" ? "100 mm por lado" : "99.0–101.0 mm"));
      button(&cell, "previewTask")->click();
      check(cell_commands.back().value("action") == "preview");
      plan["previewed"] = true;
      feedback["task"] = plan;
      cell.refresh(feedback, "offline", true, precise, true);
      check(button(&cell, "simulateTask")->isEnabled());
      button(&cell, "simulateTask")->click();
      check(cell_commands.back().value("action") == "simulate");

      feedback["source"] = backend;
      feedback["physical_task_context"] = context;
      cell.refresh(feedback, "control", true, precise, true);
      request = cell_commands.back();
      check(request.value("action") == "plan");
      plan["request_id"] = request.value("request_id");
      plan["physical_context"] = context;
      plan["physical_eligible"] = false;
      plan["physical_block_reason"] = "Verifique la geometría de la celda";
      plan["cell_geometry_verified"] = false;
      feedback["task"] = plan;
      cell.refresh(feedback, "control", true, precise, true);
      check(button(&cell, "previewTask")->isEnabled());
      check(!button(&cell, "authorizePhysicalTask")->isEnabled());
      check(button(&cell, "authorizePhysicalTask")->toolTip().contains("geometría de la celda"));
      plan["physical_eligible"] = true;
      // A provisional geometry may be reviewed within the existing authorization dialog.
      plan["cell_geometry_verified"] = false;
      plan.remove("physical_block_reason");
      feedback["task"] = plan;
      cell.refresh(feedback, "control", true, precise, true);
      check(button(&cell, "authorizePhysicalTask")->isEnabled());
      confirm(&cell, "authorizePhysicalTask");
      check(cell_commands.back().value("action") == "authorize" &&
        cell_commands.back().value("reviewed_live_route").toBool() &&
        cell_commands.back().value("reviewed_cell_geometry").toBool());
    }
  }

  std::vector<QJsonObject> unavailable_commands;
  imt342_irb120_rviz::TaskEditor unavailable(
    [&unavailable_commands](const QJsonObject & command) {unavailable_commands.push_back(command);},
    nullptr, {}, temp.filePath("unavailable"));
  QJsonObject capabilities{{"backend", "rws"}, {"cell_perimeter_available", false},
    {"cell_perimeter_block_reason", "El giro completo excede el rango de J1."}};
  unavailable.refresh(capabilities, "offline", true, precise, true);
  auto * catalog_combo = unavailable.findChild<QComboBox *>("physicalTaskPresets");
  auto * catalog_model = qobject_cast<QStandardItemModel *>(catalog_combo->model());
  const int perimeter_index = catalog_combo->findData("cell_perimeter");
  auto * perimeter_item = catalog_model->item(perimeter_index);
  check(perimeter_item && !perimeter_item->isEnabled() && perimeter_item->toolTip().contains("J1"));
  check(catalog_model->item(catalog_combo->findData("cell_wall"))->isEnabled());
  const auto command_count = unavailable_commands.size();
  catalog_combo->setCurrentIndex(perimeter_index);
  check(catalog_combo->currentIndex() != perimeter_index && unavailable_commands.size() == command_count);
  capabilities["cell_perimeter_available"] = true;
  unavailable.refresh(capabilities, "offline", true, precise, true);
  check(perimeter_item->isEnabled() && perimeter_item->toolTip().isEmpty());
}
