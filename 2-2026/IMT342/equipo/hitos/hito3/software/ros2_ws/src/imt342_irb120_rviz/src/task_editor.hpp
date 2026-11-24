#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <utility>

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QRegularExpression>
#include <QSet>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QStandardPaths>
#include <QStringList>
#include <QTableWidget>
#include <QUuid>
#include <QVBoxLayout>

#include "imt342_irb120_rviz/task_presets.hpp"
#include "pose_grid.hpp"

namespace imt342_irb120_rviz
{
class TaskEditor : public QWidget
{
public:
  explicit TaskEditor(
    std::function<void(const QJsonObject &)> send, QWidget * parent = nullptr,
    std::function<QString()> stamp = {}, QString catalog_directory = {},
    std::function<QJsonArray()> target = {}, std::function<void(const QString &)> select_backend = {})
  : QWidget(parent), send_(std::move(send)), stamp_(std::move(stamp)),
    catalog_directory_(std::move(catalog_directory)), target_(std::move(target)),
    select_backend_(std::move(select_backend))
  {
    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    dialog_ = new QDialog(this);
    dialog_->setWindowTitle(tr("Editar tarea"));
    dialog_->resize(540, 620);
    auto * dialog_layout = new QVBoxLayout(dialog_);
    auto * scroll = new QScrollArea;
    auto * content = new QWidget;
    auto * advanced_layout = new QVBoxLayout(content);
    scroll->setWidgetResizable(true);
    scroll->setWidget(content);
    dialog_layout->addWidget(scroll);
    auto * close = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(close, &QDialogButtonBox::rejected, dialog_, &QDialog::reject);
    dialog_layout->addWidget(close);
    build_library(layout, advanced_layout);
    build_waypoints(advanced_layout);
    build_execution(layout, advanced_layout);
    refresh_saved_catalog();
    edit_ = new QPushButton(tr("Editar tarea…"));
    edit_->setObjectName("editTask");
    layout->addWidget(edit_);
    connect(edit_, &QPushButton::clicked, dialog_, &QDialog::open);
    load_example();
  }

  void refresh(
    const QJsonObject & analysis, const QString & mode, bool connected,
    const QJsonArray & actual, bool fresh, bool radians = true)
  {
    const bool session_changed = mode_ != mode || connected_ != connected ||
      analysis.value("backend") != analysis_.value("backend");
    if (session_changed) {
      if (waiting_for_plan()) {
        request_.clear();
        progress_->setText(tr("Sin plan vigente"));
      }
      queued_preview_ = false;
      dispatch_pending_ = false;
      reset_authorization();
      if (mode == "offline" && connected && !working()) {
        auto_plan_ = true;
      }
    }
    mode_ = mode;
    connected_ = connected;
    actual_ = actual;
    fresh_ = fresh && !analysis.value("stale").toBool();
    analysis_ = analysis;
    refresh_live_limits();
    const auto context = read_physical_context();
    const bool context_changed = context != current_context_;
    if (context_changed) {
      current_context_ = context;
      reset_authorization();
      refresh_configuration();
    }
    if (radians != radians_) {
      apply_units(radians);
      refresh_goals();
    }
    auto packet = analysis.value("task").toObject();
    if (!packet.isEmpty()) {
      last_result_ = packet;
    }
    const auto plan_reply = analysis.value("task_plan_reply").toObject();
    if (!request_.isEmpty() && request_mode_ == mode_ &&
      plan_reply.value("request_id").toString() == request_)
    {
      if (analysis.value("physical_order_pending").toBool()) {
        auto_plan_ = true;
      }
      packet = plan_reply;
    }
    const auto plan = analysis.value("plan").toObject();
    const auto previous_preview = preview_running_;
    const auto preview_id = packet.value("preview_run_id").toString();
    preview_running_ = !preview_id.isEmpty() && plan.value("run_id").toString() == preview_id &&
      (plan.value("running").toBool() || plan.value("paused").toBool());
    const bool packet_changed = packet != packet_;
    if (packet_changed) {
      const bool new_candidates = packet.value("candidates") != packet_.value("candidates") ||
        packet.value("request_id") != packet_.value("request_id") ||
        packet.value("waypoint") != packet_.value("waypoint");
      packet_ = packet;
      refresh_authorization();
      if (new_candidates) {
        refresh_candidates();
      }
      if (advanced_->isChecked()) {
        update_payload();
      }
    }
    if (packet_changed || previous_preview != preview_running_) {
      refresh_progress();
      refresh_goals();
    }
    const bool catalog_stale = planning_reply_received() &&
      packet_.value("state").toString() == "stale";
    if (automatic_catalog() && !working() && (session_changed || context_changed || catalog_stale)) {
      auto_plan_ = true;
    }
    if (auto_plan_ && !waiting_for_plan() && can_plan() &&
      (mode_ == "offline" || automatic_catalog()))
    {
      request_plan();
    } else if (queued_preview_ && current_plan()) {
      queued_preview_ = false;
      if (ready_to_preview() && can_plan()) {
        send_plan_action("preview");
      }
    } else if (queued_preview_ && planning_reply_received()) {
      queued_preview_ = false;
    }
    refresh_buttons();
  }

  bool hasResult() const {return !last_result_.isEmpty();}

  bool exportResult()
  {
    if (!hasResult()) {
      QMessageBox::information(this, tr("Resultado de tarea"), tr("Aún no hay datos de tarea."));
      return false;
    }
    return save_json(tr("Exportar resultado de tarea"), "resultado-tarea.json", last_result_);
  }

private:
  static QString directory()
  {
    return QDir("/evidencia").exists() ? QStringLiteral("/evidencia") : QDir::homePath();
  }

  static QLabel * note(const QString & text)
  {
    auto * label = new QLabel(text);
    label->setWordWrap(true);
    label->setTextFormat(Qt::PlainText);
    return label;
  }

  static bool cell_kind(const QString & kind)
  {
    return kind == "cell_wall" || kind == "cell_perimeter" || kind == "air_triangle";
  }

  static QStringList waypoint_names(const QString & kind, const QJsonObject & task = {})
  {
    if (cell_kind(kind)) {
      return {};
    }
    if (kind == "custom" || ((kind.isEmpty() || kind == "cycle" || kind == "repeat") &&
      task.contains("order")))
    {
      QStringList names;
      for (const auto & value : task.value("order").toArray()) {
        names << value.toString();
      }
      return names;
    }
    if (kind == "joint_test" || kind == "joint_test_live") {
      return {"INICIO", "OBJETIVO"};
    }
    return {"A", "B", "C", "HOME"};
  }

  static QJsonObject empty_waypoints(const QString & kind)
  {
    QJsonObject waypoints;
    for (const auto & name : waypoint_names(kind)) {
      waypoints.insert(name, QJsonObject{});
    }
    return waypoints;
  }

  QString task_kind() const {return task_.value("kind").toString();}

  bool live_joint() const {return task_kind() == "joint_test_live";}

  bool live_demo() const {return task_kind() == "demo_cycle_live";}

  bool cell_route() const {return cell_kind(task_kind());}

  QString perimeter_block() const
  {
    const auto enabled = analysis_.value("cell_perimeter_available");
    if (!enabled.isBool() || enabled.toBool()) {
      return {};
    }
    const auto reason = analysis_.value("cell_perimeter_block_reason").toString();
    return reason.isEmpty() ?
      tr("El contorno completo excede el giro disponible de J1 en la celda nominal.") : reason;
  }

  void refresh_perimeter_option()
  {
    auto * model = qobject_cast<QStandardItemModel *>(physical_presets_->model());
    auto * item = model ? model->item(physical_presets_->findData("cell_perimeter")) : nullptr;
    if (item) {
      const auto reason = perimeter_block();
      item->setEnabled(reason.isEmpty());
      item->setToolTip(reason);
    }
  }

  bool live_route() const {return live_joint() || live_demo() || cell_route();}

  bool automatic_catalog() const
  {
    return live_route() || task_kind() == "cycle" || task_kind() == "repeat" || task_kind() == "custom";
  }

  QString catalog_root() const
  {
    auto root = catalog_directory_;
    if (root.isEmpty()) {
      root = QDir("/evidencia").exists() ? QStringLiteral("/evidencia/tareas") :
        QDir(QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation))
        .filePath("imt342/tareas");
    }
    return root;
  }

  QString catalog_path(const QString & key) const
  {
    return QDir(catalog_root()).filePath(key + ".json");
  }

  QString catalog_key() const
  {
    return task_kind() == "custom" ?
      QStringLiteral("custom-") + task_.value("catalog_id").toString() : task_kind();
  }

  static QString new_catalog_id()
  {
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
  }

  static bool valid_catalog_id(const QString & id)
  {
    static const QRegularExpression pattern(
      QStringLiteral("^[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}$"));
    return pattern.match(id).hasMatch();
  }

  void refresh_saved_catalog()
  {
    const QSignalBlocker block(physical_presets_);
    while (physical_presets_->count() > builtin_catalog_size_) {
      physical_presets_->removeItem(builtin_catalog_size_);
    }
    const auto files = QDir(catalog_root()).entryList({"custom-*.json"}, QDir::Files, QDir::Name);
    for (const auto & file : files) {
      const auto key = QFileInfo(file).completeBaseName();
      const auto id = key.mid(7);
      const auto task = valid_catalog_id(id) ? read_json(catalog_path(key)) : QJsonObject{};
      if (task.value("catalog_id").toString() == id && task.value("kind").toString() == "custom" &&
        file_task_valid(task))
      {
        physical_presets_->addItem(task.value("name").toString(tr("Tarea personalizada")), key);
      }
    }
    physical_presets_->setCurrentIndex(physical_presets_->findData(catalog_key()));
  }

  void build_library(QVBoxLayout * layout, QVBoxLayout * advanced)
  {
    library_ = new QWidget;
    auto * box = new QVBoxLayout(library_);
    box->setContentsMargins(0, 0, 0, 0);
    preset_ = new QComboBox;
    preset_->setAccessibleName(tr("Demostraciones Offline"));
    preset_->setObjectName("exampleTasks");
    preset_->setPlaceholderText(tr("Seleccionar demostración"));
    for (const auto & item : task_presets) {
      preset_->addItem(QString::fromUtf8(item.name));
    }
    preset_->setCurrentIndex(0);
    auto * offline_label = new QLabel(tr("Ejemplos Offline"));
    offline_label->setBuddy(preset_);
    box->addWidget(offline_label);
    box->addWidget(preset_);
    physical_presets_ = new QComboBox;
    physical_presets_->setObjectName("physicalTaskPresets");
    physical_presets_->setAccessibleName(tr("Catálogo de tareas"));
    physical_presets_->setPlaceholderText(tr("Seleccionar tarea del catálogo"));
    physical_presets_->addItem(tr("Demo lista · A–B–C–retorno (J1/J2)"), "demo_cycle_live");
    physical_presets_->addItem(tr("Prueba articular individual con retorno"), "joint_test_live");
    physical_presets_->addItem(tr("Ciclo A–B–C–HOME"), "cycle");
    physical_presets_->addItem(tr("Repetir ciclo validado"), "repeat");
    physical_presets_->addItem(tr("Recorrido de muro · separación 100 mm"), "cell_wall");
    physical_presets_->addItem(tr("Contorno perimetral · separación 100 mm"), "cell_perimeter");
    physical_presets_->addItem(tr("Triángulo en el aire · lado 100 mm"), "air_triangle");
    builtin_catalog_size_ = physical_presets_->count();
    physical_presets_->setCurrentIndex(-1);
    auto * physical_label = new QLabel(tr("Catálogo · Offline y control físico"));
    physical_label->setBuddy(physical_presets_);
    box->addWidget(physical_label);
    box->addWidget(physical_presets_);
    description_ = note({});
    box->addWidget(description_);
    configuration_ = note({});
    box->addWidget(configuration_);
    live_joint_controls_ = new QWidget;
    auto * joint_row = new QGridLayout(live_joint_controls_);
    joint_row->setContentsMargins(0, 0, 0, 0);
    live_axis_ = new QComboBox;
    live_axis_->setObjectName("liveJointAxis");
    for (int axis = 1; axis <= 6; ++axis) {
      live_axis_->addItem(tr("J%1").arg(axis), axis);
    }
    auto * joint_label = new QLabel(tr("Articulación"));
    joint_label->setBuddy(live_axis_);
    joint_row->addWidget(joint_label, 0, 0);
    joint_row->addWidget(live_axis_, 0, 1);
    live_offset_ = new QDoubleSpinBox;
    live_offset_->setObjectName("liveJointOffset");
    live_offset_->setRange(-offset_span_deg(5), offset_span_deg(5));
    live_offset_->setDecimals(2);
    live_offset_->setSingleStep(0.1);
    live_offset_->setValue(0.5);
    live_offset_->setSuffix(tr(" °"));
    live_offset_->setKeyboardTracking(false);
    auto * offset_label = new QLabel(tr("Desplazamiento"));
    offset_label->setBuddy(live_offset_);
    joint_row->addWidget(offset_label, 1, 0);
    joint_row->addWidget(live_offset_, 1, 1);
    live_duration_ = duration_field("liveJointDuration");
    auto * live_time_label = new QLabel(tr("Duración por tramo"));
    live_time_label->setBuddy(live_duration_);
    joint_row->addWidget(live_time_label, 2, 0);
    joint_row->addWidget(live_duration_, 2, 1);
    joint_row->addWidget(new QLabel(tr("Velocidad de referencia automática")), 3, 0, 1, 2);
    box->addWidget(live_joint_controls_);
    connect(live_axis_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {refresh_live_limits(); update_live_joint();});
    connect(live_offset_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [this](double) {update_live_joint();});
    connect(live_duration_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [this](double) {update_live_joint();});
    auto * cycle_row = new QHBoxLayout;
    cycles_ = new QSpinBox;
    cycles_->setRange(1, 8);
    cycles_->setAccessibleName(tr("Ciclos solicitados"));
    cycles_->setObjectName("taskCycles");
    auto * cycle_label = new QLabel(tr("Ciclos"));
    cycle_label->setBuddy(cycles_);
    cycle_row->addWidget(cycle_label);
    cycle_row->addWidget(cycles_);
    task_profile_ = profile_field("taskReferenceProfile", tr("Perfil de sesión"));
    auto * task_profile_label = new QLabel(tr("Perfil"));
    task_profile_label->setBuddy(task_profile_);
    cycle_row->addWidget(task_profile_label);
    cycle_row->addWidget(task_profile_);
    cycle_row->addStretch();
    box->addLayout(cycle_row);
    connect(task_profile_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
      store_profile(task_, task_profile_);
      if (!live_joint()) {
        clear_physical_review();
      }
      invalidate();
      refresh_configuration();
      auto_plan_ = true;
      if (mode_ == "offline" || automatic_catalog()) {
        request_plan();
      }
    });
    connect(cycles_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int count) {
      task_.insert("cycles", count);
      if (task_kind() != "repeat" && task_kind() != "custom") {
        clear_physical_review();
      }
      invalidate();
      refresh_configuration();
      auto_plan_ = true;
      if (automatic_catalog()) {
        request_plan();
      }
    });

    layout->addWidget(library_);

    auto * files = new QHBoxLayout;
    auto * duplicate = new QPushButton(tr("Duplicar"));
    auto * open = new QPushButton(tr("Abrir…"));
    auto * save = new QPushButton(tr("Guardar…"));
    save->setObjectName("saveTask");
    open->setObjectName("openTask");
    files->addWidget(duplicate);
    files->addWidget(open);
    files->addWidget(save);
    advanced->addLayout(files);
    save_catalog_ = new QPushButton(tr("Guardar en catálogo"));
    save_catalog_->setObjectName("saveTaskCatalog");
    auto * catalog_actions = new QHBoxLayout;
    delete_catalog_ = new QPushButton(tr("Eliminar del catálogo…"));
    delete_catalog_->setObjectName("deleteTaskCatalog");
    catalog_actions->addWidget(save_catalog_);
    catalog_actions->addWidget(delete_catalog_);
    advanced->addLayout(catalog_actions);
    connect(delete_catalog_, &QPushButton::clicked, this, [this]() {delete_catalog();});
    connect(save_catalog_, &QPushButton::clicked, this, [this]() {save_catalog();});
    connect(preset_, qOverload<int>(&QComboBox::activated), this,
      [this](int) {load_example();});
    connect(preset_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {load_example();});
    connect(physical_presets_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int index) {load_catalog(index);});
    connect(physical_presets_, qOverload<int>(&QComboBox::activated), this, [this](int) {
      if (automatic_catalog() && planning_reply_received() && !current_catalog_plan() &&
        !working() && approved_token_.isEmpty() && !authorize_pending_ && !execute_pending_)
      {
        auto_plan_ = true;
        request_plan();
      }
    });
    connect(duplicate, &QPushButton::clicked, this, [this]() {
      if (!has_pending_edit()) {
        auto task = task_;
        task.insert("name", task.value("name").toString(tr("Tarea")) + tr(" · copia"));
        task.remove("physical_reviewed");
        task.remove("physical_context");
        task.remove("physical_review_waypoints");
        task.remove("physical_review_order");
        if (task.value("kind").toString() == "custom") {
          task.insert("catalog_id", new_catalog_id());
        }
        loaded_example_ = -1;
        load_task(task);
      }
    });
    auto * new_physical = new QPushButton(tr("Nueva tarea personalizada"));
    new_physical->setObjectName("newPhysicalTask");
    advanced->addWidget(new_physical);
    custom_name_ = new QLineEdit;
    custom_name_->setObjectName("customTaskName");
    custom_name_->setPlaceholderText(tr("Nombre de tarea personalizada"));
    custom_name_->setAccessibleName(tr("Nombre de tarea personalizada"));
    custom_name_->setMaxLength(96);
    advanced->addWidget(custom_name_);
    auto * backend_row = new QHBoxLayout;
    task_backend_ = new QComboBox;
    task_backend_->setObjectName("taskBackend");
    task_backend_->setAccessibleName(tr("Backend de la tarea"));
    task_backend_->addItem(tr("Backend de sesión"), QString());
    task_backend_->addItem(tr("Normal"), QStringLiteral("rws"));
    task_backend_->addItem(tr("Trayectorias v2"), QStringLiteral("rws_trajectory"));
    backend_row->addWidget(task_backend_);
    use_task_backend_ = new QPushButton(tr("Seleccionar backend guardado"));
    backend_row->addWidget(use_task_backend_);
    advanced->addLayout(backend_row);
    connect(use_task_backend_, &QPushButton::clicked, this, [this]() {
      if (select_backend_ && !task_.value("backend").toString().isEmpty()) {
        select_backend_(task_.value("backend").toString());
      }
    });
    connect(task_backend_, qOverload<int>(&QComboBox::activated), this, [this](int) {
      const auto backend = task_backend_->currentData().toString();
      if (backend.isEmpty()) {
        task_.remove("backend");
      } else {
        task_.insert("backend", backend);
        if (select_backend_) {
          select_backend_(backend);
        }
      }
      invalidate();
      refresh_configuration();
      auto_plan_ = true;
    });
    connect(custom_name_, &QLineEdit::editingFinished, this, [this]() {
      if (task_kind() == "custom" && !custom_name_->text().trimmed().isEmpty()) {
        task_.insert("name", custom_name_->text().trimmed());
        refresh_configuration();
      }
    });
    connect(new_physical, &QPushButton::clicked, this, [this]() {
      if (!has_pending_edit() && !working()) {
        loaded_example_ = -1;
        load_task({{"kind", "custom"}, {"catalog_id", new_catalog_id()},
          {"name", tr("Tarea personalizada")}, {"cycles", 1}, {"order", QJsonArray{"P1"}},
          {"waypoints", QJsonObject{{"P1", QJsonObject{}}}}});
      }
    });
    connect(open, &QPushButton::clicked, this, [this]() {open_task();});
    connect(save, &QPushButton::clicked, this, [this]() {
      if (!has_pending_edit()) {
        save_json(tr("Guardar tarea"), "tarea.json", task_);
      }
    });
  }

  void build_waypoints(QVBoxLayout * layout)
  {
    auto * section = new QGroupBox(tr("Configurar waypoints"));
    editor_ = section;
    auto * section_layout = new QVBoxLayout(section);
    auto * contents = new QWidget;
    auto * box = new QVBoxLayout(contents);
    box->setContentsMargins(0, 0, 0, 0);
    section_layout->addWidget(contents);
    auto * setup = new QGridLayout;
    waypoint_ = new QComboBox;
    waypoint_->setAccessibleName(tr("Waypoint"));
    waypoint_->setObjectName("taskWaypoint");
    waypoint_->addItems(waypoint_names({}));
    type_ = new QComboBox;
    type_->setAccessibleName(tr("Tipo de waypoint"));
    type_->addItems({tr("Articulaciones"), tr("Pose cartesiana")});
    setup->addWidget(new QLabel(tr("Destino")), 0, 0);
    setup->addWidget(waypoint_, 0, 1);
    setup->addWidget(type_, 0, 2);
    waypoint_duration_ = duration_field("waypointDuration");
    auto * waypoint_time_label = new QLabel(tr("Duración de llegada"));
    waypoint_time_label->setBuddy(waypoint_duration_);
    setup->addWidget(waypoint_time_label, 1, 0);
    setup->addWidget(waypoint_duration_, 1, 1, 1, 2);
    waypoint_profile_ = profile_field("waypointReferenceProfile", tr("Perfil de tarea"));
    auto * waypoint_profile_label = new QLabel(tr("Perfil de llegada"));
    waypoint_profile_label->setBuddy(waypoint_profile_);
    setup->addWidget(waypoint_profile_label, 2, 0);
    setup->addWidget(waypoint_profile_, 2, 1, 1, 2);
    add_destination_ = new QPushButton(tr("Añadir destino"));
    remove_destination_ = new QPushButton(tr("Quitar destino"));
    move_destination_up_ = new QPushButton(tr("Subir destino"));
    move_destination_down_ = new QPushButton(tr("Bajar destino"));
    add_destination_->setObjectName("addTaskDestination");
    remove_destination_->setObjectName("removeTaskDestination");
    move_destination_up_->setObjectName("moveTaskDestinationUp");
    move_destination_down_->setObjectName("moveTaskDestinationDown");
    setup->addWidget(add_destination_, 3, 1);
    setup->addWidget(remove_destination_, 3, 2);
    setup->addWidget(move_destination_up_, 4, 1);
    setup->addWidget(move_destination_down_, 4, 2);
    connect(add_destination_, &QPushButton::clicked, this, [this]() {edit_destinations(true);});
    connect(remove_destination_, &QPushButton::clicked, this, [this]() {edit_destinations(false);});
    connect(move_destination_up_, &QPushButton::clicked, this, [this]() {move_destination(-1);});
    connect(move_destination_down_, &QPushButton::clicked, this, [this]() {move_destination(1);});
    connect(waypoint_duration_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [this](double) {edited();});
    connect(waypoint_profile_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {edited();});
    box->addLayout(setup);

    joint_fields_ = new QWidget;
    auto * joint_layout = new QGridLayout(joint_fields_);
    joint_layout->setContentsMargins(0, 0, 0, 0);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto name = QStringLiteral("J%1").arg(i + 1);
      auto * edit = new QDoubleSpinBox;
      edit->setAccessibleName(name);
      edit->setObjectName(name);
      edit->setDecimals(8);
      edit->setRange(imt342_irb120_rviz::lower[i], upper[i]);
      edit->setSuffix(tr(" rad"));
      edit->setKeyboardTracking(false);
      joints_[i] = edit;
      const int row = static_cast<int>(i / 2);
      const int col = static_cast<int>(i % 2) * 2;
      joint_layout->addWidget(new QLabel(name), row, col);
      joint_layout->addWidget(edit, row, col + 1);
      connect(edit, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, i](double) {
        joint_changed_[i] = true;
        edited();
      });
    }
    box->addWidget(joint_fields_);
    pose_fields_ = new QWidget;
    auto * pose_layout = new QVBoxLayout(pose_fields_);
    pose_layout->setContentsMargins(0, 0, 0, 0);
    point_ = new QComboBox;
    point_->addItem(tr("TCP activo"), "tcp");
    point_->addItem(tr("Brida"), "flange");
    point_->setCurrentIndex(1);
    pose_layout->addWidget(note(tr("Matriz base_link → punto · traslación en metros")));
    pose_layout->addWidget(point_);
    matrix_ = new PoseGrid;
    matrix_->onEdit([this]() {selected_branch_.clear(); edited();});
    pose_layout->addWidget(matrix_);
    auto * paste = new QPushButton(tr("Pegar matriz"));
    pose_layout->addWidget(paste);
    connect(paste, &QPushButton::clicked, this, [this]() {
      if (matrix_->paste(QApplication::clipboard()->text())) {
        selected_branch_.clear();
        edited();
      } else {
        validation_->setText(tr("El portapapeles debe contener una matriz de 4 × 4."));
      }
    });
    branch_ = note({});
    pose_layout->addWidget(branch_);
    box->addWidget(pose_fields_);
    auto * controls = new QGridLayout;
    capture_ = new QPushButton(tr("Copiar postura medida"));
    capture_->setObjectName("captureTaskWaypoint");
    copy_target_ = new QPushButton(tr("Copiar objetivo de FK/IK"));
    copy_target_->setObjectName("copyFkIkTaskWaypoint");
    copy_start_ = new QPushButton(tr("Copiar INICIO a OBJETIVO"));
    copy_start_->setObjectName("copyTaskStart");
    validation_ = note({});
    auto * validate = new QPushButton(tr("Validar waypoint"));
    auto * save = new QPushButton(tr("Guardar waypoint"));
    save->setObjectName("saveTaskWaypoint");
    auto * revert = new QPushButton(tr("Descartar cambios del waypoint"));
    controls->addWidget(capture_, 0, 0, 1, 2);
    controls->addWidget(validate, 1, 0);
    controls->addWidget(save, 1, 1);
    controls->addWidget(revert, 2, 0, 1, 2);
    controls->addWidget(copy_start_, 3, 0, 1, 2);
    controls->addWidget(copy_target_, 4, 0, 1, 2);
    connect(copy_target_, &QPushButton::clicked, this, [this]() {
      if (!target_) {
        return;
      }
      const auto current = target_();
      if (joint_input_valid(current)) {
        type_->setCurrentIndex(0);
        set_joints(current);
        edited();
      }
    });
    box->addLayout(controls);
    connect(revert, &QPushButton::clicked, this, [this]() {read_waypoint();});
    box->addWidget(validation_);
    layout->addWidget(section);
    connect(type_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
      display_waypoint_fields();
      edited();
    });
    connect(point_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {selected_branch_.clear(); edited();});
    connect(waypoint_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
      if (has_pending_edit()) {
        const QSignalBlocker block(waypoint_);
        waypoint_->setCurrentIndex(waypoint_index_);
        return;
      }
      waypoint_index_ = index;
      read_waypoint();
    });
    connect(capture_, &QPushButton::clicked, this, [this]() {
      if (mode_ == "offline" || !fresh_ || !joint_input_valid(actual_)) {
        return;
      }
      type_->setCurrentIndex(0);
      set_joints(actual_);
      edited();
    });
    connect(copy_start_, &QPushButton::clicked, this, [this]() {
      if (copy_start_->isEnabled()) {
        set_joints(initial_angles());
        edited();
      }
    });
    connect(validate, &QPushButton::clicked, this, [this]() {
      const auto waypoint = validated_waypoint();
      if (!waypoint.isEmpty()) {
        validation_->setText(waypoint.contains("matrix") ?
          tr("Transformación válida. El plan comprobará alcance y rama Pieper.") :
          tr("Configuración articular dentro de los límites del modelo."));
      }
    });
    connect(save, &QPushButton::clicked, this, [this]() {
      const auto waypoint = validated_waypoint();
      if (!waypoint.isEmpty()) {
        store_waypoint(waypoint_->currentText(), waypoint);
        validation_->setText(tr("Waypoint guardado."));
      }
    });
  }

  void build_execution(QVBoxLayout * layout, QVBoxLayout * advanced)
  {
    plan_ = new QPushButton(tr("Planificar / calcular IK"));
    preview_ = new QPushButton(tr("Previsualizar"));
    simulate_ = new QPushButton(tr("Ejecutar simulación"));
    stop_ = new QPushButton(tr("Detener"));
    plan_->setObjectName("planTask");
    preview_->setObjectName("previewTask");
    simulate_->setObjectName("simulateTask");
    stop_->setObjectName("cancelTask");
    panel_goals_ = note({});
    panel_goals_->setObjectName("catalogTaskGoals");
    layout->addWidget(panel_goals_);
    auto * actions = new QHBoxLayout;
    actions->addWidget(preview_);
    actions->addWidget(simulate_);
    layout->addLayout(actions);
    layout->addWidget(stop_);
    progress_ = note(tr("Seleccione una tarea Offline"));
    progress_->setObjectName("taskProgress");
    layout->addWidget(progress_);
    advanced->addWidget(plan_);
    goals_ = note({});
    goals_->setObjectName("taskGoals");
    advanced->addWidget(goals_);

    inspect_ = new QPushButton(tr("Registrar inspección física…"));
    inspect_->setObjectName("inspectPhysicalTask");
    advanced->addWidget(inspect_);
    connect(inspect_, &QPushButton::clicked, this, [this]() {inspect_task();});
    authorize_ = new QPushButton(tr("Autorizar lote…"));
    authorize_->setObjectName("authorizePhysicalTask");
    authorize_->setToolTip(tr("Requiere tarea inspeccionada para esta configuración, "
      "previsualización completa, Control físico y retroalimentación RWS reciente."));
    execute_ = new QPushButton(tr("Ejecutar lote físico"));
    execute_->setObjectName("executePhysicalTask");
    layout->addWidget(authorize_);
    layout->addWidget(execute_);
    connect(authorize_, &QPushButton::clicked, this, [this]() {authorize_task();});
    connect(execute_, &QPushButton::clicked, this, [this]() {execute_task();});

    candidates_ = new QComboBox;
    candidates_->setAccessibleName(tr("Ramas para waypoint cartesiano"));
    choose_ = new QPushButton(tr("Guardar rama elegida"));
    candidate_label_ = note({});
    advanced->addWidget(candidate_label_);
    advanced->addWidget(candidates_);
    advanced->addWidget(choose_);
    legs_ = new QTableWidget(0, 3);
    legs_->setObjectName("taskLegResults");
    legs_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    legs_->setHorizontalHeaderLabels({tr("Tramo"), tr("Referencia"), tr("Resultado")});
    legs_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    legs_->verticalHeader()->hide();
    legs_->setMaximumHeight(160);
    legs_->setVisible(false);
    advanced->addWidget(legs_);

    advanced_ = new QGroupBox(tr("JSON avanzado"));
    advanced_->setCheckable(true);
    advanced_->setChecked(false);
    auto * advanced_layout = new QVBoxLayout(advanced_);
    payload_ = new QPlainTextEdit;
    payload_->setReadOnly(true);
    payload_->setVisible(false);
    payload_->setMaximumHeight(175);
    advanced_layout->addWidget(payload_);
    advanced->addWidget(advanced_);
    connect(advanced_, &QGroupBox::toggled, this, [this](bool show) {
      payload_->setVisible(show);
      if (show) {
        update_payload();
      }
    });
    connect(plan_, &QPushButton::clicked, this, [this]() {request_plan();});
    connect(preview_, &QPushButton::clicked, this, [this]() {
      if (ready_to_preview()) {
        send_plan_action("preview");
      } else if (can_plan() && !queued_preview_) {
        if (!waiting_for_plan()) {
          request_plan();
        }
        queued_preview_ = true;
        refresh_buttons();
      }
    });
    connect(simulate_, &QPushButton::clicked, this, [this]() {send_plan_action("simulate");});
    connect(stop_, &QPushButton::clicked, this, [this]() {
      if (waiting_for_plan() || queued_preview_) {
        auto_plan_ = false;
        invalidate();
        progress_->setText(tr("Planificación cancelada"));
      } else {
        reset_authorization();
        send_plan_action("cancel");
      }
    });
    connect(candidates_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {refresh_buttons();});
    connect(choose_, &QPushButton::clicked, this, [this]() {
      if (choose_->isEnabled() && !dirty_) {
        const auto name = packet_.value("waypoint").toString();
        auto waypoint = task_.value("waypoints").toObject().value(name).toObject();
        if (waypoint.contains("matrix")) {
          waypoint.insert("branch", candidates_->currentData().toString());
          store_waypoint(name, waypoint);
          if (waypoint_->currentText() == name) {
            read_waypoint();
          } else {
            waypoint_->setCurrentText(name);
          }
          progress_->setText(tr("Rama de %1 guardada.").arg(name));
        }
      }
    });
    refresh_candidates();
    refresh_buttons();
  }

  static QDoubleSpinBox * duration_field(const char * name)
  {
    auto * field = new QDoubleSpinBox;
    field->setObjectName(name);
    field->setRange(0.0, 30.0);
    field->setDecimals(2);
    field->setSingleStep(0.1);
    field->setSuffix(tr(" s"));
    field->setSpecialValueText(tr("Automática"));
    field->setKeyboardTracking(false);
    field->setToolTip(tr("Duración de referencia. RAPID v2 solicita este tiempo a ABB."));
    return field;
  }

  static bool valid_duration(const QJsonObject & task)
  {
    const auto duration = task.value("duration_s");
    return !task.contains("duration_s") ||
      (duration.isDouble() && duration.toDouble() >= 0.1 && duration.toDouble() <= 30.0);
  }

  static void set_duration(QJsonObject & task, const QDoubleSpinBox * field)
  {
    if (field->value() > 0.0) {
      task.insert("duration_s", field->value());
    } else {
      task.remove("duration_s");
    }
  }

  static QComboBox * profile_field(const char * name, const QString & fallback)
  {
    auto * field = new QComboBox;
    field->setObjectName(name);
    field->addItem(fallback, QString());
    field->addItem(tr("LSPB"), QStringLiteral("lspb"));
    field->addItem(tr("Quíntica"), QStringLiteral("quintic"));
    return field;
  }

  static bool valid_profile(const QJsonObject & fields)
  {
    const auto profile = fields.value("profile").toString();
    return !fields.contains("profile") || profile == "lspb" || profile == "quintic";
  }

  static void store_profile(QJsonObject & fields, const QComboBox * selector)
  {
    const auto profile = selector->currentData().toString();
    if (profile.isEmpty()) {
      fields.remove("profile");
    } else {
      fields.insert("profile", profile);
    }
  }

  bool task_backend_selected() const
  {
    const auto backend = analysis_.value("backend").toString();
    return backend == "rws" || backend == "rws_trajectory";
  }

  bool current_task_feedback() const
  {
    const auto backend = analysis_.value("backend").toString();
    const auto source = analysis_.value("source").toString();
    return task_backend_selected() &&
      source == backend;
  }

  static double offset_span_deg(int axis)
  {
    return (upper[axis] - imt342_irb120_rviz::lower[axis]) * deg_per_rad;
  }

  void refresh_live_limits()
  {
    if (!live_joint()) {
      return;
    }
    const auto axis = live_axis_->currentData().toInt() - 1;
    if (axis < 0 || axis >= 6 || !joint_input_valid(actual_) || (!fresh_ && mode_ != "offline")) {
      live_offset_->setToolTip(tr("El destino debe quedar dentro de los límites articulares."));
      return;
    }
    const auto origin = actual_[axis].toDouble();
    live_offset_->setToolTip(tr("Desplazamiento posible desde la postura actual de J%1: %2 a %3 °.")
      .arg(axis + 1).arg((imt342_irb120_rviz::lower[axis] - origin) * deg_per_rad, 0, 'f', 2)
      .arg((upper[axis] - origin) * deg_per_rad, 0, 'f', 2));
  }

  QString live_joint_block() const
  {
    if (!joint_input_valid(actual_)) {
      return tr("Esperando una postura de origen válida.");
    }
    if (!valid_duration(task_)) {
      return tr("La duración por tramo debe ser automática o estar entre 0,10 y 30 s.");
    }
    const auto offset = task_.value("offset_deg").toDouble();
    if (offset == 0.0) {
      return tr("Elige un desplazamiento articular distinto de cero.");
    }
    const auto axis = task_.value("axis").toInt() - 1;
    if (axis < 0 || axis >= 6) {
      return tr("Selecciona J1–J6.");
    }
    const auto goal = actual_[axis].toDouble() + offset / deg_per_rad;
    if (goal < imt342_irb120_rviz::lower[axis] || goal > upper[axis]) {
      return tr("El objetivo supera el límite de J%1 para la postura actual.").arg(axis + 1);
    }
    return {};
  }

  QJsonObject read_physical_context() const
  {
    const auto context = analysis_.value("physical_task_context").toObject();
    return task_backend_selected() &&
      !context.value("controller_config").toObject().isEmpty() ? context : QJsonObject{};
  }

  bool physical_task_complete() const
  {
    if (task_.value("offline_example").toBool() || !file_task_valid(task_)) {
      return false;
    }
    const auto waypoints = task_.value("waypoints").toObject();
    for (const auto & name : waypoint_names(task_kind(), task_)) {
      const auto waypoint = waypoints.value(name).toObject();
      if (waypoint.isEmpty() || (waypoint.contains("matrix") && waypoint.value("branch").toString().isEmpty())) {
        return false;
      }
    }
    if (task_kind() == "joint_test") {
      const auto start = initial_angles();
      const auto goal = waypoints.value("OBJETIVO").toObject().value("q_rad").toArray();
      int changed = 0;
      for (int axis = 0; axis < 6; ++axis) {
        changed += start[axis] != goal[axis];
      }
      return changed == 1;
    }
    return true;
  }

  void inspect_task()
  {
    if (!inspect_->isEnabled()) {
      return;
    }
    const auto context = current_context_;
    const auto inspected_task = task_;
    const auto destinations = waypoint_names(task_kind(), task_).join(tr(", "));
    const auto prompt = tr("¿Contrastaste los destinos (%1) y su recorrido con la celda real "
      "y verificaste la herramienta y los marcos para la configuración actual?").arg(destinations);
    if (QMessageBox::question(this, tr("Inspección de la tarea"),
        prompt,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes ||
      !inspect_->isEnabled() || current_context_ != context || task_ != inspected_task)
    {
      return;
    }
    task_.insert("physical_reviewed", true);
    task_.insert("physical_context", context);
    if (!task_kind().isEmpty()) {
      task_.insert("physical_review_waypoints", task_.value("waypoints"));
    }
    if (task_kind() == "custom" || task_.contains("order")) {
      task_.insert("physical_review_order", task_.value("order"));
    }
    invalidate();
    refresh_configuration();
    request_plan();
  }

  void clear_physical_review()
  {
    task_.remove("physical_reviewed");
    task_.remove("physical_context");
    task_.remove("physical_review_waypoints");
    task_.remove("physical_review_order");
  }

  bool physical_ready() const
  {
    const bool saved_review = task_.value("physical_reviewed").toBool() &&
      (task_kind().isEmpty() || task_.value("physical_review_waypoints") == task_.value("waypoints")) &&
      (!task_.contains("order") || task_.value("physical_review_order") == task_.value("order")) &&
      task_.value("physical_context").toObject() == current_context_;
    const bool repeated = task_kind() == "repeat" ||
      (task_kind() == "custom" && task_.value("cycles").toInt() > 1);
    const bool route_available = task_kind() != "cell_perimeter" || perimeter_block().isEmpty();
    return route_available && current_plan() && mode_ == "control" && fresh_ &&
      current_task_feedback() && !analysis_.value("physical_order_pending").toBool() &&
      (live_route() || saved_review) && !current_context_.isEmpty() &&
      (!repeated || packet_.value("repeat_validated").toBool()) &&
      packet_.value("physical_context").toObject() == current_context_ &&
      packet_.value("physical_eligible").toBool() &&
      packet_.value("state").toString() == "planned" && packet_.value("previewed").toBool() &&
      !dirty_ && !working();
  }

  void authorize_task()
  {
    if (!authorize_->isEnabled() || !stamp_) {
      return;
    }
    const auto plan = packet_.value("plan_id");
    const bool live = live_route();
    const auto confirmation = cell_route() ?
      tr("¿Confirmas que comprobaste físicamente la posición, dimensiones y orientación de los muros, "
        "la herramienta montada y todos los destinos y retorno previsualizados? "
        "El movimiento empieza con Ejecutar lote físico.") : live ?
      tr("¿Confirmas que revisaste la postura medida, todos los destinos y el retorno "
        "previsualizados, y que inspeccionaste el recorrido, la herramienta y los marcos "
        "de la celda real? El movimiento empieza con Ejecutar lote físico.") :
      tr("¿Confirmas %1 ciclo(s) del plan previsualizado en esta sesión? "
        "Autorizar prepara una ejecución. El movimiento empieza con Ejecutar lote físico.")
      .arg(task_.value("cycles").toInt());
    if (QMessageBox::question(this, tr("Autorizar lote de juntas"), confirmation,
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No) != QMessageBox::Yes || !authorize_->isEnabled() ||
      plan != packet_.value("plan_id"))
    {
      return;
    }
    const auto stamp = stamp_();
    if (stamp.isEmpty()) {
      progress_->setText(tr("El reloj ROS de autorización no está disponible."));
      return;
    }
    authorize_pending_ = true;
    authorize_stamp_ = stamp;
    previous_token_ = packet_.value("authorization_id").toString();
    QJsonObject request{{"action", "authorize"}, {"plan_id", plan}, {"stamp_ns", stamp}};
    if (live) {
      request.insert("reviewed_live_route", true);
    }
    if (cell_route()) {
      request.insert("reviewed_cell_geometry", true);
    }
    send_(request);
    refresh_buttons();
  }

  void execute_task()
  {
    if (!execute_->isEnabled()) {
      return;
    }
    const auto token = approved_token_;
    approved_token_.clear();
    used_token_ = token;
    execute_pending_ = true;
    send_({{"action", "execute"}, {"plan_id", packet_.value("plan_id")}, {"authorization_id", token}});
    refresh_buttons();
  }

  void reset_authorization()
  {
    authorize_pending_ = false;
    execute_pending_ = false;
    authorize_stamp_.clear();
    previous_token_.clear();
    approved_token_.clear();
  }

  void refresh_authorization()
  {
    const auto token = packet_.value("authorization_id").toString();
    if (!current_plan() || packet_.value("state").toString() != "planned" ||
      !packet_.value("physical_eligible").toBool())
    {
      reset_authorization();
    } else if (authorize_pending_ && !token.isEmpty() && token != previous_token_ && token != used_token_ &&
      packet_.value("authorization_stamp_ns").toString() == authorize_stamp_)
    {
      authorize_pending_ = false;
      approved_token_ = token;
    } else if (!packet_.value("error").toString().isEmpty()) {
      reset_authorization();
    } else if (token != approved_token_) {
      approved_token_.clear();
      if (token != used_token_) {
        execute_pending_ = false;
      }
    }
  }

  bool can_plan() const
  {
    if (analysis_.value("physical_order_pending").toBool() && mode_ != "offline") {
      return false;
    }
    if (automatic_catalog()) {
      return !working() && !dirty_ && catalog_plan_block().isEmpty();
    }
    const bool online_preview = mode_ == "observe" ||
      (mode_ == "control" && !task_.value("offline_example").toBool());
    return connected_ && !working() && !dirty_ &&
      (task_kind().isEmpty() || points_configured()) &&
      (mode_ == "offline" || (online_preview && fresh_ && joint_input_valid(actual_)));
  }

  QString catalog_plan_block() const
  {
    if (!automatic_catalog()) {
      return {};
    }
    if (task_kind() == "cell_perimeter") {
      const auto reason = perimeter_block();
      if (!reason.isEmpty()) {
        return reason;
      }
    }
    if (!live_route() && !points_configured()) {
      if (task_kind() == "repeat") {
        return tr("Configura y guarda primero el ciclo A–B–C–HOME en el catálogo.");
      }
      return task_kind() == "custom" ? tr("Configura los destinos pendientes en Editar tarea.") :
        tr("Falta configurar A, B, C o HOME. Usa Editar tarea para la configuración inicial.");
    }
    if (!connected_) {
      return tr("Esperando comunicación con el coordinador.");
    }
    if (analysis_.value("physical_order_pending").toBool() && mode_ != "offline") {
      return tr("Hay una orden física en curso. Podrás planificar al terminar.");
    }
    const auto preferred = task_.value("backend").toString();
    if (!preferred.isEmpty() && preferred != analysis_.value("backend").toString()) {
      return tr("Selecciona el backend guardado en Editar tarea.");
    }
    if (mode_ == "offline") {
      return live_joint() ? live_joint_block() : QString();
    }
    if (!task_backend_selected()) {
      return tr("Selecciona Normal o Trayectorias v2.");
    }
    if (!fresh_ || !current_task_feedback()) {
      return tr("Esperando una lectura reciente de J1–J6 del controlador seleccionado.");
    }
    if (!joint_input_valid(actual_)) {
      return tr("La postura medida no es válida para los límites del modelo.");
    }
    if (current_context_.isEmpty()) {
      return tr("Falta la configuración física del IRC5.");
    }
    return live_joint() ? live_joint_block() : QString();
  }

  bool planning_reply_received() const
  {
    return !request_.isEmpty() && request_mode_ == mode_ &&
      packet_.value("request_id").toString() == request_;
  }

  bool waiting_for_plan() const
  {
    return !request_.isEmpty() && request_mode_ == mode_ && !planning_reply_received();
  }

  void request_plan()
  {
    if (!can_plan() || waiting_for_plan()) {
      return;
    }
    auto_plan_ = false;
    invalidate();
    request_ = QUuid::createUuid().toString(QUuid::WithoutBraces);
    request_mode_ = mode_;
    progress_->setText(tr("Planificando…"));
    send_({{"action", "plan"}, {"request_id", request_}, {"task", task_}});
    refresh_buttons();
  }

  void send_plan_action(const QString & action)
  {
    if (!current_plan()) {
      return;
    }
    if (action == "simulate" && mode_ != "offline") {
      return;
    }
    if (action != "cancel") {
      dispatch_pending_ = true;
    }
    send_({{"action", action}, {"plan_id", packet_.value("plan_id")}});
    refresh_buttons();
  }

  bool current_plan() const
  {
    return connected_ && planning_reply_received() &&
      !packet_.value("plan_id").toString().isEmpty();
  }

  bool current_catalog_plan() const
  {
    return current_plan() && packet_.value("state").toString() == "planned" &&
      (mode_ == "offline" || packet_.value("physical_context").toObject() == current_context_);
  }

  bool ready_to_preview() const
  {
    return automatic_catalog() ? current_catalog_plan() :
      current_plan() && packet_.value("state").toString() == "planned";
  }

  bool working() const
  {
    return current_plan() && (dispatch_pending_ || preview_running_ || packet_.value("active").toBool() ||
           packet_.value("state").toString() == "simulating" ||
           packet_.value("state").toString() == "previewing");
  }

  void refresh_buttons()
  {
    const bool current = current_plan();
    const bool active = working();
    const bool locked = active || authorize_pending_ || execute_pending_ || !approved_token_.isEmpty();
    library_->setEnabled(!locked);
    editor_->setVisible(!live_route());
    editor_->setEnabled(!locked && task_kind() != "repeat");
    live_joint_controls_->setVisible(live_joint());
    physical_presets_->setEnabled(!locked);
    refresh_perimeter_option();
    const bool custom = task_kind() == "custom";
    const auto destinations = task_.value("order").toArray().size();
    const bool cycle = task_kind() == "cycle";
    const int reorderable = cycle ? 3 : destinations;
    task_backend_->setVisible(custom);
    task_backend_->setEnabled(!locked);
    use_task_backend_->setVisible(custom && !task_.value("backend").toString().isEmpty() &&
      task_.value("backend") != analysis_.value("backend"));
    use_task_backend_->setEnabled(!locked && static_cast<bool>(select_backend_));
    custom_name_->setVisible(custom);
    custom_name_->setEnabled(!locked);
    delete_catalog_->setVisible(custom);
    delete_catalog_->setEnabled(!locked && !dirty_ && custom && valid_catalog_id(task_.value("catalog_id").toString()) &&
      QFileInfo::exists(catalog_path(catalog_key())));
    add_destination_->setVisible(custom);
    remove_destination_->setVisible(custom);
    move_destination_up_->setVisible(custom || cycle);
    move_destination_down_->setVisible(custom || cycle);
    add_destination_->setEnabled(!locked && !dirty_ && destinations < 32);
    remove_destination_->setEnabled(!locked && !dirty_ && destinations > 1);
    move_destination_up_->setEnabled(!locked && !dirty_ && waypoint_index_ > 0 && waypoint_index_ < reorderable);
    move_destination_down_->setEnabled(!locked && !dirty_ && waypoint_index_ < reorderable - 1);
    task_profile_->setEnabled(!locked && task_kind() != "repeat");
    copy_target_->setEnabled(!locked && static_cast<bool>(target_));
    save_catalog_->setVisible(!task_kind().isEmpty() && !live_demo() && !cell_route());
    save_catalog_->setEnabled(!locked && !dirty_ && (!live_joint() || file_task_valid(task_)));
    if (edit_) {
      edit_->setVisible(!cell_route());
      edit_->setEnabled(!locked && !live_demo() && !cell_route());
    }
    const QString phase = packet_.value("state").toString();
    plan_->setEnabled(can_plan() && !locked && !waiting_for_plan());
    const auto rejected_request = analysis_.value("task_plan_reply").toObject().value("request_id");
    const bool invalid_catalog = planning_reply_received() && phase == "invalid" &&
      rejected_request.toString() != request_;
    preview_->setEnabled(can_plan() && !queued_preview_ && !locked &&
      (!automatic_catalog() || (!waiting_for_plan() && !invalid_catalog)));
    simulate_->setEnabled(current && phase == "planned" && !active && !dirty_ &&
      mode_ == "offline" && packet_.value("previewed").toBool());
    stop_->setEnabled(waiting_for_plan() || queued_preview_ ||
      (current && (locked || packet_.value("stop_available").toBool())));
    inspect_->setVisible(!live_route());
    inspect_->setEnabled(!locked && !dirty_ && fresh_ && !live_route() && task_kind() != "repeat" &&
      mode_ != "offline" && !current_context_.isEmpty() &&
      current_task_feedback() && physical_task_complete());
    const bool physical_catalog = mode_ == "control" && task_backend_selected() &&
      !task_.value("offline_example").toBool();
    authorize_->setVisible(physical_catalog);
    execute_->setVisible(physical_catalog);
    authorize_->setEnabled(stamp_ && physical_ready() && !locked);
    execute_->setEnabled(physical_ready() && !approved_token_.isEmpty() &&
      approved_token_ == packet_.value("authorization_id").toString());
    capture_->setEnabled(!active && mode_ != "offline" && fresh_ && joint_input_valid(actual_) &&
      (task_kind().isEmpty() || current_task_feedback()));
    copy_start_->setVisible(task_kind() == "joint_test");
    copy_start_->setEnabled(!locked && !dirty_ && waypoint_->currentText() == "OBJETIVO" &&
      joint_input_valid(initial_angles()));
    choose_->setEnabled(!active && !dirty_ && task_kind() != "repeat" && !request_.isEmpty() &&
      packet_.value("request_id").toString() == request_ && candidates_->currentIndex() >= 0);
    const auto reason = catalog_plan_block();
    plan_->setToolTip(reason);
    preview_->setToolTip(reason);
    auto physical_block = packet_.value("physical_block_reason").toString();
    if (physical_block.isEmpty() && cell_route()) {
      physical_block = packet_.value("physical_preset_block_reason").toString();
    }
    authorize_->setToolTip(!physical_block.isEmpty() ? physical_block :
      !current || !packet_.value("previewed").toBool() ?
      tr("Previsualiza el plan de tarea vigente antes de autorizarlo.") : QString());
    execute_->setToolTip(!physical_block.isEmpty() ? physical_block :
      approved_token_.isEmpty() ? tr("Autoriza este lote antes de ejecutarlo.") : QString());
    if (reason != catalog_block_) {
      catalog_block_ = reason;
      refresh_configuration();
    }
  }

  void refresh_candidates()
  {
    const QSignalBlocker block(candidates_);
    candidates_->clear();
    const auto name = packet_.value("waypoint").toString();
    if (!request_.isEmpty() && packet_.value("request_id").toString() == request_) {
      for (const auto & candidate : packet_.value("candidates").toArray()) {
        const auto id = candidate.toObject().value("branch").toString();
        if (!id.isEmpty()) {
          candidates_->addItem(id, id);
        }
      }
    }
    candidates_->setCurrentIndex(-1);
    const bool available = candidates_->count() > 0;
    candidate_label_->setText(tr("Ramas Pieper de %1").arg(name));
    candidate_label_->setVisible(available);
    candidates_->setVisible(available);
    choose_->setVisible(available);
  }

  void refresh_progress()
  {
    if (!planning_reply_received()) {
      return;
    }
    dispatch_pending_ = false;
    refresh_buttons();
    const auto phase = packet_.value("state").toString();
    QString state = tr("Revise la configuración");
    if (phase == "planned") {
      state = preview_running_ ? tr("Previsualización en curso") :
        packet_.value("previewed").toBool() ? tr("Previsualización completa") : tr("Plan disponible");
    } else if (phase == "previewing") {
      state = tr("Previsualización en curso");
    } else if (phase == "simulating") {
      state = tr("Simulación en curso");
    } else if (phase == "simulated" || phase == "completed") {
      state = tr("Simulación terminada");
    } else if (phase == "cancelled") {
      state = tr("Tarea cancelada");
    } else if (phase == "executing") {
      state = tr("Lote físico en curso");
    } else if (phase == "verified") {
      state = tr("Lote físico verificado");
    } else if (phase == "unknown") {
      state = tr("Resultado físico incierto");
    } else if (phase == "failed") {
      state = tr("Lote físico interrumpido");
    } else if (phase == "stale") {
      state = tr("El plan requiere un nuevo cálculo desde el origen vigente");
    } else if (phase == "invalid") {
      state = tr("No se pudo planificar la tarea");
    }
    if (!approved_token_.isEmpty()) {
      const auto ttl = packet_.value("authorization_ttl_s");
      state = ttl.isDouble() ? tr("Lote autorizado · %1 s disponibles").arg(std::ceil(ttl.toDouble())) :
        tr("Lote autorizado · listo para ejecutar");
    }
    QStringList lines{state};
    const auto duration = packet_.value("planned_duration_s").toDouble(
      packet_.value("planned_lspb_s").toDouble(-1));
    if (duration >= 0) {
      const auto id = packet_.value("reference_profile").toString();
      const auto profile = id == "quintic" ? tr("quíntica") : id == "mixed" ? tr("mixta") : tr("LSPB");
      lines << tr("Duración de referencia %1 · %2 s").arg(profile)
        .arg(duration, 0, 'f', 2);
    }
    if (packet_.value("cycle").isDouble()) {
      lines << tr("Ciclo %1/%2 · %3")
        .arg(packet_.value("cycle").toInt()).arg(packet_.value("cycles").toInt())
        .arg(packet_.value("waypoint").toString());
    }
    const auto completed = packet_.value("verified_waypoints");
    if (completed.isDouble()) {
      lines << tr("Puntos completados · %1").arg(completed.toInt());
    }
    if (packet_.value("elapsed_s").isDouble()) {
      lines << tr("Tiempo %1 · %2 s")
        .arg(packet_.value("source").toString() == "virtual" ? tr("simulado") : tr("observado"))
        .arg(packet_.value("elapsed_s").toDouble(), 0, 'f', 2);
    }
    for (const auto & key : {"reason", "error", "cycle_validation_warning"}) {
      const auto error = packet_.value(key).toString();
      if (!error.isEmpty()) {
        lines << error;
      }
    }
    auto blocked = packet_.value("physical_block_reason").toString();
    if (blocked.isEmpty() && cell_route()) {
      blocked = packet_.value("physical_preset_block_reason").toString();
    }
    if (!blocked.isEmpty()) {
      lines << blocked;
    }
    const auto path = packet_.value("cell_path").toObject();
    if (cell_route() && !path.isEmpty()) {
      const auto deviation = path.value("max_tcp_deviation_m");
      const auto clearance_min = path.value("wall_clearance_min_m");
      const auto clearance_max = path.value("wall_clearance_max_m");
      if (deviation.isDouble() && clearance_min.isDouble() && clearance_max.isDouble()) {
        lines << tr("Referencia %4 · separación %1–%2 mm · desviación máx. %3 mm")
          .arg(clearance_min.toDouble() * 1000, 0, 'f', 1)
          .arg(clearance_max.toDouble() * 1000, 0, 'f', 1)
          .arg(deviation.toDouble() * 1000, 0, 'f', 1)
          .arg(path.value("reference_point").toString() == "flange" ? tr("Brida") : tr("TCP"));
      }
      if (!path.value("geometry").toObject().value("verified").toBool()) {
        lines << tr("Geometría nominal: comprueba muros y herramienta al autorizar el lote.");
      }
      if (path.value("variant").toString() == "air_triangle" && deviation.isDouble()) {
        lines << tr("Brida · triángulo de %1 mm por lado · desviación máx. %2 mm")
          .arg(path.value("side_m").toDouble() * 1000, 0, 'f', 0)
          .arg(deviation.toDouble() * 1000, 0, 'f', 2);
        lines << tr("Un destino por vértice · sin paradas a mitad de lado.");
      }
      if (path.value("execution_style").toString() == "continuous_movel") {
        lines << tr("MoveL continuo · sin paradas intermedias en ida/vuelta");
      } else if (path.value("execution_style").toString() == "segmented") {
        lines << tr("Recorrido segmentado · parada por destino");
      }
      const auto commands = path.value("command_segments");
      if (commands.isDouble()) {
        lines << tr("Tramos de mando · %1").arg(commands.toInt());
      }
    }
    const auto active_steps = packet_.value("steps").toArray();
    if (!active_steps.isEmpty() && phase == "executing") {
      const auto arrival = active_steps.last().toObject().value("motion").toObject();
      if (arrival.value("latest_goal_error_rad").isDouble()) {
        lines << tr("Error actual %1° · tolerancia ±%2°")
          .arg(arrival.value("latest_goal_error_rad").toDouble() * deg_per_rad, 0, 'f', 3)
          .arg(arrival.value("arrival_tolerance_rad").toDouble() * deg_per_rad, 0, 'f', 2);
        if (arrival.value("within_arrival_band").toBool() &&
          arrival.value("awaiting_terminal_reply").toBool())
        {
          lines << tr("Dentro de tolerancia · esperando acuse RAPID.");
        }
      }
    }
    progress_->setText(lines.join('\n'));
    const auto steps = packet_.value("steps").toArray();
    const auto checks = steps.isEmpty() ? packet_.value("leg_checks").toArray() : steps;
    if (checks != displayed_legs_) {
      displayed_legs_ = checks;
      legs_->setRowCount(checks.size());
      const auto order = packet_.value("order").toArray();
      const int points_per_cycle = std::max(1, order.size());
      for (int row = 0; row < checks.size(); ++row) {
        const auto check = checks[row].toObject();
        const int index = check.value("index").toInt(row);
        const auto route = check.value("waypoint").toString(
          !order.isEmpty() ? order[index % points_per_cycle].toString() : QString());
        const auto id = tr("%1 · ciclo %2").arg(route)
          .arg(check.value("cycle").toInt(index / points_per_cycle + 1));
        auto result = check.value("result").toString();
        if (result == "already_satisfied") {
          result = tr("Destino ya satisfecho");
        } else if (result == "simulated_complete") {
          result = tr("Simulado");
        } else if (result == "pending") {
          result = tr("Pendiente");
        } else if (result == "simulating") {
          result = tr("En curso");
        }
        legs_->setItem(row, 0, new QTableWidgetItem(id));
        const auto duration = check.value("duration_s").toDouble(check.value("lspb_s").toDouble(-1));
        const auto time = duration >= 0 ? tr("%1 s").arg(duration, 0, 'f', 2) : tr("N/D");
        legs_->setItem(row, 1, new QTableWidgetItem(time));
        const auto motion = check.value("motion").toObject();
        const auto requested = motion.value("requested_duration_s");
        const auto observed = motion.value("submit_to_terminal_reply_s");
        const auto final_error = motion.value("final_error_rad");
        auto * result_item = new QTableWidgetItem(result.isEmpty() ? tr("Planificado") : result);
        QStringList details;
        if (requested.isDouble()) {
          details << tr("Duración solicitada a ABB · %1 s").arg(requested.toDouble(), 0, 'f', 3);
        }
        if (observed.isDouble()) {
          details << tr("Envío a acuse terminal observado · %1 s").arg(observed.toDouble(), 0, 'f', 3);
        }
        if (final_error.isDouble()) {
          details << tr("Error articular final máximo · %1 rad").arg(final_error.toDouble(), 0, 'g', 5);
        }
        result_item->setToolTip(details.join('\n'));
        legs_->setItem(row, 2, result_item);
      }
      legs_->setVisible(!checks.isEmpty());
    }
  }

  void apply_units(bool radians)
  {
    Joints previous{};
    const double old_factor = radians_ ? 1.0 : deg_per_rad;
    radians_ = radians;
    const double factor = radians_ ? 1.0 : deg_per_rad;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      previous[i] = joints_[i]->value() / old_factor;
      const QSignalBlocker block(joints_[i]);
      joints_[i]->setRange(imt342_irb120_rviz::lower[i] * factor, upper[i] * factor);
      joints_[i]->setValue(previous[i] * factor);
      joints_[i]->setSuffix(radians_ ? tr(" rad") : tr(" °"));
    }
  }

  void load_example()
  {
    if (has_pending_edit()) {
      const QSignalBlocker block(preset_);
      preset_->setCurrentIndex(loaded_example_);
      return;
    }
    if (preset_->currentIndex() < 0 || loaded_example_ == preset_->currentIndex()) {
      return;
    }
    loaded_example_ = preset_->currentIndex();
    const auto & preset = task_presets.at(loaded_example_);
    QJsonObject waypoints;
    for (int i = 0; i < 4; ++i) {
      QJsonArray q;
      for (const auto axis : preset.waypoints_rad[i]) {
        q.append(axis);
      }
      waypoints.insert(waypoint_names({}).at(i), QJsonObject{{"q_rad", q}});
    }
    load_task({{"name", QString::fromUtf8(preset.name)}, {"cycles", preset.cycles},
      {"offline_example", true}, {"comparison", preset.comparison}, {"waypoints", waypoints}});
  }

  QJsonObject read_json(const QString & path) const
  {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > 64 * 1024) {
      return {};
    }
    const auto bytes = file.read(64 * 1024 + 1);
    return bytes.size() <= 64 * 1024 ? QJsonDocument::fromJson(bytes).object() : QJsonObject{};
  }

  void load_catalog(int index)
  {
    if (index < 0) {
      return;
    }
    const auto kind = physical_presets_->itemData(index).toString();
    if (has_pending_edit() || (kind == "cell_perimeter" && !perimeter_block().isEmpty())) {
      const QSignalBlocker block(physical_presets_);
      physical_presets_->setCurrentIndex(physical_presets_->findData(catalog_key()));
      return;
    }
    if (kind == catalog_key()) {
      return;
    }
    if (kind.startsWith("custom-")) {
      const auto task = read_json(catalog_path(kind));
      if (file_task_valid(task) && task.value("catalog_id").toString() == kind.mid(7)) {
        loaded_example_ = -1;
        load_task(task);
      }
      return;
    }
    if (kind == "demo_cycle_live" || cell_kind(kind)) {
      loaded_example_ = -1;
      load_task({{"kind", kind}, {"name", physical_presets_->itemText(index)}, {"cycles", 1}});
      return;
    }
    const auto source_kind = kind == "repeat" ? QStringLiteral("cycle") : kind;
    const auto source_path = catalog_path(source_kind);
    auto task = read_json(source_path);
    const bool trusted_shape = task.value("kind").toString() == source_kind &&
      !task.value("offline_example").toBool() && file_task_valid(task);
    if (!trusted_shape) {
      task = kind == "joint_test_live" ? QJsonObject{{"axis", 1}, {"offset_deg", 0.5}} :
        QJsonObject{{"waypoints", empty_waypoints(kind)}};
    }
    int cycles = 1;
    if (kind == "repeat") {
      cycles = read_json(catalog_path(kind)).value("cycles").toInt(2);
      cycles = std::clamp(cycles, 2, 8);
    }
    task.insert("kind", kind);
    task.insert("name", physical_presets_->itemText(index));
    task.insert("cycles", cycles);
    loaded_example_ = -1;
    load_task(task);
    if (!trusted_shape && QFileInfo::exists(source_path)) {
      progress_->setText(tr("La configuración guardada del catálogo es ilegible o no es válida."));
    }
  }

  void save_catalog()
  {
    if (has_pending_edit() || task_kind().isEmpty() || !file_task_valid(task_)) {
      return;
    }
    if (task_kind() == "custom" && !valid_catalog_id(task_.value("catalog_id").toString())) {
      task_.insert("catalog_id", new_catalog_id());
    }
    const auto path = catalog_path(catalog_key());
    const auto record = task_kind() == "repeat" ?
      QJsonObject{{"cycles", task_.value("cycles")}} : task_;
    if (!QDir().mkpath(QFileInfo(path).absolutePath()) || !write_json(path, record)) {
      QMessageBox::warning(this, tr("Guardar en catálogo"), tr("No se pudo guardar la tarea."));
      return;
    }
    refresh_saved_catalog();
    refresh_buttons();
    progress_->setText(tr("Configuración guardada en el catálogo."));
  }

  void delete_catalog()
  {
    if (task_kind() != "custom" || !delete_catalog_->isEnabled()) {
      return;
    }
    const auto key = catalog_key();
    if (QMessageBox::question(this, tr("Eliminar tarea local"),
        tr("¿Eliminar %1 del catálogo?").arg(task_.value("name").toString()),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes ||
      !delete_catalog_->isEnabled() || catalog_key() != key)
    {
      return;
    }
    if (!QFile::remove(catalog_path(key))) {
      progress_->setText(tr("No se pudo eliminar la tarea."));
      return;
    }
    load_example_after_delete();
  }

  void load_example_after_delete()
  {
    const QSignalBlocker blocker(preset_);
    preset_->setCurrentIndex(0);
    load_example();
    refresh_saved_catalog();
    progress_->setText(tr("Tarea eliminada del catálogo local."));
  }

  void edit_destinations(bool add)
  {
    if (task_kind() != "custom" || dirty_ ||
      !(add ? add_destination_->isEnabled() : remove_destination_->isEnabled()))
    {
      return;
    }
    auto task = task_;
    auto order = task.value("order").toArray();
    auto points = task.value("waypoints").toObject();
    QString focus;
    if (add) {
      for (int index = 1; index <= 32; ++index) {
        focus = QStringLiteral("P%1").arg(index);
        if (!points.contains(focus)) {
          break;
        }
      }
      order.append(focus);
      points.insert(focus, QJsonObject{});
    } else {
      const auto index = waypoint_->currentIndex();
      if (index < 0 || index >= order.size()) {
        return;
      }
      points.remove(order[index].toString());
      order.removeAt(index);
      focus = order[std::min(index, order.size() - 1)].toString();
    }
    task.insert("order", order);
    task.insert("waypoints", points);
    task.remove("physical_reviewed");
    task.remove("physical_context");
    task.remove("physical_review_waypoints");
    task.remove("physical_review_order");
    load_task(task);
    waypoint_->setCurrentText(focus);
  }

  void move_destination(int step)
  {
    const auto kind = task_kind();
    if ((kind != "custom" && kind != "cycle") || dirty_ || working()) {
      return;
    }
    auto names = waypoint_names(kind, task_);
    const int index = waypoint_index_;
    const int next = index + step;
    const int limit = kind == "cycle" ? 3 : names.size();
    if (index < 0 || index >= limit || next < 0 || next >= limit) {
      return;
    }
    const auto focus = names[index];
    names.swapItemsAt(index, next);
    task_.insert("order", QJsonArray::fromStringList(names));
    clear_physical_review();
    load_task(task_);
    waypoint_->setCurrentText(focus);
  }

  void load_task(const QJsonObject & task)
  {
    task_ = task;
    task_.remove("velocity_percent");
    custom_name_->setText(task_.value("name").toString());
    const QSignalBlocker backend_block(task_backend_);
    task_backend_->setCurrentIndex(std::max(0, task_backend_->findData(task_.value("backend").toString())));
    {
      const QSignalBlocker block(task_profile_);
      task_profile_->setCurrentIndex(std::max(0, task_profile_->findData(task_.value("profile").toString())));
    }
    if (loaded_example_ < 0) {
      const QSignalBlocker block(preset_);
      preset_->setCurrentIndex(-1);
    }
    {
      const QSignalBlocker block(physical_presets_);
      physical_presets_->setCurrentIndex(physical_presets_->findData(catalog_key()));
    }
    {
      const QSignalBlocker block(cycles_);
      const bool repeat = task_kind() == "repeat";
      const bool multiple = task_kind().isEmpty() || repeat || task_kind() == "custom" || cell_route();
      cycles_->setRange(repeat ? 2 : 1, multiple ? 8 : 1);
      cycles_->setValue(task.value("cycles").toInt(1));
    }
    if (live_joint()) {
      const QSignalBlocker axis_block(live_axis_);
      const QSignalBlocker offset_block(live_offset_);
      const QSignalBlocker duration_block(live_duration_);
      live_axis_->setCurrentIndex(live_axis_->findData(task.value("axis").toInt()));
      live_offset_->setValue(task.value("offset_deg").toDouble());
      live_duration_->setValue(task.value("duration_s").toDouble());
      refresh_live_limits();
    }
    {
      const QSignalBlocker block(waypoint_);
      waypoint_->clear();
      waypoint_->addItems(waypoint_names(task_kind(), task_));
      waypoint_index_ = 0;
      waypoint_->setCurrentIndex(waypoint_index_);
    }
    type_->setEnabled(task_kind() != "joint_test");
    invalidate();
    read_waypoint();
    refresh_configuration();
    auto_plan_ = true;
    if (mode_ == "offline" || automatic_catalog()) {
      request_plan();
    }
  }

  void update_live_joint()
  {
    if (!live_joint()) {
      return;
    }
    task_.insert("axis", live_axis_->currentData().toInt());
    task_.insert("offset_deg", live_offset_->value());
    set_duration(task_, live_duration_);
    invalidate();
    auto_plan_ = true;
    request_plan();
  }

  void refresh_configuration()
  {
    description_->setText(loaded_example_ >= 0 ?
      QString::fromUtf8(task_presets.at(loaded_example_).description) :
      task_.value("name").toString(tr("Tarea personalizada")));
    QString configuration = tr("Tarea personalizada");
    if (task_.value("offline_example").toBool()) {
      configuration = tr("Demostración · ejecución en Offline");
    } else if (live_demo()) {
      configuration = tr("Ciclo preprogramado: A = J1 +3°, B = J1/J2 +3°, C = J2 +3°, "
        "HOME = postura de partida. Duración automática por tramo. No requiere editar puntos.");
    } else if (live_joint()) {
      configuration = tr("INICIO y retorno usan la postura actual. En Offline, esa postura es simulada.");
    } else if (task_kind() == "air_triangle") {
      configuration = tr("Triángulo equilátero horizontal de 100 mm de lado, con la brida. "
        "Plan automático desde la postura vigente, retorno a HOME y repeticiones.");
    } else if (cell_route()) {
      configuration = tr("Plan cartesiano automático. HOME usa la postura inicial del lote "
        "y se conserva durante todas las repeticiones.");
    } else if (!task_kind().isEmpty()) {
      if (task_kind() == "custom") {
        configuration = tr("%1 destinos · %2").arg(task_.value("order").toArray().size())
          .arg(points_configured() ? tr("configurados") : tr("configuración pendiente"));
      } else if (!points_configured()) {
        configuration = task_kind() == "repeat" ?
          tr("Configura y guarda primero el ciclo A–B–C–HOME en el catálogo.") :
          tr("Configura y guarda los puntos de esta celda en Editar tarea.");
      } else if (task_.value("physical_reviewed").toBool() &&
        task_.value("physical_review_waypoints") == task_.value("waypoints") &&
        !current_context_.isEmpty() && current_context_ == task_.value("physical_context").toObject())
      {
        configuration = task_kind() == "repeat" ?
          tr("Usa el ciclo guardado. Requiere que ese ciclo haya sido verificado en el IRC5.") :
          tr("Inspección de la celda registrada para estos puntos.");
      } else {
        configuration = task_kind() == "repeat" ?
          tr("Completa la inspección y validación del ciclo A–B–C–HOME guardado.") :
          tr("Puntos configurados · inspección pendiente para la celda actual.");
      }
    } else if (task_.value("physical_reviewed").toBool()) {
      configuration = current_context_ == task_.value("physical_context").toObject() ?
        tr("Tarea personalizada · inspección registrada") :
        tr("Tarea personalizada · inspección de otra configuración");
    }
    const auto block = catalog_plan_block();
    configuration_->setText(block.isEmpty() ? configuration : block);
    refresh_goals();
    if (advanced_->isChecked()) {
      update_payload();
    }
  }

  bool points_configured() const
  {
    const auto points = task_.value("waypoints").toObject();
    const auto names = waypoint_names(task_kind(), task_);
    return std::all_of(names.begin(), names.end(), [&points](const QString & name) {
      return !points.value(name).toObject().isEmpty();
    });
  }

  QJsonArray initial_angles() const
  {
    return task_.value("waypoints").toObject().value("INICIO").toObject().value("q_rad").toArray();
  }

  void refresh_goals()
  {
    const auto points = task_.value("waypoints").toObject();
    const auto planned = (automatic_catalog() ? current_catalog_plan() : current_plan()) ?
      packet_.value("waypoints").toArray() : QJsonArray{};
    QStringList goals;
    if (cell_route()) {
      for (const auto & name : packet_.value("order").toArray()) {
        if (planning_reply_received() && name.isString()) {
          goals << name.toString();
        }
      }
      panel_goals_->setVisible(!goals.isEmpty());
      panel_goals_->setText(tr("Destinos por ciclo (%1) · retorno a HOME").arg(goals.size()));
      panel_goals_->setToolTip(goals.join(" → "));
      return;
    }
    panel_goals_->setToolTip({});
    for (const auto & name : waypoint_names(task_kind(), task_)) {
      const auto point = points.value(name).toObject();
      auto angles = point.value("q_rad").toArray();
      for (const auto & record : planned) {
        const auto target = record.toObject();
        if (target.value("name").toString() == name) {
          angles = target.value("q_rad").toArray();
          break;
        }
      }
      if (joint_input_valid(angles)) {
        QStringList values;
        for (const auto & angle : angles) {
          values << QString::number(angle.toDouble() * (radians_ ? 1.0 : deg_per_rad),
            'f', radians_ ? 5 : 3);
        }
        goals << tr("%1 · [%2] %3").arg(name, values.join(", "), radians_ ? tr("rad") : tr("°"));
      } else {
        goals << tr("%1 · %2").arg(name,
          point.isEmpty() ? (live_demo() ? tr("Se calcula desde la postura vigente") : tr("Pendiente")) :
          tr("Pose guardada · ver articulaciones al planificar"));
      }
    }
    const auto text = tr("Objetivos en orden J1–J6\n%1").arg(goals.join('\n'));
    goals_->setText(text);
    panel_goals_->setVisible((automatic_catalog() && !live_joint()) || !planned.isEmpty());
    panel_goals_->setText(text);
  }

  void display_waypoint_fields()
  {
    joint_fields_->setVisible(type_->currentIndex() == 0);
    pose_fields_->setVisible(type_->currentIndex() == 1);
  }

  void read_waypoint()
  {
    const auto waypoint = task_.value("waypoints").toObject().value(waypoint_->currentText()).toObject();
    const QSignalBlocker type_block(type_);
    const QSignalBlocker point_block(point_);
    const QSignalBlocker duration_block(waypoint_duration_);
    const QSignalBlocker profile_block(waypoint_profile_);
    waypoint_duration_->setValue(waypoint.value("duration_s").toDouble());
    waypoint_profile_->setCurrentIndex(std::max(0, waypoint_profile_->findData(waypoint.value("profile").toString())));
    const bool pose = waypoint.contains("matrix");
    type_->setCurrentIndex(pose ? 1 : 0);
    display_waypoint_fields();
    point_->setCurrentIndex(waypoint.value("point").toString(waypoint.contains("matrix") ? "tcp" : "flange") == "flange" ? 1 : 0);
    selected_branch_ = waypoint.value("branch").toString();
    set_joints(waypoint.value("q_rad").toArray());
    matrix_->setMatrix(waypoint.value("matrix").toArray());
    dirty_ = false;
    validation_->setText(waypoint.isEmpty() ? tr("Waypoint pendiente de configurar.") : QString());
    show_branch();
    refresh_buttons();
  }

  void set_joints(const QJsonArray & values)
  {
    joint_baseline_ = values;
    joint_changed_.fill(false);
    for (int axis = 0; axis < 6; ++axis) {
      const QSignalBlocker block(joints_[axis]);
      const double q = values.size() == 6 ? values[axis].toDouble() : 0.0;
      joints_[axis]->setValue(q * (radians_ ? 1.0 : deg_per_rad));
    }
  }

  void edited()
  {
    if (!dirty_) {
      clear_physical_review();
      invalidate();
      refresh_configuration();
    }
    dirty_ = true;
    validation_->setText(tr("Cambios pendientes de guardar."));
    show_branch();
    refresh_buttons();
  }

  void show_branch()
  {
    branch_->setText(selected_branch_.isEmpty() ? tr("Rama Pieper pendiente") :
      tr("Rama Pieper · %1").arg(selected_branch_));
  }

  bool has_pending_edit()
  {
    if (dirty_) {
      validation_->setText(tr("Valide y guarde primero el waypoint editado."));
    }
    return dirty_;
  }

  void invalidate()
  {
    if (!request_.isEmpty()) {
      send_({{"action", "invalidate"}, {"request_id", request_}});
      request_.clear();
    }
    queued_preview_ = false;
    dispatch_pending_ = false;
    reset_authorization();
    progress_->setText(tr("Sin plan vigente"));
    displayed_legs_ = {};
    legs_->setRowCount(0);
    legs_->setVisible(false);
    refresh_candidates();
    refresh_goals();
    refresh_buttons();
  }

  static bool joint_input_valid(const QJsonArray & joints)
  {
    if (joints.size() != 6) {
      return false;
    }
    for (int axis = 0; axis < 6; ++axis) {
      if (!joints[axis].isDouble() || !std::isfinite(joints[axis].toDouble()) ||
        joints[axis].toDouble() < imt342_irb120_rviz::lower[axis] || joints[axis].toDouble() > upper[axis])
      {
        return false;
      }
    }
    return true;
  }

  QJsonObject validated_waypoint()
  {
    if (!dirty_ && task_.value("waypoints").toObject().value(waypoint_->currentText()).toObject().isEmpty()) {
      validation_->setText(tr("Introduce un destino o captura una postura medida."));
      return {};
    }
    const auto duration = waypoint_duration_->value();
    if (duration > 0.0 && duration < 0.1) {
      validation_->setText(tr("La duración debe ser automática o estar entre 0,10 y 30 s."));
      return {};
    }
    if (type_->currentIndex() == 0) {
      QJsonArray angles;
      for (std::size_t axis = 0; axis < joints_.size(); ++axis) {
        const auto * joint = joints_[axis];
        if (!joint->hasAcceptableInput()) {
          validation_->setText(tr("Revise J%1 y su límite articular.").arg(axis + 1));
          return {};
        }
        const double q = joint->value() / (radians_ ? 1.0 : deg_per_rad);
        const auto unchanged = joint_baseline_.size() == 6 && !joint_changed_[axis];
        angles.append(unchanged ? joint_baseline_[axis].toDouble() :
          std::clamp(q, imt342_irb120_rviz::lower[axis], upper[axis]));
      }
      QJsonObject waypoint{{"q_rad", angles}};
      set_duration(waypoint, waypoint_duration_);
      store_profile(waypoint, waypoint_profile_);
      return waypoint;
    }
    const auto values = matrix_->matrix();
    const auto pose = read_pose_matrix(values);
    if (!pose) {
      validation_->setText(tr("Complete las 16 celdas con números finitos. Use punto decimal."));
      return {};
    }
    if (!is_rigid_pose(*pose)) {
      validation_->setText(tr("Compruebe la última fila [0,0,0,1] y la rotación ortogonal de determinante +1."));
      return {};
    }
    QJsonObject waypoint{{"matrix", values}, {"point", point_->currentData().toString()}};
    if (!selected_branch_.isEmpty()) {
      waypoint.insert("branch", selected_branch_);
    }
    set_duration(waypoint, waypoint_duration_);
    store_profile(waypoint, waypoint_profile_);
    return waypoint;
  }

  void store_waypoint(const QString & name, const QJsonObject & value)
  {
    auto waypoints = task_.value("waypoints").toObject();
    waypoints.insert(name, value);
    task_.insert("waypoints", waypoints);
    clear_physical_review();
    dirty_ = false;
    if (value.contains("q_rad")) {
      set_joints(value.value("q_rad").toArray());
    }
    invalidate();
    refresh_configuration();
    if (automatic_catalog()) {
      auto_plan_ = true;
      request_plan();
    }
  }

  void open_task()
  {
    if (has_pending_edit()) {
      return;
    }
    const auto path = QFileDialog::getOpenFileName(this, tr("Abrir tarea"), directory(), tr("JSON (*.json)"));
    if (path.isEmpty()) {
      return;
    }
    const auto task = read_json(path);
    if (file_task_valid(task)) {
      loaded_example_ = -1;
      load_task(task);
      return;
    }
    QMessageBox::warning(this, tr("Abrir tarea"), tr("Revise el formato JSON, el tipo de tarea, "
      "los ciclos y sus puntos."));
  }

  bool file_task_valid(const QJsonObject & task) const
  {
    const auto backend = task.value("backend").toString();
    if (!valid_profile(task) || (task.contains("backend") &&
      backend != "rws" && backend != "rws_trajectory"))
    {
      return false;
    }
    const auto cycles = task.value("cycles");
    const auto waypoints = task.value("waypoints").toObject();
    const auto kind = task.value("kind").toString();
    if (cell_kind(kind)) {
      return cycles.isDouble() && cycles.toDouble() == cycles.toInt() &&
        cycles.toInt() >= 1 && cycles.toInt() <= 8 &&
        !task.value("offline_example").toBool() && !task.contains("waypoints") &&
        !task.contains("order") && !task.contains("duration_s");
    }
    if (kind == "demo_cycle_live") {
      return !task.value("offline_example").toBool() && !task.contains("waypoints") &&
        !task.contains("order") && cycles.toDouble() == 1.0;
    }
    if (kind == "joint_test_live") {
      const auto axis = task.value("axis");
      const auto offset = task.value("offset_deg");
      return !task.value("offline_example").toBool() && !task.contains("waypoints") &&
        cycles.toDouble() == 1.0 && axis.isDouble() && axis.toInt() == axis.toDouble() &&
        axis.toInt() >= 1 && axis.toInt() <= 6 && offset.isDouble() &&
        offset.toDouble() != 0.0 && std::abs(offset.toDouble()) <= offset_span_deg(axis.toInt() - 1) &&
        valid_duration(task);
    }
    if (task.contains("kind") && kind != "joint_test" && kind != "cycle" && kind != "repeat" && kind != "custom") {
      return false;
    }
    if (!kind.isEmpty() && task.value("offline_example").toBool()) {
      return false;
    }
    const auto names = waypoint_names(kind, task);
    if (kind == "custom") {
      const QSet<QString> unique(names.begin(), names.end());
      const auto name = task.value("name").toString().trimmed();
      static const QRegularExpression point_name(QStringLiteral("^[A-Za-z0-9_ -]{1,32}$"));
      if (names.isEmpty() || names.size() > 32 || unique.size() != names.size() || name.isEmpty() ||
        name.size() > 96 ||
        std::any_of(names.begin(), names.end(), [](const QString & point) {
          return !point_name.match(point).hasMatch();
        }))
      {
        return false;
      }
    } else if (task.contains("order")) {
      const QSet<QString> standard{
        QStringLiteral("A"), QStringLiteral("B"), QStringLiteral("C"), QStringLiteral("HOME")};
      if (kind == "joint_test" || names.size() != 4 || names.back() != "HOME" ||
        QSet<QString>(names.begin(), names.end()) != standard)
      {
        return false;
      }
    }
    const int count = cycles.toInt();
    if (!cycles.isDouble() || cycles.toDouble() != count ||
      count < (kind == "repeat" ? 2 : 1) ||
      count > (kind.isEmpty() || kind == "repeat" || kind == "custom" ? 8 : 1) ||
      waypoints.size() != names.size())
    {
      return false;
    }
    for (const auto & name : names) {
      const auto value = waypoints.value(name);
      if (!value.isObject()) {
        return false;
      }
      const auto destination = value.toObject();
      if (destination.isEmpty()) {
        continue;
      }
      if (!valid_duration(destination) || !valid_profile(destination) ||
        destination.contains("q_rad") == destination.contains("matrix"))
      {
        return false;
      }
      if (destination.contains("q_rad")) {
        if (!joint_input_valid(destination.value("q_rad").toArray())) {
          return false;
        }
      } else if (kind == "joint_test") {
        return false;
      } else {
        const auto point = destination.value("point").toString("tcp");
        const auto matrix = read_pose_matrix(destination.value("matrix").toArray());
        if (!matrix || !is_rigid_pose(*matrix) || (point != "tcp" && point != "flange")) {
          return false;
        }
      }
    }
    return true;
  }

  bool save_json(const QString & title, const QString & name, const QJsonObject & object)
  {
    const auto path = QFileDialog::getSaveFileName(
      this, title, QDir(directory()).filePath(name), tr("JSON (*.json)"));
    if (path.isEmpty()) {
      return false;
    }
    if (!write_json(path, object)) {
      QMessageBox::warning(this, title, tr("No se pudo guardar el archivo."));
      return false;
    }
    return true;
  }

  static bool write_json(const QString & path, const QJsonObject & object)
  {
    const auto bytes = QJsonDocument(object).toJson();
    QSaveFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size() && file.commit();
  }

  void update_payload()
  {
    payload_->setPlainText(QString::fromUtf8(QJsonDocument(task_).toJson()));
  }

  std::function<void(const QJsonObject &)> send_;
  std::function<QString()> stamp_;
  QString catalog_directory_;
  std::function<QJsonArray()> target_;
  std::function<void(const QString &)> select_backend_;
  QJsonObject task_, packet_, last_result_, analysis_, current_context_;
  QString approved_token_, previous_token_, used_token_, authorize_stamp_;
  bool authorize_pending_{false}, execute_pending_{false};
  QJsonArray actual_, displayed_legs_, joint_baseline_;
  QString mode_, request_, request_mode_, selected_branch_, catalog_block_;
  int waypoint_index_{}, loaded_example_{-1};
  bool radians_{true}, fresh_{false}, connected_{false}, dirty_{false}, dispatch_pending_{false};
  bool preview_running_{false}, auto_plan_{false}, queued_preview_{false};
  int builtin_catalog_size_{};
  QComboBox * preset_{}, * physical_presets_{}, * waypoint_{}, * type_{}, * point_{}, * candidates_{};
  QComboBox * live_axis_{}, * task_profile_{}, * waypoint_profile_{};
  QSpinBox * cycles_{};
  QComboBox * task_backend_{};
  QPushButton * use_task_backend_{};
  QDoubleSpinBox * live_offset_{}, * live_duration_{}, * waypoint_duration_{};
  QWidget * joint_fields_{}, * pose_fields_{}, * live_joint_controls_{};
  std::array<QDoubleSpinBox *, 6> joints_{};
  std::array<bool, 6> joint_changed_{};
  PoseGrid * matrix_{};
  QLabel * description_{}, * configuration_{}, * branch_{}, * validation_{};
  QLabel * progress_{}, * candidate_label_{}, * goals_{}, * panel_goals_{};
  QPushButton * capture_{}, * copy_start_{}, * plan_{}, * preview_{}, * simulate_{}, * stop_{}, * choose_{};
  QPushButton * move_destination_up_{}, * move_destination_down_{};
  QTableWidget * legs_{};
  QGroupBox * advanced_{}, * editor_{};
  QWidget * library_{};
  QDialog * dialog_{};
  QPushButton * edit_{}, * inspect_{}, * authorize_{}, * execute_{}, * save_catalog_{};
  QPushButton * delete_catalog_{}, * add_destination_{}, * remove_destination_{}, * copy_target_{};
  QLineEdit * custom_name_{};
  QPlainTextEdit * payload_{};
};
}  // namespace imt342_irb120_rviz
