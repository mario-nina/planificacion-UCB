#include "imt342_irb120_rviz/panel_values.hpp"

#include "panel_pose.hpp"
#include "task_editor.hpp"
#include "pose_grid.hpp"
#include "panel_presentation.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QFrame>
#include <QHeaderView>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMap>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QStringList>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QUuid>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace imt342_irb120_rviz
{
using State = sensor_msgs::msg::JointState;
using Text = std_msgs::msg::String;
using Clock = std::chrono::steady_clock;

class JointPanel : public rviz_common::Panel
{
public:
  explicit JointPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * panel_layout = new QVBoxLayout(this);
    panel_layout->setSpacing(4);
    tabs_ = new QTabWidget;
    tabs_->setUsesScrollButtons(true);
    tabs_->setElideMode(Qt::ElideNone);
    tabs_->setObjectName("imt342_right_tabs");
    auto * layout = add_tab(tabs_, tr("FK"));
    auto * ik_layout = add_tab(tabs_, tr("IK"));
    auto * task_layout = add_tab(tabs_, tr("Tareas"));
    auto * diagnostics_layout = add_tab(tabs_, tr("Diagnóstico"));
    diagnostics_tab_ = tabs_->widget(tabs_->count() - 1);
    auto * settings = new QFormLayout;
    mode_ = new QComboBox;
    mode_->addItem(tr("Offline"), "offline");
    mode_->addItem(tr("Solo lectura"), "observe");
    mode_->addItem(tr("Control físico"), "control");
    backend_ = new QComboBox;
    backend_->addItem(tr("RWS/RAPID — Normal"), "rws");
    backend_->addItem(tr("RWS/RAPID — Trayectorias v2"), "rws_trajectory");
    backend_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    backend_->setMinimumContentsLength(16);
    backend_->setToolTip(tr("Selecciona el servidor RAPID para esta sesión."));
    unit_ = new QComboBox;
    unit_->addItems({tr("Grados"), tr("Radianes")});
    auto * operation = new QHBoxLayout;
    operation->setContentsMargins(0, 0, 0, 0);
    operation->addWidget(mode_, 1);
    unit_->setAccessibleName(tr("Unidades angulares"));
    unit_->setToolTip(tr("Unidades angulares"));
    operation->addWidget(unit_);
    settings->addRow(tr("Operación"), operation);
    settings->addRow(tr("Comunicación"), backend_);
    profile_ = new QComboBox;
    profile_->setObjectName("trajectoryProfile");
    profile_->addItem(tr("LSPB"), QStringLiteral("lspb"));
    profile_->addItem(tr("Quíntica"), QStringLiteral("quintic"));
    settings->addRow(tr("Perfil de referencia"), profile_);
    profile_transport_ = new QLabel;
    profile_transport_->setWordWrap(true);
    settings->addRow(profile_transport_);
    panel_layout->addLayout(settings);
    connect_irc5_ = new QPushButton(tr("Conectar y preparar IRC5"));
    connect_irc5_->setToolTip(tr("Comprueba RWS y prepara el servidor RAPID si procede. "
      "La habilitación para mover se realiza por separado."));
    connect(connect_irc5_, &QPushButton::clicked, this, [this]() {
      if (!rapid_prepare_ || rapid_prepare_->get_subscription_count() == 0) {
        connection_hint_ = tr("ROS 2 desconectado. Inicie la sesión mediante irc5_demo.sh.");
        actual_source_->setText(connection_hint_);
        return;
      }
      const auto state = analysis_.value("irc5_connection").toObject();
      const auto recovery_id = state.value("recovery_id").toString();
      const bool recovery = state.value("state").toString() == "module_missing" && !recovery_id.isEmpty();
      if (recovery && QMessageBox::question(this, tr("Recuperar servidor RAPID"),
          tr("¿Cargar el módulo propio seleccionado y preparar el servidor en T_ROB1? "
             "Si se verifican las condiciones, la recuperación coloca el puntero e inicia RAPID. "
             "Los demás módulos se conservan."),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
      {
        return;
      }
      connection_hint_.clear();
      connection_clicked_ = Clock::now();
      publish_text(rapid_prepare_, recovery ? "recover:" + recovery_id.toStdString() : "connect");
      actual_source_->setText(tr("Conectando…"));
    });
    panel_layout->addWidget(connect_irc5_);
    actual_source_ = new QLabel(tr("Esperando al coordinador"));
    actual_source_->setWordWrap(true);
    panel_layout->addWidget(actual_source_);
    arrival_status_ = new QLabel;
    arrival_status_->setObjectName("arrivalStatus");
    arrival_status_->setWordWrap(true);
    arrival_status_->setToolTip(tr("Error máximo de posición de los seis ejes. La banda no sustituye el acuse terminal RAPID."));
    panel_layout->addWidget(arrival_status_);
    panel_layout->addWidget(tabs_, 1);

    auto * grid = new QGridLayout;
    grid->setColumnStretch(2, 1);
    grid->setHorizontalSpacing(4);
    grid->setVerticalSpacing(2);
    grid->addWidget(new QLabel(tr("Eje")), 0, 0);
    grid->addWidget(new QLabel(tr("Objetivo")), 0, 1, 1, 2);
    grid->addWidget(new QLabel(tr("Actual")), 0, 3);
    auto * effective_header = new QLabel(tr("Pico ref."));
    effective_header->setToolTip(tr("Pico de velocidad calculado para el tramo vigente; N/D si no hay referencia. No es velocidad del servo."));
    auto * measured_header = new QLabel(tr("Medida"));
    measured_header->setToolTip(tr("Velocidad estimada de muestras J1–J6 recibidas. N/D con feedback insuficiente."));
    grid->addWidget(effective_header, 0, 4);
    grid->addWidget(measured_header, 0, 5);
    for (std::size_t axis = 0; axis < joint_names.size(); ++axis) {
      const int row = static_cast<int>(axis) + 1;
      auto * axis_label = new QLabel(QStringLiteral("J%1").arg(row));
      edits_[axis] = new QDoubleSpinBox;
      edits_[axis]->setKeyboardTracking(false);
      edits_[axis]->setAccessibleName(axis_label->text() + tr(" objetivo"));
      sliders_[axis] = new QSlider(Qt::Horizontal);
      sliders_[axis]->setRange(0, slider_max);
      sliders_[axis]->setAccessibleName(edits_[axis]->accessibleName());
      actual_labels_[axis] = new QLabel(QStringLiteral("—"));
      grid->addWidget(axis_label, row, 0);
      grid->addWidget(edits_[axis], row, 1);
      grid->addWidget(sliders_[axis], row, 2);
      grid->addWidget(actual_labels_[axis], row, 3);
      speed_effective_[axis] = new QLabel(tr("N/D"));
      speed_measured_[axis] = new QLabel(tr("N/D"));
      grid->addWidget(speed_effective_[axis], row, 4);
      grid->addWidget(speed_measured_[axis], row, 5);
      connect(edits_[axis], qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [this, axis](double shown) {
          target_[axis] = editor_radians(axis, shown, factor(), edits_[axis]->minimum(),
            edits_[axis]->maximum());
          const QSignalBlocker blocker(sliders_[axis]);
          sliders_[axis]->setValue(slider_value(axis, target_[axis]));
          publish_target();
        });
      connect(sliders_[axis], &QSlider::valueChanged, this, [this, axis](int tick) {
        target_[axis] = slider_radians(axis, tick);
        const QSignalBlocker blocker(edits_[axis]);
        edits_[axis]->setValue(target_[axis] * factor());
        publish_target();
      });
    }
    layout->addLayout(grid);
    auto * global = new QHBoxLayout;
    speed_units_ = new QLabel;
    global->addWidget(new QLabel(tr("Planificación automática")));
    global->addWidget(speed_units_);
    global->addStretch();
    layout->addLayout(global);
    fk_duration_ = add_duration_field(layout);

    auto * options = new QHBoxLayout;
    use_actual_ = new QPushButton(tr("Copiar actual"));
    options->addWidget(use_actual_);
    auto * zero = new QPushButton(tr("Objetivo a cero"));
    zero->setObjectName("zeroJointTarget");
    zero->setToolTip(tr("Cambia solo la consigna articular. HOME de la celda se configura por separado."));
    connect(zero, &QPushButton::clicked, this, [this]() {set_target(Joints{});});
    options->addWidget(zero);
    layout->addLayout(options);
    connect(use_actual_, &QPushButton::clicked, this, [this]() {
      if (actual_ && actual_fresh_) {
        set_target(*actual_);
      }
    });

    auto * preview_row = new QHBoxLayout;
    preview_ = new QPushButton(tr("Previsualizar"));
    cancel_ = new QPushButton(tr("Cancelar"));
    apply_ = new QPushButton(tr("Aplicar en simulación"));
    preview_row->addWidget(preview_);
    preview_row->addWidget(cancel_);
    preview_row->addWidget(apply_);
    layout->addLayout(preview_row);
    arm_ = new QPushButton(tr("Habilitar una ejecución física…"));
    arm_->setEnabled(false);
    arm_->hide();
    layout->addWidget(arm_);
    fk_execution_hint_ = new QLabel;
    fk_execution_hint_->setWordWrap(true);
    fk_execution_hint_->hide();
    layout->addWidget(fk_execution_hint_);
    connect(arm_, &QPushButton::clicked, this, [this]() {request_arm(false);});
    connect(preview_, &QPushButton::clicked, this,
      [this]() {publish_target(preview_target_);});
    connect(cancel_, &QPushButton::clicked, this,
      [this]() {publish_text(route_preview_, "cancel");});
    stop_motion_ = new QPushButton(tr("Solicitar detención de orden física"));
    stop_motion_->hide();
    layout->addWidget(stop_motion_);
    connect(stop_motion_, &QPushButton::clicked, this, [this]() {
      if (active_mode_ == "control") {
        publish_text(preview_command_, "cancel");
      }
    });
    connect(apply_, &QPushButton::clicked, this, [this]() {
      if (active_mode_ == "offline") {
        publish_target(apply_target_);
      } else if (active_mode_ == "control" && authorization_matches(false)) {
        publish_target(execute_target_);
      }
    });
    connect(mode_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
      publish_text(mode_command_, mode_->itemData(index).toString().toStdString());
      apply_->setEnabled(false);
    });
    connect(backend_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
      preferred_backend_ = backend_->itemData(index).toString();
      Q_EMIT configChanged();
      apply_->setEnabled(false);
      arm_->setEnabled(false);
      publish_text(backend_command_, preferred_backend_.toStdString());
    });
    connect(unit_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {refresh_units();});
    connect(unit_, qOverload<int>(&QComboBox::activated), this, [this](int) {
      Q_EMIT configChanged();
    });
    connect(profile_, qOverload<int>(&QComboBox::activated), this, [this](int) {
      preferred_profile_ = profile_->currentData().toString();
      profile_pending_ = true;
      invalidate_routes(true);
      Q_EMIT configChanged();
    });

    build_fk(layout);
    build_routes(ik_layout);
    task_editor_ = new TaskEditor([this](const QJsonObject & command) {
      send_task(command);
    }, this, [this]() {
      return node_ ? QString::number(node_->now().nanoseconds()) : QString();
    }, {}, [this]() {
      QJsonArray q;
      for (const auto value : target_) {
        q.append(value);
      }
      return q;
    }, [this](const QString & backend) {
      publish_text(backend_command_, backend.toStdString());
    });
    task_layout->addWidget(task_editor_);
    build_diagnostics(diagnostics_layout);
    for (auto * page : {layout, ik_layout, task_layout, diagnostics_layout}) {
      page->addStretch();
    }
    status_ = new QLabel(tr("Esperando estado de previsualización"));
    status_->setWordWrap(true);
    status_->hide();
    status_->setTextFormat(Qt::PlainText);
    panel_layout->addWidget(status_);
    refresh_units();
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {refresh_state();});
    timer->start(100);
  }

  void onInitialize() override
  {
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {
      actual_source_->setText(tr("Nodo ROS de RViz no disponible"));
      return;
    }
    node_ = abstraction->get_raw_node();
    if (!node_) {
      actual_source_->setText(tr("Nodo ROS de RViz no disponible"));
      return;
    }
    target_pub_ = node_->create_publisher<State>("/imt342/target", 10);
    preview_target_ = node_->create_publisher<State>("/imt342/preview_target", 10);
    apply_target_ = node_->create_publisher<State>("/imt342/apply_target", 10);
    execute_target_ = node_->create_publisher<State>("/imt342/execute_target", 10);
    mode_command_ = node_->create_publisher<Text>("/imt342/mode_command", 10);
    physical_command_ = node_->create_publisher<Text>("/imt342/physical_command", 10);
    rapid_prepare_ = node_->create_publisher<Text>("/imt342/rapid_prepare", 10);
    task_command_ = node_->create_publisher<Text>("/imt342/task_command", 10);
    backend_command_ = node_->create_publisher<Text>("/imt342/backend_command", 10);
    preview_command_ = node_->create_publisher<Text>("/imt342/preview_command", 10);

    route_request_ = node_->create_publisher<Text>("/imt342/pose_plan_request", 10);
    route_preview_ = node_->create_publisher<Text>("/imt342/pose_plan_preview", 10);
    actual_pose_sub_ = subscribe_text("/imt342/actual_pose_matrix", &Inbox::actual_pose);
    analysis_command_ = node_->create_publisher<Text>("/imt342/analysis_command", 10);
    analysis_sub_ = subscribe_text("/imt342/analysis", &Inbox::analysis);
    const auto inbox = inbox_;
    route_result_sub_ = node_->create_subscription<Text>(
      "/imt342/pose_plan_result", rclcpp::QoS(1).transient_local(), [inbox](Text::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->route_result = msg->data;
      });
    actual_sub_ = node_->create_subscription<State>(
      "/imt342/actual", 10, [inbox](State::ConstSharedPtr msg) {
        const auto ordered = ordered_joints(msg->name, msg->position);
        if (ordered) {
          std::lock_guard<std::mutex> lock(inbox->mutex);
          inbox->actual = ordered;
          inbox->received = Clock::now();
        }
      });
    backend_sub_ = subscribe_text("/imt342/backend_state", &Inbox::backend);
    mode_sub_ = subscribe_text("/imt342/mode_state", &Inbox::mode);

  }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Radians", unit_->currentIndex() == 1);
    config.mapSetValue("TrajectoryDurationSec", 0.0);
    // LSPB is the initial session profile even if an older configuration saved quintic.
    config.mapSetValue("TrajectoryProfile", QStringLiteral("lspb"));
    if (!preferred_backend_.isEmpty()) {
      config.mapSetValue("PreferredOfflineBackend", preferred_backend_);
    }
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    bool radians = false;
    if (config.mapGetBool("Radians", &radians)) {
      unit_->setCurrentIndex(radians ? 1 : 0);
    }
    // Do not carry an old manually requested duration into a physical session.
    set_duration_fields(0.0);
    duration_pending_ = true;
    QString preferred;
    preferred_profile_ = QStringLiteral("lspb");
    profile_->setCurrentIndex(profile_->findData(preferred_profile_));
    profile_pending_ = true;
    if (config.mapGetString("PreferredOfflineBackend", &preferred) &&
      backend_->findData(preferred) >= 0)
    {
      preferred_backend_ = preferred;
      offline_preference_pending_ = true;
    }
  }

private:
#ifdef IMT342_PANEL_TEST
  friend struct JointPanelTestAccess;
#endif

  struct Inbox
  {
    std::mutex mutex;
    std::optional<Joints> actual;
    Clock::time_point received;
    std::string mode;
    std::string backend;
    std::string analysis;
    std::string route_result;
    std::string actual_pose;
  };

  rclcpp::Subscription<Text>::SharedPtr subscribe_text(
    const char * topic, std::string Inbox::* member)
  {
    const auto inbox = inbox_;
    return node_->create_subscription<Text>(
      topic, rclcpp::QoS(1).transient_local(),
      [inbox, member](Text::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox.get()->*member = msg->data;
      });
  }

  static QVBoxLayout * add_tab(QTabWidget * tabs, const QString & name)
  {
    auto * scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto * contents = new QWidget;
    auto * layout = new QVBoxLayout(contents);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(5);
    scroll->setWidget(contents);
    tabs->addTab(scroll, name);
    return layout;
  }

  void publish_analysis(const QJsonObject & request)
  {
    publish_text(analysis_command_,
      QJsonDocument(request).toJson(QJsonDocument::Compact).toStdString());
  }

  static QString translation_text(const QJsonArray & matrix)
  {
    const auto pose = read_pose_matrix(matrix);
    return pose ? tr("X %1 · Y %2 · Z %3")
      .arg((*pose)[0][3], 0, 'f', 5).arg((*pose)[1][3], 0, 'f', 5)
      .arg((*pose)[2][3], 0, 'f', 5) : tr("N/D");
  }

  void request_arm(bool ik)
  {
    const auto * button = ik ? ik_arm_ : arm_;
    if (!button->isEnabled() || !node_) {
      return;
    }
    if (analysis_.value("physical_armed").toBool()) {
      publish_text(physical_command_, "disarm");
      return;
    }
    const auto requested_backend = backend_->currentData();
    const auto requested_route = route_id_;
    const auto requested_branch = selected_route().value("branch").toString();
    const auto requested_target = target_;
    if (QMessageBox::question(this, tr("Habilitar ejecución física"),
        tr("Confirme que el operador y el laboratorio autorizan una ejecución y "
           "que la celda está despejada. La habilitación permite una orden posterior."),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes ||
      !button->isEnabled() || requested_backend != backend_->currentData() ||
      analysis_.value("physical_armed").toBool())
    {
      return;
    }
    if ((ik && (requested_route != route_id_ ||
      requested_branch != selected_route().value("branch").toString() || !ik_preview_matches())) ||
      (!ik && (requested_target != target_ || !fk_preview_matches())))
    {
      return;
    }
    const auto scope = ik ? QStringLiteral(":ik:%1:%2").arg(requested_route, requested_branch) :
      QStringLiteral(":fk");
    publish_text(physical_command_, "arm:" + std::to_string(node_->now().nanoseconds()) +
      scope.toStdString());
  }

  void build_fk(QVBoxLayout * layout)
  {
    auto * point_row = new QHBoxLayout;
    point_row->addWidget(new QLabel(tr("Pose resultante")));
    fk_point_ = new QComboBox;
    fk_point_->addItem(tr("TCP activo"), "tcp");
    fk_point_->addItem(tr("Brida"), "flange");
    fk_point_->setCurrentIndex(1);
    point_row->addWidget(fk_point_);
    layout->addLayout(point_row);
    fk_values_ = new QLabel;
    fk_values_->setToolTip(tr("El TCP activo carece de calibración física independiente."));
    fk_values_->setWordWrap(true);
    layout->addWidget(fk_values_);
    fk_matrix_ = new PoseGrid(this, false, true);
    fk_matrix_->setAccessibleName(tr("Matriz de la pose resultante 4×4"));
    layout->addWidget(fk_matrix_);
    connect(fk_point_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {refresh_fk();});
  }

  void refresh_fk()
  {
    const bool observe = active_mode_ == "observe";
    const auto matrix = analysis_.value(observe ? "actual_fk" : "target_fk").toObject()
      .value(fk_point_->currentData().toString()).toObject().value("matrix").toArray();
    const auto visible = observe && (!actual_fresh_ || !live_analysis()) ? QJsonArray{} : matrix;
    if (visible != last_fk_matrix_) {
      last_fk_matrix_ = visible;
      fk_matrix_->setMatrix(visible);
    }
    fk_values_->setText(tr("Base del robot → %1 · %2\nPosición [m] · %3")
      .arg(fk_point_->currentText(), observe ? tr("Postura observada") : tr("Objetivo"), translation_text(visible)));
  }

  void switch_goal_point(const QString & point)
  {
    if (point == goal_key_) {
      return;
    }
    const auto current = read_pose_matrix(read_matrix());
    const auto tool = read_pose_matrix(analysis_.value("tool_offset").toObject().value("matrix").toArray());
    if (current && is_rigid_pose(*current) && tool) {
      set_matrix(pose_json(multiply_pose(*current, point == "flange" ? rigid_inverse(*tool) : *tool)));
      ik_feedback_->setText(tr("Destino convertido al punto seleccionado mediante el offset activo."));
    } else {
      set_matrix({});
      ik_feedback_->setText(tr("Cargue o escriba una matriz para el nuevo punto seleccionado."));
    }
    goal_key_ = point;
    invalidate_routes(true);
  }

  void receive_analysis(const QJsonObject & packet)
  {
    if (packet.isEmpty()) {
      return;
    }
    analysis_ = packet;
    last_analysis_ = Clock::now();
    const auto incident = packet.value("incident").toObject();
    if (route_options_->rowCount() > 0 && packet.contains("pose_plan_id") &&
      packet.value("pose_plan_id").toString().isEmpty() &&
      incident.value("code").toString() == "RUTA_OBSOLETA")
    {
      invalidate_routes();
      route_summary_->setText(incident.value("detail").toString());
    }
    const auto plan = packet.value("plan").toObject();
    const auto branch = plan.value("branch").toString();
    const bool route_running = !branch.isEmpty() && !route_id_.isEmpty() &&
      packet.value("pose_plan_id").toString() == route_id_ &&
      (plan.value("phase").toString() == "running" || plan.value("paused").toBool());
    route_playback_->setVisible(route_running);
    route_pause_->setVisible(!plan.value("paused").toBool());
    route_resume_->setVisible(plan.value("paused").toBool());
  }

  void set_matrix(const QJsonArray & matrix)
  {
    matrix_->setMatrix(matrix);
  }

  QJsonArray read_matrix() const
  {
    return matrix_->matrix();
  }

  void invalidate_routes(bool notify = false)
  {
    if (notify && !route_id_.isEmpty()) {
      publish_analysis(QJsonObject{{"invalidate_route", route_id_}});
    }
    route_id_.clear();
    route_origin_ = QJsonArray();
    route_options_->setRowCount(0);
    route_summary_->clear();
  }

  void build_routes(QVBoxLayout * layout)
  {
    auto * point = new QHBoxLayout;
    point->addWidget(new QLabel(tr("Base del robot →")));
    goal_point_ = new QComboBox;
    goal_point_->addItem(tr("Brida"), "flange");
    goal_point_->addItem(tr("TCP activo"), "tcp");
    goal_point_->setCurrentIndex(0);
    point->addWidget(goal_point_, 1);
    layout->addLayout(point);
    matrix_ = new PoseGrid(this, true);
    matrix_->setToolTip(tr("La última columna expresa posición en metros."));
    layout->addWidget(matrix_);
    matrix_->onEdit([this]() {invalidate_routes(true);});
    connect(goal_point_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
      switch_goal_point(goal_point_->itemData(index).toString());
    });
    plan_from_tcp_ = new QPushButton(tr("Calcular IK"));
    layout->addWidget(plan_from_tcp_);
    ik_feedback_ = new QLabel;
    ik_feedback_->setWordWrap(true);
    ik_feedback_->hide();
    layout->addWidget(ik_feedback_);
    connect(plan_from_tcp_, &QPushButton::clicked, this, [this]() {request_routes();});
    route_options_ = new QTableWidget(0, 4);
    route_options_->setAccessibleName(tr("Comparación de ramas y rutas"));
    route_options_->setHorizontalHeaderLabels({tr("N.º"), tr("Hombro · codo · muñeca"), tr("LSPB [s]"), tr("Estado")});
    route_options_->verticalHeader()->hide();
    auto * columns = route_options_->horizontalHeader();
    columns->setMinimumSectionSize(38);
    columns->setSectionResizeMode(QHeaderView::Fixed);
    columns->resizeSection(0, 38);
    columns->resizeSection(2, 66);
    columns->resizeSection(3, 72);
    columns->setSectionResizeMode(1, QHeaderView::Stretch);
    route_options_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    route_options_->setSelectionBehavior(QAbstractItemView::SelectRows);
    route_options_->setSelectionMode(QAbstractItemView::SingleSelection);
    route_options_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    route_options_->setFixedHeight(128);
    route_summary_ = new QLabel;
    route_summary_->setTextFormat(Qt::PlainText);
    route_summary_->setWordWrap(true);
    route_summary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ik_duration_ = add_duration_field(layout);
    preview_route_ = new QPushButton(tr("Previsualizar ruta"));
    use_route_ = new QPushButton(tr("Aplicar en simulación"));
    use_route_->setObjectName("executeIkRoute");
    ik_arm_ = new QPushButton;
    ik_arm_->setObjectName("authorizeIkRoute");
    ik_arm_->hide();
    connect(ik_arm_, &QPushButton::clicked, this, [this]() {
      request_arm(true);
    });
    layout->addWidget(route_options_);
    auto * actions = new QHBoxLayout;
    actions->addWidget(preview_route_);
    actions->addWidget(use_route_);
    layout->addLayout(actions);
    copy_route_ = new QPushButton(tr("Copiar rama al objetivo articular"));
    copy_route_->setObjectName("copyIkJointGoal");
    copy_route_->setEnabled(false);
    layout->addWidget(copy_route_);
    connect(copy_route_, &QPushButton::clicked, this, [this]() {
      if (copy_route_->isEnabled()) {
        if (const auto q = selected_route_joints()) {
          set_target(*q);
        }
      }
    });
    layout->addWidget(ik_arm_);
    ik_execution_hint_ = new QLabel;
    ik_execution_hint_->setWordWrap(true);
    ik_execution_hint_->setObjectName("ikExecutionHint");
    layout->addWidget(ik_execution_hint_);
    layout->addWidget(route_summary_);
    route_playback_ = new QWidget;
    auto * playback = new QHBoxLayout(route_playback_);
    playback->setContentsMargins(0, 0, 0, 0);
    route_pause_ = new QPushButton(tr("Pausar"));
    route_resume_ = new QPushButton(tr("Continuar"));
    auto * stop = new QPushButton(tr("Cancelar ruta"));
    playback->addWidget(route_pause_);
    playback->addWidget(route_resume_);
    playback->addWidget(stop);
    connect(route_pause_, &QPushButton::clicked, this, [this]() {publish_text(route_preview_, "pause");});
    connect(route_resume_, &QPushButton::clicked, this, [this]() {publish_text(route_preview_, "resume");});
    connect(stop, &QPushButton::clicked, this, [this]() {publish_text(route_preview_, "cancel");});
    route_playback_->hide();
    layout->addWidget(route_playback_);
    connect(preview_route_, &QPushButton::clicked, this, [this]() {
      const QJsonObject selection{
        {"branch", selected_route().value("branch").toString()}, {"request_id", route_id_}};
      publish_text(route_preview_,
        QJsonDocument(selection).toJson(QJsonDocument::Compact).toStdString());
    });
    connect(use_route_, &QPushButton::clicked, this, [this]() {
      if (!use_route_->isEnabled() || !route_current() ||
        !selected_route().value("reason").toString().isEmpty())
      {
        return;
      }
      if (const auto q = selected_route_joints()) {
        const auto frame = QStringLiteral("imt342/ik/%1/%2")
          .arg(route_id_, selected_route().value("branch").toString()).toStdString();
        if (active_mode_ == "offline") {
          publish_joint_goal(apply_target_, *q, frame);
        } else if (active_mode_ == "control" && authorization_matches(true)) {
          publish_joint_goal(execute_target_, *q, frame);
        }
      }
    });
    connect(route_options_, &QTableWidget::itemSelectionChanged, this, [this]() {show_route();});
  }

  QDoubleSpinBox * add_duration_field(QVBoxLayout * layout)
  {
    auto * holder = new QWidget;
    auto * row = new QHBoxLayout(holder);
    row->setContentsMargins(0, 0, 0, 0);
    auto * label = new QLabel(tr("Duración por tramo"));
    auto * field = new QDoubleSpinBox;
    field->setDecimals(1);
    field->setRange(0.0, 30.0);
    field->setSingleStep(0.1);
    field->setSuffix(tr(" s"));
    field->setSpecialValueText(tr("Automático"));
    field->setKeyboardTracking(false);
    field->setToolTip(tr("El perfil y los límites fijan la duración mínima. "
      "RAPID v2 solicita la duración a ABB."));
    label->setBuddy(field);
    row->addWidget(label);
    row->addWidget(field, 1);
    holder->hide();
    layout->addWidget(holder);
    connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double duration) {
      set_duration_fields(duration);
      invalidate_routes(true);
      duration_pending_ = true;
      Q_EMIT configChanged();
    });
    return field;
  }

  void set_duration_fields(double duration)
  {
    const QSignalBlocker fk(fk_duration_);
    const QSignalBlocker ik(ik_duration_);
    fk_duration_->setValue(duration);
    ik_duration_->setValue(duration);
  }

  void refresh_speed()
  {
    // Show the computed peak of the current segment, not the uniform study cap.
    auto nominal = analysis_.value("plan").toObject().value("peak_velocity_rad_s").toArray();
    if (nominal.size() != 6) {
      const auto run = analysis_.value("physical_run").toObject();
      nominal = run.value("reference").toObject().value("peak_velocity_rad_s").toArray();
    }
    const auto measured = analysis_.value("measured_qdot").toArray();
    const auto rate = analysis_.value("feedback_hz");
    const auto source = analysis_.value("source").toString();
    const bool mock = source == "MEDIDO (SIMULADO/MOCK)";
    const bool has_measurement = actual_fresh_ && !analysis_.value("stale").toBool() &&
      (source == "rws" || source == "rws_trajectory" || mock);
    const auto frequency = has_measurement && rate.isDouble() ?
      tr("%1 Hz").arg(rate.toDouble(), 0, 'f', 1) : tr("N/D");
    speed_units_->setText(tr("Velocidad [%1/s]").arg(suffix().trimmed()));
    speed_units_->setToolTip(tr("Adquisición %1 · %2").arg(mock ? tr("mock") : tr("RWS"), frequency));
    for (std::size_t i = 0; i < speed_effective_.size(); ++i) {
      const auto model = nominal.size() == 6 ? nominal.at(i) : QJsonValue();
      const auto estimate = measured.size() == 6 ? measured.at(i) : QJsonValue();
      speed_effective_[i]->setText(model.isDouble() ?
        QString::number(factor() * model.toDouble(), 'f', 2) : tr("N/D"));
      speed_measured_[i]->setText(has_measurement && estimate.isDouble() ?
        QString::number(factor() * estimate.toDouble(), 'f', 2) : tr("N/D"));
    }
  }

  void send_task(const QJsonObject & command)
  {
    publish_text(task_command_, QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString());
  }

  bool request_routes()
  {
    const auto matrix = read_matrix();
    const auto pose = read_pose_matrix(matrix);
    if (!pose) {
      ik_feedback_->setText(tr("Complete las tres filas editables con números finitos."));
      return false;
    }
    switch (check_pose(*pose)) {
      case PoseError::non_finite:
        ik_feedback_->setText(tr("Todos los valores deben ser finitos."));
        return false;
      case PoseError::last_row:
        ik_feedback_->setText(tr("La última fila debe ser [0, 0, 0, 1]."));
        return false;
      case PoseError::orthogonality:
        ik_feedback_->setText(tr("Las columnas del bloque 3×3 deben ser ortogonales y unitarias."));
        return false;
      case PoseError::determinant:
        ik_feedback_->setText(tr("El determinante de rotación debe ser +1. Revise el sentido de los ejes."));
        return false;
      case PoseError::none:
        break;
    }
    invalidate_routes(true);
    ik_feedback_->clear();
    route_id_ = QUuid::createUuid().toString(QUuid::WithoutBraces);
    QJsonObject request;
    request.insert("matrix", matrix);
    request.insert("request_id", route_id_);
    request.insert("point", goal_point_->currentData().toString());
    route_summary_->setText(tr("Calculando ramas…"));
    publish_text(route_request_, QJsonDocument(request).toJson(QJsonDocument::Compact).toStdString());
    return true;
  }

  QJsonObject selected_route() const
  {
    if (route_options_->selectedItems().isEmpty()) {
      return {};
    }
    const auto * item = route_options_->item(route_options_->currentRow(), 1);
    return item ? QJsonObject::fromVariantMap(item->data(Qt::UserRole).toMap()) : QJsonObject{};
  }

  static bool joint_goal_matches(const QJsonValue & packet, const Joints & joints)
  {
    const auto coordinates = packet.toArray();
    if (coordinates.size() != static_cast<int>(joints.size())) {
      return false;
    }
    Joints goal{};
    for (std::size_t i = 0; i < goal.size(); ++i) {
      goal[i] = coordinates[i].toDouble(std::nan(""));
    }
    return same_joint_origin(goal, joints, 1e-8);
  }

  bool fk_preview_matches() const
  {
    const auto duration = analysis_.value("trajectory_duration_s").toDouble(0.0);
    return analysis_.value("physical_fk_reviewed").toBool() && !profile_pending_ &&
      !duration_pending_ &&
      analysis_.value("trajectory_profile").toString() == profile_->currentData().toString() &&
      std::abs(duration - fk_duration_->value()) <= 1e-8 &&
      joint_goal_matches(analysis_.value("target"), target_);
  }

  bool ik_preview_matches() const
  {
    const auto review = analysis_.value("physical_ik_review").toObject();
    const auto branch = selected_route().value("branch").toString();
    return route_current() && !branch.isEmpty() &&
      review.value("route_id").toString() == route_id_ &&
      review.value("branch").toString() == branch;
  }

  bool authorization_matches(bool ik) const
  {
    if (!analysis_.value("physical_armed").toBool()) {
      return false;
    }
    const auto authorization = analysis_.value("physical_authorization").toObject();
    if (!ik) {
      return authorization.value("kind").toString() == "fk" && fk_preview_matches() &&
        joint_goal_matches(authorization.value("target_q_rad"), target_);
    }
    const auto q = selected_route_joints();
    return authorization.value("kind").toString() == "ik" && q && ik_preview_matches() &&
      authorization.value("route_id").toString() == route_id_ &&
      authorization.value("branch") == selected_route().value("branch") &&
      joint_goal_matches(authorization.value("target_q_rad"), *q);
  }

  bool route_current() const
  {
    if (route_id_.isEmpty() || route_id_ != analysis_.value("pose_plan_id").toString() ||
      !actual_ || !actual_fresh_ || !live_analysis() || analysis_.value("stale").toBool() ||
      route_origin_.size() != 6)
    {
      return false;
    }
    Joints origin{};
    for (std::size_t i = 0; i < origin.size(); ++i) {
      origin[i] = route_origin_[i].toDouble(std::nan(""));
    }
    return same_joint_origin(origin, *actual_,
      analysis_.value("plan_origin_tolerance_rad").toDouble(1e-4));
  }

  std::optional<Joints> selected_route_joints() const
  {
    const auto angles = selected_route().value("q").toArray();
    if (angles.size() != 6) {
      return {};
    }
    Joints result{};
    for (int i = 0; i < angles.size(); ++i) {
      const double angle = angles[i].toDouble(std::nan(""));
      if (!std::isfinite(angle) || angle < imt342_irb120_rviz::lower[i] || angle > upper[i]) {
        return {};
      }
      result[i] = angle;
    }
    return result;
  }

  void show_route()
  {
    const auto route = selected_route();
    if (route.isEmpty()) {
      if (route_options_->rowCount() > 0) {
        route_summary_->setText(tr("Seleccione explícitamente una rama para inspeccionarla."));
      }
      return;
    }
    QStringList angles;
    if (const auto q = selected_route_joints()) {
      for (const auto radians : *q) {
        angles << QString::number(radians * factor(), 'f', 3);
      }
    }
    const auto reason = route.value("reason").toString();
    const auto duration = route.value("duration").toDouble();
    QStringList lines;
    if (!angles.isEmpty()) {
      lines << tr("J1–J6%1 · %2").arg(suffix(), angles.join(", "));
    }
    lines << tr("Error FK %1 m · %2%3")
      .arg(route.value("position_error_m").toDouble(), 0, 'g', 3)
      .arg(route.value("orientation_error_rad").toDouble() * factor(), 0, 'g', 3).arg(suffix());
    lines << (route.value("singular").toBool() ? tr("Configuración singular") :
      route.value("min_sigma").isDouble() ?
      tr("σ mínima %1").arg(route.value("min_sigma").toDouble(), 0, 'g', 3) : tr("Singularidad N/D"));
    lines << (reason.isEmpty() && duration > 0 ? tr("Dentro de límites") :
      tr("Ruta no disponible · %1").arg(reason));
    lines << tr("Colisiones no verificadas");
    route_summary_->setText(lines.join('\n'));
  }

  void receive_routes(const std::string & message)
  {
    const auto packet = QJsonDocument::fromJson(QByteArray::fromStdString(message)).object();
    const auto identifier = packet.value("request_id").toString();
    if (route_id_.isEmpty()) {
      return;
    }
    if (identifier.isEmpty() && route_options_->rowCount() > 0) {
      invalidate_routes();
      route_summary_->setText(packet.value("error").toString(tr("Ruta obsoleta · Vuelva a calcular.")));
      return;
    }
    if (identifier != route_id_) {
      return;
    }
    route_options_->setRowCount(0);
    route_origin_ = packet.value("origin").toObject().value("q").toArray();
    const auto options = packet.value("routes").toArray();
    for (const auto & item : options) {
      const auto route = item.toObject();
      const auto branch = route.value("branch").toString();
      const auto reason = route.value("reason").toString();
      const bool planned = reason.isEmpty() && route.value("duration").isDouble();
      const int row = route_options_->rowCount();
      route_options_->insertRow(row);
      const auto shoulder = branch.section('/', 0, 0);
      const auto wrist = branch.section('/', 2, 2);
      const auto configuration = tr("%1 · %2 · %3").arg(
        shoulder == "H+" ? tr("Hombro 1") : shoulder == "H-" ? tr("Hombro 2") :
        shoulder == "H0-ref" ? tr("Hombro ref.") : tr("Hombro singular"),
        branch.section('/', 1, 1) == "C+" ? tr("Codo 1") : tr("Codo 2"),
        wrist == "M+" ? tr("Muñeca 1") : wrist == "M-" ? tr("Muñeca 2") : tr("Muñeca singular"));
      const QStringList values{
        QString::number(row + 1),
        configuration,
        planned ? QString::number(route.value("duration").toDouble(), 'f', 3) : QStringLiteral("—"),
        planned ? tr("Válida") : tr("Inválida")};
      for (int column = 0; column < values.size(); ++column) {
        auto * cell = new QTableWidgetItem(values[column]);
        cell->setToolTip(branch_label(branch) + "\n" + branch + (reason.isEmpty() ? QString() : "\n" + reason));
        if (column == 1) {
          cell->setData(Qt::UserRole, route.toVariantMap());
        } else {
          cell->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        }
        route_options_->setItem(row, column, cell);
      }
    }
    route_options_->resizeRowsToContents();
    if (!options.isEmpty()) {
      route_summary_->setText(tr("Seleccione explícitamente una rama para inspeccionarla."));
    }
    if (options.isEmpty()) {
      const auto error = packet.value("error").toString();
      route_summary_->setText(error.isEmpty() ? tr("Sin ramas válidas para esta pose") : error);
    }
  }

  double factor() const
  {
    return unit_->currentIndex() == 1 ? 1.0 : deg_per_rad;
  }

  QString suffix() const
  {
    return unit_->currentIndex() == 1 ? tr(" rad") : tr(" °");
  }

  void refresh_units()
  {
    const int decimals = unit_->currentIndex() == 1 ? 6 : 5;
    for (std::size_t axis = 0; axis < joint_names.size(); ++axis) {
      const QSignalBlocker edit_blocker(edits_[axis]);
      const QSignalBlocker slider_blocker(sliders_[axis]);
      edits_[axis]->setDecimals(decimals);
      edits_[axis]->setRange(imt342_irb120_rviz::lower[axis] * factor(), upper[axis] * factor());
      edits_[axis]->setSuffix(suffix());
      edits_[axis]->setSingleStep(unit_->currentIndex() == 1 ? 0.01 : 1.0);
      edits_[axis]->setValue(target_[axis] * factor());
      sliders_[axis]->setValue(slider_value(axis, target_[axis]));
    }
    QWidget * host = this;
    while (host->parentWidget()) {
      host = host->parentWidget();
    }
    host->setProperty("imt342_radians", unit_->currentIndex() == 1);
    show_route();
    refresh_state();
  }

  void set_target(const Joints & target)
  {
    for (std::size_t axis = 0; axis < target.size(); ++axis) {
      target_[axis] = std::clamp(target[axis], imt342_irb120_rviz::lower[axis], upper[axis]);
    }
    refresh_units();
    publish_target();
  }

  void publish_target()
  {
    publish_target(target_pub_);
  }

  void publish_target(const rclcpp::Publisher<State>::SharedPtr & publisher)
  {
    publish_joint_goal(publisher, target_, "base_link");
  }

  void publish_joint_goal(
    const rclcpp::Publisher<State>::SharedPtr & publisher, const Joints & goal,
    const std::string & frame)
  {
    if (publisher) {
      State msg;
      msg.header.stamp = node_->now();
      msg.header.frame_id = frame;
      msg.name.assign(joint_names.begin(), joint_names.end());
      msg.position.assign(goal.begin(), goal.end());
      publisher->publish(msg);
    }
  }

  static void publish_text(const rclcpp::Publisher<Text>::SharedPtr & publisher,
    const std::string & data)
  {
    if (publisher) {
      Text msg;
      msg.data = data;
      publisher->publish(msg);
    }
  }

  void build_diagnostics(QVBoxLayout * layout)
  {
    auto title = [layout](const QString & text) {
        auto * label = new QLabel(text);
        auto font = label->font();
        font.setBold(true);
        label->setFont(font);
        layout->addWidget(label);
      };
    title(tr("Estado general"));
    diagnostics_ = new QLabel;
    diagnostics_->setTextFormat(Qt::PlainText);
    diagnostics_->setWordWrap(true);
    layout->addWidget(diagnostics_);
    title(tr("Preparación y alertas"));
    diagnosis_error_ = new QLabel;
    diagnosis_error_->setTextFormat(Qt::PlainText);
    diagnosis_error_->setWordWrap(true);
    layout->addWidget(diagnosis_error_);
    auto * detail = new QToolButton;
    detail->setText(tr("Detalle técnico"));
    detail->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    detail->setCheckable(true);
    detail->setArrowType(Qt::RightArrow);
    layout->addWidget(detail, 0, Qt::AlignLeft);
    diagnosis_detail_ = new QPlainTextEdit;
    diagnosis_detail_->setReadOnly(true);
    diagnosis_detail_->hide();
    layout->addWidget(diagnosis_detail_);
    connect(detail, &QToolButton::toggled, this, [this, detail](bool visible) {
      detail->setArrowType(visible ? Qt::DownArrow : Qt::RightArrow);
      diagnosis_detail_->setVisible(visible);
    });
  }

  bool live_analysis() const
  {
    return last_analysis_ != Clock::time_point{} &&
           Clock::now() - last_analysis_ < std::chrono::seconds(2);
  }

  bool trajectory_selected() const
  {
    return backend_->currentData().toString() == "rws_trajectory";
  }

  bool trajectory_configured() const
  {
    return live_analysis() && analysis_.value("rws_trajectory_configured").toBool();
  }

  bool trajectory_motion_enabled() const
  {
    return trajectory_configured() && analysis_.value("rws_trajectory_motion_enabled").toBool();
  }

  static void enable_option(QComboBox * combo, const char * value, bool enabled)
  {
    auto * model = qobject_cast<QStandardItemModel *>(combo->model());
    auto * item = model ? model->item(combo->findData(QString::fromLatin1(value))) : nullptr;
    if (item) {
      item->setEnabled(enabled);
    }
  }

  void update_transport_choices()
  {
    const bool offline = active_mode_ == "offline";
    const bool trajectory = trajectory_selected();
    enable_option(backend_, "rws_trajectory", offline || trajectory_configured());
    enable_option(mode_, "observe", !trajectory || trajectory_configured());
    // Mode selection alone does not arm or send a physical command.
    enable_option(mode_, "control", !trajectory || trajectory_configured());
    route_options_->horizontalHeaderItem(2)->setText(tr("Ref. [s]"));
    fk_duration_->parentWidget()->setVisible(offline);
    ik_duration_->parentWidget()->setVisible(offline);
    if (!offline && fk_duration_->value() != 0.0) {
      set_duration_fields(0.0);
      duration_pending_ = true;
    }
    profile_->setToolTip(tr("LSPB: aceleración, crucero y frenado; perfil predeterminado. "
      "Quíntica: referencia polinómica suave opcional. ABB interpola el movimiento RWS."));
    const auto transport = analysis_.value("transport").toObject();
    const bool normal_known = live_analysis() && transport.value("protocol_version").isDouble();
    profile_transport_->setText(offline ? tr("Plan y movimiento simulados") :
      trajectory ? tr("RAPID v2 · duración automática solicitada a ABB") :
      normal_known ? tr("RAPID Normal legado · destino con velocidad RAPID v10") :
      tr("RAPID Normal · destino articular e interpolación ABB"));
  }

  static QString seconds(const QJsonValue & value)
  {
    return value.isDouble() && std::isfinite(value.toDouble()) && value.toDouble() >= 0 ?
      tr("%1 s").arg(value.toDouble(), 0, 'f', 3) : tr("N/D");
  }

  QStringList trajectory_diagnostics(
    const QJsonObject & transport, const QString & mode) const
  {
    QStringList messages;
    messages << tr("Trayectorias v2 · un destino por orden RAPID · ABB planifica MoveAbsJ con \\T.");
    if (!trajectory_configured()) {
      messages << tr("Para consultar el IRC5, abre Solo lectura desde el lanzador IMT-342.");
    } else if (mode != "offline") {
      const auto status = transport.value("trajectory_status").toString();
      if (!status.isEmpty()) {
        messages << status;
      }
      const auto pointer = transport.value("pointer").toArray();
      if (transport.value("execution").toString() != "running" ||
        pointer.size() != 2 || pointer[0].toString() != "Imt342TrajectoryRapid")
      {
        messages << tr("Preparación requerida · Imt342TrajectoryRapid/ServeTrajectoryOrders.");
      }
      if (!trajectory_motion_enabled()) {
        messages << tr("Para habilitar una ejecución, prepara el servidor en esta sesión.");
      }
    }
    return messages;
  }

  QStringList trial_diagnostics(const QJsonObject & trial) const
  {
    QStringList messages;
    if (trial.value("backend") == analysis_.value("backend") &&
      !trial.value("trial_id").toString().isEmpty())
    {
      const auto motion = trial.value("motion").toObject();
      const auto requested = trial.value("requested_duration_s").isDouble() ?
        trial.value("requested_duration_s") : motion.value("requested_duration_s");
      const auto profile = trial.value("reference_profile").toString();
      const auto name = profile == "quintic" ? tr("Quíntica") : profile == "lspb" ? tr("LSPB") : tr("Plan");
      messages << tr("Referencia %1 · %2 · duración solicitada %3")
        .arg(name, seconds(trial.value("reference_duration_s")), seconds(requested));
      messages << tr("Tiempo observado RWS, envío a respuesta terminal · %1")
        .arg(seconds(motion.value("submit_to_terminal_reply_s")));
      const auto error = motion.value("final_error_rad");
      if (error.isDouble()) {
        messages << tr("Error articular máximo final · %1 rad").arg(error.toDouble(), 0, 'g', 4);
      }
    }
    return messages;
  }

  void update_connection_indicator()
  {
    const auto request = analysis_.value("irc5_connection").toObject();
    const auto state = live_analysis() ? request.value("state").toString() : QString();
    const bool connecting = state == "connecting" || state == "preparing" || state == "verifying";
    const bool clicked = connection_clicked_ != Clock::time_point{} &&
      Clock::now() - connection_clicked_ < std::chrono::seconds(1);
    connect_irc5_->setEnabled(!connecting && !clicked);
    connect_irc5_->setText(state == "module_missing" ? tr("Recuperar servidor RAPID…") :
      tr("Conectar y preparar IRC5"));
    connect_irc5_->setToolTip(tr("Comprueba RWS y prepara el servidor RAPID si procede. "
      "La habilitación para mover se realiza por separado."));
    actual_source_->setToolTip(QString());
    if (!connection_hint_.isEmpty() && !live_analysis()) {
      actual_source_->setText(connection_hint_);
    } else if (state == "error" || state == "module_missing" ||
      state == "module_modified" || state == "foreign_program")
    {
      actual_source_->setText(tr("Error: %1").arg(request.value("detail").toString()));
    } else if (state == "preparing" || state == "verifying") {
      actual_source_->setText(tr("IRC5 conectado · preparando RAPID…"));
    } else if (state == "ready" && actual_fresh_ && !analysis_.value("stale").toBool()) {
      const auto transport = analysis_.value("transport").toObject();
      const auto arrival = transport.value("motion_evidence").toObject();
      const auto outcome = transport.value("result").toString();
      if (arrival.value("within_arrival_band").toBool() &&
        arrival.value("awaiting_terminal_reply").toBool())
      {
        actual_source_->setText(tr("Dentro de tolerancia · esperando acuse RAPID"));
      } else if (outcome == "executing" || outcome == "accepted" || outcome == "transmitted") {
        actual_source_->setText(tr("Orden física en curso · esperando confirmación"));
      } else if (outcome == "unknown") {
        actual_source_->setText(tr("Resultado físico sin confirmar · consulte Diagnóstico"));
      } else if (transport.value("trajectory_status").toString().contains("esperando retorno")) {
        actual_source_->setText(transport.value("trajectory_status").toString());
      } else {
        actual_source_->setText(tr("Listo para operar · telemetría reciente · RAPID en marcha"));
        actual_source_->setToolTip(tr("La orden física requiere su habilitación individual."));
      }
    } else if (connecting || clicked) {
      actual_source_->setText(tr("Conectando con IRC5…"));
    } else if (state == "connected" && actual_fresh_) {
      const auto rapid = analysis_.value("transport").toObject().value("execution").toString(tr("sin datos"));
      actual_source_->setText(trajectory_selected() && !trajectory_motion_enabled() ?
        tr("IRC5 conectado · pulsa Conectar y preparar IRC5 para habilitar v2") :
        tr("IRC5 conectado · J1–J6 recientes · RAPID %1").arg(rapid));
    }
  }

  void refresh_diagnostics()
  {
    const bool trajectory = trajectory_selected();
    const bool live = live_analysis() &&
      (!trajectory || analysis_.value("backend").toString() == "rws_trajectory");
    const auto transport = live ? analysis_.value("transport").toObject() : QJsonObject{};
    const auto request = live ? analysis_.value("irc5_connection").toObject() : QJsonObject{};
    const auto diagnostic = live ? analysis_.value("diagnostic").toObject() : QJsonObject{};
    const auto prep = live ?
      analysis_.value("rapid_preparation").toObject() : QJsonObject{};
    const auto trial = live ? analysis_.value("individual_trial").toObject() : QJsonObject{};
    const auto outcome = transport.value("result").toString();
    const auto last_result = trajectory && trial.value("backend").toString() == "rws_trajectory" ?
      trial.value("result").toString(outcome) : outcome;
    const auto state = request.value("state").toString();
    const auto latest = last_result == "verified" ? tr("Verificado") : last_result == "failed" ?
      tr("Fallido") : last_result == "unknown" ? tr("Desconocido") :
      last_result == "cancelled" ? tr("Cancelado") : tr("Sin datos");
    const auto motion = outcome == "executing" || outcome == "accepted" || outcome == "transmitted" ?
      tr("Ejecutando") : outcome == "failed" || outcome == "unknown" ? tr("Error") : tr("Inactivo");
    const bool rws = live && (analysis_.value("backend").toString() == "rws" || trajectory) &&
      analysis_.value("backend_connected").toBool() && actual_fresh_;
    const auto feedback = live && analysis_.value("mode").toString() == "offline" ? tr("Simulados") :
      live && analysis_.value("stale").toBool() ? tr("Vencidos") :
      live && actual_fresh_ ? tr("Recientes") : tr("Sin datos");
    const auto mode = analysis_.value("mode").toString();
    const auto execution = transport.value("execution").toString();
    const auto rapid = execution == "running" ? tr("En marcha") :
      execution == "stopped" ? tr("Detenido") : tr("Sin datos");
    const auto operational = !live ? tr("Sesión sin comunicación") :
      mode == "offline" ? tr("Simulación Offline") :
      mode == "observe" && actual_fresh_ ? tr("Listo para observar · solo lectura") :
      mode == "control" && rws && (trajectory ? analysis_.value("physical_can_arm").toBool() :
      state == "ready") ? tr("Control disponible · requiere habilitación individual") :
      mode == "observe" ? tr("Solo lectura · esperando telemetría") : tr("Control físico · compruebe el estado");
    diagnostics_->setText(tr("%1\nROS 2 · %2  |  IRC5 · %3\nTelemetría J1–J6 · %4\n"
      "RAPID · %5\nMovimiento · %6  |  Último resultado · %7")
      .arg(operational, live ? tr("Conectado") : tr("Desconectado"),
        rws ? tr("Conectado") : tr("Desconectado"), feedback, rapid, motion, latest));
    QStringList messages;
    if (trajectory) {
      messages << trajectory_diagnostics(transport, mode);
    } else if (mode != "offline" && transport.value("protocol_version").isDouble()) {
      messages << tr("Normal original · el servidor RAPID aplica v10 al destino.");
    }
    messages << trial_diagnostics(trial);
    const auto arrival = transport.value("motion_evidence").toObject();
    if (arrival.value("latest_goal_error_rad").isDouble() &&
      arrival.value("arrival_tolerance_rad").isDouble())
    {
      messages << (arrival.value("arrival_frozen").toBool() ?
        tr("Error al cierre %1° · banda ±%2° por eje") : tr("Error actual %1° · banda ±%2° por eje"))
        .arg(arrival.value("latest_goal_error_rad").toDouble() * deg_per_rad, 0, 'f', 3)
        .arg(arrival.value("arrival_tolerance_rad").toDouble() * deg_per_rad, 0, 'f', 2);
      if (arrival.value("observed_band_to_terminal_s").isDouble()) {
        messages << tr("Desde entrada observada en banda hasta acuse: %1 s")
          .arg(arrival.value("observed_band_to_terminal_s").toDouble(), 0, 'f', 3);
      }
      if (arrival.value("within_arrival_band").toBool() &&
        arrival.value("awaiting_terminal_reply").toBool())
      {
        messages << tr("Dentro de tolerancia · esperando acuse RAPID; no se envía otra orden.");
      }
    }
    if (state == "error" || state == "module_missing" ||
      state == "module_modified" || state == "foreign_program")
    {
      messages << request.value("detail").toString();
    } else if (state == "ready" && rws) {
      messages << tr("Servidor propio preparado");
    } else if (state == "preparing" || state == "verifying" || state == "connecting") {
      messages << request.value("detail").toString();
    } else {
      messages << tr("Preparación del servidor sin confirmar");
    }
    if (transport.value("http_status").toInt() == 401) {
      messages << tr("Falta autenticación RWS. Revise la cuenta del llavero.");
    }
    if (diagnostic.value("severity").toString() == "ERROR" ||
      diagnostic.value("severity").toString() == "STALE")
    {
      messages << diagnostic.value("detail").toString() << diagnostic.value("action").toString();
    }
    diagnosis_error_->setText(messages.join('\n').trimmed());
    QJsonObject details;
    for (const auto & key : {"mode", "motors", "execution", "pointer", "protocol_version",
        "order_sequence",
        "reply", "http_status", "motion_evidence", "trajectory_status", "module_sha256", "active_tasks"})
    {
      if (transport.contains(key)) {
        details.insert(key, transport.value(key));
      }
    }
    QJsonObject inspected{{"Controlador", details},
      {"Edad del feedback [s]", live ? analysis_.value("feedback_age_s") : QJsonValue()},
      {"Frecuencia [Hz]", live ? analysis_.value("feedback_hz") : QJsonValue()},
      {"TCP", live ? analysis_.value("tool_source") : QJsonValue()}};
    inspected.insert("Preparación", prep);
    const auto text = QString::fromUtf8(QJsonDocument(inspected).toJson());
    if (diagnosis_detail_->toPlainText() != text) {
      diagnosis_detail_->setPlainText(text);
    }
  }

  void refresh_state()
  {
    Clock::time_point received;
    std::string mode;
    std::string backend;
    std::string analysis;
    std::string actual_pose;
    std::string route_result;
    {
      std::lock_guard<std::mutex> lock(inbox_->mutex);
      actual_ = inbox_->actual;
      received = inbox_->received;
      mode = inbox_->mode;
      backend = inbox_->backend;
      analysis.swap(inbox_->analysis);
      actual_pose = inbox_->actual_pose;
      route_result.swap(inbox_->route_result);

    }
    if (!route_result.empty()) {
      receive_routes(route_result);
    }
    actual_pose_ = QJsonDocument::fromJson(QByteArray::fromStdString(actual_pose)).object();
    if (!analysis.empty()) {
      receive_analysis(QJsonDocument::fromJson(QByteArray::fromStdString(analysis)).object());
    }
    const auto source = actual_pose_.value("source").toString();
    const auto feedback_source = analysis_.value("source").toString();
    const bool fresh = actual_ && Clock::now() - received <= std::chrono::seconds(1);
    actual_fresh_ = fresh;
    const bool planner_ready = route_request_ && route_request_->get_subscription_count() > 0;
    plan_from_tcp_->setEnabled(planner_ready);
    ik_feedback_->setVisible(!ik_feedback_->text().isEmpty());
    const auto route = selected_route();
    const bool order_pending = live_analysis() && analysis_.value("physical_order_pending").toBool();
    preview_route_->setEnabled(!order_pending &&
      route_preview_ && route_preview_->get_subscription_count() > 0 &&
      route_current() && !route.isEmpty() && route.value("reason").toString().isEmpty() &&
      route.value("duration").toDouble() > 0);
    copy_route_->setEnabled(route_current() && selected_route_joints().has_value() &&
      route.value("reason").toString().isEmpty());

    if (duration_pending_ && analysis_command_ && analysis_command_->get_subscription_count() > 0) {
      const auto duration = fk_duration_->value();
      publish_analysis(QJsonObject{{"trajectory_duration_s",
        duration > 0.0 ? QJsonValue(duration) : QJsonValue(QJsonValue::Null)}});
      duration_pending_ = false;
    }
    if (profile_pending_ && analysis_command_ && analysis_command_->get_subscription_count() > 0) {
      publish_analysis(QJsonObject{{"trajectory_profile", preferred_profile_}});
      profile_pending_ = false;
    } else if (preferred_profile_.isEmpty() && live_analysis()) {
      const int index = profile_->findData(analysis_.value("trajectory_profile").toString());
      if (index >= 0) {
        const QSignalBlocker blocker(profile_);
        profile_->setCurrentIndex(index);
      }
    }
    QJsonArray joints_now;
    if (actual_) {
      for (const auto q : *actual_) {
        joints_now.append(q);
      }
    }
    task_editor_->refresh(analysis_, QString::fromStdString(mode),
      task_command_ && task_command_->get_subscription_count() > 0, joints_now, fresh,
      unit_->currentIndex() == 1);
    refresh_speed();
    active_mode_ = mode;
    refresh_fk();
    const int mode_index = mode_->findData(QString::fromStdString(mode));
    if (mode_index >= 0 && mode_index != mode_->currentIndex()) {
      const QSignalBlocker blocker(mode_);
      mode_->setCurrentIndex(mode_index);
    }
    const int backend_index = backend_->findData(QString::fromStdString(backend));
    if (backend_index >= 0 && backend_index != backend_->currentIndex()) {
      const QSignalBlocker blocker(backend_);
      backend_->setCurrentIndex(backend_index);
    }
    if (offline_preference_pending_ && active_mode_ == "offline" && live_analysis() &&
      backend_command_ && backend_command_->get_subscription_count() > 0)
    {
      offline_preference_pending_ = false;
      if (preferred_backend_ != backend_->currentData().toString()) {
        publish_text(backend_command_, preferred_backend_.toStdString());
      }
    }
    update_transport_choices();
    refresh_diagnostics();
    const bool connected = target_pub_ && target_pub_->get_subscription_count() > 0;
    if (connected && !connected_) {
      publish_target();
    }
    connected_ = connected;
    const bool control = active_mode_ == "control";
    const bool offline = active_mode_ == "offline";
    const bool arm_state_current = control && live_analysis() &&
      analysis_.value("mode").toString() == QStringLiteral("control") &&
      analysis_.value("backend").toString() == QString::fromStdString(backend);
    const bool arm_reported = arm_state_current && analysis_.value("physical_armed").toBool();
    const bool motion_enabled = !trajectory_selected() || trajectory_motion_enabled();
    const bool motion_available = arm_state_current && motion_enabled && fresh && !order_pending &&
      !analysis_.value("stale").toBool();
    const bool can_authorize = motion_available && analysis_.value("physical_can_arm").toBool();
    const bool arm_connected = physical_command_ && physical_command_->get_subscription_count() > 0;
    const bool fk_reviewed = fk_preview_matches();
    const bool valid_ik = route_current() && !route.isEmpty() &&
      route.value("reason").toString().isEmpty() && selected_route_joints().has_value();
    const bool ik_reviewed = valid_ik && ik_preview_matches();
    const bool apply_connected = apply_target_ && apply_target_->get_subscription_count() > 0;
    const bool execute_connected = execute_target_ && execute_target_->get_subscription_count() > 0;
    const bool fk_authorized = arm_reported && authorization_matches(false);
    const bool ik_authorized = arm_reported && authorization_matches(true);
    arm_->setVisible(control);
    arm_->setText(arm_reported ? tr("Revocar habilitación") : tr("Habilitar una ejecución física…"));
    arm_->setEnabled(arm_connected && (arm_reported || (can_authorize && fk_reviewed)));
    ik_arm_->setVisible(control);
    ik_arm_->setText(arm_->text());
    ik_arm_->setEnabled(arm_connected && (arm_reported || (can_authorize && ik_reviewed)));
    use_route_->setText(offline ? tr("Aplicar en simulación") : tr("Ejecutar ruta IK en el robot"));
    use_route_->setEnabled(valid_ik && !order_pending &&
      ((offline && apply_connected) ||
      (motion_available && execute_connected && ik_authorized)));
    apply_->setVisible(offline || control);
    apply_->setText(offline ? tr("Aplicar en simulación") : tr("Ejecutar en el robot"));
    apply_->setEnabled(!order_pending &&
      ((offline && apply_connected) ||
      (motion_available && execute_connected && fk_authorized)));

    QString block = analysis_.value("physical_block_reason").toString();
    if (!arm_state_current || !arm_connected) {
      block = tr("Esperando el estado vigente del servidor seleccionado.");
    } else if (order_pending) {
      block = tr("Hay una orden física en curso. El resultado se actualizará al terminar.");
    } else if (!fresh || analysis_.value("stale").toBool()) {
      block = tr("Esperando una lectura reciente de J1–J6.");
    } else if (!can_authorize && block.isEmpty() && !arm_reported) {
      block = tr("Comprueba el estado del IRC5 en Diagnóstico.");
    }
    const auto execution_hint = [this, &block, arm_reported, motion_available, execute_connected](
      bool reviewed, bool authorized, const QString & preview_hint) {
        if (authorized && motion_available && execute_connected) {
          return tr("Destino habilitado para una ejecución.");
        }
        if (arm_reported && !authorized) {
          return tr("La habilitación corresponde a otro destino. Revócala para cambiar de objetivo.");
        }
        if (!block.isEmpty()) {
          return block;
        }
        return reviewed ? tr("Habilita una ejecución física para este destino.") : preview_hint;
      };
    fk_execution_hint_->setVisible(control);
    fk_execution_hint_->setText(execution_hint(fk_reviewed, fk_authorized,
      tr("Previsualiza el destino vigente antes de habilitar la ejecución.")));
    ik_execution_hint_->setText(active_mode_ == "observe" ?
      tr("Solo lectura: selecciona Control físico para habilitar y ejecutar directamente desde IK.") :
      !valid_ik ? tr("Calcula IK y selecciona una rama vigente.") :
      offline ? tr("Selecciona Aplicar en simulación para recorrer la rama.") :
      execution_hint(ik_reviewed, ik_authorized,
      tr("Previsualiza la rama seleccionada antes de habilitar su ejecución.")));
    backend_->setEnabled(!order_pending &&
      backend_command_ && backend_command_->get_subscription_count() > 0);
    profile_->setEnabled(!order_pending);
    preview_->setEnabled(!order_pending &&
      preview_target_ && preview_target_->get_subscription_count() > 0);
    const auto plan = analysis_.value("plan").toObject();
    const bool preview_available = !plan.isEmpty() && live_analysis();
    cancel_->setEnabled(!order_pending && preview_available &&
      route_preview_ && route_preview_->get_subscription_count() > 0);
    stop_motion_->setVisible(control);
    stop_motion_->setEnabled(order_pending &&
      preview_command_ && preview_command_->get_subscription_count() > 0);
    use_actual_->setEnabled(fresh);
    mode_->setEnabled(!order_pending && mode_command_ && mode_command_->get_subscription_count() > 0);
    const bool source_confirmed = mode == "offline" ?
      source == "virtual" : source.startsWith(QStringLiteral("medida"));
    const bool packet_current = analysis_.value("mode").toString() == QString::fromStdString(mode) &&
      analysis_.value("backend").toString() == QString::fromStdString(backend);
    if (actual_ && (!fresh || analysis_.value("stale").toBool() ||
      source.startsWith(QStringLiteral("medida vencida"))))
    {
      actual_source_->setText(tr("Telemetría vencida · última postura sin actualizar"));
    } else if (!actual_ || !source_confirmed || !packet_current) {
      actual_source_->setText(tr("N/D · postura actual pendiente"));
    } else if (mode == "offline") {
      actual_source_->setText(tr("SIMULACIÓN · postura virtual"));
    } else if ((feedback_source == "rws" || feedback_source == "rws_trajectory") &&
      analysis_.value("backend_connected").toBool())
    {
      actual_source_->setText((feedback_source == "rws_trajectory" ?
        tr("RWS v2 · postura física reciente (%1 s)") : tr("RWS · postura física reciente (%1 s)"))
        .arg(analysis_.value("feedback_age_s").toDouble(), 0, 'f', 2));
    } else if (feedback_source == QStringLiteral("MEDIDO (SIMULADO/MOCK)")) {
      actual_source_->setText(tr("SIMULACRO · telemetría de prueba"));
    } else if (feedback_source == QStringLiteral("ROS externo")) {
      actual_source_->setText(tr("ROS externo · postura recibida"));
    } else {
      actual_source_->setText(tr("Sin confirmación de feedback vigente"));
    }
    for (std::size_t axis = 0; axis < actual_labels_.size(); ++axis) {
      const auto bounds = tr("J%1 · %2 a %3%4").arg(axis + 1)
        .arg(imt342_irb120_rviz::lower[axis] * factor(), 0, 'f', 3).arg(upper[axis] * factor(), 0, 'f', 3).arg(suffix());
      edits_[axis]->setToolTip(bounds);
      sliders_[axis]->setToolTip(bounds);
      actual_labels_[axis]->setText(fresh && source_confirmed && packet_current ?
        QString::number((*actual_)[axis] * factor(), 'f', 2) : tr("N/D"));
    }
    update_connection_indicator();
    const auto arrival = packet_current && fresh && !analysis_.value("stale").toBool() ?
      analysis_.value("arrival").toObject() : QJsonObject{};
    const double band = analysis_.value("arrival_tolerance_rad").toDouble(0.005235987755982989);
    const auto limit = tr("±%1%2").arg(band * factor(), 0, 'f', unit_->currentIndex() == 1 ? 5 : 2).arg(suffix());
    if (arrival.value("max_error_rad").isDouble()) {
      const auto origin = arrival.value("source").toString() == "last_rws_order" ?
        tr("Última consigna RWS") : tr("Objetivo seleccionado");
      arrival_status_->setText(tr("%1 · error máx. %2%3 · banda %4 · %5%6")
        .arg(origin).arg(arrival.value("max_error_rad").toDouble() * factor(), 0, 'f', 5)
        .arg(suffix()).arg(limit)
        .arg(arrival.value("within_band").toBool() ? tr("Dentro") : tr("Fuera"))
        .arg(arrival.value("awaiting_terminal_reply").toBool() ? tr(" · esperando acuse RAPID") : QString()));
    } else {
      arrival_status_->setText(tr("Banda de posición por eje: %1 · sin medida comparable").arg(limit));
    }
    const auto operation = analysis_.value("status").toString();
    if (!operation.isEmpty() && packet_current) {
      status_->setText(operation);
      status_->setVisible(active_mode_ != "offline");
    } else {
      status_->hide();
    }
  }

  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<State>::SharedPtr target_pub_;
  rclcpp::Publisher<State>::SharedPtr preview_target_;
  rclcpp::Publisher<State>::SharedPtr apply_target_;
  rclcpp::Publisher<State>::SharedPtr execute_target_;
  rclcpp::Publisher<Text>::SharedPtr mode_command_;
  rclcpp::Publisher<Text>::SharedPtr physical_command_;
  rclcpp::Publisher<Text>::SharedPtr rapid_prepare_;
  rclcpp::Publisher<Text>::SharedPtr task_command_;
  rclcpp::Publisher<Text>::SharedPtr backend_command_;
  rclcpp::Publisher<Text>::SharedPtr preview_command_;
  rclcpp::Subscription<State>::SharedPtr actual_sub_;
  rclcpp::Subscription<Text>::SharedPtr mode_sub_;
  rclcpp::Subscription<Text>::SharedPtr backend_sub_;
  rclcpp::Subscription<Text>::SharedPtr analysis_sub_;
  rclcpp::Publisher<Text>::SharedPtr analysis_command_;
  rclcpp::Publisher<Text>::SharedPtr route_request_;
  rclcpp::Publisher<Text>::SharedPtr route_preview_;
  rclcpp::Subscription<Text>::SharedPtr route_result_sub_;
  rclcpp::Subscription<Text>::SharedPtr actual_pose_sub_;
  QJsonObject actual_pose_;
  QJsonObject analysis_;
  QTabWidget * tabs_;
  QWidget * diagnostics_tab_;
  PoseGrid * matrix_;
  QLabel * ik_feedback_;
  QLabel * ik_execution_hint_{};
  QLabel * fk_execution_hint_{};
  TaskEditor * task_editor_;
  PoseGrid * fk_matrix_;
  QComboBox * fk_point_;
  QJsonArray last_fk_matrix_;
  QPushButton * ik_arm_;
  QLabel * fk_values_;
  std::array<QLabel *, 6> speed_effective_;
  std::array<QLabel *, 6> speed_measured_;
  QComboBox * goal_point_;
  QWidget * route_playback_;
  QPushButton * route_pause_;
  QPushButton * route_resume_;
  QString goal_key_{QStringLiteral("flange")};
  QString route_id_;
  QJsonArray route_origin_;
  QTableWidget * route_options_;
  QLabel * route_summary_;
  QPushButton * plan_from_tcp_;
  QPushButton * preview_route_;
  QPushButton * use_route_;
  QPushButton * copy_route_;

  Joints target_{};
  std::optional<Joints> actual_;
  bool actual_fresh_{};
  std::string active_mode_;
  bool connected_ = false;
  QComboBox * mode_;
  QComboBox * backend_;
  QComboBox * profile_;
  QLabel * profile_transport_;
  QComboBox * unit_;
  std::array<QDoubleSpinBox *, 6> edits_;
  std::array<QSlider *, 6> sliders_;
  std::array<QLabel *, 6> actual_labels_;
  QLabel * actual_source_;
  QLabel * arrival_status_;
  QLabel * status_;
  QDoubleSpinBox * fk_duration_, * ik_duration_;
  QPushButton * connect_irc5_;
  QLabel * speed_units_;
  QLabel * diagnostics_;
  QLabel * diagnosis_error_;
  QPlainTextEdit * diagnosis_detail_;
  Clock::time_point last_analysis_{};
  QString preferred_profile_;
  QString preferred_backend_;
  bool offline_preference_pending_{false};
  bool duration_pending_{false};
  bool profile_pending_{false};
  Clock::time_point connection_clicked_{};
  QString connection_hint_;
  QPushButton * use_actual_;
  QPushButton * preview_;
  QPushButton * cancel_;
  QPushButton * stop_motion_;
  QPushButton * apply_;
  QPushButton * arm_;
};
}  // namespace imt342_irb120_rviz

#ifndef IMT342_PANEL_TEST
PLUGINLIB_EXPORT_CLASS(imt342_irb120_rviz::JointPanel, rviz_common::Panel)
#endif
