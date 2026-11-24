#define IMT342_PANEL_TEST
#include "../src/joint_panel.cpp"

#include <cstdlib>
#include <iostream>
#include <thread>

namespace imt342_irb120_rviz
{
struct JointPanelTestAccess
{
  static bool check(JointPanel & panel)
  {
    const QStringList ordered{"FK", "IK", "Tareas", "Diagnóstico"};
    if (panel.tabs_->count() != ordered.size()) {
      return false;
    }
    for (int i = 0; i < ordered.size(); ++i) {
      if (panel.tabs_->tabText(i) != ordered[i]) {
        return false;
      }
    }
    for (const auto * backend : {"rws", "rws_trajectory"}) {
      if (panel.backend_->findData(backend) < 0) {
        return false;
      }
    }
    auto * input = panel.matrix_;
    if (!input->paste("1 0 0 0.25\n0 1 0 -0.12\n0 0 1 0.45\n0 0 0 1") ||
      input->paste("1 0\n0 1") || input->paste("nan 0 0 1\n0 1 0 1\n0 0 1 1\n0 0 0 1"))
    {
      return false;
    }
    QApplication::clipboard()->setText("1 0 0 0.25\n0 1 0 -0.12\n0 0 1 0.46\n0 0 0 1");
    auto * first = input->findChild<QLineEdit *>();
    QKeyEvent paste(QEvent::KeyPress, Qt::Key_V, Qt::ControlModifier);
    QApplication::sendEvent(first, &paste);
    const auto matrix = input->matrix();
    const auto cells = input->findChildren<QLineEdit *>();
    if (cells.size() != 16 || !cells[12]->isReadOnly() || cells[15]->text() != "1" ||
      input->paste("1 0 0 0\n0 1 0 0\n0 0 1 0\n1 0 0 1"))
    {
      return false;
    }
    if (matrix[2].toArray()[3].toDouble() != .46) {
      return false;
    }
    panel.edits_[0]->setValue(17);
    const auto setpoint = panel.target_;
    for (int i = 0; i < ordered.size(); ++i) {
      panel.tabs_->setCurrentIndex(i);
      if (input->matrix() != matrix || panel.target_ != setpoint ||
        panel.arm_->isEnabled() || panel.apply_->isEnabled() || !panel.route_id_.isEmpty())
      {
        return false;
      }
    }

    auto * zero = panel.findChild<QPushButton *>("zeroJointTarget");
    if (!zero) {
      return false;
    }
    zero->click();
    if (panel.target_ != Joints{}) {
      return false;
    }
    for (std::size_t i = 0; i < panel.edits_.size(); ++i) {
      if (panel.edits_[i]->value() != 0 || panel.sliders_[i]->value() != slider_value(i, 0)) {
        return false;
      }
    }
    auto pose = *read_pose_matrix(matrix);
    if (check_pose(pose) != PoseError::none) {
      return false;
    }
    pose[3][0] = .01;
    if (check_pose(pose) != PoseError::last_row) {
      return false;
    }
    pose[3][0] = 0;
    pose[0][0] = .5;
    if (check_pose(pose) != PoseError::orthogonality) {
      return false;
    }
    pose[0][0] = -1;
    if (check_pose(pose) != PoseError::determinant) {
      return false;
    }
    if (!branch_label("H+/C+/M0/4").contains("Solución 4") ||
      !branch_label("H+/C+/M0/4").contains("Muñeca singular M0") ||
      !branch_label("H0-ref/C-/M-/1").contains("Hombro H0-ref") ||
      !branch_label("H-/C-/M+/2").contains("Muñeca M+"))
    {
      return false;
    }

    for (int unit = 0; unit < 2; ++unit) {
      panel.unit_->setCurrentIndex(unit);
      for (std::size_t axis = 0; axis < panel.edits_.size(); ++axis) {
        auto * slider = panel.sliders_[axis];
        auto * editor = panel.edits_[axis];
        slider->setValue(0);
        if (panel.target_[axis] != lower[axis]) {
          return false;
        }
        slider->setValue(slider_max);
        if (panel.target_[axis] != upper[axis]) {
          return false;
        }
        editor->setValue(editor->minimum());
        if (panel.target_[axis] != lower[axis]) {
          return false;
        }
        editor->setValue(editor->maximum());
        if (panel.target_[axis] != upper[axis]) {
          return false;
        }
      }
    }
    const QJsonArray nominal{4.0, 4.0, 4.0, 5.0, 5.0, 7.0};
    const auto pose_record = QJsonObject{{"matrix", matrix}};
    if (panel.fk_point_->currentData().toString() != "flange" ||
      panel.goal_point_->currentData().toString() != "flange") {
      return false;
    }
    panel.fk_point_->setCurrentIndex(0);
    panel.receive_analysis(QJsonObject{{"target_fk", QJsonObject{{"tcp", pose_record}}},
      {"model_velocity_rad_s", nominal}});
    panel.refresh_fk();
    const auto first_matrix = panel.fk_matrix_->matrix();
    pose[0][0] = 1;
    pose[0][3] = .6;
    panel.receive_analysis(QJsonObject{{"target_fk", QJsonObject{{"tcp", QJsonObject{
      {"matrix", pose_json(pose)}}}}}, {"model_velocity_rad_s", nominal}});
    panel.refresh_fk();
    if (first_matrix.isEmpty() || panel.fk_matrix_->matrix() == first_matrix ||
      !panel.fk_values_->text().contains("0.60000"))
    {
      return false;
    }
    panel.inbox_->mode = "observe";
    panel.inbox_->backend = "rws";
    panel.inbox_->actual = Joints{};
    panel.inbox_->received = Clock::now();
    panel.receive_analysis(QJsonObject{{"mode", "observe"}, {"backend", "rws"},
      {"actual_fk", QJsonObject{{"flange", pose_record}}}, {"model_velocity_rad_s", nominal}});
    panel.fk_point_->setCurrentIndex(1);
    panel.refresh_state();
    if (!panel.fk_values_->text().contains("Brida") ||
      !panel.fk_values_->text().contains("Postura observada") ||
      panel.fk_matrix_->matrix().isEmpty() || panel.use_route_->isHidden() ||
      panel.use_route_->isEnabled() || !panel.ik_execution_hint_->text().contains("Solo lectura") ||
      !panel.ik_arm_->isHidden())
    {
      return false;
    }
    panel.fk_point_->setCurrentIndex(0);
    panel.analysis_.insert("lspb_velocity_rad_s", QJsonArray{.6, .6, .6, .6, .6, .6});
    panel.refresh_speed();
    if (panel.speed_effective_[0]->text() != "N/D") {
      return false;
    }
    panel.analysis_.insert("plan", QJsonObject{
      {"peak_velocity_rad_s", QJsonArray{.42, .25, .18, .12, .09, .04}}});
    panel.refresh_speed();
    if (panel.speed_effective_[1]->text() != "0.25") {
      return false;
    }
    if (panel.speed_effective_[0]->text() != "0.42" || panel.speed_measured_[0]->text() != "N/D")
    {
      return false;
    }
    rviz_common::Config config;
    panel.save(config);
    config.mapSetValue("TrajectoryProfile", QStringLiteral("quintic"));
    config.mapSetValue("LSPB_0", 1.0);  // Old preferences cannot scale the automatic plan.
    panel.load(config);
    panel.refresh_speed();
    if (panel.speed_effective_[0]->text() != "0.42" ||
      panel.profile_->currentData().toString() != "lspb") {
      return false;
    }
    panel.actual_fresh_ = true;
    panel.analysis_.insert("source", "MEDIDO (SIMULADO/MOCK)");
    panel.analysis_.insert("measured_qdot", QJsonArray{.2, 0, 0, 0, 0, 0});
    panel.analysis_.insert("feedback_hz", 10);
    panel.refresh_speed();
    if (panel.speed_measured_[0]->text() != "0.20" ||
      !panel.speed_units_->toolTip().contains("mock · 10.0 Hz"))
    {
      return false;
    }
    panel.analysis_.insert("stale", true);
    panel.refresh_speed();
    panel.receive_analysis(QJsonObject{{"mode", "observe"}, {"backend", "rws"},
      {"irc5_connection", QJsonObject{{"state", "connected"}}}});
    panel.update_connection_indicator();
    if (!panel.connect_irc5_->isEnabled()) {
      return false;
    }
    panel.receive_analysis(QJsonObject{{"mode", "observe"}, {"backend", "rws"},
      {"irc5_connection", QJsonObject{{"state", "error"}, {"detail", "Credenciales RWS rechazadas"}}},
      {"transport", QJsonObject{{"http_status", 401}, {"cookie", "synthetic-secret"}}}});
    panel.refresh_diagnostics();
    panel.update_connection_indicator();
    if (panel.speed_measured_[0]->text() != "N/D" ||
      !panel.actual_source_->text().contains("Credenciales") ||
      !panel.diagnosis_detail_->toPlainText().contains("401") ||
      panel.diagnosis_detail_->toPlainText().contains("synthetic-secret"))
    {
      return false;
    }

    auto option_enabled = [](QComboBox * combo, const char * value) {
        auto * model = qobject_cast<QStandardItemModel *>(combo->model());
        auto * item = model->item(combo->findData(value));
        return item && item->isEnabled();
      };
    panel.inbox_->mode = "offline";
    panel.inbox_->backend = "rws_trajectory";
    panel.inbox_->actual_pose = R"({"source":"virtual"})";
    panel.receive_analysis(QJsonObject{{"mode", "offline"}, {"backend", "rws_trajectory"}});
    panel.refresh_state();
    if (!option_enabled(panel.backend_, "rws_trajectory") ||
      option_enabled(panel.mode_, "observe") || option_enabled(panel.mode_, "control") ||
      !panel.actual_source_->text().contains("SIMULACIÓN") ||
      panel.connect_irc5_->text() != "Conectar y preparar IRC5")
    {
      return false;
    }
    if (panel.backend_->count() != 2 || panel.backend_->findData("egm") >= 0 ||
      !panel.diagnosis_error_->text().contains("MoveAbsJ") ||
      panel.profile_->findData("lspb") < 0 || panel.profile_->findData("quintic") < 0)
    {
      return false;
    }
    panel.inbox_->mode = "observe";
    panel.inbox_->actual_pose = R"({"source":"medida"})";
    panel.inbox_->received = Clock::now();
    QJsonObject timed{
      {"mode", "observe"}, {"backend", "rws_trajectory"}, {"source", "rws_trajectory"},
      {"backend_connected", true}, {"rws_trajectory_configured", true},
      {"rws_trajectory_motion_enabled", false}, {"measured_qdot", QJsonArray{.2, 0, 0, 0, 0, 0}},
      {"transport", QJsonObject{{"trajectory_status", "PP supervisado pendiente"},
        {"execution", "stopped"}}}};
    panel.receive_analysis(timed);
    panel.refresh_state();
    if (!option_enabled(panel.mode_, "observe") || !option_enabled(panel.mode_, "control") ||
      panel.arm_->isEnabled() ||
      panel.speed_measured_[0]->text() != "0.20" ||
      !panel.actual_source_->text().contains("RWS v2") ||
      !panel.diagnosis_error_->text().contains("PP supervisado pendiente") ||
      !panel.diagnosis_error_->text().contains("Preparación requerida"))
    {
      return false;
    }
    timed.insert("rws_trajectory_motion_enabled", true);
    timed.insert("individual_trial", QJsonObject{
      {"backend", "rws_trajectory"}, {"trial_id", "prueba-local"},
      {"reference_duration_s", 1.25}, {"requested_duration_s", 1.3},
      {"motion", QJsonObject{{"submit_to_terminal_reply_s", 1.6}, {"final_error_rad", 0.002}}}});
    timed.insert("transport", QJsonObject{{"result", "verified"}, {"execution", "running"},
      {"pointer", QJsonArray{"Imt342TrajectoryRapid", "ServeTrajectoryOrders"}},
      {"trajectory_status", "Servidor experimental en marcha"}});
    panel.receive_analysis(timed);
    panel.refresh_state();
    return option_enabled(panel.mode_, "control") &&
           panel.diagnostics_->text().contains("Verificado") &&
           panel.diagnosis_error_->text().contains("1.300 s") &&
           panel.diagnosis_error_->text().contains("1.600 s") &&
           !panel.diagnosis_error_->text().contains("LSPB") &&
           panel.diagnosis_detail_->toPlainText().contains("ServeTrajectoryOrders");
  }

  static bool check_execution_buttons()
  {
    JointPanel panel;
    panel.node_ = std::make_shared<rclcpp::Node>("imt342_qt_button_test");
    auto node = panel.node_;
    auto declaration = node->create_publisher<Text>("/imt342_qt_test/authorization", 10);
    auto execution = node->create_publisher<State>("/imt342_qt_test/execute", 10);
    panel.physical_command_ = declaration;
    panel.execute_target_ = execution;
    QString scope;
    State received;
    auto scope_sub = node->create_subscription<Text>("/imt342_qt_test/authorization", 10,
      [&scope](Text::ConstSharedPtr msg) {scope = QString::fromStdString(msg->data);});
    auto execution_sub = node->create_subscription<State>("/imt342_qt_test/execute", 10,
      [&received](State::ConstSharedPtr msg) {received = *msg;});
    auto wait_for = [&node](auto ready) {
        for (int i = 0; i < 100; ++i) {
          rclcpp::spin_some(node);
          if (ready()) {
            return true;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
      };
    if (!wait_for([&]() {return declaration->get_subscription_count() &&
      execution->get_subscription_count();}))
    {
      return false;
    }
    auto confirm = [](QPushButton * button) {
        QTimer::singleShot(30, []() {
          if (auto * dialog = qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) {
            dialog->button(QMessageBox::Yes)->click();
          }
        });
        button->click();
      };
    const QString branch("H+/C+/M-/1");
    const QJsonArray zero{0, 0, 0, 0, 0, 0};
    const QJsonArray goal_fk{.1, 0, 0, 0, 0, 0};
    const QJsonArray goal_ik{.2, 0, 0, 0, 0, 0};
    const QJsonObject route{{"branch", branch}, {"q", goal_ik},
      {"duration", 1.2}, {"within_joint_limits", true}};
    for (const auto * backend : {"rws", "rws_trajectory"}) {
      panel.inbox_->mode = "control";
      panel.inbox_->backend = backend;
      panel.inbox_->actual = Joints{};
      panel.inbox_->actual_pose = R"({"source":"medida"})";
      panel.set_target(Joints{.1, 0, 0, 0, 0, 0});
      panel.route_id_ = QStringLiteral("current-route");
      panel.receive_routes(QJsonDocument(QJsonObject{
        {"request_id", panel.route_id_}, {"routes", QJsonArray{route}},
        {"origin", QJsonObject{{"q", zero}}}
      }).toJson(QJsonDocument::Compact).toStdString());
      panel.route_options_->selectRow(0);
      QJsonObject packet{{"mode", "control"}, {"backend", backend},
        {"target", goal_fk}, {"trajectory_profile", "lspb"}, {"physical_can_arm", true},
        {"pose_plan_id", panel.route_id_}, {"plan_origin_tolerance_rad", .001},
        {"rws_trajectory_configured", true}, {"rws_trajectory_motion_enabled", true}};
      auto refresh = [&]() {
          panel.inbox_->received = Clock::now();
          panel.receive_analysis(packet);
          panel.refresh_state();
        };
      refresh();
      if (panel.arm_->isEnabled() || panel.ik_arm_->isEnabled() ||
        !panel.fk_execution_hint_->text().contains("Previsualiza"))
      {
        return false;
      }
      packet["physical_fk_reviewed"] = true;
      refresh();
      if (!panel.arm_->isEnabled() || panel.ik_arm_->isEnabled() || panel.apply_->isEnabled()) {
        return false;
      }
      scope.clear();
      confirm(panel.arm_);
      if (!wait_for([&]() {return scope.endsWith(":fk");})) {
        return false;
      }
      packet["physical_armed"] = true;
      packet["physical_authorization"] = QJsonObject{{"kind", "fk"}, {"target_q_rad", goal_fk}};
      refresh();
      if (!panel.apply_->isEnabled() || panel.use_route_->isEnabled()) {
        return false;
      }
      panel.set_target(Joints{.15, 0, 0, 0, 0, 0});
      if (panel.apply_->isEnabled()) {
        return false;
      }
      packet["physical_armed"] = false;
      packet["physical_authorization"] = QJsonValue();
      packet["physical_fk_reviewed"] = false;
      packet["physical_order_pending"] = true;
      refresh();
      if (panel.arm_->isEnabled() ||
        !panel.fk_execution_hint_->text().contains("en curso"))
      {
        return false;
      }
      packet["physical_order_pending"] = false;
      packet["physical_ik_review"] = QJsonObject{{"route_id", panel.route_id_}, {"branch", branch}};
      refresh();
      if (!panel.ik_arm_->isEnabled() || panel.arm_->isEnabled()) {
        return false;
      }
      scope.clear();
      confirm(panel.ik_arm_);
      if (!wait_for([&]() {return scope.endsWith(":ik:current-route:" + branch);})) {
        return false;
      }
      packet["physical_armed"] = true;
      packet["physical_authorization"] = QJsonObject{{"kind", "ik"}, {"target_q_rad", goal_ik},
        {"route_id", panel.route_id_}, {"branch", branch}};
      panel.inbox_->actual = Joints{.0002, 0, 0, 0, 0, 0};
      refresh();
      if (!panel.use_route_->isEnabled() || panel.apply_->isEnabled()) {
        return false;
      }
      const auto unmodified_fk_target = panel.target_;
      received = State{};
      panel.use_route_->click();
      if (!wait_for([&]() {return !received.position.empty();}) ||
        received.header.frame_id != "imt342/ik/current-route/H+/C+/M-/1" ||
        received.position.front() != .2 || panel.target_ != unmodified_fk_target)
      {
        return false;
      }
      panel.inbox_->actual = Joints{.008, 0, 0, 0, 0, 0};
      refresh();
      if (panel.use_route_->isEnabled()) {
        return false;
      }
    }
    return true;
  }

  static bool capture(JointPanel & panel, const QString & path)
  {
    if (path.isEmpty()) {
      return true;
    }
    const QJsonArray matrix{
      QJsonArray{1, 0, 0, .25}, QJsonArray{0, 1, 0, -.12},
      QJsonArray{0, 0, 1, .45}, QJsonArray{0, 0, 0, 1}};
    const QJsonObject pose{{"matrix", matrix}, {"rpy", QJsonArray{0, 0, 0}}, {"xyzw", QJsonArray{0, 0, 0, 1}}};
    panel.unit_->setCurrentIndex(0);
    panel.set_target(Joints{.2, -.25, .5, .1, -.3, .1});
    panel.inbox_->mode = "offline";
    panel.inbox_->backend = "external";
    panel.inbox_->actual = Joints{};
    panel.inbox_->received = Clock::now();
    panel.inbox_->actual_pose = R"({"source":"virtual"})";
    panel.analysis_ = QJsonObject{{"mode", "offline"}, {"backend", "external"}, {"source", "SIMULACIÓN OFFLINE"},
      {"actual", QJsonArray{0, 0, 0, 0, 0, 0}}, {"elapsed_s", 2.0}, {"status", "Ejemplo Offline · captura Qt"},
      {"target_fk", QJsonObject{{"tcp", pose}}}, {"preview_run_id", "demo-panel-01"},
      {"model_velocity_rad_s", QJsonArray{4.363, 4.363, 4.363, 5.585, 5.585, 7.330}}};
    panel.refresh_fk();
    panel.route_id_ = "captura-panel";
    const QJsonObject route{
      {"branch", "H+/C+/M-/1"}, {"q", QJsonArray{.2, -.25, .5, .1, -.3, .1}},
      {"duration", 1.8}, {"score", 2.42}, {"position_error_m", 1e-11}, {"orientation_error_rad", 1e-10},
      {"min_sigma", .12}, {"joint_distance_rad", .61}, {"within_joint_limits", true}, {"source", "Pieper"}};
    panel.receive_routes(QJsonDocument(QJsonObject{
      {"request_id", "captura-panel"}, {"routes", QJsonArray{route}},
      {"origin", QJsonObject{{"q", QJsonArray{0, 0, 0, 0, 0, 0}}}}
    }).toJson(QJsonDocument::Compact).toStdString());
    panel.route_options_->selectRow(0);
    panel.last_analysis_ = Clock::now();
    panel.refresh_state();
    panel.resize(520, 1110);
    panel.show();
    for (int i = 0; i < panel.tabs_->count(); ++i) {
      panel.tabs_->setCurrentIndex(i);
      QApplication::processEvents();
      if (!panel.grab().save(QDir(path).filePath(QString("0%1.png").arg(i + 1)))) {
        return false;
      }
    }
    return true;
  }
};
}  // namespace imt342_irb120_rviz

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  imt342_irb120_rviz::JointPanel panel;
  using imt342_irb120_rviz::JointPanelTestAccess;
  if (!JointPanelTestAccess::check(panel)) {
    std::cerr << "Error en navegación, matriz o velocidades\n";
    return EXIT_FAILURE;
  }
  rclcpp::init(0, nullptr);
  const bool execution_ok = JointPanelTestAccess::check_execution_buttons();
  rclcpp::shutdown();
  if (!execution_ok) {
    std::cerr << "Error en revisión, habilitación o despacho de destino FK/IK\n";
    return EXIT_FAILURE;
  }
  return JointPanelTestAccess::capture(panel, qEnvironmentVariable("IMT342_PANEL_CAPTURE_DIR")) ?
         EXIT_SUCCESS : EXIT_FAILURE;
}
