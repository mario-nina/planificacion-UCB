#!/usr/bin/env python3
"""Coordinates virtual state, measured-state input and independent preview."""
from ast import literal_eval
from bisect import bisect_left, bisect_right
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
import time
from urllib.parse import urlsplit
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from cell_scene import CELL_TASK_KINDS, CellScene, task_path_markers

from imt342_irb120_control.batch import BatchRun
from imt342_irb120_control.supervisor import Supervisor
from imt342_irb120_control.adapters import (
    ARRIVAL_TOLERANCE_RAD, PLAN_ORIGIN_TOLERANCE_RAD, configured_backends,
)
from imt342_irb120_kinematics import JOINT_NAMES, Model, pose_from_rpy
from imt342_irb120_kinematics.analysis import (
    ACTIVE_TCP_FRAME, BASE_FRAME, DYNAMICS_1R, FLANGE_FRAME, POINTS, PointModels,
    jacobian_record, pose_record, pose_records, pose_residual, twist,
)
from imt342_irb120_planning import (
    AUTO_REFERENCE_VELOCITIES, MODEL_VELOCITIES, PROFILE_NAMES, Sequence, plan_candidates, plan_sequence,
    planner_for_profile, tcp_samples,
)
from imt342_irb120_control.rws_trajectory import MAX_DURATION, MIN_DURATION

from imt342_irb120_planning.task import TaskInputError, plan_task, preprogrammed_demo


from imt342_irb120_planning.smooth import QuinticSegment
from imt342_irb120_planning.tracking import (TrackingMetrics, reference_limits,
                                           local_derivatives, reference_at)

PREFIX = "/imt342/"
MODES = {"offline", "observe", "control"}
FRESHNESS_SEC = 1.0
IRC5_VERIFICATION_TIMEOUT_SEC = 8.0
SCORE_WEIGHTS = (1.0, 1.0, 0.1)


def time_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def pose_matrix(pose):
    translation = (pose.position.x, pose.position.y, pose.position.z)
    raw = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    norm = math.hypot(*raw)
    if not all(map(math.isfinite, translation + raw)) or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Pose o cuaternión inválido")
    x, y, z, w = (value / norm for value in raw)
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), translation[0]),
        (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), translation[1]),
        (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


class Coordinator(Node):
    def __init__(self):
        super().__init__("imt342_coordinator")
        xml = self.declare_parameter("model_xml", "").value
        self.model = Model.from_xml(xml)
        self.model_sha256 = hashlib.sha256(xml.encode("utf-8")).hexdigest()
        self.model_points = PointModels.from_model(self.model)
        self.points = self.model_points
        cell_config = self.declare_parameter("cell_config", "").value
        self.cell_scene = CellScene(cell_config) if cell_config else None
        self.cell = self.make_cell(xml) if self.cell_scene else None
        self.cell_perimeter_available = self.check_cell_perimeter()
        self.analysis_point = "flange"
        self.tool_source = "OFFSET DE MODELO / NO CALIBRADO"
        self.tool_revision = 0
        self.started_at = time.monotonic()
        self.feedback_arrival = deque(maxlen=32)
        self.feedback_derivative = deque(maxlen=5)
        self.feedback_frequency_hz = None
        self.event = None
        self.incident_resolution = None
        self.incident_history = deque(maxlen=128)
        self.last_diagnostic = None
        self.plan_trajectory = None
        self.plan_branch = None
        self.plan_sample = None
        # These identifiers describe preview sessions, never controller motion.
        self.preview_epoch = 0
        self.preview_seq = 0
        self.preview_session = str(time.time_ns())
        self.preview_run_id = ""
        self.preview_end = None
        self.plan_cycles = None
        self.route_result = None
        self.mode = self.declare_parameter("mode", "offline").value
        if self.mode not in MODES:
            raise ValueError("Modo inválido")
        self.virtual = self.target = (0.0,) * 6
        self.measured = None
        self.last_feedback = None
        rapid_directory = Path(get_package_share_directory("imt342_irb120_bringup")) / "rapid"
        try:
            trajectory_source = (rapid_directory / "Imt342TrajectoryRapid.mod").read_bytes()
        except OSError:
            trajectory_source = None
        try:
            self.irc5_module_source = (rapid_directory / "Imt342JointServer.mod").read_bytes()
        except OSError:
            self.irc5_module_source = None
        self.trajectory_module_source = trajectory_source
        factories = configured_backends(self, trajectory_source=trajectory_source,
                                        normal_source=self.irc5_module_source)
        self.backends = Supervisor(factories, self.declare_parameter("backend", "external").value)
        if not self.backends.set_mode(self.mode):
            raise ValueError("No se pudo configurar el modo de operación")
        self.controller_config = self.rws_diagnostic_config()
        self.irc5_connection = {"state": "disconnected", "detail": "IRC5 desconectado"}
        self.irc5_recovery_deadline = 0.0
        self.irc5_generation = None
        self.irc5_deadline = 0.0
        self.irc5_verified_after = 0.0
        self.irc5_detail_before = 0.0
        self.control_context_ns = self.get_clock().now().nanoseconds
        self.last_arm_request_ns = 0
        self.armed_at_ns = None
        self.armed_review = None
        self.reviewed_motions = {}
        self.preview_motion_key = None
        self.lspb_velocity = AUTO_REFERENCE_VELOCITIES
        self.reference_duration = None
        self.selected_profile = None
        self.fk_review = None
        self.task_plan = None
        self.task_context = None
        self.task_status = {}
        self.task_plan_reply = None
        self.task_authorization = None
        self.task_cycle_receipts = {}
        evidence = Path("/evidencia")
        self.task_cycle_receipt_dir = evidence / "ciclos-verificados" if evidence.is_dir() else None
        self.last_task_confirmation_ns = 0
        self.batch = None
        self.individual_trial = None
        self.physical_record = None  # Separate from preview lifecycle; analysis only.
        self.batch_timeout = self.declare_parameter("rws_task_leg_timeout_sec", 120.0).value
        if not math.isfinite(self.batch_timeout) or self.batch_timeout <= 0:
            raise ValueError("El plazo RWS por tramo debe ser positivo y finito")
        self.playback = None
        self.preview_hold = self.target
        self.pose_routes = {}
        self.pose_plan_id = ""
        self.pose_plan_origin = None
        self.route_preview = False
        self.route_paused_at = None
        self.last_actual_pose = None
        self.last_status = None
        self.operation = "Listo"
        self.target_stamp = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.actual_out = self.create_publisher(JointState, PREFIX + "actual", 10)
        self.preview_out = self.create_publisher(JointState, PREFIX + "preview", 10)
        self.trajectory_out = self.create_publisher(JointTrajectory, PREFIX + "trajectory", latched)
        self.actual_pose_out = self.create_publisher(String, PREFIX + "actual_pose_matrix", latched)
        self.pose_markers = self.create_publisher(MarkerArray, PREFIX + "pose_route_markers", latched)
        self.pose_plan_out = self.create_publisher(String, PREFIX + "pose_plan_result", latched)
        self.analysis_out = self.create_publisher(String, PREFIX + "analysis", latched)
        self.singularity_out = self.create_publisher(MarkerArray, PREFIX + "singularity_marker", latched)
        self.tf = TransformBroadcaster(self)
        self.cell_paths = self.create_publisher(MarkerArray, PREFIX + "cell_task_markers", latched)
        if self.cell_scene is not None:
            self.cell_static = StaticTransformBroadcaster(self)
            self.cell_walls = self.create_publisher(MarkerArray, PREFIX + "cell_markers", latched)
            stamp = self.get_clock().now().to_msg()
            self.cell_static.sendTransform(self.cell_scene.static_transform(stamp))
            self.cell_walls.publish(self.cell_scene.markers(stamp))
        self.create_subscription(String, PREFIX + "rapid_prepare", self.on_rapid_prepare, 10)
        self.create_subscription(String, PREFIX + "task_command", self.on_task_command, 10)
        self.create_subscription(String, PREFIX + "analysis_command", self.on_analysis_command, 10)
        self.last_singularity_marker = None
        self.create_subscription(String, PREFIX + "pose_plan_request", self.on_pose_plan, 10)
        self.create_subscription(String, PREFIX + "pose_plan_preview", self.on_pose_preview, 10)
        self.ik_out = self.create_publisher(JointState, PREFIX + "ik_solution", 10)
        self.technical_out = self.create_publisher(String, PREFIX + "kinematics_status", latched)
        self.status_out = self.create_publisher(String, PREFIX + "status", latched)
        self.backend_out = self.create_publisher(String, PREFIX + "backend_state", latched)
        self.create_subscription(String, PREFIX + "backend_command", self.on_backend, 10)
        self.backend_out.publish(String(data=self.backends.selected))
        self.mode_out = self.create_publisher(String, PREFIX + "mode_state", latched)
        self.create_subscription(JointState, PREFIX + "target", self.on_target, 10)
        self.create_subscription(JointState, PREFIX + "preview_target", self.on_preview_target, 10)
        self.create_subscription(JointState, PREFIX + "apply_target", self.on_apply_target, 10)
        self.create_subscription(JointState, PREFIX + "execute_target", self.on_execute_target, 10)
        self.create_subscription(JointState, PREFIX + "measured", self.on_measured, 10)
        self.create_subscription(String, PREFIX + "mode_command", self.on_mode, 10)
        self.create_subscription(String, PREFIX + "physical_command", self.on_physical_command, 10)
        self.create_subscription(String, PREFIX + "preview_command", self.on_command, 10)
        self.create_subscription(PoseStamped, PREFIX + "tcp_target", self.on_ik, 10)
        self.create_subscription(JointTrajectory, PREFIX + "preview_waypoints", self.on_waypoints, 10)
        self.mode_out.publish(String(data=self.mode))
        self.technical(self.target)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.10, self.publish_analysis)
        self.create_timer(0.002, self.tick_backends)

    def make_cell(self, xml):
        from imt342_irb120_kinematics.cell import Cell
        scene = self.cell_scene
        description = Path(get_package_share_directory("imt342_irb120_description"))
        return Cell.from_urdf(scene.config, xml, description)

    def check_cell_perimeter(self):
        if self.cell is None:
            return None
        from imt342_irb120_planning.cell_task import perimeter_available
        return perimeter_available(self.points, self.cell)

    def validate_cell_trajectory(self, trajectory):
        if self.cell is not None:
            report = self.cell.check_trajectory(self.points, trajectory)
            if not report.ok:
                raise ValueError(report.reason)

    def publish_cell_path(self, path=None):
        path = path or {}
        self.cell_paths.publish(task_path_markers(
            self.get_clock().now().to_msg(), path.get("reference_xyz_m", ()), path.get("planned_xyz_m", ()),
        ))

    def rws_diagnostic_config(self):
        names = ("url", "task", "module")
        if not all(self.has_parameter(f"rws_{key}") for key in names):
            return {}
        values = {key: self.get_parameter(f"rws_{key}").value for key in names}
        try:
            url = urlsplit(values["url"])
            port = url.port
            host = url.hostname
        except ValueError:
            return {}
        if (url.scheme not in ("http", "https") or not host or not re.fullmatch(r"[0-9A-Za-z.:-]+", host)
                or url.username is not None or url.password is not None or url.query or url.fragment):
            return {}
        origin = f"{url.scheme}://[{host}]" if ":" in host else f"{url.scheme}://{host}"
        fields = {key: values[key] for key in ("task", "module")
                  if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", values[key])}
        if self.has_parameter("rws_mechunit"):
            mechunit = self.get_parameter("rws_mechunit").value
            if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", mechunit):
                fields["mechunit"] = mechunit
        return dict(fields, endpoint=f"{origin}:{port}" if port else origin)

    def report_event(self, severity, detail, code, cause, action):
        self.operation = detail
        if self.event is not None:
            self.incident_history.append({**self.event, "state": "superseded"})
        self.event = dict(state="active", severity=severity, detail=detail, code=code, cause=cause,
                          action=action, source="coordinator",
                          timestamp_ns=self.get_clock().now().nanoseconds)

    def resolve_event(self, code, reason):
        if self.event is not None and self.event["code"] == code:
            self.incident_resolution = {
                **self.event, "state": "resolved", "resolution": reason,
                "resolved_timestamp_ns": self.get_clock().now().nanoseconds,
            }
            self.incident_history.append(self.incident_resolution)
            self.event = None

    def on_analysis_command(self, msg):
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError("Se requiere un objeto de configuración")
            if "fk_review" in command:
                review = command["fk_review"]
                source, point = review["source"], review["point"]
                if source not in ("actual", "target") or point not in POINTS:
                    raise ValueError("Origen o punto FK inválido")
                q = self.target if source == "target" else self.pose_origin()[0]
                if q is None or (source == "actual" and self.mode != "offline"
                                 and not self.has_fresh_measurement()):
                    raise ValueError("La FK actual requiere una postura vigente")
                self.fk_review = dict(
                    trial_id=uuid4().hex, timestamp_ns_text=str(self.get_clock().now().nanoseconds),
                    source=source, point=point, q_rad=q, pose=pose_record(self.points.at(point).fk(q)),
                )
                return
            if "lspb_scale" in command:
                self.on_lspb_scale(command["lspb_scale"])
                return
            if "trajectory_duration_s" in command:
                duration = command["trajectory_duration_s"] if self.mode == "offline" else None
                if duration is not None and (type(duration) not in (float, int)
                                            or not MIN_DURATION <= duration <= MAX_DURATION):
                    self.operation = "Duración por tramo fuera de 0,1–30 s"
                    return
                if not self.task_active() and self.backends.owner is None and duration != self.reference_duration:
                    self.reference_duration = duration
                    self.invalidate_backend_plan("Cambió la duración de referencia")
                return
            if "trajectory_profile" in command:
                profile = command["trajectory_profile"]
                if profile not in PROFILE_NAMES:
                    self.operation = "Selecciona LSPB o quíntica"
                    return
                if not self.task_active() and self.backends.owner is None and profile != self.selected_profile:
                    self.selected_profile = profile
                    self.invalidate_backend_plan("Cambió el perfil de planificación")
                return
            if "invalidate_route" in command:
                identifier = command["invalidate_route"]
                if isinstance(identifier, str) and identifier and identifier == self.pose_plan_id:
                    self.clear_pose_routes("Cambió el destino o el límite LSPB. Recalcule las rutas.")
                return
            point = command.get("point", self.analysis_point)
            if point not in (*POINTS, "both"):
                raise ValueError("Punto permitido: Brida, TCP activo o Ambos")
            points = self.points
            tool_source = self.tool_source
            if command.get("reset_offset") is True:
                points = self.model_points
                tool_source = "OFFSET DE MODELO / NO CALIBRADO"
            elif "offset_xyz" in command or "offset_rpy" in command:
                matrix = pose_from_rpy(command["offset_xyz"], command["offset_rpy"])
                points = self.model_points.with_tool(matrix)
                tool_source = "AJUSTE MANUAL / NO CALIBRADO"
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.report_event("ERROR", str(error), "OFFSET_INVALIDO",
                              "Configuración de punto/herramienta rechazada",
                              "Revise XYZ [m] y RPY [rad] finitos en la brida")
            return
        if points.tool_offset != self.points.tool_offset or tool_source != self.tool_source:
            self.points, self.tool_source = points, tool_source
            self.cell_perimeter_available = self.check_cell_perimeter()
            self.tool_revision += 1
            self.clear_pose_routes("Cambió el offset del TCP. Recalcule las rutas.")
        self.analysis_point = point

    def on_lspb_scale(self, scales):
        global_scale = scales["global_scale"]
        axes = scales["axis_scales"]
        if (not isinstance(axes, list) or len(axes) != 6
                or any(type(x) not in (float, int) or not 0 < x <= 1
                       for x in [global_scale, *axes])):
            raise ValueError("Las escalas LSPB requieren siete valores en (0, 1]")
        # Legacy scale commands are accepted for protocol compatibility only.
        # Session planning now uses a fixed moderate automatic policy.

    def on_rapid_prepare(self, msg):
        if msg.data == "connect" or (msg.data == "prepare" and self.mode == "observe"):
            self.connect_irc5()
        elif msg.data.startswith("recover:"):
            self.recover_irc5(msg.data.removeprefix("recover:"))
        elif msg.data == "prepare":
            self.operation = "Preparación requiere Observe, RWS vigente y ninguna orden en curso"

    def set_irc5_connection(self, state, detail):
        self.irc5_connection = {"state": state, "detail": detail}

    def controller_feedback_ready(self):
        return (self.backends.connected and self.has_fresh_measurement()
                and self.measured[3] == self.backends.selected)

    def irc5_feedback_ready(self):
        return (self.backends.selected in {"rws", "rws_trajectory"} and self.controller_feedback_ready())

    def connect_irc5(self):
        state = self.irc5_connection["state"]
        if state in {"connecting", "preparing", "verifying"} or self.backends.preparing_rapid:
            return
        backend = "rws_trajectory" if self.backends.selected == "rws_trajectory" else "rws"
        if backend not in self.backends.factories:
            self.set_irc5_connection(
                "error", "En Fedora, abre IMT-342 · ABB IRB 120 y elige Solo lectura "
                         "(alternativa: ./scripts/irc5_demo.sh observe)")
            return
        if not self.selected_rapid_source():
            self.set_irc5_connection("error", "Falta el módulo RAPID local. Reconstruye el workspace")
            return
        if self.backends.owner is not None or self.task_active():
            self.set_irc5_connection("error", "Finaliza la orden activa antes de preparar IRC5")
            return
        if (state == "ready" and self.irc5_feedback_ready()
                and self.irc5_contract_ready(self.transport_details())):
            return
        self.on_backend(String(data=backend))
        self.on_mode(String(data="observe"))
        if self.mode != "observe" or self.backends.selected != backend:
            self.set_irc5_connection("error", "No se pudo seleccionar Solo lectura con RWS")
            return
        self.irc5_generation = self.backends.generation
        self.irc5_verified_after = 0.0
        self.irc5_detail_before = self.backends.next_detail
        self.irc5_deadline = time.monotonic() + IRC5_VERIFICATION_TIMEOUT_SEC
        self.set_irc5_connection("connecting", "Conectando con IRC5")

    def recover_irc5(self, recovery_id):
        pending = self.irc5_connection
        if (not recovery_id or pending["state"] != "module_missing"
                or recovery_id != pending.get("recovery_id")):
            return
        if (self.mode != "observe" or self.backends.generation != self.irc5_generation
                or time.monotonic() >= self.irc5_recovery_deadline or not self.irc5_feedback_ready()
                or not self.selected_rapid_source() or self.backends.owner is not None):
            self.set_irc5_connection("error", "Recuperación vencida. Repite el diagnóstico antes de confirmar")
        elif self.backends.request_prepare_rapid(self.selected_rapid_source(), recover_missing=True):
            self.set_irc5_connection("preparing", "Transfiriendo y verificando el módulo RAPID propio")
        else:
            self.set_irc5_connection("error", "No se pudo iniciar la recuperación RAPID")

    def selected_rapid_source(self):
        if self.backends.selected == "rws_trajectory":
            return self.trajectory_module_source
        return self.irc5_module_source

    def irc5_contract_ready(self, transport):
        if self.backends.selected == "rws_trajectory":
            return self.trajectory_motion_enabled() and self.trajectory_contract_ready(transport)
        pointer = transport.get("pointer") or ()
        reply = transport.get("reply") or ()
        sequence = transport.get("order_sequence")
        return (
            (transport.get("mode"), transport.get("motors"), transport.get("execution"))
            == ("AUTO", "motoron", "running")
            and transport.get("protocol_version") == 1
            and isinstance(pointer, (tuple, list)) and len(pointer) == 2
            and pointer[0] == self.controller_config.get("module", "Imt342JointServer")
            and pointer[1] in ("ServeJointOrders", "RunJointOrder")
            and self.backends.owner is None and transport.get("pending_sequence") is None
            and isinstance(reply, (tuple, list)) and len(reply) == 2
            and type(sequence) is int and reply[0] == sequence and reply[1] not in (1, 2)
        )

    def trajectory_contract_ready(self, transport):
        pointer = transport.get("pointer")
        reply = transport.get("reply")
        sequence = transport.get("order_sequence")
        expected = ("Imt342TrajectoryRapid", "ServeTrajectoryOrders")
        return (
            (transport.get("mode"), transport.get("motors"), transport.get("execution"))
            == ("AUTO", "motoron", "running")
            and transport.get("protocol_version") == 2
            and isinstance(pointer, (tuple, list)) and tuple(pointer) == expected
            and self.backends.owner is None and transport.get("pending_sequence") is None
            and isinstance(reply, (tuple, list)) and len(reply) == 2
            and type(sequence) is int and reply[0] == sequence and reply[1] in (0, 3, 4, 5)
        )


    def update_irc5_connection(self):
        state = self.irc5_connection["state"]
        if state in {"connecting", "preparing", "verifying"}:
            if (self.mode != "observe" or self.backends.selected not in {"rws", "rws_trajectory"}
                    or self.backends.generation != self.irc5_generation):
                self.set_irc5_connection("error", "Cambió la sesión durante la conexión")
                return
        if state == "connecting":
            if self.irc5_feedback_ready():
                if self.backends.request_prepare_rapid(self.selected_rapid_source()):
                    self.set_irc5_connection("preparing", "RWS conectado · RAPID preparando")
                else:
                    self.set_irc5_connection("error", "No se pudo iniciar la preparación RAPID")
            elif self.backends.connected and self.measured is not None and any(
                not lower <= q <= upper for q, (lower, upper) in zip(self.measured[0], self.model.limits)
            ):
                self.set_irc5_connection("error", "La postura medida excede los límites del URDF")
            else:
                # The supervisor updates its diagnostic timestamp only after a new result.
                http = (self.transport_details().get("http_status")
                        if self.backends.next_detail > self.irc5_detail_before else None)
                if http == 401:
                    self.set_irc5_connection("error", "Autenticación RWS rechazada. Revisa el llavero de Fedora")
                elif http == 403:
                    self.set_irc5_connection("error", "RWS denegó el acceso al controlador")
                elif time.monotonic() >= self.irc5_deadline:
                    self.set_irc5_connection(
                        "error", "Sin lectura RWS vigente. Revisa Ethernet y ejecuta ./scripts/irc5_demo.sh diagnose")
        elif state == "preparing":
            preparation = self.backends.rapid_preparation
            if preparation["state"] in {"started", "already_running", "switched"}:
                self.irc5_verified_after = time.monotonic()
                self.irc5_detail_before = self.backends.next_detail
                self.irc5_deadline = self.irc5_verified_after + IRC5_VERIFICATION_TIMEOUT_SEC
                self.set_irc5_connection("verifying", "Verificando RAPID y telemetría reciente")
            elif preparation["state"] == "module_missing":
                self.set_irc5_connection("module_missing", preparation["detail"])
                self.irc5_connection["recovery_id"] = uuid4().hex
                self.irc5_recovery_deadline = time.monotonic() + 60.0
            elif preparation["state"] != "preparing":
                state = preparation["state"]
                if state not in {"module_modified", "foreign_program"}:
                    state = "error"
                self.set_irc5_connection(state, preparation["detail"])
        elif state == "verifying":
            if (self.irc5_feedback_ready() and self.measured[2] > self.irc5_verified_after
                    and self.backends.next_detail > self.irc5_detail_before
                    and self.irc5_contract_ready(self.transport_details())):
                self.irc5_ready()
            elif time.monotonic() >= self.irc5_deadline:
                self.set_irc5_connection(
                    "error", "Falta confirmación reciente del servidor propio o de J1–J6")
        elif state == "module_missing":
            if (self.mode != "observe" or self.backends.generation != self.irc5_generation
                    or time.monotonic() >= self.irc5_recovery_deadline or not self.irc5_feedback_ready()):
                self.set_irc5_connection("error", "Recuperación vencida. Repite el diagnóstico antes de confirmar")
        elif state not in {"error", "module_modified", "foreign_program"}:
            if not self.irc5_feedback_ready():
                if state == "ready" and self.irc5_generation == self.backends.generation:
                    self.set_irc5_connection("error", "Se perdió la telemetría RWS reciente")
                elif state != "disconnected":
                    self.set_irc5_connection("disconnected", "IRC5 desconectado")
            elif state == "ready" and self.backends.owner is not None:
                # A running order retains the prepared session, while ownership
                # and preflight independently block the next command.
                return
            elif (self.irc5_verified_after and self.measured[2] > self.irc5_verified_after
                    and self.irc5_contract_ready(self.transport_details())):
                if state != "ready":
                    self.irc5_ready()
            elif state != "connected":
                self.set_irc5_connection("connected", "IRC5 conectado · RWS OK · J1–J6 recientes")

    def irc5_ready(self):
        self.irc5_generation = self.backends.generation
        self.set_irc5_connection(
            "ready", "Listo para operar · J1–J6 recientes · habilitación individual separada")


    def task_active(self):
        return self.batch is not None and self.batch.active

    def task_origin_context(self):
        return (self.mode, self.backends.generation, self.tool_revision, self.lspb_velocity, self.planning_profile())

    def physical_task_context(self):
        context = dict(schema_version=1, model_sha256=self.model_sha256,
                       tool_offset=pose_record(self.points.tool_offset), controller_config=self.controller_config,
                       base_frame=BASE_FRAME, tcp_frame=ACTIVE_TCP_FRAME, flange_frame=FLANGE_FRAME)
        return json.loads(json.dumps(context, allow_nan=False))

    def revoke_task_authorization(self):
        self.task_authorization = None
        self.task_status.pop("authorization_id", None)
        self.task_status.pop("authorization_ttl_s", None)
        self.task_status.pop("authorization_stamp_ns", None)

    @staticmethod
    def task_cycle_key(context, goals, settings):
        route = dict(context=context, rapid_tool="tool0",
                     goals=[tuple(map(float, goal)) for goal in goals], settings=settings)
        encoded = json.dumps(route, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(encoded.encode()).hexdigest()

    def task_cycle_receipt(self, route_key):
        if route_key in self.task_cycle_receipts:
            return self.task_cycle_receipts[route_key]
        directory = self.task_cycle_receipt_dir
        if directory is None:
            return None
        try:
            proof = json.loads((directory / f"{route_key}.json").read_text(encoding="utf-8"))
            if (proof["route_sha256"] != route_key or proof["state"] != "verified"
                    or not 1 <= proof["verified_waypoints"] == len(proof["steps"]) <= 32
                    or not re.fullmatch(r"[a-f0-9]{32}", proof["plan_id"])
                    or any(step["result"] not in {"verified", "already_satisfied"} for step in proof["steps"])
                    or not any(step["result"] == "verified" for step in proof["steps"])
                    or self.task_cycle_key(proof["controller"],
                                           (step["target_q_rad"] for step in proof["steps"]),
                                           proof["settings"]) != route_key):
                return None
        except (OSError, KeyError, TypeError, ValueError, OverflowError):
            return None
        self.task_cycle_receipts[route_key] = proof
        return proof

    def record_verified_task_cycle(self, run):
        if self.task_plan.kind == "custom":
            first = run.trajectory.segments[0].start
            last = run.trajectory.segments[-1].goal
            if any(abs(a - b) > 1e-4 for a, b in zip(first, last)):
                return
        if not any(step["result"] == "verified" for step in run.steps):
            return
        route_key = self.task_status["cycle_route_sha256"]
        proof = dict(route_sha256=route_key, state="verified",
                     verified_waypoints=run.index, plan_id=self.task_status["plan_id"],
                     finished_ns=self.task_status["finished_ns"],
                     controller=self.task_status["physical_context"],
                     settings=self.task_status["cycle_settings"],
                     start_q_rad=self.task_status["start_q_rad"], steps=run.steps)
        self.task_cycle_receipts[route_key] = proof
        directory = self.task_cycle_receipt_dir
        if directory is not None:
            temporary = None
            try:
                directory.mkdir(parents=True, exist_ok=True)
                with NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory, delete=False) as file:
                    temporary = Path(file.name)
                    json.dump(proof, file, ensure_ascii=False, allow_nan=False)
                temporary.replace(directory / f"{route_key}.json")
            except (OSError, ValueError):
                if temporary is not None:
                    try:
                        temporary.unlink(missing_ok=True)
                    except OSError:
                        pass
                self.task_status["cycle_validation_warning"] = (
                    "El ciclo se verificó. La evidencia para repetir quedó disponible en esta sesión."
                )

    def task_feedback_ready(self):
        return self.backends.selected in {"rws", "rws_trajectory"} and self.controller_feedback_ready()

    def task_eligibility_block(self):
        if self.task_plan is None or self.batch is not None or self.task_status.get("state") != "planned":
            return "Se requiere un plan vigente para autorizar otro lote"
        preset_block = self.task_status.get("physical_preset_block_reason")
        if preset_block:
            return preset_block
        if self.task_status.get("physical_context") != self.physical_task_context():
            return "Cambió la configuración del controlador, modelo, marcos o herramienta"
        backend = self.backends.selected
        if not self.controller_config or backend not in {"rws", "rws_trajectory"}:
            return "Selecciona un backend con un IRC5 configurado"
        if not self.task_feedback_ready():
            return "Se requiere telemetría J1–J6 reciente del IRC5"
        if not self.task_origin_matches():
            return "Cambió la postura medida o la configuración del plan"
        if self.mode != "control":
            return "Para autorizar el lote, selecciona Control físico"
        transport = self.transport_details()
        if backend == "rws_trajectory" and not self.trajectory_motion_enabled():
            return "Prepara Trayectorias v2 para habilitar solicitudes en esta sesión"
        if (self.task_status.get("cell_path", {}).get("execution_style") == "continuous_movel"
                and (backend != "rws_trajectory" or transport.get("module_revision") != "continuous_candidate")):
            return "El plan MoveL requiere la revisión continua v2 residente y verificada"
        ready = self.irc5_contract_ready(transport)
        if not ready:
            if transport.get("mode") is None:
                return "Falta una lectura nueva del estado RAPID para autorizar"
            if transport.get("mode") != "AUTO":
                return "El IRC5 debe informar modo AUTO para autorizar"
            if transport.get("motors") != "motoron":
                return "El IRC5 debe informar motores encendidos para autorizar"
            if transport.get("execution") != "running":
                return "Prepara el servidor RAPID para autorizar"
            if self.backends.owner is not None or transport.get("pending_sequence") is not None:
                return "Resuelve la orden RWS pendiente antes de autorizar"
            protocol = 2 if backend == "rws_trajectory" else 1
            return f"El puntero, protocolo v{protocol} o acuse de RAPID siguen sin confirmar"
        if not self.backends.can_arm:
            return "El backend todavía no está listo para autorizar el lote"
        if self.task_status.get("previewed") is not True:
            return "Completa la previsualización del recorrido antes de autorizar"
        return ""

    def task_physical_eligible(self):
        return not self.task_eligibility_block()

    def authorize_task(self, request):
        raw = request.get("stamp_ns")
        stamp = int(raw) if isinstance(raw, str) and re.fullmatch(r"[0-9]{1,20}", raw) else 0
        now = self.get_clock().now().nanoseconds
        if not max(self.control_context_ns, now - 2_000_000_000) <= stamp <= now or (
            stamp <= self.last_task_confirmation_ns
        ):
            raise ValueError("Confirmación de lote vencida o repetida")
        if self.task_authorization is not None:
            raise ValueError("El lote ya tiene una autorización pendiente")
        blocker = self.task_eligibility_block()
        if blocker:
            raise ValueError(blocker)
        kind = self.task_status.get("kind")
        live_route = kind in {"joint_test_live", "demo_cycle_live"} or kind in CELL_TASK_KINDS
        if live_route and request.get("reviewed_live_route") is not True:
            raise ValueError("Confirma la inspección del recorrido articular medido al autorizar el lote")
        if kind in CELL_TASK_KINDS and not self.cell_scene.verified:
            if request.get("reviewed_cell_geometry") is not True:
                raise ValueError("Confirma la geometría real, marcos y herramienta en la autorización del lote")
            self.task_status["cell_geometry_reviewed_ns"] = raw
        self.last_task_confirmation_ns = stamp
        self.backends.disarm()
        self.armed_at_ns = None
        token = uuid4().hex
        self.task_authorization = (token, self.task_status["plan_id"], self.task_plan,
                                   self.task_context, time.monotonic() + 12.0)
        self.task_status.update(authorization_id=token, authorization_stamp_ns=raw,
                                authorization_ttl_s=12, error="")
        if live_route:
            self.task_status.update(physical_reviewed=True, live_route_reviewed_ns=raw)

    def execute_task(self, request):
        authorization = self.task_authorization
        if authorization is None or request.get("authorization_id") != authorization[0]:
            raise ValueError("Se requiere una autorización independiente del lote")
        token, plan_id, plan, context, deadline = authorization
        self.revoke_task_authorization()
        if (plan_id != request["plan_id"] or plan is not self.task_plan
                or context != self.task_context or time.monotonic() >= deadline
                or self.task_status.get("physical_reviewed") is not True
                or not self.task_physical_eligible()):
            raise ValueError("Autorización de lote vencida o plan modificado")
        self.stop_task_preview("Inicio de lote físico")
        self.batch = BatchRun(plan.trajectory, time.monotonic(), self.batch_timeout,
                              backend=self.backends.selected)
        self.begin_physical_record(plan.trajectory, "task", plan_id, self.batch.started)
        self.task_status.update(started_ns=str(self.get_clock().now().nanoseconds), error="",
                                execution_authorization_id=token)
        self.update_task()

    def on_task_command(self, msg):
        request = {}
        try:
            if len(msg.data) > 70 * 1024:
                raise ValueError("Configuración de tarea demasiado grande")
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError("Comando de tarea inválido")
            action = request["action"]
            matches = bool(self.task_status.get("plan_id")) and (
                request.get("plan_id") == self.task_status["plan_id"]
            )
            if action == "cancel" and matches:
                self.revoke_task_authorization()
                if self.batch is None:
                    self.stop_task_preview("Tarea cancelada", phase="cancelled")
                    self.task_status["previewed"] = False
                elif self.batch.active or (self.batch.state == "unknown"
                        and self.batch.owns_latest_motion(self.backends)
                        and self.backends.owner == self.backends.selected):
                    self.batch.stop("cancel_requested", "Parada del lote solicitada")
                    if self.batch.owns_latest_motion(self.backends):
                        self.backends.cancel()
                return
            if action == "plan":
                self.task_plan_reply = None
            if self.task_status.get("state") == "simulating":
                raise ValueError("Finalice o cancele la simulación en curso")
            if self.task_active() or (self.batch and self.batch.state == "cancel_requested"):
                raise ValueError("Hay un lote en curso")
            if action == "invalidate":
                if self.batch is None and request.get("request_id") == self.task_status.get("request_id"):
                    self.revoke_task_authorization()
                    self.stop_task_preview("Cambió la tarea")
                    self.task_plan = None
                    self.task_status = {}
                    self.publish_cell_path()
                return
            if action == "plan":
                if self.backends.owner is not None:
                    raise ValueError("Resuelva primero la orden física anterior")
                self.plan_physical_task(request)
                return
            if not matches or self.task_plan is None or self.batch is not None:
                raise ValueError("Se requiere un plan de tarea vigente; recalcule antes de operar")
            if not self.task_origin_matches():
                raise ValueError("Cambió el origen, el modo o la configuración; vuelva a planificar")
            if action == "authorize":
                self.authorize_task(request)
            elif action == "execute":
                self.execute_task(request)
            elif action == "preview":
                self.revoke_task_authorization()
                self.invalidate_motion_review()
                self.start_playback(False, trajectory=self.task_plan.trajectory, cycle_info=dict(
                    total=self.task_plan.cycles, waypoints_per_cycle=len(self.task_plan.order)))
                self.task_status.update(previewed=False, preview_run_id=self.preview_run_id, error="")
            elif action == "simulate":
                if self.mode != "offline" or not self.task_status.get("previewed"):
                    raise ValueError("Simulación requiere Offline y vista previa completa")
                self.start_playback(True, trajectory=self.task_plan.trajectory, cycle_info=dict(
                    total=self.task_plan.cycles, waypoints_per_cycle=len(self.task_plan.order)))
                self.task_status.update(state="simulating", preview_run_id=self.preview_run_id,
                                        started_ns=str(self.get_clock().now().nanoseconds), error="")
                self.task_simulation_progress(0.0)
            else:
                raise ValueError("Acción de tarea desconocida")
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            if isinstance(request, dict) and request.get("action") == "plan":
                identifier = request.get("request_id")
                if isinstance(identifier, str) and identifier != self.task_status.get("request_id"):
                    self.task_plan_reply = dict(request_id=identifier, state="invalid", error=str(exc))
                    return
            self.task_status["error"] = str(exc)

    def live_joint_task(self, task, q):
        if self.mode != "offline" and not self.task_feedback_ready():
            raise ValueError("La prueba articular requiere J1–J6 medidos y recientes del IRC5")
        axis, offset = task.get("axis"), task.get("offset_deg")
        if type(task.get("cycles")) is not int or task["cycles"] != 1:
            raise ValueError("La prueba articular realiza una ida y un retorno")
        if type(axis) is not int or not 1 <= axis <= len(JOINT_NAMES):
            raise ValueError("Selecciona una articulación entre J1 y J6")
        if type(offset) not in (int, float) or not math.isfinite(offset) or offset == 0:
            raise ValueError("El desplazamiento debe ser finito y distinto de cero")
        target = list(q)
        target[axis - 1] += math.radians(offset)
        lower, upper = self.model.limits[axis - 1]
        if not lower <= target[axis - 1] <= upper:
            raise ValueError(f"J{axis}: el desplazamiento desde la postura medida excede los límites del modelo")
        times = {"duration_s": task["duration_s"]} if "duration_s" in task else {}
        return dict(kind="joint_test", cycles=1, profile=task.get("profile"), waypoints={
            "INICIO": {"q_rad": list(q), **times}, "OBJETIVO": {"q_rad": target, **times},
        })

    def plan_physical_task(self, request):
        identifier = request.get("request_id")
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 80:
            raise ValueError("ID de solicitud de tarea inválido")
        self.revoke_task_authorization()
        self.stop_task_preview("Se solicitó otro plan de tarea")
        self.publish_cell_path()
        self.task_plan = self.batch = None
        self.task_status = dict(request_id=identifier, state="invalid", previewed=False)
        task = request["task"]
        if not isinstance(task, dict):
            raise ValueError("Se requiere un objeto de tarea")
        preferred_backend = task.get("backend", self.backends.selected)
        if preferred_backend != self.backends.selected:
            raise ValueError("Selecciona el backend guardado en la tarea")
        live_joint = task.get("kind") == "joint_test_live"
        live_demo = task.get("kind") == "demo_cycle_live"
        cell_task = task.get("kind") in CELL_TASK_KINDS
        live_route = live_joint or live_demo or cell_task
        if cell_task and self.cell is None:
            raise ValueError("La sesión requiere un modelo de celda para este recorrido")
        q, source = self.pose_origin()
        if q is None or (self.mode != "offline" and not self.has_fresh_measurement()):
            raise ValueError("Se requiere una postura de partida vigente")
        if (live_demo or cell_task) and self.mode != "offline" and not self.task_feedback_ready():
            raise ValueError("El recorrido requiere J1–J6 medidos y recientes del IRC5")
        planning_task = (self.live_joint_task(task, q) if live_joint else
                         preprogrammed_demo(q, task) if live_demo else task)
        velocity = self.lspb_velocity
        if live_joint:
            percent = task.get("velocity_percent", 100)
            if type(percent) is not int or not 1 <= percent <= 100:
                raise ValueError("La velocidad de referencia debe estar entre 1 y 100 %")
            # Legacy catalog percentages do not change the automatic policy.
        cell_path = None
        try:
            planner = planner_for_profile(self.planning_profile())
            if task.get("kind") == "air_triangle":
                from imt342_irb120_planning.air_triangle import plan_air_triangle
                plan, cell_path = plan_air_triangle(self.points, q, task, velocity, self.cell,
                                                     segment_planner=planner, max_duration=MAX_DURATION)
                self.validate_cell_trajectory(plan.trajectory)
            elif cell_task:
                from imt342_irb120_planning.cell_task import plan_cell_task
                continuous = (task.get("kind") == "cell_wall" and task.get("point", "flange") == "flange"
                              and self.backends.selected == "rws_trajectory"
                              and (task.get("profile") or self.planning_profile()) == "lspb"
                              and (self.mode == "offline" or self.transport_details().get("module_revision")
                                   == "continuous_candidate"))
                if continuous:
                    from imt342_irb120_planning.cell_continuous import plan_continuous_wall
                    plan, cell_path = plan_continuous_wall(self.points, q, task, velocity, self.cell,
                                                          segment_planner=planner, max_duration=MAX_DURATION)
                else:
                    plan, cell_path = plan_cell_task(self.points, q, task, velocity, self.cell,
                                                    segment_planner=planner, max_duration=MAX_DURATION)
                    cell_path["execution_style"] = "segmented"
            else:
                plan = plan_task(self.points, q, planning_task, velocity, segment_planner=planner)
                self.validate_cell_trajectory(plan.trajectory)
        except TaskInputError as exc:
            self.task_status.update(waypoint=exc.waypoint, candidates=exc.candidates)
            raise
        profiles = plan.trajectory.profiles
        reference_profile = profiles[0] if all(value == profiles[0] for value in profiles) else "mixed"
        v2 = self.backends.selected == "rws_trajectory"
        timed = self.timed_motion_available()
        if planning_task.get("comparison") is True and reference_profile == "lspb":
            goals = [segment.goal for segment in plan.trajectory.segments]
            slower = plan_sequence(q, goals, max_velocity=tuple(0.15 * x for x in self.lspb_velocity))

            def peaks(trajectory):
                return tuple(max(abs(segment.sample(segment.duration / 2).velocities[axis])
                                 for segment in trajectory.segments) for axis in range(6))

            self.task_status["comparison"] = dict(
                reference_lspb_s=plan.trajectory.duration, reduced_lspb_s=slower.duration,
                reference_peak_rad_s=peaks(plan.trajectory), reduced_peak_rad_s=peaks(slower),
            )
        physical_context = self.physical_task_context()
        reviewed = (not live_route and task.get("physical_reviewed") is True and not task.get("offline_example")
                    and task.get("physical_context") == physical_context)
        if plan.kind != "legacy" and not live_route:
            reviewed = reviewed and task.get("physical_review_waypoints") == task["waypoints"]
        if plan.kind == "custom" or "order" in task:
            reviewed = reviewed and task.get("physical_review_order") == list(plan.order)
        preset_block = ""
        if not live_route and not reviewed:
            if task.get("offline_example"):
                preset_block = "La demostración se ejecuta en Offline"
            elif task.get("physical_reviewed") is not True:
                preset_block = "Falta registrar la revisión de los puntos para esta configuración"
            elif task.get("physical_context") != physical_context:
                preset_block = "La inspección guardada corresponde a otro IRC5, modelo, marcos o herramienta"
            else:
                preset_block = "Cambiaron los puntos o el orden desde la inspección guardada"
        elif (plan.required_start is not None or (plan.kind == "custom" and plan.cycles > 1)) and not live_route:
            origin_name = plan.order[-1]
            nominal_start = plan.required_start or plan.waypoints[-1]["q_rad"]
            tolerance = ARRIVAL_TOLERANCE_RAD if self.mode != "offline" else 1e-4
            at_origin = all(abs(actual - saved) <= tolerance for actual, saved in zip(q, nominal_start))
            if not at_origin:
                preset_block = f"La postura real debe coincidir con {origin_name} guardado"
        if not cell_task and plan.kind in {"cycle", "repeat", "custom"}:
            settings = dict(backend=self.backends.selected, profile=reference_profile,
                            velocity_rad_s=list(self.lspb_velocity),
                            duration_s=[planning_task["waypoints"][name].get("duration_s") for name in plan.order])
            if reference_profile == "mixed":
                settings["segment_profiles"] = list(profiles[:len(plan.order)])
            route_key = self.task_cycle_key(physical_context, (item["q_rad"] for item in plan.waypoints), settings)
            self.task_status.update(cycle_route_sha256=route_key, cycle_settings=settings)
            if plan.cycles > 1:
                proof = self.task_cycle_receipt(route_key)
                self.task_status["repeat_validated"] = proof is not None
                if proof is not None:
                    self.task_status["verified_cycle_plan_id"] = proof["plan_id"]
                elif not preset_block:
                    preset_block = "Primero ejecuta y verifica un ciclo único de este recorrido"
        kind = task["kind"] if live_route else plan.kind
        if timed and not preset_block:
            for index, segment in enumerate(plan.trajectory.segments):
                name = plan.order[index % len(plan.order)]
                if not MIN_DURATION <= segment.duration <= MAX_DURATION and segment.start != segment.goal:
                    preset_block = (f"{name}: el protocolo temporizado requiere una duración "
                                    f"entre {MIN_DURATION:g} y {MAX_DURATION:g} s")
                    break
        joint_test = None
        if live_joint:
            joint_test = dict(axis=task["axis"], offset_deg=task["offset_deg"], automatic_velocity=True)
            if self.mode != "offline":
                joint_test["measured_stamp_ns"] = str(time_ns(self.measured[1]))
        self.task_plan = plan
        if cell_path is not None:
            geometry = dict(xyz_m=self.cell_scene.xyz, rpy_rad=self.cell_scene.rpy,
                            outer_xy_m=self.cell_scene.outer, height_m=self.cell_scene.height,
                            thickness_m=self.cell_scene.thickness, verified=self.cell_scene.verified)
            cell_path["geometry"] = geometry
            self.task_status["cell_path"] = cell_path
            self.publish_cell_path(cell_path)
        manifest_fields = dict(kind=kind, plan=asdict(plan), context=physical_context, joint_test=joint_test)
        if cell_path is not None:
            manifest_fields["cell_path"] = cell_path
        manifest = json.dumps(manifest_fields, ensure_ascii=False, allow_nan=False,
                              sort_keys=True, separators=(",", ":"))
        self.task_status.update(physical_reviewed=reviewed, physical_context=physical_context,
                                physical_preset_block_reason=preset_block, kind=kind,
                                joint_test=joint_test,
                                plan_sha256=hashlib.sha256(manifest.encode()).hexdigest())
        self.task_context = self.task_origin_context()
        self.task_status.update(
            state="planned", plan_id=uuid4().hex, source=source, start_q_rad=q,
            required_start_q_rad=plan.required_start,
            session_id=self.preview_session, planned_ns=str(self.get_clock().now().nanoseconds),
            model_sha256=self.model_sha256, tool_source=self.tool_source, tool_revision=self.tool_revision,
            tool_offset=pose_record(self.points.tool_offset), base_frame=BASE_FRAME,
            flange_frame=FLANGE_FRAME, tcp_frame=ACTIVE_TCP_FRAME,
            order=plan.order, cycles=plan.cycles, waypoints=plan.waypoints, leg_checks=plan.leg_checks,
            planned_duration_s=plan.trajectory.duration,
            planned_lspb_s=plan.trajectory.duration if reference_profile == "lspb" else None,
            reference_profile=reference_profile, lspb_velocity_rad_s=velocity,
            collision=("Trayectoria comprobada contra cuatro muros"
                       if self.cell is not None else "Sin modelo de colisión de celda"),
            rapid=dict(
                version=2 if v2 else 1, speeddata="v10", zone="fine", tool="tool0",
                adjustable=timed, estimated_duration_s=None),
        )
        if kind != "legacy" or self.mode == "control":
            self.task_status["physical_block_reason"] = self.task_eligibility_block()

    def stop_task_preview(self, reason, *, phase="interrupted"):
        if self.task_status.get("preview_run_id") != self.preview_run_id:
            return
        self.finish_preview(phase, reason)
        self.route_preview = False
        self.route_paused_at = None
        self.preview_hold = self.target
        self.preview_epoch += 1
        self.clear_plan_sample()

    def plan_origin_tolerance(self):
        return 1e-4 if self.mode == "offline" else PLAN_ORIGIN_TOLERANCE_RAD

    def task_origin_matches(self):
        q, _ = self.pose_origin()
        return (self.task_context == self.task_origin_context() and q is not None
                and (self.mode == "offline" or self.has_fresh_measurement())
                and all(abs(a - b) <= self.plan_origin_tolerance() for a, b in zip(q, self.task_status["start_q_rad"])))

    def task_simulation_progress(self, elapsed):
        trajectory = self.task_plan.trajectory
        order = self.task_plan.order
        index = bisect_right(trajectory.waypoint_times, elapsed)
        last = min(index, len(trajectory.segments) - 1)
        steps = [
            dict(index=i, duration_s=segment.duration,
                 result="simulated_complete" if i < index else "simulating" if i == index else "pending")
            for i, segment in enumerate(trajectory.segments)
        ]
        self.task_status.update(elapsed_s=elapsed, verified_waypoints=index, verified_cycles=index // len(order),
                                waypoint=order[last % len(order)], cycle=1 + last // len(order), steps=steps)

    def update_task(self):
        if self.task_authorization is not None and (
            time.monotonic() >= self.task_authorization[-1] or not self.task_physical_eligible()
        ):
            self.revoke_task_authorization()
        if self.task_plan is not None:
            blocker = self.task_eligibility_block()
            self.task_status["physical_eligible"] = not blocker
            if self.task_status.get("kind") != "legacy" or self.mode == "control":
                self.task_status["physical_block_reason"] = blocker if self.batch is None else ""
            if self.task_authorization is not None:
                self.task_status["authorization_ttl_s"] = max(
                    0.0, self.task_authorization[-1] - time.monotonic())
        if self.batch is None:
            if self.task_status.get("state") == "simulating" and self.plan_sample is not None:
                self.task_simulation_progress(self.plan_sample.time)
            if self.task_plan is not None and self.task_status["state"] == "planned":
                q, _ = self.pose_origin()
                origin_changed = (q is not None and (self.mode == "offline" or self.has_fresh_measurement())
                                  and not self.task_origin_matches())
                if self.task_context != self.task_origin_context() or origin_changed:
                    self.stop_task_preview("Cambió el origen o la configuración")
                    self.task_status.update(state="stale", previewed=False,
                                            error="Cambió el origen o la configuración; vuelva a planificar")
                    self.task_plan = None
                    self.publish_cell_path()
            return
        run = self.batch
        active = run.active
        settling_cancel = run.state == "cancel_requested"
        now = time.monotonic()
        latest = run.owns_latest_motion(self.backends)
        if run.state == "cancel_requested" and (not latest or self.backends.owner is None):
            observed = self.backends.motion_observation if latest else None
            result = observed[0].value if observed else "before_submission"
            run.stop("cancelled" if result in ("cancelled", "before_submission") else "aborted",
                     f"Lote interrumpido · último resultado: {result}")
            if run.steps and observed:
                run.steps[-1].update(result=result, motion=observed[1])
            self.task_status["finished_ns"] = str(self.get_clock().now().nanoseconds)
        if active and self.task_context != self.task_origin_context():
            run.stop("unknown" if run.waiting else "failed", "Cambió la configuración del lote")
            self.backends.disarm()
            self.task_status["finished_ns"] = str(self.get_clock().now().nanoseconds)
        if run.active:
            run.tick(self.backends, self.task_feedback_ready(), now)
            if not run.active:
                self.backends.disarm()
                self.task_status["finished_ns"] = str(self.get_clock().now().nanoseconds)
                if (run.state == "verified" and "cycle_route_sha256" in self.task_status
                        and self.task_plan.kind in {"cycle", "custom"} and self.task_plan.cycles == 1):
                    self.record_verified_task_cycle(run)
        if run.state == "unknown" and latest and self.backends.motion_observation:
            result, evidence = self.backends.motion_observation
            self.task_status["last_controller_observation"] = dict(result=result.value, motion=evidence)
        if active or settling_cancel:
            self.task_status.update(elapsed_s=now - run.started,
                                    feedback_stale=not self.has_fresh_measurement())
            if not self.task_status["feedback_stale"]:
                q, stamp, _, source = self.measured
                self.task_status.update(latest_q_rad=q, latest_stamp_ns=str(time_ns(stamp)), latest_source=source)
        stop_available = latest and self.backends.owner == self.backends.selected and run.state == "unknown"
        index = run.index
        last_step = min(index, len(run.trajectory.segments) - 1)
        order = self.task_plan.order
        self.task_status.update(
            active=run.active, state=run.state, reason=run.reason, stop_available=stop_available,
            verified_waypoints=index, verified_cycles=index // len(order),
            waypoint=order[last_step % len(order)], cycle=1 + last_step // len(order),
            steps=run.steps,
        )

    def state(self, q, stamp=None):
        msg = JointState(name=list(JOINT_NAMES), position=list(q))
        msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        return msg

    def joint_vector(self, msg, bounded):
        if len(msg.name) != 6 or len(msg.position) != 6 or set(msg.name) != set(JOINT_NAMES):
            raise ValueError("Se requieren joint_1…joint_6 sin duplicados")
        by_name = dict(zip(msg.name, msg.position))
        q = tuple(by_name[name] for name in JOINT_NAMES)
        if not all(map(math.isfinite, q)):
            raise ValueError("Articulaciones no finitas")
        if bounded and any(not lo <= value <= hi for value, (lo, hi) in zip(q, self.model.limits)):
            raise ValueError("Objetivo fuera de límites URDF")
        return q

    def status(self, operation):
        if self.mode == "offline":
            source = "Offline · estado virtual"
        elif self.measured is None:
            source = "Online · sin telemetría"
        else:
            age = time.monotonic() - self.measured[2]
            origin = self.measured[3]
            if age > FRESHNESS_SEC:
                source = f"Online · telemetría vencida ({origin})"
            else:
                source = f"Online · medida ({origin}, {age:.1f} s)"
        if self.mode != "offline":
            source += f" · {self.backends.detail}"
        if self.mode == "control":
            source += " · ejecución habilitada" if self.backends.armed else " · ejecución bloqueada"
        rendered = f"{source} · {operation}"
        if rendered != self.last_status:
            self.last_status = rendered
            self.status_out.publish(String(data=rendered))

    def technical(self, q):
        fk = self.model.fk(q)
        xyz = ", ".join(f"{row[3]:.4f}" for row in fk[:3])
        values = ", ".join(f"{entry:.4f}" for entry in self.model.singular_values(q))
        self.technical_out.publish(String(data=f"TCP [m] ({xyz}) · σ(J escalado) [{values}]"))

    def on_target(self, msg):
        try:
            q = self.joint_vector(msg, bounded=True)
        except ValueError as error:
            self.report_event("ERROR", str(error), "OBJETIVO_INVALIDO",
                              "Consigna articular rechazada", "Revise nombres y límites del URDF")
            return False
        self.resolve_event("OBJETIVO_INVALIDO", "Objetivo articular válido")
        stamp = time_ns(msg.header.stamp)
        if stamp and stamp < self.target_stamp:
            return False
        self.target_stamp = max(stamp, self.target_stamp)
        if q != self.target:
            self.invalidate_motion_review("fk")
            self.target = q
            self.finish_preview("interrupted", "Cambió el objetivo")
            self.route_preview = False
            self.route_paused_at = None
            self.preview_hold = self.target
            self.preview_epoch += 1  # Instant repositioning is not a robot trajectory.
            self.clear_plan_sample()
            self.operation = "Objetivo actualizado"
            self.technical(q)
        return True

    def on_preview_target(self, msg):
        if self.on_target(msg):
            self.start_playback(apply=False, motion_key=("fk", self.target))

    def on_apply_target(self, msg):
        if self.mode != "offline":
            return
        try:
            context = self.execution_ik_context(msg)
        except ValueError as error:
            self.operation = str(error)
            return
        if context:
            self.start_playback(apply=True, trajectory=self.pose_routes[context["branch"]])
        elif self.on_target(msg):
            self.start_playback(apply=True)

    def individual_motion_block(self):
        if self.mode != "control":
            return "Selecciona Control físico"
        if self.task_active() or self.backends.owner is not None or (self.batch and self.batch.state == "cancel_requested"):
            return "Espera el cierre de la orden o del lote activo"
        if self.irc5_connection["state"] in {
            "error", "preparing", "connecting", "verifying", "module_missing", "module_modified", "foreign_program",
        }:
            return self.irc5_connection["detail"]
        if not self.controller_feedback_ready():
            return "Espera una lectura reciente de J1–J6 del controlador seleccionado"
        if self.backends.selected == "rws_trajectory" and (
            not self.trajectory_motion_enabled() or self.irc5_connection["state"] != "ready"
        ):
            return "Prepara y verifica Trayectorias v2 con Conectar y preparar IRC5"
        if not self.backends.can_arm:
            return "Falta confirmar que el servidor RAPID está listo y sin órdenes pendientes"
        return ""

    def invalidate_motion_review(self, kind=None):
        if kind is None:
            self.reviewed_motions.clear()
        else:
            self.reviewed_motions.pop(kind, None)
        if self.armed_review and (kind is None or self.armed_review[0][0] == kind):
            self.backends.disarm()
            self.armed_at_ns = self.armed_review = None

    def motion_review(self, key):
        record = self.reviewed_motions.get(key[0])
        if (not record or record[0] != key or record[2] != self.task_origin_context()
                or not self.controller_feedback_ready()):
            return None
        trajectory = record[1]
        origin = trajectory.sample(0).positions
        if max(abs(a - b) for a, b in zip(self.measured[0], origin)) > self.plan_origin_tolerance():
            return None
        if key[0] == "ik" and (key[1] != self.pose_plan_id or key[2] not in self.pose_routes):
            return None
        return record

    def on_physical_command(self, msg):
        if self.task_active():
            return
        if msg.data == "disarm":
            self.backends.disarm()
            self.armed_at_ns = self.armed_review = None
            self.operation = "Habilitación física revocada"
            return
        if self.task_authorization is not None:
            self.revoke_task_authorization()
        prefix, _, payload = msg.data.partition(":")
        raw_stamp, _, target = payload.partition(":")
        now = self.get_clock().now().nanoseconds
        stamp = int(raw_stamp) if re.fullmatch(r"[0-9]{1,20}", raw_stamp) else 0
        if (prefix != "arm" or stamp <= self.last_arm_request_ns
                or not max(self.control_context_ns, now - 2_000_000_000) <= stamp <= now):
            self.operation = "Solicitud de habilitación vencida o repetida"
            return
        block = self.individual_motion_block()
        if block:
            self.operation = f"Habilitación rechazada · {block}"
            return
        if target.startswith("ik:"):
            _, _, pair = target.partition(":")
            route_id, _, branch = pair.partition(":")
            key = ("ik", route_id, branch)
        elif target == "fk":
            key = ("fk", self.target)
        else:
            self.operation = "Selecciona la habilitación de FK o de la rama IK"
            return
        review = self.motion_review(key)
        if review is None:
            self.operation = "Previsualiza el destino y la postura inicial vigentes"
            return
        duration = review[1].duration
        if self.timed_motion_available() and not MIN_DURATION <= duration <= MAX_DURATION:
            self.operation = "El protocolo temporizado admite tramos de 0,1 a 30 s"
            return
        if not self.backends.arm():
            self.operation = "El servidor RAPID dejó de estar disponible"
            return
        self.last_arm_request_ns = stamp
        self.armed_at_ns = now
        self.armed_review = review
        self.operation = "Habilitado para el destino previsualizado"

    def has_fresh_measurement(self):
        return (
            self.measured is not None and 0 <= time.monotonic() - self.measured[2] <= FRESHNESS_SEC
            and all(lo <= q <= hi for q, (lo, hi) in zip(self.measured[0], self.model.limits))
        )

    def trajectory_motion_enabled(self):
        client = self.backends.clients.get("rws_trajectory")
        return getattr(client, "motion_enabled", False) is True

    def timed_motion_available(self):
        return self.backends.selected == "rws_trajectory"

    def planning_profile(self):
        return self.selected_profile or "lspb"

    def plan_direct_segment(self, start, goal, **limits):
        return planner_for_profile(self.planning_profile())(start, goal, duration=self.reference_duration if self.mode == "offline" else None, **limits)


    def revoke_trajectory_session(self):
        revoke = getattr(self.backends.clients.get("rws_trajectory"), "disable_session_motion", None)
        if revoke:
            revoke()

    def invalidate_backend_plan(self, detail):
        self.invalidate_motion_review()
        self.backends.disarm()
        self.armed_at_ns = None
        self.control_context_ns = self.get_clock().now().nanoseconds
        self.revoke_task_authorization()
        self.finish_preview("interrupted", detail)
        self.preview_hold = self.target
        self.clear_plan_sample()
        self.clear_pose_routes()
        self.preview_epoch += 1

    def on_execute_target(self, msg):
        if (self.task_active() or self.mode != "control" or not self.backends.armed
                or self.armed_review is None or not self.controller_feedback_ready()):
            self.operation = "Ejecución bloqueada · requiere habilitación vigente y telemetría reciente"
            return
        stamp = time_ns(msg.header.stamp)
        if self.armed_at_ns is None or not self.armed_at_ns < stamp <= self.get_clock().now().nanoseconds:
            self.operation = "La orden debe ser posterior a la habilitación"
            return
        try:
            goal = self.joint_vector(msg, bounded=True)
            ik = self.execution_ik_context(msg)
            key = ("ik", ik["plan_id"], ik["branch"]) if ik else ("fk", goal)
            record = self.motion_review(key)
            if record is not self.armed_review:
                raise ValueError("La habilitación corresponde a otro destino o a una vista previa vencida")
            if any(abs(a - b) > 1e-8 for a, b in zip(record[1].segments[-1].goal, goal)):
                raise ValueError("El destino difiere de la trayectoria habilitada")
        except ValueError as error:
            self.backends.disarm()
            self.armed_at_ns = self.armed_review = None
            self.operation = str(error)
            return
        if record[1].segments[0].start == goal:
            self.backends.disarm()
            self.armed_at_ns = self.armed_review = None
            self.operation = "Destino ya satisfecho · sin desplazamiento"
            self.individual_trial = dict(
                trial_id=uuid4().hex, timestamp_ns_text=str(stamp), backend=self.backends.selected,
                result="already_satisfied", finished=True, ik=ik,
                target_q_rad=goal, start_q_rad=self.measured[0],
                motion=dict(motion_sent=False, final_feedback_rad=self.measured[0],
                            final_error_rad=max(abs(a - b) for a, b in zip(goal, self.measured[0]))),
            )
            return
        self.start_playback(apply=False, physical=True, trajectory=record[1], ik_context=ik)

    def execution_ik_context(self, msg):
        prefix = "imt342/ik/"
        if not msg.header.frame_id.startswith(prefix):
            return None
        identifier, _, branch = msg.header.frame_id[len(prefix):].partition("/")
        trajectory = self.pose_routes.get(branch)
        if (not identifier or identifier != self.pose_plan_id or trajectory is None
                or self.pose_plan_origin is None):
            raise ValueError("Rama IK vencida; calcule y seleccione de nuevo")
        mode, backend, start, _ = self.pose_plan_origin
        actual = self.pose_origin()[0]
        target = self.joint_vector(msg, bounded=True)
        if ((mode, backend) != (self.mode, self.backends.selected)
                or actual is None
                or any(abs(a - b) > self.plan_origin_tolerance() for a, b in zip(actual, start))
                or any(abs(a - b) > 1e-8 for a, b in zip(trajectory.goal, target))):
            raise ValueError("Cambió la partida o el destino de la rama IK")
        return dict(plan_id=identifier, branch=branch, point=self.route_result["point"],
                    requested_matrix=self.route_result["goal_matrix"])

    def on_measured(self, msg):
        if self.mode == "offline" or self.backends.selected != "external":
            return
        try:
            q = self.joint_vector(msg, bounded=False)
        except ValueError as error:
            self.operation = str(error)
            return
        stamp = msg.header.stamp
        if not (stamp.sec or stamp.nanosec):
            stamp = self.get_clock().now().to_msg()
        source = "MEDIDO (SIMULADO/MOCK)" if msg.header.frame_id == "imt342_mock" else "ROS externo"
        original_stamp = bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        self.record_feedback(q, stamp, source, original_stamp=original_stamp)
        self.actual_out.publish(self.state(q, stamp))

    def record_feedback(self, q, stamp, source, *, original_stamp, received_at=None):
        now = time.monotonic() if received_at is None else received_at
        if self.measured and (now - self.measured[2] > FRESHNESS_SEC or source != self.measured[3]):
            self.reset_feedback_history()
        self.measured = (q, stamp, now, source)
        self.feedback_arrival.append(now)
        span = self.feedback_arrival[-1] - self.feedback_arrival[0]
        self.feedback_frequency_hz = (len(self.feedback_arrival) - 1) / span if span > 0 else None
        derivative_ns = (time_ns(stamp) if original_stamp else
                         round(received_at * 1e9) if received_at is not None else None)
        if derivative_ns is not None:
            stamp_ns = derivative_ns
            if self.feedback_derivative and stamp_ns <= self.feedback_derivative[-1][0]:
                self.feedback_derivative.clear()
            if not self.feedback_derivative or stamp_ns - self.feedback_derivative[-1][0] >= 15_000_000:
                self.feedback_derivative.append((stamp_ns, q))
        else:
            self.feedback_derivative.clear()

    def reset_feedback_history(self):
        self.feedback_arrival.clear()
        self.feedback_derivative.clear()
        self.feedback_frequency_hz = None

    def on_backend(self, msg):
        old = self.backends.selected
        if not self.task_active() and not self.backends.preparing_rapid:
            if not self.backends.select(msg.data):
                self.operation = "Cambio rechazado: compruebe configuración y órdenes pendientes"
            elif old != msg.data:
                if self.mode == "control":
                    self.on_mode(String(data="observe"))
                if old == "rws_trajectory":
                    self.revoke_trajectory_session()
                self.invalidate_backend_plan("Cambió el backend")
                self.publish_cell_path()
                self.set_irc5_connection("disconnected", "Selecciona Conectar y preparar IRC5")
                self.irc5_verified_after = 0.0
                self.measured = None
                self.reset_feedback_history()
                self.last_feedback = None
        self.backend_out.publish(String(data=self.backends.selected))

    def tick_backends(self):
        self.backends.tick(self.mode != "offline")
        trial = self.individual_trial
        if trial and not trial["finished"]:
            observation = self.backends.motion_observation
            if observation:
                result, evidence = observation
                trial.update(result=result.value, motion=evidence)
            if self.backends.owner is None:
                trial["finished"] = True
                if not observation:
                    trial["result"] = "not_dispatched"
                trial["finished_ns"] = str(self.get_clock().now().nanoseconds)
        feedback = self.backends.feedback
        if self.mode == "offline" or feedback is None or feedback is self.last_feedback:
            return
        self.last_feedback = feedback
        if any(not lo <= q <= hi for q, (lo, hi) in zip(feedback.positions, self.model.limits)):
            self.backends.disarm()
            self.armed_at_ns = None
        message = self.state(feedback.positions)
        self.record_feedback(feedback.positions, message.header.stamp, feedback.source,
                             original_stamp=False, received_at=feedback.received_at)
        self.actual_out.publish(message)

    def on_mode(self, msg):
        if self.task_active() or self.backends.preparing_rapid:
            self.mode_out.publish(String(data=self.mode))
            return
        if msg.data in MODES and msg.data != self.mode:
            self.armed_at_ns = None
            if (msg.data == "control" and self.backends.selected == "rws_trajectory"
                    and "rws_trajectory" not in self.backends.factories):
                self.operation = "Abre Solo lectura desde el lanzador para configurar Trayectorias v2."
                self.mode_out.publish(String(data=self.mode))
                return
            if not self.backends.set_mode(msg.data):
                self.operation = "Cambio de modo rechazado durante ejecución"
                self.mode_out.publish(String(data=self.mode))
                return
            if msg.data == "offline":
                self.revoke_trajectory_session()
                self.set_irc5_connection("disconnected", "Simulación aislada · IRC5 desconectado")
            elif self.irc5_connection["state"] == "ready":
                self.set_irc5_connection("connected", "Actualizando la telemetría de la sesión")
            self.measured = None
            self.reset_feedback_history()
            self.last_feedback = None
            self.mode = msg.data
            if self.mode != "offline":
                self.reference_duration = None
            self.control_context_ns = self.get_clock().now().nanoseconds
            self.invalidate_motion_review()
            self.finish_preview("interrupted", "Cambió el modo")
            self.clear_pose_routes()
            self.clear_plan_sample()
            self.preview_hold = self.target
            self.preview_epoch += 1
            self.operation = (
                "IRC5 no disponible en esta sesión. Usa Solo lectura desde el lanzador"
                if self.mode != "offline" and not self.backends.factories else
                "Control seleccionado · pulsa Conectar y preparar IRC5 para habilitar v2"
                if self.mode == "control" and self.backends.selected == "rws_trajectory"
                and not self.trajectory_motion_enabled() else "Listo"
            )
            self.mode_out.publish(String(data=self.mode))

    def on_command(self, msg):
        if msg.data == "cancel":
            self.revoke_task_authorization()
            if self.task_active():
                self.batch.stop("cancel_requested", "Parada RAPID solicitada; consulte estado de la orden")
            self.backends.disarm()
            self.armed_at_ns = None
            self.backends.cancel()
            self.finish_preview("cancelled", "Cancelación solicitada")
            self.route_preview = False
            self.route_paused_at = None
            self.preview_hold = self.target
            self.preview_epoch += 1
            self.clear_plan_sample()
            self.operation = "Reproducción cancelada"
            return
        elif msg.data in {"preview", "apply"}:
            if msg.data == "preview" or self.mode == "offline":
                self.start_playback(apply=msg.data == "apply", motion_key=("fk", self.target))

    def on_waypoints(self, msg):
        if not 1 <= len(msg.points) <= 32:
            self.operation = "Se requieren 1–32 puntos de paso"
            return
        try:
            goals = [
                self.joint_vector(
                    JointState(name=msg.joint_names, position=point.positions), bounded=True
                )
                for point in msg.points
            ]
        except ValueError as error:
            self.operation = str(error)
            return
        # Optional metadata is accepted only from the OFFLINE RViz cycle editor.
        cycles = None
        label = msg.header.frame_id
        if label.startswith("imt342/offline_cycles/"):
            try:
                _, _, count, per_cycle = label.split("/")
                count, per_cycle = int(count), int(per_cycle)
                if self.mode != "offline" or not (1 <= count <= 8 and 1 <= per_cycle <= 32):
                    raise ValueError("Ciclos permitidos únicamente Offline (1–8)")
                if len(goals) != count * per_cycle:
                    raise ValueError("Cantidad de waypoints incompatible con los ciclos")
                reference = goals[:per_cycle]
                for index in range(count):
                    if goals[index * per_cycle:(index + 1) * per_cycle] != reference:
                        raise ValueError("Todos los ciclos deben tener los mismos waypoints")
                if any(abs(a - b) > 1e-8 for a, b in zip(goals[-1], self.virtual)):
                    raise ValueError("El ciclo offline debe terminar en la postura HOME virtual")
                cycles = dict(total=count, waypoints_per_cycle=per_cycle)
            except (ValueError, TypeError) as error:
                self.report_event("WARNING", f"Secuencia repetitiva rechazada: {error}",
                                  "CICLO_INVALIDO", "Waypoints o cierre inconsistentes",
                                  "Compruebe HOME, ciclo idéntico, límites y modo Offline")
                return
        elif label not in ("", "base_link"):
            self.operation = "Marco de secuencia no reconocido"
            return
        self.start_playback(apply=False, goals=goals, cycle_info=cycles)

    def start_playback(self, apply, goals=None, *, physical=False, cycle_info=None,
                       trajectory=None, ik_context=None, motion_key=None):
        if physical and (
            self.mode != "control" or not self.backends.armed or not self.has_fresh_measurement()
        ):
            self.backends.disarm()
            self.armed_at_ns = None
            self.operation = "Ejecución requiere Control, habilitación y postura medida reciente"
            return
        start = self.virtual
        source = "virtual"
        if self.mode != "offline":
            if self.measured is not None:
                start = self.measured[0]
                source = "medida"
                if time.monotonic() - self.measured[2] > FRESHNESS_SEC:
                    source = "última medida"
        try:
            if trajectory is None:
                trajectory = plan_sequence(
                    start, [self.target] if goals is None else goals,
                    max_velocity=self.lspb_velocity,
                    segment_planner=self.plan_direct_segment,
                )
                self.validate_cell_trajectory(trajectory)
        except ValueError as error:
            if physical:
                self.backends.disarm()
                self.armed_at_ns = None
            self.operation = str(error)
            return
        if cycle_info:
            self.resolve_event("CICLO_INVALIDO", "Secuencia repetitiva válida")
        self.publish_trajectory(trajectory, period=max(0.02, trajectory.duration / 5000))
        self.route_preview = not apply and not physical
        self.route_paused_at = None
        if physical:
            self.finish_preview("interrupted", "Solicitud de ejecución")
            self.clear_plan_sample()
            self.armed_at_ns = self.armed_review = None
            goal = trajectory.segments[-1].goal
            timed = self.timed_motion_available()
            accepted = self.backends.execute(goal, trajectory)
            self.operation = "Solicitud de ejecución" if accepted else "Ejecución no iniciada"
            if accepted:
                self.individual_trial = dict(
                    trial_id=uuid4().hex, timestamp_ns_text=str(self.get_clock().now().nanoseconds),
                    backend=self.backends.selected, result="requested", finished=False,
                    start_q_rad=start, target_q_rad=goal,
                    model_target_tcp=pose_record(self.points.tcp.fk(goal)), ik=ik_context,
                    model_sha256=self.model_sha256, tool_revision=self.tool_revision,
                )
                self.begin_physical_record(trajectory, "individual", self.individual_trial["trial_id"],
                                           time.monotonic())
                self.individual_trial.update(
                    reference_profile=self.planning_profile(), reference_duration_s=trajectory.duration,
                    motion_generation=self.backends.motion_generation,
                    preview_run_id=self.preview_run_id,
                    reviewed_start_q_rad=trajectory.sample(0.0).positions,
                )
                if timed:
                    self.individual_trial.update(requested_duration_s=trajectory.duration, timing="requested_only")
                elif self.backends.selected == "rws" and self.planning_profile() == "lspb":
                    self.individual_trial["lspb_s"] = trajectory.duration
            return
        self.new_preview_run()
        if motion_key is not None and not apply:
            self.invalidate_motion_review(motion_key[0])
            self.preview_motion_key = motion_key
        self.preview_hold = trajectory.sample(0).positions
        self.plan_trajectory = trajectory
        self.plan_branch = None
        self.plan_cycles = cycle_info
        self.plan_sample = trajectory.sample(0)
        self.playback = (trajectory, time.monotonic(), apply, source)

    def finish_preview(self, phase, reason):
        if self.playback is not None:
            key, self.preview_motion_key = self.preview_motion_key, None
            if phase == "completed" and key is not None:
                trajectory = self.playback[0]
                if isinstance(trajectory, Sequence):
                    if len(trajectory.segments) == 1:
                        self.reviewed_motions[key[0]] = (key, trajectory, self.task_origin_context())
                else:
                    sequence = Sequence((trajectory,), (trajectory.duration,))
                    self.reviewed_motions[key[0]] = (key, sequence, self.task_origin_context())
            if self.task_status.get("preview_run_id") == self.preview_run_id:
                if self.task_status.get("state") == "simulating":
                    completed = phase == "completed"
                    self.task_status.update(
                        state="simulated" if completed else "cancelled", reason=reason,
                        elapsed_s=self.plan_sample.time if self.plan_sample is not None else 0.0,
                        finished_ns=str(self.get_clock().now().nanoseconds),
                    )
                    if completed:
                        self.task_simulation_progress(self.task_plan.trajectory.duration)
                    else:
                        for step in self.task_status.get("steps", ()):
                            if step["result"] == "simulating":
                                step["result"] = "cancelled"
                else:
                    self.task_status["previewed"] = phase == "completed" and self.task_origin_matches()
            self.preview_end = dict(run_id=self.preview_run_id, phase=phase, reason=reason,
                                    timestamp_ns_text=str(self.get_clock().now().nanoseconds))
            self.playback = None

    def new_preview_run(self):
        """Mark a new preview, so charts never join independent motions."""
        self.finish_preview("interrupted", "Comenzó otra previsualización")
        self.preview_epoch += 1
        self.preview_seq += 1
        self.preview_run_id = f"preview-{self.preview_session}-{self.preview_seq}"

    def clear_plan_sample(self):
        self.plan_trajectory = None
        self.plan_branch = None
        self.plan_sample = None
        self.plan_cycles = None

    def publish_trajectory(self, trajectory, period=0.02):
        msg = JointTrajectory(joint_names=list(JOINT_NAMES))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for sample in trajectory.samples(period):
            point = JointTrajectoryPoint(positions=list(sample.positions),
                                         velocities=list(sample.velocities),
                                         accelerations=list(sample.accelerations))
            ns = round(sample.time * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            msg.points.append(point)
        self.trajectory_out.publish(msg)

    def pose_origin(self):
        if self.mode == "offline":
            return self.virtual, "virtual"
        if self.measured is None:
            return None, "sin telemetría"
        q, _, received_at, source = self.measured
        label = "medida vencida" if time.monotonic() - received_at > FRESHNESS_SEC else "medida"
        return q, f"{label} · {source}"

    def on_pose_plan(self, msg):
        request_id = ""
        routes = []
        error = ""
        point = "tcp"
        matrix = origin = None
        self.clear_pose_routes(publish=False)
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError("Se requiere un objeto con matriz 4×4")
            identifier = request.get("request_id", "")
            if not isinstance(identifier, str):
                raise ValueError("request_id debe ser texto")
            request_id = identifier
            point = request.get("point", "tcp")
            model = self.points.at(point)
            matrix = request["matrix"]
            q, source = self.pose_origin()
            if q is None or (self.mode != "offline" and not self.has_fresh_measurement()):
                raise ValueError("Se requiere una postura vigente para planificar")
            origin = dict(q=q, source=source)
            candidates = model.ik_pieper(matrix, seed=q)
            if not candidates:
                raise ValueError("Destino sin rama Pieper admisible; compruebe el alcance y los límites articulares")
            # Legacy explicitly scaled IK exercises remain possible in Offline only.
            global_scale = float(request.get("global_scale", 1.0)) if self.mode == "offline" else 1.0
            axis_scales = tuple(float(value) for value in request.get("axis_scales", (1.0,) * 6)) if self.mode == "offline" else (1.0,) * 6
            min_sigma = float(request.get("min_sigma", 0.0))
            paths = plan_candidates(model, q, candidates, global_scale=global_scale,
                                    axis_scales=axis_scales, min_sigma=min_sigma,
                                    max_velocity=self.lspb_velocity, score_weights=SCORE_WEIGHTS,
                                    segment_planner=self.plan_direct_segment)
            by_branch = {candidate.branch: candidate for candidate in candidates}
            for path in paths:
                candidate = by_branch[path.branch]
                trajectory = path.trajectory
                reason = path.reason
                cell_collision = None
                if trajectory and self.cell is not None:
                    try:
                        self.validate_cell_trajectory(trajectory)
                    except ValueError as issue:
                        trajectory = None
                        reason = str(issue)
                        cell_collision = "blocked"
                    else:
                        cell_collision = "clear"
                if trajectory and trajectory.duration >= 2**31 - 1:
                    trajectory = None
                    reason = "La duración excede el formato ROS Duration"
                peaks = self.route_peaks(model, trajectory) if trajectory else {}
                if trajectory:
                    self.pose_routes[path.branch] = trajectory
                routes.append(dict(
                    branch=path.branch, q=candidate.q, score=path.score,
                    source=candidate.source, reason=reason, collision=path.collision,
                    cell_collision=cell_collision, duration=trajectory.duration if trajectory else None,
                    min_sigma=path.min_sigma, joint_distance_rad=path.joint_distance_rad,
                    position_error_m=candidate.position_error_m,
                    orientation_error_rad=candidate.orientation_error_rad,
                    singular=candidate.singular, within_joint_limits=True, point=point,
                    **peaks,
                ))
            self.pose_plan_origin = (self.mode, self.backends.selected, q, source)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            error = str(exc) or type(exc).__name__
            self.pose_routes.clear()
            routes = []
            self.report_event("ERROR", error, "PLAN_RECHAZADO", "El cálculo de ramas o ruta falló",
                              "Revise la matriz, el punto de destino y la postura inicial")
        if not error:
            self.resolve_event("PLAN_RECHAZADO", "Plan válido")
            self.resolve_event("RUTA_OBSOLETA", "Rutas recalculadas")
        self.pose_plan_id = request_id
        self.publish_route_markers()
        config = dict(global_scale=global_scale, axis_scales=axis_scales, min_sigma=min_sigma,
                      model_velocity_rad_s=MODEL_VELOCITIES,
                      reference_velocity_rad_s=self.lspb_velocity) if not error else None
        self.route_result = dict(request_id=request_id, routes=routes, error=error, config=config,
                                 point=point, goal_matrix=matrix if not error else None,
                                 origin=origin, score_weights=SCORE_WEIGHTS,
                                 tool_revision=self.tool_revision, joint_limits=self.model.limits)
        self.pose_plan_out.publish(String(data=json.dumps(self.route_result, allow_nan=False)))

    @staticmethod
    def route_peaks(model, segment):
        duration = segment.duration
        values = [model.singular_values(sample.positions)
                  for sample in segment.samples(duration / 10)]
        ratios = [sigma[0] / sigma[-1] for sigma in values if sigma[-1] > 1e-12]
        smooth = isinstance(segment, QuinticSegment)
        acceleration_sample = duration * (3 - math.sqrt(3)) / 6 if smooth else 0
        return dict(
            max_qdot_rad_s=tuple(map(abs, segment.sample(duration / 2).velocities)),
            max_qddot_rad_s2=tuple(map(abs, segment.sample(acceleration_sample).accelerations)),
            blend_time_s=None if smooth else segment.blend_fraction * duration, kappa_samples=len(values),
            kappa_max=max(ratios) if len(ratios) == len(values) else None,
            kappa_singular_samples=len(values) - len(ratios),
        )

    def clear_pose_routes(self, reason=None, *, publish=True):
        self.invalidate_motion_review("ik")
        self.pose_routes.clear()
        self.pose_plan_id = ""
        self.route_result = None
        self.pose_plan_origin = None
        if self.route_preview:
            self.finish_preview("interrupted", reason or "Cambió la configuración de la ruta")
            self.preview_hold = self.target
            self.preview_epoch += 1
            self.clear_plan_sample()
        self.route_preview = False
        self.route_paused_at = None
        if publish:
            self.publish_route_markers()
        if reason:
            self.report_event("WARNING", reason, "RUTA_OBSOLETA",
                              "Cambió la configuración del plan", "Vuelva a calcular las ramas")

    def publish_route_markers(self, selected=None):
        colors = ((0.2, 0.8, 1.0), (1.0, 0.65, 0.2), (0.8, 0.6, 1.0), (0.5, 1.0, 0.4))
        markers = [Marker(action=Marker.DELETEALL)]
        for index, (branch, segment) in enumerate(self.pose_routes.items()):
            marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, id=index, ns="tcp_lspb")
            marker.header.frame_id = "base_link"
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.009 if branch == selected else 0.004
            marker.color.r, marker.color.g, marker.color.b = colors[index % len(colors)]
            marker.color.a = 1.0 if selected is None or branch == selected else 0.45
            for _, matrix in tcp_samples(self.points.tcp, segment, period=segment.duration / 100):
                marker.points.append(Point(x=matrix[0][3], y=matrix[1][3], z=matrix[2][3]))
            markers.append(marker)
        self.pose_markers.publish(MarkerArray(markers=markers))

    def on_pose_preview(self, msg):
        if msg.data in {"cancel", "pause", "resume"}:
            if not self.route_preview:
                return
            now = time.monotonic()
            if msg.data == "cancel":
                self.finish_preview("cancelled", "Cancelación solicitada")
                self.preview_hold = self.target
                self.preview_epoch += 1
                self.route_preview = False
                self.route_paused_at = None
                self.clear_plan_sample()
                self.operation = "Vista de ruta cancelada"
            elif msg.data == "pause" and self.playback and self.route_paused_at is None:
                self.route_paused_at = now
                self.operation = "Vista de ruta en pausa"
            elif msg.data == "resume" and self.playback and self.route_paused_at is not None:
                trajectory, started, apply, source = self.playback
                self.playback = (trajectory, started + now - self.route_paused_at, apply, source)
                self.route_paused_at = None
            return
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict) or request.get("request_id", "") != self.pose_plan_id:
                raise ValueError("Solicitud de ruta desactualizada")
            branch = request["branch"]
            trajectory = self.pose_routes.get(branch) if isinstance(branch, str) else None
            if trajectory is None:
                raise ValueError("Ruta no disponible")
            scrub = "time_s" in request
            elapsed = float(request["time_s"]) if scrub else 0.0
            if not math.isfinite(elapsed) or not 0.0 <= elapsed <= trajectory.duration:
                raise ValueError("Tiempo de previsualización fuera de la trayectoria")
            mode, backend, start, source = self.pose_plan_origin
            q, _ = self.pose_origin()
            if ((mode, backend) != (self.mode, self.backends.selected) or q is None
                    or (self.mode != "offline" and not self.has_fresh_measurement())
                    or any(abs(a - b) > self.plan_origin_tolerance() for a, b in zip(q, start))):
                raise ValueError("La postura inicial cambió. Recalcule las rutas.")
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.report_event("WARNING", str(error), "PREVIEW_NO_DISPONIBLE",
                              "Solicitud de previsualización rechazada",
                              "Seleccione una ruta del cálculo vigente")
            return
        if not scrub or self.plan_branch != branch or self.plan_trajectory is not trajectory:
            self.publish_trajectory(trajectory, period=max(0.02, trajectory.duration / 5000))
            self.publish_route_markers(selected=branch)
        if (not scrub or self.plan_branch != branch or
                self.plan_trajectory is not trajectory or self.playback is None):
            self.new_preview_run()
        else:
            self.preview_epoch += 1  # Moving the time cursor is not physical motion.
        self.plan_trajectory = trajectory
        self.plan_branch = branch
        if not scrub:
            self.invalidate_motion_review("ik")
            self.preview_motion_key = ("ik", self.pose_plan_id, branch)
        else:
            self.preview_motion_key = None
        self.plan_cycles = None
        self.plan_sample = trajectory.sample(elapsed)
        self.preview_hold = self.plan_sample.positions
        self.resolve_event("PREVIEW_NO_DISPONIBLE", "Ruta vigente previsualizada")
        now = time.monotonic()
        self.playback = (trajectory, now - elapsed, False, source)
        self.route_preview = True
        self.route_paused_at = now if scrub else None
        if scrub:
            self.operation = "Vista de ruta en pausa"

    def on_ik(self, msg):
        try:
            if msg.header.frame_id != "base_link":
                raise ValueError("Se requiere la pose en base_link")
            result = self.model.ik(pose_matrix(msg.pose), self.target)
        except ValueError as error:
            self.report_event("ERROR", str(error), "IK_POSE_INVALIDA", "Pose objetivo rechazada",
                              "Revise marco, posición y cuaternión")
            return
        detail = (f"IK {'convergente' if result.converged else 'sin convergencia'} · "
                  f"{result.position_error_m:.3g} m, {result.orientation_error_rad:.3g} rad")
        if result.converged:
            self.resolve_event("IK_POSE_INVALIDA", "Pose aceptada")
            self.resolve_event("IK_NO_CONVERGE", "IK convergente")
            self.ik_out.publish(self.state(result.q, msg.header.stamp))
            self.operation = detail
        else:
            self.report_event("WARNING", detail, "IK_NO_CONVERGE",
                              "El solver numérico local no encontró una solución dentro de la tolerancia",
                              "Pruebe otra semilla o calcule las ramas Pieper")

    def transport_details(self):
        backend = self.backends.selected
        if backend not in {"rws", "rws_trajectory"}:
            return {}
        detail = self.backends.detail.partition(" · ")[2]
        try:
            fields = literal_eval(detail)
        except (ValueError, SyntaxError):
            return {}
        if not isinstance(fields, dict):
            return {}
        keys = ("protocol_version", "motion_evidence", "mode", "motors", "execution", "pointer",
                "pending_sequence", "order_sequence", "reply", "result", "final_error_rad",
                "trajectory_status", "robotware_version", "module_sha256", "module_revision", "active_tasks")
        response = {key: fields[key] for key in keys if key in fields}
        # Publish only predefined fields and recognized error codes, never transport URLs.
        error = " ".join(str(fields.get(key) or "") for key in
                         ("error", "last_error", "rapid_diagnostic", "detail"))
        http = re.search(r"\bHTTP ([45]\d\d)\b", error)
        response["http_status"] = int(http[1]) if http else None
        response["reported_error"] = bool(fields.get("error") or fields.get("last_error"))
        return response

    def diagnostic_record(self, actual, stale, transport):
        result = dict(source=self.backends.selected, code=None, cause=None, action=None)
        if self.mode == "offline":
            result.update(severity="OFFLINE", detail="Simulación; sin transporte al controlador")
        elif stale:
            result.update(severity="STALE", detail="Última postura recibida vencida", code="FEEDBACK_STALE",
                          cause="No llegaron muestras nuevas en 1 s", action="Revise la fuente de telemetría")
        elif actual is None:
            result.update(severity="N/D", detail="Sin muestras en esta sesión",
                          code="SIN_FEEDBACK", action="Revise el backend y la publicación de articulaciones")
        else:
            result.update(severity="OK", detail="Retroalimentación reciente")
        http = transport.get("http_status")
        if http is not None:
            advice = {
                401: "Revise la autenticación RWS configurada en el laboratorio",
                403: "Revise permisos RWS y estado del controlador",
            }
            result.update(severity="ERROR", detail=f"Respuesta HTTP {http}", code=f"HTTP_{http}",
                          cause="Respuesta de error observada por RWS",
                          action=advice.get(http, "Revise el diagnóstico RWS de solo lectura"))
        elif transport.get("reported_error") or transport.get("result") == "failed":
            result.update(severity="ERROR", detail="El transporte indicó un fallo",
                          cause="El adaptador reportó un error sin código reconocido",
                          action="Consulte su estado de solo lectura")
        if self.last_diagnostic is None or self.last_diagnostic[0] != result:
            self.last_diagnostic = (result, self.get_clock().now().nanoseconds)
        return {**result, "timestamp_ns": self.last_diagnostic[1]}

    def measured_velocity(self, stale):
        if self.measured and self.measured[3] in {"rws", "rws_trajectory"}:
            estimates = local_derivatives(self.feedback_derivative) if not stale else None
            return estimates[0] if estimates is not None else None
        if stale or len(self.feedback_derivative) < 3:
            return None
        first_ns, first_q = self.feedback_derivative[0]
        last_ns, last_q = self.feedback_derivative[-1]
        dt = (last_ns - first_ns) / 1e9
        if not 0.05 <= dt <= 0.8:
            return None
        return tuple((last - first) / dt for first, last in zip(first_q, last_q))

    def estimated_acceleration(self, stale):
        if self.measured and self.measured[3] in {"rws", "rws_trajectory"}:
            estimates = local_derivatives(self.feedback_derivative) if not stale else None
            return estimates[1] if estimates is not None else None
        samples = self.feedback_derivative
        if stale or len(samples) < 3:
            return None
        first_ns, first_q = samples[0]
        middle_ns, middle_q = samples[len(samples) // 2]
        last_ns, last_q = samples[-1]
        before = (middle_ns - first_ns) / 1e9
        after = (last_ns - middle_ns) / 1e9
        span = before + after
        if min(before, after) < .015 or not .05 <= span <= .8:
            return None
        return tuple(2 * ((c - b) / after - (b - a) / before) / span
                     for a, b, c in zip(first_q, middle_q, last_q))

    def publish_tool_tf(self, stamp, tool):
        transforms = []
        for prefix in ("", "preview_"):
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = stamp, prefix + FLANGE_FRAME
            tf.child_frame_id = prefix + ACTIVE_TCP_FRAME
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = tool["xyz"]
            quaternion = tf.transform.rotation
            quaternion.x, quaternion.y, quaternion.z, quaternion.w = tool["xyzw"]
            transforms.append(tf)
        self.tf.sendTransform(transforms)

    def publish_singularity(self, state_fk, jacobians):
        point = "flange" if self.analysis_point == "flange" else "tcp"
        rank = jacobians[point]["rank"] if jacobians else None
        xyz = tuple(state_fk[point]["xyz"]) if rank is not None and rank < 6 else None
        key = (point, rank, xyz)
        if key == self.last_singularity_marker:
            return
        self.last_singularity_marker = key
        marker = Marker(id=0, ns="singularidad", type=Marker.SPHERE,
                        action=Marker.ADD if xyz is not None else Marker.DELETE)
        marker.header.frame_id = BASE_FRAME
        if xyz is not None:
            marker.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.062
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.92, 0.21, 0.70, 0.74)
        self.singularity_out.publish(MarkerArray(markers=[marker]))

    def transmitted_goal(self):
        if self.mode == "offline" or self.backends.selected not in {"rws", "rws_trajectory"}:
            return None
        generation = self.backends.motion_generation
        if (self.batch and self.batch.steps and self.batch.motion_generation == generation
                and self.batch.backend == self.backends.selected):
            step = self.batch.steps[-1]
            motion = step.get("motion") or {}
            operation_id = self.task_status["plan_id"]
        elif (self.individual_trial and self.individual_trial.get("motion_generation") == generation
              and self.individual_trial["backend"] == self.backends.selected):
            motion = self.individual_trial.get("motion") or {}
            operation_id = self.individual_trial["trial_id"]
        else:
            return None
        target = motion.get("target_rad")
        if not target or not motion.get("submit_confirmed"):
            return None
        final = motion.get("final_feedback_rad")
        return dict(id=f"{operation_id}:{motion.get('sequence')}", q=target,
                    final_error=tuple(goal - measured for goal, measured in zip(target, final)) if final else None)

    def arrival_record(self, actual, stale, transmitted, transport):
        if stale or actual is None:
            return None
        goal = transmitted.get("q") if transmitted else self.target
        if goal is None or len(goal) != 6 or len(actual) != 6:
            return None
        error = tuple(d - q for d, q in zip(goal, actual))
        maximum = max(map(abs, error))
        return dict(goal_rad=goal, error_rad=error, max_error_rad=maximum,
                    tolerance_rad=ARRIVAL_TOLERANCE_RAD,
                    within_band=maximum <= ARRIVAL_TOLERANCE_RAD,
                    source="last_rws_order" if transmitted else "selected_target",
                    awaiting_terminal_reply=bool(transmitted and
                        (transport.get("motion_evidence") or {}).get("awaiting_terminal_reply")),
                    physical=self.mode != "offline")

    def begin_physical_record(self, trajectory, kind, identifier, started):
        self.physical_record = dict(id=identifier, kind=kind, backend=self.backends.selected,
                                    generation=self.backends.generation, trajectory=trajectory,
                                    started=started, finished=None, preview_epoch=self.preview_epoch,
                                    metrics=TrackingMetrics())

    def physical_analysis_record(self, now, actual, stale):
        """Reference sampled on host dispatch/observation time, not ABB servo time."""
        record = self.physical_record
        if (record is None or self.mode != "control" or record["backend"] != self.backends.selected
                or record["generation"] != self.backends.generation):
            return None
        if record["finished"] is not None and record["preview_epoch"] != self.preview_epoch:
            return None
        trajectory = record["trajectory"]
        segments = getattr(trajectory, "segments", (trajectory,))
        if record["kind"] == "individual":
            trial = self.individual_trial or {}
            if trial.get("trial_id") != record["id"]:
                return None
            evidence = trial.get("motion") or {}
            done = trial.get("finished", False)
            result = trial.get("result", "requested")
            index = 0
            fallback_start = record["started"]
        else:
            run = self.batch
            if run is None or self.task_status.get("plan_id") != record["id"]:
                return None
            step = run.steps[-1] if run.steps else {}
            evidence = step.get("motion") or {}
            owns_pending_order = run.owns_latest_motion(self.backends) and self.backends.owner is not None
            done = not run.active and not owns_pending_order and run.state != "cancel_requested"
            if run.state == "unknown" and run.owns_latest_motion(self.backends):
                observation = self.backends.motion_observation
                if observation is not None:
                    evidence = observation[1] or evidence
            result = run.state
            index = min(step.get("index", 0), len(segments) - 1)
            fallback_start = record["started"] + step.get("request_elapsed_s", 0.0)
        if done and record["finished"] is None:
            record["finished"] = evidence.get("host_release_monotonic_s") or now
        finished = record["finished"]
        endpoint = now if finished is None else finished
        result_packet = dict(id=record["id"], kind=record["kind"], backend=record["backend"],
                             state=result, finished=finished is not None,
                             started_s=record["started"] - self.started_at,
                             ended_s=None if finished is None else finished - self.started_at,
                             elapsed_s=max(0.0, endpoint - record["started"]),
                             timing="Reloj monotónico de Fedora",
                             planned_duration_s=trajectory.duration, goal_rad=segments[index].goal,
                             leg_index=index, measurement_session_s=(self.measured[2] - self.started_at)
                                                   if actual is not None and not stale and self.measured else None)
        if not evidence.get("submit_confirmed"):
            return result_packet
        segment = segments[index]
        sent = evidence.get("host_submit_monotonic_s", fallback_start)
        phase_delay = evidence.get("submit_to_observed_phase_2_s")
        origin = sent + phase_delay if phase_delay is not None else sent
        measured_time = self.measured[2] if self.measured is not None else endpoint
        reference_time = min(now, endpoint)
        sample = reference_at(segment, reference_time, origin)
        t = sample.time
        result_packet["motion"] = evidence
        result_packet["reference"] = dict(profile_id=getattr(segment, "profile", "lspb"),
            q=sample.positions, profile_qdot=sample.velocities, profile_qddot=sample.accelerations,
            profile_jerk=segment.jerk(t) if isinstance(segment, QuinticSegment) else None,
            time_s=t, duration_s=segment.duration, segment_index=index, sequence=evidence.get("sequence"),
            origin_session_s=origin - self.started_at, submit_session_s=sent - self.started_at,
            origin="rapid_phase_2_observed" if phase_delay is not None else "host_submit",
            measurement_session_s=measured_time - self.started_at,
            plot_session_s=reference_time - self.started_at)
        result_packet["reference"].update(reference_limits(segment))
        if (actual is not None and not stale and self.measured is not None
                and self.measured[3] == record["backend"]
                and origin <= measured_time <= endpoint):
            measured_reference = reference_at(segment, measured_time, origin)
            error = tuple(a - b for a, b in zip(measured_reference.positions, actual))
            result_packet["reference_error_rad"] = error
            # Count each real feedback once, and do not mix host idle time with tracking.
            if phase_delay is not None and measured_time - origin < segment.duration:
                record["metrics"].add(index, evidence.get("sequence"), measured_time, error)
        result_packet["tracking_metrics"] = record["metrics"].report(index, evidence.get("sequence"))
        if evidence.get("submit_to_terminal_reply_s") is not None:
            result_packet["ack_minus_reference_s"] = evidence["submit_to_terminal_reply_s"] - segment.duration
        return result_packet

    def publish_analysis(self):
        now = time.monotonic()
        stamp = self.get_clock().now()
        actual, _ = self.pose_origin()
        actual_fresh = actual is not None and (self.mode == "offline" or self.has_fresh_measurement())
        if self.armed_review and actual_fresh and self.motion_review(self.armed_review[0]) is None:
            self.backends.disarm()
            self.armed_at_ns = self.armed_review = None
        if self.pose_plan_origin and actual_fresh:
            initial = self.pose_plan_origin[2]
            if any(abs(now_q - start_q) > self.plan_origin_tolerance() for now_q, start_q in zip(actual, initial)):
                self.clear_pose_routes("Cambió la postura inicial. Recalcule las rutas.")
        preview, target = self.preview_hold, self.target
        age = now - self.measured[2] if self.measured is not None else None
        stale = age is not None and age > FRESHNESS_SEC
        if self.mode == "offline":
            source = "SIMULACIÓN OFFLINE"
            actual_stamp = stamp.nanoseconds
        else:
            source = self.measured[3] if self.measured else "N/D"
            actual_stamp = time_ns(self.measured[1]) if self.measured else None
        actual_fk = pose_records(self.points, actual) if actual is not None and not stale else None
        target_fk = pose_records(self.points, target)
        preview_fk = pose_records(self.points, preview) if preview is not None else None
        sample = self.plan_sample
        jacobian_q = sample.positions if sample else (actual if not stale else None)
        jacobian_source = "plan_preview" if sample else "actual"
        jacobians = {point: jacobian_record(self.points.at(point), jacobian_q)
                     for point in POINTS} if jacobian_q is not None else None
        self.publish_singularity(preview_fk if sample else actual_fk, jacobians)

        errors = dict(comparison="INCOMPARABLE", reason="Falta referencia o sincronía verificable")
        if actual_fk and (self.mode == "offline" or source == "MEDIDO (SIMULADO/MOCK)"):
            errors = dict(
                comparison="comparable_model", reference="Objetivo vigente del mismo modelo",
                reason="Modelo interno, sin referencia física independiente",
                joint_rad=tuple(d - x for d, x in zip(target, actual)),
                **{point: pose_residual(target_fk[point]["matrix"], actual_fk[point]["matrix"])
                   for point in POINTS},
            )

        plan = None
        if sample:
            dynamics = DYNAMICS_1R
            trajectory = self.plan_trajectory
            waypoint_times = getattr(trajectory, "waypoint_times", (trajectory.duration,))
            segment_index = min(bisect_right(waypoint_times, sample.time), len(waypoint_times) - 1)
            segment_start = waypoint_times[segment_index - 1] if segment_index else 0.0
            segments = getattr(trajectory, "segments", (trajectory,))
            smooth = isinstance(segments[segment_index], QuinticSegment)
            blend_time = None if smooth else segments[segment_index].blend_fraction * segments[segment_index].duration
            paused = self.route_paused_at is not None
            running = self.playback is not None and not paused
            phase = "paused" if paused else "running" if running else (
                "completed" if sample.time >= trajectory.duration else "stopped")
            # qdot/qddot describe motion NOW, not the derivative of a frozen cursor.
            # The ideal mathematical profile remains available separately.
            qdot = sample.velocities if running else (0.0,) * 6
            qddot = sample.accelerations if running else (0.0,) * 6
            cycles = self.plan_cycles or {}
            final_q = trajectory.sample(trajectory.duration).positions
            plan = dict(
                run_id=self.preview_run_id, phase=phase,
                profile_id="quintic" if smooth else "lspb",
                profile_source=("LSPB cartesiano · IK aproximada" if getattr(segments[segment_index], "linear_cartesian", False)
                                else "Cotas sintéticas de aceleración y jerk" if smooth else "LSPB ideal"),
                time_s=sample.time, duration_s=trajectory.duration, blend_s=blend_time,
                waypoint_times_s=waypoint_times, segment_index=segment_index,
                cycle_index=min(bisect_left(waypoint_times, sample.time) //
                    cycles["waypoints_per_cycle"] + 1, cycles["total"]) if cycles else None,
                cycles_total=cycles.get("total"),
                waypoints_per_cycle=cycles.get("waypoints_per_cycle"),
                q=sample.positions, qdot=qdot, qddot=qddot,
                final_q=final_q,
                remaining_joint_rad=tuple(goal - q for goal, q in zip(final_q, sample.positions)),
                profile_qdot=sample.velocities, profile_qddot=sample.accelerations,
                profile_jerk=segments[segment_index].jerk(min(segments[segment_index].duration,
                    max(0.0, sample.time - segment_start))) if smooth else None,
                paused=paused, branch=self.plan_branch,
                running=running,
                torque_1r_nm=dynamics.torque(sample.positions[0], qddot[0]),
                dynamics_1r={**asdict(dynamics), "source": "Dinámica docente 1R · parámetros sintéticos",
                             "axis": "J1 como entrada del ejemplo 1R"},
                twist={point: twist(self.points.at(point).jacobian(sample.positions), qdot)
                       for point in POINTS},
            )
            plan.update(reference_limits(segments[segment_index]))

        transport = self.transport_details() if self.mode != "offline" else {}
        diagnostic = self.diagnostic_record(actual, stale, transport)
        transmitted = self.transmitted_goal()
        arrival = self.arrival_record(actual, stale, transmitted, transport)
        tool = pose_record(self.points.tool_offset)
        self.publish_tool_tf(stamp.to_msg(), tool)
        virtual_motion = None
        if self.mode == "offline":
            applied = self.playback is not None and self.playback[2] and self.route_paused_at is None
            moving = sample if applied else None
            virtual_motion = dict(applied=applied, qdot=moving.velocities if moving else (0.0,) * 6,
                                  qddot=moving.accelerations if moving else (0.0,) * 6,
                                  source="Simulación ideal aplicada" if applied else "Simulación en reposo")
        block = self.individual_motion_block()
        fk_review = self.motion_review(("fk", self.target))
        ik = self.reviewed_motions.get("ik")
        ik_review = self.motion_review(ik[0]) if ik else None
        authorization = None
        if self.backends.armed and self.armed_review and self.motion_review(self.armed_review[0]):
            key, trajectory, _ = self.armed_review
            authorization = dict(kind=key[0], target_q_rad=trajectory.segments[-1].goal)
            if key[0] == "ik":
                authorization.update(route_id=key[1], branch=key[2])
        packet = dict(
            virtual_motion=virtual_motion,
            timestamp_ns=stamp.nanoseconds, timestamp_ns_text=str(stamp.nanoseconds),
            elapsed_s=now - self.started_at, session_id=self.preview_session,
            sampled_monotonic_ns=str(round(now * 1e9)),
            source=source, mode=self.mode, backend=self.backends.selected, stale=stale,
            preview_run_id=self.preview_run_id, preview_epoch=self.preview_epoch,
            preview_end=self.preview_end,
            feedback_age_s=age, feedback_hz=self.feedback_frequency_hz if not stale else None,
            clock="ROS + steady", joint_order=JOINT_NAMES, joint_unit="rad",
            joint_limits=self.model.limits, joint_signs=(1,) * 6, joint_offsets_rad=(0,) * 6,
            base_frame=BASE_FRAME, flange_frame=FLANGE_FRAME, tcp_frame=ACTIVE_TCP_FRAME,
            tool_offset=tool, model_tool_offset=pose_record(self.points.model_tool_offset),
            tool_source=self.tool_source, tool_revision=self.tool_revision, model_sha256=self.model_sha256,
            actual=actual if not stale else None, target=target, preview=preview,
            actual_received_session_s=(self.measured[2] - self.started_at
                                       if self.mode != "offline" and self.measured and not stale else None),
            actual_stamp_ns=actual_stamp,
            actual_stamp_ns_text=str(actual_stamp) if actual_stamp is not None else None,
            point=self.analysis_point,
            actual_fk=actual_fk, target_fk=target_fk, preview_fk=preview_fk,
            jacobian=jacobians, jacobian_q=jacobian_q, jacobian_source=jacobian_source,
            plan=plan, error=errors, status=self.operation, diagnostic=diagnostic,
            cell_geometry_verified=self.cell_scene.verified if self.cell_scene is not None else None,
            cell_perimeter_available=self.cell_perimeter_available,
            incident=self.event, incident_resolution=self.incident_resolution,
            incident_history=list(self.incident_history),
            physical_armed=authorization is not None,
            physical_authorization=authorization,
            physical_can_arm=not block,
            physical_block_reason=block,
            physical_fk_reviewed=fk_review is not None,
            physical_ik_review=dict(route_id=ik[0][1], branch=ik[0][2]) if ik_review else None,
            physical_preview_reviewed=fk_review is not None or ik_review is not None,
            physical_run=self.physical_analysis_record(now, actual, stale),
            rws_trajectory_configured="rws_trajectory" in self.backends.factories,
            rws_trajectory_motion_enabled=(self.trajectory_motion_enabled()
                                           and (self.mode == "control" or self.irc5_connection["state"] == "ready")),
            trajectory_duration_s=self.reference_duration,
            trajectory_profile=self.planning_profile(),
            lspb_velocity_rad_s=self.lspb_velocity, model_velocity_rad_s=MODEL_VELOCITIES,
            plan_origin_tolerance_rad=self.plan_origin_tolerance(),
            arrival_tolerance_rad=ARRIVAL_TOLERANCE_RAD,
            rapid_preparation=self.backends.rapid_preparation, task=self.task_status,
            task_plan_reply=self.task_plan_reply,
            physical_order_pending=self.backends.owner is not None or self.task_active()
                                   or bool(self.batch and self.batch.state == "cancel_requested"),
            physical_task_context=self.physical_task_context(),
            irc5_connection=self.irc5_connection,
            individual_trial=self.individual_trial, fk_review=self.fk_review,
            transmitted_goal=transmitted, arrival=arrival,
            backend_connected=self.mode != "offline" and self.backends.connected,
            route_result=self.route_result, pose_plan_id=self.pose_plan_id,
            transport=transport, measured_qdot=self.measured_velocity(stale),
            measured_qddot=self.estimated_acceleration(stale),
            controller_config=self.controller_config if self.backends.selected in {"rws", "rws_trajectory"} else {},
            measured_qdot_filter=("Ajuste cuadrático local de hasta 5 muestras RWS en el último instante; "
                                  "estimación, no dato del servo" if source in {"rws", "rws_trajectory"} else
                                  "Secante de 3–5 muestras (0,05–0,8 s); recepción monotónica para RWS"),
        )
        self.analysis_out.publish(String(data=json.dumps(packet, ensure_ascii=False, allow_nan=False)))

    def tick(self):
        self.update_irc5_connection()
        self.update_task()
        if self.playback is not None and not (self.route_preview and self.route_paused_at is not None):
            trajectory, started, apply, source = self.playback
            elapsed = min(time.monotonic() - started, trajectory.duration)
            self.plan_sample = trajectory.sample(elapsed)
            q = self.plan_sample.positions
            self.preview_hold = q
            if apply:
                self.virtual = q
            if elapsed >= trajectory.duration:
                self.operation = "Aplicación virtual terminada" if apply else "Previsualización terminada"
                self.finish_preview("completed", self.operation)
            else:
                percent = 100 * elapsed / trajectory.duration
                label = "Aplicando" if apply else "Previsualizando"
                self.operation = f"{label} {percent:.0f}% · partida {source}"
        if self.mode == "offline":
            self.actual_out.publish(self.state(self.virtual))
        q, source = self.pose_origin()
        if (q, source) != self.last_actual_pose:
            self.last_actual_pose = (q, source)
            self.actual_pose_out.publish(String(data=json.dumps(
                dict(matrix=self.model.fk(q) if q is not None else None, source=source),
                allow_nan=False,
            )))
        if self.preview_hold is not None:
            self.preview_out.publish(self.state(self.preview_hold))
        self.status(self.operation)


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.backends.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
