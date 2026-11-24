"""Exercise production callbacks against a backend double without DDS discovery."""
import json
import sys
from importlib.util import module_from_spec, spec_from_file_location
from math import radians
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory
from textwrap import indent
from time import monotonic, sleep
from unittest import TestCase, main
from unittest.mock import Mock, patch

from ament_index_python.packages import get_package_share_directory
from rclpy import init, try_shutdown
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from imt342_irb120_control.rws_backend import RwsError
from imt342_irb120_control.state import Feedback, Result


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
spec = spec_from_file_location(
    "coordinator", Path(__file__).resolve().parents[1] / "scripts/coordinator.py"
)
coordinator = module_from_spec(spec)
spec.loader.exec_module(coordinator)


class BackendDouble:
    ready = False

    def __init__(self):
        self.connected = True
        self.requests = []
        self.positions = (0.0,) * 6
        self.motion_evidence = {}
        self.result = Result.IDLE
        self.prepare_calls = 0
        self.rapid_running = True
        self.rapid_pointer = ("Imt342JointServer", "ServeJointOrders")
        self.http_status = None
        self.lose_feedback_after_prepare = False
        self.module_missing = False
        self.recovery_calls = 0

    def start_server(self, authorized, module_source=None, recover_missing=False):
        self.prepare_calls += 1
        if not authorized():
            raise RwsError("Preparación revocada")
        if self.module_missing:
            if not recover_missing:
                raise RwsError("Falta el módulo", state="module_missing")
            self.recovery_calls += 1
            self.module_missing = False
        if self.rapid_running:
            if self.rapid_pointer != ("Imt342JointServer", "ServeJointOrders"):
                raise RwsError("PP fuera de Imt342JointServer/ServeJointOrders", state="foreign_program")
            result = "already_running"
        else:
            self.rapid_running = True
            self.rapid_pointer = ("Imt342JointServer", "ServeJointOrders")
            result = "started"
        if self.lose_feedback_after_prepare:
            self.connected = False
        return result

    def poll(self, *, check_ready=False):
        available = self.connected and self.http_status is None
        self.ready = available and check_ready
        feedback = Feedback(self.positions, monotonic(), "rws") if available else None
        transport = dict(
            mode="AUTO", motors="motoron", execution="running" if self.rapid_running else "stopped",
            protocol_version=1, pointer=self.rapid_pointer, order_sequence=0, reply=(0, 0),
        )
        if self.http_status:
            transport["error"] = f"RWS HTTP {self.http_status}"
        return feedback, self.result, str(transport)

    def execute(self, goal, trajectory, authorized, expected_prior_sequence=None):
        if authorized():
            self.requests.append(goal)
            self.motion_evidence = dict(sequence=len(self.requests), submit_confirmed=True,
                                        target_rad=goal, start_feedback_rad=self.positions,
                                        host_submit_monotonic_s=monotonic())
            self.result = Result.EXECUTING

    def complete(self):
        self.positions = self.requests[-1]
        self.motion_evidence = {**self.motion_evidence, "reply": (len(self.requests), 3),
                                "final_feedback_rad": self.positions}
        self.result = Result.VERIFIED

    def cancel(self):
        self.result = Result.CANCELLED

    def close(self):
        pass


class TrajectoryDouble(BackendDouble):
    motion_enabled = False

    def __init__(self):
        super().__init__()
        self.legs = []
        self.rapid_running = False
        self.rapid_pointer = ("MainModule", "main")

    def poll(self, *, check_ready=False):
        feedback, result, _ = super().poll(check_ready=check_ready)
        self.ready = self.ready and self.motion_enabled
        if feedback is not None:
            feedback = Feedback(feedback.positions, feedback.received_at, "rws_trajectory")
        return feedback, result, str(dict(
            protocol_version=2, pointer=self.rapid_pointer, mode="AUTO", motors="motoron",
            execution="running" if self.rapid_running else "stopped",
            order_sequence=len(self.requests), reply=(len(self.requests), {
                Result.EXECUTING: 1, Result.VERIFIED: 3,
            }.get(self.result, 0)),
            trajectory_status="Diagnóstico v2 simulado",
        ))

    def start_server(self, authorized, module_source=None, recover_missing=False):
        self.prepare_calls += 1
        if not authorized():
            raise RwsError("Preparación revocada")
        already_running = self.rapid_running
        self.rapid_running = self.motion_enabled = True
        self.rapid_pointer = ("Imt342TrajectoryRapid", "ServeTrajectoryOrders")
        return "already_running" if already_running else "started"

    def disable_session_motion(self):
        self.motion_enabled = False

    def execute(self, goal, trajectory, authorized, expected_prior_sequence=None):
        super().execute(goal, trajectory, authorized, expected_prior_sequence)
        if self.result is Result.EXECUTING:
            self.legs.append((trajectory, expected_prior_sequence))
            self.motion_evidence.update(requested_duration_s=trajectory.duration,
                                        submit_to_observed_phase_2_s=.02)


class CoordinatorTest(TestCase):
    def setUp(self):
        description = Path(get_package_share_directory("imt342_irb120_description"))
        params = NamedTemporaryFile(mode="w")
        self.addCleanup(params.close)
        xml = (description / "urdf/irb120.urdf").read_text(encoding="utf-8")
        params.write("imt342_coordinator:\n  ros__parameters:\n    model_xml: |\n")
        params.write(indent(xml, "      "))
        params.flush()
        init(args=["--ros-args", "--params-file", params.name])
        self.addCleanup(try_shutdown)
        self.backend = BackendDouble()
        with patch.object(coordinator, "configured_backends", return_value={"rws": lambda: self.backend}):
            self.node = coordinator.Coordinator()
        self.node.declare_parameter("rws_tolerance_rad", 0.005)
        if self.node.irc5_module_source is None:
            self.node.irc5_module_source = (Path(__file__).resolve().parents[4] /
                                            "rapid/imt342/Imt342JointServer.mod").read_bytes()
        self.addCleanup(self.node.destroy_node)
        self.addCleanup(self.node.backends.close)

    def drive(self, until):
        deadline = monotonic() + 2.0
        while monotonic() < deadline:
            self.node.tick_backends()
            self.node.update_irc5_connection()
            self.node.update_task()
            if until():
                return
            sleep(0.005)
        self.fail("El supervisor no alcanzó el estado esperado")

    def mode(self, mode):
        self.node.on_mode(String(data=mode))

    def arm_message(self, scope="fk"):
        return String(data=f"arm:{self.node.get_clock().now().nanoseconds}:{scope}")

    def review_fk(self, goal=None):
        msg = self.order() if goal is None else self.node.state(goal)
        self.node.on_preview_target(msg)
        self.node.finish_preview("completed", "Previsualización completada")

    def order(self):
        return self.node.state((0.2, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_mode_and_confirmation_are_independent(self):
        self.node.on_backend(String(data="rws"))
        self.node.on_execute_target(self.order())
        self.mode("observe")
        self.drive(lambda: self.node.measured is not None)
        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.order())
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.backend.requests, [])
        self.mode("control")
        self.node.on_physical_command(self.arm_message())
        self.assertFalse(self.node.backends.armed)
        self.drive(lambda: self.node.backends.can_arm and self.node.measured is not None)
        self.node.on_execute_target(self.order())
        self.assertEqual(self.backend.requests, [])

        self.review_fk()
        early_order = self.order()
        confirmation = self.arm_message()
        self.node.on_physical_command(confirmation)
        self.assertTrue(self.node.backends.armed)
        self.node.on_execute_target(early_order)
        self.assertIsNone(self.node.backends.pending)
        self.assertEqual(self.backend.requests, [])
        self.node.on_execute_target(self.order())
        self.drive(lambda: bool(self.backend.requests))
        self.assertEqual(len(self.backend.requests), 1)

        self.mode("observe")
        self.assertEqual(self.node.mode, "control")
        self.node.on_command(String(data="cancel"))
        self.drive(lambda: self.node.backends.owner is None)
        self.node.on_physical_command(confirmation)
        self.assertFalse(self.node.backends.armed)
        self.mode("observe")
        self.assertEqual(self.node.mode, "observe")

    def test_communication_loss_clears_authorization(self):
        self.node.on_backend(String(data="rws"))
        self.mode("control")
        self.drive(lambda: self.node.backends.can_arm and self.node.measured is not None)
        self.review_fk()
        self.node.on_physical_command(self.arm_message())
        self.assertTrue(self.node.backends.armed)
        self.backend.connected = False
        self.drive(lambda: not self.node.backends.connected)
        self.node.on_execute_target(self.order())
        self.backend.connected = True
        self.drive(lambda: self.node.backends.can_arm)
        self.node.on_execute_target(self.order())
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.backend.requests, [])

    def test_batch_rejects_unreviewed_plan_and_individual_authorization(self):
        self.node.on_backend(String(data="rws"))
        self.mode("control")
        self.drive(lambda: self.node.backends.can_arm and self.node.measured is not None)

        def command(action, **fields):
            self.node.on_task_command(String(data=json.dumps(dict(action=action, **fields))))

        goals = {name: dict(q_rad=[i * 0.01, 0, 0, 0, 0, 0])
                 for i, name in enumerate(("A", "B", "C", "HOME"), 1)}
        command("plan", request_id="test", task=dict(cycles=1, waypoints=goals))
        plan_id = self.node.task_status["plan_id"]
        self.node.on_physical_command(self.arm_message())
        command("execute", plan_id=plan_id)
        self.assertIsNone(self.node.batch)
        self.assertFalse(self.backend.requests)
        command("preview", plan_id=plan_id)
        command("execute", plan_id=plan_id)
        self.assertIsNone(self.node.batch)
        trajectory, _, apply, source = self.node.playback
        self.node.playback = (trajectory, monotonic() - trajectory.duration, apply, source)
        self.node.tick()
        self.assertTrue(self.node.task_status["previewed"])
        command("execute", plan_id="another-plan")
        self.assertIsNone(self.node.batch)
        for attempt in (plan_id, "another-plan", plan_id):
            command("execute", plan_id=attempt)
            self.assertIsNone(self.node.batch)
            self.assertRegex(self.node.task_status["error"], "autorización|plan")
        command("simulate", plan_id=plan_id)
        self.assertIn("Offline", self.node.task_status["error"])
        self.assertEqual(self.backend.requests, [])
        self.assertIsNone(self.node.backends.owner)

    def task_command(self, action, **fields):
        self.node.on_task_command(String(data=json.dumps(dict(action=action, **fields))))

    def prepare_batch(self, cycles):
        self.node.controller_config = dict(endpoint="http://192.0.2.1", task="T_ROB1",
                                           module="Imt342JointServer")
        self.node.on_backend(String(data="rws"))
        self.mode("control")
        self.drive(lambda: self.node.backends.can_arm and self.node.measured is not None)
        goals = {name: dict(q_rad=[i * 0.01, 0, 0, 0, 0, 0])
                 for i, name in enumerate(("A", "B", "C", "HOME"), 1)}
        self.task_command("plan", request_id="physical-review",
                          task=dict(cycles=cycles, waypoints=goals, physical_reviewed=True,
                                    physical_context=self.node.physical_task_context()))
        plan_id = self.node.task_status["plan_id"]
        self.task_command("preview", plan_id=plan_id)
        trajectory, _, apply, source = self.node.playback
        self.node.playback = (trajectory, monotonic() - trajectory.duration, apply, source)
        self.node.tick()
        self.assertTrue(self.node.task_status["previewed"])
        self.assertTrue(self.node.task_status["physical_reviewed"])
        self.assertIsNone(self.node.batch)
        return plan_id

    def confirm_batch(self, plan_id, **confirmation):
        stamp = str(self.node.get_clock().now().nanoseconds)
        self.task_command("authorize", plan_id=plan_id, stamp_ns=stamp, **confirmation)
        self.assertEqual(self.node.task_status.get("authorization_stamp_ns"), stamp,
                         self.node.task_status.get("error"))
        return self.node.task_status["authorization_id"]

    def test_complete_reviewed_batch_uses_one_token_and_two_cycles(self):
        plan_id = self.prepare_batch(2)
        token = self.confirm_batch(plan_id)
        self.task_command("authorize", plan_id=plan_id, stamp_ns=str(self.node.get_clock().now().nanoseconds))
        self.assertEqual(self.node.task_status["authorization_id"], token)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.assertNotIn("authorization_id", self.node.task_status)
        for count in range(1, 9):
            self.drive(lambda: len(self.backend.requests) == count)
            self.backend.complete()
        self.drive(lambda: self.node.task_status.get("state") == "verified")
        self.assertEqual(self.node.task_status["verified_cycles"], 2)
        self.assertEqual(len(self.backend.requests), 8)

    def test_complete_single_cycle_after_explicit_review(self):
        plan_id = self.prepare_batch(1)
        self.task_command("execute", plan_id=plan_id, authorization_id=self.confirm_batch(plan_id))
        for count in range(1, 5):
            self.drive(lambda: len(self.backend.requests) == count)
            self.backend.complete()
        self.drive(lambda: self.node.task_status.get("state") == "verified")
        self.assertEqual(self.node.task_status["verified_cycles"], 1)

    def test_new_plan_with_changed_cycles_rejects_old_confirmation(self):
        plan_id = self.prepare_batch(1)
        token = self.confirm_batch(plan_id)
        replacement = self.prepare_batch(2)
        self.task_command("execute", plan_id=replacement, authorization_id=token)
        self.assertIsNone(self.node.batch)
        self.assertFalse(self.backend.requests)

    def test_changed_origin_rejects_confirmed_batch(self):
        plan_id = self.prepare_batch(1)
        token = self.confirm_batch(plan_id)
        q, stamp, received, source = self.node.measured
        self.node.measured = ((q[0] + .001, *q[1:]), stamp, received, source)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.node.update_task()
        self.assertEqual(self.node.task_status["state"], "stale")
        self.assertFalse(self.backend.requests)

    def test_small_feedback_jitter_preserves_plan_but_larger_change_invalidates(self):
        self.prepare_batch(1)
        q, stamp, received, source = self.node.measured
        self.node.measured = ((q[0] + radians(.02), *q[1:]), stamp, received, source)
        self.assertTrue(self.node.task_origin_matches())
        self.node.measured = ((q[0] + radians(.10), *q[1:]), stamp, received, source)
        self.assertFalse(self.node.task_origin_matches())
        self.assertFalse(self.backend.requests)

    def test_batch_token_is_revoked_on_cancel(self):
        plan_id = self.prepare_batch(1)
        token = self.confirm_batch(plan_id)
        self.task_command("cancel", plan_id=plan_id)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.assertIsNone(self.node.batch)
        self.assertEqual(self.backend.requests, [])

    def test_batch_token_expires(self):
        plan_id = self.prepare_batch(1)
        token = self.confirm_batch(plan_id)
        self.node.task_authorization = (*self.node.task_authorization[:-1], monotonic() - 1)
        self.node.update_task()
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.assertNotIn("authorization_id", self.node.task_status)
        self.assertEqual(self.backend.requests, [])

    def test_tool_revision_invalidates_batch_authorization(self):
        plan_id = self.prepare_batch(1)
        token = self.confirm_batch(plan_id)
        self.node.tool_revision += 1
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.node.update_task()
        self.assertEqual(self.node.task_status["state"], "stale")
        self.assertNotIn("authorization_id", self.node.task_status)
        self.assertEqual(self.backend.requests, [])

    def connect_catalog_rws(self, position, *, mode="control"):
        self.node.controller_config = dict(endpoint="http://192.0.2.1", task="T_ROB1",
                                           module="Imt342JointServer", mechunit="ROB_1")
        self.backend.positions = position
        self.node.on_backend(String(data="rws"))
        self.mode(mode)
        self.drive(lambda: self.node.irc5_feedback_ready() and self.node.measured[0] == position
                   and (mode == "observe" or (self.node.backends.can_arm and
                        self.node.irc5_contract_ready(self.node.transport_details()))))

    def catalog_task(self, kind, waypoints, *, cycles=1):
        return dict(kind=kind, cycles=cycles, waypoints=waypoints, physical_reviewed=True,
                    physical_context=self.node.physical_task_context(),
                    physical_review_waypoints=waypoints)

    def catalog_preview(self, task, *, repeat_validated=None):
        self.task_command("plan", request_id="catalog", task=task)
        self.assertEqual(self.node.task_status["state"], "planned", self.node.task_status)
        if repeat_validated is not None:
            self.assertIs(self.node.task_status["repeat_validated"], repeat_validated)
        plan_id = self.node.task_status["plan_id"]
        self.task_command("preview", plan_id=plan_id)
        self.finish_catalog_playback(apply=False)
        self.assertTrue(self.node.task_status["previewed"])
        return plan_id

    def finish_catalog_playback(self, *, apply):
        trajectory, _, playback_apply, source = self.node.playback
        self.assertEqual(playback_apply, apply)
        self.node.playback = trajectory, monotonic() - trajectory.duration - 1, apply, source
        self.node.tick()

    def deny_catalog_authorization(self, plan_id):
        self.task_command("authorize", plan_id=plan_id,
                          stamp_ns=str(self.node.get_clock().now().nanoseconds))
        self.assertNotIn("authorization_id", self.node.task_status)
        self.assertIsNone(self.node.batch)

    def complete_catalog_batch(self, plan_id, goals, **confirmation):
        token = self.confirm_batch(plan_id, **confirmation)
        previous = len(self.backend.requests)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.assertNotIn("authorization_id", self.node.task_status)
        for count in range(previous + 1, previous + len(goals) + 1):
            self.drive(lambda: len(self.backend.requests) == count)
            self.backend.complete()
        self.drive(lambda: self.node.task_status.get("state") == "verified")
        self.assertEqual(self.backend.requests[previous:], goals)
        return token

    def configure_cell(self, *, verified=False):
        config = Path(__file__).resolve().parents[1] / "config/cell.yaml"
        self.node.cell_scene = coordinator.CellScene(config)
        self.node.cell_scene.verified = verified
        self.node.cell_scene.config["geometry_verified"] = verified
        xml = self.node.get_parameter("model_xml").value
        self.node.cell = self.node.make_cell(xml)

    def preview_wall_task(self, *, cycles=1, kind="cell_wall"):
        self.task_command("plan", request_id="wall", task=dict(kind=kind, cycles=cycles))
        self.assertEqual(self.node.task_status["state"], "planned", self.node.task_status)
        if self.node.mode != "offline":
            planned = monotonic()
            self.drive(lambda: self.node.measured[2] > planned)
        plan_id = self.node.task_status["plan_id"]
        self.task_command("preview", plan_id=plan_id)
        self.finish_catalog_playback(apply=False)
        self.assertTrue(self.node.task_status["previewed"])
        return plan_id

    def test_fk_preview_blocks_a_wall_crossing_with_clear_endpoints(self):
        self.configure_cell()
        start = (0.82, 1.54, 0.52, -1.73, 0.37, -4.65)
        end = (2.71, -0.98, -0.08, -0.88, -1.79, -4.94)
        self.assertTrue(self.node.cell.check_pose(start, self.node.points.tool_offset).ok)
        self.assertTrue(self.node.cell.check_pose(end, self.node.points.tool_offset).ok)
        self.node.virtual = start
        self.node.trajectory_out = Mock()
        self.node.on_preview_target(self.node.state(end))
        self.assertIsNone(self.node.playback)
        self.assertIn("y_max", self.node.operation)
        self.node.trajectory_out.publish.assert_not_called()

        self.node.virtual = (0.,) * 6
        self.node.on_preview_target(self.node.state((.06, .08, .005, 0., 0., 0.)))
        self.assertIsNotNone(self.node.playback)
        self.node.trajectory_out.publish.assert_called_once()

    def test_wall_offline_repetition_preserves_start_and_does_not_use_rws(self):
        self.configure_cell()
        plan_id = self.preview_wall_task(cycles=2)
        plan = self.node.task_plan
        self.assertEqual(plan.cycles, 2)
        self.assertEqual(plan.trajectory.segments[len(plan.order) - 1].goal, (0.,) * 6)
        self.assertEqual(plan.trajectory.segments[-1].goal, (0.,) * 6)
        metrics = self.node.task_status["cell_path"]
        self.assertTrue(metrics["reference_xyz_m"])
        self.assertLess(metrics["max_tcp_deviation_m"], .025)
        self.assertFalse(metrics["geometry"]["verified"])
        self.task_command("simulate", plan_id=plan_id)
        self.finish_catalog_playback(apply=True)
        self.assertEqual(self.node.task_status["state"], "simulated")
        self.assertEqual(self.node.task_status["verified_cycles"], 2)
        self.assertEqual(self.node.virtual, (0.,) * 6)
        self.assertFalse(self.backend.requests)

    def test_provisional_wall_geometry_requires_review_in_existing_authorization(self):
        self.configure_cell()
        self.connect_catalog_rws((0.,) * 6)
        plan_id = self.preview_wall_task()
        self.task_command("authorize", plan_id=plan_id, reviewed_live_route=True,
                          stamp_ns=str(self.node.get_clock().now().nanoseconds))
        self.assertNotIn("authorization_id", self.node.task_status)
        self.assertIn("Confirma la geometría real", self.node.task_status["error"])
        self.assertFalse(self.backend.requests)

    def test_wall_normal_double_uses_the_same_discrete_batch(self):
        self.configure_cell(verified=False)
        self.connect_catalog_rws((0.,) * 6)
        plan_id = self.preview_wall_task()
        goals = [segment.goal for segment in self.node.task_plan.trajectory.segments
                 if segment.start != segment.goal]
        self.complete_catalog_batch(plan_id, goals, reviewed_live_route=True, reviewed_cell_geometry=True)
        self.assertEqual(self.node.task_status["kind"], "cell_wall")
        self.assertIn("cell_geometry_reviewed_ns", self.node.task_status)
        self.assertEqual(self.backend.requests[-1], (0.,) * 6)

    def test_wall_v2_double_keeps_automatic_durations_and_one_token(self):
        self.configure_cell(verified=False)
        backend = self.connect_task_trajectory()
        plan_id = self.preview_wall_task()
        segments = [segment for segment in self.node.task_plan.trajectory.segments
                    if segment.start != segment.goal]
        token = self.confirm_batch(plan_id, reviewed_live_route=True, reviewed_cell_geometry=True)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        for count in range(1, len(segments) + 1):
            self.drive(lambda: len(backend.requests) == count)
            backend.complete()
        self.drive(lambda: self.node.task_status["state"] == "verified")
        self.assertEqual(backend.requests, [segment.goal for segment in segments])
        self.assertEqual([leg.duration for leg, _ in backend.legs],
                         [segment.duration for segment in segments])
        self.assertTrue(all(.1 <= segment.duration <= 30 for segment in segments))
        self.assertFalse(self.backend.requests)

    def test_air_triangle_normal_double(self):
        self.configure_cell(verified=False)
        self.connect_catalog_rws((0.,) * 6)
        plan_id = self.preview_wall_task(kind="air_triangle")
        plan = self.node.task_plan
        self.assertEqual(plan.order, ("APROX", "A", "B", "C", "CIERRE_A", "SALIDA", "HOME"))
        goals = [s.goal for s in plan.trajectory.segments if s.start != s.goal]
        self.complete_catalog_batch(plan_id, goals, reviewed_live_route=True, reviewed_cell_geometry=True)
        self.assertEqual(self.node.task_status["kind"], "air_triangle")
        self.assertEqual(self.backend.requests[-1], (0.,) * 6)

    def test_air_triangle_v2_double(self):
        self.configure_cell(verified=False)
        backend = self.connect_task_trajectory()
        plan_id = self.preview_wall_task(kind="air_triangle")
        segments = [s for s in self.node.task_plan.trajectory.segments if s.start != s.goal]
        token = self.confirm_batch(plan_id, reviewed_live_route=True, reviewed_cell_geometry=True)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        for count in range(1, len(segments)+1):
            self.drive(lambda: len(backend.requests) == count)
            backend.complete()
        self.drive(lambda: self.node.task_status["state"] == "verified")
        self.assertEqual(backend.requests, [s.goal for s in segments])
        self.assertEqual([leg.duration for leg, _ in backend.legs], [s.duration for s in segments])

    def test_catalog_joint_out_and_back_uses_a_single_batch_token(self):
        initial = (0.03, -0.2, 0.4, 0.05, -0.3, 0.07)
        goal = (*initial[:3], initial[3] + 0.04, *initial[4:])
        measured = tuple(value + 0.00005 for value in initial)
        expected_goal = (*measured[:3], goal[3], *measured[4:])
        self.connect_catalog_rws(measured)
        points = dict(INICIO=dict(q_rad=list(initial)), OBJETIVO=dict(q_rad=list(goal)))
        plan_id = self.catalog_preview(self.catalog_task("joint_test", points))
        self.assertEqual(tuple(self.node.task_status["order"]), ("OBJETIVO", "INICIO"))
        self.assertEqual(self.backend.requests, [])
        self.complete_catalog_batch(plan_id, [expected_goal, measured])
        self.assertEqual(self.node.task_status["verified_waypoints"], 2)
        self.assertEqual(self.node.task_status["verified_cycles"], 1)
        self.assertEqual(self.node.task_status["waypoint"], "INICIO")

    def test_live_joint_preview_in_observe_and_reviewed_batch_in_control(self):
        initial = (0.03, -0.2, 0.4, 0.05, -0.3, 0.07)
        task = dict(kind="joint_test_live", cycles=1, axis=4, offset_deg=-1.25)
        goal = (*initial[:3], initial[3] + radians(task["offset_deg"]), *initial[4:])

        self.connect_catalog_rws(initial, mode="observe")
        observe_id = self.catalog_preview(task)
        self.assertEqual(self.node.task_status["kind"], "joint_test_live")
        self.assertEqual(tuple(self.node.task_status["order"]), ("OBJETIVO", "INICIO"))
        self.assertEqual(tuple(self.node.task_status["start_q_rad"]), initial)
        self.assertEqual([tuple(point["q_rad"]) for point in self.node.task_status["waypoints"]],
                         [goal, initial])
        self.task_command("authorize", plan_id=observe_id, reviewed_live_route=True,
                          stamp_ns=str(self.node.get_clock().now().nanoseconds))
        self.assertNotIn("authorization_id", self.node.task_status)
        self.assertFalse(self.backend.requests)

        self.connect_catalog_rws(initial)
        self.task_command("plan", request_id="live-control", task=task)
        self.assertEqual(self.node.task_status["state"], "planned", self.node.task_status)
        control_id = self.node.task_status["plan_id"]
        self.assertNotEqual(control_id, observe_id)
        self.task_command("authorize", plan_id=control_id, reviewed_live_route=True,
                          stamp_ns=str(self.node.get_clock().now().nanoseconds))
        self.assertNotIn("authorization_id", self.node.task_status)
        self.task_command("preview", plan_id=control_id)
        self.finish_catalog_playback(apply=False)
        self.deny_catalog_authorization(control_id)
        self.assertRegex(self.node.task_status["error"].lower(), "revis|inspec|confirm")
        self.assertFalse(self.backend.requests)
        self.assertEqual(self.backend.prepare_calls, 0)

        self.complete_catalog_batch(control_id, [goal, initial], reviewed_live_route=True)
        self.assertEqual(self.node.task_status["verified_waypoints"], 2)
        self.assertEqual(self.backend.positions, initial)

    def test_live_joint_parameters_limits_and_untrusted_feedback_block_planning(self):
        task = dict(kind="joint_test_live", cycles=1, axis=1, offset_deg=1)

        def reject_plan(candidate):
            self.task_command("plan", request_id="invalid-live", task=candidate)
            self.assertEqual(self.node.task_status["state"], "invalid", self.node.task_status)
            self.assertTrue(self.node.task_status.get("error"))
            self.assertIsNone(self.node.task_plan)
            self.assertIsNone(self.node.playback)
            self.assertFalse(self.backend.requests)

        offline = self.catalog_preview(dict(task, offset_deg=-15))
        self.assertEqual(self.node.task_status["source"], "virtual")
        self.assertNotIn("measured_stamp_ns", self.node.task_status["joint_test"])
        fast_duration = self.node.task_status["planned_duration_s"]
        self.deny_catalog_authorization(offline)
        self.catalog_preview(dict(task, offset_deg=-15, velocity_percent=1))
        self.assertAlmostEqual(self.node.task_status["planned_duration_s"], fast_duration)
        self.assertTrue(self.node.task_status["joint_test"]["automatic_velocity"])
        self.connect_catalog_rws((0.03, -0.2, 0.4, 0.05, -0.3, 0.07), mode="observe")
        for changes in (dict(axis=0), dict(axis=7), dict(axis=True), dict(axis=1.5),
                        dict(offset_deg=0), dict(offset_deg=500), dict(offset_deg=-500),
                        dict(offset_deg="1"), dict(cycles=2),
                        dict(velocity_percent=0), dict(velocity_percent=101), dict(velocity_percent=True)):
            with self.subTest(changes=changes):
                reject_plan(dict(task, **changes))

        q, stamp, received, _ = self.node.measured
        with patch.object(self.node, "measured", (q, stamp, received, "external")):
            reject_plan(task)
        with patch.object(self.node, "measured", (q, stamp, monotonic() - 5, "rws")):
            reject_plan(task)

        for bound, delta in ((self.node.model.limits[0][0], -1), (self.node.model.limits[0][1], 1)):
            with self.subTest(bound=bound, delta=delta):
                self.backend.positions = (bound, *q[1:])
                self.drive(lambda: self.node.measured[0] == self.backend.positions)
                reject_plan(dict(task, offset_deg=delta))

    def connect_task_trajectory(self):
        backend = TrajectoryDouble()
        self.node.backends.factories["rws_trajectory"] = lambda: backend
        self.node.controller_config = dict(endpoint="http://192.0.2.1", task="T_ROB1",
                                           module="Imt342JointServer", mechunit="ROB_1")
        self.mode("observe")
        self.node.on_backend(String(data="rws_trajectory"))
        self.assertEqual(backend.prepare_calls, 0)
        self.assertEqual(backend.requests, [])
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.mode("control")
        self.drive(lambda: self.node.backends.can_arm and self.node.task_feedback_ready()
                   and self.node.irc5_connection["state"] == "ready")
        return backend

    def test_preprogrammed_cycle_normal_without_catalog_file(self):
        initial = (0.03, -0.2, 0.4, 0.05, -0.3, 0.07)
        self.connect_catalog_rws(initial)
        plan_id = self.catalog_preview(dict(kind="demo_cycle_live", cycles=1, profile="lspb"))
        self.assertEqual(self.node.task_status["kind"], "demo_cycle_live")
        self.assertEqual(tuple(self.node.task_status["order"]), ("A", "B", "C", "HOME"))
        self.assertFalse(self.node.task_status["physical_reviewed"])
        self.deny_catalog_authorization(plan_id)
        d = radians(3)
        goals = [(initial[0] + d, *initial[1:]),
                 (initial[0] + d, initial[1] + d, *initial[2:]),
                 (initial[0], initial[1] + d, *initial[2:]), initial]
        self.complete_catalog_batch(plan_id, goals, reviewed_live_route=True)
        self.assertEqual(self.node.task_status["verified_waypoints"], 4)

    def test_preprogrammed_cycle_v2_without_catalog_file(self):
        backend = self.connect_task_trajectory()
        plan_id = self.catalog_preview(dict(kind="demo_cycle_live", cycles=1, profile="quintic"))
        expected_leg_duration = (60 * radians(3) / 5) ** (1 / 3)
        self.assertAlmostEqual(self.node.task_status["planned_duration_s"], 4 * expected_leg_duration)
        self.deny_catalog_authorization(plan_id)
        token = self.confirm_batch(plan_id, reviewed_live_route=True)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        for count in range(1, 5):
            self.drive(lambda: len(backend.requests) == count)
            backend.complete()
        self.drive(lambda: self.node.task_status["state"] == "verified")
        d = radians(3)
        self.assertEqual(backend.requests, [(d, 0, 0, 0, 0, 0), (d, d, 0, 0, 0, 0),
                                           (0, d, 0, 0, 0, 0), (0,) * 6])
        for leg, _ in backend.legs:
            self.assertAlmostEqual(leg.duration, expected_leg_duration)
        self.assertFalse(self.backend.requests)

    def execute_selected_ik_branch(self, backend):
        q = (0.06, -0.12, 0.18, 0.08, -0.16, 0.04)
        matrix = self.node.points.tcp.fk(q)
        self.node.on_pose_plan(String(data=json.dumps(dict(matrix=matrix, request_id="ik-direct"))))
        self.assertTrue(self.node.pose_routes, self.node.route_result)
        branch, segment = min(self.node.pose_routes.items(), key=lambda pair: sum(
            abs(a - b) for a, b in zip(pair[1].goal, q)))
        self.node.on_pose_preview(String(data=json.dumps(dict(request_id="ik-direct", branch=branch))))
        self.node.finish_preview("completed", "Previsualización completada")
        fk_before = self.node.target
        self.node.on_physical_command(self.arm_message(f"ik:ik-direct:{branch}"))
        self.assertTrue(self.node.backends.armed)
        order = self.node.state(segment.goal)
        order.header.frame_id = f"imt342/ik/ik-direct/{branch}"
        self.node.on_execute_target(order)
        self.drive(lambda: bool(backend.requests))
        self.assertEqual(backend.requests, [segment.goal])
        self.assertEqual(self.node.individual_trial["ik"]["branch"], branch)
        self.assertEqual(self.node.target, fk_before)
        backend.complete()
        self.drive(lambda: self.node.individual_trial["finished"])
        self.assertEqual(self.node.individual_trial["result"], "verified")

    def test_direct_ik_normal(self):
        self.connect_catalog_rws((0.,) * 6)
        self.execute_selected_ik_branch(self.backend)

    def test_direct_ik_v2(self):
        self.execute_selected_ik_branch(self.connect_task_trajectory())

    def test_trajectory_task_sends_reviewed_quintic_legs_in_exact_order(self):
        backend = self.connect_task_trajectory()
        self.node.on_analysis_command(String(data='{"trajectory_profile":"quintic"}'))
        plan_id = self.catalog_preview(
            dict(kind="joint_test_live", cycles=1, axis=1, offset_deg=12, duration_s=2))
        self.assertEqual(self.node.task_status["reference_profile"], "quintic")
        self.assertEqual(self.node.task_status["planned_duration_s"], 4)
        self.assertIsNone(self.node.task_status["planned_lspb_s"])
        self.assertEqual(backend.requests, [])

        token = self.confirm_batch(plan_id, reviewed_live_route=True)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        for count in (1, 2):
            self.drive(lambda: len(backend.requests) == count)
            backend.complete()
        self.drive(lambda: self.node.task_status["state"] == "verified")
        self.assertEqual(backend.requests, [(radians(12), 0, 0, 0, 0, 0), (0,) * 6])
        self.assertEqual([prior for _, prior in backend.legs], [None, 1])
        self.assertEqual([leg.duration for leg, _ in backend.legs], [2, 2])
        self.assertTrue(all(getattr(leg, "profiles", (getattr(leg, "profile", None),)) == ("quintic",)
                            for leg, _ in backend.legs))
        self.assertEqual(self.backend.requests, [])

    def test_trajectory_batch_uncertainty_blocks_remaining_legs_and_allows_stop(self):
        backend = self.connect_task_trajectory()
        plan_id = self.catalog_preview(dict(kind="joint_test_live", cycles=1, axis=4, offset_deg=2.5))
        token = self.confirm_batch(plan_id, reviewed_live_route=True)
        self.task_command("execute", plan_id=plan_id, authorization_id=token)
        self.drive(lambda: len(backend.requests) == 1)
        backend.result = Result.UNKNOWN
        self.drive(lambda: self.node.task_status["state"] == "unknown")
        self.assertTrue(self.node.task_status["stop_available"])
        self.task_command("cancel", plan_id=plan_id)
        self.drive(lambda: self.node.task_status["state"] == "cancelled")
        self.assertEqual(len(backend.requests), 1)

    def test_catalog_joint_needs_exact_review_and_matching_measured_origin(self):
        initial = (0.03, -0.2, 0.4, 0.05, -0.3, 0.07)
        target = (initial[0] + 0.04, *initial[1:])
        self.connect_catalog_rws(initial)
        points = dict(INICIO=dict(q_rad=list(initial)), OBJETIVO=dict(q_rad=list(target)))
        draft = dict(kind="joint_test", cycles=1, waypoints=points)
        self.task_command("plan", request_id="empty-draft", task=dict(
            draft, waypoints=dict(INICIO={}, OBJETIVO={})))
        self.assertEqual(self.node.task_status["state"], "invalid")
        self.assertIsNone(self.node.task_plan)
        self.deny_catalog_authorization(self.catalog_preview(draft))

        task = self.catalog_task("joint_test", points)
        revised = dict(points, OBJETIVO=dict(q_rad=[target[0] + 0.02, *target[1:]]))
        self.deny_catalog_authorization(self.catalog_preview(dict(task, waypoints=revised)))

        self.backend.positions = (initial[0] + 0.01, *initial[1:])
        self.drive(lambda: self.node.measured[0] == self.backend.positions)
        self.deny_catalog_authorization(self.catalog_preview(task))
        self.assertIn("INICIO", self.node.task_status["physical_block_reason"])
        self.assertFalse(self.backend.requests)

    def test_custom_physical_review_is_bound_to_waypoint_order(self):
        self.connect_catalog_rws((0.0,) * 6)
        waypoints = {"P1": dict(q_rad=[0.03, 0, 0, 0, 0, 0]),
                     "P2": dict(q_rad=[0.06, 0, 0, 0, 0, 0])}
        config = dict(self.catalog_task("custom", waypoints),
                      order=["P2", "P1"], physical_review_order=["P1", "P2"])
        self.deny_catalog_authorization(self.catalog_preview(config))
        self.assertIn("orden", self.node.task_status["physical_block_reason"])
        plan_id = self.catalog_preview(dict(config, physical_review_order=["P2", "P1"]))
        self.complete_catalog_batch(plan_id, [(0.06, 0, 0, 0, 0, 0), (0.03, 0, 0, 0, 0, 0)])

    def test_catalog_repeat_requires_verified_single_cycle_and_exact_route_context(self):
        self.node.task_cycle_receipt_dir = Path(self.enterContext(TemporaryDirectory()))
        home = (0.03, -0.2, 0.4, 0.05, -0.3, 0.07)
        goals = [(home[0] + offset, *home[1:]) for offset in (0.03, 0.06, 0.08, 0.0)]
        waypoints = {name: dict(q_rad=list(q)) for name, q in zip(("A", "B", "C", "HOME"), goals)}
        cycle = self.catalog_task("cycle", waypoints)
        plan_id = self.catalog_preview(cycle)
        self.task_command("simulate", plan_id=plan_id)
        self.finish_catalog_playback(apply=True)
        self.assertEqual(self.node.task_status["state"], "simulated")
        self.assertEqual(self.node.virtual, home)
        self.assertFalse(list(self.node.task_cycle_receipt_dir.iterdir()))

        self.connect_catalog_rws(home)
        repeat = self.catalog_task("repeat", waypoints, cycles=2)
        self.deny_catalog_authorization(self.catalog_preview(repeat, repeat_validated=False))
        self.assertEqual(self.backend.requests, [])

        single_id = self.catalog_preview(self.catalog_task("cycle", waypoints))
        first_token = self.complete_catalog_batch(single_id, goals)
        self.assertTrue(list(self.node.task_cycle_receipt_dir.iterdir()))
        self.node.task_cycle_receipts.clear()
        changed = dict(waypoints, A=dict(q_rad=[goals[0][0] + 0.01, *goals[0][1:]]))
        self.deny_catalog_authorization(self.catalog_preview(
            self.catalog_task("repeat", changed, cycles=2), repeat_validated=False))
        changed_timing = dict(waypoints, A=dict(waypoints["A"], duration_s=1.5))
        self.deny_catalog_authorization(self.catalog_preview(
            self.catalog_task("repeat", changed_timing, cycles=2), repeat_validated=False))

        changes = {
            "model_sha256": "a" * 64,
            "controller_config": dict(self.node.controller_config, endpoint="http://192.0.2.2"),
            "points": self.node.points.with_tool(coordinator.pose_from_rpy((0.01, 0, 0), (0, 0, 0))),
        }
        for field, value in changes.items():
            with self.subTest(field=field), patch.object(self.node, field, value):
                self.deny_catalog_authorization(self.catalog_preview(
                    self.catalog_task("repeat", waypoints, cycles=2), repeat_validated=False))

        repeat_id = self.catalog_preview(repeat, repeat_validated=True)
        self.assertEqual(self.node.task_status["verified_cycle_plan_id"], single_id)
        self.task_command("execute", plan_id=repeat_id, authorization_id=first_token)
        self.assertIsNone(self.node.batch)
        self.complete_catalog_batch(repeat_id, goals * 2)
        self.assertEqual(self.node.task_status["verified_waypoints"], 8)
        self.assertEqual(self.node.task_status["verified_cycles"], 2)

    def test_applied_virtual_derivatives_share_the_sample_and_clock(self):
        self.node.analysis_out = Mock()
        goals = [(0.5, -.4, .3, 0., 0., 0.)]

        def packet():
            self.node.publish_analysis()
            return json.loads(self.node.analysis_out.publish.call_args.args[0].data)

        for apply in (False, True):
            self.node.start_playback(apply, goals=goals)
            trajectory, _, _, source = self.node.playback
            duration = trajectory.duration
            for progress in (.125, .5, .875):
                self.node.playback = trajectory, monotonic() - progress * duration, apply, source
                self.node.tick()
                row = packet()
                self.assertEqual(row["virtual_motion"]["applied"], apply)
                if apply:
                    self.assertEqual(row["actual"], row["plan"]["q"])
                    self.assertEqual(row["virtual_motion"]["qdot"], row["plan"]["profile_qdot"])
                    self.assertEqual(row["virtual_motion"]["qddot"], row["plan"]["profile_qddot"])
                    qddot = row["virtual_motion"]["qddot"][0]
                    blend = trajectory.segments[0].blend_fraction
                    peak = 0.5 / (blend * (1 - blend) * duration**2)
                    analytical = {.125: peak, .5: 0, .875: -peak}
                    self.assertAlmostEqual(qddot, analytical[progress], places=10)
                else:
                    self.assertEqual(row["actual"], [0.0] * 6)
                    self.assertEqual(row["virtual_motion"]["qdot"], [0.0] * 6)
            self.node.playback = trajectory, monotonic() - duration - 1., apply, source
            self.node.tick()
            self.assertEqual(packet()["virtual_motion"]["qddot"], [0.0] * 6)

    def test_task_simulation_starts_after_completed_preview_and_finishes_at_home(self):
        def command(action, **fields):
            self.node.on_task_command(String(data=json.dumps(dict(action=action, **fields))))

        goals = {name: dict(q_rad=[i * 0.1, 0, 0, 0, 0, 0])
                 for i, name in enumerate(("A", "B", "C", "HOME"), 1)}
        command("plan", request_id="example",
                task=dict(cycles=2, waypoints=goals, comparison=True))
        comparison = self.node.task_status["comparison"]
        self.assertGreaterEqual(comparison["reduced_lspb_s"], comparison["reference_lspb_s"])
        plan_id = self.node.task_status["plan_id"]
        command("simulate", plan_id=plan_id)
        self.assertIsNone(self.node.playback)
        command("preview", plan_id=plan_id)
        trajectory, _, apply, source = self.node.playback
        self.assertFalse(apply)
        self.node.playback = (trajectory, monotonic() - trajectory.duration, apply, source)
        self.node.tick()
        self.assertTrue(self.node.task_status["previewed"])
        self.assertEqual(self.node.virtual, (0.0,) * 6)
        command("simulate", plan_id=plan_id)
        trajectory, _, apply, source = self.node.playback
        self.assertTrue(apply)
        self.node.playback = (trajectory, monotonic() - trajectory.duration, apply, source)
        self.node.tick()
        self.assertEqual(self.node.task_status["state"], "simulated")
        self.assertEqual(self.node.task_status["verified_waypoints"], 8)
        self.assertEqual(self.node.virtual, tuple(goals["HOME"]["q_rad"]))
        self.assertEqual(self.backend.requests, [])

    def connect_irc5(self):
        self.node.on_rapid_prepare(String(data="connect"))

    def test_irc5_connect_prepares_once_and_preserves_individual_authorization(self):
        self.backend.rapid_running = False
        self.backend.rapid_pointer = ("Other", "main")
        self.connect_irc5()
        self.connect_irc5()
        self.assertEqual((self.node.mode, self.node.backends.selected), ("observe", "rws"))
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.connect_irc5()
        self.assertEqual(self.backend.prepare_calls, 1)
        self.assertEqual(self.node.backends.rapid_preparation["state"], "started")
        self.assertGreater(self.node.measured[2], self.node.irc5_verified_after)
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.backend.requests, [])
        self.mode("control")
        self.node.update_irc5_connection()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.backend.requests, [])

    def test_irc5_connect_prepares_normal_from_stopped_main_module(self):
        self.backend.rapid_running = False
        self.backend.rapid_pointer = ("MainModule", "main")
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.backend.rapid_pointer, ("Imt342JointServer", "ServeJointOrders"))
        self.assertEqual(self.backend.prepare_calls, 1)
        self.assertFalse(self.backend.requests)

    def test_irc5_already_running_and_disconnect_never_starts_again(self):
        self.mode("observe")
        self.node.on_backend(String(data="rws"))
        self.drive(lambda: self.node.irc5_connection["state"] == "connected")
        self.assertEqual(self.backend.prepare_calls, 0)
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.node.backends.rapid_preparation["state"], "already_running")
        self.backend.connected = False
        self.drive(lambda: self.node.irc5_connection["state"] == "error")
        self.backend.connected = True
        self.drive(lambda: self.node.backends.connected)
        self.assertEqual(self.node.irc5_connection["state"], "error")
        self.assertEqual(self.backend.prepare_calls, 1)
        self.assertEqual(self.backend.requests, [])

    def test_irc5_needs_new_measurement_after_prepare(self):
        self.backend.lose_feedback_after_prepare = True
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "verifying")
        self.assertLess(self.node.measured[2], self.node.irc5_verified_after)
        self.connect_irc5()
        self.assertEqual(self.backend.prepare_calls, 1)
        self.node.irc5_deadline = monotonic() - 1
        self.node.update_irc5_connection()
        self.assertEqual(self.node.irc5_connection["state"], "error")
        self.assertEqual(self.backend.requests, [])

    def test_irc5_authentication_failure_and_foreign_pointer_do_not_start(self):
        self.backend.http_status = 401
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "error")
        self.assertIn("Autenticación", self.node.irc5_connection["detail"])
        self.assertEqual(self.backend.prepare_calls, 0)
        self.backend.http_status = None
        self.backend.rapid_pointer = ("Other", "main")
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "foreign_program")
        self.assertIn("PP fuera", self.node.irc5_connection["detail"])
        self.assertEqual(self.backend.prepare_calls, 1)
        self.assertEqual(self.backend.rapid_pointer, ("Other", "main"))
        self.assertEqual(self.backend.requests, [])

    def test_irc5_missing_module_confirmation_is_bound_and_single_use(self):
        self.backend.module_missing = True
        self.backend.rapid_running = False
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "module_missing")
        recovery_id = self.node.irc5_connection["recovery_id"]
        self.node.on_rapid_prepare(String(data="recover:otro"))
        self.assertEqual(self.backend.recovery_calls, 0)
        self.node.on_rapid_prepare(String(data="recover:" + recovery_id))
        self.node.on_rapid_prepare(String(data="recover:" + recovery_id))
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.backend.recovery_calls, 1)
        self.assertEqual(self.backend.requests, [])
        self.assertFalse(self.node.backends.armed)

    def test_irc5_default_offline_refers_to_fedora_launcher(self):
        self.node.backends.factories.clear()
        self.connect_irc5()
        self.assertEqual(self.node.mode, "offline")
        self.assertEqual(self.node.irc5_connection["state"], "error")
        self.assertIn("./scripts/irc5_demo.sh observe", self.node.irc5_connection["detail"])
        self.assertEqual(self.backend.prepare_calls, 0)

    def test_rws_velocity_uses_monotonic_reception_and_drops_stale_windows(self):
        self.node.on_backend(String(data="rws"))
        self.mode("observe")
        start = monotonic()
        stamp = self.node.get_clock().now().to_msg()
        self.assertIsNone(self.node.measured_velocity(False))
        for i in range(5):
            self.node.record_feedback((i * 0.02,) * 6, stamp, "rws", original_stamp=False,
                                      received_at=start + i * 0.1)
        self.assertAlmostEqual(self.node.measured_velocity(False)[0], 0.2, places=6)
        self.assertAlmostEqual(self.node.feedback_frequency_hz, 10.0, places=5)
        self.assertIsNone(self.node.measured_velocity(True))
        self.node.record_feedback((0.0,) * 6, stamp, "rws", original_stamp=False,
                                  received_at=start + 2.0)
        self.assertIsNone(self.node.measured_velocity(False))

    def test_estimated_acceleration_requires_fresh_same_source_samples(self):
        start = monotonic()
        stamp = self.node.get_clock().now().to_msg()
        for dt in (0., .04, .09, .13, .2):
            self.node.record_feedback((dt ** 2,) * 6, stamp, "rws_trajectory", original_stamp=False,
                                      received_at=start + dt)
        for estimate in self.node.estimated_acceleration(False):
            self.assertAlmostEqual(estimate, 2., places=6)
        self.assertIsNone(self.node.estimated_acceleration(True))
        self.node.record_feedback((0.,) * 6, stamp, "rws", original_stamp=False, received_at=start + .3)
        self.assertIsNone(self.node.estimated_acceleration(False))

    def test_target_fk_updates_without_review_request(self):
        self.node.analysis_out = Mock()
        for q in ((.1, -.2, .5, .3, -.4, .1), (.2, -.1, .4, .2, -.3, .2)):
            self.node.on_target(self.node.state(q))
            self.node.publish_analysis()
            packet = json.loads(self.node.analysis_out.publish.call_args.args[0].data)
            self.assertEqual(packet["target"], list(q))
            expected = json.loads(json.dumps(self.node.points.tcp.fk(q)))
            self.assertEqual(packet["target_fk"]["tcp"]["matrix"], expected)

    def test_unconfigured_v2_is_available_for_offline_lspb_fk_preview(self):
        self.node.analysis_out = Mock()
        self.node.on_backend(String(data="rws_trajectory"))
        self.assertEqual(self.node.backends.selected, "rws_trajectory")
        self.mode("control")
        self.assertEqual(self.node.mode, "offline")
        self.node.start_playback(False, goals=[(.3, -.4, 0., 0., 0., 0.)])
        self.node.publish_analysis()
        payload = json.loads(self.node.analysis_out.publish.call_args.args[0].data)
        self.assertEqual(payload["plan"]["profile_id"], "lspb")
        self.assertIsNotNone(payload["plan"]["blend_s"])
        self.assertEqual(payload["actual"], [0.] * 6)
        self.assertFalse(payload["physical_can_arm"])
        self.node.on_backend(String(data="unknown"))
        self.assertEqual(self.node.backends.selected, "rws_trajectory")
        self.mode("observe")
        self.assertEqual(self.node.mode, "observe")
        self.assertFalse(self.node.backends.connected)
        self.assertNotIn("rws_trajectory", self.node.backends.clients)
        self.node.on_backend(String(data="rws"))
        self.drive(lambda: self.node.backends.connected)
        self.assertFalse(self.backend.requests)

    def test_v2_control_mode_is_selectable_but_unprepared_motion_is_blocked(self):
        backend = TrajectoryDouble()
        self.node.backends.factories["rws_trajectory"] = lambda: backend
        self.mode("observe")
        self.node.on_backend(String(data="rws_trajectory"))
        self.drive(lambda: self.node.backends.connected)
        self.mode("control")
        self.assertEqual(self.node.mode, "control")
        self.assertFalse(self.node.trajectory_motion_enabled())
        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.order())
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(backend.requests, [])
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.node.mode, "observe")
        self.mode("control")
        self.drive(lambda: self.node.backends.can_arm and self.node.irc5_connection["state"] == "ready")
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.node.planning_profile(), "lspb")

    def test_shared_arrival_band_reports_error_without_completing_order(self):
        self.node.target = (0.,) * 6
        goal = (0.1,) * 6
        for backend in ("rws", "rws_trajectory"):
            for profile in ("lspb", "quintic"):
                with self.subTest(backend=backend, profile=profile):
                    self.node.backends.selected = backend
                    self.node.selected_profile = profile
                    measured = (goal[0] + radians(.2), *goal[1:])
                    record = self.node.arrival_record(measured, False, {"q": goal},
                        {"motion_evidence": {"awaiting_terminal_reply": True}})
                    self.assertTrue(record["within_band"])
                    self.assertTrue(record["awaiting_terminal_reply"])
                    self.assertAlmostEqual(record["max_error_rad"], radians(.2))
                    outside = (goal[0] + radians(.31), *goal[1:])
                    self.assertFalse(self.node.arrival_record(outside, False, {"q": goal}, {})["within_band"])
                    self.assertIsNone(self.node.arrival_record(measured, True, {"q": goal}, {}))

    def test_connect_and_prepare_switches_between_rws_servers_without_motion(self):
        backend = TrajectoryDouble()
        self.node.backends.factories["rws_trajectory"] = lambda: backend
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.backend.prepare_calls, 1)

        self.node.on_backend(String(data="rws_trajectory"))
        self.assertEqual(backend.prepare_calls, 0)
        self.connect_irc5()
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(backend.prepare_calls, 1)
        self.assertTrue(self.node.trajectory_motion_enabled())
        self.assertFalse(self.node.backends.armed)

        self.node.on_backend(String(data="rws"))
        self.assertEqual(self.backend.prepare_calls, 1)
        self.connect_irc5()
        self.drive(lambda: self.node.irc5_connection["state"] == "ready")
        self.assertEqual(self.backend.prepare_calls, 2)
        self.assertEqual(self.node.backends.selected, "rws")
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(self.backend.requests + backend.requests, [])

    def test_trajectory_dispatch_requires_reviewed_single_segment_and_current_target(self):
        backend = self.connect_task_trajectory()
        self.node.on_physical_command(self.arm_message())
        self.assertFalse(self.node.backends.armed)
        # Availability alone is not a reviewed command.
        self.node.on_execute_target(self.order())
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(backend.requests, [])
        self.node.on_preview_target(self.order())
        segment = self.node.plan_trajectory
        self.node.finish_preview("completed", "Previsualización completada")

        self.node.on_physical_command(self.arm_message())
        self.assertTrue(self.node.backends.armed)
        self.node.on_execute_target(self.node.state((0.1,) + (0.0,) * 5))
        self.assertFalse(self.node.backends.armed)
        self.assertEqual(backend.requests, [])

        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.order())
        self.drive(lambda: bool(backend.requests))
        self.assertEqual(backend.requests, [tuple(self.order().position)])
        self.assertEqual(backend.motion_evidence["requested_duration_s"], segment.duration)
        self.assertEqual(self.node.individual_trial["reference_profile"], "lspb")
        backend.complete()
        self.drive(lambda: self.node.individual_trial["finished"])
        self.assertEqual(self.node.individual_trial["result"], "verified")
        self.assertEqual(self.backend.requests, [])
        # The preview was cleared by execution, but the independent physical
        # reference and final sample are still available for the chart.
        self.assertIsNone(self.node.plan_trajectory)
        physical = self.node.physical_analysis_record(monotonic(), self.node.measured[0], False)
        self.assertEqual(physical["id"], self.node.individual_trial["trial_id"])
        self.assertTrue(physical["finished"])
        self.assertEqual(physical["reference"]["sequence"], 1)
        self.assertEqual(physical["backend"], "rws_trajectory")

    def test_v2_lspb_uses_same_duration_contract(self):
        backend = self.connect_task_trajectory()
        self.node.on_analysis_command(String(data='{"trajectory_profile":"lspb"}'))
        self.node.on_preview_target(self.order())
        self.node.finish_preview("completed", "Previsualización completada")
        self.node.on_physical_command(self.arm_message())
        self.assertTrue(self.node.backends.armed)
        self.node.on_execute_target(self.order())
        self.drive(lambda: bool(backend.legs))
        trajectory, _ = backend.legs[0]
        self.assertEqual(trajectory.profiles, ("lspb",))
        self.assertEqual(self.node.individual_trial["requested_duration_s"], trajectory.duration)
        self.assertEqual(self.node.individual_trial["timing"], "requested_only")

    def test_fk_rearms_after_terminal_and_drift_is_distinct_from_arrival(self):
        for name in ("rws", "rws_trajectory"):
            with self.subTest(backend=name):
                if name == "rws":
                    self.connect_catalog_rws((0.0,) * 6)
                    backend = self.backend
                else:
                    backend = self.connect_task_trajectory()
                first = (.08, 0., 0., 0., 0., 0.)
                second = (.12, 0., 0., 0., 0., 0.)
                self.review_fk(first)
                backend.positions = (radians(.02), 0., 0., 0., 0., 0.)
                self.drive(lambda: self.node.measured[0] == backend.positions)
                self.node.on_physical_command(self.arm_message())
                self.assertTrue(self.node.backends.armed)
                self.node.on_execute_target(self.node.state(second))
                self.assertFalse(self.node.backends.armed)
                self.assertEqual(backend.requests, [])

                # A real change requires replanning even within the arrival band.
                backend.positions = (radians(.2), 0., 0., 0., 0., 0.)
                self.drive(lambda: self.node.measured[0] == backend.positions)
                self.node.on_physical_command(self.arm_message())
                self.assertFalse(self.node.backends.armed)
                for count, goal in enumerate((first, second), 1):
                    self.review_fk(goal)
                    self.node.on_physical_command(self.arm_message())
                    self.assertTrue(self.node.backends.armed, self.node.operation)
                    self.node.on_execute_target(self.node.state(goal))
                    self.drive(lambda: len(backend.requests) == count)
                    self.assertNotEqual(self.node.backends.owner, None)
                    self.assertFalse(self.node.individual_trial["finished"])
                    backend.complete()
                    self.drive(lambda: self.node.individual_trial["finished"] and self.node.backends.can_arm)
                self.assertEqual(backend.requests, [first, second])

    def test_v2_uses_the_reviewed_duration_and_physical_clock(self):
        backend = self.connect_task_trajectory()
        self.assertTrue(self.node.timed_motion_available())
        self.node.on_analysis_command(String(data='{"trajectory_duration_s":20.0}'))
        self.assertIsNone(self.node.reference_duration)
        self.review_fk()
        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.order())
        self.drive(lambda: bool(backend.requests) and self.node.individual_trial.get("motion"))
        trial = self.node.individual_trial
        self.assertEqual(trial["requested_duration_s"], trial["reference_duration_s"])
        self.assertLess(trial["reference_duration_s"], 20.0)
        motion = trial["motion"]
        self.assertEqual(motion["requested_duration_s"], trial["reference_duration_s"])
        run = self.node.physical_analysis_record(monotonic(), self.node.measured[0], False)
        self.assertEqual(run["reference"]["origin"], "rapid_phase_2_observed")
        self.assertAlmostEqual(run["reference"]["origin_session_s"] -
                               run["reference"]["submit_session_s"], .02)
        backend.complete()
        self.drive(lambda: self.node.individual_trial["finished"] and self.node.backends.can_arm)

    def test_already_satisfied_fk_does_not_dispatch_a_motion(self):
        self.connect_catalog_rws((0.0,) * 6)
        self.review_fk((0.0,) * 6)
        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.node.state((0.0,) * 6))
        self.assertEqual(self.node.individual_trial["result"], "already_satisfied")
        self.assertFalse(self.node.individual_trial["motion"]["motion_sent"])
        self.assertFalse(self.backend.requests)
        self.assertIsNone(self.node.backends.owner)

    def test_terminal_demo_can_be_planned_again(self):
        self.connect_catalog_rws((0.0,) * 6)
        task = dict(kind="demo_cycle_live", cycles=1)
        plan_id = self.catalog_preview(task)
        d = radians(3)
        self.complete_catalog_batch(plan_id, [(d, 0, 0, 0, 0, 0), (d, d, 0, 0, 0, 0),
                                            (0, d, 0, 0, 0, 0), (0,) * 6],
                                    reviewed_live_route=True)
        second = self.catalog_preview(task)
        self.assertNotEqual(plan_id, second)
        self.assertTrue(self.node.task_physical_eligible())
        self.assertEqual(self.node.task_status["waypoints"][-1]["q_rad"], (0.0,) * 6)

    def test_task_plan_rejection_correlates_while_an_order_is_pending(self):
        self.connect_catalog_rws((0.0,) * 6)
        self.review_fk()
        self.node.on_physical_command(self.arm_message())
        self.node.on_execute_target(self.order())
        self.drive(lambda: bool(self.backend.requests))
        self.task_command("plan", request_id="blocked", task=dict(kind="demo_cycle_live", cycles=1))
        rejection = self.node.task_plan_reply
        self.assertEqual(rejection["request_id"], "blocked")
        self.assertEqual(rejection["state"], "invalid")
        self.assertIn("anterior", rejection["error"])

    def test_cycles_and_terminal_analysis(self):
        self.node.analysis_out = Mock()
        sequence = ([.2, -.3, .7, .25, -.7, .3], [.35, -.4, .6, .2, -.6, .2],
                    [.18, -.25, .55, .12, -.42, .17], [0.0] * 6)

        def packet():
            self.node.publish_analysis()
            return json.loads(self.node.analysis_out.publish.call_args.args[0].data)

        request = JointTrajectory(
            joint_names=list(coordinator.JOINT_NAMES),
            points=[JointTrajectoryPoint(positions=goal) for goal in sequence * 3],
        )
        request.header.frame_id = "imt342/offline_cycles/3/4"
        invalid = JointTrajectory(joint_names=request.joint_names, header=request.header,
                                  points=request.points[:-1])
        self.node.on_waypoints(invalid)
        rejected = packet()["incident"]
        self.assertEqual(rejected["code"], "CICLO_INVALIDO")
        self.node.on_target(self.node.state((0.0, .34906585, 0.0, 0.0, 0.0, 0.0)))
        self.node.on_waypoints(request)
        active = packet()
        self.assertIsNone(active["incident"])
        self.assertEqual(active["incident_resolution"]["timestamp_ns"], rejected["timestamp_ns"])
        self.assertEqual(active["plan"]["cycle_index"], 1)
        self.assertEqual(len(active["plan"]["waypoint_times_s"]), 12)

        self.node.on_pose_preview(String(data="pause"))
        paused = packet()["plan"]
        self.assertEqual(paused["phase"], "paused")
        self.assertEqual(paused["qdot"], [0] * 6)
        self.assertEqual(paused["qddot"], [0] * 6)
        self.node.on_pose_preview(String(data="resume"))
        trajectory, _, apply, source = self.node.playback
        self.node.playback = (trajectory, monotonic() - trajectory.duration, apply, source)
        self.node.tick()
        ended = packet()
        self.assertEqual(ended["plan"]["phase"], "completed")
        self.assertEqual(ended["plan"]["cycle_index"], 3)
        self.assertEqual(ended["actual"], ended["preview"])
        self.assertEqual(ended["preview"], ended["plan"]["final_q"])
        self.assertEqual(ended["target"][1], .34906585)
        self.assertEqual(ended["plan"]["qddot"], [0] * 6)
        self.assertEqual(ended["plan"]["remaining_joint_rad"], [0] * 6)
        self.assertEqual(ended["preview_end"]["run_id"], active["preview_run_id"])

        self.node.on_waypoints(request)
        self.mode("observe")
        interrupted = packet()
        self.assertIsNone(interrupted["plan"])
        self.assertEqual(interrupted["preview_end"]["phase"], "interrupted")
        self.assertEqual(self.backend.requests, [])


if __name__ == "__main__":
    main()
