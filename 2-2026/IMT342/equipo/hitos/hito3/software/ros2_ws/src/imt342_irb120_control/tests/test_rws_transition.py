"""Exercise the RViz preparation and v2 batch entry points with synthetic RWS."""

from ast import literal_eval
from math import degrees, radians
from os import environ
from pathlib import Path
from time import monotonic, sleep
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
from xml.sax.saxutils import escape

from imt342_irb120_control.adapters import ARRIVAL_TOLERANCE_RAD, configured_backends
from imt342_irb120_control.batch import BatchRun
from imt342_irb120_control.rws_backend import EXECUTION, MASTER, MODULES, TASKS
from imt342_irb120_control.rws_transition import NORMAL, NORMAL_ENTRY, STOP_FORM, V2_ENTRY
from imt342_irb120_control.state import Result
from imt342_irb120_control.supervisor import Supervisor
from imt342_irb120_planning import plan_sequence
from test_rws_trajectory import CONFIG, ORDER, START as ZERO_AXES, STOP, ZERO, state
from test_rws_trajectory_install import InstallHttp, LOAD

START = EXECUTION + "?action=start"
TASK = f"{TASKS}/T_ROB1"
PP = TASK + "/pcp?action=set-pp-routine"
BUILD = TASK + "?action=build"
ABORT = TASK + "?action=abortexeclevel"
NORMAL_SYMBOLS = f"/rw/rapid/symbol/data/RAPID/T_ROB1/{NORMAL}"
NORMAL_ORDER = NORMAL_SYMBOLS + "/pcOrder?action=set"


class TransitionHttp(InstallHttp):
    def __init__(self, trajectory, normal):
        super().__init__(trajectory)
        self.normal_source = normal
        self.module_present = True
        self.execution = "running"
        self.apply_pointer = True
        self.lost_stop = False
        self.lost_start = False
        self.build_errors = False
        self.close_count = 0
        self.level = "normal"
        self.normal_present = True
        self.stop_effective = True
        self.normal_protocol = 1
        self.discard_on_start = True
        self.normal_order = f"[4,{ZERO}]"

    def request(self, method, path, body):
        if method == "GET" and path.startswith(NORMAL_SYMBOLS + "/"):
            symbol = path.partition("?")[0].rsplit("/", 1)[1]
            fields = {"pcOrder": self.normal_order, "rbReply": self.normal_reply,
                      "rbVersion": str(self.normal_protocol)}
            if symbol in fields:
                return state(value=fields[symbol])
        if method == "POST" and path == NORMAL_ORDER:
            self.calls.append((method, path, body))
            assert self.mastership and self.execution == "running" and self.module == NORMAL
            self.normal_order = body["value"]
            return 204, ""
        if method == "GET" and path == EXECUTION + "?action=show":
            return 200, ('<html><form id="stop" method="post">'
                         '<select name="stopmode"><option value="instr"/></select>'
                         '<select name="usetsp"><option value="normal"/></select></form></html>')
        if method == "GET" and path in {TASK, TASK + "?json=1"}:
            return state(execlevel=self.level)
        if method == "GET" and path.startswith(f"{MODULES}/{NORMAL}"):
            return 200, ('<html><li class="rap-module-text">'
                         f'<span class="module-text">{escape(self.normal_source)}</span></li></html>')
        if method == "GET" and path.startswith(TASK + "/program/builderror?"):
            errors = '<li>Syntax error</li>' if self.build_errors else ''
            return 200, f'<html><div class="state"><ul>{errors}</ul></div></html>'
        if method == "GET" and path.startswith(MODULES + "?task="):
            present = ([NORMAL] if self.normal_present else []) + ([V2_ENTRY[0]] if self.module_present else [])
            modules = "".join(f'<li class="rap-module-info-li"><span class="name">{name}</span></li>' for name in present)
            return 200, f'<html><div class="state"><ul>{modules}</ul></div></html>'
        if method == "POST" and path == LOAD:
            remote = body["modulepath"].replace("$HOME/", "/fileservice/$home/")
            if self.uploaded.get(remote) == self.normal_source:
                self.calls.append((method, path, body))
                assert self.mastership and self.execution == "stopped" and body["replace"] == "false"
                self.normal_present = True
                self.normal_order, self.normal_reply = f"[0,{ZERO}]", "[0,0]"
                return 204, ""
        if method == "POST" and path in {STOP, PP, START, BUILD, ABORT}:
            self.calls.append((method, path, body))
            assert self.mastership
            if path == STOP:
                assert self.execution == "running" and body in (STOP_FORM, {"stopmode": "stop"})
                if self.stop_effective:
                    self.execution = "stopped"
                if self.lost_stop:
                    raise TimeoutError
            else:
                assert self.execution == "stopped"
                if path == ABORT:
                    self.level = "normal"
                    self.module, self.routine = "MainModule", "main"
                if path == PP and self.apply_pointer:
                    self.module, self.routine = body["module"], body["routine"]
                if path == START:
                    self.execution = "running"
                    if self.discard_on_start:
                        if (self.module, self.routine) == NORMAL_ENTRY:
                            seq = literal_eval(self.normal_order.replace("9E9", "9e9"))[0]
                            reply, phase = literal_eval(self.normal_reply)
                            if reply != seq or phase in (1, 2):
                                self.normal_reply = f"[{seq},5]"
                        elif (self.module, self.routine) == V2_ENTRY:
                            seq = literal_eval(self.order.replace("9E9", "9e9"))[0]
                            reply, phase = literal_eval(self.reply)
                            if reply != seq or phase in (1, 2):
                                self.reply = f"[{seq},5]"
                    if self.lost_start:
                        raise TimeoutError
            return 204, ""
        return super().request(method, path, body)

    def close(self):
        self.close_count += 1


class RwsTransitionTests(TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[4]
        self.normal_source = Path(environ.get(
            "IMT342_RAPID_SOURCE", root / "rapid/imt342/Imt342JointServer.mod",
        )).read_bytes()
        self.trajectory_source = Path(environ.get(
            "IMT342_RWS_TRAJECTORY_SOURCE",
            root / "experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod",
        )).read_bytes()
        self.http = TransitionHttp(self.trajectory_source.decode("ascii"), self.normal_source.decode("ascii"))

    def make_supervisor(self, selected="rws", source=None):
        values = {
            "rws_url": CONFIG.base_url, "rws_task": CONFIG.task, "rws_module": NORMAL,
            "rws_mechunit": CONFIG.mechunit, "rws_tolerance_rad": CONFIG.tolerance_rad,
        }
        node = SimpleNamespace(declare_parameter=lambda name, default: SimpleNamespace(
            value=values.get(name, default),
        ))
        factories = configured_backends(
            node, trajectory_source=self.trajectory_source if source is None else source,
            normal_source=self.normal_source,
        )
        supervisor = Supervisor(factories, selected)
        mock_credentials = patch.dict(environ, {
            "IMT342_RWS_USERNAME": "synthetic-user", "IMT342_RWS_PASSWORD": "synthetic-password",
        })
        mock_credentials.start()
        self.addCleanup(mock_credentials.stop)
        http_factory = patch("imt342_irb120_control.rws_backend.DigestHttp", return_value=self.http)
        self.http_factory = http_factory.start()
        self.addCleanup(http_factory.stop)
        self.supervisor = supervisor
        self.addCleanup(supervisor.close)
        return supervisor

    def drive(self, condition, batch=None):
        deadline = monotonic() + 4
        while not condition():
            self.supervisor.tick(True)
            if batch is not None:
                batch.tick(self.supervisor, self.supervisor.connected, monotonic())
            if monotonic() >= deadline:
                self.fail(f"Timeout del doble RWS: {self.supervisor.detail}")
            sleep(0.002)

    def prepare(self, selected, recover=False):
        supervisor = self.supervisor
        self.assertTrue(supervisor.set_mode("observe"))
        self.assertTrue(supervisor.select(selected))
        self.drive(lambda: supervisor.connected)
        self.assertTrue(supervisor.request_prepare_rapid(
            self.normal_source if selected == "rws" else self.trajectory_source,
            recover_missing=recover,
        ))
        self.drive(lambda: not supervisor.preparing_rapid)
        return supervisor.rapid_preparation["state"]

    def writes(self):
        return [path for method, path, _ in self.http.calls if method != "GET" and not path.startswith(MASTER)]

    def test_normal_v2_normal_use_one_digest_session_and_select_does_not_write(self):
        supervisor = self.make_supervisor()
        self.assertEqual(self.prepare("rws"), "already_running")
        self.assertEqual(supervisor.clients["rws"].backend.config.tolerance_rad, ARRIVAL_TOLERANCE_RAD)
        self.assertEqual(self.writes(), [])
        self.assertTrue(supervisor.select("rws_trajectory"))
        self.drive(lambda: supervisor.connected)
        self.assertEqual(supervisor.clients["rws_trajectory"].backend.config.tolerance_rad, ARRIVAL_TOLERANCE_RAD)
        self.assertEqual(self.writes(), [])
        self.assertFalse(supervisor.clients["rws_trajectory"].motion_enabled)
        self.http.lost_stop = True  # Reconcile an accepted STOP with a missing HTTP response.
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes(), [STOP, PP, START])
        self.assertEqual((self.http.module, self.http.routine), V2_ENTRY)
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        self.assertEqual(self.prepare("rws"), "started")
        self.assertEqual((self.http.module, self.http.routine), NORMAL_ENTRY)
        self.assertEqual(self.writes(), [STOP, PP, START, STOP, PP, START])
        self.assertEqual(supervisor.clients["rws"].backend._closed_sequence(), (4, 3))
        self.assertEqual(supervisor.clients["rws_trajectory"].backend._closed_sequence(), (0, 0))
        self.http_factory.assert_called_once()
        supervisor.close()
        self.assertEqual(self.http.close_count, 1)

    def test_normal_keeps_stopped_main_path_even_if_v2_source_is_invalid(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "MainModule", "main"
        self.http.build_errors = True   # Only v2 uses this endpoint.
        self.make_supervisor(source=b"invalid module")
        self.assertEqual(set(self.supervisor.factories), {"rws"})
        self.assertEqual(self.prepare("rws"), "started")
        self.assertEqual((self.http.module, self.http.routine), NORMAL_ENTRY)
        self.assertEqual(self.writes(), [PP, START])

    def test_v2_takes_over_running_main_only_on_explicit_prepare(self):
        self.http.module, self.http.routine = "MainModule", "main"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual((self.http.module, self.http.routine), V2_ENTRY)
        self.assertEqual(self.writes(), [STOP, PP, START])

    def test_foreign_stopped_user_level_to_normal_then_v2(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "IMT342_Automatico", "ServidorAutomatico"
        self.http.level = "user"
        self.make_supervisor()
        self.assertEqual(self.prepare("rws"), "started")
        self.assertEqual(self.writes(), [ABORT, PP, START])
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes(), [ABORT, PP, START, STOP, PP, START])

    def test_foreign_running_user_level_to_v2(self):
        self.http.module, self.http.routine = "IMT342_Automatico", "ServidorAutomatico"
        self.http.level = "user"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes(), [STOP, ABORT, PP, START])

    def test_closed_normal_move_pointer_can_switch_to_v2(self):
        self.http.module, self.http.routine = NORMAL, "RunJointOrder"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes(), [STOP, PP, START])

    def test_pending_normal_move_pointer_never_stops_or_switches(self):
        self.http.module, self.http.routine = NORMAL, "RunJointOrder"
        self.http.normal_reply = "[4,2]"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "failed")
        self.assertEqual(self.writes(), [])

    def test_missing_both_modules_automatically_bootstraps_normal_then_v2(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "MainModule", "main"
        self.http.normal_present = self.http.module_present = False
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes().count(LOAD), 2)
        self.assertEqual(self.writes()[-3:], [BUILD, PP, START])
        self.assertEqual((self.http.module, self.http.routine), V2_ENTRY)

    def test_foreign_stop_not_confirmed_never_changes_pp_or_starts(self):
        self.http.module, self.http.routine = "OtherProgram", "main"
        self.http.stop_effective = False
        self.make_supervisor(selected="rws_trajectory")
        from imt342_irb120_control import rws_transition
        from itertools import count
        clock = count(step=5.0)
        with patch.object(rws_transition, "monotonic", side_effect=lambda: next(clock)):
            self.assertEqual(self.prepare("rws_trajectory"), "stop_unconfirmed")
        self.assertEqual(self.writes(), [STOP])
        self.assertEqual(self.http.execution, "running")

    def test_foreign_program_can_switch_with_inactive_abandoned_order(self):
        self.http.module, self.http.routine = "OtherProgram", "main"
        self.http.normal_reply = "[4,2]"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes(), [STOP, PP, START])
        self.assertEqual(self.http.normal_reply, "[4,2]")

    def test_foreign_program_with_multiple_active_tasks_is_not_stopped(self):
        self.http.module, self.http.routine = "OtherProgram", "main"
        self.http.active_tasks.append("OtherTask")
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "failed")
        self.assertEqual(self.writes(), [])

    def test_pending_order_blocks_stop_before_writing(self):
        self.http.normal_reply = "[4,2]"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "failed")
        self.assertIn("sin cerrar", self.supervisor.rapid_preparation["detail"])
        self.assertEqual(self.writes(), [])
        self.assertEqual((self.http.module, self.http.routine), NORMAL_ENTRY)

    def test_reported_stopped_foreign_order7_and_missing_v2(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "IMT342_Automatico", "ServidorAutomatico"
        self.http.level = "user"
        self.http.normal_order, self.http.normal_reply = f"[7,{ZERO}]", "[7,2]"
        self.http.module_present = False
        self.make_supervisor()
        self.assertEqual(self.prepare("rws"), "started")
        self.assertEqual(self.http.normal_reply, "[7,5]")
        self.assertEqual(self.writes(), [ABORT, PP, START])
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.writes().count(LOAD), 1)
        self.assertEqual((self.http.module, self.http.routine), V2_ENTRY)
        self.assertNotIn(NORMAL_ORDER, self.writes())
        self.assertNotIn(ORDER, self.writes())

    def test_select_v2_directly_with_stale_normal_and_missing_v2(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "IMT342_Automatico", "ServidorAutomatico"
        self.http.level = "user"
        self.http.normal_order, self.http.normal_reply = f"[7,{ZERO}]", "[7,2]"
        self.http.module_present = False
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.http.normal_reply, "[7,2]")
        self.assertEqual(self.writes().count(LOAD), 1)
        self.assertEqual(self.prepare("rws"), "started")
        self.assertEqual(self.http.normal_reply, "[7,5]")

    def test_stopped_v2_abandoned_sequence_mismatch(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = V2_ENTRY
        self.http.order, self.http.reply = f"[7,{ZERO},{ZERO},0.5]", "[6,3]"
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertEqual(self.http.reply, "[7,5]")
        self.assertEqual(self.writes(), [PP, START])

    def test_stopped_abandoned_normal_not_ready_without_phase5(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = NORMAL_ENTRY
        self.http.normal_reply = "[4,2]"
        self.http.discard_on_start = False
        self.make_supervisor()
        from imt342_irb120_control import rws_backend
        from itertools import count
        clock = count(step=4.0)
        with patch.object(rws_backend, "monotonic", side_effect=lambda: next(clock)):
            self.assertEqual(self.prepare("rws"), "failed")
        self.assertEqual(self.writes(), [PP, START])
        self.assertEqual(self.http.normal_reply, "[4,2]")

    def test_missing_v2_loads_additively_on_prepare(self):
        self.http.module_present = False
        self.make_supervisor(selected="rws_trajectory")
        self.http.version = "7.6"
        self.assertEqual(self.prepare("rws_trajectory"), "robotware_incompatible")
        self.assertEqual(self.writes(), [])
        self.http.version = "6.12.1014"
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        stop, upload, *rest = self.writes()
        self.assertEqual(stop, STOP)
        self.assertTrue(upload.startswith("/fileservice/$home/imt342-"))
        self.assertEqual(rest, [LOAD, BUILD, PP, START])
        self.assertEqual((self.http.module, self.http.routine), V2_ENTRY)

    def test_v2_will_not_start_after_unconfirmed_pp_or_repeat_lost_start(self):
        self.http.execution = "stopped"
        self.http.apply_pointer = False
        self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "failed")
        self.assertEqual(self.writes(), [PP])
        self.http.apply_pointer = True
        self.http.lost_start = True
        self.assertEqual(self.prepare("rws_trajectory"), "start_unconfirmed")
        self.assertFalse(self.supervisor.clients["rws_trajectory"].motion_enabled)
        self.assertEqual(self.writes(), [PP, PP, START])
        self.assertEqual(self.prepare("rws_trajectory"), "already_running")
        self.assertEqual(self.writes(), [PP, PP, START])

    def test_v2_multileg_waits_for_measured_arrival_and_correlated_reply(self):
        supervisor = self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        goals = ((0.01, *ZERO_AXES[1:]), ZERO_AXES, (0.02, *ZERO_AXES[1:]))
        plan = plan_sequence(ZERO_AXES, goals)
        batch = BatchRun(plan, monotonic(), 4.0, backend="rws_trajectory")
        for sequence, (goal, segment) in enumerate(zip(goals, plan.segments), 1):
            self.drive(lambda: self.writes().count(ORDER) == sequence, batch)
            self.assertEqual(batch.index, sequence - 1)
            payload = literal_eval(self.http.order)
            self.assertEqual(payload[0], sequence)
            self.assertAlmostEqual(payload[-1], segment.duration)
            self.http.q_deg = tuple(map(degrees, goal))
            self.http.reply = f"[{sequence},3]"
        self.drive(lambda: not batch.active, batch)
        self.assertEqual(batch.state, "verified")
        self.assertEqual([item["motion"]["reply"] for item in batch.steps], [(1, 3), (2, 3), (3, 3)])
        self.assertEqual([item["motion"]["requested_duration_s"] for item in batch.steps],
                         [segment.duration for segment in plan.segments])

    def test_v2_uncertain_order_does_not_run_following_destination(self):
        supervisor = self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        goal = 0.01, *ZERO_AXES[1:]
        plan = plan_sequence(ZERO_AXES, (goal, ZERO_AXES))
        batch = BatchRun(plan, monotonic(), 4.0, backend="rws_trajectory")
        self.http.lose_after_set = True
        self.drive(lambda: not batch.active, batch)
        self.assertEqual(batch.state, "unknown")
        self.assertEqual(supervisor.owner, "rws_trajectory")
        self.assertFalse(supervisor.select("rws"))
        self.http.q_deg = tuple(map(degrees, goal))
        self.http.reply = "[1,3]"
        self.drive(lambda: supervisor.owner is None)
        self.assertEqual(self.writes().count(ORDER), 1)
        self.assertFalse(supervisor.clients["rws_trajectory"].motion_enabled)
        self.assertEqual(supervisor.motion_observation[0], Result.VERIFIED)


    def test_v1_normal_still_sends_legacy_record(self):
        supervisor = self.make_supervisor()
        self.assertEqual(self.prepare("rws"), "already_running")
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        self.assertEqual(type(supervisor.clients["rws"].backend).__name__, "RwsRapidBackend")
        goal = (0.03, *ZERO_AXES[1:])
        plan = plan_sequence(ZERO_AXES, (goal,))
        self.assertTrue(supervisor.arm())
        self.assertTrue(supervisor.execute(goal, plan))
        self.drive(lambda: NORMAL_ORDER in self.writes())
        sequence, target = literal_eval(self.http.normal_order)
        self.assertEqual(sequence, 5)
        self.assertAlmostEqual(target[0][0], degrees(goal[0]))
        self.http.q_deg = tuple(map(degrees, goal))
        self.http.normal_reply = "[5,3]"
        self.drive(lambda: supervisor.owner is None)
        self.assertNotIn("requested_duration_s", supervisor.motion_observation[1])
        self.assertIsNone(supervisor.motion_observation[1]["controller_timing_s"])

    def test_retired_protocol_three_is_not_accepted_as_normal(self):
        self.http.normal_protocol = 3
        self.make_supervisor()
        state = self.prepare("rws")
        self.assertNotIn(state, {"started", "already_running"})
        self.assertEqual(self.writes(), [])

    def test_origin_band_rejects_changed_start_under_mastership(self):
        supervisor = self.make_supervisor()
        self.assertEqual(self.prepare("rws"), "already_running")
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        goal = (0.03, *ZERO_AXES[1:])
        planned_from_zero = plan_sequence(ZERO_AXES, (goal,))
        self.assertTrue(supervisor.arm())
        self.http.q_deg = (.09, *ZERO_AXES[1:])  # Within arrival band but outside plan-origin band.
        self.assertTrue(supervisor.execute(goal, planned_from_zero))
        self.drive(lambda: supervisor.owner is None)
        self.assertIs(supervisor.motion_observation[0], Result.FAILED)
        self.assertIn("cambió desde la planificación", supervisor.detail)
        self.assertEqual(self.writes(), [])

    def test_batch_already_satisfied_does_not_create_a_sequence(self):
        supervisor = self.make_supervisor(selected="rws_trajectory")
        self.assertEqual(self.prepare("rws_trajectory"), "started")
        self.assertTrue(supervisor.set_mode("control"))
        self.drive(lambda: supervisor.can_arm)
        first = (radians(.15), *ZERO_AXES[1:])  # Intentional short move.
        second = (radians(1.5), *ZERO_AXES[1:])
        plan = plan_sequence(ZERO_AXES, (ZERO_AXES, first, first, second))
        batch = BatchRun(plan, monotonic(), 4.0, backend="rws_trajectory")
        for sequence, goal in ((1, first), (2, second)):
            self.drive(lambda: self.writes().count(ORDER) == sequence, batch)
            self.assertEqual(literal_eval(self.http.order)[0], sequence)
            self.http.q_deg = tuple(map(degrees, goal))
            self.http.reply = f"[{sequence},3]"
        self.drive(lambda: not batch.active, batch)
        self.assertEqual(batch.state, "verified")
        self.assertEqual([step["result"] for step in batch.steps],
                         ["already_satisfied", "verified", "already_satisfied", "verified"])
        self.assertIs(batch.steps[2]["motion"]["motion_sent"], False)
        self.assertNotIn("sequence", batch.steps[2]["motion"])
