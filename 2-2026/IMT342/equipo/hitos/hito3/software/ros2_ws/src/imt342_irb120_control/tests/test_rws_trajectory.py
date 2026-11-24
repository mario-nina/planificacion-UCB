"""Offline HTTP contract tests for the experimental RAPID endpoint service."""

from dataclasses import replace
from ast import literal_eval
from json import dumps
from math import degrees
from os import environ
from pathlib import Path
from unittest import TestCase
from unittest.mock import patch
from urllib.parse import urlsplit
from xml.sax.saxutils import escape

from imt342_irb120_control.adapters import ARRIVAL_TOLERANCE_RAD, RwsTrajectoryAdapter
from imt342_irb120_control.rws_backend import EXECUTION, MASTER, MODE, MODULES, MOTOR, TASKS, RwsConfig, RwsError
from imt342_irb120_control.rws_trajectory import MODULE_NAME, MOVE_ROUTINE, ROUTINE, RwsTrajectoryBackend
from imt342_irb120_control.state import Result

CONFIG = RwsConfig("http://127.0.0.1", "", "", "T_ROB1", MODULE_NAME, "ROB_1", 0.005)
START = (0.0,) * 6
GOAL = (0.03, -0.02, 0.0, 0.0, 0.0, 0.0)
ZERO = "[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]"
ORDER = f"/rw/rapid/symbol/data/RAPID/{CONFIG.task}/{MODULE_NAME}/trjOrder?action=set"
STOP = f"{EXECUTION}?action=stop"


def state(**fields):
    return 200, dumps({"_embedded": {"_state": [fields]}})


class TrajectoryHttp:
    def __init__(self, source):
        self.source = source
        self.source_as_file = False
        self.version = "6.12.1040"
        self.protocol = "2"
        self.module = MODULE_NAME
        self.module_present = True
        self.routine = ROUTINE
        self.mode = "AUTO"
        self.motors = "motoron"
        self.execution = "running"
        self.active_tasks = [CONFIG.task]
        self.q_deg = START
        self.order = f"[0,{ZERO},{ZERO},0]"
        self.reply = "[0,0]"
        self.mastership = False
        self.on_mastership = None
        self.lose_after_set = False
        self.calls = []

    def request(self, method, url, body):
        self.calls.append((method, url, body))
        if method == "POST":
            if url == f"{MASTER}?action=request":
                self.mastership = True
                if self.on_mastership is not None:
                    self.on_mastership()
                return 204, ""
            if url == f"{MASTER}?action=release":
                self.mastership = False
                return 204, ""
            if self.mastership and url == ORDER:
                self.order = body["value"]
                if self.lose_after_set:
                    raise TimeoutError
                return 204, ""
            if self.mastership and url == STOP:
                return 204, ""
            return 403, ""

        if method != "GET":
            return 405, ""
        path = urlsplit(url).path
        if path == MODE:
            return state(opmode=self.mode)
        if path == MOTOR:
            return state(ctrlstate=self.motors)
        if path == EXECUTION:
            return state(ctrlexecstate=self.execution)
        if path == f"{TASKS}/{CONFIG.task}/pcp":
            return 200, (f'<html><li title="progpointer"><span class="modulemame">{self.module}</span>'
                         f'<span class="routinename">{self.routine}</span></li></html>')
        if path == TASKS:
            tasks = "".join(f'<li class="rap-task-li"><span class="name">{name}</span>'
                            '<span class="active">On</span></li>' for name in self.active_tasks)
            return 200, f"<html>{tasks}</html>"
        if path == MODULES:
            module = f'<li class="rap-module-info-li"><span class="name">{MODULE_NAME}</span></li>'
            return 200, f'<html><div class="state"><ul>{module if self.module_present else ""}</ul></div></html>'
        if path == f"{MODULES}/{MODULE_NAME}":
            if self.source_as_file:
                return 200, ('<html><li class="rap-module-text">'
                             '<span class="file-path">"/hd0a/TEMPpusres.123"</span></li></html>')
            return 200, ('<html><li class="rap-module-text">'
                         f'<span class="module-text">{escape(self.source)}</span></li></html>')
        if path == "/fileservice/hd0a/TEMPpusres.123" and self.source_as_file:
            return 200, self.source
        if path == "/rw/system":
            return 200, f'<html><span class="rwversion">{self.version}</span></html>'
        if path.endswith("/trjVersion"):
            return state(value=self.protocol)
        if path.endswith("/trjOrder"):
            return state(value=self.order)
        if path.endswith("/trjReply"):
            return state(value=self.reply)
        if path.endswith("/jointtarget"):
            return state(**{f"rax_{axis}": value for axis, value in enumerate(self.q_deg, 1)})
        return 404, ""


class RwsTrajectoryTests(TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[4]
        local = root / "experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod"
        self.source = Path(environ.get("IMT342_RWS_TRAJECTORY_SOURCE", local)).read_bytes()
        self.http = TrajectoryHttp(self.source.decode("ascii"))
        self.now = 1.0
        self.backend = RwsTrajectoryBackend(CONFIG, self.source, self.http, lambda: self.now)

    def execute(self, *, authorized=lambda: True, duration=2.0, start=START,
                expected_prior_sequence=None):
        return self.backend.execute(GOAL, duration=duration, authorized=authorized, expected_start=start,
                                    expected_prior_sequence=expected_prior_sequence)

    def posts(self):
        return [(url, body) for method, url, body in self.http.calls if method == "POST"]

    def test_source_pin_task_and_mechanical_unit(self):
        for config in (replace(CONFIG, task="OTHER"), replace(CONFIG, mechunit="OTHER"),
                       replace(CONFIG, module="Imt342JointServer")):
            with self.assertRaises(ValueError):
                RwsTrajectoryBackend(config, self.source, self.http)
        with self.assertRaises(ValueError):
            RwsTrajectoryBackend(CONFIG, self.source + b"\n", self.http)
        self.assertEqual(self.http.calls, [])

    def test_prepare_is_read_only_and_accepts_live_persistent_values(self):
        old = self.http.source
        start = old.index(" := ", old.index("PERS ImtTrjOrder")) + len(" := ")
        end = old.index(";", start)
        self.http.order = f"[15,{ZERO},{ZERO},2.5]"
        self.http.reply = "[15,3]"
        self.http.source = (old[:start] + self.http.order + old[end:]).replace("[0,0];", "[15,3];")
        self.assertEqual(self.backend.prepare(), 15)
        self.assertEqual(self.backend.start_server(), "already_running")
        self.assertEqual(self.backend.read_state().source, "rws_trajectory")
        self.assertEqual(self.backend.diagnostic()["protocol_version"], 2)
        self.assertEqual(self.posts(), [])
        with self.assertRaises(RwsError):
            self.backend.start_server(recover_missing=True)
        self.assertEqual(self.posts(), [])

    def test_blocks_bad_state_before_requesting_mastership(self):
        failures = (
            ("version", "6.15.01", "6.12"),
            ("module_present", False, "Falta Imt342TrajectoryRapid"),
            ("source", self.http.source.replace("WaitTime 0.05;", "WaitTime 0.02;"), "fuente"),
            ("module", "Imt342JointServer", "ServeTrajectoryOrders"),
            ("active_tasks", [CONFIG.task, "Other"], "tarea"),
            ("protocol", "1", "v2"),
            ("motors", "motoroff", "motores"),
            ("execution", "stopped", "RAPID en marcha"),
            ("reply", "[0,2]", "sin cerrar"),
        )
        for field, value, message in failures:
            with self.subTest(field=field):
                self.setUp()
                setattr(self.http, field, value)
                with self.assertRaisesRegex(RwsError, message):
                    self.execute()
                self.assertEqual(self.posts(), [])

    def test_prepare_verifies_temporary_module_export(self):
        self.http.source_as_file = True
        self.assertEqual(self.backend.prepare(), 0)
        self.assertIn(("GET", "/fileservice/hd0a/TEMPpusres.123", None), self.http.calls)
        self.assertEqual(self.posts(), [])

    def test_checks_fresh_start_and_source_under_mastership(self):
        self.http.on_mastership = lambda: setattr(self.http, "q_deg", (1.0,) + START[1:])
        with self.assertRaisesRegex(RwsError, "postura"):
            self.execute()
        self.assertEqual(self.posts(), [(f"{MASTER}?action=request", None), (f"{MASTER}?action=release", None)])
        self.http.q_deg = START
        self.http.on_mastership = lambda: setattr(self.http, "source", self.http.source.replace(
            "WaitTime 0.05;", "WaitTime 0.02;"
        ))
        with self.assertRaisesRegex(RwsError, "fuente"):
            self.execute()
        self.assertFalse(self.http.mastership)
        self.assertNotIn(ORDER, [url for url, _ in self.posts()])

    def test_writes_one_sequence_and_checks_measured_arrival(self):
        self.assertEqual(self.execute(duration=3.25), Result.TRANSMITTED)
        commands = [body["value"] for url, body in self.posts() if url == ORDER]
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("[1,[[0.00000000,0.00000000,"))
        self.assertIn(f"[[{degrees(GOAL[0]):.8f},{degrees(GOAL[1]):.8f},", commands[0])
        self.assertTrue(commands[0].endswith(",3.25000000]"))
        self.http.reply = "[1,2]"
        self.assertEqual(self.backend.result(), Result.EXECUTING)
        self.http.reply = "[1,3]"
        self.http.q_deg = tuple(map(degrees, GOAL))
        self.assertEqual(self.backend.result(), Result.VERIFIED)
        evidence = self.backend.motion_evidence
        self.assertEqual(evidence["requested_duration_s"], 3.25)
        self.assertEqual(evidence["reply"], (1, 3))
        self.assertIsNone(evidence["controller_timing_s"])
        self.assertNotIn("commanded_speeddata", evidence)

    def test_duration_no_movement_and_authorization_are_checked_without_posts(self):
        for duration in (True, 0.09, 30.01, float("nan")):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.execute(duration=duration)
        with self.assertRaises(ValueError):
            self.execute(start=GOAL)
        with self.assertRaises(RwsError):
            self.execute(authorized=lambda: False)
        self.assertEqual(self.posts(), [])

    def test_expected_lot_sequence_checked_before_and_under_mastership(self):
        with self.assertRaisesRegex(RwsError, "entre tramos"):
            self.execute(expected_prior_sequence=1)
        self.assertEqual(self.posts(), [])

        def advance():
            self.http.order = f"[1,{ZERO},{ZERO},2.0]"
            self.http.reply = "[1,3]"

        self.http.on_mastership = advance
        with self.assertRaisesRegex(RwsError, "secuencia RAPID"):
            self.execute(expected_prior_sequence=0)
        self.assertFalse(self.http.mastership)
        self.assertNotIn(ORDER, [url for url, _ in self.posts()])

    def test_uncertain_submit_never_retries_and_cannot_stop_foreign_pp(self):
        self.http.lose_after_set = True
        self.assertEqual(self.execute(), Result.UNKNOWN)
        self.http.reply = "[1,2]"
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        self.http.module = "Imt342JointServer"
        self.assertEqual(self.backend.cancel(), Result.UNKNOWN)
        self.assertNotIn(STOP, [url for url, _ in self.posts()])
        with self.assertRaisesRegex(RwsError, "sin resolver"):
            self.execute()
        self.assertEqual([url for url, _ in self.posts()].count(ORDER), 1)
        self.http.module = MODULE_NAME
        self.http.routine = MOVE_ROUTINE
        self.assertEqual(self.backend.cancel(), Result.TRANSMITTED)
        self.backend.cancel()
        self.assertEqual([url for url, _ in self.posts()].count(STOP), 1)
        self.http.execution = "stopped"
        self.assertEqual(self.backend.result(), Result.CANCELLED)

    def test_final_deadline_and_unverified_posture(self):
        self.execute(duration=2.0)
        self.http.reply = "[1,2]"
        self.now += 8
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        self.http.reply = "[1,3]"
        self.assertEqual(self.backend.result(), Result.FAILED)

    def test_transient_idle_read_backoff_then_automatic_ready(self):
        adapter = RwsTrajectoryAdapter(self.backend, motion_enabled=True)
        adapter.poll(check_ready=True)
        self.assertTrue(adapter.ready)
        with patch.object(self.backend, "read_state", side_effect=RwsError("RWS GET /jointtarget: HTTP 500", 500)) as read:
            with patch("imt342_irb120_control.adapters.monotonic", return_value=100.0):
                self.assertIsNone(adapter.poll(check_ready=True)[0])
                self.assertFalse(adapter.ready)
                self.assertTrue(adapter.motion_enabled)
                self.assertIsNone(adapter.poll(check_ready=True)[0])
                self.assertEqual(read.call_count, 1)
        with patch("imt342_irb120_control.adapters.monotonic", return_value=101.0):
            feedback, _, detail = adapter.poll(check_ready=True)
        self.assertIsNotNone(feedback)
        self.assertTrue(adapter.ready)
        self.assertTrue(adapter.motion_enabled)
        self.assertIn("HTTP 500", literal_eval(detail)["first_transport_error"])
        self.assertEqual(adapter.read_failures, 0)
        self.assertEqual(self.posts(), [])

    def test_transient_preflight_retries_without_revoking_v2(self):
        adapter = RwsTrajectoryAdapter(self.backend, motion_enabled=True)
        adapter.next_diagnostic = 1000.0
        with patch.object(self.backend, "prepare", side_effect=RwsError("HTTP 503", 503)):
            with patch("imt342_irb120_control.adapters.monotonic", return_value=100.0):
                adapter.poll(check_ready=True)
            with patch("imt342_irb120_control.adapters.monotonic", return_value=100.2):
                adapter.poll(check_ready=True)
            self.assertTrue(adapter.motion_enabled)
            self.assertFalse(adapter.ready)
        with patch("imt342_irb120_control.adapters.monotonic", return_value=102.0):
            adapter.poll(check_ready=True)
        self.assertTrue(adapter.ready)
        self.assertEqual(self.posts(), [])

    def test_nontransient_wrong_program_still_revokes_v2(self):
        adapter = RwsTrajectoryAdapter(self.backend, motion_enabled=True)
        adapter.poll(check_ready=True)
        self.http.module = "OtherProgram"
        adapter.next_preflight = 0.0
        adapter.poll(check_ready=True)
        self.assertFalse(adapter.motion_enabled)
        self.assertFalse(adapter.ready)
        self.assertEqual(self.posts(), [])

    def test_terminal_reply_inside_move_routine_waits_for_idle_without_revoking(self):
        config = replace(CONFIG, tolerance_rad=ARRIVAL_TOLERANCE_RAD)
        self.backend = RwsTrajectoryBackend(config, self.source, self.http, lambda: self.now)
        adapter = RwsTrajectoryAdapter(self.backend, motion_enabled=True)
        self.execute()
        self.http.routine = MOVE_ROUTINE
        self.http.q_deg = tuple(map(degrees, GOAL))
        self.http.q_deg = (self.http.q_deg[0] + .20, *self.http.q_deg[1:])
        self.http.reply = "[1,2]"
        self.assertEqual(adapter.poll(check_ready=True)[1], Result.EXECUTING)
        self.assertTrue(adapter.motion_evidence["within_arrival_band"])
        self.assertFalse(adapter.ready)
        self.http.reply = "[1,3]"
        _, result, text = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.VERIFIED)
        self.assertTrue(adapter.motion_enabled)
        self.assertFalse(adapter.ready)
        self.assertIn("esperando retorno", text)
        self.http.routine = ROUTINE
        adapter.next_diagnostic = 0.0
        _, result, _ = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.VERIFIED)
        self.assertTrue(adapter.ready)
        self.assertTrue(adapter.motion_enabled)
        self.assertEqual(len([url for url, _ in self.posts() if url == ORDER]), 1)

    def test_adapter_reports_pending_motion_and_refreshes_the_verified_reply(self):
        adapter = RwsTrajectoryAdapter(self.backend, motion_enabled=True)
        adapter.poll(check_ready=True)
        self.assertTrue(adapter.ready)
        self.execute()
        self.http.reply = "[1,2]"
        _, result, text = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.EXECUTING)
        self.assertIsNone(literal_eval(text)["error"])
        self.assertFalse(adapter.ready)

        self.http.reply = "[1,3]"
        self.http.q_deg = tuple(map(degrees, GOAL))
        _, result, text = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.VERIFIED)
        diagnostic = literal_eval(text)
        self.assertEqual(diagnostic["reply"], (1, 3))
        self.assertEqual(diagnostic["order_sequence"], 1)
        self.assertIsNone(diagnostic["error"])
        self.assertTrue(adapter.ready)
