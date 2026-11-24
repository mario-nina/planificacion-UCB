"""Offline RWS protocol tests using a stateful HTTP double."""
from json import dumps
from dataclasses import replace
import math
from os import environ
from pathlib import Path
import unittest
from time import monotonic, sleep
from types import SimpleNamespace
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit
from xml.sax.saxutils import escape

from imt342_irb120_control.rws_backend import (
    DigestHttp, EXECUTION, MASTER, MODE, MODULES, MODULE_NAME, MOTOR, TASKS,
    RwsConfig, RwsError, RwsRapidBackend,
)
from imt342_irb120_control.adapters import ARRIVAL_TOLERANCE_RAD, RwsAdapter
from imt342_irb120_control.state import Result
from imt342_irb120_control.supervisor import Supervisor

CFG = RwsConfig("http://127.0.0.1", "", "", "TEST_TASK", "TestModule", "TEST_ARM", 0.001)
GOAL = (0.0, 0.01, -0.02, 0.0, 0.0, 0.0)
MODULE_EXPORT = "/hd0a/TEMPpusres.123"
MODULE_EXPORT_GET = "/fileservice" + MODULE_EXPORT


def authorized():
    return True


class RapidHttp:
    def __init__(self):
        self.requests = []
        self.mode = "AUTO"
        self.execution = "running"
        self.module = CFG.module
        self.routine = "ServeJointOrders"
        self.level = "Normal"
        self.other_task = False
        self.verify_pointer = True
        self.verify_start = True
        self.prepare_failure = {}
        self.motors = "motoron"
        self.version = "1"
        self.master = False
        self.order = "[0,[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]]"
        self.reply = "[0,0]"
        self.axes = (0.0,) * 6
        self.loss_after_command = False
        self.loss_before_command = False
        self.fail_commit = None
        self.fail_mastership = False
        self.fail_stop = False
        self.fail_feedback = False
        self.bad_reply = False

    @staticmethod
    def json_state(**fields):
        return 200, dumps({"_embedded": {"_state": [fields]}})

    def request(self, method, url, body=None):
        self.requests.append((method, url, body))
        path = urlsplit(url).path
        action = parse_qs(urlsplit(url).query).get("action", [""])[0]
        if method == "POST":
            if action in self.prepare_failure:
                return self.prepare_failure[action], ""
            if path == MASTER:
                if self.fail_mastership:
                    return 403, ""
                self.master = action == "request"
                return 204, ""
            if not self.master:
                return 403, ""
            if path == f"{TASKS}/{CFG.task}" and action == "abortexeclevel":
                self.level = "Normal"
                return 204, ""
            if path == f"{TASKS}/{CFG.task}/pcp" and action == "set-pp-routine":
                if self.verify_pointer:
                    self.module, self.routine = body["module"], body["routine"]
                return 204, ""
            if path == EXECUTION and action == "start":
                if self.verify_start:
                    self.execution = "running"
                return 204, ""
            if path.endswith("/pcOrder") and action == "set":
                if self.fail_commit:
                    return self.fail_commit, ""
                if self.loss_before_command:
                    raise TimeoutError
                self.order = body["value"]
                if self.loss_after_command:
                    raise TimeoutError
                return 204, ""
            if path == EXECUTION and action == "stop":
                if self.fail_stop:
                    return 403, ""
                return 204, ""
        if method == "GET":
            if path == TASKS:
                names = [CFG.task, "OtherTask"] if self.other_task else [CFG.task]
                return 200, "<html>" + "".join(
                    f'<li class="rap-task-li"><span class="name">{name}</span>'
                    '<span class="active">On</span></li>' for name in names
                ) + "</html>"
            if path == f"{TASKS}/{CFG.task}":
                return self.json_state(execlevel=self.level)
            if path == MODE:
                return self.json_state(opmode=self.mode)
            if path == MOTOR:
                return self.json_state(ctrlstate=self.motors)
            if path == EXECUTION:
                return self.json_state(ctrlexecstate=self.execution)
            if path.endswith("/pcp"):
                return 200, (
                    '<html xmlns="http://www.w3.org/1999/xhtml"><li title="progpointer">'
                    f'<span class="modulemame">{self.module}</span>'
                    f'<span class="routinename">{self.routine}</span></li></html>'
                )
            if path.endswith("/rbVersion"):
                return self.json_state(value=self.version)
            if path.endswith("/pcOrder"):
                return self.json_state(value=self.order)
            if path.endswith("/rbReply"):
                return self.json_state(value="malformed" if self.bad_reply else self.reply)
            if path.endswith("/jointtarget"):
                if self.fail_feedback:
                    return 503, ""
                return self.json_state(**{f"rax_{i}": axis for i, axis in enumerate(self.axes, 1)})
        return 404, ""


class RapidRecoveryHttp(RapidHttp):
    def __init__(self, source):
        super().__init__()
        self.execution = "stopped"
        self.module, self.routine = "MainModule", "main"
        self.loaded = False
        self.staged = None
        self.source = source
        self.source_response = None
        self.modules_response = None
        self.export_path = None
        self.export_status = 200
        self.range_supported = False
        self.range_changed = False
        self.extension_reads = 0
        self.escape_self_link = True
        self.corrupt_staging = False
        self.corrupt_program = False
        self.race_at_upload = False
        self.robotware = "6.12.1014"

    def request(self, method, url, body=None):
        path = urlsplit(url).path
        params = parse_qs(urlsplit(url).query)
        action = params.get("action", [""])[0]
        if method == "GET" and path == "/rw/system":
            self.requests.append((method, url, body))
            return 200, f'<html><span class="rwversion">{self.robotware}</span></html>'
        if method == "GET" and path == "/ctrl/identity":
            self.requests.append((method, url, body))
            return 200, '<html><span class="ctrl-name">IRC5-Test</span></html>'
        if path == MODULES and method == "GET":
            assert params == {"task": [CFG.task]}
            self.requests.append((method, url, body))
            if self.modules_response is not None:
                return self.modules_response
            names = ["MainModule", "LabolimRws"]
            if self.loaded:
                names.append(MODULE_NAME)
            rows = "".join(
                f'<li class="rap-module-info-li" title="{CFG.task}/{name}">'
                f'<span class="name">{name}</span><span class="type">ProgMod</span></li>'
                for name in names
            )
            return 200, (
                '<html xmlns="http://www.w3.org/1999/xhtml">'
                f'<div class="state"><ul>{rows}</ul></div></html>'
            )
        if method == "GET" and path == f"{MODULES}/{MODULE_NAME}":
            self.requests.append((method, url, body))
            assert params["task"] == [CFG.task]
            resource = params.get("resource")
            if resource != ["module-text"]:
                if not self.range_supported:
                    return 404, ""
                lines = self.source.splitlines()
                if resource == ["module-extension"]:
                    self.extension_reads += 1
                    count = 217221 if self.range_changed and self.extension_reads > 1 else 217220
                    return 200, (
                        '<html xmlns="http://www.w3.org/1999/xhtml"><div class="state">'
                        '<ul><li class="rap-module-extension" title="extension">'
                        f'<span class="num-of-lines">{len(lines)}</span>'
                        f'<span class="count">{count}</span>'
                        '<span class="max-num-of-col">118</span></li></ul></div></html>'
                    )
                first = int(params["startrow"][0])
                last = int(params["endrow"][0])
                assert params["startcol"] == ["1"] and params["endcol"] == ["-1"]
                # The physical IRC5's text-range response can leave RAPID operators unescaped.
                fragment = "\n".join(lines[first - 1:last])
                return 200, (
                    '<html xmlns="http://www.w3.org/1999/xhtml"><div class="state">'
                    '<ul><li class="rap-mod-text" title="pgm">'
                    f'<span class="text">{fragment}</span></li></ul></div></html>'
                )
            if self.source_response is not None:
                return self.source_response
            if not self.loaded:
                return 404, ""
            source = self.source.replace("v10", "v100") if self.corrupt_program else self.source
            value = (
                f'<span class="file-path">"{escape(self.export_path)}"</span>'
                if self.export_path is not None else f'<span class="module-text">{escape(source)}</span>'
            )
            separator = "&amp;" if self.escape_self_link else "&"
            return 200, (
                '<html xmlns="http://www.w3.org/1999/xhtml"><body><div class="state">'
                f'<a href="modules/{MODULE_NAME}?resource=module-text{separator}task={CFG.task}" rel="self"/>'
                '<ul><li class="rap-module-text" title="moduletext">'
                f'<span class="change-count">217220</span>{value}'
                '<span class="module-length">1368</span></li></ul></div></body></html>'
            )
        if path == MODULE_EXPORT_GET:
            self.requests.append((method, url, body))
            assert method == "GET" and not params
            return self.export_status, self.source if self.export_status == 200 else ""
        if path.startswith("/fileservice/"):
            self.requests.append((method, url, body))
            if method == "GET":
                if self.staged is None:
                    return 404, ""
                return 200, "corrupto" if self.corrupt_staging else self.staged.decode("ascii")
            assert method == "PUT" and self.master
            self.staged = body
            if self.race_at_upload:
                self.loaded = True
            return 201, ""
        if method == "POST" and action == "loadmod":
            self.requests.append((method, url, body))
            assert self.master and not self.loaded
            assert body["replace"] == "false" and body["modulepath"].startswith("$HOME/imt342-")
            if action in self.prepare_failure:
                return self.prepare_failure[action], ""
            self.loaded = True
            return 204, ""
        return super().request(method, url, body)


class RwsRecoveryTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[4]
        source = Path(environ.get("IMT342_RAPID_SOURCE", root / "rapid/imt342/Imt342JointServer.mod"))
        self.source = source.read_bytes()
        self.http = RapidRecoveryHttp(self.source.decode("ascii"))
        self.backend = RwsRapidBackend(replace(CFG, module=MODULE_NAME), self.http)

    def prepare(self, *, recover_missing=False, permit=authorized):
        return self.backend.start_server(authorized=permit, module_source=self.source,
                                         recover_missing=recover_missing)

    def test_missing_module_requires_separate_authorization_and_loads_without_replace(self):
        with self.assertRaises(RwsError) as missing:
            self.prepare()
        self.assertEqual(missing.exception.state, "module_missing")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))
        self.assertEqual(self.prepare(recover_missing=True), "started")
        verbs = [(method, parse_qs(urlsplit(route).query).get("action", [""])[0])
                 for method, route, _ in self.http.requests if method != "GET"]
        self.assertEqual(verbs, [("POST", "request"), ("PUT", ""), ("POST", "loadmod"),
                                 ("POST", "set-pp-routine"), ("POST", "start"), ("POST", "release")])
        self.assertEqual(self.http.staged, self.source)
        self.assertEqual((self.http.module, self.http.routine), (MODULE_NAME, "ServeJointOrders"))
        self.assertEqual(self.http.reply, "[0,0]")
        self.assertEqual(self.backend.controller_metadata,
                         {"controller_name": "IRC5-Test", "robotware_version": "6.12.1014",
                          "active_tasks": [CFG.task]})

    def test_missing_module_without_confirmed_robotware6_is_left_to_operator(self):
        self.http.robotware = "7.1"
        with self.assertRaisesRegex(RwsError, "RobotWare 6"):
            self.prepare(recover_missing=True)
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_owned_running_checks_selected_tasks_without_writes(self):
        self.http.loaded = True
        self.http.execution = "running"
        self.http.module, self.http.routine = MODULE_NAME, "ServeJointOrders"
        self.http.other_task = True
        with self.assertRaisesRegex(RwsError, "solo la tarea configurada"):
            self.prepare()
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_stopped_server_with_another_active_task_blocks_before_mastership(self):
        self.http.loaded = True
        self.http.other_task = True
        with self.assertRaisesRegex(RwsError, "solo la tarea configurada"):
            self.prepare()
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_stopped_foreign_user_level_is_preserved(self):
        self.http.loaded = True
        self.http.level = "User"
        self.http.module, self.http.routine = "LabolimRws", "ServidorPC"
        with self.assertRaises(RwsError) as blocked:
            self.prepare()
        self.assertEqual(blocked.exception.state, "foreign_program")
        self.assertIn("LabolimRws/ServidorPC", str(blocked.exception))
        self.assertEqual(self.http.level, "User")
        self.assertEqual([route for method, route, _ in self.http.requests if method != "GET"],
                         [f"{MASTER}?action=request", f"{MASTER}?action=release"])

    def test_modified_and_foreign_running_programs_are_not_written(self):
        self.http.loaded = True
        self.http.module, self.http.routine = "LabolimRws", "ServidorPC"
        self.http.corrupt_program = True
        with self.assertRaises(RwsError) as changed:
            self.prepare()
        self.assertEqual(changed.exception.state, "module_modified")
        self.http.execution = "running"
        with self.assertRaises(RwsError) as foreign:
            self.prepare()
        self.assertEqual(foreign.exception.state, "foreign_program")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_compatible_source_does_not_override_live_protocol_or_pending_order(self):
        for version, reply, detail in (("2", "[0,0]", "incompatible"), ("1", "[0,2]", "sin cerrar")):
            with self.subTest(version=version, reply=reply):
                self.setUp()
                self.http.loaded = True
                self.http.module, self.http.routine = "LabolimRws", "ServidorPC"
                self.http.version, self.http.reply = version, reply
                with self.assertRaisesRegex(RwsError, detail):
                    self.prepare()
                self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_owned_running_ignores_serialized_live_registers_only(self):
        self.http.loaded = True
        self.http.execution = "running"
        self.http.module, self.http.routine = MODULE_NAME, "ServeJointOrders"
        self.http.escape_self_link = False
        self.http.source = self.http.source.replace("[0,0];", "[40,5];")
        self.assertEqual(self.prepare(), "already_running")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_stopped_foreign_pp_with_compatible_export_starts_owned_server_once(self):
        self.http.loaded = True
        self.http.module, self.http.routine = "LabolimRws", "ServidorPC"
        self.http.export_path = MODULE_EXPORT
        source = self.http.source.replace("[0,0];", "[15,3];")
        self.http.order = self.http.order.replace("[0,", "[15,", 1)
        self.http.reply = "[15,3]"
        source = source.replace("pcOrder := [0,", "pcOrder := [15,", 1)
        header = "\ufeff%%%\nVERSION:1\nLANGUAGE:ENGLISH\n%%%\n"
        self.http.source = (header + source).replace("\n", "\r\n")

        self.assertEqual(self.prepare(), "started")
        self.assertIn(("GET", MODULE_EXPORT_GET, None), self.http.requests)
        writes = [(route, body) for method, route, body in self.http.requests if method != "GET"]
        self.assertEqual([route for route, _ in writes], [
            f"{MASTER}?action=request",
            f"{TASKS}/{CFG.task}/pcp?action=set-pp-routine",
            f"{EXECUTION}?action=start",
            f"{MASTER}?action=release",
        ])
        self.assertEqual(writes[1][1]["module"], MODULE_NAME)
        self.assertEqual(writes[1][1]["routine"], "ServeJointOrders")
        self.assertEqual(self.http.reply, "[15,3]")
        self.http.requests.clear()

        self.assertEqual(self.prepare(), "already_running")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_source_http_errors_are_distinguished_before_mastership(self):
        cases = ((401, "module_source_denied"), (403, "module_source_denied"),
                 (404, "module_source_unavailable"), (503, "module_source_rws_error"))
        for status, state in cases:
            with self.subTest(status=status):
                self.setUp()
                self.http.loaded = True
                self.http.source_response = (status, "")
                with self.assertRaises(RwsError) as error:
                    self.prepare()
                self.assertEqual((error.exception.status, error.exception.state), (status, state))
                self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_rejected_or_unavailable_export_keeps_module_intact(self):
        cases = (
            (MODULE_EXPORT, 403, "module_source_denied", True),
            (MODULE_EXPORT, 404, "module_source_unavailable", True),
            ("/hd0a/OtherProgram.mod", 200, "module_source_unavailable", False),
            (MODULE_EXPORT + "?extra=1", 200, "module_source_unavailable", False),
        )
        for path, status, state, read_export in cases:
            with self.subTest(path=path, status=status):
                self.setUp()
                self.http.loaded = True
                self.http.export_path = path
                self.http.export_status = status
                with self.assertRaises(RwsError) as error:
                    self.prepare()
                self.assertEqual(error.exception.state, state)
                self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))
                self.assertEqual(any(route.startswith("/fileservice/") for _, route, _ in self.http.requests),
                                 read_export)

    def test_module_text_ranges_reassemble_rws_export_when_file_is_missing(self):
        self.http.loaded = True
        self.http.export_path = MODULE_EXPORT
        self.http.export_status = 404
        self.http.range_supported = True
        self.http.execution = "running"
        self.http.module, self.http.routine = MODULE_NAME, "ServeJointOrders"

        self.assertEqual(self.prepare(), "already_running")
        self.assertEqual(self.http.extension_reads, 2)
        ranges = []
        for method, route, _ in self.http.requests:
            self.assertEqual(method, "GET")
            params = parse_qs(urlsplit(route).query)
            if "startrow" in params:
                ranges.append((int(params["startrow"][0]), int(params["endrow"][0])))
        self.assertEqual(ranges, [(1, 20), (21, 40), (41, 43)])

    def test_module_text_ranges_refuse_changes_during_read(self):
        self.http.loaded = True
        self.http.export_path = MODULE_EXPORT
        self.http.export_status = 404
        self.http.range_supported = True
        self.http.range_changed = True

        with self.assertRaises(RwsError) as error:
            self.prepare()
        self.assertEqual(error.exception.state, "module_source_invalid")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_malformed_response_and_truncated_source_are_not_reported_as_modified(self):
        cases = (
            "<html><li unclosed>",
            "<html><div class=\"state\"/></html>",
            ('<html><li class="rap-module-text"><span class="module-text">'
             'MODULE Imt342JointServer CONST num rbVersion := 1;</span></li></html>'),
        )
        for response in cases:
            with self.subTest(response=response):
                self.setUp()
                self.http.loaded = True
                self.http.source_response = (200, response)
                with self.assertRaises(RwsError) as error:
                    self.prepare()
                self.assertEqual(error.exception.state, "module_source_invalid")
                self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_bad_module_inventory_cannot_trigger_recovery(self):
        self.http.modules_response = (200, '<html><div class="state"><ul><li>other</li></ul></div></html>')
        with self.assertRaises(RwsError) as error:
            self.prepare(recover_missing=True)
        self.assertNotEqual(error.exception.state, "module_missing")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_transfer_mismatch_concurrent_load_and_unsupported_load_refuse_start(self):
        for fault in ("corrupt_staging", "race_at_upload", "unsupported"):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == "unsupported":
                    self.http.prepare_failure["loadmod"] = 400
                else:
                    setattr(self.http, fault, True)
                with self.assertRaises(RwsError):
                    self.prepare(recover_missing=True)
                self.assertFalse(any("action=start" in route for _, route, _ in self.http.requests))
                self.assertFalse(self.http.master)

    def test_recovery_requires_canonical_bytes_and_no_other_selected_task(self):
        with self.assertRaisesRegex(RwsError, "huella"):
            self.backend.start_server(authorized=authorized, module_source=self.source + b" ",
                                      recover_missing=True)
        self.assertFalse(self.http.requests)
        self.http.other_task = True
        with self.assertRaises(RwsError):
            self.prepare(recover_missing=True)
        self.assertFalse(any(method == "PUT" for method, _, _ in self.http.requests))
        self.assertFalse(self.http.master)


class RwsBackendTest(unittest.TestCase):
    def setUp(self):
        self.http = RapidHttp()
        self.now = 5.0
        self.backend = RwsRapidBackend(CFG, self.http, clock=lambda: self.now)

    def test_connect_and_observe_when_rapid_is_absent(self):
        self.http.execution = "stopped"
        self.http.axes = (90.0, -90.0, 0, 0, 0, 0)
        self.assertEqual(self.backend.connect().execution, "stopped")
        feedback = self.backend.read_state()
        self.assertEqual(feedback.positions[:2], (math.pi / 2, -math.pi / 2))
        self.assertEqual(feedback.received_at, 5)
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_preparation_of_running_server_is_read_only(self):
        self.assertEqual(self.backend.start_server(authorized=authorized), "already_running")
        self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_preparation_requires_explicit_authorization(self):
        with self.assertRaisesRegex(RwsError, "autorización"):
            self.backend.start_server()
        self.assertFalse(self.http.requests)

    def test_prepare_stopped_server_relocates_and_verifies_without_order(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "MainModule", "main"
        self.http.level = "User"
        self.assertEqual(self.backend.start_server(authorized=authorized), "started")
        writes = [(route, body) for method, route, body in self.http.requests if method == "POST"]
        self.assertEqual(writes, [
            (f"{MASTER}?action=request", None),
            (f"{TASKS}/{CFG.task}?action=abortexeclevel", None),
            (f"{TASKS}/{CFG.task}/pcp?action=set-pp-routine",
             {"module": CFG.module, "routine": "ServeJointOrders", "userlevel": "false"}),
            (f"{EXECUTION}?action=start", {
                "regain": "clear", "execmode": "continue", "cycle": "forever",
                "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false",
            }),
            (f"{MASTER}?action=release", None),
        ])
        self.assertEqual(self.http.order.split(",")[0], "[0")
        self.assertEqual(self.http.reply, "[0,0]")
        self.assertFalse(self.http.master)

    def test_prepare_stopped_server_relocates_even_when_pointer_already_matches(self):
        self.http.execution = "stopped"
        self.assertEqual(self.backend.start_server(authorized=authorized), "started")
        self.assertIn(("POST", f"{TASKS}/{CFG.task}/pcp?action=set-pp-routine",
                       {"module": CFG.module, "routine": "ServeJointOrders", "userlevel": "false"}),
                      self.http.requests)

    def test_preparation_fails_closed_on_pending_order_incompatible_version_or_wrong_running_pp(self):
        cases = (
            ("stopped", CFG.module, "1", "[0,2]", "sin cerrar"),
            ("stopped", CFG.module, "2", "[0,0]", "incompatible"),
            ("running", "MainModule", "1", "[0,0]", "PP fuera"),
        )
        for execution, module, version, reply, error in cases:
            with self.subTest(error=error):
                self.http.execution, self.http.module = execution, module
                self.http.version, self.http.reply = version, reply
                self.http.requests.clear()
                with self.assertRaisesRegex(RwsError, error):
                    self.backend.start_server(authorized=authorized)
                self.assertTrue(all(method == "GET" for method, _, _ in self.http.requests))

    def test_preparation_refuses_other_active_task_and_never_repositions(self):
        self.http.execution = "stopped"
        self.http.other_task = True
        with self.assertRaisesRegex(RwsError, "solo la tarea configurada"):
            self.backend.start_server(authorized=authorized)
        writes = [route for method, route, _ in self.http.requests if method == "POST"]
        self.assertEqual(writes, [f"{MASTER}?action=request", f"{MASTER}?action=release"])

    def test_preparation_checks_pointer_and_running_after_204(self):
        self.http.execution = "stopped"
        self.http.module, self.http.routine = "MainModule", "main"
        self.http.verify_pointer = False
        with self.assertRaisesRegex(RwsError, "No se confirmó el puntero"):
            self.backend.start_server(authorized=authorized)
        self.assertFalse(any("action=start" in route for _, route, _ in self.http.requests))
        self.http.verify_pointer = True
        self.http.verify_start = False
        with self.assertRaisesRegex(RwsError, "Inicio RAPID sin confirmar"):
            self.backend.start_server(authorized=authorized)
        self.assertFalse(self.http.master)

    def test_prepare_preserves_http_error_and_releases_mastership(self):
        self.http.execution = "stopped"
        for status in (401, 403, 415):
            with self.subTest(status=status):
                self.http.prepare_failure["set-pp-routine"] = status
                with self.assertRaisesRegex(RwsError, f"HTTP {status}") as caught:
                    self.backend.start_server(authorized=authorized)
                self.assertEqual(caught.exception.status, status)
                self.assertFalse(self.http.master)
        self.assertFalse(any("action=start" in route for _, route, _ in self.http.requests))

    def test_release_refusal_after_start_is_reported_and_never_restarts(self):
        self.http.execution = "stopped"
        self.http.prepare_failure["release"] = 403
        with self.assertRaisesRegex(RwsError, "liberación de mastership incierta"):
            self.backend.start_server(authorized=authorized)
        self.assertEqual(self.backend.start_server(authorized=authorized), "already_running")
        self.assertEqual(sum("action=start" in route for _, route, _ in self.http.requests), 1)

    def test_measured_order_timing_is_from_rws_observations(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.now += 0.5
        self.http.reply = "[1,2]"
        self.assertEqual(self.backend.result(), Result.EXECUTING)
        self.now += 1.5
        self.http.reply = "[1,3]"
        self.http.axes = tuple(map(math.degrees, GOAL))
        self.assertEqual(self.backend.result(), Result.VERIFIED)
        evidence = self.backend.motion_evidence
        self.assertEqual(evidence["commanded_speeddata"], "v10")
        self.assertIsNone(evidence["controller_timing_s"])
        self.assertEqual(evidence["submit_to_first_reply_s"], 0.5)
        self.assertEqual(evidence["observed_phase_2_to_terminal_s"], 1.5)
        self.assertEqual(evidence["submit_to_resolution_s"], 2.0)

    def test_absent_permission_keeps_transport_read_only(self):
        with self.assertRaisesRegex(RwsError, "Habilitación"):
            self.backend.execute(GOAL)
        self.assertEqual(self.http.requests, [])

    def test_revocation_during_preflight_releases_mastership_without_order(self):
        decisions = iter((True, False))
        with self.assertRaisesRegex(RwsError, "revocada antes del envío"):
            self.backend.execute(GOAL, authorized=lambda: next(decisions))
        self.assertFalse(self.http.master)
        self.assertFalse(any("action=set" in route for _, route, _ in self.http.requests))

    def test_failed_joint_read_blocks_write(self):
        self.http.fail_feedback = True
        with self.assertRaisesRegex(RwsError, "HTTP 503"):
            self.backend.execute(GOAL, authorized=authorized)
        self.assertFalse(any("action=set" in route for _, route, _ in self.http.requests))

    def test_changed_start_pose_blocks_command(self):
        self.http.axes = (0, 2, 0, 0, 0, 0)
        with self.assertRaisesRegex(RwsError, "cambió desde la planificación"):
            self.backend.execute(GOAL, authorized=authorized, expected_start=(0.0,) * 6)
        self.assertFalse(any("action=set" in route for _, route, _ in self.http.requests))

    def test_non_text_controller_status_is_rejected(self):
        self.http.mode = None
        with self.assertRaisesRegex(RwsError, "valor de estado inválido"):
            self.backend.connect()

    def test_partial_and_invalid_joint_feedback(self):
        for axes in ((0,) * 5, (True,) * 6, ("NaN",) * 6, (None,) * 6):
            with self.subTest(axes=axes):
                self.http.axes = axes
                with self.assertRaises(RwsError):
                    self.backend.read_state()

    def test_authentication_failure_preserves_http_code(self):
        self.http.request = lambda *args: (401, "")
        with self.assertRaisesRegex(RwsError, "HTTP 401") as caught:
            self.backend.connect()
        self.assertEqual(caught.exception.status, 401)

    def test_each_order_uses_matching_ack_and_measured_feedback(self):
        for number in range(1, 4):
            self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.TRANSMITTED)
            self.assertFalse(self.http.master)
            self.assertTrue(self.http.order.startswith(f"[{number},[[0.00000000,0.57295780,-1.14591559,"))
            self.assertEqual(self.backend.result(), Result.TRANSMITTED)
            self.http.reply = f"[{number},1]"
            self.assertEqual(self.backend.result(), Result.ACCEPTED)
            self.http.reply = f"[{number},2]"
            self.assertEqual(self.backend.result(), Result.EXECUTING)
            self.http.reply = f"[{number},3]"
            self.http.axes = tuple(map(math.degrees, GOAL))
            self.assertEqual(self.backend.result(), Result.VERIFIED)
            self.assertLess(self.backend.final_error_rad, 1e-12)

    def test_arrival_band_waits_for_matching_terminal_reply(self):
        config = replace(CFG, tolerance_rad=ARRIVAL_TOLERANCE_RAD)
        self.backend = RwsRapidBackend(config, self.http, clock=lambda: self.now)
        adapter = RwsAdapter(self.backend)
        self.backend.execute(GOAL, authorized=authorized)
        self.http.axes = tuple(map(math.degrees, GOAL))
        self.http.axes = (self.http.axes[0] + 0.20, *self.http.axes[1:])
        self.http.reply = "[1,2]"
        _, result, _ = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.EXECUTING)
        self.assertTrue(adapter.motion_evidence["within_arrival_band"])
        self.assertTrue(adapter.motion_evidence["awaiting_terminal_reply"])
        self.assertFalse(adapter.ready)
        self.now += 2.0
        self.http.reply = "[1,3]"
        _, result, _ = adapter.poll(check_ready=True)
        self.assertEqual(result, Result.VERIFIED)
        self.assertTrue(adapter.ready)
        self.assertFalse(adapter.motion_evidence["awaiting_terminal_reply"])
        self.assertAlmostEqual(adapter.motion_evidence["final_error_rad"], math.radians(0.20))
        self.assertAlmostEqual(adapter.motion_evidence["observed_band_to_terminal_s"], 2.0)
        self.http.axes = (30.0,) * 6  # Later unrelated motion cannot rewrite the old arrival.
        adapter.poll(check_ready=True)
        self.assertTrue(adapter.motion_evidence["within_arrival_band"])
        self.assertAlmostEqual(adapter.motion_evidence["latest_goal_error_rad"], math.radians(0.20))
        self.backend.execute(GOAL, authorized=authorized)
        self.http.axes = (0.35, *self.http.axes[1:])
        self.http.reply = "[2,3]"
        self.assertEqual(adapter.poll(check_ready=True)[1], Result.FAILED)
        self.assertFalse(adapter.motion_evidence["within_arrival_band"])

    def test_rapid_completion_without_matching_measured_pose_fails(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,3]"
        self.assertEqual(self.backend.result(), Result.FAILED)
        self.assertAlmostEqual(self.backend.final_error_rad, 0.02)

    def test_final_feedback_outage_recovers_without_second_order(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,3]"
        self.http.fail_feedback = True
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        self.http.fail_feedback = False
        self.http.axes = tuple(map(math.degrees, GOAL))
        self.assertEqual(self.backend.result(), Result.VERIFIED)

    def test_interlocks_and_program_pointer_reject_before_commit(self):
        for field, value in (("module", "Other"), ("motors", "guardstop"), ("mode", "MANR")):
            with self.subTest(field=field):
                original = getattr(self.http, field)
                setattr(self.http, field, value)
                with self.assertRaises(RwsError):
                    self.backend.execute(GOAL, authorized=authorized)
                self.assertEqual(self.backend.result(), Result.FAILED)
                self.assertFalse(self.http.master)
                setattr(self.http, field, original)
        self.assertFalse(any("pcOrder?action=set" in route for _, route, _ in self.http.requests))

    def test_corrupt_reply_and_mastership_refusal_fail_before_commit(self):
        self.http.fail_mastership = True
        with self.assertRaisesRegex(RwsError, "HTTP 403"):
            self.backend.execute(GOAL, authorized=authorized)
        self.assertEqual(self.backend.result(), Result.FAILED)
        self.http.fail_mastership = False
        self.http.bad_reply = True
        with self.assertRaisesRegex(RwsError, "formato incompatible"):
            self.backend.execute(GOAL, authorized=authorized)
        self.assertEqual(self.backend.result(), Result.FAILED)

    def test_write_rejection_releases_mastership(self):
        self.http.fail_commit = 403
        with self.assertRaisesRegex(RwsError, "HTTP 403"):
            self.backend.execute(GOAL, authorized=authorized)
        self.assertFalse(self.http.master)
        self.assertEqual(self.backend.result(), Result.FAILED)
        self.http.fail_commit = None
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.TRANSMITTED)

    def test_ambiguous_server_error_does_not_release_order(self):
        self.http.fail_commit = 503
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.UNKNOWN)
        with self.assertRaisesRegex(RwsError, "sin resolver"):
            self.backend.execute(GOAL, authorized=authorized)
        writes = [route for _, route, _ in self.http.requests if "pcOrder?action=set" in route]
        self.assertEqual(len(writes), 1)

    def test_lost_reply_never_reissues_order_and_reconciles(self):
        self.http.loss_after_command = True
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.UNKNOWN)
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        with self.assertRaisesRegex(RwsError, "sin resolver"):
            self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,3]"
        self.http.axes = tuple(map(math.degrees, GOAL))
        self.assertEqual(self.backend.result(), Result.VERIFIED)
        writes = [route for _, route, _ in self.http.requests if "pcOrder?action=set" in route]
        self.assertEqual(len(writes), 1)

    def test_lost_request_before_commit_is_unknown_until_stop_reconciles(self):
        self.http.loss_before_command = True
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.UNKNOWN)
        self.assertTrue(self.http.order.startswith("[0,"))
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        self.assertEqual(self.backend.cancel(), Result.TRANSMITTED)
        self.http.execution = "stopped"
        self.assertEqual(self.backend.result(), Result.CANCELLED)
        self.http.execution = "running"
        self.http.loss_before_command = False
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.TRANSMITTED)
        self.assertTrue(self.http.order.startswith("[1,"))

    def test_transmission_without_rapid_ack_times_out_but_keeps_ownership(self):
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.TRANSMITTED)
        self.now += CFG.ack_timeout
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        with self.assertRaisesRegex(RwsError, "sin resolver"):
            self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,1]"
        self.assertEqual(self.backend.result(), Result.ACCEPTED)

    def test_stop_waits_for_stopped_execution_and_measured_feedback(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,2]"
        self.assertEqual(self.backend.cancel(), Result.TRANSMITTED)
        self.assertEqual(self.backend.result(), Result.EXECUTING)
        self.http.execution = "stopped"
        self.http.fail_feedback = True
        self.assertEqual(self.backend.result(), Result.UNKNOWN)
        self.http.fail_feedback = False
        self.assertEqual(self.backend.result(), Result.CANCELLED)
        self.assertIn("reubicar PP", self.backend.last_error)
        self.http.execution = "running"
        with self.assertRaisesRegex(RwsError, "orden anterior"):
            self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,5]"  # Emulates PP restart and discard by the server.
        self.assertEqual(self.backend.execute(GOAL, authorized=authorized), Result.TRANSMITTED)
        self.assertTrue(self.http.order.startswith("[2,"))

    def test_rejected_stop_keeps_ownership_until_verified(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,2]"
        self.http.fail_stop = True
        self.assertEqual(self.backend.cancel(), Result.UNKNOWN)
        with self.assertRaises(RwsError):
            self.backend.execute(GOAL, authorized=authorized)
        self.http.execution = "stopped"
        self.assertEqual(self.backend.result(), Result.CANCELLED)

    def test_occupied_controller_blocks_other_client(self):
        self.backend.execute(GOAL, authorized=authorized)
        other = RwsRapidBackend(CFG, self.http)
        with self.assertRaisesRegex(RwsError, "orden anterior"):
            other.execute(GOAL, authorized=authorized)
        self.http.reply = "[1,2]"
        with self.assertRaisesRegex(RwsError, "orden anterior"):
            other.execute(GOAL, authorized=authorized)

    def test_cancel_refuses_to_stop_a_different_program(self):
        self.backend.execute(GOAL, authorized=authorized)
        self.http.module = "OtherModule"
        self.assertEqual(self.backend.cancel(), Result.UNKNOWN)
        self.assertFalse(any("action=stop" in route for _, route, _ in self.http.requests))


class RwsSupervisorTest(unittest.TestCase):
    def test_explicit_preparation_runs_in_observe_worker_and_never_arms_motion(self):
        http = RapidHttp()
        http.execution = "stopped"
        backend = RwsRapidBackend(CFG, http)
        supervisor = Supervisor({"rws": lambda: RwsAdapter(backend)}, "rws")
        self.addCleanup(supervisor.close)

        def drive(condition):
            deadline = monotonic() + 2
            while not condition() and monotonic() < deadline:
                supervisor.tick(True)
                sleep(0.001)
            self.assertTrue(condition())

        self.assertFalse(supervisor.request_prepare_rapid())
        self.assertTrue(supervisor.set_mode("observe"))
        drive(lambda: supervisor.connected)
        self.assertTrue(all(method == "GET" for method, _, _ in http.requests))
        self.assertTrue(supervisor.request_prepare_rapid())
        self.assertFalse(supervisor.request_prepare_rapid())
        self.assertFalse(supervisor.set_mode("control"))
        drive(lambda: not supervisor.preparing_rapid)
        self.assertEqual(supervisor.rapid_preparation["state"], "started")
        self.assertFalse(supervisor.armed)
        self.assertIsNone(supervisor.owner)
        self.assertEqual(http.reply, "[0,0]")

    def test_mock_read_only_and_uncertain_commit_without_retransmission(self):
        http = RapidHttp()
        backend = RwsRapidBackend(CFG, http)
        supervisor = Supervisor({"rws": lambda: RwsAdapter(backend)}, "rws")
        self.addCleanup(supervisor.close)
        trajectory = SimpleNamespace(sample=lambda _: SimpleNamespace(positions=(0.0,) * 6))

        def drive(condition):
            deadline = monotonic() + 2
            while not condition() and monotonic() < deadline:
                supervisor.tick(True)
                sleep(0.001)
            self.assertTrue(condition())

        supervisor.tick(True)
        self.assertEqual(http.requests, [])
        self.assertTrue(supervisor.set_mode("observe"))
        drive(lambda: supervisor.connected)
        self.assertFalse(supervisor.arm())
        self.assertFalse(supervisor.execute(GOAL, trajectory))
        self.assertTrue(all(method == "GET" for method, _, _ in http.requests))

        self.assertTrue(supervisor.set_mode("control"))
        drive(lambda: supervisor.can_arm)
        self.assertFalse(supervisor.execute(GOAL, trajectory))
        self.assertTrue(supervisor.arm())
        http.loss_after_command = True
        self.assertTrue(supervisor.execute(GOAL, trajectory))
        drive(lambda: "'result': 'unknown'" in supervisor.detail)
        self.assertFalse(supervisor.arm())
        self.assertFalse(supervisor.set_mode("observe"))
        self.assertFalse(supervisor.execute(GOAL, trajectory))

        http.loss_after_command = False
        http.reply = "[1,3]"
        http.axes = tuple(map(math.degrees, GOAL))
        drive(lambda: supervisor.owner is None)
        self.assertFalse(supervisor.armed)
        self.assertFalse(supervisor.execute(GOAL, trajectory))
        writes = [route for _, route, _ in http.requests if "pcOrder?action=set" in route]
        self.assertEqual(len(writes), 1)


class RwsBatchTest(unittest.TestCase):
    def setUp(self):
        from imt342_irb120_control.batch import BatchRun
        from imt342_irb120_planning import plan_sequence

        self.http = RapidHttp()
        self.backend = RwsRapidBackend(CFG, self.http)
        self.supervisor = Supervisor({"rws": lambda: RwsAdapter(self.backend)}, "rws")
        self.addCleanup(self.supervisor.close)
        self.supervisor.set_mode("control")
        self.goals = tuple((0.01 * i, *GOAL[1:]) for i in range(1, 5)) * 2
        self.plan = plan_sequence((0.0,) * 6, self.goals)
        self.batch = BatchRun(self.plan, monotonic(), 10.0)
        self.drive(lambda: self.supervisor.can_arm, run=False)

    def drive(self, condition, *, run=True):
        deadline = monotonic() + 3.0
        while not condition():
            self.supervisor.tick(True)
            if run:
                self.batch.tick(self.supervisor, self.supervisor.connected, monotonic())
            if monotonic() >= deadline:
                self.fail(f"Timeout del mock: {self.batch.state} {self.supervisor.detail}")
            sleep(0.002)

    def commands(self):
        return [body["value"] for _, url, body in self.http.requests if "pcOrder?action=set" in url]

    def submitted(self, sequence):
        return len(self.commands()) == sequence and bool(
            self.supervisor.motion_observation and self.supervisor.motion_observation[1].get("submit_confirmed")
        )

    def complete(self, sequence, goal):
        self.http.axes = tuple(map(math.degrees, goal))
        self.http.reply = f"[{sequence},3]"

    def test_two_cycles_only_advance_after_matching_terminal_reply_and_pose(self):
        self.assertTrue(self.supervisor.arm())  # Existing single-order authorization is consumed.
        for sequence, goal in enumerate(self.goals, start=1):
            self.drive(lambda: self.submitted(sequence))
            self.assertFalse(self.supervisor.armed)
            self.assertEqual(self.batch.index, sequence - 1)
            self.assertEqual(len(self.commands()), sequence)
            self.complete(sequence, goal)
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "verified")
        self.assertEqual(len(self.batch.steps), 8)
        self.assertEqual([row["motion"]["reply"] for row in self.batch.steps],
                         [(i, 3) for i in range(1, 9)])
        self.assertTrue(all(row["motion"]["commanded_speeddata"] == "v10" for row in self.batch.steps))
        self.assertTrue(all(row["motion"]["controller_timing_s"] is None for row in self.batch.steps))

    def test_ambiguous_post_followed_by_late_success_never_continues(self):
        self.http.loss_after_command = True
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "unknown")
        self.http.loss_after_command = False
        self.complete(1, self.goals[0])
        self.drive(lambda: self.supervisor.owner is None)
        self.assertEqual(len(self.commands()), 1)

    def test_matching_reply_with_wrong_position_stops_batch(self):
        self.drive(lambda: self.submitted(1))
        self.http.reply = "[1,3]"
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "failed")
        self.assertEqual(len(self.commands()), 1)

    def test_changed_initial_pose_rejects_before_write(self):
        self.http.axes = (20.0,) * 6
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "failed")
        self.assertFalse(self.commands())

    def test_rapid_stop_and_stale_feedback_block_successors(self):
        self.drive(lambda: self.submitted(1))
        self.http.execution = "stopped"
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "unknown")
        self.assertEqual(len(self.commands()), 1)

    def test_lost_feedback_blocks_successors(self):
        self.drive(lambda: self.submitted(1))
        self.http.fail_feedback = True
        self.drive(lambda: not self.batch.active)
        self.assertEqual(self.batch.state, "unknown")
        self.assertEqual(len(self.commands()), 1)

    def test_deadline_never_replays_unresolved_order(self):
        self.drive(lambda: self.submitted(1))
        self.batch.tick(self.supervisor, True, self.batch.deadline)
        self.complete(1, self.goals[0])
        self.drive(lambda: self.supervisor.owner is None)
        self.assertEqual(self.batch.state, "unknown")
        self.assertEqual(len(self.commands()), 1)

    def test_cancel_revokes_batch_even_when_inflight_target_finishes(self):
        self.drive(lambda: self.submitted(1))
        self.batch.stop("cancel_requested", "mock de parada")
        self.supervisor.cancel()
        self.complete(1, self.goals[0])
        self.drive(lambda: self.supervisor.owner is None)
        self.assertEqual(len(self.commands()), 1)

    def test_lspb_scale_changes_duration_with_identical_v1_wire_payload(self):
        from imt342_irb120_planning import plan_sequence

        captured = []
        durations = []
        for speed in (1.0, 0.01):
            http = RapidHttp()
            adapter = RwsAdapter(RwsRapidBackend(CFG, http))
            plan = plan_sequence((0.0,) * 6, [self.goals[0]], max_velocity=(speed,) * 6)
            durations.append(plan.duration)
            adapter.execute(self.goals[0], plan, authorized)
            captured.append([body for _, url, body in http.requests if "pcOrder?action=set" in url])
            self.assertEqual(adapter.motion_evidence["commanded_speeddata"], "v10")
            self.assertEqual(len(captured[-1]), 1)
        self.assertGreater(durations[1], durations[0])
        self.assertEqual(captured[0], captured[1])

    def test_old_unknown_batch_does_not_own_a_new_individual_order(self):
        self.http.loss_after_command = True
        self.drive(lambda: not self.batch.active)
        self.assertTrue(self.batch.owns_latest_motion(self.supervisor))
        self.http.loss_after_command = False
        self.complete(1, self.goals[0])
        self.drive(lambda: self.supervisor.owner is None and self.supervisor.can_arm)
        from imt342_irb120_planning import plan_sequence

        self.assertTrue(self.supervisor.arm())
        plan = plan_sequence(self.goals[0], [self.goals[1]])
        self.assertTrue(self.supervisor.execute(self.goals[1], plan))
        self.assertFalse(self.batch.owns_latest_motion(self.supervisor))

    def test_backend_rejects_intervening_sequence_before_post(self):
        with self.assertRaisesRegex(RwsError, "Cambió la secuencia"):
            self.backend.execute(GOAL, authorized=authorized, expected_prior_sequence=1)
        self.assertFalse(self.commands())


class DigestHttpTest(unittest.TestCase):
    @patch("requests.Session")
    def test_reuses_digest_session_with_bounded_requests_and_no_redirect(self, session_factory):
        from requests.auth import HTTPDigestAuth

        config = replace(CFG, username="mock-user", password="mock-pass")
        http = DigestHttp(config)
        session = session_factory.return_value
        self.assertIsInstance(session.auth, HTTPDigestAuth)
        self.assertFalse(session.trust_env)
        session.request.return_value.status_code = 200
        session.request.return_value.text = "body"
        self.assertEqual(http.request("GET", "/rw/resource"), (200, "body"))
        http.request("POST", "/rw/resource", {"value": "x"})
        self.assertEqual(session_factory.call_count, 1)
        session.request.assert_called_with(
            "POST", "http://127.0.0.1/rw/resource", data={"value": "x"},
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timeout=CFG.timeout, allow_redirects=False,
        )
        http.close()
        session.request.assert_called_with("GET", "http://127.0.0.1/logout",
                                           timeout=CFG.timeout, allow_redirects=False)
        http.close()
        self.assertEqual(session.request.call_count, 3)
        session.close.assert_called_once()

    @patch("requests.Session")
    def test_logout_failure_still_closes_local_session(self, session_factory):
        import requests
        http = DigestHttp(replace(CFG, username="mock-user", password="mock-pass"))
        session = session_factory.return_value
        session.request.return_value.status_code = 200
        http.request("GET", "/rw/resource")
        session.request.side_effect = requests.Timeout()
        http.close()
        http.close()
        session.close.assert_called_once()
        self.assertEqual(session.request.call_count, 2)

    @patch("requests.Session")
    def test_unused_session_does_not_log_in_for_logout(self, session_factory):
        http = DigestHttp(replace(CFG, username="mock-user", password="mock-pass"))
        http.close()
        session_factory.return_value.request.assert_not_called()
        session_factory.return_value.close.assert_called_once()

    @patch("requests.Session")
    def test_module_export_route_is_read_only_and_controller_local(self, session_factory):
        config = replace(CFG, username="mock-user", password="mock-pass")
        http = DigestHttp(config)
        request = session_factory.return_value.request
        request.return_value.status_code = 200
        request.return_value.text = "export"
        self.assertEqual(http.request("GET", MODULE_EXPORT_GET), (200, "export"))
        self.assertEqual(request.call_args.args, ("GET", "http://127.0.0.1" + MODULE_EXPORT_GET))
        self.assertFalse(request.call_args.kwargs["allow_redirects"])
        for method, path in (
            ("PUT", MODULE_EXPORT_GET), ("DELETE", MODULE_EXPORT_GET),
            ("GET", MODULE_EXPORT_GET + "?extra=1"), ("GET", "/fileservice/hd0a/OtherProgram.mod"),
            ("GET", "http://example.invalid" + MODULE_EXPORT_GET),
        ):
            with self.subTest(method=method, path=path), self.assertRaises(ValueError):
                http.request(method, path)
        self.assertEqual(request.call_count, 1)


if __name__ == "__main__":
    unittest.main()
