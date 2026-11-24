"""Offline tests for the explicitly authorized, add-only RAPID installation."""

from os import environ
from pathlib import Path
from unittest import TestCase
from urllib.parse import urlsplit

from imt342_irb120_control.rws_backend import MASTER, MODULES, MODULE_NAME as NORMAL_MODULE, TASKS, RwsError
from imt342_irb120_control.rws_trajectory import MODULE_NAME, MODULE_SHA256
from imt342_irb120_control.rws_trajectory_install import RwsTrajectoryInstaller
from test_rws_trajectory import CONFIG, ZERO, TrajectoryHttp, state


LOAD = f"{TASKS}/T_ROB1?action=loadmod"


class InstallHttp(TrajectoryHttp):
    def __init__(self, source):
        super().__init__(source)
        self.execution = "stopped"
        self.module = NORMAL_MODULE
        self.routine = "ServeJointOrders"
        self.module_present = False
        self.uploaded = {}
        self.normal_reply = "[4,3]"
        self.replace_available = True
        self.corrupt_upload = False
        self.lose_load_response = False
        self.on_upload = lambda: None
        self.on_load = lambda: None
        self.on_release = lambda: None

    def request(self, method, url, body):
        if url == LOAD and method == "POST":
            self.calls.append((method, url, body))
            assert self.mastership and body["replace"] == "false"
            assert body["modulepath"].startswith("$HOME/imt342-")
            remote = body["modulepath"].replace("$HOME/", "/fileservice/$home/")
            assert remote in self.uploaded
            self.module_present = True
            self.on_load()
            if self.lose_load_response:
                raise TimeoutError
            return 204, ""
        if url == f"{MASTER}?action=release" and method == "POST":
            self.on_release()
        if url.startswith("/fileservice/$home/"):
            self.calls.append((method, url, body))
            if method == "PUT":
                assert self.mastership and url not in self.uploaded
                self.uploaded[url] = body.decode("ascii")
                self.on_upload()
                return 201, ""
            if method == "GET" and url in self.uploaded:
                return 200, self.uploaded[url] + ("\n" if self.corrupt_upload else "")
            return 404, ""
        if url == f"{TASKS}/T_ROB1?action=show":
            yes = '<option value="false">false</option>' if self.replace_available else ''
            return 200, (
                '<html><form id="loadmod" method="post"><select name="replace">'
                f'<option value="true">true</option>{yes}</select></form>'
                '<form id="build" method="post"/></html>'
            )
        if method == "GET" and urlsplit(url).path == MODULES:
            present = (NORMAL_MODULE, MODULE_NAME) if self.module_present else (NORMAL_MODULE,)
            modules = "".join(f'<li class="rap-module-info-li"><span class="name">{name}</span></li>'
                              for name in present)
            return 200, f'<html><div class="state"><ul>{modules}</ul></div></html>'
        if f"/{NORMAL_MODULE}/" in url:
            if "/rbVersion" in url:
                return state(value="1")
            if "/pcOrder" in url:
                return state(value=f"[4,{ZERO}]")
            if "/rbReply" in url:
                return state(value=self.normal_reply)
        return super().request(method, url, body)


class RwsTrajectoryInstallTests(TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[4]
        local = root / "experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod"
        self.source = Path(environ.get("IMT342_RWS_TRAJECTORY_SOURCE", local)).read_bytes()
        self.http = InstallHttp(self.source.decode("ascii"))
        self.installer = RwsTrajectoryInstaller(CONFIG, self.source, self.http)

    def writes(self):
        return [(method, url, body) for method, url, body in self.http.calls if method != "GET"]

    def test_preflight_and_explicit_authorization(self):
        plan = self.installer.preflight()
        self.assertEqual(plan["pointer"], (NORMAL_MODULE, "ServeJointOrders"))
        self.assertEqual((plan["normal_sequence"], plan["sha256"]), (4, MODULE_SHA256))
        self.assertTrue(plan["build_advertised"])
        with self.assertRaisesRegex(RwsError, "autorización explícita"):
            self.installer.install()
        self.assertEqual(self.writes(), [])

    def test_blocks_unrelated_or_incompatible_state_before_writing(self):
        cases = (
            ("module_present", True, "ya existe"),
            ("execution", "running", "Detén RAPID"),
            ("version", "7.6", "RobotWare 6.12"),
            ("active_tasks", [CONFIG.task, "Other"], "tarea"),
            ("normal_reply", "[4,2]", "sin cerrar"),
            ("replace_available", False, "replace=false"),
        )
        for field, value, message in cases:
            with self.subTest(field=field):
                self.setUp()
                setattr(self.http, field, value)
                with self.assertRaisesRegex(RwsError, message):
                    self.installer.install(authorized=lambda: True)
                self.assertEqual(self.writes(), [])

    def test_exact_additive_write_set_and_postcheck(self):
        report = self.installer.install(authorized=lambda: True)
        self.assertEqual(report["protocol_version"], 2)
        self.assertEqual(report["sequence"], 0)
        self.assertEqual(report["pointer"], (NORMAL_MODULE, "ServeJointOrders"))
        self.assertEqual(report["check_program"], "pending")
        self.assertTrue(self.http.module_present)
        self.assertFalse(self.http.mastership)
        request, put, load, release = self.writes()
        self.assertEqual(request, ("POST", f"{MASTER}?action=request", None))
        self.assertEqual(put, ("PUT", report["staging_file"].replace("$HOME/", "/fileservice/$home/"), self.source))
        self.assertEqual(load, ("POST", LOAD, {"modulepath": report["staging_file"], "replace": "false"}))
        self.assertEqual(release, ("POST", f"{MASTER}?action=release", None))

    def test_corrupt_upload_and_state_race_block_load(self):
        for fault in ("bytes", "running", "already_present"):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == "bytes":
                    self.http.corrupt_upload = True
                elif fault == "running":
                    self.http.on_upload = lambda: setattr(self.http, "execution", "running")
                else:
                    self.http.on_upload = lambda: setattr(self.http, "module_present", True)
                with self.assertRaises(RwsError):
                    self.installer.install(authorized=lambda: True)
                self.assertNotIn(LOAD, [url for _, url, _ in self.writes()])
                self.assertFalse(self.http.mastership)

    def test_unconfirmed_load_is_never_retried(self):
        self.http.lose_load_response = True
        with self.assertRaises(RwsError) as first:
            self.installer.install(authorized=lambda: True)
        self.assertEqual(first.exception.state, "load_unconfirmed")
        self.assertTrue(self.http.module_present)
        self.assertFalse(self.http.mastership)
        with self.assertRaisesRegex(RwsError, "ya fue intentada"):
            self.installer.install(authorized=lambda: True)
        self.assertEqual([url for _, url, _ in self.writes()].count(LOAD), 1)

    def test_detects_unexpected_pointer_after_load_and_never_restores_it(self):
        self.http.on_load = lambda: setattr(self.http, "routine", "Main")
        with self.assertRaisesRegex(RwsError, "Cambió el PP"):
            self.installer.install(authorized=lambda: True)
        self.assertEqual(self.http.routine, "Main")
        self.assertFalse(self.http.mastership)
