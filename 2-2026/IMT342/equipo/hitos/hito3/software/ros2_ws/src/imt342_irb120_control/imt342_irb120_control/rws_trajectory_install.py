"""One additive load of the pinned trajectory module, independent of motion control."""

from dataclasses import replace
from re import fullmatch
from uuid import uuid4
from xml.etree import ElementTree

from .rws_backend import (
    MASTER, MODULE_NAME as NORMAL_MODULE, TASKS, RwsError, RwsRapidBackend,
)
from .rws_trajectory import MODULE_NAME, MODULE_SHA256, RwsTrajectoryBackend


class RwsTrajectoryInstaller:
    """Inspect and add the v2 module. Preparation, build and execution are separate."""

    def __init__(self, config, module_source, http=None, *, normal=None, allow_abandoned_normal=False):
        self._trajectory = RwsTrajectoryBackend(config, module_source, http)
        self._normal = (RwsRapidBackend(replace(config, module=NORMAL_MODULE), self._trajectory.http)
                        if normal is None else normal)
        if (self._normal.http is not self._trajectory.http
                or (self._normal.config.task, self._normal.config.module, self._normal.config.mechunit)
                != (config.task, NORMAL_MODULE, config.mechunit)):
            raise ValueError("La carga v2 requiere Normal en la misma sesión RWS de T_ROB1/ROB_1")
        self._load_path = f"{TASKS}/{config.task}?action=loadmod"
        self._attempted = False
        self._allow_abandoned_normal = allow_abandoned_normal

    def _load_actions(self):
        raw = self._trajectory._request("GET", f"{TASKS}/T_ROB1?action=show", 200)
        try:
            root = ElementTree.fromstring(raw)
        except ElementTree.ParseError:
            raise RwsError("RWS devolvió un formulario de operaciones RAPID ilegible") from None
        actions = {
            element.get("id"): element for element in root.iter()
            if element.tag.split("}")[-1] == "form" and element.get("method", "").lower() == "post"
        }
        form = actions.get("loadmod")
        if form is None or not any(
            choice.get("value") == "false"
            for select in form.iter() if select.get("name") == "replace"
            for choice in select
        ):
            raise RwsError("RWS no confirmó la opción loadmod con replace=false")
        return "build" in actions

    def _snapshot(self):
        trajectory = self._trajectory
        status = trajectory.connect()
        if status.execution != "stopped":
            raise RwsError("Detén RAPID de forma supervisada antes de cargar el módulo")
        trajectory._check_active_tasks()
        if not self._normal._loaded_module():
            raise RwsError(f"No se confirmó {NORMAL_MODULE} en T_ROB1")
        self._normal._protocol_version()
        record = (self._normal._preparation_sequence() if self._allow_abandoned_normal
                  else self._normal._closed_sequence())
        normal_sequence, normal_phase = record[:2]
        return {
            "execution": status.execution,
            "mode": status.mode,
            "motors": status.motors,
            "pointer": trajectory._pointer(),
            "normal_sequence": normal_sequence,
            "normal_phase": normal_phase,
            "normal_record": record,
        }

    def preflight(self):
        """Confirm hardware, stopped RAPID and add-only capability via GET."""
        with self._trajectory._lock:
            return self._preflight()

    def _preflight(self):
        trajectory = self._trajectory
        trajectory.controller_metadata.clear()
        trajectory._optional_controller_metadata()
        version = trajectory.controller_metadata.get("robotware_version", "")
        if not fullmatch(r"6\.12(?:\.[0-9]+){0,3}", version):
            raise RwsError("Se requiere RobotWare 6.12 confirmado para esta carga")
        if trajectory._loaded_module():
            raise RwsError(f"{MODULE_NAME} ya existe. No se reemplazará.", state="module_present")
        state = self._snapshot()
        return {
            **state,
            "task": "T_ROB1",
            "module": MODULE_NAME,
            "sha256": MODULE_SHA256,
            "robotware_version": version,
            "build_advertised": self._load_actions(),
            "replace": False,
        }

    def _check_before_load(self, expected_state):
        if self._trajectory._loaded_module():
            raise RwsError(f"{MODULE_NAME} apareció en T_ROB1. No se reemplazará.")
        state = self._snapshot()
        if any(expected_state[key] != value for key, value in state.items()):
            raise RwsError("Cambió el estado del controlador o la orden Normal. Repite el diagnóstico.")

    def install(self, *, authorized=lambda: False):
        """Upload once, verify staged bytes and load without replacing any module."""
        with self._trajectory._lock:
            return self._install(authorized)

    def _install(self, authorized):
        if self._attempted:
            raise RwsError("La carga ya fue intentada. Inspecciona T_ROB1 antes de considerar otra.")
        if not authorized():
            raise RwsError("Carga RAPID sin autorización explícita")
        plan = self._preflight()
        name = f"imt342-{uuid4().hex}.mod"
        remote = f"/fileservice/$home/{name}"
        trajectory = self._trajectory
        normal = self._normal
        normal._request("GET", remote, 404)
        if not authorized():
            raise RwsError("Autorización de carga revocada")
        self._attempted = True
        normal._mastership("request")
        try:
            self._check_before_load(plan)
            if not authorized():
                raise RwsError("Autorización de carga revocada")
            try:
                normal._request("PUT", remote, (200, 201), trajectory.module_source)
                staged_source = normal._request("GET", remote, 200)
                if staged_source != trajectory.module_source.decode("ascii"):
                    raise RwsError("La copia remota no coincide con la fuente fijada")
            except RwsError as error:
                raise RwsError(f"Archivo temporal {name}: {error}", error.status, "stage_unconfirmed") from None
            self._check_before_load(plan)
            if not authorized():
                raise RwsError(f"Autorización de carga revocada. Archivo temporal: {name}")
            try:
                normal._request("POST", self._load_path, 204,
                                {"modulepath": f"$HOME/{name}", "replace": "false"})
                trajectory._verify_identity(force=True)
                protocol = trajectory._protocol_version()
                sequence, phase = trajectory._closed_sequence()
                observed = self._snapshot()
                if any(plan[key] != value for key, value in observed.items()):
                    raise RwsError("Cambió el PP, la orden Normal o el estado del controlador")
            except RwsError as error:
                raise RwsError(
                    f"Carga o estado posterior sin confirmar · {error} · archivo temporal: {name}",
                    error.status, "load_unconfirmed",
                ) from None
        finally:
            try:
                normal._mastership("release")
            except RwsError as error:
                raise RwsError(
                    f"Mastership sin liberación confirmada · {error} · archivo temporal: {name}",
                    error.status, "mastership_unconfirmed",
                )
        return {
            **observed,
            "module": MODULE_NAME,
            "sha256": MODULE_SHA256,
            "staging_file": f"$HOME/{name}",
            "protocol_version": protocol,
            "sequence": sequence,
            "phase": phase,
            "check_program": "pending",
        }

    def close(self):
        self._trajectory.close()
