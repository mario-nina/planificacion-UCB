"""Prepare the v2 server and hand the program pointer back to Normal."""

from dataclasses import dataclass, replace
from hashlib import sha256
from re import fullmatch
from time import monotonic, sleep
from xml.etree import ElementTree

from .rws_backend import (
    EXECUTION, MAX_SEQUENCE, MODULE_NAME as NORMAL, MODULE_SHA256 as NORMAL_SHA,
    ROUTINE as NORMAL_ROUTINE, MOVE_ROUTINE as NORMAL_MOVE, TASKS, ControllerStatus, RwsError,
)
from .rws_trajectory import MODULE_NAME as V2, ROUTINE as V2_ROUTINE, MOVE_ROUTINE as V2_MOVE
from .rws_trajectory_install import RwsTrajectoryInstaller


NORMAL_ENTRY = (NORMAL, NORMAL_ROUTINE)
V2_ENTRY = (V2, V2_ROUTINE)
ENTRIES = {NORMAL: NORMAL_ENTRY, V2: V2_ENTRY}
MOVES = {NORMAL: (NORMAL, NORMAL_MOVE), V2: (V2, V2_MOVE)}
OWN_POINTERS = tuple(ENTRIES.values()) + tuple(MOVES.values())
DEFAULT_ENTRY = ("mainmodule", "main")
STOP_FORM = {"stopmode": "instr", "usetsp": "normal"}
START_FORM = {
    "regain": "clear", "execmode": "continue", "cycle": "forever",
    "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false",
}


@dataclass(frozen=True)
class PreparationState:
    controller: ControllerStatus
    pointer: tuple[str, str]
    orders: dict
    level: str = "normal"


class RwsTrajectoryTransition:
    """Explicit button-driven handover through the existing Normal RWS session."""

    def __init__(self, normal, trajectory, normal_source):
        self.normal = normal
        self.trajectory = trajectory
        self.normal_source = normal_source
        self.task_path = f"{TASKS}/{normal.config.task}"
        self.v2_pointer_possible = False

    @staticmethod
    def _require_authorized(authorized):
        if not authorized():
            raise RwsError("Preparación RAPID revocada", state="not_authorized")

    def _snapshot(self, target, *, preserve=()):
        controller = self.normal.connect()
        if (controller.mode != "AUTO" or controller.motors != "motoron"
                or controller.execution not in {"stopped", "running"}):
            raise RwsError("Preparación RAPID requiere AUTO y motores habilitados")
        self.normal._check_active_tasks()
        raw = tuple(part.lower() for part in self.normal._pointer())
        pointer = next((entry for entry in OWN_POINTERS
                        if raw == tuple(part.lower() for part in entry)), raw)
        level = self.normal._status_field(self.task_path, "execlevel").lower()
        if level not in {"normal", "user"}:
            raise RwsError("Nivel RAPID incompatible; no se cambia el PP", state="foreign_program")
        names = dict.fromkeys((target, *preserve))
        if pointer in OWN_POINTERS:
            names[pointer[0]] = None
        elif pointer not in {DEFAULT_ENTRY, ("", "")} or controller.execution == "running":
            # A deliberate prepare click can hand over a foreign program. Do not
            # ignore any outstanding orders in either of our resident servers.
            names.update(dict.fromkeys(ENTRIES))
        orders = {name: self._validated_order(
            name, allow_abandoned=(controller.execution == "stopped" or pointer[0].lower() != name.lower()))
            for name in names}
        if pointer in OWN_POINTERS and orders[pointer[0]] is None:
            raise RwsError("El PP apunta a un módulo que no figura en T_ROB1", state="module_missing")
        return PreparationState(controller, pointer, orders, level)

    def _idle_snapshot(self, target, *, preserve=()):
        initial = self._snapshot(target, preserve=preserve)
        if initial.controller.execution != "running" or initial.pointer not in MOVES.values():
            return initial
        # The terminal record may be visible before RAPID returns from Run*Order.
        # Let the server finish its own epilogue instead of treating it as foreign.
        deadline = monotonic() + 0.5
        while monotonic() < deadline:
            sleep(0.1)
            current = self._snapshot(target, preserve=initial.orders)
            if current.orders != initial.orders or current.level != initial.level:
                raise RwsError("Cambió la orden RAPID mientras retornaba al servidor")
            if current.pointer == ENTRIES[initial.pointer[0]] and current.controller == initial.controller:
                return current
            if current != initial:
                raise RwsError("Cambió el estado durante el retorno RAPID")
        if initial.orders[initial.pointer[0]][1] in (3, 4, 5):
            # A closed order may leave the PCP on its last instruction. STOP
            # and re-enter the server rather than restarting Run*Order.
            return initial
        raise RwsError("PP en movimiento sin acuse terminal compatible", state="server_returning")

    def _normal_level(self, target, stopped, authorized):
        if stopped.level == "normal":
            return stopped
        self._require_authorized(authorized)
        self.normal._mastership("request")
        try:
            self._require_same(target, stopped)
            self._require_authorized(authorized)
            self.normal._request("POST", self.task_path + "?action=abortexeclevel", 204)
            current = self._snapshot(target, preserve=stopped.orders)
            if (current.controller != stopped.controller or current.orders != stopped.orders
                    or current.level != "normal"):
                raise RwsError("No se confirmó el retorno al nivel normal RAPID")
            return current
        finally:
            self._release()

    def _validated_order(self, name, *, allow_abandoned=False):
        backend = self.normal if name == NORMAL else self.trajectory
        if backend._sequence is not None:
            raise RwsError(f"{name} conserva una orden local sin resolver", state="pending_order")
        if not backend._loaded_module():
            return None
        if name == NORMAL:
            if not isinstance(self.normal_source, bytes) or sha256(self.normal_source).hexdigest() != NORMAL_SHA:
                raise RwsError("La fuente local del servidor Normal no coincide con la versión fijada")
            verify_resident = getattr(self.normal, "verify_server_identity", self.normal._verify_loaded_module)
            verify_resident(self.normal_source)
        else:
            self.trajectory._verify_identity(force=True)
        backend._protocol_version()
        return backend._preparation_sequence() if allow_abandoned else backend._closed_sequence()

    def _require_same(self, target, expected):
        if self._snapshot(target, preserve=expected.orders) != expected:
            raise RwsError("Cambió el estado, PP o una orden RAPID durante la preparación")

    def _request_xml(self, path):
        try:
            return ElementTree.fromstring(self.normal._request("GET", path, 200))
        except ElementTree.ParseError:
            raise RwsError("Respuesta XHTML RAPID ilegible") from None

    def _check_build_errors(self):
        root = self._request_xml(self.task_path + "/program/builderror?start=1&limit=30")
        state = next((item for item in root.iter() if item.get("class") == "state"), None)
        if state is None:
            raise RwsError("RWS no devolvió un listado de errores RAPID verificable")
        if any(item.tag.split("}")[-1] == "li" or item.get("rel") == "next" for item in state.iter()):
            raise RwsError("RAPID tiene errores de compilación", state="build_errors")

    def _verify_stop_action(self):
        root = self._request_xml(EXECUTION + "?action=show")
        choices = {
            select.get("name"): {item.get("value") for item in select}
            for form in root.iter()
            if form.get("id") == "stop" and form.get("method", "").lower() == "post"
            for select in form
            if select.tag.split("}")[-1] == "select"
        }
        if any(value not in choices.get(key, ()) for key, value in STOP_FORM.items()):
            raise RwsError("El controlador no ofrece STOP instr para las tareas normales")

    def _release(self):
        try:
            self.normal._mastership("release")
        except RwsError as error:
            raise RwsError(f"Liberación de mastership sin confirmar · {error}",
                           error.status, "mastership_unconfirmed") from None

    def _stop_idle(self, target, initial, authorized):
        if initial.pointer in MOVES.values() and initial.orders[initial.pointer[0]][1] not in (3, 4, 5):
            raise RwsError("RAPID todavía tiene una orden sin acuse terminal", state="pending_order")
        self._verify_stop_action()
        self._require_authorized(authorized)
        self.normal._mastership("request")
        try:
            current = self._snapshot(target, preserve=initial.orders)
            returned = (initial.pointer in MOVES.values()
                        and current == replace(initial, pointer=ENTRIES[initial.pointer[0]]))
            if current != initial and not returned:
                raise RwsError("Cambió el estado antes de STOP")
            initial = current
            self._require_authorized(authorized)
            try:
                self.normal._request("POST", EXECUTION + "?action=stop", 204, STOP_FORM)
            except RwsError as error:
                if error.status is not None and 400 <= error.status < 500:
                    raise
            stopped = replace(initial, controller=replace(initial.controller, execution="stopped"))
            deadline = monotonic() + 4.0
            while True:
                try:
                    current = self._snapshot(target, preserve=initial.orders)
                except RwsError as error:
                    raise RwsError(f"STOP sin confirmar · {error}", error.status, "stop_unconfirmed") from None
                if current == stopped:
                    return current
                # STOP instr may advance the foreign routine to another instruction
                # before stopping. Do not accept a different program/module.
                if ((initial.pointer not in OWN_POINTERS or initial.pointer in MOVES.values())
                        and current.controller == stopped.controller
                        and current.pointer[0] == initial.pointer[0] and current.level == initial.level
                        and current.orders == initial.orders
                        and (initial.pointer not in OWN_POINTERS or current.pointer in OWN_POINTERS)):
                    return current
                if current != initial or monotonic() >= deadline:
                    raise RwsError("STOP sin confirmar en el mismo PP y orden", state="stop_unconfirmed")
                sleep(0.1)
        finally:
            self._release()

    def _recovery_installer(self):
        self.trajectory._optional_controller_metadata()
        version = self.trajectory.controller_metadata.get("robotware_version", "")
        if not fullmatch(r"6\.12(?:\.[0-9]+){0,3}", version):
            raise RwsError("La carga v2 requiere RobotWare 6.12 confirmado", state="robotware_incompatible")
        installer = RwsTrajectoryInstaller(self.trajectory.config, self.trajectory.module_source,
                                           http=self.normal.http, normal=self.normal,
                                           allow_abandoned_normal=True)
        if not installer._load_actions():
            raise RwsError("RWS no confirmó compilación de T_ROB1")
        return installer

    def _install_v2(self, installer, initial, authorized):
        self._require_authorized(authorized)

        def load_authorized():
            if not authorized():
                return False
            self._require_same(V2, initial)
            return True

        installer.install(authorized=load_authorized)
        installed = replace(initial, orders={**initial.orders, V2: (0, 0)})
        self._require_authorized(authorized)
        self.normal._mastership("request")
        try:
            self._require_same(V2, installed)
            self._require_authorized(authorized)
            self.normal._request("POST", self.task_path + "?action=build", (200, 204))
            self._require_same(V2, installed)
            self._check_build_errors()
        finally:
            self._release()
        return installed

    def prepare_v2(self, authorized, *, recover_missing=False):
        self._require_authorized(authorized)
        initial = self._idle_snapshot(V2)
        self.v2_pointer_possible |= initial.pointer == V2_ENTRY
        if initial.pointer == V2_ENTRY and initial.controller.execution == "running" and initial.level == "normal":
            if recover_missing:
                raise RwsError(f"{V2} ya está instalado", state="module_present")
            self.trajectory.prepare()
            return "already_running"

        installer = None
        if initial.orders[V2] is None:
            if not recover_missing:
                raise RwsError(f"Falta {V2} en T_ROB1", state="module_missing")
            initial = self._idle_snapshot(V2, preserve=(NORMAL,))
            if initial.orders[NORMAL] is None:
                raise RwsError("Instala y verifica Normal antes de recuperar v2", state="module_missing")
            installer = self._recovery_installer()
        elif recover_missing:
            raise RwsError(f"{V2} ya está instalado", state="module_present")
        elif initial.orders[V2][0] >= MAX_SEQUENCE:
            raise RwsError("Secuencia RAPID agotada; requiere reinicialización supervisada")

        self._check_build_errors()
        stopped = (self._stop_idle(V2, initial, authorized)
                   if initial.controller.execution == "running" else initial)
        stopped = self._normal_level(V2, stopped, authorized)
        if installer is not None:
            stopped = self._install_v2(installer, stopped, authorized)

        self._require_authorized(authorized)
        self.normal._mastership("request")
        try:
            self._require_same(V2, stopped)
            self._check_build_errors()
            self._require_authorized(authorized)
            self.v2_pointer_possible = True
            self.normal._request("POST", self.task_path + "/pcp?action=set-pp-routine", 204, {
                "module": V2, "routine": V2_ROUTINE, "userlevel": "false",
            })
            at_entry = replace(stopped, pointer=V2_ENTRY)
            self._require_same(V2, at_entry)
            self._require_authorized(authorized)
            try:
                self.normal._request("POST", EXECUTION + "?action=start", 204, START_FORM)
                expected_orders = dict(at_entry.orders)
                if len(expected_orders[V2]) == 3:
                    self.trajectory._wait_abandoned_restart(expected_orders[V2])
                    expected_orders[V2] = (expected_orders[V2][0], 5)
                self._require_same(V2, replace(at_entry, orders=expected_orders,
                                               controller=ControllerStatus("AUTO", "motoron", "running")))
                if self.trajectory.prepare() != stopped.orders[V2][0]:
                    raise RwsError("Cambió la secuencia de trayectorias al iniciar RAPID")
            except RwsError as error:
                raise RwsError(f"Inicio de trayectorias sin confirmar · {error}", error.status,
                               "start_unconfirmed") from None
        finally:
            self._release()
        return "started"

    def prepare_normal(self, authorized, module_source=None, recover_missing=False):
        """Use the original start routine, with an authorized idle PP handover."""
        self._require_authorized(authorized)
        pointer = self.normal._pointer()
        status = self.normal.connect()
        raw = tuple(part.lower() for part in pointer)
        normal_raw = tuple(part.lower() for part in NORMAL_ENTRY)
        normal_move_raw = tuple(part.lower() for part in MOVES[NORMAL])
        level = self.normal._status_field(self.task_path, "execlevel").lower()
        # Keep the original verified path for Normal already running or stopped
        # at its own entry/Main. No extra writes and no dependency on v2.
        if ((raw == normal_raw and level == "normal")
                or (status.execution == "stopped" and level == "normal"
                    and raw in {DEFAULT_ENTRY, ("", ""), normal_move_raw})):
            self.v2_pointer_possible = False
            return self.normal.start_server(authorized=authorized, module_source=module_source,
                                            recover_missing=recover_missing, recover_abandoned=True)
        initial = self._idle_snapshot(NORMAL, preserve=(V2,))
        if initial.pointer == NORMAL_ENTRY and initial.controller.execution == "running" and initial.level == "normal":
            return self.normal.start_server(authorized=authorized, module_source=module_source,
                                            recover_missing=recover_missing, recover_abandoned=True)
        if initial.orders[NORMAL] is None and not recover_missing:
            raise RwsError("Falta el servidor Normal", state="module_missing")
        self._require_authorized(authorized)
        stopped = (self._stop_idle(NORMAL, initial, authorized)
                   if initial.controller.execution == "running" else initial)
        stopped = self._normal_level(NORMAL, stopped, authorized)

        def transition_authorized():
            if not authorized():
                return False
            current = tuple(part.lower() for part in self.normal._pointer())
            if current not in {tuple(part.lower() for part in stopped.pointer), normal_raw}:
                raise RwsError("Cambió el PP durante la preparación de Normal")
            actual = self.trajectory._preparation_sequence() if self.trajectory._loaded_module() else None
            if actual != stopped.orders[V2]:
                raise RwsError("Cambió la orden v2 al devolver el control a Normal")
            return True

        self._require_same(NORMAL, stopped)
        result = self.normal.start_server(authorized=transition_authorized, module_source=module_source,
                                          recover_missing=recover_missing, recover_abandoned=True)
        self.v2_pointer_possible = False
        return result
