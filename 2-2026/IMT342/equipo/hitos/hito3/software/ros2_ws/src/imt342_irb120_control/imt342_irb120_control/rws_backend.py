"""ABB RWS 1.0 transport and IMT342 discrete RAPID protocol."""

from dataclasses import dataclass, field
from hashlib import sha256
from json import loads
from math import degrees, isfinite, radians
from re import finditer, fullmatch, sub
from threading import RLock
from time import monotonic, sleep
from urllib.parse import urlsplit
from uuid import uuid4
from xml.etree import ElementTree

from .state import Feedback, Result, finite_positions, validate_target

MOTOR = "/rw/panel/ctrlstate"
MODE = "/rw/panel/opmode"
EXECUTION = "/rw/rapid/execution"
MASTER = "/rw/mastership/rapid"
TASKS = "/rw/rapid/tasks"
MODULES = "/rw/rapid/modules"
MODULE_NAME = "Imt342JointServer"
MODULE_SHA256 = "e24649eb6e6d4d73d3534e11ec972a923f7a4bd9c1a1ee479971bcfa6e8e1a2c"
MODULE_FILE_ROUTE = r"/fileservice/\$home/imt342-[0-9a-f]{32}\.mod"
MODULE_EXPORT_PATH = r"/hd0a/TEMPpusres\.[0-9]{1,20}"
RAPID_NUMBER = r"(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?"
RAPID_TOKEN = (
    rf'"(?:[^"\r\n]|"")*"|![^\r\n]*|[A-Za-z_][A-Za-z_0-9]*|'
    rf'{RAPID_NUMBER}|:=|<>|<=|>=|[^\s]'
)
ROUTINE = "ServeJointOrders"
MOVE_ROUTINE = "RunJointOrder"
MAX_SEQUENCE = 8_000_000  # Integers exactly representable by a RAPID num.


class RwsError(RuntimeError):
    """RWS transport, protocol or controller failure."""

    def __init__(self, message, status=None, state="failed"):
        super().__init__(message)
        self.status = status
        self.state = state


@dataclass(frozen=True)
class RwsConfig:
    base_url: str
    username: str = field(repr=False)
    password: str = field(repr=False)
    task: str
    module: str
    mechunit: str
    tolerance_rad: float
    timeout: float = 2.0
    ack_timeout: float = 5.0

    def __post_init__(self):
        origin = urlsplit(self.base_url)
        if (
            origin.scheme not in ("http", "https")
            or not origin.hostname
            or origin.username is not None
            or origin.path not in ("", "/")
            or origin.query
            or origin.fragment
        ):
            raise ValueError("Se requiere la URL base de RWS sin credenciales ni ruta")
        for identifier in (self.task, self.module, self.mechunit):
            if not fullmatch(r"[A-Za-z][A-Za-z0-9_]*", identifier):
                raise ValueError("Identificador ABB inválido")
        if not all(isfinite(x) and x > 0 for x in
                   (self.timeout, self.ack_timeout, self.tolerance_rad)):
            raise ValueError("Timeouts y tolerancia deben ser positivos y finitos")


class DigestHttp:
    """Persistent HTTP session, restricted to the configured origin."""

    def __init__(self, config):
        import requests
        from requests.auth import HTTPDigestAuth

        if not config.username or not config.password:
            raise ValueError("Configure las credenciales RWS fuera del código")
        self.config = config
        self.session = requests.Session()
        self.session.trust_env = False
        self.session.auth = HTTPDigestAuth(config.username, config.password)
        self.error = requests.RequestException
        self._used = False
        self._closed = False

    def request(self, method, path, body=None):
        module_file = method in ("PUT", "GET") and fullmatch(MODULE_FILE_ROUTE, path)
        module_export = method == "GET" and fullmatch("/fileservice" + MODULE_EXPORT_PATH, path)
        if (not path.startswith("/rw/") and not module_file and not module_export
                and (method, path) != ("GET", "/ctrl/identity")):
            raise ValueError("Ruta RWS inválida")
        try:
            response = self.session.request(
                method, self.config.base_url.rstrip("/") + path, data=body,
                headers=(
                    {"Content-Type": "application/x-www-form-urlencoded"}
                    if method.upper() == "POST" else
                    {"Content-Type": "application/octet-stream"} if method == "PUT" else None
                ),
                timeout=self.config.timeout, allow_redirects=False,
            )
        except self.error as exc:
            raise RwsError(f"RWS {method} {path}: {type(exc).__name__}", state="transport_error") from None
        if 200 <= response.status_code < 300:
            self._used = True
        return response.status_code, response.text

    def close(self):
        # RWS 1.0: release the resources of this same authenticated session.
        # Do not create a session just to log out an unused client.
        if self._closed:
            return
        self._closed = True
        try:
            if self._used:
                self.session.request("GET", self.config.base_url.rstrip("/") + "/logout",
                                     timeout=self.config.timeout, allow_redirects=False)
        except self.error:
            pass  # Controller may already be offline; local cleanup must proceed.
        finally:
            self.session.close()


@dataclass(frozen=True)
class ControllerStatus:
    mode: str
    motors: str
    execution: str


class RwsRapidBackend:
    """RWS reads plus one in-flight, sequence-correlated discrete command."""

    def __init__(self, config, http=None, clock=monotonic):
        self.config = config
        self.http = DigestHttp(config) if http is None else http
        self.clock = clock
        self._lock = RLock()
        self._sequence = None
        self._goal = None
        self._sent_at = None
        self._stop_requested = False
        self._result = Result.IDLE
        self.last_error = ""
        self.final_feedback = None
        self.final_error_rad = None
        self._motion_evidence = {}
        self.controller_metadata = {}

    @property
    def motion_evidence(self):
        with self._lock:
            return dict(self._motion_evidence)

    def _request(self, method, path, expected, body=None):
        try:
            status, response = self.http.request(method, path, body)
        except OSError as exc:
            raise RwsError(f"RWS {method}: {type(exc).__name__}") from None
        if status not in (expected if isinstance(expected, tuple) else (expected,)):
            raise RwsError(f"RWS {method} {path}: HTTP {status}", status)
        return response

    def _get(self, path, *fields):
        raw = self._request("GET", path + "?json=1", 200)
        try:
            rows = loads(raw)["_embedded"]["_state"]
            return next(row for row in rows
                        if isinstance(row, dict) and all(field in row for field in fields))
        except (ValueError, KeyError, TypeError, StopIteration):
            raise RwsError(f"RWS {path}: respuesta JSON sin estado esperado") from None

    def _status_field(self, path, field):
        value = self._get(path, field)[field]
        if not isinstance(value, str) or not fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,31}", value):
            raise RwsError(f"RWS {path}: valor de estado inválido")
        return value

    def connect(self):
        """Read controller status. Leave RAPID, pointer and motors untouched."""
        with self._lock:
            return ControllerStatus(
                self._status_field(MODE, "opmode").upper(),
                self._status_field(MOTOR, "ctrlstate").lower(),
                self._status_field(EXECUTION, "ctrlexecstate").lower(),
            )

    def read_state(self):
        with self._lock:
            path = f"/rw/motionsystem/mechunits/{self.config.mechunit}/jointtarget"
            axes = tuple(f"rax_{i}" for i in range(1, 7))
            row = self._get(path, *axes)
            try:
                if any(isinstance(row[axis], bool) for axis in axes):
                    raise ValueError("Eje booleano")
                positions = finite_positions(row[axis] for axis in axes)
            except (ValueError, TypeError, OverflowError):
                raise RwsError("RWS jointtarget: J1–J6 no válidas") from None
            return Feedback(tuple(map(radians, positions)), self.clock(), "rws")

    def _symbol(self, name):
        return f"/rw/rapid/symbol/data/RAPID/{self.config.task}/{self.config.module}/{name}"

    def _record(self, name, pattern):
        value = self._get(self._symbol(name), "value")["value"]
        match = fullmatch(pattern, str(value).strip())
        if not match:
            raise RwsError(f"RAPID {name}: formato incompatible")
        if len(match[1]) > len(str(MAX_SEQUENCE)):
            raise RwsError(f"RAPID {name}: secuencia fuera de rango")
        result = tuple(map(int, match.groups()))
        if result[0] > MAX_SEQUENCE:
            raise RwsError(f"RAPID {name}: secuencia fuera de rango")
        return result

    def _reply(self):
        return self._record("rbReply", r"\[\s*([0-9]+)\s*,\s*([0-5])\s*\]")

    def _order_sequence(self):
        return self._record("pcOrder", r"\[\s*([0-9]+)\s*,\s*\[[\s\S]*\]\s*\]")[0]

    def _pointer(self):
        path = f"/rw/rapid/tasks/{self.config.task}/pcp"
        raw = self._request("GET", path, 200)
        try:
            root = ElementTree.fromstring(raw)
            info = next(
                item for item in root.iter()
                if item.get("title") == "progpointer" and item.tag.split("}")[-1] == "li"
            )
            fields = {item.get("class"): item.text for item in info}
            if "modulename" not in fields and "modulemame" not in fields:
                raise ValueError("Falta el módulo del PCP")
            module = (fields.get("modulename") or fields.get("modulemame") or "").strip()
            routine = (fields["routinename"] or "").strip()
            if not module and not routine:
                return "", ""
            if not all(fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) for name in (module, routine)):
                raise ValueError("Identidad PCP inválida")
            if module.lower() == self.config.module.lower():
                module = self.config.module
                routine = next((name for name in (ROUTINE, MOVE_ROUTINE) if name.lower() == routine.lower()),
                               routine)
            return module, routine
        except (ElementTree.ParseError, StopIteration, KeyError, ValueError):
            raise RwsError(f"RWS PCP: puntero de programa de {self.config.task} ilegible. "
                           "Compruébalo en FlexPendant") from None

    def _protocol_version(self):
        version = self._get(self._symbol("rbVersion"), "value")["value"]
        if version != "1" and (type(version) is not int or version != 1):
            raise RwsError("Versión RAPID incompatible")
        return 1

    def _closed_sequence(self):
        order_sequence = self._order_sequence()
        reply_sequence, phase = self._reply()
        if order_sequence != reply_sequence or phase in (1, 2):
            raise RwsError("RAPID conserva una orden anterior sin cerrar")
        return order_sequence, phase

    def _preparation_sequence(self):
        """Read raw persisted order; a third field identifies an unclosed record.

        Only preparation may use this snapshot. Motion authorization still
        requires _closed_sequence().
        """
        order = self._order_sequence()
        reply, phase = self._reply()
        return (order, phase) if order == reply and phase not in (1, 2) else (order, phase, reply)

    def _wait_abandoned_restart(self, before):
        """The pinned RAPID entry discards an abandoned order as phase 5."""
        expected = (before[0], 5)
        deadline = monotonic() + 3.0
        while True:
            if self.connect() != ControllerStatus("AUTO", "motoron", "running"):
                raise RwsError("RAPID no está en marcha tras recuperar la orden")
            current = self._preparation_sequence()
            if current == expected:
                return
            if current != before or monotonic() >= deadline:
                raise RwsError("No se confirmó la cancelación de la orden abandonada (fase 5)")
            sleep(0.05)

    def _observe_uncertain_motion(self, reason):
        self.last_error = reason
        self._motion_evidence["uncertainty_observed"] = True
        return Result.UNKNOWN

    def _ready_sequence(self):
        status = self.connect()
        if (status.mode, status.motors, status.execution) != ("AUTO", "motoron", "running"):
            raise RwsError(
                f"IRC5 no listo para este cliente AUTO: {status.mode}, "
                f"{status.motors}, RAPID {status.execution}"
            )
        module, routine = self._pointer()
        if module != self.config.module or routine not in (ROUTINE, MOVE_ROUTINE):
            raise RwsError(f"PP fuera de {self.config.module}/{ROUTINE}")
        self._protocol_version()
        order_sequence, phase = self._closed_sequence()
        if phase == 4 and routine == MOVE_ROUTINE:
            raise RwsError("RAPID está saliendo de una ejecución con error")
        if order_sequence >= MAX_SEQUENCE:
            raise RwsError("Secuencia RAPID agotada; requiere reinicialización supervisada")
        return order_sequence

    def prepare(self):
        """Check controller, live PP, protocol and persistent order state."""
        with self._lock:
            return self._ready_sequence()

    def _check_active_tasks(self):
        # START is controller-wide. Require the configured task to be the only
        # selected task before issuing it, without changing task activation.
        raw = self._request("GET", TASKS, 200)
        try:
            root = ElementTree.fromstring(raw)
        except ElementTree.ParseError:
            raise RwsError("RAPID: lista de tareas ilegible") from None
        tasks = [
            {item.get("class"): item.text for item in row}
            for row in root.iter()
            if row.tag.split("}")[-1] == "li" and row.get("class") == "rap-task-li"
        ]
        if any(row.get("active") not in {"On", "Off"} or not row.get("name") for row in tasks):
            raise RwsError("RAPID: estado de activación de tareas ilegible")
        active = [row["name"] for row in tasks if row["active"] == "On"]
        if any(not fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) for name in active):
            raise RwsError("RAPID: identificación de tareas inválida")
        self.controller_metadata["active_tasks"] = active
        if active != [self.config.task]:
            raise RwsError("Inicio RAPID requiere que solo la tarea configurada esté activa")

    def _optional_controller_metadata(self):
        lookups = (
            ("/ctrl/identity", "ctrl-name", "controller_name", r"[A-Za-z0-9_.-]{1,64}"),
            ("/rw/system", "rwversion", "robotware_version", r"[0-9]+(?:\.[0-9]+){1,4}"),
        )
        for path, tag, key, pattern in lookups:
            try:
                raw = self._request("GET", path, 200)
                root = ElementTree.fromstring(raw)
                value = next(item.text.strip() for item in root.iter()
                             if item.get("class") == tag and item.text)
            except (RwsError, ElementTree.ParseError, StopIteration):
                continue
            if fullmatch(pattern, value):
                self.controller_metadata[key] = value

    @staticmethod
    def _program_text(text):
        # Keep string literals intact, including case, whitespace and comment markers.
        text = sub(r"^\s*%%%[^%]*%%%\s*", "", text.lstrip("\ufeff"), count=1)
        return [
            token if token.startswith('"') else token.lower()
            for match in finditer(RAPID_TOKEN, text)
            if not (token := match[0]).startswith("!")
        ]

    @staticmethod
    def _without_persistent_values(tokens):
        # Only the numeric initializers of the two known v1 records can vary.
        value = rf"[+-]?{RAPID_NUMBER}"
        axes = rf"\[{value}(?:,{value}){{5}}\]"
        records = (
            (["pers", "imtjointorder", "pcorder", ":="], rf"\[{value},\[{axes},{axes}\]\]"),
            (["pers", "imtjointreply", "rbreply", ":="], rf"\[{value},{value}\]"),
        )
        normalized = tokens.copy()
        for declaration, pattern in records:
            for index in range(len(normalized) - len(declaration)):
                start = index + len(declaration)
                if normalized[index:start] != declaration:
                    continue
                try:
                    end = normalized.index(";", start)
                except ValueError:
                    break
                if fullmatch(pattern, "".join(normalized[start:end])):
                    normalized[start:end] = ["[persistent value]"]
                break
        return normalized

    def _loaded_module(self):
        path = f"{MODULES}?task={self.config.task}"
        try:
            raw = self._request("GET", path, 200)
        except RwsError as error:
            raise RwsError(f"Fallo al consultar módulos RAPID en {self.config.task} · {error}",
                           error.status, "module_list_error") from None
        try:
            root = ElementTree.fromstring(raw)
            state = next(item for item in root.iter() if item.get("class") == "state")
            listing = next(item for item in state if item.tag.split("}")[-1] == "ul")
            if any(item.get("class") != "rap-module-info-li" for item in listing):
                raise ValueError("Lista inválida")
            names = [
                next(child.text for child in item if child.get("class") == "name")
                for item in listing
            ]
            if any(not name or not fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) for name in names):
                raise ValueError("Lista inválida")
            return any(name.lower() == self.config.module.lower() for name in names)
        except (ElementTree.ParseError, StopIteration, ValueError):
            raise RwsError(f"RWS devolvió una lista de módulos RAPID ilegible en {self.config.task}",
                           state="module_list_error") from None

    @staticmethod
    def _source_fields(raw, resource, text_field=None):
        try:
            root = ElementTree.fromstring(raw)
        except ElementTree.ParseError:
            # Some RW6 replies contain a bare '<' in the RAPID span. Repair
            # only that span and unescaped delimiters in RWS hyperlink attributes.
            def escape_ampersands(value):
                return sub(r"&(?!amp;|lt;|gt;|apos;|quot;|#[0-9]+;|#x[0-9a-fA-F]+;)", "&amp;", value)

            raw = sub(r'''(<a\s+[^<>]*?\bhref\s*=\s*)(["'])(.*?)\2''',
                      lambda m: m[1] + m[2] + escape_ampersands(m[3]) + m[2], raw)
            if text_field:
                raw = sub(
                    rf'''(<span\s+class=["']{text_field}["']\s*>)([\s\S]*?)(</span>)''',
                    lambda m: m[1] + escape_ampersands(m[2]).replace("<", "&lt;") + m[3], raw,
                )
            root = ElementTree.fromstring(raw)
        block, = (item for item in root.iter() if item.get("class") == resource
                  and item.tag.split("}")[-1] == "li")
        fields = {}
        for item in block:
            if item.tag.split("}")[-1] != "span":
                continue
            name = item.get("class")
            if name in fields or len(item):
                raise ValueError("Campo RWS ambiguo")
            fields[name] = item.text or ""
        return fields

    def _read_source_ranges(self, path):
        extension = f"{path}?resource=module-extension&task={self.config.task}"

        def extent():
            raw = self._request("GET", extension, 200)
            fields = self._source_fields(raw, "rap-module-extension")
            return int(fields["num-of-lines"]), int(fields["count"])

        rows, change_count = extent()
        if not 1 <= rows <= 256 or change_count < 0:
            raise ValueError("Extensión RAPID no verificable")
        lines = []
        for start in range(1, rows + 1, 20):
            end = min(start + 19, rows)
            query = f"?task={self.config.task}&startrow={start}&startcol=1&endrow={end}&endcol=-1"
            raw = self._request("GET", path + query, 200)
            fields = self._source_fields(raw, "rap-mod-text", "text")
            chunk = fields["text"].splitlines()
            if len(chunk) != end - start + 1:
                raise ValueError("Rango RAPID incompleto")
            lines.extend(chunk)
        if extent() != (rows, change_count):
            raise ValueError("Cambió el módulo mientras se leían sus rangos")
        return "\n".join(lines)

    def _read_module_source(self):
        path = f"{MODULES}/{self.config.module}"
        raw = self._request("GET", f"{path}?resource=module-text&task={self.config.task}", 200)
        fields = self._source_fields(raw, "rap-module-text", "module-text")
        source = fields.get("module-text", "")
        if source.strip():
            return source
        if "file-path" not in fields:
            raise ValueError("RWS omitió el texto y la referencia al archivo")
        remote = fields["file-path"].strip().strip('"')
        if not fullmatch(MODULE_EXPORT_PATH, remote):
            raise RwsError("RWS devolvió una ruta de exportación de fuente no admitida. "
                           "Verifica el módulo en RobotStudio o FlexPendant",
                           state="module_source_unavailable")
        try:
            return self._request("GET", "/fileservice" + remote, 200)
        except RwsError as error:
            if error.status not in (400, 404):
                raise
            try:
                return self._read_source_ranges(path)
            except RwsError as range_error:
                if range_error.status in (400, 404):
                    raise RwsError(f"El archivo temporal no está disponible (HTTP {error.status}) "
                                   f"y RWS rechazó la lectura por rangos (HTTP {range_error.status})",
                                   error.status, "module_source_unavailable") from None
                raise

    def _verify_loaded_module(self, module_source):
        identity = f"{self.config.task}/{self.config.module}"
        try:
            tokens = self._program_text(self._read_module_source())
        except RwsError as error:
            if error.state != "failed":
                raise
            if error.status in (401, 403):
                detail, state = "Revisa los permisos UAS de lectura", "module_source_denied"
            elif error.status == 404:
                detail, state = "Fuente no disponible. Revisa el módulo en FlexPendant", "module_source_unavailable"
            else:
                detail, state = "Falló la consulta RWS de la fuente", "module_source_rws_error"
            raise RwsError(f"{identity} · {detail} · {error}", error.status, state) from None
        except ElementTree.ParseError:
            raise RwsError(f"{identity} · XHTML de la fuente RWS ilegible",
                           state="module_source_invalid") from None
        except (KeyError, ValueError):
            raise RwsError(f"{identity} · exportación RWS inválida o inconsistente. "
                           "Comprueba la fuente en RobotStudio o FlexPendant",
                           state="module_source_invalid") from None
        if len(tokens) < 3 or tokens[0] != "module" or tokens[-1] != "endmodule":
            raise RwsError(f"{identity} · la fuente RWS está incompleta. Revisa la exportación en FlexPendant",
                           state="module_source_invalid")
        if tokens[1] != self.config.module.lower():
            raise RwsError(f"{identity} · el nombre interno de la fuente no coincide. Revisa la exportación",
                           state="module_source_invalid")
        reference = self._program_text(module_source.decode("ascii"))
        if self._without_persistent_values(tokens) != self._without_persistent_values(reference):
            raise RwsError(f"{identity} · la fuente difiere del módulo v1 instalado. "
                           "Conserva el módulo y compara un respaldo en RobotStudio o FlexPendant",
                           state="module_modified")

    def _recover_missing_module(self, module_source, authorized):
        self._check_active_tasks()
        if self._loaded_module():
            raise RwsError("El módulo ya existe. Se canceló la recuperación")
        name = f"imt342-{uuid4().hex}.mod"
        remote = f"/fileservice/$home/{name}"
        self._request("GET", remote, 404)
        if not authorized():
            raise RwsError("Recuperación RAPID revocada")
        self._request("PUT", remote, (200, 201), module_source)
        staged = self._request("GET", remote, 200)
        if staged != module_source.decode("ascii"):
            raise RwsError("El archivo transferido no coincide con el módulo de v1")
        if (not authorized() or self.connect() != ControllerStatus("AUTO", "motoron", "stopped")
                or self._loaded_module()):
            raise RwsError("Cambió el controlador antes de cargar el módulo")
        self._check_active_tasks()
        self._request("POST", f"{TASKS}/{self.config.task}?action=loadmod", 204,
                      {"modulepath": f"$HOME/{name}", "replace": "false"})
        if not self._loaded_module():
            raise RwsError("RAPID no confirmó la carga del módulo")
        self._verify_loaded_module(module_source)

    def start_server(self, *, authorized=lambda: False, module_source=None, recover_missing=False,
                     recover_abandoned=False):
        """Explicitly start the idle joint server under persistent RAPID mastership."""
        with self._lock:
            if not authorized() or self._sequence is not None:
                raise RwsError("Preparación RAPID sin autorización vigente u orden sin resolver")
            if recover_missing and module_source is None:
                raise RwsError("Falta el módulo v1 instalado para la recuperación")
            if module_source is not None and (
                self.config.module != MODULE_NAME or sha256(module_source).hexdigest() != MODULE_SHA256
            ):
                raise RwsError("El módulo v1 instalado no coincide con su huella")
            status = self.connect()
            if module_source is not None:
                self.controller_metadata = {}
                self._optional_controller_metadata()
            if status.execution == "running":
                if module_source is not None:
                    module, routine = self._pointer()
                    if module != self.config.module or routine not in (ROUTINE, MOVE_ROUTINE):
                        pointer = f"PP en {module}/{routine}" if module else "PP sin módulo ni rutina"
                        raise RwsError(f"RAPID en marcha · {pointer}. "
                                       "Revisa la ejecución con el operador en FlexPendant",
                                       state="foreign_program")
                    self._check_active_tasks()
                    self._verify_loaded_module(module_source)
                self._ready_sequence()
                return "already_running"
            if status != ControllerStatus("AUTO", "motoron", "stopped"):
                raise RwsError("Preparación RAPID requiere AUTO, motores encendidos y RAPID detenido")
            if module_source is not None:
                self._check_active_tasks()
                loaded = self._loaded_module()
                if loaded:
                    if recover_missing:
                        raise RwsError("El módulo apareció después del diagnóstico. "
                                       "Vuelve a pulsar Conectar y preparar IRC5 para verificarlo")
                    self._verify_loaded_module(module_source)
                else:
                    version = self.controller_metadata.get("robotware_version", "")
                    if not version.startswith("6."):
                        raise RwsError(
                            "Falta el módulo y RobotWare 6 no se confirmó. Cárgalo en FlexPendant")
                    if not recover_missing:
                        raise RwsError(f"Falta {self.config.module} en {self.config.task}. "
                                       "Confirma Recuperar servidor RAPID para cargar la copia v1",
                                       state="module_missing")
            if not recover_missing:
                self._protocol_version()
                sequence = (self._preparation_sequence() if recover_abandoned and module_source is not None
                            else self._closed_sequence())
                if sequence[0] >= MAX_SEQUENCE:
                    raise RwsError("Secuencia RAPID agotada; requiere reinicialización supervisada")
            if not authorized():
                raise RwsError("Autorización de preparación RAPID revocada")
            self._mastership("request")
            operation_error = None
            try:
                if recover_missing:
                    if self.connect() != ControllerStatus("AUTO", "motoron", "stopped"):
                        raise RwsError("Cambió el controlador antes de transferir el módulo")
                    self._recover_missing_module(module_source, authorized)
                elif module_source is not None:
                    self._verify_loaded_module(module_source)
                self._start_stopped_server(authorized, module_verified=module_source is not None,
                                           recover_abandoned=recover_abandoned and module_source is not None)
            except RwsError as error:
                operation_error = error
                raise
            finally:
                try:
                    self._mastership("release")
                except RwsError as error:
                    detail = str(operation_error) if operation_error else "Verifique el estado RAPID"
                    raise RwsError(f"{detail} · liberación de mastership incierta: {error}",
                                   error.status or getattr(operation_error, "status", None)) from None
            self.last_error = ""
            return "started"

    def _start_stopped_server(self, authorized, *, module_verified, recover_abandoned=False):
        task_path = f"{TASKS}/{self.config.task}"
        pointer_path = task_path + "/pcp"
        if self.connect() != ControllerStatus("AUTO", "motoron", "stopped"):
            raise RwsError("Cambió el estado del IRC5 durante la preparación RAPID")
        self._check_active_tasks()
        self._protocol_version()
        read_sequence = self._preparation_sequence if recover_abandoned else self._closed_sequence
        sequence = read_sequence()
        if sequence[0] >= MAX_SEQUENCE:
            raise RwsError("Secuencia RAPID agotada; requiere reinicialización supervisada")
        level = self._status_field(task_path, "execlevel").lower()
        pointer = self._pointer()
        if level == "user":
            if module_verified and pointer[0] != self.config.module:
                location = f"PP en {pointer[0]}/{pointer[1]}" if pointer[0] else "PP sin módulo ni rutina"
                raise RwsError(f"Nivel RAPID de usuario · {location}. "
                               "Revisa esa ejecución en FlexPendant", state="foreign_program")
            if not authorized():
                raise RwsError("Autorización de preparación RAPID revocada")
            self._request("POST", task_path + "?action=abortexeclevel", 204)
            level = self._status_field(task_path, "execlevel").lower()
        if level != "normal":
            raise RwsError("Nivel de ejecución RAPID incompatible; inspeccione la tarea en FlexPendant")
        if not authorized():
            raise RwsError("Autorización de preparación RAPID revocada")
        # Always relocate a stopped PP to the entry, even when the routine
        # matches. A stopped PP inside the loop is not an entry point.
        self._request("POST", pointer_path + "?action=set-pp-routine", 204, {
            "module": self.config.module, "routine": ROUTINE, "userlevel": "false",
        })
        if self._pointer() != (self.config.module, ROUTINE):
            raise RwsError("No se confirmó el puntero de ServeJointOrders después del cambio")
        if (self.connect() != ControllerStatus("AUTO", "motoron", "stopped")
                or read_sequence() != sequence):
            raise RwsError("Cambió el estado RAPID antes del inicio del servidor")
        if not authorized():
            raise RwsError("Autorización de preparación RAPID revocada antes del inicio")
        try:
            self._request("POST", EXECUTION + "?action=start", 204, {
                "regain": "clear", "execmode": "continue", "cycle": "forever",
                "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false",
            })
            if len(sequence) == 3:
                self._wait_abandoned_restart(sequence)
            if self._ready_sequence() != sequence[0]:
                raise RwsError("Cambió la secuencia del servidor al iniciar RAPID")
        except RwsError as error:
            raise RwsError(f"Inicio RAPID sin confirmar; inspeccione el controlador: {error}",
                           error.status) from None

    def diagnostic(self):
        """Read live controller status and, when available, the RAPID protocol."""
        with self._lock:
            status = self.connect()
            details = {"mode": status.mode, "motors": status.motors,
                       "execution": status.execution, "last_error": self.last_error,
                       "pending_sequence": self._sequence}
            try:
                details["pointer"] = self._pointer()
                details["protocol_version"] = self._protocol_version()
                details["order_sequence"] = self._order_sequence()
                details["reply"] = self._reply()
            except RwsError as exc:
                details["rapid_diagnostic"] = str(exc)
            return details

    def _mastership(self, action):
        self._request("POST", f"{MASTER}?action={action}", 204)

    def _release_mastership(self):
        try:
            self._mastership("release")
        except RwsError as exc:
            self.last_error = f"Mastership: {exc}"
            if self._motion_evidence:
                self._motion_evidence["uncertainty_observed"] = True

    def _prepare_order(self):
        return self._ready_sequence()

    def _encode_order(self, sequence, goal, duration):
        axes = ",".join(f"{degrees(axis):.8f}" for axis in goal)
        return {"value": f"[{sequence},[[{axes}],[9E9,9E9,9E9,9E9,9E9,9E9]]]"}, {}

    def execute(self, goal_rad, *, authorized=lambda: False, expected_start=None,
                expected_prior_sequence=None, duration=None, origin_tolerance_rad=None):
        with self._lock:
            if self._sequence is not None:
                raise RwsError("Hay una orden RWS sin resolver")
            self._result = Result.FAILED
            self._motion_evidence = {}
            self.final_feedback = self.final_error_rad = None
            requested_at = self.clock()
            goal = validate_target(goal_rad)
            self.last_error = ""
            if not authorized():
                raise RwsError("Habilitación RWS ausente o revocada")
            self._mastership("request")
            try:
                prior_sequence = self._prepare_order()
                if expected_prior_sequence is not None and prior_sequence != expected_prior_sequence:
                    raise RwsError("Cambió la secuencia RAPID entre tramos del lote")
                sequence = prior_sequence + 1
                observed = self.read_state()
                origin_tolerance = self.config.tolerance_rad if origin_tolerance_rad is None else origin_tolerance_rad
                if not isfinite(origin_tolerance) or not 0 < origin_tolerance <= self.config.tolerance_rad:
                    raise ValueError("Tolerancia del origen RWS inválida")
                if expected_start is not None and any(
                    abs(measured - expected) > origin_tolerance
                    for measured, expected in zip(observed.positions, validate_target(expected_start))
                ):
                    raise RwsError("La postura RWS cambió desde la planificación")
                if not authorized():
                    raise RwsError("Habilitación RWS revocada antes del envío")
                data, order_evidence = self._encode_order(sequence, goal, duration)
                self._sequence = sequence
                self._goal = goal
                sent_at = self._sent_at = self.clock()
                self._motion_evidence = {
                    "sequence": sequence, "commanded_speeddata": "v10",
                    "protocol_version": 1, "commanded_zone": "fine", "commanded_tool": "tool0",
                    "target_rad": goal, "start_feedback_rad": observed.positions,
                    "controller_timing_s": None, "submit_confirmed": False,
                    "uncertainty_observed": False,
                    "host_submit_monotonic_s": sent_at,
                    "preflight_s": self._sent_at - requested_at,
                    **order_evidence,
                }
                self._stop_requested = False
                self.final_feedback = None
                self.final_error_rad = None
                self._result = Result.UNKNOWN
                try:
                    self._request("POST", self._symbol("pcOrder") + "?action=set", 204, data)
                except RwsError as exc:
                    self.last_error = str(exc)
                    if exc.status in (400, 401, 403, 404, 405, 409, 415, 422):
                        self._finish(Result.FAILED)
                        raise
                    self._result = self._observe_uncertain_motion(str(exc))
                else:
                    self._result = Result.TRANSMITTED
                    self._motion_evidence["submit_confirmed"] = True
                finally:
                    self._motion_evidence["order_http_s"] = self.clock() - sent_at
                return self._result
            finally:
                self._release_mastership()

    def _finish(self, result):
        self._motion_evidence["submit_to_resolution_s"] = self.clock() - self._sent_at
        if self.final_feedback is not None:
            self._motion_evidence["final_feedback_rad"] = self.final_feedback.positions
        self._sequence = None
        self._goal = None
        self._sent_at = None
        self._result = result
        return result

    def result(self):
        with self._lock:
            if self._sequence is None:
                return self._result
            try:
                execution = self._status_field(EXECUTION, "ctrlexecstate").lower()
                reply_sequence, phase = self._reply()
                if reply_sequence == self._sequence and phase != 0:
                    elapsed = self.clock() - self._sent_at
                    self._motion_evidence["reply"] = (reply_sequence, phase)
                    self._motion_evidence.setdefault("submit_to_first_reply_s", elapsed)
                    if phase == 2:
                        self._motion_evidence.setdefault("submit_to_observed_phase_2_s", elapsed)
                    if phase in (3, 4, 5):
                        self._motion_evidence.setdefault("submit_to_terminal_reply_s", elapsed)
                        first_moving = self._motion_evidence.get("submit_to_observed_phase_2_s")
                        if first_moving is not None:
                            self._motion_evidence["observed_phase_2_to_terminal_s"] = (
                                self._motion_evidence["submit_to_terminal_reply_s"] - first_moving
                            )
                if reply_sequence == self._sequence and phase in (3, 4, 5):
                    self.final_feedback = self.read_state()
                    if phase == 3:
                        self.final_error_rad = max(
                            abs(measured - target)
                            for measured, target in zip(self.final_feedback.positions, self._goal)
                        )
                        if self.final_error_rad <= self.config.tolerance_rad:
                            if not self.last_error.startswith("Mastership:"):
                                self.last_error = ""
                            return self._finish(Result.VERIFIED)
                        self.last_error = "RAPID finalizó con error de postura medido"
                        return self._finish(Result.FAILED)
                    if phase == 5:
                        return self._finish(Result.CANCELLED)
                    self.last_error = "RAPID reportó error de ejecución"
                    return self._finish(Result.FAILED)
                if self._stop_requested and execution == "stopped":
                    self.final_feedback = self.read_state()
                    self.last_error = "RAPID detenido; reubicar PP al inicio antes de reanudar"
                    return self._finish(Result.CANCELLED)
                if execution != "running":
                    return self._observe_uncertain_motion(f"RAPID {execution}; orden sin resolución")
                if reply_sequence == self._sequence and phase in (1, 2):
                    self._result = Result.ACCEPTED if phase == 1 else Result.EXECUTING
                elif self.clock() - self._sent_at >= self.config.ack_timeout:
                    return self._observe_uncertain_motion(
                        "Venció el plazo de acuse de RAPID; orden sin resolución"
                    )
                return self._result
            except RwsError as exc:
                return self._observe_uncertain_motion(str(exc))

    def cancel(self):
        """Request RAPID stop. Resolve only on later controller observations."""
        with self._lock:
            current = self.result()
            if self._sequence is None:
                return current
            try:
                self._mastership("request")
                try:
                    module, routine = self._pointer()
                    if module != self.config.module or routine not in (ROUTINE, MOVE_ROUTINE):
                        raise RwsError("PP fuera del servidor articular; no se solicitó parada RAPID")
                    if self._order_sequence() > self._sequence:
                        raise RwsError("Hay una secuencia RAPID posterior; no se solicitó parada")
                    self._stop_requested = True
                    self._request(
                        "POST", EXECUTION + "?action=stop", 204, {"stopmode": "stop"},
                    )
                finally:
                    self._release_mastership()
            except RwsError as exc:
                self.last_error = str(exc)
                return Result.UNKNOWN
            return Result.TRANSMITTED

    def close(self):
        with self._lock:
            close = getattr(self.http, "close", None)
            if close is not None:
                close()
