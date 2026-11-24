"""RWS endpoint trials for the separate RobotWare 6.12 trajectory service."""

from collections import deque
from dataclasses import replace
from hashlib import sha256
from math import degrees, isfinite, radians
from re import fullmatch
from time import monotonic

from .rws_backend import EXECUTION, MASTER, MAX_SEQUENCE, RAPID_NUMBER, RwsError, RwsRapidBackend
from .state import Result, validate_target
from .rws_lspb_revision import CANDIDATE_SOURCE, CANDIDATE_SHA256
from .rws_continuous_revision import CONTINUOUS_SOURCE, CONTINUOUS_SHA256

ADAPTIVE_REVISIONS = {"lspb_candidate", "continuous_candidate"}
from .lspb_calibration import AccelerationCalibration, fit_segment

MODULE_NAME = "Imt342TrajectoryRapid"
MODULE_SHA256 = "be70f1682fdb9548ff174a6899080c480858d906f81522379d9bfe468d9b1410"
ROUTINE = "ServeTrajectoryOrders"
MOVE_ROUTINE = "RunTrajectoryOrder"
PROTOCOL_VERSION = 2
MIN_DURATION = 0.1
MAX_DURATION = 30.0
RAPID_TOLERANCE_RAD = radians(0.30)
IDENTITY_TTL = 30.0  # Forced identity checks at preparation and dispatch remain unchanged.
NUMBER = rf"\s*[+-]?{RAPID_NUMBER}\s*"
AXES = rf"\[{NUMBER}(?:,{NUMBER}){{5}}\]"
JOINT_TARGET = rf"\[\s*{AXES}\s*,\s*{AXES}\s*\]"
ORDER_PAYLOAD = rf"\s*,\s*{JOINT_TARGET}\s*,\s*{JOINT_TARGET}\s*,{NUMBER}"


def _joint_target(q):
    axes = ",".join(f"{degrees(axis):.8f}" for axis in q)
    return f"[[{axes}],[9E9,9E9,9E9,9E9,9E9,9E9]]"


class RwsTrajectoryBackend(RwsRapidBackend):
    """Send one timed endpoint and verify its RAPID reply and measured arrival."""

    def __init__(self, config, module_source, http=None, clock=monotonic, *, allow_candidate=False):
        if config.module != MODULE_NAME or config.task != "T_ROB1" or config.mechunit != "ROB_1":
            raise ValueError("Trayectorias requiere Imt342TrajectoryRapid en T_ROB1/ROB_1")
        if not isinstance(module_source, bytes) or sha256(module_source).hexdigest() != MODULE_SHA256:
            raise ValueError("La fuente local del módulo experimental no coincide con su huella")
        super().__init__(replace(config, tolerance_rad=min(config.tolerance_rad, RAPID_TOLERANCE_RAD)),
                         http=http, clock=clock)
        self.module_source = module_source
        self._identity_expires = 0.0
        self._arrival_deadline = None
        self.ready_diagnostic = {}
        self.module_revision = None
        self.allow_candidate = allow_candidate
        self._shape_samples = deque(maxlen=512)
        self._shape_key = None
        self._calibration = AccelerationCalibration()

    def reset_shape_calibration(self):
        self._calibration = AccelerationCalibration()

    def _request(self, method, path, expected, body=None):
        writes = {
            f"{MASTER}?action=request", f"{MASTER}?action=release",
            f"{EXECUTION}?action=stop", self._symbol("trjOrder") + "?action=set",
        }
        if self.module_revision in ADAPTIVE_REVISIONS:
            writes.add(self._symbol("trjTune") + "?action=set")
        if self.module_revision == "continuous_candidate":
            writes.add(self._symbol("trjLinear") + "?action=set")
        if method != "GET" and (method != "POST" or path not in writes):
            raise RwsError("Operación fuera del protocolo de trayectorias")
        return super()._request(method, path, expected, body)

    def read_state(self):
        feedback = replace(super().read_state(), source="rws_trajectory")
        if (self._sequence is not None and self._shape_key is not None
                and self._motion_evidence.get("submit_confirmed")
                and (not self._shape_samples or feedback.received_at > self._shape_samples[-1][0])):
            self._shape_samples.append((feedback.received_at, feedback.positions))
        return feedback

    def _protocol_version(self):
        version = self._get(self._symbol("trjVersion"), "value")["value"]
        if version != str(PROTOCOL_VERSION) and (type(version) is not int or version != PROTOCOL_VERSION):
            raise RwsError("Protocolo RAPID de trayectorias incompatible: se requiere v2")
        return PROTOCOL_VERSION

    def _reply(self):
        return self._record("trjReply", r"\[\s*([0-9]+)\s*,\s*([0-5])\s*\]")

    def _order_sequence(self):
        return self._record("trjOrder", rf"\[\s*([0-9]+){ORDER_PAYLOAD}\]")[0]

    @staticmethod
    def _without_persistent_values(tokens):
        normalized = tokens.copy()
        declarations = {
            "trjorder": ("imttrjorder", rf"\[{NUMBER}{ORDER_PAYLOAD}\]"),
            "trjreply": ("imttrjreply", rf"\[{NUMBER},{NUMBER}\]"),
            "trjtune": ("imttrjtune", rf"\[{NUMBER},{NUMBER}\]"),
            "trjlinear": ("imttrjlinear", rf"\[{NUMBER},{NUMBER}\]"),
            "trjtiming": ("imttrjtiming", rf"\[{NUMBER},{NUMBER},{NUMBER},{NUMBER},{NUMBER}\]"),
        }
        for name, (record_type, shape) in declarations.items():
            declaration = ["pers", record_type, name, ":="]
            for index in range(len(normalized) - len(declaration)):
                start = index + len(declaration)
                if normalized[index:start] != declaration:
                    continue
                try:
                    end = normalized.index(";", start)
                except ValueError:
                    break
                if fullmatch(shape, "".join(normalized[start:end])):
                    normalized[start:end] = ["[persistent value]"]
                break
        return normalized

    def _verify_identity(self, *, force=False):
        if not force and self.clock() < self._identity_expires:
            return
        self._identity_expires = 0.0
        self.controller_metadata.clear()
        self._optional_controller_metadata()
        version = self.controller_metadata.get("robotware_version", "")
        if not fullmatch(r"6\.12(?:\.[0-9]+){0,3}", version):
            raise RwsError(
                f"Trayectorias requiere RobotWare 6.12 · versión informada: {version or 'sin confirmar'}",
                state="robotware_incompatible",
            )
        if not self._loaded_module():
            raise RwsError("Falta Imt342TrajectoryRapid en T_ROB1", state="module_missing")
        previous_revision = self.module_revision
        # Prefer the already verified revision to avoid an extra read of the
        # resident source on every periodic verification. Recovery still uses
        # the original module_source and neither file is accepted by name alone.
        revisions = [("original", self.module_source)]
        if self.allow_candidate:
            revisions.extend((("lspb_candidate", CANDIDATE_SOURCE), ("continuous_candidate", CONTINUOUS_SOURCE)))
        revisions.sort(key=lambda item: item[0] != previous_revision)
        self.module_revision = None
        for revision, source in revisions:
            try:
                self._verify_loaded_module(source)
            except RwsError as error:
                if error.state != "module_modified":
                    self._calibration = AccelerationCalibration()
                    raise
            else:
                self.module_revision = revision
                break
        if self.module_revision is None:
            self._calibration = AccelerationCalibration()
            raise RwsError("La fuente de trayectorias en T_ROB1 no coincide con ninguna revision v2 fijada",
                           state="module_modified")
        if previous_revision != self.module_revision:
            self._calibration = AccelerationCalibration()
        self.controller_metadata["module_sha256"] = {"original": MODULE_SHA256,
            "lspb_candidate": CANDIDATE_SHA256, "continuous_candidate": CONTINUOUS_SHA256}[self.module_revision]
        self.controller_metadata["module_revision"] = self.module_revision
        self._identity_expires = self.clock() + IDENTITY_TTL

    def _owns_pointer(self, *, idle=False):
        module, routine = self._pointer()
        routines = {ROUTINE.lower()} if idle else {ROUTINE.lower(), MOVE_ROUTINE.lower()}
        return module == MODULE_NAME and routine.lower() in routines

    def _ready_sequence(self):
        status = self.connect()
        if (status.mode, status.motors, status.execution) != ("AUTO", "motoron", "running"):
            raise RwsError(f"Trayectorias requiere AUTO, motores encendidos y RAPID en marcha · "
                           f"{status.mode}, {status.motors}, {status.execution}")
        module, routine = self._pointer()
        if (module, routine.lower()) != (MODULE_NAME, ROUTINE.lower()):
            if ((module, routine.lower()) == (MODULE_NAME, MOVE_ROUTINE.lower())
                    and self._sequence is None and self._result in {Result.VERIFIED, Result.FAILED}
                    and self._motion_evidence.get("reply") == self._closed_sequence()
                    and self._motion_evidence["reply"][1] in (3, 4)):
                raise RwsError("Destino resuelto · esperando retorno RAPID a ServeTrajectoryOrders",
                               state="server_returning")
            raise RwsError("Selecciona e inicia ServeTrajectoryOrders en T_ROB1 antes de autorizar",
                           state="foreign_program")
        self._check_active_tasks()
        self._protocol_version()
        sequence, phase = self._closed_sequence()
        if sequence >= MAX_SEQUENCE:
            raise RwsError("Secuencia RAPID agotada; requiere reinicialización supervisada")
        self.ready_diagnostic = {
            "mode": status.mode, "motors": status.motors, "execution": status.execution,
            "pointer": (module, ROUTINE), "protocol_version": PROTOCOL_VERSION,
            "module_revision": self.module_revision,
            "order_sequence": sequence, "reply": (sequence, phase), "pending_sequence": None,
        }
        return sequence

    def prepare(self):
        """Confirm module, version, task, pointer and protocol using only reads."""
        with self._lock:
            if self._sequence is not None:
                raise RwsError("Hay una consigna de trayectorias sin resolver")
            try:
                self._verify_identity()
                return self._ready_sequence()
            except RwsError as error:
                if error.state != "server_returning":
                    self._identity_expires = 0.0
                raise

    def start_server(self, *, authorized=lambda: False, module_source=None, recover_missing=False):
        if recover_missing or module_source is not None:
            raise RwsError("La preparación de trayectorias en RViz es de solo lectura")
        self.prepare()
        return "already_running"

    def diagnostic(self):
        with self._lock:
            details = super().diagnostic()
            for field in ("robotware_version", "module_sha256", "module_revision", "active_tasks"):
                if field in self.controller_metadata:
                    details[field] = self.controller_metadata[field]
            return details

    def execute(self, goal_rad, *, duration, authorized=lambda: False, expected_start,
                expected_prior_sequence=None, origin_tolerance_rad=None, reference_blend=None,
                linear_cartesian=False):
        with self._lock:
            if self._sequence is not None:
                raise RwsError("Hay una consigna de trayectorias sin resolver")
            self._result = Result.FAILED
            self._motion_evidence = {}
            self.last_error = ""
            self.final_feedback = self.final_error_rad = None
            self._shape_samples.clear()
            self._shape_key = None
            if type(duration) not in (int, float) or not MIN_DURATION <= duration <= MAX_DURATION:
                raise ValueError(f"La duración RAPID debe estar entre {MIN_DURATION:g} y {MAX_DURATION:g} s")
            goal, start = validate_target(goal_rad), validate_target(expected_start)
            if goal == start:
                raise ValueError("La consigna requiere un destino distinto del origen")
            if not authorized():
                raise RwsError("Habilitación de trayectorias ausente o revocada")
            requested_at = self.clock()
            prior = self.prepare()
            if expected_prior_sequence is not None and prior != expected_prior_sequence:
                raise RwsError("Cambió la secuencia RAPID entre tramos del lote")
            if not authorized():
                raise RwsError("Habilitación de trayectorias revocada antes de mastership")
            self._mastership("request")
            try:
                self._verify_identity(force=True)
                if linear_cartesian and self.module_revision != "continuous_candidate":
                    raise RwsError("MoveL continuo requiere la revisión v2 candidata validada")
                if self._ready_sequence() != prior:
                    raise RwsError("Cambió la secuencia RAPID durante la preparación")
                observed = self.read_state()
                origin_tolerance = self.config.tolerance_rad if origin_tolerance_rad is None else origin_tolerance_rad
                if not isfinite(origin_tolerance) or not 0 < origin_tolerance <= self.config.tolerance_rad:
                    raise ValueError("Tolerancia del origen RWS inválida")
                if any(abs(q - planned) > origin_tolerance for q, planned in zip(observed.positions, start)):
                    raise RwsError("La postura RWS cambió desde el plan de trayectorias")
                validate_target(observed.positions)
                if not authorized():
                    raise RwsError("Habilitación de trayectorias revocada antes de la consigna")

                sequence = prior + 1
                # LSPB reference shape comes from the actual dispatched segment.
                # This is not a third backend, and the original v2 never gets extra writes.
                if (not linear_cartesian and type(reference_blend) in (int,float) and 0 < reference_blend <= .5):
                    self._shape_key = (start, goal, duration, reference_blend)
                ratio, reason = 1., "revision original o perfil no LSPB"
                if self.module_revision in ADAPTIVE_REVISIONS:
                    if self._shape_key is not None:
                        ratio, reason = self._calibration.ratio(self._shape_key, self.clock())
                    # Write non-motion metadata first, bound to the same sequence.
                    # A failed or uncertain tuning write cannot dispatch an order.
                    self._request("POST", self._symbol("trjTune") + "?action=set", 204,
                                  {"value": f"[{sequence},{ratio:.8f}]"})
                    if not authorized():
                        raise RwsError("Habilitación revocada antes de la consigna")
                if self.module_revision == "continuous_candidate":
                    self._request("POST", self._symbol("trjLinear") + "?action=set", 204,
                                  {"value": f"[{sequence},{int(linear_cartesian)}]"})
                    if not authorized():
                        raise RwsError("Habilitación revocada antes de la consigna")
                payload = f"[{sequence},{_joint_target(start)},{_joint_target(goal)},{duration:.8f}]"
                self._sequence, self._goal = sequence, goal
                sent_at = self._sent_at = self.clock()
                self._arrival_deadline = sent_at + duration + self.config.ack_timeout
                self._stop_requested = False
                self._result = Result.UNKNOWN
                self._motion_evidence = {
                    "sequence": sequence, "protocol_version": PROTOCOL_VERSION,
                    "speeddata_argument": "v10", "commanded_zone": "fine", "commanded_tool": "tool0",
                    "motion_instruction": "MoveL" if linear_cartesian else "MoveAbsJ",
                    "requested_duration_s": duration, "timing": "requested_only",
                    "target_rad": goal, "planned_start_rad": start, "start_feedback_rad": observed.positions,
                    "controller_timing_s": None, "submit_confirmed": False, "uncertainty_observed": False,
                    "host_submit_monotonic_s": sent_at,
                    "preflight_s": sent_at - requested_at,
                    "module_revision": self.module_revision,
                    "lspb_acceleration_ratio_requested": ratio,
                    "lspb_acceleration_reason": reason,
                }
                try:
                    self._request("POST", self._symbol("trjOrder") + "?action=set", 204, {"value": payload})
                except RwsError as error:
                    self.last_error = str(error)
                    if error.status in (400, 401, 403, 404, 405, 409, 415, 422):
                        self._finish(Result.FAILED)
                        raise
                    self._result = self._observe_uncertain_motion(str(error))
                else:
                    self._result = Result.TRANSMITTED
                    self._motion_evidence["submit_confirmed"] = True
                finally:
                    self._motion_evidence["order_http_s"] = self.clock() - sent_at
                return self._result
            finally:
                self._release_mastership()

    def _candidate_timing(self, sequence):
        """One optional read after terminal feedback, never changes a result."""
        raw = self._get(self._symbol("trjTiming"), "value")["value"]
        match = fullmatch(rf"\[\s*([0-9]+)\s*,({NUMBER}),({NUMBER}),({NUMBER}),({NUMBER})\]",
                          str(raw).strip())
        if not match or int(match[1]) != sequence:
            raise ValueError("RAPID trjTiming sin correlacion de secuencia")
        move, wait, base, applied = map(float, match.groups()[1:])
        if (not all(isfinite(v) for v in (move,wait,base,applied)) or move <= 0 or wait < 0
                or not 0 < applied <= base <= 100):
            raise ValueError("RAPID trjTiming fuera de rango")
        return dict(controller_timing_s=move, controller_wait_inpos_s=wait,
                    controller_base_acc_percent=base, controller_applied_acc_percent=applied)

    def _finish(self, result):
        # Base class still owns final-position, sequence and terminal semantics.
        sent = self._sent_at
        outcome = super()._finish(result)
        evidence = self._motion_evidence
        key = self._shape_key
        self._shape_key = None
        if (result is Result.VERIFIED and self.module_revision in ADAPTIVE_REVISIONS
                and evidence.get("sequence") is not None):
            try:
                evidence.update(self._candidate_timing(evidence["sequence"]))
            except (RwsError, ValueError, KeyError, TypeError, OverflowError) as exc:
                evidence["controller_timing_unavailable"] = type(exc).__name__
        if (result is Result.VERIFIED and key is not None and sent is not None
                and evidence.get("submit_confirmed") and not evidence.get("uncertainty_observed")
                and self.final_feedback is not None):
            delay = evidence.get("submit_to_observed_phase_2_s")
            fit = None
            if delay is not None:
                try:
                    fit = fit_segment(*key, tuple(self._shape_samples), sent, sent+delay,
                                      self.final_feedback.received_at)
                except (ValueError, ArithmeticError, TypeError):
                    # Diagnostics must not change an already verified result.
                    pass
            evidence["lspb_equivalent_fit"] = fit
            ratio = evidence.get("lspb_acceleration_ratio_requested", 1.)
            if self.module_revision in ADAPTIVE_REVISIONS:
                controller_duration = evidence.get("controller_timing_s")
                # Do not learn from a stale/unknown controller timing or a
                # movement whose actual instruction time differs materially.
                base = evidence.get("controller_base_acc_percent")
                applied = evidence.get("controller_applied_acc_percent")
                observed_setting = (base is not None and applied is not None and
                                    (abs(applied-base) < .01 if ratio == 1. else applied < base-.01))
                reliable = (fit is not None and controller_duration is not None and observed_setting
                            and abs(controller_duration/key[2]-1) <= .10
                            and abs(controller_duration-fit["fitted_duration_s"])/key[2] <= .10)
                self._calibration.record(key, fit if reliable else None, self.clock(), ratio=ratio)
        if result is not Result.VERIFIED or evidence.get("uncertainty_observed"):
            self._calibration = AccelerationCalibration()
        self._shape_samples.clear()
        return outcome

    def result(self):
        with self._lock:
            result = super().result()
            if self._sequence is not None:
                if self.clock() >= self._arrival_deadline:
                    return self._observe_uncertain_motion(
                        "Venció el plazo de confirmación final en RAPID; consigna sin resolver"
                    )
                if self._motion_evidence.get("uncertainty_observed"):
                    return Result.UNKNOWN
            return result

    def cancel(self):
        """Stop only while the matching experimental order still has ownership."""
        with self._lock:
            current = self.result()
            if self._sequence is None or self._stop_requested:
                return current
            try:
                self._mastership("request")
                try:
                    if not self._owns_pointer() or self._order_sequence() != self._sequence:
                        raise RwsError("El PP o la secuencia ya no corresponde a la consigna de trayectorias")
                    self._check_active_tasks()
                    self._stop_requested = True
                    self._request("POST", EXECUTION + "?action=stop", 204, {"stopmode": "stop"})
                finally:
                    self._release_mastership()
            except RwsError as error:
                return self._observe_uncertain_motion(str(error))
            return Result.TRANSMITTED
