"""Offline ABB instruction drafts and an in-process acknowledgement model."""

from dataclasses import asdict, dataclass
import hashlib
import json
from math import degrees, isfinite
from uuid import uuid4

from imt342_irb120_planning import (
    DEFAULT_ACCELERATION, DEFAULT_VELOCITY, JOINT_LIMITS, JOINT_NAMES, MODEL_VELOCITIES,
    plan as plan_lspb,
)

SCHEMA = "imt342.rapid-trajectory-draft.v1"
MAX_INTERVALS = 64
NO_EXTERNAL_AXES = (9e9,) * 6


def six(values, label, *, positive=False):
    try:
        vector = tuple(values)
    except TypeError as error:
        raise ValueError(f"{label}: se requieren seis números") from error
    if len(vector) != 6 or any(type(value) not in (int, float) for value in vector):
        raise ValueError(f"{label}: se requieren seis números")
    try:
        if any(not isfinite(value) or (positive and value <= 0) for value in vector):
            raise ValueError(f"{label}: valores no válidos")
    except OverflowError as error:
        raise ValueError(f"{label}: valores fuera del rango numérico") from error
    return tuple(map(float, vector))


def rapid_number(value):
    return format(value, ".17g").upper()


def rapid_target(positions_deg):
    arm = ",".join(map(rapid_number, positions_deg))
    external = ",".join("9E9" for _ in NO_EXTERNAL_AXES)
    return f"[[{arm}],[{external}]]"


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: int
    target_rad: tuple[float, ...]
    target_deg: tuple[float, ...]
    model_time_s: float
    time_request_s: float
    model_stop_min_s: float
    short_for_model_stop: bool
    stationary: bool

    @property
    def rapid_preview(self):
        if self.stationary:
            return None
        time = rapid_number(self.time_request_s)
        return f"MoveAbsJ {rapid_target(self.target_deg)},v10\\T:={time},fine,tool0;"


@dataclass(frozen=True)
class RapidDraft:
    profile: str
    start_rad: tuple[float, ...]
    goal_rad: tuple[float, ...]
    duration_s: float
    max_velocity_rad_s: tuple[float, ...]
    max_acceleration_rad_s2: tuple[float, ...]
    max_jerk_rad_s3: tuple[float, ...] | None
    waypoints: tuple[Waypoint, ...]

    def record(self):
        inputs = asdict(self)
        signature = json.dumps(inputs, sort_keys=True, allow_nan=False, separators=(",", ":"))
        return {
            "schema": SCHEMA,
            "plan_id": hashlib.sha256(signature.encode()).hexdigest(),
            "scope": "offline_draft",
            "hardware_enabled": False,
            "controller_tracking_validated": False,
            "collision_validated": False,
            "joint_order": JOINT_NAMES,
            "reference_units": {"position": "rad", "time": "s"},
            "rapid_units": {"position": "deg", "time": "s"},
            "start_deg": tuple(map(degrees, self.start_rad)),
            "goal_deg": tuple(map(degrees, self.goal_rad)),
            "rapid": {"instruction": "MoveAbsJ", "zone": "fine", "tool": "tool0",
                      "speeddata_argument": "v10", "external_axes": NO_EXTERNAL_AXES,
                      "timing": "requested_only"},
            **inputs,
            "waypoints": [{**asdict(item), "rapid_preview": item.rapid_preview} for item in self.waypoints],
        }


def build_plan(start, goal, *, profile="lspb", duration=None, max_velocity=DEFAULT_VELOCITY,
               max_acceleration=DEFAULT_ACCELERATION, max_jerk=None, intervals=1):
    """Map a validated ROS reference into an inert stop-at-target RAPID draft."""
    if type(intervals) is not int or not 1 <= intervals <= MAX_INTERVALS:
        raise ValueError(f"Se requieren entre 1 y {MAX_INTERVALS} intervalos")
    if duration is not None and (
        type(duration) not in (int, float) or duration <= 0 or not isfinite(duration)
    ):
        raise ValueError("La duración debe ser finita y positiva")
    start = six(start, "Inicio")
    goal = six(goal, "Destino")
    max_velocity = tuple(min(limit, model) for limit, model in zip(
        six(max_velocity, "Límite de velocidad", positive=True), MODEL_VELOCITIES
    ))
    max_acceleration = six(max_acceleration, "Límite de aceleración", positive=True)
    options = dict(max_velocity=max_velocity, max_acceleration=max_acceleration)
    if profile == "lspb":
        if max_jerk is not None:
            raise ValueError("LSPB ideal tiene saltos de aceleración y no satisface una cota finita de jerk")
        planner = plan_lspb
    elif profile == "quintic":
        from imt342_irb120_planning.smooth import plan_quintic
        options["max_jerk"] = max_jerk = six(max_jerk, "Límite de jerk", positive=True)
        planner = plan_quintic
    else:
        raise ValueError("El perfil debe ser lspb o quintic")

    reference = planner(start, goal, duration=duration, **options)
    steps = []
    previous_time, previous_q = 0.0, reference.start
    for index in range(1, intervals + 1):
        time = reference.duration * (index / intervals)
        q = reference.sample(time).positions
        requested = time - previous_time
        if requested <= 0 or not isfinite(requested):
            raise ValueError("La discretización no produjo tiempos estrictamente crecientes")
        stationary = q == previous_q
        minimum = 0.0 if stationary else planner(previous_q, q, **options).duration
        steps.append(Waypoint(
            index, q, tuple(map(degrees, q)), time, requested, minimum,
            requested < minimum and not stationary, stationary,
        ))
        previous_time, previous_q = time, q
    return RapidDraft(profile, reference.start, reference.goal, reference.duration,
                      max_velocity, max_acceleration, max_jerk, tuple(steps))


def load_plan(record):
    """Rebuild an exported draft; derived targets, timing, and ID must match."""
    if not isinstance(record, dict) or record.get("schema") != SCHEMA:
        raise ValueError("El registro no corresponde al esquema de borrador RAPID")
    try:
        draft = build_plan(
            record["start_rad"], record["goal_rad"], profile=record["profile"],
            duration=record["duration_s"], max_velocity=record["max_velocity_rad_s"],
            max_acceleration=record["max_acceleration_rad_s2"],
            max_jerk=record["max_jerk_rad_s3"], intervals=len(record["waypoints"]),
        )
        artifact = {key: value for key, value in record.items() if key != "simulation"}
        expected = json.loads(json.dumps(draft.record(), allow_nan=False))
        normalized = json.loads(json.dumps(artifact, allow_nan=False))
    except (KeyError, TypeError, OverflowError) as error:
        raise ValueError("El registro contiene campos inválidos") from error
    if normalized != expected:
        raise ValueError("El ID o los datos derivados del borrador no coinciden")
    return draft


@dataclass(frozen=True)
class MockOrder:
    run_id: str
    plan_id: str
    waypoint: Waypoint


class OfflineRapidRun:
    """Single-use trial with no connection, controller or transport."""

    def __init__(self, draft, *, tolerance_deg=0.05):
        if type(tolerance_deg) not in (int, float) or not isfinite(tolerance_deg) or tolerance_deg <= 0:
            raise ValueError("La tolerancia del mock debe ser finita y positiva")
        self.run_id = uuid4().hex
        self.plan_id = draft.record()["plan_id"]
        self.draft = draft
        self.tolerance_deg = tolerance_deg
        self.state = "loaded"
        self.completed = 0
        self.pending = None
        self.events = []

    def start(self):
        if self.state != "loaded":
            return False
        if any(item.short_for_model_stop for item in self.draft.waypoints):
            self.state = "rejected"
            return False
        self.state = "running"
        return True

    def dispatch_next(self):
        if self.state != "running" or self.pending is not None:
            return None
        self.pending = MockOrder(self.run_id, self.plan_id, self.draft.waypoints[self.completed])
        return self.pending

    def _matches(self, run_id, plan_id, waypoint_id):
        return self.pending is not None and type(waypoint_id) is int and (
            run_id, plan_id, waypoint_id,
        ) == (self.run_id, self.plan_id, self.pending.waypoint.waypoint_id)

    def acknowledge(self, run_id, plan_id, waypoint_id, *, outcome, positions_deg=None):
        if self.state not in ("running", "cancel_requested") or not self._matches(run_id, plan_id, waypoint_id):
            return False
        if outcome not in ("completed", "cancelled", "unknown"):
            raise ValueError("Resultado mock inválido")
        if outcome == "completed":
            try:
                observed = six(positions_deg, "Postura final")
                target = self.pending.waypoint.target_deg
                limits = [(degrees(lo), degrees(hi)) for lo, hi in JOINT_LIMITS]
                if not all(lo <= q <= hi and abs(q - expected) <= self.tolerance_deg
                           for q, expected, (lo, hi) in zip(observed, target, limits)):
                    outcome = "unknown"
            except ValueError:
                outcome = "unknown"
        self.events.append({"waypoint_id": waypoint_id, "outcome": outcome})
        self.pending = None
        if outcome == "completed":
            self.completed += 1
            if self.state == "cancel_requested":
                self.state = "cancelled"
            elif self.completed == len(self.draft.waypoints):
                self.state = "completed"
        else:
            self.state = outcome
        return True

    def cancel(self):
        if self.state not in ("running", "loaded"):
            return False
        self.state = "cancelled" if self.pending is None else "cancel_requested"
        return True

    def timeout(self):
        if self.pending is None:
            return False
        return self.acknowledge(self.run_id, self.plan_id, self.pending.waypoint.waypoint_id,
                                outcome="unknown")

    def summary(self):
        return {"scope": "mock", "hardware_enabled": False, "run_id": self.run_id,
                "plan_id": self.plan_id, "state": self.state, "completed_waypoints": self.completed,
                "tolerance_deg": self.tolerance_deg, "events": list(self.events)}


def simulate_completion(draft):
    run = OfflineRapidRun(draft)
    run.start()
    while (order := run.dispatch_next()) is not None:
        run.acknowledge(order.run_id, order.plan_id, order.waypoint.waypoint_id,
                        outcome="completed", positions_deg=order.waypoint.target_deg)
    return run.summary()
