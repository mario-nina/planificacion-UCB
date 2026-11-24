"""Plan and inspect catalog tasks independently of the transport."""

from dataclasses import dataclass
from math import ceil, radians
from re import fullmatch

from . import Sequence, _joint_vector, plan, plan_sequence, planner_for_profile

WAYPOINTS = ("A", "B", "C", "HOME")
JOINT_TEST_ORDER = ("OBJETIVO", "INICIO")
TASK_CYCLE_BOUNDS = {
    "legacy": (1, 8), "joint_test": (1, 1), "cycle": (1, 1), "repeat": (2, 8), "custom": (1, 8),
}
MAX_TASK_WAYPOINTS = 32


@dataclass(frozen=True)
class TaskPlan:
    trajectory: Sequence
    waypoints: tuple
    cycles: int
    leg_checks: tuple
    kind: str = "legacy"
    order: tuple[str, ...] = WAYPOINTS
    required_start: tuple[float, ...] | None = None


class TaskInputError(ValueError):
    def __init__(self, message, waypoint=None, candidates=()):
        super().__init__(message)
        self.waypoint = waypoint
        self.candidates = candidates


def preprogrammed_demo(start, task):
    """Resolve four fixed targets relative to the measured or simulated HOME."""
    if (not isinstance(task, dict) or task.get("kind") != "demo_cycle_live"
            or type(task.get("cycles")) is not int or task["cycles"] != 1):
        raise TaskInputError("La demostración preprogramada realiza un solo ciclo")
    if task.get("offline_example") or "waypoints" in task or "order" in task:
        raise TaskInputError("La demostración resuelve sus puntos desde la postura vigente")
    try:
        start = _joint_vector(start, "Inicio")
    except ValueError as exc:
        raise TaskInputError("Se requiere una postura de partida válida") from exc
    delta = radians(3.0)
    a = (start[0] + delta, *start[1:])
    b = (a[0], start[1] + delta, *start[2:])
    c = (start[0], b[1], *start[2:])
    return dict(kind="cycle", cycles=1, profile=task.get("profile"), waypoints={
        name: dict(q_rad=list(goal))
        for name, goal in zip(WAYPOINTS, (a, b, c, start))
    })


def plan_task(points, start, task, velocity, *, segment_planner=plan):
    if not isinstance(task, dict):
        raise TaskInputError("Se requiere un objeto de tarea")
    kind = task.get("kind", "legacy")
    if not isinstance(kind, str) or kind not in TASK_CYCLE_BOUNDS:
        raise TaskInputError("Tipo de tarea desconocido")
    minimum, maximum = TASK_CYCLE_BOUNDS[kind]
    cycles = task.get("cycles")
    if type(cycles) is not int or not minimum <= cycles <= maximum:
        raise TaskInputError("Se requiere un solo ciclo" if minimum == maximum
                             else f"Se requieren entre {minimum} y {maximum} ciclos")
    order = JOINT_TEST_ORDER if kind == "joint_test" else WAYPOINTS
    if kind == "custom":
        names = task.get("order")
        if (not isinstance(names, list) or not 1 <= len(names) <= MAX_TASK_WAYPOINTS
                or any(not isinstance(name, str) or not fullmatch(r"[A-Za-z0-9_ -]{1,32}", name)
                       for name in names) or len(set(names)) != len(names)):
            raise TaskInputError("Indique entre 1 y 32 nombres de tramo únicos en el orden del recorrido")
        order = tuple(names)
    elif kind in {"legacy", "cycle", "repeat"} and "order" in task:
        names = task["order"]
        if (not isinstance(names, list) or len(names) != len(WAYPOINTS)
                or any(not isinstance(name, str) for name in names)
                or set(names) != set(WAYPOINTS) or names[-1] != "HOME"):
            raise TaskInputError("Indique el orden de A, B y C, seguido de HOME")
        order = tuple(names)
    default_profile = task.get("profile")
    if default_profile is not None:
        try:
            planner_for_profile(default_profile)
        except ValueError as exc:
            raise TaskInputError(str(exc)) from exc
    waypoints = task.get("waypoints")
    if not isinstance(waypoints, dict) or set(waypoints) != set(order):
        raise TaskInputError("Defina INICIO y OBJETIVO explícitamente" if kind == "joint_test"
                             else "Defina un objetivo por tramo" if kind == "custom"
                             else "Defina A, B, C y HOME explícitamente")
    start = tuple(start)
    seed = start
    goals = []
    records = []
    durations = []
    profiles = []
    for name in order:
        waypoint = waypoints[name]
        try:
            if not isinstance(waypoint, dict) or ("q_rad" in waypoint) == ("matrix" in waypoint):
                raise ValueError("Indique seis valores q_rad o una matriz de pose")
            duration = waypoint.get("duration_s")
            if "duration_s" in waypoint and (type(duration) not in (int, float) or not 0.1 <= duration <= 30.0):
                raise ValueError("La duración debe estar entre 0,1 y 30 s")
            profile = waypoint.get("profile")
            if profile is None:
                profile = default_profile
            if profile is not None:
                planner_for_profile(profile)
            if "q_rad" in waypoint:
                model = points.flange
                values = waypoint["q_rad"]
                if not isinstance(values, list) or any(type(q) not in (float, int) for q in values):
                    raise ValueError("q_rad debe ser una lista de números en radianes")
                goal = tuple(values)
                record = dict(point="flange", branch=None)
            else:
                if kind == "joint_test":
                    raise ValueError("La prueba articular requiere seis valores q_rad")
                point = waypoint.get("point", "tcp")
                model = points.at(point)
                solutions = model.ik_pieper(waypoint["matrix"], seed=seed)
                candidates = [dict(branch=item.branch, q_rad=item.q, singular=item.singular,
                                   position_error_m=item.position_error_m,
                                   orientation_error_rad=item.orientation_error_rad)
                              for item in solutions]
                selected = next((item for item in candidates if item["branch"] == waypoint.get("branch")), None)
                if selected is None:
                    raise TaskInputError(f"{name}: seleccione una rama Pieper aplicable", name, candidates)
                goal = selected["q_rad"]
                record = dict(point=point, requested_matrix=waypoint["matrix"], **selected)
            segment = plan_sequence(seed, [goal], max_velocity=velocity).segments[0]
            goal = segment.goal
            if any(not lower <= q <= upper for q, (lower, upper) in zip(goal, model.limits)):
                raise ValueError("La configuración está fuera del límite del modelo")
        except TaskInputError:
            raise
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise TaskInputError(f"{name}: {exc}", name) from exc
        records.append(dict(name=name, **record))
        durations.append(duration)
        profiles.append(profile)
        goals.append(goal)
        seed = goal
    required_start = goals[-1] if kind in {"joint_test", "cycle", "repeat"} else None
    if kind == "joint_test":
        configured_target, configured_start = goals
        axes = [i for i, (a, b) in enumerate(zip(configured_target, configured_start)) if a != b]
        if len(axes) != 1:
            raise TaskInputError("OBJETIVO debe cambiar exactamente una articulación de INICIO", "OBJETIVO")
        axis = axes[0]
        if start[axis] == configured_target[axis]:
            raise TaskInputError("La articulación seleccionada ya está en OBJETIVO", "OBJETIVO")
        for record, goal in zip(records, goals):
            record["configured_q_rad"] = goal
        # Keep untested joints at their live positions and return to the same origin.
        target = list(start)
        target[axis] = configured_target[axis]
        goals = (target, start)

    try:
        trajectory = plan_sequence(start, goals * cycles, max_velocity=velocity,
                                   segment_planner=segment_planner, durations=durations * cycles,
                                   profiles=profiles * cycles)
    except ValueError as exc:
        # Resolve the first infeasible explicit time for display beside its waypoint.
        seed = start
        for i, (goal, duration, profile) in enumerate(
            zip(goals * cycles, durations * cycles, profiles * cycles, strict=True)
        ):
            if duration is not None:
                planner = segment_planner if profile is None else planner_for_profile(profile)
                minimum = planner(seed, goal, max_velocity=velocity).duration
                if duration < minimum:
                    name = order[i % len(order)]
                    raise TaskInputError(
                        f"{name}: {duration:.3g} s no es factible. Mínimo {minimum:.3f} s con estos límites",
                        name,
                    ) from exc
            seed = goal
        raise TaskInputError(str(exc)) from exc
    for record, segment in zip(records, trajectory.segments):
        model = points.at(record["point"])
        goal = segment.goal
        margins = tuple(min(q - lower, upper - q) for q, (lower, upper) in zip(goal, model.limits))
        if kind == "joint_test" and any(margin < 0 for margin in margins):
            raise TaskInputError(f"{record['name']}: configuración fuera del límite del modelo", record["name"])
        record.update(q_rad=goal, calculated_matrix=model.fk(goal),
                      first_duration_s=segment.duration, limit_margin_rad=margins, profile=segment.profile)
        if segment.profile == "lspb":
            record["first_lspb_s"] = segment.duration
    checks = []
    # First traversal and, if applicable, the transition from the final point to a new cycle.
    for index, segment in enumerate(trajectory.segments[:len(order) + 1]):
        delta = [b - a for a, b in zip(segment.start, segment.goal)]
        intervals = max(1, ceil(max(map(abs, delta)) / 0.05))
        sigma = min(
            points.flange.singular_values(
                tuple(a + d * step / intervals for a, d in zip(segment.start, delta)), 0.25
            )[-1]
            for step in range(intervals + 1)
        )
        checks.append(dict(
            waypoint=order[index % len(order)], cycle=1 + index // len(order),
            max_delta_rad=max(map(abs, delta)), min_sigma_scaled=sigma,
            duration_s=segment.duration, profile=segment.profile,
        ))
        if segment.profile == "lspb":
            checks[-1]["lspb_s"] = segment.duration
    return TaskPlan(trajectory, tuple(records), cycles, tuple(checks), kind=kind, order=order,
                    required_start=required_start)
