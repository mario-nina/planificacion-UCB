"""Cartesian references and checked angular chords for the four-wall cell."""

from dataclasses import dataclass, replace
from math import ceil, dist, hypot, isfinite, pi, tau

from imt342_irb120_kinematics import pose_from_rpy
from imt342_irb120_kinematics.analysis import pose_residual

from . import _joint_vector, plan, planner_for_profile
from .task import MAX_TASK_WAYPOINTS, TaskInputError, plan_task


SEPARATION_M = .100
SEPARATION_TOLERANCE_M = .010
MAX_DEVIATION_M = .008
MAX_ORIENTATION_ERROR_RAD = .14
MIN_SCALED_SIGMA = .025
MAX_TRACKING_STEP_RAD = .6
APPROACH_RISE_M = .150
WALLS = ("x_max", "y_min", "y_max", "x_min")
WALL_LABELS = {"x_min": "X−", "x_max": "X+", "y_min": "Y−", "y_max": "Y+"}


@dataclass(frozen=True)
class _Reference:
    variant: str
    corners: tuple
    walls: tuple
    closed: bool = False


@dataclass(frozen=True)
class _Destination:
    name: str
    q: tuple
    pose: tuple
    branch: str | None
    wall: str | None = None


def _references(cell, kind, wall):
    (xmin, xmax), (ymin, ymax) = cell.inner_bounds
    half = cell.wall_thickness / 2
    top = cell.wall_top_z
    if kind == "cell_wall":
        walls = WALLS if wall == "auto" else (wall,)
        for selected in walls:
            axis, side = selected.split("_")
            low, high = (ymin, ymax) if axis == "x" else (xmin, xmax)
            inset = min(.10, (high - low) / 5)
            fixed_low, fixed_high = (xmin, xmax) if axis == "x" else (ymin, ymax)
            fixed = fixed_low - half if side == "min" else fixed_high + half
            for trim in (inset, min(inset + .05, (high - low) / 3)):
                ends = (low + trim, high - trim)
                line = tuple((fixed, pos, top + SEPARATION_M) if axis == "x"
                             else (pos, fixed, top + SEPARATION_M) for pos in ends)
                yield _Reference("over_wall", line, (selected,))
        return

    walls = ("y_min", "x_max", "y_max", "x_min")
    for variant, offset, z in (
        ("over_perimeter", -half, top + SEPARATION_M),
        ("inner_perimeter", SEPARATION_M, top),
        ("inner_perimeter", SEPARATION_M, top - min(.025, top / 4)),
    ):
        lower_x, upper_x = xmin + offset, xmax - offset
        lower_y, upper_y = ymin + offset, ymax - offset
        if lower_x >= upper_x or lower_y >= upper_y:
            continue
        corners = ((lower_x, lower_y, z), (upper_x, lower_y, z),
                   (upper_x, upper_y, z), (lower_x, upper_y, z))
        yield _Reference(variant, (*corners, corners[0]), walls, closed=True)


def _perimeter_winding_blocked(points, cell, reference, point="flange"):
    # Pieper's wrist center lies in the plane spanned by J1's radial direction
    # and the vertical. If the TCP loop surrounds J1 even after erosion by the
    # largest possible horizontal wrist offset, the wrist must wind once too.
    if not reference.closed:
        return False
    model = points.flange
    lower, upper = model.limits[0]
    if upper - lower >= tau:
        return False
    axis, center = cell.first_joint_axis()
    if axis != (0., 0., 1.):
        return False
    # Include the accepted Cartesian chord deviation in the exclusion bound.
    offset = points.tool_offset if point == "tcp" else pose_from_rpy((0.,) * 3, (0.,) * 3)
    wrist_offset = cell.wrist_tcp_bound(offset) + MAX_DEVIATION_M
    vertices = tuple((x - center[0], y - center[1])
                     for x, y, _ in map(cell.to_base, reference.corners))
    edges = tuple((x * v - y * u, hypot(u - x, v - y))
                  for (x, y), (u, v) in zip(vertices, vertices[1:]))
    return (all(cross > wrist_offset * length for cross, length in edges)
            or all(-cross > wrist_offset * length for cross, length in edges))


def perimeter_available(points, cell, point="flange"):
    """Indicate whether a perimeter candidate passes the winding precheck."""
    return any(not _perimeter_winding_blocked(points, cell, reference, point)
               for reference in _references(cell, "cell_perimeter", "auto"))


def _linear(first, last, fraction):
    return tuple(a + (b - a) * fraction for a, b in zip(first, last, strict=True))


def _waypoints(reference, step):
    result = [(reference.corners[0], None)]
    for start, goal, wall in zip(reference.corners[:-1], reference.corners[1:], reference.walls, strict=True):
        count = max(1, ceil(dist(start, goal) / step))
        result.extend((_linear(start, goal, i / count), wall) for i in range(1, count))
        result.append((goal, wall))
    x, y, z = result[0][0]
    return [((x, y, z + APPROACH_RISE_M), None), *result]


def _joint_step(first, last):
    return max(abs(a - b) for a, b in zip(first, last, strict=True))


def _branch_family(solution):
    return solution.branch.rsplit("/", 1)[0]


def _ik_routes(model, cell, points, start):
    for roll, pitch, yaw in ((pi, 0., 0.), (pi, 0., pi / 2), (pi, 0., -pi / 2),
                             (pi, pi / 6, 0.), (pi, -pi / 6, 0.),
                             (pi, pi / 6, pi / 2), (pi, -pi / 6, pi / 2)):
        poses = tuple(cell.pose_to_base(pose_from_rpy(xyz, (roll, pitch, yaw))) for xyz, _ in points)
        solutions_by_pose = tuple(model.ik_pieper(pose, seed=start) for pose in poses)
        if not all(solutions_by_pose):
            continue
        for approach in solutions_by_pose[0]:
            if approach.singular:
                continue
            chain = [approach]
            for candidates in solutions_by_pose[1:]:
                compatible = (candidate for candidate in candidates if not candidate.singular
                              and _branch_family(candidate) == _branch_family(approach)
                              and _joint_step(candidate.q, chain[-1].q) <= MAX_TRACKING_STEP_RAD)
                chosen = min(compatible, key=lambda candidate: dist(candidate.q, chain[-1].q), default=None)
                if chosen is None:
                    break
                chain.append(chosen)
            if len(chain) == len(poses):
                yield tuple(zip(chain, poses, strict=True))


def _destinations(route, knots, reference, start, home_pose):
    names = ("APROX", "INICIO", *(f"AVANCE_{i:02d}" for i in range(1, len(knots) - 1)))
    outbound = tuple(_Destination(name, solution.q, pose, solution.branch, wall)
                     for name, (solution, pose), (_, wall) in zip(names, route, knots, strict=True))
    if reference.closed:
        if _joint_step(outbound[-1].q, outbound[1].q) > 1e-7:
            return ()
        reverse = ()
    else:
        reverse = tuple(replace(outbound[i], name=f"REGRESO_{len(outbound) - 1 - i:02d}",
                                wall=outbound[i + 1].wall)
                        for i in range(len(outbound) - 2, 0, -1))
    return (*outbound, *reverse, replace(outbound[0], name="SALIDA"),
            _Destination("HOME", start, home_pose, None))


def _xyz(pose):
    return tuple(row[3] for row in pose[:3])


def _distance_to_segment(point, start, goal):
    delta = tuple(b - a for a, b in zip(start, goal, strict=True))
    length2 = sum(value * value for value in delta)
    fraction = sum((p - a) * d for p, a, d in zip(point, start, delta, strict=True)) / length2
    return dist(point, _linear(start, goal, min(1., max(0., fraction)))), fraction


def _measure_route(model, cell, home, destinations):
    planned = []
    deviations, separations, rotations, tracking_sigma, transit_sigma = [], [], [], [], []
    previous = home
    for goal in destinations:
        n = max(8, ceil(_joint_step(previous.q, goal.q) / .025))
        progress = 0.
        for index in range(n + 1):
            q = _linear(previous.q, goal.q, index / n)
            pose = model.fk(q)
            xyz = _xyz(pose)
            sigma = model.singular_values(q)[-1]
            if not planned or index:
                planned.append(xyz)
            if goal.wall is None:
                transit_sigma.append(sigma)
                continue

            deviation, fraction = _distance_to_segment(xyz, _xyz(previous.pose), _xyz(goal.pose))
            clearance = cell.wall_distance(goal.wall, xyz)
            rotation = pose_residual(goal.pose, pose)["orientation_error_rad"]
            if fraction < progress - .002:
                raise TaskInputError(f"{goal.name}: el punto de referencia retrocede durante el recorrido")
            progress = fraction
            if (deviation > MAX_DEVIATION_M or abs(clearance - SEPARATION_M) > SEPARATION_TOLERANCE_M
                    or rotation > MAX_ORIENTATION_ERROR_RAD or sigma < MIN_SCALED_SIGMA):
                raise TaskInputError(f"{goal.name}: desviación, separación, orientación o singularidad fuera de cota")
            deviations.append(deviation)
            separations.append(clearance)
            rotations.append(rotation)
            tracking_sigma.append(sigma)
        previous = goal

    return dict(
        planned_xyz_m=planned,
        max_tcp_deviation_m=max(deviations),
        wall_clearance_min_m=min(separations),
        wall_clearance_max_m=max(separations),
        max_tcp_orientation_error_rad=max(rotations),
        min_sigma_scaled=min(tracking_sigma),
        min_transit_sigma_scaled=min(transit_sigma),
        sampled_configurations=len(planned),
    )


def _check_wall_collision(points, cell, start, destinations):
    previous = start
    checked = set()
    configurations = 0
    for destination in destinations:
        edge = frozenset((previous, destination.q))
        if edge not in checked:
            report = cell.check_segment(previous, destination.q, tool_offset=points.tool_offset)
            if not report.ok:
                raise TaskInputError(f"{destination.name}: {report.reason}", destination.name)
            configurations += report.configurations_checked
            checked.add(edge)
        previous = destination.q
    return configurations


def _plan_route(points, start, task, velocity, cell, reference, segment_planner, max_duration):
    point = task.get("point", "flange")
    model = points.at(point)
    home_pose = model.fk(start)
    home = _Destination("HOME", start, home_pose, None)
    last_error = None
    for spacing in (.14, .105, .078):
        knots = _waypoints(reference, spacing)
        count = len(knots) + 2 if reference.closed else 2 * len(knots)
        if count > MAX_TASK_WAYPOINTS:
            continue
        for ik_route in _ik_routes(model, cell, knots, start):
            destinations = _destinations(ik_route, knots, reference, start, home_pose)
            if not destinations:
                continue
            try:
                metrics = _measure_route(model, cell, home, destinations)
                checked = _check_wall_collision(points, cell, start, destinations)
                names = tuple(item.name for item in destinations)
                plan_config = dict(kind="custom", cycles=task["cycles"], order=list(names),
                                   profile=task.get("profile"), waypoints={
                                       item.name: {"q_rad": list(item.q)} for item in destinations})
                planned = plan_task(points, start, plan_config, velocity, segment_planner=segment_planner)
                if max_duration is not None and any(
                    segment.duration > max_duration for segment in planned.trajectory.segments
                ):
                    raise TaskInputError("Un tramo supera la duración admitida por RAPID")
            except TaskInputError as exc:
                last_error = exc
                continue
            records = tuple(dict(record, point=point, branch=destination.branch,
                                 requested_matrix=destination.pose, calculated_matrix=model.fk(destination.q),
                                 wall=destination.wall)
                            for record, destination in zip(planned.waypoints, destinations, strict=True))
            tracked = tuple(_xyz(item.pose) for item in destinations[1:] if
                            item.name == "INICIO" or item.wall is not None)
            result = replace(planned, kind=task["kind"], waypoints=records, required_start=start)
            return result, dict(
                reference_point=point, variant=reference.variant, wall=reference.walls[0] if not reference.closed else None,
                wall_label=WALL_LABELS[reference.walls[0]] if not reference.closed else None,
                reference_xyz_m=tracked, home_q_rad=start, waypoint_names=names,
                nominal_separation_m=SEPARATION_M, command_segments=len(destinations),
                total_command_segments=len(planned.trajectory.segments),
                collision_configurations=checked, **metrics,
            )
    if last_error:
        raise last_error
    raise TaskInputError("No hay una rama IK continua para aproximación, recorrido y salida")


def plan_cell_task(points, start, task, velocity, cell, *, segment_planner=plan, max_duration=None):
    """Build a catalog-compatible plan with one captured HOME and Cartesian evidence."""
    if not isinstance(task, dict) or task.get("kind") not in ("cell_wall", "cell_perimeter"):
        raise TaskInputError("Tipo de recorrido de celda no reconocido")
    kind = task["kind"]
    point = task.get("point", "flange")
    if point not in ("flange", "tcp"):
        raise TaskInputError("Punto de recorrido debe ser Brida o TCP activo")
    cycles = task.get("cycles")
    if type(cycles) is not int or not 1 <= cycles <= 8:
        raise TaskInputError("El recorrido requiere de 1 a 8 ciclos")
    wall = task.get("wall", "auto")
    if wall not in (*WALLS, "auto") or (kind == "cell_perimeter" and wall != "auto"):
        raise TaskInputError("Muro de celda no reconocido")
    if max_duration is not None and (not isfinite(max_duration) or max_duration <= 0.):
        raise TaskInputError("La duración máxima debe ser positiva y finita")
    if task.get("profile") is not None:
        try:
            planner_for_profile(task["profile"])
        except ValueError as exc:
            raise TaskInputError(str(exc)) from exc
    try:
        home = _joint_vector(start, "HOME")
    except ValueError as exc:
        raise TaskInputError(str(exc), "HOME") from exc
    if any(not lower <= angle <= upper for angle, (lower, upper) in zip(home, points.flange.limits, strict=True)):
        raise TaskInputError("HOME fuera de los límites articulares", "HOME")
    report = cell.check_pose(home, tool_offset=points.tool_offset)
    if not report.ok:
        raise TaskInputError(f"HOME: {report.reason}", "HOME")
    failures = {}
    for reference in _references(cell, kind, wall):
        label = reference.variant if reference.closed else WALL_LABELS[reference.walls[0]]
        if _perimeter_winding_blocked(points, cell, reference, point):
            failures[label] = "el cierre exigiría una vuelta completa de J1 fuera de sus límites"
            continue
        try:
            return _plan_route(points, home, task, velocity, cell, reference, segment_planner, max_duration)
        except TaskInputError as exc:
            failures[label] = str(exc)
    detail = ", ".join(f"{name}: {reason}" for name, reason in failures.items())
    raise TaskInputError(f"No se encontró un recorrido factible. {detail}".rstrip())
