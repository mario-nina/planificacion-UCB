"""Shared joint-space trajectory planning in radians and seconds."""

from bisect import bisect_right
from dataclasses import dataclass
from math import ceil, hypot, isfinite, sqrt
from typing import ClassVar, Iterable, Protocol

JOINT_NAMES = tuple(f"joint_{joint}" for joint in range(1, 7))
# Matches the limits in the audited robot description, in ROS joint order.
JOINT_LIMITS = (
    (-2.879793, 2.879793),
    (-1.919862, 1.919862),
    (-1.919862, 1.221730),
    (-2.792527, 2.792527),
    (-2.094395, 2.094395),
    (-6.981317, 6.981317),
)
MODEL_VELOCITIES = (4.363323, 4.363323, 4.363323, 5.585054, 5.585054, 7.330383)

# Fraction of total time spent in each parabolic blend.
BLEND = 0.25
DEFAULT_VELOCITY = (1.0,) * 6
# Moderate study/reference ceiling; not a measured servo speed or an ABB rating.
AUTO_REFERENCE_VELOCITIES = (0.6,) * 6
DEFAULT_ACCELERATION = (1.0,) * 6
PROFILE_NAMES = ("lspb", "quintic")


@dataclass(frozen=True)
class Sample:
    time: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]


@dataclass(frozen=True)
class PathCandidate:
    branch: str
    source: str
    q: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float
    score: float | None = None
    reason: str = ""
    trajectory: "Segment | None" = None
    min_sigma: float | None = None
    joint_distance_rad: float | None = None
    collision: str = "NO_VERIFICADA"


class _IKCandidate(Protocol):
    branch: str
    source: str
    q: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float


class _KinematicModel(Protocol):
    def singular_values(self, q, characteristic_length_m: float) -> tuple[float, ...]: ...
    def fk(self, q) -> tuple[tuple[float, ...], ...]: ...


def _check_time(t, duration):
    if not isfinite(t) or not 0.0 <= t <= duration:
        raise ValueError("El tiempo debe pertenecer al intervalo de la trayectoria")


def _sample_times(duration, period):
    if not isfinite(period) or period <= 0:
        raise ValueError("El periodo de muestreo debe ser finito y positivo")
    count = duration / period
    if count > 100_000:
        raise ValueError("La trayectoria excede 100000 intervalos de muestreo")
    intervals = max(1, ceil(count))
    return tuple(duration * (i / intervals) for i in range(intervals)) + (duration,)


@dataclass(frozen=True)
class Segment:
    profile: ClassVar[str] = "lspb"
    start: tuple[float, ...]
    goal: tuple[float, ...]
    duration: float
    blend_fraction: float = BLEND

    def sample(self, t):
        """Evaluate synchronized position, velocity and ideal acceleration."""
        _check_time(t, self.duration)
        t_scaled = t / self.duration
        blend = self.blend_fraction
        cruise = 1.0 / (1.0 - blend)
        accel = cruise / blend
        if t_scaled < blend:
            pos = 0.5 * accel * t_scaled**2
            vel = accel * t_scaled
        elif t_scaled < 1.0 - blend:
            pos = cruise * (t_scaled - blend / 2.0)
            vel = cruise
            accel = 0.0
        else:
            # Roundoff in the subtraction near the cruise boundary may make
            # the remaining fraction slightly longer than the blend itself.
            remaining = min(blend, (self.duration - t) / self.duration)
            pos = 1.0 - 0.5 * accel * remaining**2
            vel = accel * remaining
            accel = -accel
        delta = tuple(target - initial for initial, target in zip(self.start, self.goal))
        if t == 0:
            positions = self.start
        elif t == self.duration:
            positions = self.goal
            # At the stopped endpoint the acceleration is zero. The negative
            # one-sided deceleration remains available just before arrival.
            accel = 0.0
        else:
            positions = tuple(
                initial + displacement * pos
                for initial, displacement in zip(self.start, delta)
            )
        return Sample(
            t,
            positions,
            tuple(displacement * vel / self.duration for displacement in delta),
            tuple(displacement * accel / self.duration / self.duration for displacement in delta),
        )

    def samples(self, period=0.02):
        """Return uniformly spaced samples including both endpoints."""
        return tuple(self.sample(t) for t in _sample_times(self.duration, period))


@dataclass(frozen=True)
class Sequence:
    segments: tuple[Segment, ...]
    waypoint_times: tuple[float, ...]

    @property
    def duration(self):
        return self.waypoint_times[-1]

    @property
    def profiles(self):
        return tuple(segment.profile for segment in self.segments)

    def sample(self, t):
        _check_time(t, self.duration)
        index = min(bisect_right(self.waypoint_times, t), len(self.segments) - 1)
        segment = self.segments[index]
        start = self.waypoint_times[index - 1] if index else 0.0
        local_t = segment.duration if t == self.duration else t - start
        sample = segment.sample(min(local_t, segment.duration))
        return Sample(t, sample.positions, sample.velocities, sample.accelerations)

    def samples(self, period=0.02):
        """Include each waypoint exactly once, with zero waypoint velocity."""
        times = sorted(set(_sample_times(self.duration, period)) | set(self.waypoint_times))
        return tuple(self.sample(t) for t in times)


def _joint_vector(values, description):
    try:
        vector = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{description}: se requieren seis valores numéricos") from exc
    if len(vector) != 6 or not all(isfinite(value) for value in vector):
        raise ValueError(f"{description}: se requieren seis valores finitos")
    return vector


def plan(
    start,
    goal,
    *,
    duration=None,
    max_velocity=DEFAULT_VELOCITY,
    max_acceleration=DEFAULT_ACCELERATION,
    blend_fraction=None,
):
    """Plan a straight joint-space segment (radians, seconds, rest to rest).

    All joints share one progress variable. Automatic timing uses a triangular
    profile if it can stop before cruise is needed, or a trapezoid otherwise.
    With explicit duration, the historical quarter-duration blend remains
    available when feasible. An explicit blend always fixes the profile shape.
    """
    start = _joint_vector(start, "Inicio")
    goal = _joint_vector(goal, "Objetivo")
    velocity = _joint_vector(max_velocity, "Velocidad máxima")
    acceleration = _joint_vector(max_acceleration, "Aceleración máxima")
    velocity_time = acceleration_time = 0.0
    for name, (lower, upper), nominal, initial, target, vmax, amax in zip(
        JOINT_NAMES, JOINT_LIMITS, MODEL_VELOCITIES, start, goal, velocity, acceleration
    ):
        if not (lower <= initial <= upper and lower <= target <= upper):
            raise ValueError(f"{name}: posición fuera del límite")
        if vmax <= 0.0 or amax <= 0.0:
            raise ValueError(f"{name}: se requieren cotas dinámicas positivas")
        displacement = abs(target - initial)
        velocity_time = max(velocity_time, displacement / min(vmax, nominal))
        acceleration_time = max(acceleration_time, sqrt(displacement / amax))
    if duration is not None:
        duration = _duration(duration)

    def minimum_for_fraction(fraction):
        return max(velocity_time / (1.0 - fraction),
                   acceleration_time / sqrt(fraction * (1.0 - fraction)))

    if blend_fraction is not None:
        try:
            blend_fraction = _duration(blend_fraction)
        except ValueError as exc:
            raise ValueError("La fracción de cada transición LSPB debe estar en (0, 0,5]") from exc
        if blend_fraction > 0.5 or not isfinite(1.0 / (blend_fraction * (1.0 - blend_fraction))):
            raise ValueError("La fracción de cada transición LSPB debe estar en (0, 0,5]")
        minimum = minimum_for_fraction(blend_fraction)
    elif duration is not None and duration >= minimum_for_fraction(BLEND):
        blend_fraction = BLEND
        minimum = minimum_for_fraction(blend_fraction)
    elif velocity_time <= acceleration_time:
        blend_fraction = 0.5
        minimum = 2.0 * acceleration_time
    else:
        ramp = acceleration_time * (acceleration_time / velocity_time)
        minimum = velocity_time + ramp
        blend_fraction = ramp / minimum

    duration = _duration(max(0.1, minimum) if duration is None else duration)
    if duration < minimum or not 0.0 < blend_fraction <= 0.5:
        raise ValueError("La duración debe ser positiva, finita y factible")
    return Segment(start, goal, duration, blend_fraction)


def _duration(value):
    try:
        if isinstance(value, bool):
            raise ValueError
        seconds = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("La duración debe ser positiva y finita") from exc
    if not isfinite(seconds) or seconds <= 0:
        raise ValueError("La duración debe ser positiva y finita")
    return seconds


def planner_for_profile(profile):
    """Resolve a catalog profile without coupling it to a controller."""
    if profile == "lspb":
        return plan
    if profile == "quintic":
        from .smooth import plan_quintic
        return plan_quintic
    raise ValueError("Perfil desconocido. Seleccione LSPB o quíntico")


def plan_sequence(
    start,
    goals,
    *,
    max_velocity=DEFAULT_VELOCITY,
    max_acceleration=DEFAULT_ACCELERATION,
    segment_planner=plan,
    durations=None,
    profiles=None,
):
    """Join rest-to-rest segments through ordered joint-space waypoints."""
    segments = []
    times = []
    elapsed = 0.0
    velocity = tuple(max_velocity)
    acceleration = tuple(max_acceleration)
    goals = tuple(goals)
    durations = (None,) * len(goals) if durations is None else tuple(durations)
    profiles = (None,) * len(goals) if profiles is None else tuple(profiles)
    if len(durations) != len(goals) or len(profiles) != len(goals):
        raise ValueError("Se requiere una duración y un perfil por objetivo, o modo automático")
    for goal, duration, profile in zip(goals, durations, profiles, strict=True):
        planner = segment_planner if profile is None else planner_for_profile(profile)
        timing = {"duration": duration} if duration is not None else {}
        segment = planner(
            start, goal, max_velocity=velocity, max_acceleration=acceleration, **timing,
        )
        segments.append(segment)
        elapsed += segment.duration
        if not isfinite(elapsed) or (times and elapsed <= times[-1]):
            raise ValueError("La secuencia no tiene tiempos finitos y crecientes")
        times.append(elapsed)
        start = segment.goal
    if not segments:
        raise ValueError("Se requiere al menos un objetivo")
    return Sequence(tuple(segments), tuple(times))


def plan_candidates(
    model: _KinematicModel,
    start: Iterable[float],
    candidates: Iterable[_IKCandidate],
    *,
    global_scale=1.0,
    axis_scales=(1.0,) * 6,
    max_acceleration=DEFAULT_ACCELERATION,
    max_velocity=MODEL_VELOCITIES,
    min_sigma=0.0,
    characteristic_length_m=0.5,
    max_joint_delta_rad=None,
    score_weights=(1.0, 1.0, 0.1),
    segment_planner=plan,
) -> tuple[PathCandidate, ...]:
    """Rank direct joint paths, retaining rejected candidates and their reasons.

    The score adds duration in seconds, straight joint distance in radians and
    the reciprocal of (1 + minimum sampled scaled singular value), using the
    respective weights. The result is an empirical ranking of these routes.
    The singularity filter samples the unwrapped joint segment every 0.05 rad
    or less along each axis. Zero threshold disables rejection by that metric.
    """
    start = _joint_vector(start, "Inicio")
    axis_scales = _joint_vector(axis_scales, "Escalas por eje")
    if not all(0 < scale <= 1 for scale in (global_scale, *axis_scales)):
        raise ValueError("Las escalas de velocidad deben estar en (0, 1]")
    if not isfinite(min_sigma) or min_sigma < 0:
        raise ValueError("El umbral de sigma debe ser finito y no negativo")
    if not isfinite(characteristic_length_m) or characteristic_length_m <= 0:
        raise ValueError("La longitud característica debe ser finita y positiva")
    if max_joint_delta_rad is not None and (
        not isfinite(max_joint_delta_rad) or max_joint_delta_rad <= 0
    ):
        raise ValueError("El desplazamiento articular máximo debe ser finito y positivo")
    score_weights = tuple(score_weights)
    if len(score_weights) != 3 or not all(isfinite(weight) and weight >= 0 for weight in score_weights):
        raise ValueError("Se requieren tres pesos finitos y no negativos")
    velocity = tuple(
        nominal * global_scale * axis for nominal, axis in zip(
            _joint_vector(max_velocity, "Velocidad máxima"), axis_scales)
    )
    acceleration = _joint_vector(max_acceleration, "Aceleración máxima")
    # Validate common inputs even if IK produced an empty candidate list.
    segment_planner(start, start, max_velocity=velocity, max_acceleration=acceleration)
    result = []
    for candidate in candidates:
        q = tuple(candidate.q)
        fields = (candidate.branch, candidate.source, q,
                  candidate.position_error_m, candidate.orientation_error_rad)
        try:
            trajectory = segment_planner(start, q, max_velocity=velocity, max_acceleration=acceleration)
        except ValueError as error:
            result.append(PathCandidate(*fields, reason=str(error)))
            continue
        delta = tuple(goal - initial for initial, goal in zip(start, trajectory.goal))
        largest_delta = max(map(abs, delta))
        distance = hypot(*delta)
        if max_joint_delta_rad is not None and largest_delta > max_joint_delta_rad:
            result.append(PathCandidate(
                *fields, reason="Desplazamiento articular máximo excedido",
                joint_distance_rad=distance,
            ))
            continue
        intervals = max(1, ceil(largest_delta / 0.05))
        grid = (
            tuple(initial + offset * (index / intervals) for initial, offset in zip(start, delta))
            for index in range(intervals + 1)
        )
        route_sigma = min(model.singular_values(q_grid, characteristic_length_m)[-1] for q_grid in grid)
        if route_sigma < min_sigma:
            result.append(PathCandidate(
                *fields, reason="Sigma mínimo muestreado bajo el umbral",
                min_sigma=route_sigma, joint_distance_rad=distance,
            ))
            continue
        duration_weight, distance_weight, sigma_weight = score_weights
        score = (duration_weight * trajectory.duration + distance_weight * distance
                 + sigma_weight / (1 + route_sigma))
        result.append(PathCandidate(
            *fields, score=score, trajectory=trajectory,
            min_sigma=route_sigma, joint_distance_rad=distance,
        ))
    return tuple(sorted(result, key=lambda path: (path.score is None, path.score or 0, path.branch)))


def tcp_samples(model, trajectory, period=0.02):
    """Pair each planned sample with its base_link to TCP transform."""
    return tuple((sample, model.fk(sample.positions)) for sample in trajectory.samples(period))
