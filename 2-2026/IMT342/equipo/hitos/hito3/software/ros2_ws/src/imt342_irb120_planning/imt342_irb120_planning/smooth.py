"""Rest-to-rest synchronized quintic with bounded analytic jerk.

The acceleration and jerk inputs are study parameters, not ABB ratings.
"""
from math import cbrt, sqrt

from . import (
    DEFAULT_ACCELERATION, DEFAULT_VELOCITY, JOINT_LIMITS, MODEL_VELOCITIES,
    Sample, Segment, _check_time, _duration, _joint_vector,
)

SYNTHETIC_JERK = (5.0,) * 6


class QuinticSegment(Segment):
    profile = "quintic"

    def sample(self, t):
        _check_time(t, self.duration)
        if t in (0.0, self.duration):
            return Sample(t, self.start if t == 0 else self.goal, (0.0,) * 6, (0.0,) * 6)
        u = t / self.duration
        fraction = u**3 * (10 + u * (-15 + 6 * u))
        speed = 30 * u**2 * (1 - u)**2
        acceleration = 60 * u * (1 - u) * (1 - 2 * u)
        delta = tuple(b - a for a, b in zip(self.start, self.goal))
        return Sample(t, tuple(min(max(a, b), max(min(a, b), a + (b - a) * fraction))
                               for a, b in zip(self.start, self.goal)),
                      tuple(d * speed / self.duration for d in delta),
                      tuple(d * acceleration / self.duration / self.duration for d in delta))

    def jerk(self, t):
        _check_time(t, self.duration)
        u = t / self.duration
        scale = 60 * (1 - 6 * u + 6 * u**2)
        return tuple((b - a) * scale / self.duration / self.duration / self.duration
                     for a, b in zip(self.start, self.goal))


def plan_quintic(start, goal, max_velocity=DEFAULT_VELOCITY,
                 max_acceleration=DEFAULT_ACCELERATION, max_jerk=SYNTHETIC_JERK, duration=None):
    start, goal, velocity, acceleration, jerk = (
        _joint_vector(values, name) for values, name in
        ((start, "Inicio"), (goal, "Objetivo"), (max_velocity, "Velocidad máxima"),
         (max_acceleration, "Aceleración máxima"), (max_jerk, "Jerk máximo"))
    )
    required = 0.0
    for (lo, hi), nominal, a, b, v, acc, j in zip(
        JOINT_LIMITS, MODEL_VELOCITIES, start, goal, velocity, acceleration, jerk
    ):
        if not (lo <= a <= hi and lo <= b <= hi):
            raise ValueError("Inicio u objetivo fuera de límites articulares")
        if min(v, acc, j) <= 0:
            raise ValueError("Velocidad, aceleración y jerk deben ser positivos")
        d = abs(b - a)
        # Analytic extrema in normalized time of 10u³ − 15u⁴ + 6u⁵.
        required = max(required, (15 / 8) * d / min(v, nominal),
                       sqrt((10 / sqrt(3)) * d / acc), cbrt(60 * d / j))
    chosen = _duration(max(0.1, required) if duration is None else duration)
    if chosen < required:
        raise ValueError("Duración no factible para el perfil quíntico")
    return QuinticSegment(start, goal, chosen)
