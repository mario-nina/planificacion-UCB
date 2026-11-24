"""Synthetic study fixture with a shared simulation clock. No transport."""
from dataclasses import dataclass
from math import isfinite, sqrt
from random import Random

from imt342_irb120_planning import JOINT_LIMITS


@dataclass(frozen=True)
class Config:
    dt_s: float = .02
    delay_s: float = .06
    velocity_rad_s: float = .75
    acceleration_rad_s2: float = 1.2
    response_s: float = .12
    noise_rad: float = .0005
    loss_probability: float = .03
    seed: int = 342


def simulate(plan, config=Config()):
    values = (config.dt_s, config.velocity_rad_s, config.acceleration_rad_s2, config.response_s)
    if not all(isfinite(value) and value > 0 for value in values) or not (
        isfinite(config.delay_s) and config.delay_s >= 0 and isfinite(config.noise_rad)
        and config.noise_rad >= 0 and 0 <= config.loss_probability <= 1
    ):
        raise ValueError("Parámetros sintéticos inválidos")
    intervals = plan.duration / config.dt_s
    if intervals > 100_000:
        raise ValueError("Demasiadas muestras")
    times = [i * config.dt_s for i in range(int(intervals) + 1) if i * config.dt_s < plan.duration]
    times.append(plan.duration)
    rng = Random(config.seed)
    q, velocity = list(plan.start), [0.] * 6
    previous = 0.
    rows = []
    for t in times:
        dt = t - previous
        previous = t
        reference = plan.sample(t)
        delayed = plan.sample(max(0., t - config.delay_s)).positions
        limited_axes = 0
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            target_speed = (delayed[i] - q[i]) / config.response_s
            speed = max(-config.velocity_rad_s, min(config.velocity_rad_s, target_speed))
            max_change = config.acceleration_rad_s2 * dt
            increment = max(-max_change, min(max_change, speed - velocity[i]))
            next_speed = velocity[i] + increment
            predicted = q[i] + dt * (next_speed + velocity[i]) / 2
            q[i] = max(lo, min(hi, predicted))
            velocity[i] = next_speed if q[i] == predicted else 0.
            limited_axes += int(speed != target_speed or next_speed != speed or q[i] != predicted)
        measured = None if rng.random() < config.loss_probability else tuple(
            value + rng.gauss(0., config.noise_rad) for value in q)
        rows.append(dict(time_s=t, reference_rad=reference.positions, planned_qdot_rad_s=reference.velocities,
                         planned_qddot_rad_s2=reference.accelerations, delayed_rad=delayed,
                         synthetic_measured_rad=measured, limited_axes=limited_axes))
    return rows


def summarize(rows):
    errors = [[] for _ in JOINT_LIMITS]
    for row in rows:
        measured = row["synthetic_measured_rad"]
        if measured is not None:
            for axis, (actual, expected) in enumerate(zip(measured, row["reference_rad"])):
                errors[axis].append(actual - expected)
    last = rows[-1]
    final = last["synthetic_measured_rad"]
    return dict(source="Simulador sintético; reloj común del ensayo",
                rms_rad=[sqrt(sum(x*x for x in axis)/len(axis)) if axis else None for axis in errors],
                max_abs_rad=[max(map(abs, axis)) if axis else None for axis in errors],
                final_error_rad=None if final is None else [x-y for x,y in zip(final,last["reference_rad"])],
                missing_samples=sum(row["synthetic_measured_rad"] is None for row in rows),
                limited_samples=sum(bool(row["limited_axes"]) for row in rows))
