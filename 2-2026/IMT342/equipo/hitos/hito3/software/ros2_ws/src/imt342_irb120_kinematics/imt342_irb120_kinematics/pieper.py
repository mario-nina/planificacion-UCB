"""Closed-form solution for the ZYY shoulder/elbow and XYX spherical wrist."""

from dataclasses import dataclass
from itertools import product
from math import atan2, ceil, floor, hypot, pi, sqrt, tau

from .model import _IDENTITY, _dot, _multiply, _pose_error, _rotation, _vector
from .pose import validate_pose


@dataclass(frozen=True)
class IKCandidate:
    q: tuple[float, ...]
    branch: str
    position_error_m: float
    orientation_error_rad: float
    singular: bool
    source: str = "pieper"


def _geometry(model):
    joints = model._chain[:6]
    axes = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0))
    if (tuple(joint.axis for joint in joints) != axes
            or any(joint.axis is not None for joint in model._chain[6:])):
        raise ValueError("Pieper requiere la topología ZYY–XYX del URDF IRB120")
    translations = []
    for joint, axis_index in zip(joints, (None, 2, 2, 2, 0, 0)):
        translation = tuple(row[3] for row in joint.origin[:3])
        if (tuple(row[:3] for row in joint.origin[:3]) != tuple(row[:3] for row in _IDENTITY[:3])
                or any(value != 0 for i, value in enumerate(translation) if i != axis_index)):
            raise ValueError("Origen articular incompatible con la derivación Pieper")
        translations.append(translation[axis_index] if axis_index is not None else 0)
    tail = _IDENTITY
    for joint in model._chain[6:]:
        tail = _multiply(tail, joint.origin)
    return (*translations[1:], tail)


def _inverse(transform):
    rotation = tuple(zip(*(row[:3] for row in transform[:3])))
    translation = tuple(row[3] for row in transform[:3])
    return tuple(row + (-_dot(row, translation),) for row in rotation) + (_IDENTITY[3],)


def _turns(angle, limits):
    low, high = limits
    first = ceil((low - angle - 1e-10) / tau)
    last = floor((high - angle + 1e-10) / tau)
    return tuple(min(high, max(low, angle + tau * turn)) for turn in range(first, last + 1))


def _wrist(rotation, limits, seed):
    magnitude = hypot(rotation[1][0], rotation[2][0])
    if magnitude > 1e-10:
        for sign, label in ((1, "M+"), (-1, "M-")):
            yield label, False, (
                atan2(sign * rotation[1][0], -sign * rotation[2][0]),
                atan2(sign * magnitude, rotation[0][0]),
                atan2(sign * rotation[0][1], sign * rotation[0][2]),
            )
        return

    # At q5=0 only the sum q4+q6 is observable. Pick the nearest
    # pair to the seed on each feasible line within the joint bounds.
    # The other XYX degeneracy q5=±π lies outside the audited IRB120 limits.
    if rotation[0][0] < 0:
        return
    angle = atan2(rotation[2][1], rotation[1][1])
    lo4, hi4 = limits[3]
    lo6, hi6 = limits[5]
    for total in _turns(angle, (lo4 + lo6, hi4 + hi6)):
        lower, upper = max(lo4, total - hi6), min(hi4, total - lo6)
        q4 = min(upper, max(lower, (seed[3] + total - seed[5]) / 2))
        yield "M0", True, (q4, 0.0, total - q4)


def solve(model, target, seed=None) -> tuple[IKCandidate, ...]:
    """Enumerate up to eight nonsingular geometric branches, plus allowed full turns.

    The base-axis and wrist singularities admit continuous families. Return
    representatives anchored to the seed and joint bounds for those cases.
    """
    target = validate_pose(target)
    seed = (0.0,) * 6 if seed is None else _vector(seed, 6)
    height, upper_arm, elbow_z, elbow_x, flange_x, tcp_tail = _geometry(model)
    flange = _multiply(target, _inverse(tcp_tail))
    x, y, z = (row[3] - flange_x * row[0] for row in flange[:3])
    z -= height
    radial = hypot(x, y)
    forearm = hypot(elbow_x, elbow_z)
    offset = atan2(elbow_x, elbow_z)
    distance = hypot(radial, z)
    if distance > upper_arm + forearm + 1e-10:
        return ()
    cosine = (distance**2 - upper_arm**2 - forearm**2) / (2 * upper_arm * forearm)
    if abs(cosine) > 1 + 1e-10:
        return ()
    cosine = min(1.0, max(-1.0, cosine))
    sine = sqrt(max(0.0, 1 - cosine**2))
    limits = model.limits
    if radial <= 1e-10:
        low, high = limits[0]
        shoulders = (("H0", min(high, max(low, seed[0])), 0.0),
                     ("H0-ref", min(high, max(low, 0.0)), 0.0))
    else:
        azimuth = atan2(y, x)
        shoulders = (("H+", azimuth, radial), ("H-", azimuth + pi, -radial))

    results = []
    labels = {}
    for shoulder, q1, signed_radius in shoulders:
        for sign, elbow in ((1, "C+"), (-1, "C-")):
            q2 = (atan2(signed_radius, z)
                  - atan2(sign * sine * forearm, upper_arm + cosine * forearm))
            q3 = atan2(sign * sine, cosine) - offset
            arm = _multiply(_rotation((0, 0, 1), q1), _rotation((0, 1, 0), q2 + q3))
            wrist = _multiply(_inverse(arm), flange)
            for label, wrist_singular, angles in _wrist(wrist, limits, seed):
                choices = tuple(_turns(angle, bound) for angle, bound in zip(
                    (q1, q2, q3) + angles, limits
                ))
                branch = f"{shoulder}/{elbow}/{label}"
                for q in product(*choices):
                    if any(hypot(*(a - b for a, b in zip(q, item.q))) < 1e-7
                           for item in results):
                        continue
                    error = _pose_error(target, model.fk(q))
                    position, orientation = hypot(*error[:3]), hypot(*error[3:])
                    if position <= 1e-6 and orientation <= 1e-6:
                        labels[branch] = labels.get(branch, 0) + 1
                        results.append(IKCandidate(
                            q, f"{branch}/{labels[branch]}", position, orientation,
                            radial <= 1e-10 or sine <= 1e-7 or wrist_singular,
                        ))
    results.sort(key=lambda item: hypot(*(a - b for a, b in zip(item.q, seed))))
    return tuple(results)
