"""Shared data contracts for the control backends."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite

from imt342_irb120_planning import JOINT_LIMITS


class Result(Enum):
    IDLE = "idle"
    EXECUTING = "executing"
    VERIFIED = "verified"
    UNKNOWN = "unknown"
    ACCEPTED = "accepted"
    SIMULATED_COMPLETE = "simulated_complete"
    TRANSMITTED = "transmitted"
    CANCELLED = "cancelled"
    FAILED = "failed"


@dataclass(frozen=True)
class Feedback:
    """Joint feedback in radians with a local monotonic arrival timestamp."""

    positions: tuple[float, ...]
    received_at: float
    source: str
    sequence: int | None = None


def finite_positions(values):
    positions = tuple(float(value) for value in values)
    if len(positions) != 6 or not all(isfinite(value) for value in positions):
        raise ValueError("Se requieren seis valores articulares finitos")
    return positions


def validate_target(values):
    positions = finite_positions(values)
    if any(
        not lower <= value <= upper
        for value, (lower, upper) in zip(positions, JOINT_LIMITS)
    ):
        raise ValueError("Objetivo fuera del límite articular")
    return positions
