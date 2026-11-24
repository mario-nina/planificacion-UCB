"""Single rigid link teaching model with explicit SI parameters."""

from dataclasses import dataclass
from math import isfinite, sin


@dataclass(frozen=True)
class SingleLinkDynamics:
    """Inertia about the joint, with angle zero at hanging equilibrium."""

    inertia_kg_m2: float
    mass_kg: float
    com_radius_m: float
    gravity_m_s2: float = 9.81

    def __post_init__(self):
        if not all(isfinite(value) and value > 0 for value in (
            self.inertia_kg_m2, self.mass_kg, self.com_radius_m, self.gravity_m_s2
        )):
            raise ValueError("Los parámetros físicos 1R deben ser positivos y finitos")

    def components(self, q_rad: float, qddot_rad_s2: float) -> tuple[float, float]:
        """Return inertial and gravitational torque respectively in N·m."""
        if not all(isfinite(value) for value in (q_rad, qddot_rad_s2)):
            raise ValueError("La posición y la aceleración 1R deben ser finitas")
        return (
            self.inertia_kg_m2 * qddot_rad_s2,
            self.mass_kg * self.gravity_m_s2 * self.com_radius_m * sin(q_rad),
        )

    def torque(self, q_rad: float, qddot_rad_s2: float) -> float:
        return sum(self.components(q_rad, qddot_rad_s2))
