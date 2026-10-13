"""Cinematica inversa analitica por Pieper del ABB IRB 120 (modelo IMT-342).

Convencion: DH estandar; theta_2 = q_2 - pi/2; unidades SI y radianes.
No utiliza solucionadores numericos de IK.

Alcance: enumera ramas regulares de brazo y muneca, con representantes
particulares de familias singulares. No impone limites articulares, ni colisiones.
"""

from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt

import numpy as np

from .irb120_fk import forward_kinematics

# Geometria en metros del modelo validado en el Hito 1.
D1 = 0.290
A2 = 0.270
A3 = 0.070
D4 = 0.302
D6 = 0.072
TCP_Z6 = 0.120


@dataclass
class PieperSolution:
    """Candidata que reconstruye la pose objetivo, pendiente de limites fisicos."""

    q: np.ndarray
    arm_branch: str
    wrist_branch: str
    position_error_m: float
    orientation_error_rad: float
    singular: bool = False


@dataclass
class PieperResult:
    """Ramas matematicas y diagnosticos de la cinemática inversa."""

    solutions: list[PieperSolution]
    wrist_center_m: np.ndarray
    diagnostics: list[str]


def _wrap(q):
    """Angulos representativos en [-pi, pi)."""
    return (np.asarray(q, dtype=float) + pi) % (2 * pi) - pi


def _rotation03(q1, q2, q3):
    """R03 derivada de las tres primeras matrices DH del Hito 1."""
    c1, s1 = cos(q1), sin(q1)
    c23, s23 = cos(q2 + q3), sin(q2 + q3)
    return np.array([
        [c1 * s23, s1, c1 * c23],
        [s1 * s23, -c1, s1 * c23],
        [c23, 0.0, -s23],
    ])


def _orientation_error_rad(R_calculated, R_target):
    """Angulo SO(3) estable cerca de cero y de pi."""
    E = R_target.T @ R_calculated
    v = np.array([
        E[2, 1] - E[1, 2],
        E[0, 2] - E[2, 0],
        E[1, 0] - E[0, 1],
    ])
    sin_angle = 0.5 * np.linalg.norm(v)
    cos_angle = 0.5 * (np.trace(E) - 1.0)
    return float(atan2(sin_angle, cos_angle))


def _check_pose(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError("La pose debe ser una matriz finita 4x4.")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10):
        raise ValueError("La ultima fila debe ser [0, 0, 0, 1].")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-8) or not np.isclose(
        np.linalg.det(R), 1.0, atol=1e-8
    ):
        raise ValueError("La orientacion debe pertenecer a SO(3).")
    return T


def solve_pieper(
    T_target,
    *,
    target_frame="flange",
    q_seed=None,
    position_tolerance_m=1e-8,
    orientation_tolerance_rad=1e-7,
    singularity_tolerance=1e-9,
):
    """Resuelve IK de Pieper para la brida o el TCP academico del IRB 120.

    Parametros:
        T_target: pose 4x4 en el marco base (brida o TCP).
        target_frame: 'flange' o 'tcp'.
        q_seed: referencia opcional en radianes para elegir representantes
                singulares de q1 o de la muneca; NO filtra soluciones.

    Devuelve:
        PieperResult con ramas que superan FK. En singularidades se retorna
        un representante de la familia, no todas las soluciones infinitas.
        No filtra limites articulares, colisiones ni velocidades.
    """
    if target_frame not in ("flange", "tcp"):
        raise ValueError("target_frame debe ser 'flange' o 'tcp'.")
    for name, value in (
        ("position_tolerance_m", position_tolerance_m),
        ("orientation_tolerance_rad", orientation_tolerance_rad),
        ("singularity_tolerance", singularity_tolerance),
    ):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} debe ser positivo y finito.")

    if q_seed is None:
        seed = np.zeros(6)
    else:
        seed = np.asarray(q_seed, dtype=float)
        if seed.shape != (6,) or not np.all(np.isfinite(seed)):
            raise ValueError("q_seed debe contener seis angulos finitos.")

    T = _check_pose(T_target).copy()
    if target_frame == "tcp":
        # La herramienta academica tiene traslacion local +0.120 m en z6.
        T[:3, 3] -= TCP_Z6 * T[:3, 2]

    R_d, p_d = T[:3, :3], T[:3, 3]
    p_wc = p_d - D6 * R_d[:, 2]
    xw, yw, zw = p_wc
    rho = float(np.hypot(xw, yw))
    h = zw - D1

    L = float(np.hypot(A3, D4))
    beta = atan2(D4, A3)
    D_squared = rho**2 + h**2
    c_delta = (D_squared - A2**2 - L**2) / (2 * A2 * L)
    diagnostics = []
    if c_delta < -1 - singularity_tolerance or c_delta > 1 + singularity_tolerance:
        diagnostics.append("Centro de muneca fuera del alcance geometrico de J2-J3.")
        return PieperResult([], p_wc, diagnostics)
    c_delta = float(np.clip(c_delta, -1, 1))
    s_delta_abs = sqrt(max(0.0, 1 - c_delta**2))
    if s_delta_abs <= singularity_tolerance:
        diagnostics.append("Ramas de codo coincidentes: triangulo degenerado.")

    if rho <= singularity_tolerance:
        # La posicion no determina q1: seleccionar un representante.
        base_branches = [(float(seed[0]), 0.0, "base_indeterminada")]
        diagnostics.append(
            "Centro sobre el eje J1: infinitas direcciones base; "
            "se usa un representante q1 tomado de q_seed (o cero)."
        )
    else:
        q1 = atan2(yw, xw)
        base_branches = [
            (q1, rho, "base_directa"),
            (q1 + pi, -rho, "base_opuesta"),
        ]

    solutions = []
    for q1, r, base_name in base_branches:
        for elbow_sign in (1, -1):
            delta = atan2(elbow_sign * s_delta_abs, c_delta)
            q2 = atan2(r, h) - atan2(L * sin(delta), A2 + L * cos(delta))
            q3 = delta - beta
            arm_name = f"{base_name}/codo_{'+' if elbow_sign > 0 else '-'}"
            R36 = _rotation03(q1, q2, q3).T @ R_d
            s5_abs = float(np.hypot(R36[0, 2], R36[1, 2]))

            if s5_abs <= singularity_tolerance:
                q4 = float(seed[3])
                if R36[2, 2] >= 0:
                    q5 = 0.0
                    combined = atan2(R36[1, 0], R36[0, 0])
                    q6 = combined - q4
                    wrist_name = "muneca_singular_q5_0"
                else:
                    q5 = pi
                    difference = atan2(-R36[1, 0], -R36[0, 0])
                    q6 = q4 - difference
                    wrist_name = "muneca_singular_q5_pi"
                candidates = [(q4, q5, q6, wrist_name, True)]
                diagnostics.append(
                    f"{arm_name}: muneca singular; se devuelve un representante "
                    "de una familia continua."
                )
            else:
                candidates = []
                for wrist_sign in (1, -1):
                    s5 = wrist_sign * s5_abs
                    q5 = atan2(s5, R36[2, 2])
                    q4 = atan2(-R36[1, 2] / s5, -R36[0, 2] / s5)
                    q6 = atan2(-R36[2, 1] / s5, R36[2, 0] / s5)
                    wrist_name = f"muneca_{'+' if wrist_sign > 0 else '-'}"
                    candidates.append((q4, q5, q6, wrist_name, False))

            for q4, q5, q6, wrist_name, is_singular in candidates:
                q = _wrap([q1, q2, q3, q4, q5, q6])
                T_check = forward_kinematics(q)
                e_p = float(np.linalg.norm(T_check[:3, 3] - p_d))
                e_R = _orientation_error_rad(T_check[:3, :3], R_d)
                if e_p > position_tolerance_m or e_R > orientation_tolerance_rad:
                    diagnostics.append(
                        f"{arm_name}/{wrist_name}: fallo FK "
                        f"(pos={e_p:.3e} m, rot={e_R:.3e} rad)."
                    )
                    continue
                if any(
                    np.allclose(_wrap(q - item.q), 0, atol=singularity_tolerance, rtol=0)
                    for item in solutions
                ):
                    continue
                solutions.append(
                    PieperSolution(q, arm_name, wrist_name, e_p, e_R, is_singular)
                )

    return PieperResult(solutions, p_wc, diagnostics)
