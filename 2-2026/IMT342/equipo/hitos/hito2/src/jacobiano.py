"""Jacobiano geométrico 6R del ABB IRB 120 con D-H estándar del Hito 1.

Fundamento docente: IMT-342, S10, hito_2_guia.pdf (pp. 31-32
    del consolidado), y w10_s01_practica.pdf (pp. 13-14).
Convenciones:
    - J de 6x6, filas [velocidad lineal (m/s); angular (rad/s)].
    - Ambas velocidades expresadas en el marco base {0}.
    - Cada columna corresponde a una junta revoluta física q_i.
    - Soporta efector en brida D-H {6} o TCP académico del Hito 1.
    - No estima velocidades ni comprueba singularidades por sí solo.
"""

import numpy as np

from .dh import dh_matrix
from .irb120_fk import irb120_dh_params, tcp_transform


def _vector_articular(q, nombre="q"):
    try:
        vector = np.asarray(q, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{nombre} debe contener seis valores numéricos.") from exc
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{nombre} debe ser un vector finito de seis componentes.")
    return vector


def transformaciones_intermedias(q):
    """Retorna (0T0, 0T1, ..., 0T6), calculadas con D-H estándar.

    Las transformaciones se expresan todas respecto a la base {0}.
    El offset del segundo eje está incluido por irb120_dh_params(q).
    """
    q = _vector_articular(q)
    acumulada = np.eye(4)
    resultado = [acumulada.copy()]
    for theta, d, a, alpha in irb120_dh_params(q):
        acumulada = acumulada @ dh_matrix(theta, d, a, alpha)
        resultado.append(acumulada.copy())
    return tuple(resultado)


def jacobiano_geometrico(q, *, target_frame="flange"):
    """Calcula 0J de brida o TCP, orden [lineal; angular], dimensión 6x6.

    Columna i de cada junta revoluta:
       Jv_i = 0z_(i-1) x (0p_e - 0o_(i-1))
       Jw_i = 0z_(i-1)

    El resultado no convierte velocidades a coordenadas del efector.
    """
    if target_frame not in ("flange", "tcp"):
        raise ValueError("target_frame debe ser 'flange' o 'tcp'.")
    transformaciones = transformaciones_intermedias(q)
    t_final = transformaciones[6]
    if target_frame == "tcp":
        t_final = t_final @ tcp_transform()
    p_efector = t_final[:3, 3]

    jacobiano = np.empty((6, 6), dtype=float)
    for i in range(6):
        t_anterior = transformaciones[i]
        z_anterior = t_anterior[:3, 2]
        o_anterior = t_anterior[:3, 3]
        jacobiano[:3, i] = np.cross(z_anterior, p_efector - o_anterior)
        jacobiano[3:, i] = z_anterior
    return jacobiano


def velocidad_cartesiana(q, qdot, *, target_frame="flange"):
    """Retorna [0v_e; 0omega_e] = 0J(q) qdot en el marco base {0}."""
    velocidades = _vector_articular(qdot, nombre="qdot")
    return jacobiano_geometrico(q, target_frame=target_frame) @ velocidades
