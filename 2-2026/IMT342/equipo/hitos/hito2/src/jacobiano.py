
"""Jacobiano geométrico del ABB IRB 120 según el modelo DH del Hito 1.

Convenciones:
- Jacobiano 6x6, con velocidad lineal arriba y angular abajo.
- Resultados expresados en el marco base {0}.
- Cada columna corresponde a una articulación revoluta.
- Se admite como punto efector la brida {6} o el TCP académico.

Referencia: material oficial S10 de IMT-342, Hito 2.
"""

import numpy as np

from .dh import dh_matrix
from .irb120_fk import irb120_dh_params, tcp_transform


def _vector_articular(q, nombre="q"):
    """Valida un vector de seis componentes numéricas finitas."""
    try:
        vector = np.asarray(q, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{nombre} debe contener seis valores numéricos.") from exc

    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{nombre} debe ser un vector finito de seis componentes.")

    return vector


def transformaciones_intermedias(q):
    """Obtiene T00, T01, ..., T06 mediante DH estándar.

    Todas las transformaciones quedan expresadas respecto a la base.
    El offset angular de J2 se incorpora desde irb120_dh_params().
    """
    q = _vector_articular(q)

    acumulada = np.eye(4)
    resultado = [acumulada.copy()]

    for theta, d, a, alpha in irb120_dh_params(q):
        acumulada = acumulada @ dh_matrix(theta, d, a, alpha)
        resultado.append(acumulada.copy())

    return tuple(resultado)


def jacobiano_geometrico(q, *, target_frame="flange"):
    """Calcula el Jacobiano geométrico 6x6 de la brida o el TCP.

    Para cada articulación revoluta:
        Jv_i = z_(i-1) x (p_e - o_(i-1))
        Jw_i = z_(i-1)

    Todas las posiciones y direcciones se expresan en la base {0}.
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
    """Calcula la velocidad lineal y angular del efector en la base."""
    velocidades = _vector_articular(qdot, nombre="qdot")
    return jacobiano_geometrico(q, target_frame=target_frame) @ velocidades
