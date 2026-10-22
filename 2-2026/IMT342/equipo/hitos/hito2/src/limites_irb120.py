
"""Límites articulares nominales del ABB IRB 120.

Fuentes ABB (revisión W):
- Especificaciones del producto, 3HAC035960-005, sección 1.8.1.
- Manual del producto, 3HAC035728-005, sección 2.2.2.

Convenciones:
- Entradas articulares en radianes.
- Límites documentados en grados.
- Orden de articulaciones: J1, J2, J3, J4, J5, J6.

Los límites nominales no verifican colisiones ni certifican la
configuración o seguridad de un controlador específico.
"""

from math import ceil, floor, pi

import numpy as np


NOMBRES_JUNTAS = ("J1", "J2", "J3", "J4", "J5", "J6")

# Límites mínimo y máximo de cada articulación, en grados.
LIMITES_ABB_GRADOS = np.array(
    [
        [-165.0, 165.0],
        [-110.0, 110.0],
        [-110.0, 70.0],
        [-160.0, 160.0],
        [-120.0, 120.0],
        [-400.0, 400.0],
    ],
    dtype=float,
)

# Evitar modificaciones accidentales de la tabla compartida.
LIMITES_ABB_GRADOS.setflags(write=False)


def _vector_articular(q):
    """Valida y devuelve una copia de seis ángulos finitos en radianes."""
    try:
        q = np.asarray(q, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError("q debe contener seis angulos numericos.") from exc

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("q debe contener seis angulos finitos en radianes.")

    return q.copy()


def violaciones_limites(q, tolerancia_rad=1e-10):
    """Informa las articulaciones que exceden los límites nominales ABB.

    Los ángulos se comparan sin normalizar para conservar las vueltas
    permitidas de J6. La tolerancia es numérica, no de seguridad.
    """
    q = _vector_articular(q)

    if not np.isfinite(tolerancia_rad) or tolerancia_rad < 0:
        raise ValueError("tolerancia_rad debe ser finita y no negativa.")

    limites_rad = np.deg2rad(LIMITES_ABB_GRADOS)
    errores = []

    for i, valor in enumerate(q):
        minimo, maximo = limites_rad[i]

        if valor < minimo - tolerancia_rad or valor > maximo + tolerancia_rad:
            errores.append(
                {
                    "junta": NOMBRES_JUNTAS[i],
                    "angulo_grados": float(np.rad2deg(valor)),
                    "minimo_grados": float(LIMITES_ABB_GRADOS[i, 0]),
                    "maximo_grados": float(LIMITES_ABB_GRADOS[i, 1]),
                }
            )

    return errores


def variantes_equivalentes_j6(q):
    """Genera vueltas equivalentes de J6 dentro de su rango nominal.

    Calcula q6 + 2*k*pi manteniendo J1...J5 sin cambios.
    No filtra las demás articulaciones ni selecciona una trayectoria.
    """
    q = _vector_articular(q)

    minimo, maximo = np.deg2rad(LIMITES_ABB_GRADOS[5])
    dos_pi = 2 * pi

    # Tolerancia pequeña para no perder variantes por redondeo numérico.
    epsilon = 1e-12

    k_min = ceil((minimo - q[5]) / dos_pi - epsilon)
    k_max = floor((maximo - q[5]) / dos_pi + epsilon)

    variantes = []

    for k in range(k_min, k_max + 1):
        candidato = q.copy()
        candidato[5] += k * dos_pi

        if minimo - epsilon <= candidato[5] <= maximo + epsilon:
            variantes.append(candidato)

    return variantes
