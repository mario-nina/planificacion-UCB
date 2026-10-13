"""Limites articulares nominales del ABB IRB 120.

Fuentes ABB (revisiones W):
- Especificaciones del producto, 3HAC035960-005, sec. 1.8.1, p. 37.
- Manual del producto, 3HAC035728-005, sec. 2.2.2, p. 50.

Los limites son rangos nominales de movimiento, no certifican ausencia de
colisiones ni representan la configuracion de un controlador especifico.
Los vectores articulares usan q1..q6 en RADIANES, en la convencion del Hito 1.
"""

from math import ceil, floor, pi

import numpy as np

NOMBRES_JUNTAS = ("J1", "J2", "J3", "J4", "J5", "J6")
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
LIMITES_ABB_GRADOS.setflags(write=False)


def _vector_articular(q):
    """Devuelve copia de un vector articular valido, o produce ValueError."""
    try:
        q = np.asarray(q, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError("q debe contener seis angulos numericos.") from exc
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("q debe contener seis angulos finitos en radianes.")
    return q.copy()


def violaciones_limites(q, tolerancia_rad=1e-10):
    """Lista de dicts con juntas que exceden rangos nominales ABB.

    Se compara SIN normalizar los angulos: q6=390 grados es valido,
    aunque se encuentre fuera del intervalo [-pi, pi).
    La tolerancia es solo numerica, no un margen de seguridad.
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
    """Genera q6 + 2*k*pi en el rango nominal ABB; J1..J5 intactas.

    Retorna lista de vectores de longitud 6. NO filtra otras juntas,
    ni selecciona una trayectoria. Si ninguna vuelta cabe en J6, [] .
    No asume extensiones de giro por configuracion del controlador.
    """
    q = _vector_articular(q)
    minimo, maximo = np.deg2rad(LIMITES_ABB_GRADOS[5])
    dos_pi = 2 * pi
    # Redondeo para evitar perder una variante exacta por ruido de punto flotante.
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
