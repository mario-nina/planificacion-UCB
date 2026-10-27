
"""Filtrado y selección de soluciones de Pieper para el ABB IRB 120.

Se utilizan los límites nominales ABB definidos en limites_irb120.py.

El módulo descarta configuraciones fuera de rango y selecciona la de
menor desplazamiento articular ponderado respecto a q_inicio.

Convenciones: seis articulaciones, ángulos en radianes.
La selección no garantiza ausencia de colisiones ni factibilidad dinámica.
"""

from dataclasses import dataclass

import numpy as np

from .limites_irb120 import variantes_equivalentes_j6, violaciones_limites
from .pieper import PieperResult


@dataclass
class SolucionAdmisible:
    """Configuración dentro de límites ABB, con costo de desplazamiento."""

    q: np.ndarray
    rama_brazo: str
    rama_muneca: str
    error_posicion_m: float
    error_orientacion_rad: float
    singular: bool
    costo: float


@dataclass
class SolucionDescartada:
    """Configuración rechazada por incumplir los límites nominales."""

    q: np.ndarray
    rama_brazo: str
    rama_muneca: str
    violaciones: list[dict]


@dataclass
class ResultadoSeleccion:
    """Resultado completo del filtrado y de la selección articular."""

    admisibles: list[SolucionAdmisible]
    descartadas: list[SolucionDescartada]
    seleccionada: SolucionAdmisible | None
    diagnosticos: list[str]


def _vector_seis(nombre, valores):
    """Convierte y valida un vector numérico de seis elementos."""
    try:
        q = np.asarray(valores, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{nombre} debe ser un vector numerico de seis elementos."
        ) from exc

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"{nombre} debe contener seis valores finitos.")

    return q.copy()


def filtrar_y_seleccionar_pieper(resultado, q_inicio, *, pesos=None):
    """Filtra soluciones de Pieper y selecciona la de menor costo.

    resultado : PieperResult
        Resultado de la cinemática inversa, previamente verificado por FK.
    q_inicio : array-like
        Configuración inicial de seis ángulos, en radianes.
    pesos : array-like, opcional
        Ponderación no negativa de las articulaciones. Por defecto,
        todas tienen peso 1 y al menos un peso debe ser positivo.

    Criterio: C(q) = sum(w_i * (q_i - q_inicio_i)**2).

    Retorna un ResultadoSeleccion con candidatas admisibles,
    descartadas, selección final y mensajes de diagnóstico.
    """
    if not isinstance(resultado, PieperResult):
        raise TypeError(
            "resultado debe ser la salida PieperResult de solve_pieper."
        )

    inicial = _vector_seis("q_inicio", q_inicio)

    if violaciones_limites(inicial):
        raise ValueError(
            "q_inicio excede los limites nominales ABB del IRB 120."
        )

    if pesos is None:
        w = np.ones(6, dtype=float)
    else:
        w = _vector_seis("pesos", pesos)
        if np.any(w < 0) or not np.any(w > 0):
            raise ValueError(
                "pesos debe ser no negativo y contener algun valor positivo."
            )

    admisibles = []
    descartadas = []
    diagnosticos = list(resultado.diagnostics)

    # Examinar cada rama matemática encontrada por Pieper.
    for solucion in resultado.solutions:
        q = _vector_seis("solucion.q", solucion.q)
        violaciones = violaciones_limites(q)

        if violaciones:
            descartadas.append(
                SolucionDescartada(
                    q=q,
                    rama_brazo=solucion.arm_branch,
                    rama_muneca=solucion.wrist_branch,
                    violaciones=violaciones,
                )
            )
            continue

        # Considerar vueltas equivalentes de J6 dentro de sus límites ABB.
        variantes = variantes_equivalentes_j6(q)

        if not variantes:
            descartadas.append(
                SolucionDescartada(
                    q=q,
                    rama_brazo=solucion.arm_branch,
                    rama_muneca=solucion.wrist_branch,
                    violaciones=[
                        {
                            "junta": "J6",
                            "motivo": "Ninguna vuelta equivalente dentro de limites ABB.",
                        }
                    ],
                )
            )
            continue

        for variante in variantes:

            # Comprobación defensiva antes de calcular el costo.
            if violaciones_limites(variante):
                raise RuntimeError(
                    "Error interno: variante fuera de limites ABB."
                )

            # Costo de desplazamiento articular ponderado.
            costo = float(np.sum(w * np.square(variante - inicial)))

            admisibles.append(
                SolucionAdmisible(
                    q=variante.copy(),
                    rama_brazo=solucion.arm_branch,
                    rama_muneca=solucion.wrist_branch,
                    error_posicion_m=float(solucion.position_error_m),
                    error_orientacion_rad=float(solucion.orientation_error_rad),
                    singular=bool(solucion.singular),
                    costo=costo,
                )
            )

    # Ordenar por costo; resolver empates de forma reproducible.
    admisibles.sort(
        key=lambda s: (
            s.costo,
            s.rama_brazo,
            s.rama_muneca,
            float(s.q[5]),
        )
    )

    elegida = admisibles[0] if admisibles else None

    if elegida is None:
        diagnosticos.append(
            "No se encontro una candidata dentro de los limites nominales ABB."
        )

    if any(s.singular for s in admisibles):
        diagnosticos.append(
            "Las soluciones de muneca singular son solo representantes; "
            "el filtro no explora toda la familia continua."
        )

    return ResultadoSeleccion(
        admisibles,
        descartadas,
        elegida,
        diagnosticos,
    )
