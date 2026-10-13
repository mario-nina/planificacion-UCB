"""Filtrado de Pieper por rangos nominales ABB y seleccion para LSPB.

Fuentes para rangos (definidas en limites_irb120.py):
- ABB IRB 120, Especificaciones del producto 3HAC035960-005 rev. W,
  seccion 1.8.1, pagina impresa 37.
- ABB IRB 120, Manual del producto 3HAC035728-005 rev. W,
  seccion 2.2.2, pagina impresa 50.

Convencion: q1..q6 en RADIANES, como en el modelo DH del Hito 1.
Este modulo NO modifica el solucionador analitico, ni certifica colisiones,
seguridad del controlador, continuidad de trayectorias o alcanzabilidad dinamica.
Las soluciones singulares de Pieper son representantes, no familias exhaustivas.
"""

from dataclasses import dataclass

import numpy as np

from .limites_irb120 import variantes_equivalentes_j6, violaciones_limites
from .pieper import PieperResult


@dataclass
class SolucionAdmisible:
    """Una configuracion articular dentro de limites, incluso su vuelta J6."""

    q: np.ndarray
    rama_brazo: str
    rama_muneca: str
    error_posicion_m: float
    error_orientacion_rad: float
    singular: bool
    costo: float


@dataclass
class SolucionDescartada:
    """Una rama matematica cuya representacion excede limites nominales."""

    q: np.ndarray
    rama_brazo: str
    rama_muneca: str
    violaciones: list[dict]


@dataclass
class ResultadoSeleccion:
    """Salida sin perder las ramas descartadas ni el diagnostico de Pieper."""

    admisibles: list[SolucionAdmisible]
    descartadas: list[SolucionDescartada]
    seleccionada: SolucionAdmisible | None
    diagnosticos: list[str]


def _vector_seis(nombre, valores):
    try:
        q = np.asarray(valores, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{nombre} debe ser un vector numerico de seis elementos.") from exc
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"{nombre} debe contener seis valores finitos.")
    return q.copy()


def filtrar_y_seleccionar_pieper(resultado, q_inicio, *, pesos=None):
    """Filtra ramas FK-validas de Pieper y elige la de menor movimiento ponderado.

    `resultado` se obtiene de solve_pieper(T_objetivo, q_seed=q_inicio).
    `q_inicio` tiene seis angulos en radianes y debe estar en limites ABB.
    `pesos` son factores no negativos, por defecto 1 en cada articulacion;
    al menos uno debe ser positivo. El costo es sum(w_i * (q_i - q_inicio_i)^2).
    Se prueba cada representacion permitida de J6 separada por 2*pi.

    El costo mide desplazamiento articular estimado y NO prueba seguridad,
    factibilidad LSPB, colisiones ni velocidad/aceleracion maxima.
    """
    if not isinstance(resultado, PieperResult):
        raise TypeError("resultado debe ser la salida PieperResult de solve_pieper.")

    inicial = _vector_seis("q_inicio", q_inicio)
    if violaciones_limites(inicial):
        raise ValueError("q_inicio excede los limites nominales ABB del IRB 120.")

    if pesos is None:
        w = np.ones(6, dtype=float)
    else:
        w = _vector_seis("pesos", pesos)
        if np.any(w < 0) or not np.any(w > 0):
            raise ValueError("pesos debe ser no negativo y contener algun valor positivo.")

    admisibles = []
    descartadas = []
    diagnosticos = list(resultado.diagnostics)

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

        # El solver entrega q6 normalizado a [-pi,pi); se consideran
        # las vueltas equivalentes dentro de los +-400 grados nominales.
        variantes = variantes_equivalentes_j6(q)
        if not variantes:
            descartadas.append(
                SolucionDescartada(
                    q=q,
                    rama_brazo=solucion.arm_branch,
                    rama_muneca=solucion.wrist_branch,
                    violaciones=[{"junta": "J6", "motivo": "Ninguna vuelta equivalente dentro de limites ABB."}],
                )
            )
            continue

        for variante in variantes:
            # Defensa adicional: el modulo de variantes no filtra J1..J5.
            if violaciones_limites(variante):
                raise RuntimeError("Error interno: variante fuera de limites ABB.")
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

    # Orden determinista; desempate por rama y valor de J6.
    admisibles.sort(key=lambda s: (s.costo, s.rama_brazo, s.rama_muneca, float(s.q[5])))
    elegida = admisibles[0] if admisibles else None
    if elegida is None:
        diagnosticos.append("No se encontro una candidata dentro de los limites nominales ABB.")
    if any(s.singular for s in admisibles):
        diagnosticos.append(
            "Las soluciones de muneca singular son solo representantes; "
            "el filtro no explora toda la familia continua."
        )
    return ResultadoSeleccion(admisibles, descartadas, elegida, diagnosticos)
