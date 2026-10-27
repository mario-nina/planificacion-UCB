
"""Planificación de trayectorias articulares mediante perfiles LSPB.

Genera movimientos de reposo a reposo para una o seis articulaciones,
con un tiempo final común.

Referencia académica: material oficial S09, W09-S01 de IMT-342.
Velocidades nominales: especificación ABB IRB 120, 3HAC035960-005.

Convenciones:
- Posiciones en radianes.
- Velocidades en rad/s.
- Aceleraciones en rad/s².
- Tiempo en segundos.

Las aceleraciones son parámetros elegidos para la planificación,
no límites certificados por ABB. No se comprueban colisiones ni torque.
"""

from dataclasses import dataclass

import numpy as np

from .limites_irb120 import violaciones_limites


# Velocidades articulares nominales del IRB 120, en grados/s.
VELOCIDADES_ABB_GRADOS_S = np.array(
    [250.0, 250.0, 250.0, 320.0, 320.0, 420.0],
    dtype=float,
)
VELOCIDADES_ABB_GRADOS_S.setflags(write=False)

# El planificador trabaja internamente con radianes.
VELOCIDADES_ABB_RAD_S = np.deg2rad(VELOCIDADES_ABB_GRADOS_S)
VELOCIDADES_ABB_RAD_S.setflags(write=False)


@dataclass
class MuestraLSPB:
    """Posición, velocidad y aceleración evaluadas en uno o varios tiempos."""

    q: np.ndarray
    qdot: np.ndarray
    qddot: np.ndarray


@dataclass
class PerfilLSPB:
    """Parámetros de una trayectoria LSPB para una o seis articulaciones."""

    q_inicio: np.ndarray
    q_final: np.ndarray
    tf: float
    aceleraciones: np.ndarray
    tc: np.ndarray
    v_crucero: np.ndarray
    velocidades_maximas: np.ndarray

    def evaluar(self, t):
        """Evalúa q, qdot y qddot dentro del intervalo [0, tf].

        Para un tiempo devuelve vectores (n,).
        Para varios tiempos devuelve matrices (N, n).
        La aceleración ideal es discontinua entre fases.
        """
        try:
            tiempos = np.asarray(t, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "t debe ser un tiempo o array numerico."
            ) from exc

        if tiempos.ndim > 1 or not np.all(np.isfinite(tiempos)):
            raise ValueError(
                "t debe ser escalar o vector unidimensional finito."
            )

        escalar = tiempos.ndim == 0
        tiempos = np.atleast_1d(tiempos)

        if np.any(tiempos < 0) or np.any(tiempos > self.tf):
            raise ValueError("Los tiempos deben pertenecer a [0, tf].")

        n = self.q_inicio.size

        # Reservar espacio para los valores de cada articulación.
        q = np.empty((tiempos.size, n), dtype=float)
        qdot = np.empty_like(q)
        qddot = np.empty_like(q)

        for i in range(n):
            qi, qf = self.q_inicio[i], self.q_final[i]
            tc, vc = self.tc[i], self.v_crucero[i]

            # La aceleración lleva el signo del movimiento.
            a = np.sign(qf - qi) * self.aceleraciones[i]

            # Identificar las muestras de las tres fases del perfil.
            ta = tiempos <= tc
            tfren = (tiempos >= self.tf - tc) & ~ta
            tconst = ~(ta | tfren)

            # Fase 1: aceleración constante.
            q[ta, i] = qi + 0.5 * a * tiempos[ta] ** 2
            qdot[ta, i] = a * tiempos[ta]
            qddot[ta, i] = a

            # Fase 2: velocidad constante.
            q[tconst, i] = qi + vc * (tiempos[tconst] - tc / 2)
            qdot[tconst, i] = vc
            qddot[tconst, i] = 0.0

            # Fase 3: desaceleración hasta alcanzar q_final.
            restante = self.tf - tiempos[tfren]
            q[tfren, i] = qf - 0.5 * a * restante ** 2
            qdot[tfren, i] = a * restante
            qddot[tfren, i] = -a

        if escalar:
            return MuestraLSPB(q[0], qdot[0], qddot[0])

        return MuestraLSPB(q, qdot, qddot)


def _vector(nombre, valor):
    """Valida un vector de una o seis componentes numéricas finitas."""
    try:
        vector = np.asarray(valor, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{nombre} debe ser vector numerico."
        ) from exc

    if (
        vector.ndim != 1
        or vector.size not in (1, 6)
        or not np.all(np.isfinite(vector))
    ):
        raise ValueError(
            f"{nombre} debe tener uno o seis valores finitos."
        )

    return vector.copy()


def planificar_lspb(
    q_inicio,
    q_final,
    tf,
    aceleraciones,
    *,
    velocidades_maximas=None,
):
    """Construye un perfil LSPB de reposo a reposo.

    Acepta una articulación (1R) o las seis articulaciones (6R).
    Las aceleraciones son magnitudes positivas en rad/s².

    Para 6R comprueba los límites nominales ABB de posición y
    velocidad. Las velocidades_maximas opcionales pueden añadir
    restricciones más estrictas.

    Retorna un PerfilLSPB con los parámetros de la trayectoria.
    """
    inicio = _vector("q_inicio", q_inicio)
    final = _vector("q_final", q_final)
    aceleraciones = _vector("aceleraciones", aceleraciones)

    if inicio.shape != final.shape or inicio.shape != aceleraciones.shape:
        raise ValueError(
            "Las dimensiones de q_inicio, q_final y aceleraciones deben coincidir."
        )

    if np.any(aceleraciones <= 0):
        raise ValueError(
            "Las aceleraciones de planificacion deben ser positivas."
        )

    try:
        tf = float(tf)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(
            "tf debe ser un escalar positivo finito."
        ) from exc

    if not np.isfinite(tf) or tf <= 0:
        raise ValueError("tf debe ser un escalar positivo finito.")

    n = inicio.size

    if n == 6:
        # Para el robot 6R, comprobar los límites nominales ABB.
        if violaciones_limites(inicio) or violaciones_limites(final):
            raise ValueError(
                "Las posiciones inicial y final deben cumplir limites nominales ABB."
            )

        vmax = VELOCIDADES_ABB_RAD_S.copy()
    else:
        # El caso didáctico 1R no recibe límites de velocidad ABB.
        vmax = np.full(n, np.inf)

    if velocidades_maximas is not None:
        cotas = _vector("velocidades_maximas", velocidades_maximas)

        if cotas.shape != inicio.shape or np.any(cotas <= 0):
            raise ValueError(
                "Las velocidades_maximas deben tener la misma dimension y ser positivas."
            )

        vmax = np.minimum(vmax, cotas)

    desplazamientos = np.abs(final - inicio)

    # Factibilidad LSPB: a >= 4*abs(q_final-q_inicio)/tf².
    fraccion = 4.0 * desplazamientos / (aceleraciones * tf * tf)

    if not np.all(np.isfinite(fraccion)) or np.any(fraccion > 1.0 + 1e-12):
        raise ValueError(
            "Aceleracion insuficiente para la duracion tf (LSPB inviable)."
        )

    # Calcular los tiempos de mezcla de forma numéricamente estable.
    fraccion = np.minimum(fraccion, 1.0)
    raiz = np.sqrt(1.0 - fraccion)
    tc = (2.0 * desplazamientos / aceleraciones) / (
        tf * (1.0 + raiz)
    )

    # La velocidad de crucero conserva el sentido del desplazamiento.
    v_crucero = np.sign(final - inicio) * aceleraciones * tc

    if np.any(np.abs(v_crucero) > vmax + 1e-12):
        raise ValueError(
            "La velocidad de crucero excede los limites configurados o nominales ABB."
        )

    return PerfilLSPB(
        q_inicio=inicio,
        q_final=final,
        tf=tf,
        aceleraciones=aceleraciones.copy(),
        tc=tc,
        v_crucero=v_crucero,
        velocidades_maximas=vmax,
    )
