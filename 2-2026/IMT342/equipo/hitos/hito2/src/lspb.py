"""Perfil LSPB articular: movimiento punto a punto con reposo en extremos.

Fundamento docente: IMT-342, material oficial S09, W09-S01,
"Generacion de Trayectorias: Perfil LSPB (1/2)" y (2/2),
"Ejemplo Guiado: LSPB" (diapositivas 8-10).

Velocidades ABB: IRB 120-3/0.6, especificaciones de producto
3HAC035960-005 rev. W, seccion 1.8.3, pagina impresa 40.

Las aceleraciones se SUMINISTRAN en rad/s^2: no son datos certificados
por ABB. Los limites de velocidad son cotas nominales de referencia,
no prueban seguridad, colisiones, par disponible ni viabilidad dinamica.
"""

from dataclasses import dataclass

import numpy as np

from .limites_irb120 import violaciones_limites

VELOCIDADES_ABB_GRADOS_S = np.array(
    [250.0, 250.0, 250.0, 320.0, 320.0, 420.0], dtype=float
)
VELOCIDADES_ABB_GRADOS_S.setflags(write=False)
VELOCIDADES_ABB_RAD_S = np.deg2rad(VELOCIDADES_ABB_GRADOS_S)
VELOCIDADES_ABB_RAD_S.setflags(write=False)


@dataclass
class MuestraLSPB:
    """Para t escalar: vectores (n,); para t vectorial: matrices (N,n)."""

    q: np.ndarray
    qdot: np.ndarray
    qddot: np.ndarray


@dataclass
class PerfilLSPB:
    """Plan de n=1 o n=6 juntas, con duracion final compartida."""

    q_inicio: np.ndarray
    q_final: np.ndarray
    tf: float
    aceleraciones: np.ndarray  # magnitudes positivas, rad/s^2
    tc: np.ndarray  # tiempos de mezcla, s
    v_crucero: np.ndarray  # velocidad CON SIGNO, rad/s
    velocidades_maximas: np.ndarray  # magnitudes efectivas, rad/s

    def evaluar(self, t):
        """Evalua q, qdot y qddot en tiempos dentro de [0, tf].

        En los puntos de quiebre la aceleracion es discontinua;
        se devuelve una de las ramas y no una aceleracion unica fisica.
        """
        try:
            tiempos = np.asarray(t, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("t debe ser un tiempo o array numerico.") from exc
        if tiempos.ndim > 1 or not np.all(np.isfinite(tiempos)):
            raise ValueError("t debe ser escalar o vector unidimensional finito.")
        escalar = tiempos.ndim == 0
        tiempos = np.atleast_1d(tiempos)
        if np.any(tiempos < 0) or np.any(tiempos > self.tf):
            raise ValueError("Los tiempos deben pertenecer a [0, tf].")

        n = self.q_inicio.size
        q = np.empty((tiempos.size, n), dtype=float)
        qdot = np.empty_like(q)
        qddot = np.empty_like(q)
        for i in range(n):
            qi, qf = self.q_inicio[i], self.q_final[i]
            tc, vc = self.tc[i], self.v_crucero[i]
            a = np.sign(qf - qi) * self.aceleraciones[i]
            ta = tiempos <= tc
            tfren = (tiempos >= self.tf - tc) & ~ta
            tconst = ~(ta | tfren)
            q[ta, i] = qi + 0.5 * a * tiempos[ta] ** 2
            qdot[ta, i] = a * tiempos[ta]
            qddot[ta, i] = a
            q[tconst, i] = qi + vc * (tiempos[tconst] - tc / 2)
            qdot[tconst, i] = vc
            qddot[tconst, i] = 0.0
            restante = self.tf - tiempos[tfren]
            q[tfren, i] = qf - 0.5 * a * restante ** 2
            qdot[tfren, i] = a * restante
            qddot[tfren, i] = -a

        if escalar:
            return MuestraLSPB(q[0], qdot[0], qddot[0])
        return MuestraLSPB(q, qdot, qddot)


def _vector(nombre, valor):
    try:
        vector = np.asarray(valor, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{nombre} debe ser vector numerico.") from exc
    if vector.ndim != 1 or vector.size not in (1, 6) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{nombre} debe tener uno o seis valores finitos.")
    return vector.copy()


def planificar_lspb(q_inicio, q_final, tf, aceleraciones, *, velocidades_maximas=None):
    """Construye LSPB 1R o 6R de reposo a reposo, tiempo global tf.

    Las aceleraciones son magnitudes elegidas por el usuario (rad/s^2).
    En 6R se aplican siempre los maximos nominales ABB y limites ABB
    de posicion. `velocidades_maximas` agrega cotas opcionales mas
    restrictivas. En 1R no se atribuyen cotas fisicas a ABB.

    Raises ValueError si la duracion no permite el perfil con las
    aceleraciones indicadas o si la velocidad de crucero excede las cotas.
    """
    inicio = _vector("q_inicio", q_inicio)
    final = _vector("q_final", q_final)
    aceleraciones = _vector("aceleraciones", aceleraciones)
    if inicio.shape != final.shape or inicio.shape != aceleraciones.shape:
        raise ValueError("Las dimensiones de q_inicio, q_final y aceleraciones deben coincidir.")
    if np.any(aceleraciones <= 0):
        raise ValueError("Las aceleraciones de planificacion deben ser positivas.")
    try:
        tf = float(tf)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError("tf debe ser un escalar positivo finito.") from exc
    if not np.isfinite(tf) or tf <= 0:
        raise ValueError("tf debe ser un escalar positivo finito.")

    n = inicio.size
    if n == 6:
        if violaciones_limites(inicio) or violaciones_limites(final):
            raise ValueError("Las posiciones inicial y final deben cumplir limites nominales ABB.")
        vmax = VELOCIDADES_ABB_RAD_S.copy()
    else:
        vmax = np.full(n, np.inf)
    if velocidades_maximas is not None:
        cotas = _vector("velocidades_maximas", velocidades_maximas)
        if cotas.shape != inicio.shape or np.any(cotas <= 0):
            raise ValueError("Las velocidades_maximas deben tener la misma dimension y ser positivas.")
        vmax = np.minimum(vmax, cotas)

    desplazamientos = np.abs(final - inicio)
    # Condicion de factibilidad de W09-S01: a >= 4*abs(dq)/tf**2.
    # Se calcula sin restas de numeros proximos para mayor estabilidad.
    fraccion = 4.0 * desplazamientos / (aceleraciones * tf * tf)
    if not np.all(np.isfinite(fraccion)) or np.any(fraccion > 1.0 + 1e-12):
        raise ValueError("Aceleracion insuficiente para la duracion tf (LSPB inviable).")
    fraccion = np.minimum(fraccion, 1.0)
    raiz = np.sqrt(1.0 - fraccion)
    tc = (2.0 * desplazamientos / aceleraciones) / (tf * (1.0 + raiz))
    v_crucero = np.sign(final - inicio) * aceleraciones * tc
    if np.any(np.abs(v_crucero) > vmax + 1e-12):
        raise ValueError("La velocidad de crucero excede los limites configurados o nominales ABB.")

    return PerfilLSPB(
        q_inicio=inicio, q_final=final, tf=tf,
        aceleraciones=aceleraciones.copy(), tc=tc,
        v_crucero=v_crucero, velocidades_maximas=vmax,
    )
