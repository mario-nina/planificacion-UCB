
"""Dinámica reducida 1R alimentada por una trayectoria LSPB existente.

Modelo académico de IMT-342:
    tau = J_1R*qddot + m*g*lc*sin(q)
    J_1R = Ic + m*lc**2

El ángulo se mide desde la vertical inferior. J_1R es inercia,
no el Jacobiano geométrico.

No representa la dinámica completa ni los torques reales del ABB IRB 120.
Si se utiliza una junta de un perfil 6R, su trayectoria actúa solamente
como señal de entrada al modelo pedagógico 1R.
"""

from dataclasses import dataclass

import numpy as np

from .lspb import PerfilLSPB


def _escalar(nombre, valor, *, estrictamente_positivo=True):
    """Valida un parámetro numérico finito y su condición de signo."""
    try:
        numero = float(valor)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{nombre} debe ser un escalar finito.") from exc

    if not np.isfinite(numero):
        raise ValueError(f"{nombre} debe ser un escalar finito.")
    if estrictamente_positivo and numero <= 0:
        raise ValueError(f"{nombre} debe ser positivo.")
    if not estrictamente_positivo and numero < 0:
        raise ValueError(f"{nombre} no puede ser negativo.")

    return numero


@dataclass(frozen=True)
class ParametrosDinamica1R:
    """Parámetros didácticos en SI; no corresponden a datos ABB.

    inercia_kg_m2: J_1R = Ic + m*lc**2.
    masa_kg: masa del modelo.
    longitud_cm_m: distancia del eje al centro de masa.
    gravedad_m_s2: magnitud de gravedad; puede ser cero.
    """

    inercia_kg_m2: float
    masa_kg: float
    longitud_cm_m: float
    gravedad_m_s2: float = 9.81

    def __post_init__(self):
        for nombre in ("inercia_kg_m2", "masa_kg", "longitud_cm_m"):
            object.__setattr__(self, nombre, _escalar(nombre, getattr(self, nombre)))
        object.__setattr__(
            self, "gravedad_m_s2",
            _escalar("gravedad_m_s2", self.gravedad_m_s2, estrictamente_positivo=False),
        )


@dataclass(frozen=True)
class MuestraDinamica1R:
    """Estado cinemático y componentes del torque para uno o varios tiempos."""

    t_s: float | np.ndarray
    q_rad: float | np.ndarray
    qdot_rad_s: float | np.ndarray
    qddot_rad_s2: float | np.ndarray
    tau_inercial_nm: float | np.ndarray
    tau_gravitatorio_nm: float | np.ndarray
    tau_total_nm: float | np.ndarray
    indice_articulacion: int  # 0 para 1R; índice 0..5 para una señal 6R


def _escalar_o_vector(valor):
    """Devuelve un float para escalares o una copia para arreglos."""
    salida = np.array(valor, dtype=float, copy=True)
    return float(salida) if salida.ndim == 0 else salida


def evaluar_dinamica_1r(perfil, t, parametros, *, indice_articulacion=None):
    """Evalúa el torque 1R usando q, qdot y qddot de un perfil LSPB.

    Un perfil 1R utiliza su única articulación.
    Un perfil 6R exige seleccionar explícitamente una de sus juntas.

    No calcula acoplamientos dinámicos del manipulador completo.
    En los cambios de fase se utiliza la aceleración dada por LSPB.
    """
    if not isinstance(perfil, PerfilLSPB):
        raise ValueError("perfil debe ser un PerfilLSPB existente.")
    if not isinstance(parametros, ParametrosDinamica1R):
        raise ValueError("parametros debe ser ParametrosDinamica1R.")

    n = perfil.q_inicio.size

    if n == 1:
        if indice_articulacion not in (None, 0) or isinstance(indice_articulacion, bool):
            raise ValueError("En un perfil 1R, indice_articulacion debe ser 0 o None.")
        indice = 0
    elif n == 6:
        if (isinstance(indice_articulacion, bool)
                or not isinstance(indice_articulacion, (int, np.integer))
                or not 0 <= indice_articulacion < 6):
            raise ValueError("En un perfil 6R debe indicarse una articulacion entre 0 y 5.")
        indice = int(indice_articulacion)
    else:
        raise ValueError("El perfil debe contener una o seis articulaciones.")

    # Reutilizar la trayectoria ya calculada por el planificador LSPB.
    muestra = perfil.evaluar(t)
    q = _escalar_o_vector(muestra.q[..., indice])
    qdot = _escalar_o_vector(muestra.qdot[..., indice])
    qddot = _escalar_o_vector(muestra.qddot[..., indice])

    # Separar el torque de inercia y el de gravedad.
    tau_inercial = parametros.inercia_kg_m2 * qddot
    tau_gravitatorio = (
        parametros.masa_kg * parametros.gravedad_m_s2
        * parametros.longitud_cm_m * np.sin(q)
    )
    tau_total = tau_inercial + tau_gravitatorio

    return MuestraDinamica1R(
        t_s=_escalar_o_vector(t),
        q_rad=q,
        qdot_rad_s=qdot,
        qddot_rad_s2=qddot,
        tau_inercial_nm=_escalar_o_vector(tau_inercial),
        tau_gravitatorio_nm=_escalar_o_vector(tau_gravitatorio),
        tau_total_nm=_escalar_o_vector(tau_total),
        indice_articulacion=indice,
    )
