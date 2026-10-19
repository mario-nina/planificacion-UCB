"""Dinámica reducida 1R alimentada por una trayectoria LSPB existente.

Fuente docente: IMT-342, S10_material_oficial.pdf, guia oficial del Hito 2
(pp. 31-32) y solucionario W10-S01 (pp. 16-17).

Modelo educativo: tau = J_1R*qddot + m*g*lc*sin(q), medido desde la
vertical inferior, con J_1R = Ic + m*lc**2 constante. El término de
Coriolis/centrífugo no aparece en ESTE modelo 1R particular.

NO es el modelo dinámico completo del ABB IRB 120 ni predice el torque
real de sus actuadores. No se deben atribuir estas masas o inercias a ABB.
Si se elige una junta de un LSPB 6R, su q se emplea como señal de entrada
pedagógica: no se identifica automáticamente con el ángulo gravitatorio
físico de esa junta (que depende de marcos y acoplamientos).
"""

from dataclasses import dataclass

import numpy as np

from .lspb import PerfilLSPB


def _escalar(nombre, valor, *, estrictamente_positivo=True):
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
    """Parametros didacticos del 1R en unidades SI, no parametros ABB.

    inercia_kg_m2: Ic + m*lc**2, kg*m^2 (no Jacobiano geometrico).
    masa_kg: masa concentrada/equivalente del modelo, kg.
    longitud_cm_m: distancia del eje al centro de masa, m.
    gravedad_m_s2: magnitud de g, m/s^2; cero permite aislar la inercia.
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
    """Para t escalar, cantidades float; para tiempos vectoriales, arrays (N,)."""

    t_s: float | np.ndarray
    q_rad: float | np.ndarray
    qdot_rad_s: float | np.ndarray
    qddot_rad_s2: float | np.ndarray
    tau_inercial_nm: float | np.ndarray
    tau_gravitatorio_nm: float | np.ndarray
    tau_total_nm: float | np.ndarray
    indice_articulacion: int  # 0 en 1R, índice 0..5 para señal tomada de LSPB 6R


def _escalar_o_vector(valor):
    salida = np.array(valor, dtype=float, copy=True)
    return float(salida) if salida.ndim == 0 else salida


def evaluar_dinamica_1r(perfil, t, parametros, *, indice_articulacion=None):
    """Calcula tau(t) para una señal articular obtenida desde LSPB.

    `perfil` debe ser PerfilLSPB de 1 o 6 juntas. Para seis articulaciones
    se exige seleccionar explícitamente `indice_articulacion` (0 a 5).
    Esta selección es SOLO una entrada al modelo académico 1R; no modela
    la dinámica acoplada de la articulación seleccionada del ABB.

    Los puntos exactos de cambio de fase LSPB tienen aceleración ideal
    discontinua; se utiliza la rama que devuelve `perfil.evaluar(t)`.
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

    muestra = perfil.evaluar(t)  # conserva validación temporal del planificador
    q = _escalar_o_vector(muestra.q[..., indice])
    qdot = _escalar_o_vector(muestra.qdot[..., indice])
    qddot = _escalar_o_vector(muestra.qddot[..., indice])
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
