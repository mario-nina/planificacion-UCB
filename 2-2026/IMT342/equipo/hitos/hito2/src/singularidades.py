
"""Diagnóstico independiente de singularidad de muñeca del ABB IRB 120.

Referencia académica: guía y rúbrica del Hito 2, Semana 10 de IMT-342.

El rango del Jacobiano determina la pérdida de movilidad instantánea.
La SVD complementa este análisis con valores singulares, direcciones
de movimiento y sensibilidad ante una velocidad cartesiana solicitada.

Para comparar las componentes lineales y angulares del Jacobiano se usa
una longitud de referencia L. Por defecto L = 1 m, como convención
de análisis del equipo.

Las velocidades calculadas son resultados matemáticos, no comandos
seguros para el robot físico.
"""

from dataclasses import dataclass

import numpy as np

from .jacobiano import jacobiano_geometrico
from .limites_irb120 import violaciones_limites


@dataclass(frozen=True)
class DiagnosticoSingularidad:
    """Resultado del análisis de una configuración articular."""

    q: np.ndarray
    rango: int
    valores_singulares: np.ndarray
    condicion: float
    direccion_cartesiana_debil: np.ndarray
    modo_nulo_articular: np.ndarray


@dataclass(frozen=True)
class ExperimentoSingularidad:
    """Resultados obtenidos al aproximar J5 a su postura singular."""

    angulos_q5_rad: np.ndarray
    rangos: np.ndarray
    sigma_min: np.ndarray
    condiciones: np.ndarray
    normas_qdot: np.ndarray
    errores_consigna: np.ndarray
    consigna_cartesiana: np.ndarray
    direccion_perdida_normalizada: np.ndarray
    modo_nulo_articular: np.ndarray
    longitud_referencia_m: float
    diagnosticos: tuple[DiagnosticoSingularidad, ...]


def _validar_positivo(valor, nombre):
    """Exige un escalar positivo y finito."""
    try:
        valor = float(valor)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{nombre} debe ser un escalar positivo finito.") from exc

    if not np.isfinite(valor) or valor <= 0:
        raise ValueError(f"{nombre} debe ser un escalar positivo finito.")

    return valor


def _validar_q(q):
    """Valida seis ángulos articulares finitos, en radianes."""
    try:
        vector = np.asarray(q, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("q debe contener seis valores numéricos.") from exc

    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError("q debe contener seis valores articulares finitos.")

    return vector.copy()


def _svd_jacobiano(q, target_frame, longitud_referencia_m):
    """Calcula el Jacobiano y su descomposición SVD normalizada."""
    jac = jacobiano_geometrico(q, target_frame=target_frame)

    # Escalar la velocidad lineal mediante una longitud de referencia.
    escala = np.array([1.0 / longitud_referencia_m] * 3 + [1.0] * 3)
    jac_reducido = escala[:, None] * jac

    u, s, vh = np.linalg.svd(jac_reducido)

    return jac, jac_reducido, u, s, vh


def analizar_singularidad(
    q,
    *,
    target_frame="flange",
    tolerancia_relativa=1e-9,
    longitud_referencia_m=1.0,
):
    """Analiza rango, valores singulares y direcciones críticas.

    La dirección cartesiana débil se expresa en coordenadas
    normalizadas [v/L, omega]. En singularidad, indica una dirección
    ortogonal a las velocidades cartesianas instantáneamente alcanzables.
    """
    q = _validar_q(q)

    tol = _validar_positivo(tolerancia_relativa, "tolerancia_relativa")
    if tol >= 1:
        raise ValueError("tolerancia_relativa debe ser menor que 1.")

    L = _validar_positivo(longitud_referencia_m, "longitud_referencia_m")

    _, _, u, s, vh = _svd_jacobiano(q, target_frame, L)

    # Rango numérico: contar los valores singulares significativos.
    rango = int(np.sum(s > tol * s[0]))

    # La condición indica la sensibilidad; infinito si hay pérdida de rango.
    condicion = float(s[0] / s[-1]) if rango == 6 else float("inf")

    return DiagnosticoSingularidad(
        q=q,
        rango=rango,
        valores_singulares=s.copy(),
        condicion=condicion,
        direccion_cartesiana_debil=u[:, -1].copy(),
        modo_nulo_articular=vh[-1, :].copy(),
    )


def experimento_singularidad_muneca(
    q_referencia=None,
    *,
    angulos_q5_rad=None,
    target_frame="flange",
    magnitud_twist_normalizado=0.01,
    tolerancia_relativa=1e-9,
    longitud_referencia_m=1.0,
):
    """Estudia una secuencia de configuraciones con q5 -> 0.

    La consigna cartesiana es fija y apunta a la dirección que se
    pierde en la singularidad final. Para posturas regulares se
    resuelve J(q) qdot = twist.

    En la postura singular no se invierte J: se registra el rango,
    una norma de velocidad NaN y el error de la consigna inalcanzable.

    Las velocidades obtenidas no se validan contra límites ABB.
    """
    if q_referencia is None:
        q_referencia = [0.30, -0.35, 0.25, 0.40, 0.0, -0.60]

    q_referencia = _validar_q(q_referencia)

    if violaciones_limites(q_referencia):
        raise ValueError("q_referencia excede los límites nominales ABB.")

    if angulos_q5_rad is None:
        angulos_q5_rad = [0.2, 0.1, 0.05, 0.01, 0.001, 0.0001, 0.0]

    try:
        angulos = np.asarray(angulos_q5_rad, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError("angulos_q5_rad deben ser valores numéricos.") from exc

    if (
        angulos.ndim != 1
        or angulos.size < 3
        or not np.all(np.isfinite(angulos))
        or not np.all(angulos[:-1] > angulos[1:])
        or angulos[-1] != 0
        or np.any(angulos < 0)
    ):
        raise ValueError(
            "angulos_q5_rad debe descender estrictamente desde positivos hasta 0."
        )

    mag = _validar_positivo(
        magnitud_twist_normalizado,
        "magnitud_twist_normalizado",
    )

    tol = _validar_positivo(tolerancia_relativa, "tolerancia_relativa")

    if tol >= 1:
        raise ValueError("tolerancia_relativa debe ser menor que 1.")

    L = _validar_positivo(longitud_referencia_m, "longitud_referencia_m")

    if target_frame not in ("flange", "tcp"):
        raise ValueError("target_frame debe ser 'flange' o 'tcp'.")

    # Obtener la dirección que se vuelve inalcanzable en q5 = 0.
    q_singular = q_referencia.copy()
    q_singular[4] = 0.0

    diagnostico_singular = analizar_singularidad(
        q_singular,
        target_frame=target_frame,
        tolerancia_relativa=tol,
        longitud_referencia_m=L,
    )

    if diagnostico_singular.rango != 5:
        raise ValueError(
            "La referencia no produce una singularidad de rango 5 en q5=0."
        )

    # Una sola consigna, constante para toda la secuencia.
    u_perdida = diagnostico_singular.direccion_cartesiana_debil
    twist_norm = mag * u_perdida

    # Recuperar unidades físicas de la componente lineal.
    twist_fisico = twist_norm.copy()
    twist_fisico[:3] *= L

    diagnosticos = []
    normas = []
    errores = []

    for angulo in angulos:
        q = q_referencia.copy()
        q[4] = angulo

        if violaciones_limites(q):
            raise ValueError(
                "Una postura del experimento excede los límites ABB."
            )

        d = analizar_singularidad(
            q,
            target_frame=target_frame,
            tolerancia_relativa=tol,
            longitud_referencia_m=L,
        )

        diagnosticos.append(d)

        if d.rango == 6:
            # Postura regular: resolver la velocidad articular requerida.
            qdot = np.linalg.solve(
                jacobiano_geometrico(q, target_frame=target_frame),
                twist_fisico,
            )

            normas.append(float(np.linalg.norm(qdot)))

            errores.append(
                float(np.linalg.norm(
                    jacobiano_geometrico(q, target_frame=target_frame)
                    @ qdot - twist_fisico
                ))
            )

        else:
            # Postura singular: medir la componente no alcanzable.
            normas.append(float("nan"))

            _, _, u, _, _ = _svd_jacobiano(q, target_frame, L)

            proy = u[:, d.rango:].T @ twist_norm
            errores.append(float(np.linalg.norm(proy)))

    return ExperimentoSingularidad(
        angulos_q5_rad=angulos.copy(),
        rangos=np.asarray([d.rango for d in diagnosticos], dtype=int),
        sigma_min=np.asarray([
            d.valores_singulares[-1] for d in diagnosticos
        ]),
        condiciones=np.asarray([d.condicion for d in diagnosticos]),
        normas_qdot=np.asarray(normas),
        errores_consigna=np.asarray(errores),
        consigna_cartesiana=twist_fisico.copy(),
        direccion_perdida_normalizada=u_perdida.copy(),
        modo_nulo_articular=diagnostico_singular.modo_nulo_articular.copy(),
        longitud_referencia_m=L,
        diagnosticos=tuple(diagnosticos),
    )
