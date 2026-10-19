"""Diagnóstico independiente de singularidad de muñeca del ABB IRB 120.

Fuente académica: IMT-342, S10_material_oficial.pdf, hito_2_guia.pdf,
p. 31; hito_2_rubrica.pdf, p. 35. Se exige rango, movilidad perdida,
y amplificación articular ante una demanda cartesiana de prueba.

Los valores singulares y vectores SVD son diagnósticos COMPLEMENTARIOS:
no reemplazan el criterio de rango. Para tratar coherentemente las
unidades, se normaliza la componente lineal de la velocidad por una
longitud de referencia L (m). Así, la matriz reducida Jn = diag(1/L,
1/L, 1/L, 1, 1, 1) J tiene filas expresadas en 1/s para qdot en rad/s.
La elección L=1 m es una convención de análisis del equipo; modifica
los valores singulares y el vector de prueba, no la singularidad exacta.

Ninguna velocidad articular calculada constituye una orden de control
válida para el equipo ABB; no se comprueban límites de velocidad aquí.
"""

from dataclasses import dataclass

import numpy as np

from .jacobiano import jacobiano_geometrico
from .limites_irb120 import violaciones_limites


@dataclass(frozen=True)
class DiagnosticoSingularidad:
    q: np.ndarray
    rango: int
    valores_singulares: np.ndarray
    condicion: float
    direccion_cartesiana_debil: np.ndarray
    modo_nulo_articular: np.ndarray


@dataclass(frozen=True)
class ExperimentoSingularidad:
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
    try:
        valor = float(valor)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{nombre} debe ser un escalar positivo finito.") from exc
    if not np.isfinite(valor) or valor <= 0:
        raise ValueError(f"{nombre} debe ser un escalar positivo finito.")
    return valor


def _validar_q(q):
    try:
        vector = np.asarray(q, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("q debe contener seis valores numéricos.") from exc
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError("q debe contener seis valores articulares finitos.")
    return vector.copy()


def _svd_jacobiano(q, target_frame, longitud_referencia_m):
    jac = jacobiano_geometrico(q, target_frame=target_frame)
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
    """Rango 6x6 y dirección cartesiana débil, todos en base {0}.

    direccion_cartesiana_debil se expresa para el twist normalizado
    [v_lineal/L; omega]. En singularidad es un vector ortogonal al
    espacio columna de Jn, indicando una dirección no realizable.
    """
    q = _validar_q(q)
    tol = _validar_positivo(tolerancia_relativa, "tolerancia_relativa")
    if tol >= 1:
        raise ValueError("tolerancia_relativa debe ser menor que 1.")
    L = _validar_positivo(longitud_referencia_m, "longitud_referencia_m")
    _, _, u, s, vh = _svd_jacobiano(q, target_frame, L)
    rango = int(np.sum(s > tol * s[0]))
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
    """Aproxima q5 -> 0 manteniendo q1..q4,q6 constantes.

    Usa una única consigna cartesiana *fija*, construida desde la
    dirección izquierda nula en q5=0. En pasos regulares se resuelve
    J(q) qdot = twist mediante solve; en q5=0 no se intenta invertir la
    matriz, se registra rango 5, norma qdot = NaN y error de consigna
    correspondiente a la componente inalcanzable.

    Los qdot son ejemplos matemáticos para mostrar amplificación y
    pueden exceder los límites del fabricante; NO son comandos seguros.
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
        raise ValueError("angulos_q5_rad debe descender estrictamente desde positivos hasta 0.")
    mag = _validar_positivo(magnitud_twist_normalizado, "magnitud_twist_normalizado")
    tol = _validar_positivo(tolerancia_relativa, "tolerancia_relativa")
    if tol >= 1:
        raise ValueError("tolerancia_relativa debe ser menor que 1.")
    L = _validar_positivo(longitud_referencia_m, "longitud_referencia_m")
    if target_frame not in ("flange", "tcp"):
        raise ValueError("target_frame debe ser 'flange' o 'tcp'.")

    q_singular = q_referencia.copy()
    q_singular[4] = 0.0
    diagnostico_singular = analizar_singularidad(
        q_singular,
        target_frame=target_frame,
        tolerancia_relativa=tol,
        longitud_referencia_m=L,
    )
    if diagnostico_singular.rango != 5:
        raise ValueError("La referencia no produce una singularidad de rango 5 en q5=0.")

    u_perdida = diagnostico_singular.direccion_cartesiana_debil
    twist_norm = mag * u_perdida
    twist_fisico = twist_norm.copy()
    twist_fisico[:3] *= L

    diagnosticos = []
    normas = []
    errores = []
    for angulo in angulos:
        q = q_referencia.copy()
        q[4] = angulo
        if violaciones_limites(q):
            raise ValueError("Una postura del experimento excede los límites ABB.")
        d = analizar_singularidad(
            q,
            target_frame=target_frame,
            tolerancia_relativa=tol,
            longitud_referencia_m=L,
        )
        diagnosticos.append(d)
        if d.rango == 6:
            qdot = np.linalg.solve(jacobiano_geometrico(q, target_frame=target_frame), twist_fisico)
            normas.append(float(np.linalg.norm(qdot)))
            errores.append(float(np.linalg.norm(
                jacobiano_geometrico(q, target_frame=target_frame) @ qdot - twist_fisico
            )))
        else:
            normas.append(float("nan"))
            # Proyección sobre el complemento ortogonal al espacio columna
            # en coordenadas normalizadas, sin calcular pseudoinversa.
            _, _, u, _, _ = _svd_jacobiano(q, target_frame, L)
            proy = u[:, d.rango:].T @ twist_norm
            errores.append(float(np.linalg.norm(proy)))

    return ExperimentoSingularidad(
        angulos_q5_rad=angulos.copy(),
        rangos=np.asarray([d.rango for d in diagnosticos], dtype=int),
        sigma_min=np.asarray([d.valores_singulares[-1] for d in diagnosticos]),
        condiciones=np.asarray([d.condicion for d in diagnosticos]),
        normas_qdot=np.asarray(normas),
        errores_consigna=np.asarray(errores),
        consigna_cartesiana=twist_fisico.copy(),
        direccion_perdida_normalizada=u_perdida.copy(),
        modo_nulo_articular=diagnostico_singular.modo_nulo_articular.copy(),
        longitud_referencia_m=L,
        diagnosticos=tuple(diagnosticos),
    )
