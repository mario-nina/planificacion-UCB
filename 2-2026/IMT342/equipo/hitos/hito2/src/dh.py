
"""Herramientas de transformaciones homogéneas D-H estándar.

Este módulo es genérico: no contiene dimensiones específicas del ABB IRB 120.

Convenciones:
- D-H estándar: Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha).
- Ángulos en radianes.
- Distancias en metros.
- Vectores expresados como columnas.
- Matrices homogéneas de dimensión 4x4.
"""

import numpy as np


def dh_matrix(theta, d, a, alpha):
    """Construye la transformación homogénea D-H estándar.

    Parámetros
    ----------
    theta : float
        Rotación alrededor del eje Z anterior, en radianes.
    d : float
        Desplazamiento a lo largo del eje Z anterior, en metros.
    a : float
        Desplazamiento a lo largo del eje X del sistema siguiente,
        en metros.
    alpha : float
        Rotación alrededor de ese eje X, en radianes.

    Retorna
    -------
    numpy.ndarray
        Transformación homogénea A_i de dimensión 4x4.
    """

    # Calcular una sola vez los senos y cosenos de los ángulos.
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # Filas 1-3:
    #   Columnas 1-3 -> orientación (matriz de rotación R).
    #   Columna 4     -> posición del origen siguiente (vector p).
    #
    # Fila 4:
    #   [0, 0, 0, 1] completa la transformación homogénea.

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,       ca,      d],
        [0.0,      0.0,      0.0,    1.0],
    ], dtype=float)


def fk_chain(dh_params):
    """Encadena transformaciones DH para obtener la pose final.

    Parámetros
    ----------
    dh_params : iterable
        Secuencia ordenada de tuplas (theta, d, a, alpha).

    Retorna
    -------
    numpy.ndarray
        Transformación homogénea acumulada 4x4.
    """

    # Partimos de la identidad: inicialmente no se ha aplicado
    # ninguna transformación al marco base.
    T = np.eye(4)

    # Recorrer cada fila de la tabla DH en el orden del robot.
    for theta, d, a, alpha in dh_params:

        # Transformación relativa del eslabón actual.
        T_i = dh_matrix(theta, d, a, alpha)

        # Acumular mediante multiplicación matricial.
        # El orden importa: T @ T_i, no T_i @ T.
        T = T @ T_i

    # Al terminar, T describe el último marco respecto a la base.
    return T
