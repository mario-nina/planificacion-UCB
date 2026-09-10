import numpy as np


def dh_matrix(theta, d, a, alpha):
    """
    Construye la matriz de transformación homogénea D-H estándar.

    Parámetros
    ----------
    theta : float
        Rotación alrededor de Z, en radianes.
    d : float
        Traslación a lo largo de Z, en metros.
    a : float
        Traslación a lo largo de X, en metros.
    alpha : float
        Rotación alrededor de X, en radianes.

    Retorna
    -------
    numpy.ndarray
        Matriz homogénea 4x4.
    """

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,       ca,      d],
        [0.0,      0.0,      0.0,    1.0],
    ], dtype=float)


def fk_chain(dh_params):
    """
    Calcula la cinemática directa encadenando matrices D-H estándar.

    Parámetros
    ----------
    dh_params : iterable
        Secuencia de tuplas (theta, d, a, alpha).

    Retorna
    -------
    numpy.ndarray
        Transformación homogénea total 4x4.
    """

    T = np.eye(4)

    for theta, d, a, alpha in dh_params:
        T_i = dh_matrix(theta, d, a, alpha)
        T = T @ T_i

    return T