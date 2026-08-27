import sympy as sp
import numpy as np


def dh_matrix_symbolic(theta, d, a, alpha):
    """
    Construye la matriz de transformación homogénea
    D-H estándar de forma simbólica.
    """

    ct = sp.cos(theta)
    st = sp.sin(theta)
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)

    T = sp.Matrix([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,      d],
        [0,        0,        0,       1]
    ])

    return sp.simplify(T)


def dh_matrix_numeric(theta, d, a, alpha):
    """
    Construye la matriz de transformación homogénea
    D-H estándar de forma numérica.
    """

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,     1.0]
    ], dtype=float)

    return T


def fk_chain(dh_params):
    """
    Calcula la transformación homogénea total
    de una cadena de parámetros D-H.
    """

    T = np.eye(4)

    for theta, d, a, alpha in dh_params:
        T_i = dh_matrix_numeric(theta, d, a, alpha)
        T = T @ T_i

    return T
