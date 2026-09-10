import numpy as np

from dh import fk_chain


def irb120_dh_params(q):
    """
    Construye los parámetros D-H del ABB IRB 120
    para una configuración articular dada.

    Parámetros
    ----------
    q : array-like
        Vector articular [q1, q2, q3, q4, q5, q6] en radianes.

    Retorna
    -------
    list[tuple]
        Parámetros (theta, d, a, alpha) para las seis articulaciones.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (6,):
        raise ValueError("q debe contener exactamente 6 valores articulares.")

    q1, q2, q3, q4, q5, q6 = q

    return [
        (q1,             0.290, 0.000, -np.pi / 2),
        (q2 - np.pi / 2, 0.000, 0.270,  0.0),
        (q3,             0.000, 0.070, -np.pi / 2),
        (q4,             0.302, 0.000,  np.pi / 2),
        (q5,             0.000, 0.000, -np.pi / 2),
        (q6,             0.072, 0.000,  0.0),
    ]


def forward_kinematics(q):
    """
    Calcula la cinemática directa del ABB IRB 120.

    Parámetros
    ----------
    q : array-like
        Vector articular [q1, q2, q3, q4, q5, q6] en radianes.

    Retorna
    -------
    numpy.ndarray
        Transformación homogénea 0T6.
    """

    dh_params = irb120_dh_params(q)

    return fk_chain(dh_params)