
"""Cinemática directa del robot ABB IRB 120.

Este módulo utiliza la convención Denavit-Hartenberg estándar
para calcular la posición y orientación del extremo del robot.

Se distinguen dos marcos finales:
- Marco {6}: extremo de la cadena cinemática DH.
- TCP: punto académico de herramienta, desplazado 0.120 m
  sobre el eje Z local del marco {6}, sin rotación adicional.

Convenciones:
- Ángulos articulares en radianes.
- Distancias en metros.
- Transformaciones homogéneas de dimensión 4x4.

Modelo cinemático reutilizado del Hito 1 de IMT-342.
"""

import numpy as np

# Importa la función del módulo dh.py que multiplica
# las matrices homogéneas de una cadena cinemática.
from .dh import fk_chain


def irb120_dh_params(q):
    """Construye la tabla DH del ABB IRB 120.

    Parámetros
    ----------
    q : array-like
        Vector [q1, q2, q3, q4, q5, q6] en radianes.

    Retorna
    -------
    list[tuple]
        Seis tuplas (theta, d, a, alpha), una por articulación.
        Las distancias están expresadas en metros.
    """

    # Convierte la entrada a un arreglo NumPy de números reales.
    q = np.asarray(q, dtype=float)

    # El IRB 120 tiene seis articulaciones revolutas.
    # Cada una necesita un valor angular.
    if q.shape != (6,):
        raise ValueError("q debe contener exactamente 6 valores articulares.")

    # Asigna un nombre a cada elemento del vector articular.
    q1, q2, q3, q4, q5, q6 = q

    # Cada fila sigue el orden DH estándar:
    # (theta, d, a, alpha).
    #
    # theta depende del ángulo articular correspondiente.
    # d y a representan desplazamientos geométricos.
    # alpha representa la torsión entre ejes consecutivos.
    #
    # En J2 se utiliza theta2 = q2 - pi/2 para respetar
    # la correspondencia angular del modelo DH del Hito 1.
    return [
        (q1,             0.290, 0.000, -np.pi / 2),
        (q2 - np.pi / 2, 0.000, 0.270,  0.0),
        (q3,             0.000, 0.070, -np.pi / 2),
        (q4,             0.302, 0.000,  np.pi / 2),
        (q5,             0.000, 0.000, -np.pi / 2),
        (q6,             0.072, 0.000,  0.0),
    ]


def forward_kinematics(q):
    """Calcula la transformación desde la base hasta el marco {6}.

    Parámetros
    ----------
    q : array-like
        Vector de seis ángulos articulares en radianes.

    Retorna
    -------
    numpy.ndarray
        Matriz homogénea 4x4 correspondiente a T_0_6.
    """

    # Construye las seis filas DH para la configuración dada.
    dh_params = irb120_dh_params(q)

    # Multiplica A1 @ A2 @ A3 @ A4 @ A5 @ A6.
    # El resultado expresa el marco {6} respecto a la base {0}.
    return fk_chain(dh_params)


def tcp_transform():
    """Construye la transformación fija del marco {6} al TCP.

    El TCP académico se encuentra 0.120 m sobre el eje
    Z local del marco {6}, sin cambio de orientación.

    Retorna
    -------
    numpy.ndarray
        Matriz homogénea 4x4 correspondiente a T_6_TCP.
    """

    # La identidad representa una transformación sin
    # rotación y sin desplazamiento.
    T_6_tcp = np.eye(4)

    # Establece una traslación de 120 mm sobre Z local.
    # Índice [2, 3]: tercera fila y cuarta columna.
    T_6_tcp[2, 3] = 0.120

    return T_6_tcp


def forward_kinematics_tcp(q):
    """Calcula la pose del TCP respecto al marco base {0}.

    Parámetros
    ----------
    q : array-like
        Vector de seis ángulos articulares en radianes.

    Retorna
    -------
    numpy.ndarray
        Matriz homogénea 4x4 correspondiente a T_0_TCP.
    """

    # Primero obtiene la pose del marco {6} respecto a {0}.
    T_0_6 = forward_kinematics(q)

    # Obtiene el desplazamiento fijo del marco {6} al TCP.
    T_6_tcp = tcp_transform()

    # Compone ambas transformaciones:
    # T_0_TCP = T_0_6 @ T_6_TCP.
    return T_0_6 @ T_6_tcp
