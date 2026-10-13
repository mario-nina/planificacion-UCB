
import unittest

import numpy as np

from src.irb120_fk import (
    forward_kinematics,
    forward_kinematics_tcp,
)


class TestCinematicaDirecta(unittest.TestCase):

    def test_configuracion_cero(self):
        q = np.zeros(6)

        T = forward_kinematics(q)

        posicion_esperada = np.array([0.374, 0.0, 0.630])

        np.testing.assert_allclose(
            T[:3, 3],
            posicion_esperada,
            atol=1e-10,
            rtol=0,
        )

    def test_matriz_rotacion(self):
        q = np.array([0.2, -0.3, 0.4, 0.5, -0.6, 0.7])

        T = forward_kinematics(q)
        R = T[:3, :3]

        np.testing.assert_allclose(
            R.T @ R,
            np.eye(3),
            atol=1e-10,
        )

        self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_desplazamiento_tcp(self):
        q = np.array([0.2, -0.3, 0.4, 0.5, -0.6, 0.7])

        T6 = forward_kinematics(q)
        Ttcp = forward_kinematics_tcp(q)

        distancia = np.linalg.norm(
            Ttcp[:3, 3] - T6[:3, 3]
        )

        self.assertAlmostEqual(distancia, 0.120)

    def test_dimension_articular_invalida(self):
        q = np.zeros(5)

        with self.assertRaises(ValueError):
            forward_kinematics(q)


if __name__ == "__main__":
    unittest.main()
