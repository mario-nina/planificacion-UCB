"""Pruebas para 0J del IRB 120: caso manual y validación numérica FK."""

import unittest
import numpy as np

from src.irb120_fk import forward_kinematics, forward_kinematics_tcp
from src.jacobiano import (
    jacobiano_geometrico,
    transformaciones_intermedias,
    velocidad_cartesiana,
)


class TestJacobianoGeometrico(unittest.TestCase):
    def test_transformaciones_coinciden_con_fk_y_offset(self):
        q = np.array([0.2, -0.3, 0.4, 0.5, -0.6, 0.7])
        ts = transformaciones_intermedias(q)
        self.assertEqual(len(ts), 7)
        np.testing.assert_allclose(ts[0], np.eye(4), atol=1e-14)
        np.testing.assert_allclose(ts[-1], forward_kinematics(q), atol=1e-14)
        # Posición a2 en q=0: el offset q2-pi/2 apunta el brazo hacia +Z.
        cero = transformaciones_intermedias(np.zeros(6))
        np.testing.assert_allclose(cero[2][:3, 3], [0, 0, .560], atol=1e-14)

    def test_columna_j1_calculo_manual_en_cero_brida(self):
        # z0=[0,0,1], o0=[0,0,0], p6=[.374,0,.630]
        # z0 x (p6-o0)=[0,.374,0]
        j = jacobiano_geometrico(np.zeros(6))
        np.testing.assert_allclose(j[:, 0], [0, .374, 0, 0, 0, 1], atol=1e-12)

    def test_columna_j2_calculo_manual_en_cero_brida(self):
        # z1=[0,1,0], o1=[0,0,.290], p6=[.374,0,.630]
        # z1 x (p6-o1)=[.340,0,-.374]
        j = jacobiano_geometrico(np.zeros(6))
        np.testing.assert_allclose(j[:, 1], [.340, 0, -.374, 0, 1, 0], atol=1e-12)

    def test_j1_tcp_efecto_del_desplazamiento(self):
        # p_tcp=[.494,0,.630] en q=0.
        j = jacobiano_geometrico(np.zeros(6), target_frame="tcp")
        np.testing.assert_allclose(j[:, 0], [0, .494, 0, 0, 0, 1], atol=1e-12)

    def test_j6_lineal_nulo_para_brida_y_tcp(self):
        q = [.2, -.3, .4, .5, -.6, .7]
        for marco in ("flange", "tcp"):
            with self.subTest(marco=marco):
                j = jacobiano_geometrico(q, target_frame=marco)
                np.testing.assert_allclose(j[:3, 5], 0, atol=1e-12)
                self.assertAlmostEqual(np.linalg.norm(j[3:, 5]), 1, places=12)

    def test_diferencias_finitas_posicion_y_angular(self):
        # Validación independiente: derivar numéricamente T(q) evaluada por FK.
        h = 1e-6
        poses = (
            np.array([.2, -.3, .4, .5, -.6, .7]),
            np.array([-.45, .32, -.2, 1.0, .3, -.8]),
            np.array([.1, .2, .1, -.2, .8, .4]),
        )
        for marco in ("flange", "tcp"):
            fk = forward_kinematics if marco == "flange" else forward_kinematics_tcp
            for q in poses:
                j = jacobiano_geometrico(q, target_frame=marco)
                r_cero = fk(q)[:3, :3]
                for i in range(6):
                    e = np.zeros(6)
                    e[i] = h
                    t_mas = fk(q + e)
                    t_menos = fk(q - e)
                    v_fd = (t_mas[:3, 3] - t_menos[:3, 3]) / (2*h)
                    r_punto = (t_mas[:3, :3] - t_menos[:3, :3]) / (2*h)
                    omega_cruz = r_punto @ r_cero.T
                    omega_fd = np.array([
                        omega_cruz[2, 1],
                        omega_cruz[0, 2],
                        omega_cruz[1, 0],
                    ])
                    np.testing.assert_allclose(j[:3, i], v_fd, rtol=0, atol=2e-8)
                    np.testing.assert_allclose(j[3:, i], omega_fd, rtol=0, atol=2e-8)

    def test_velocidad_cartesiana_con_fk_diferencial(self):
        q = np.array([.2, -.3, .4, .5, -.6, .7])
        qdot = np.array([.4, -.5, .2, 1., -.3, .6])
        h = 1e-6
        for marco in ("flange", "tcp"):
            fk = forward_kinematics if marco == "flange" else forward_kinematics_tcp
            t_mas = fk(q + h*qdot)
            t_menos = fk(q - h*qdot)
            r = fk(q)[:3, :3]
            v_fd = (t_mas[:3, 3] - t_menos[:3, 3])/(2*h)
            omega_mat = ((t_mas[:3, :3]-t_menos[:3, :3])/(2*h)) @ r.T
            omega_fd = [omega_mat[2,1],omega_mat[0,2],omega_mat[1,0]]
            twist = velocidad_cartesiana(q, qdot, target_frame=marco)
            np.testing.assert_allclose(twist[:3], v_fd, atol=3e-8, rtol=0)
            np.testing.assert_allclose(twist[3:], omega_fd, atol=3e-8, rtol=0)

    def test_velocidad_nula_y_dimensiones(self):
        q = np.zeros(6)
        np.testing.assert_array_equal(velocidad_cartesiana(q, np.zeros(6)), np.zeros(6))
        self.assertEqual(jacobiano_geometrico(q).shape, (6, 6))

    def test_invalidos(self):
        for q in ([0]*5, [0]*7, [0,0,0,np.nan,0,0], [0,0,0,np.inf,0,0]):
            with self.subTest(q=q):
                with self.assertRaises(ValueError):
                    jacobiano_geometrico(q)
        for qdot in ([0]*5, [0,0,0,0,0,np.nan]):
            with self.subTest(qdot=qdot):
                with self.assertRaises(ValueError):
                    velocidad_cartesiana(np.zeros(6), qdot)
        with self.assertRaises(ValueError):
            jacobiano_geometrico(np.zeros(6), target_frame="mundo")


if __name__ == "__main__":
    unittest.main()
