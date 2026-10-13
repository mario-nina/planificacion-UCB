"""Validacion LSPB: fuente docente S09 W09-S01 y velocidades ABB rev. W."""
import unittest

import numpy as np

from src.lspb import VELOCIDADES_ABB_RAD_S, planificar_lspb


class TestLSPB(unittest.TestCase):
    def test_ejemplo_oficial_docente(self):
        perfil = planificar_lspb([0], [2], 3, [1])
        self.assertAlmostEqual(perfil.tc[0], 1.0)
        self.assertAlmostEqual(perfil.v_crucero[0], 1.0)
        m = perfil.evaluar(np.array([0, 0.5, 1, 1.5, 2, 2.5, 3]))
        np.testing.assert_allclose(m.q[:, 0], [0, .125, .5, 1, 1.5, 1.875, 2], atol=1e-12)
        np.testing.assert_allclose(m.qdot[:, 0], [0, .5, 1, 1, 1, .5, 0], atol=1e-12)
        self.assertEqual(m.q.shape, (7, 1))

    def test_aceleracion_saltos_no_exigir_continuidad(self):
        p = planificar_lspb([0], [2], 3, [1])
        m = p.evaluar([.5, 1.5, 2.5])
        np.testing.assert_allclose(m.qddot[:, 0], [1, 0, -1], atol=1e-12)

    def test_perfil_triangular_en_igualdad(self):
        p = planificar_lspb([0], [2], 3, [8/9])
        self.assertAlmostEqual(p.tc[0], 1.5)
        m = p.evaluar([0, 1.5, 3])
        np.testing.assert_allclose(m.q[:, 0], [0, 1, 2], atol=1e-12)
        np.testing.assert_allclose(m.qdot[[0,2],0], [0,0], atol=1e-12)

    def test_aceleracion_insuficiente(self):
        with self.assertRaisesRegex(ValueError, "Aceleracion insuficiente"):
            planificar_lspb([0], [2], 3, [.8])

    def test_movimiento_negativo(self):
        p = planificar_lspb([2], [0], 3, [1])
        self.assertAlmostEqual(p.v_crucero[0], -1.0)
        m = p.evaluar([0, .5, 1.5, 2.5, 3])
        np.testing.assert_allclose(m.q[:, 0], [2, 1.875, 1, .125, 0], atol=1e-12)
        np.testing.assert_allclose(m.qdot[[0,-1], 0], [0, 0], atol=1e-12)

    def test_sin_desplazamiento(self):
        p = planificar_lspb([1], [1], 3, [.5])
        self.assertEqual(p.tc[0], 0)
        m = p.evaluar([0, 1, 2, 3])
        np.testing.assert_allclose(m.q[:, 0], [1]*4)
        np.testing.assert_allclose(m.qdot, 0)
        np.testing.assert_allclose(m.qddot, 0)

    def test_seis_juntas_sincronizadas(self):
        qi = np.zeros(6)
        qf = np.array([.2, -.3, .1, .4, 0, .35])
        p = planificar_lspb(qi, qf, 3, np.ones(6))
        muestras = p.evaluar(np.linspace(0,3,61))
        np.testing.assert_allclose(muestras.q[0], qi, atol=1e-12)
        np.testing.assert_allclose(muestras.q[-1], qf, atol=1e-12)
        np.testing.assert_allclose(muestras.qdot[[0,-1]], 0, atol=1e-12)
        self.assertEqual(muestras.q.shape, (61,6))
        self.assertTrue(np.all(np.abs(muestras.qdot) <= VELOCIDADES_ABB_RAD_S + 1e-12))
        self.assertTrue(np.all(np.abs(muestras.qddot) <= 1.0 + 1e-12))
        np.testing.assert_allclose(muestras.q[:,4], 0)

    def test_velocidad_maxima_abb(self):
        # J1 a 2 rad/s excede cualquier cota elegida de 1 rad/s.
        qf = np.zeros(6)
        qf[0] = 2.0
        with self.assertRaisesRegex(ValueError, "velocidad de crucero"):
            planificar_lspb(np.zeros(6), qf, 1.5, np.full(6, 4.0),
                           velocidades_maximas=np.full(6, 1.0))

    def test_velocidad_nominal_abb_no_configurable_por_encima(self):
        # Sin velocidades_maximas explicitas, J1 queda limitado a 250 deg/s.
        qf = np.zeros(6)
        qf[0] = 2.0
        with self.assertRaisesRegex(ValueError, "velocidad de crucero"):
            planificar_lspb(np.zeros(6), qf, 0.5, np.full(6, 40.0))

    def test_integracion_fk_pieper_seleccion_y_lspb(self):
        from src.irb120_fk import forward_kinematics
        from src.pieper import solve_pieper
        from src.seleccion_pieper import filtrar_y_seleccionar_pieper

        q_inicio = np.zeros(6)
        objetivo = np.array([0.3, -0.2, 0.1, 0.2, -0.4, 0.5])
        T_objetivo = forward_kinematics(objetivo)
        soluciones = solve_pieper(T_objetivo, q_seed=q_inicio)
        eleccion = filtrar_y_seleccionar_pieper(soluciones, q_inicio)
        self.assertIsNotNone(eleccion.seleccionada)
        q_final = eleccion.seleccionada.q
        perfil = planificar_lspb(q_inicio, q_final, 8.0, np.ones(6))
        muestras = perfil.evaluar(np.linspace(0, 8, 101))
        np.testing.assert_allclose(muestras.q[0], q_inicio, atol=1e-12)
        np.testing.assert_allclose(muestras.q[-1], q_final, atol=1e-12)
        np.testing.assert_allclose(
            forward_kinematics(muestras.q[-1]), T_objetivo, atol=1e-8
        )

    def test_abb_limites_de_posicion(self):
        qf = np.zeros(6)
        qf[2] = np.deg2rad(80)
        with self.assertRaisesRegex(ValueError, "posiciones inicial y final"):
            planificar_lspb(np.zeros(6), qf, 3, np.ones(6))

    def test_validacion_entradas(self):
        casos = [
            ([0], [1], 0, [1]),
            ([0], [1], 1, [-1]),
            ([0], [np.nan], 1, [1]),
            ([0], [1], np.inf, [1]),
            ([0, 1], [1, 2], 3, [1, 1]),
            ([0], [1], 3, [1, 2]),
        ]
        for args in casos:
            with self.subTest(args=args), self.assertRaises(ValueError):
                planificar_lspb(*args)

    def test_tiempo_fuera_y_evaluacion_escalar(self):
        p = planificar_lspb([0], [2], 3, [1])
        m = p.evaluar(1.5)
        self.assertEqual(m.q.shape, (1,))
        self.assertAlmostEqual(m.q[0], 1)
        for invalid_t in [-1e-3, 3.01, [0, np.nan], [[0,1]]]:
            with self.subTest(t=invalid_t), self.assertRaises(ValueError):
                p.evaluar(invalid_t)

    def test_entradas_no_mutadas(self):
        qi = np.zeros(6)
        qf = np.array([.2,0,0,0,0,0])
        qa = np.ones(6)
        p = planificar_lspb(qi, qf, 3, qa)
        qi[0] = 2
        qf[0] = 2
        qa[0] = 8
        self.assertAlmostEqual(p.q_inicio[0], 0)
        self.assertAlmostEqual(p.q_final[0], .2)
        self.assertAlmostEqual(p.aceleraciones[0], 1)


if __name__ == "__main__":
    unittest.main()
