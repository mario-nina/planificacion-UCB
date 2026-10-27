"""Ensayo independiente de singularidad de muñeca del ABB IRB 120."""

import unittest

import numpy as np

from src.jacobiano import jacobiano_geometrico
from src.singularidades import analizar_singularidad, experimento_singularidad_muneca


class TestSingularidadMuneca(unittest.TestCase):
    def setUp(self):
        self.q0 = np.array([.30, -.35, .25, .4, 0., -.6])

    def test_q5_cero_perdida_rango(self):
        d = analizar_singularidad(self.q0)
        self.assertEqual(d.rango, 5)
        self.assertTrue(np.isinf(d.condicion))
        self.assertLess(d.valores_singulares[-1], 1e-12)

    def test_q5_regular_rango_seis(self):
        for a in (.1, .01, .001):
            with self.subTest(a=a):
                q = self.q0.copy()
                q[4] = a
                self.assertEqual(analizar_singularidad(q).rango, 6)

    def test_j4_y_j6_coinciden_en_singular(self):
        j = jacobiano_geometrico(self.q0)
        np.testing.assert_allclose(j[:, 3], j[:, 5], atol=1e-12)
        e4_menos_e6 = np.array([0, 0, 0, 1, 0, -1])
        np.testing.assert_allclose(j @ e4_menos_e6, np.zeros(6), atol=1e-12)

    def test_direccion_cartesiana_no_alcanzable(self):
        d = analizar_singularidad(self.q0)
        j = jacobiano_geometrico(self.q0)
        np.testing.assert_allclose(j.T @ d.direccion_cartesiana_debil, np.zeros(6), atol=1e-12)
        np.testing.assert_allclose(j @ d.modo_nulo_articular, np.zeros(6), atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(d.direccion_cartesiana_debil), 1)

    def test_experimento_rango_y_amplificacion(self):
        ex = experimento_singularidad_muneca(self.q0)
        np.testing.assert_array_equal(ex.rangos, [6, 6, 6, 6, 6, 6, 5])
        self.assertTrue(np.all(np.diff(ex.sigma_min) < 0))
        self.assertTrue(np.all(np.diff(ex.normas_qdot[:-1]) > 0))
        self.assertTrue(np.isnan(ex.normas_qdot[-1]))
        self.assertTrue(np.isinf(ex.condiciones[-1]))
        self.assertGreater(ex.normas_qdot[-2], 1000 * ex.normas_qdot[0])
        self.assertLess(np.max(ex.errores_consigna[:-1]), 1e-9)
        self.assertAlmostEqual(ex.errores_consigna[-1], .01, places=11)

    def test_twist_fijo_y_no_inversion_singular(self):
        ex = experimento_singularidad_muneca(angulos_q5_rad=[.1, .01, 0])
        self.assertEqual(ex.consigna_cartesiana.shape, (6,))
        self.assertAlmostEqual(np.linalg.norm(ex.consigna_cartesiana), .01, places=12)
        for d in ex.diagnosticos:
            self.assertEqual(d.q.shape, (6,))

    def test_flange_y_tcp_comparten_singularidad(self):
        for marco in ("flange", "tcp"):
            with self.subTest(marco=marco):
                ex = experimento_singularidad_muneca(target_frame=marco)
                self.assertEqual(ex.rangos[-1], 5)
                self.assertEqual(ex.rangos[0], 6)
                np.testing.assert_allclose(
                    jacobiano_geometrico(self.q0, target_frame=marco)[:, 3],
                    jacobiano_geometrico(self.q0, target_frame=marco)[:, 5],
                    atol=1e-12,
                )

    def test_longitud_referencia_convencion_no_rango(self):
        for L in (.5, 1., 2.):
            with self.subTest(L=L):
                d = analizar_singularidad(self.q0, longitud_referencia_m=L)
                self.assertEqual(d.rango, 5)
                ex = experimento_singularidad_muneca(longitud_referencia_m=L)
                self.assertEqual(ex.rangos[-1], 5)
                self.assertTrue(np.isfinite(ex.normas_qdot[0]))

    def test_no_muta_entrada(self):
        q = self.q0.copy()
        antes = q.copy()
        ex = experimento_singularidad_muneca(q)
        np.testing.assert_array_equal(q, antes)
        self.assertFalse(np.shares_memory(q, ex.diagnosticos[0].q))

    def test_entradas_invalidas(self):
        kwargs_invalidos = [
            {"angulos_q5_rad": [.1, 0.]},
            {"angulos_q5_rad": [.1, .2, 0]},
            {"angulos_q5_rad": [.1, .01, .001]},
            {"angulos_q5_rad": [.1, -.01, 0]},
            {"angulos_q5_rad": [.1, np.nan, 0]},
            {"angulos_q5_rad": [.1, .01, -.0, 0]},
            {"angulos_q5_rad": [3., .1, 0]},
            {"magnitud_twist_normalizado": 0},
            {"longitud_referencia_m": -1},
            {"tolerancia_relativa": 0},
            {"tolerancia_relativa": 1},
            {"target_frame": "mundo"},
            {"q_referencia": [0]*5},
            {"q_referencia": [0, 0, 1.5, 0, 0, 0]},
        ]
        for kwargs in kwargs_invalidos:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    experimento_singularidad_muneca(**kwargs)
        for kwargs in ({"tolerancia_relativa": -1}, {"longitud_referencia_m": 0}):
            with self.assertRaises(ValueError):
                analizar_singularidad(self.q0, **kwargs)


if __name__ == "__main__":
    unittest.main()
