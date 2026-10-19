"""Pruebas del modelo dinámico reducido 1R exigido en S10, Hito 2.

Caso de referencia: práctica integrada W10-S01 y solucionario, S10 pp. 13-17.
"""

import unittest

import numpy as np

from src.dinamica_1r import ParametrosDinamica1R, evaluar_dinamica_1r
from src.lspb import planificar_lspb


class TestDinamicaReducida1R(unittest.TestCase):
    def setUp(self):
        # Parámetros didácticos publicados por el docente, NO datos ABB.
        self.params = ParametrosDinamica1R(0.25, 1.0, 0.5, 9.81)
        self.perfil = planificar_lspb([0.0], [2.0], 3.0, [1.0])

    def test_ejemplo_oficial_t_2_5(self):
        r = evaluar_dinamica_1r(self.perfil, 2.5, self.params)
        self.assertAlmostEqual(r.q_rad, 1.875, places=12)
        self.assertAlmostEqual(r.qdot_rad_s, 0.5, places=12)
        self.assertAlmostEqual(r.qddot_rad_s2, -1.0, places=12)
        self.assertAlmostEqual(r.tau_inercial_nm, -0.25, places=12)
        self.assertAlmostEqual(r.tau_gravitatorio_nm, 4.905 * np.sin(1.875), places=12)
        self.assertAlmostEqual(r.tau_total_nm, -0.25 + 4.905 * np.sin(1.875), places=12)
        self.assertAlmostEqual(r.tau_total_nm, 4.43, delta=0.02)

    def test_tres_fases_inercial(self):
        r = evaluar_dinamica_1r(self.perfil, [0.5, 1.5, 2.5], self.params)
        np.testing.assert_allclose(r.q_rad, [0.125, 1.0, 1.875], atol=1e-12)
        np.testing.assert_allclose(r.qddot_rad_s2, [1.0, 0.0, -1.0], atol=1e-12)
        np.testing.assert_allclose(r.tau_inercial_nm, [0.25, 0.0, -0.25], atol=1e-12)
        np.testing.assert_allclose(r.tau_total_nm, r.tau_inercial_nm + r.tau_gravitatorio_nm)
        self.assertEqual(r.q_rad.shape, (3,))
        self.assertEqual(r.t_s.shape, (3,))

    def test_crucero_velocidad_constante_torque_gravedad(self):
        r = evaluar_dinamica_1r(self.perfil, 1.5, self.params)
        self.assertAlmostEqual(r.qddot_rad_s2, 0.0)
        self.assertAlmostEqual(r.qdot_rad_s, 1.0)
        self.assertAlmostEqual(r.tau_inercial_nm, 0.0)
        self.assertAlmostEqual(r.tau_total_nm, 4.905 * np.sin(1.0), places=12)
        self.assertGreater(r.tau_total_nm, 0.0)

    def test_movimiento_negativo_cambia_signo_inercial(self):
        p = planificar_lspb([2.0], [0.0], 3.0, [1.0])
        r = evaluar_dinamica_1r(p, [0.5, 1.5, 2.5], self.params)
        np.testing.assert_allclose(r.tau_inercial_nm, [-0.25, 0.0, 0.25], atol=1e-12)
        np.testing.assert_allclose(r.q_rad, [1.875, 1.0, 0.125], atol=1e-12)

    def test_vector_y_escalar_concuerdan(self):
        r = evaluar_dinamica_1r(self.perfil, [0.5, 1.5, 2.5], self.params)
        campos = ["q_rad", "qdot_rad_s", "qddot_rad_s2",
                  "tau_inercial_nm", "tau_gravitatorio_nm", "tau_total_nm"]
        for i, t in enumerate([0.5, 1.5, 2.5]):
            s = evaluar_dinamica_1r(self.perfil, t, self.params)
            self.assertIsInstance(s.tau_total_nm, float)
            for campo in campos:
                self.assertAlmostEqual(getattr(s, campo), getattr(r, campo)[i], places=12)

    def test_perfil_6r_articulacion_explicitamente_seleccionada(self):
        p = planificar_lspb(
            np.zeros(6), np.array([0.2, -0.3, 0.1, 0.4, 0.0, 0.35]),
            3.0, np.ones(6),
        )
        t = np.array([0.5, 1.5, 2.5])
        r = evaluar_dinamica_1r(p, t, self.params, indice_articulacion=1)
        m = p.evaluar(t)
        self.assertEqual(r.indice_articulacion, 1)
        np.testing.assert_allclose(r.q_rad, m.q[:, 1])
        np.testing.assert_allclose(r.qddot_rad_s2, m.qddot[:, 1])
        np.testing.assert_allclose(
            r.tau_total_nm,
            self.params.inercia_kg_m2 * m.qddot[:, 1]
            + 4.905 * np.sin(m.q[:, 1]), atol=1e-12,
        )

    def test_no_permite_6r_sin_seleccion(self):
        p = planificar_lspb(np.zeros(6), np.ones(6) * 0.1, 3.0, np.ones(6))
        with self.assertRaisesRegex(ValueError, "debe indicarse"):
            evaluar_dinamica_1r(p, 1.5, self.params)
        for invalido in (-1, 6, 1.5, True, '2'):
            with self.subTest(indice=invalido), self.assertRaises(ValueError):
                evaluar_dinamica_1r(p, 1.5, self.params, indice_articulacion=invalido)

    def test_1r_solo_indice_cero_o_none(self):
        self.assertEqual(evaluar_dinamica_1r(self.perfil, 1.5, self.params, indice_articulacion=0).indice_articulacion, 0)
        for invalido in (1, True, -1, '0'):
            with self.subTest(indice=invalido), self.assertRaises(ValueError):
                evaluar_dinamica_1r(self.perfil, 1.5, self.params, indice_articulacion=invalido)

    def test_parametros_invalidos(self):
        for args in ((0, 1, 0.5), (-0.1, 1, 0.5), (np.nan, 1, 0.5),
                     (0.25, 0, 0.5), (0.25, -1, 0.5), (0.25, 1, 0),
                     (0.25, 1, np.inf), (0.25, 1, 0.5, -9.81),
                     (0.25, 1, 0.5, np.nan)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                ParametrosDinamica1R(*args)

    def test_gravedad_cero_anula_componente_gravitatoria(self):
        p = ParametrosDinamica1R(0.25, 1.0, 0.5, 0.0)
        r = evaluar_dinamica_1r(self.perfil, [0.5, 1.5, 2.5], p)
        np.testing.assert_allclose(r.tau_gravitatorio_nm, 0.0)
        np.testing.assert_allclose(r.tau_total_nm, r.tau_inercial_nm)

    def test_perfil_estatico_torque_gravedad(self):
        p = planificar_lspb([1.0], [1.0], 3.0, [0.5])
        r = evaluar_dinamica_1r(p, [0, 1.5, 3], self.params)
        np.testing.assert_allclose(r.qdot_rad_s, 0)
        np.testing.assert_allclose(r.tau_inercial_nm, 0)
        np.testing.assert_allclose(r.tau_total_nm, 4.905 * np.sin(1.0))

    def test_tiempos_invalidos_lspb(self):
        for t in (-0.1, 3.1, [0, np.nan], [[0, 1]]):
            with self.subTest(t=t), self.assertRaises(ValueError):
                evaluar_dinamica_1r(self.perfil, t, self.params)

    def test_tipos_invalidos(self):
        with self.assertRaises(ValueError):
            evaluar_dinamica_1r(None, 1.0, self.params)
        with self.assertRaises(ValueError):
            evaluar_dinamica_1r(self.perfil, 1.0, None)

    def test_no_muta_perfil_ni_parametros(self):
        qi = self.perfil.q_inicio.copy()
        qf = self.perfil.q_final.copy()
        r = evaluar_dinamica_1r(self.perfil, [0.5, 1.5], self.params)
        r.q_rad[0] = 500.0
        np.testing.assert_array_equal(self.perfil.q_inicio, qi)
        np.testing.assert_array_equal(self.perfil.q_final, qf)
        self.assertEqual(self.params.inercia_kg_m2, 0.25)


if __name__ == "__main__":
    unittest.main()
