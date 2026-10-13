"""Pruebas de integracion: Pieper -> limites nominales ABB -> seleccion."""

import unittest

import numpy as np

from src.irb120_fk import forward_kinematics
from src.limites_irb120 import violaciones_limites
from src.pieper import PieperResult, PieperSolution, solve_pieper
from src.seleccion_pieper import filtrar_y_seleccionar_pieper


class TestSeleccionPieper(unittest.TestCase):
    def test_ocho_ramas_dos_admisibles_para_pose_de_referencia(self):
        rho = np.hypot(0.4, 0.1)
        h = 0.138
        L = np.hypot(0.070, 0.302)
        beta = np.arctan2(0.302, 0.070)
        delta = np.arccos((rho**2 + h**2 - 0.270**2 - L**2) / (2 * 0.270 * L))
        q_obj = np.array([
            np.arctan2(0.1, 0.4),
            np.arctan2(rho, h) - np.arctan2(L * np.sin(delta), 0.270 + L * np.cos(delta)),
            delta - beta, 0.5, -0.6, 0.7,
        ])
        T_d = forward_kinematics(q_obj)
        pieper = solve_pieper(T_d, q_seed=q_obj)
        self.assertEqual(len(pieper.solutions), 8)

        salida = filtrar_y_seleccionar_pieper(pieper, np.zeros(6))
        self.assertEqual(len(salida.descartadas), 6)
        self.assertEqual(len(salida.admisibles), 4)  # Dos ramas, dos variantes de J6 por rama
        self.assertEqual(len({(s.rama_brazo, s.rama_muneca) for s in salida.admisibles}), 2)
        self.assertIsNotNone(salida.seleccionada)
        for s in salida.admisibles:
            self.assertEqual(violaciones_limites(s.q), [])
            np.testing.assert_allclose(forward_kinematics(s.q), T_d, rtol=0, atol=1e-8)
        motivos = [v["junta"] for d in salida.descartadas for v in d.violaciones]
        self.assertIn("J2", motivos)
        self.assertIn("J1", motivos)

    def test_elije_j6_cercana_en_vueltas_equivalentes(self):
        q = np.zeros(6)
        sol = PieperSolution(q, "brazo", "muneca", 0.0, 0.0, False)
        resultado = PieperResult([sol], np.zeros(3), [])
        inicio = np.deg2rad([0, 0, 0, 0, 0, 350])
        salida = filtrar_y_seleccionar_pieper(resultado, inicio)
        self.assertEqual(len(salida.admisibles), 3)
        self.assertAlmostEqual(np.rad2deg(salida.seleccionada.q[5]), 360.0)
        self.assertAlmostEqual(salida.seleccionada.costo, np.deg2rad(10)**2)

    def test_descarta_y_explica_violacion_j3(self):
        q = np.deg2rad([0, 0, 75, 0, 0, 0])
        sol = PieperSolution(q, "brazo", "muneca", 0.0, 0.0, False)
        salida = filtrar_y_seleccionar_pieper(PieperResult([sol], np.zeros(3), []), np.zeros(6))
        self.assertIsNone(salida.seleccionada)
        self.assertEqual(len(salida.admisibles), 0)
        self.assertEqual([v["junta"] for v in salida.descartadas[0].violaciones], ["J3"])

    def test_no_hay_soluciones_preserva_diagnostico(self):
        resultado = PieperResult([], np.zeros(3), ["Pose inalcanzable"])
        salida = filtrar_y_seleccionar_pieper(resultado, np.zeros(6))
        self.assertIsNone(salida.seleccionada)
        self.assertIn("Pose inalcanzable", salida.diagnosticos)

    def test_valida_inicio_y_pesos(self):
        r = PieperResult([], np.zeros(3), [])
        with self.assertRaises(ValueError):
            filtrar_y_seleccionar_pieper(r, np.deg2rad([166, 0, 0, 0, 0, 0]))
        with self.assertRaises(ValueError):
            filtrar_y_seleccionar_pieper(r, np.zeros(6), pesos=[1, 1, 1, -1, 1, 1])
        with self.assertRaises(ValueError):
            filtrar_y_seleccionar_pieper(r, np.zeros(6), pesos=np.zeros(6))
        with self.assertRaises(ValueError):
            filtrar_y_seleccionar_pieper(r, [np.nan, 0, 0, 0, 0, 0])

    def test_no_modifica_q_original_ni_inicio(self):
        q = np.zeros(6)
        inicio = np.deg2rad([10, 0, 0, 0, 0, 350])
        q_previa, inicio_previo = q.copy(), inicio.copy()
        sol = PieperSolution(q, "brazo", "muneca", 0.0, 0.0)
        filtrar_y_seleccionar_pieper(PieperResult([sol], np.zeros(3), []), inicio)
        np.testing.assert_array_equal(q, q_previa)
        np.testing.assert_array_equal(inicio, inicio_previo)

    def test_singular_reporta_representante_no_exhaustivo(self):
        q = np.zeros(6)
        resultado = solve_pieper(forward_kinematics(q), q_seed=q)
        salida = filtrar_y_seleccionar_pieper(resultado, q)
        self.assertIsNotNone(salida.seleccionada)
        self.assertTrue(any("representantes" in d for d in salida.diagnosticos))
        np.testing.assert_allclose(forward_kinematics(salida.seleccionada.q), forward_kinematics(q), atol=1e-8)


if __name__ == "__main__":
    unittest.main()
