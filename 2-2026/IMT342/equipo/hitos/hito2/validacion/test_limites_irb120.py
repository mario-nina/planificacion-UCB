"""Pruebas de rangos nominales ABB y vueltas equivalentes de J6."""
import unittest

import numpy as np

from src.limites_irb120 import (
    LIMITES_ABB_GRADOS,
    variantes_equivalentes_j6,
    violaciones_limites,
)
from src.irb120_fk import forward_kinematics


class TestLimitesABB(unittest.TestCase):
    def test_cero_y_fronteras_inclusivas(self):
        self.assertEqual(violaciones_limites(np.zeros(6)), [])
        self.assertEqual(violaciones_limites(np.deg2rad(LIMITES_ABB_GRADOS[:, 0])), [])
        self.assertEqual(violaciones_limites(np.deg2rad(LIMITES_ABB_GRADOS[:, 1])), [])

    def test_j3_asimetrica(self):
        q = np.zeros(6)
        q[2] = np.deg2rad(75)
        errores = violaciones_limites(q)
        self.assertEqual([e["junta"] for e in errores], ["J3"])
        self.assertEqual(errores[0]["maximo_grados"], 70)

    def test_informa_varias_juntas_fuera(self):
        q = np.deg2rad([166, -111, 0, 0, 0, 401])
        self.assertEqual(
            [e["junta"] for e in violaciones_limites(q)], ["J1", "J2", "J6"]
        )

    def test_j6_mas_alla_de_180_es_admisible(self):
        q = np.deg2rad([0, 0, 0, 0, 0, 390])
        self.assertEqual(violaciones_limites(q), [])

    def test_variantes_de_j6_y_equivalencia_fk(self):
        q = np.deg2rad([10, -20, 30, 40, 15, 0])
        variantes = variantes_equivalentes_j6(q)
        np.testing.assert_allclose(
            [v[5] for v in variantes], np.deg2rad([-360, 0, 360]), atol=1e-12
        )
        T0 = forward_kinematics(q)
        for v in variantes:
            np.testing.assert_allclose(forward_kinematics(v), T0, atol=1e-12)

    def test_error_entrada(self):
        with self.assertRaises(ValueError):
            violaciones_limites(np.zeros(5))
        with self.assertRaises(ValueError):
            variantes_equivalentes_j6([0, 0, 0, 0, 0, np.nan])


if __name__ == "__main__":
    unittest.main()
