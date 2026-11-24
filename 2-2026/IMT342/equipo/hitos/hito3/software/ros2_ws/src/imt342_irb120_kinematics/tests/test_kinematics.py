"""Pruebas matemáticas sin ROS ni dependencias de terceros."""

import unittest
from math import cos, inf, pi, sin
from pathlib import Path

from imt342_irb120_kinematics import Model


URDF = (Path(__file__).resolve().parents[2] /
        "imt342_irb120_description/urdf/irb120.urdf")
REGULAR = (0.3, -0.2, 0.7, 0.4, -0.5, 1.0)
CONFIGURATIONS = ((0,) * 6, REGULAR, (-0.9, 0.4, -1.1, 0.9, 0.2, -3.7))


def historic_dh(q):
    """Tabla D-H de integracion_candidata_offline_v1/src/irb120_fk.py."""
    a = (0, 0.270, 0.070, 0, 0, 0, 0)
    d = (0.290, 0, 0, 0.302, 0, 0.072, 0.120)
    alpha = (-pi/2, 0, -pi/2, pi/2, -pi/2, 0, 0)
    theta = (q[0], q[1] - pi/2, *q[2:], 0)
    result = [[float(i == j) for j in range(4)] for i in range(4)]
    for angle, offset, length, twist in zip(theta, d, a, alpha):
        c, s, ca, sa = cos(angle), sin(angle), cos(twist), sin(twist)
        row = ((c, -s*ca, s*sa, length*c), (s, c*ca, -c*sa, length*s),
               (0, sa, ca, offset), (0, 0, 0, 1))
        result = [[sum(result[i][k] * row[k][j] for k in range(4))
                   for j in range(4)] for i in range(4)]
    return result


class KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model.from_file(URDF)

    def test_fk_matches_independent_dh(self):
        for q in CONFIGURATIONS:
            with self.subTest(q=q):
                actual = self.model.fk(q)
                expected = historic_dh(q)
                self.assertLess(max(abs(a - b) for row, reference in zip(actual, expected)
                                    for a, b in zip(row, reference)), 1e-12)

    def test_jacobian_matches_centered_differences(self):
        epsilon = 1e-6
        for q in CONFIGURATIONS:
            rotation = self.model.fk(q)
            jacobian = self.model.jacobian(q)
            for col in range(6):
                with self.subTest(q=q, col=col):
                    before, after = list(q), list(q)
                    before[col] -= epsilon
                    after[col] += epsilon
                    low, high = self.model.fk(before), self.model.fk(after)
                    derivative = [[(high[i][j] - low[i][j]) / (2 * epsilon)
                                   for j in range(4)] for i in range(3)]
                    angular = [sum(derivative[i][k] * rotation[j][k] for k in range(3))
                               for i, j in ((2, 1), (0, 2), (1, 0))]
                    numeric = [row[3] for row in derivative] + angular
                    self.assertLess(max(abs(jacobian[i][col] - value)
                                        for i, value in enumerate(numeric)), 1e-8)

    def test_ik_roundtrip_and_joint_limits(self):
        for q in CONFIGURATIONS:
            with self.subTest(q=q):
                result = self.model.ik(self.model.fk(q), tuple(angle + 0.15 for angle in q))
                self.assertTrue(result.converged, result)
                self.assertLessEqual(result.position_error_m, 1e-6)
                self.assertLessEqual(result.orientation_error_rad, 1e-6)
                for angle, (low, high) in zip(result.q, self.model.limits):
                    self.assertTrue(low <= angle <= high)

    def test_ik_unreachable_never_marks_convergence(self):
        target = [list(row) for row in self.model.fk(REGULAR)]
        target[0][3] = 10
        result = self.model.ik(target, (0,) * 6)
        self.assertFalse(result.converged)
        self.assertGreater(result.position_error_m, 9)
        self.assertTrue(all(low <= value <= high
                            for value, (low, high) in zip(result.q, self.model.limits)))

    def test_wrist_singularity(self):
        singular = self.model.singular_values((0,) * 6)
        regular = self.model.singular_values(REGULAR)
        self.assertLess(singular[-1], 1e-12)
        self.assertGreater(regular[-1], 0.1)
        self.assertEqual(regular, tuple(sorted(regular, reverse=True)))

    def test_rejects_nonfinite_inputs_and_reflected_rotations(self):
        with self.assertRaises(ValueError):
            self.model.fk((inf,) * 6)
        reflected = [list(row) for row in self.model.fk(REGULAR)]
        reflected[0][:3] = [-element for element in reflected[0][:3]]
        with self.assertRaises(ValueError):
            self.model.ik(reflected, REGULAR)

    def test_ik_with_half_turn_error(self):
        target_joints = (*REGULAR[:5], REGULAR[5] + pi)
        result = self.model.ik(self.model.fk(target_joints), REGULAR)
        self.assertTrue(result.converged, result)
        self.assertLessEqual(result.orientation_error_rad, 1e-6)


if __name__ == "__main__":
    unittest.main()
