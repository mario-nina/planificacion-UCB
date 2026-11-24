"""Offline regressions for the analytic branches and the teaching kernels."""

from math import atan2, hypot, pi, tau
from pathlib import Path
from random import Random
import unittest

from imt342_irb120_kinematics import (
    Model, SingleLinkDynamics, pose_from_quaternion, pose_from_rpy,
    pose_to_quaternion, pose_to_rpy,
)


URDF = Path(__file__).resolve().parents[2] / "imt342_irb120_description/urdf/irb120.urdf"
EIGHT_BRANCH_SEED = (-1.919, 1.081, -1.857, 1.435, -0.632, 1.035)


def max_error(first, second):
    return max(abs(a - b) for row_a, row_b in zip(first, second) for a, b in zip(row_a, row_b))


class PieperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model.from_file(URDF)

    def test_multiseed_keeps_distinct_numerical_results(self):
        seed = EIGHT_BRANCH_SEED
        wrapped = seed[:5] + (seed[5] - tau,)
        pose = self.model.fk(seed)
        results = self.model.ik_multiseed(pose, [seed, seed, wrapped])
        self.assertEqual([result.q for result in results], [seed, wrapped])
        self.assertTrue(all(result.source == "numerical" for result in results))

    def test_eight_geometric_branches_with_bounds_and_full_turns(self):
        target = self.model.fk(EIGHT_BRANCH_SEED)
        result = self.model.ik_pieper(target, EIGHT_BRANCH_SEED)
        self.assertEqual(len({candidate.branch.rsplit("/", 1)[0] for candidate in result}), 8)
        self.assertEqual(len({candidate.branch for candidate in result}), len(result))
        self.assertGreater(len(result), 8)
        self.assertTrue(any(abs(candidate.q[5] - (EIGHT_BRANCH_SEED[5] - tau)) < 1e-10
                            for candidate in result))
        for candidate in result:
            self.assertEqual(candidate.source, "pieper")
            self.assertFalse(candidate.singular)
            self.assertLess(max_error(target, self.model.fk(candidate.q)), 1e-9)
            self.assertLess(candidate.position_error_m, 1e-9)
            self.assertLess(candidate.orientation_error_rad, 1e-9)
            self.assertTrue(all(low <= q <= high for q, (low, high) in
                                zip(candidate.q, self.model.limits)))

    def test_seeded_roundtrip_with_wrist_singularities(self):
        random = Random(23)
        for index in range(150):
            joints = [random.uniform(*bounds) for bounds in self.model.limits]
            if index % 10 == 0:
                joints[4] = 0.0
            with self.subTest(q=joints):
                result = self.model.ik_pieper(self.model.fk(joints), joints)
                self.assertTrue(result)
                self.assertLess(hypot(*(a - b for a, b in zip(joints, result[0].q))), 1e-7)
                if joints[4] == 0:
                    self.assertTrue(result[0].singular)

    def test_shoulder_axis_singularity_with_seed_representative(self):
        q = (1.8, -atan2(0.302, 0.270 + 0.070), 0.0, 0.4, 0.8, 0.6)
        result = self.model.ik_pieper(self.model.fk(q), q)
        self.assertTrue(result)
        self.assertTrue(result[0].singular)
        self.assertTrue(result[0].branch.startswith("H0/"))
        self.assertLess(hypot(*(a - b for a, b in zip(q, result[0].q))), 1e-8)

    def test_reaches_joint_limit_edges(self):
        for column in (0, 1):
            edge = tuple(bounds[column] for bounds in self.model.limits)
            result = self.model.ik_pieper(self.model.fk(edge), edge)
            self.assertTrue(result)
            self.assertLess(hypot(*(a - b for a, b in zip(edge, result[0].q))), 1e-8)

    def test_unreachable_nonhomogeneous_and_reflected_targets(self):
        for x in (10, 1e308):
            self.assertEqual(self.model.ik_pieper(pose_from_rpy((x, 0, 0), (0, 0, 0))), ())
        pose = [list(row) for row in self.model.fk(EIGHT_BRANCH_SEED)]
        pose[3][2] = 1
        with self.assertRaises(ValueError):
            self.model.ik_pieper(pose)
        pose[3][2] = 0
        pose[0][:3] = [-x for x in pose[0][:3]]
        with self.assertRaises(ValueError):
            self.model.ik_pieper(pose)

    def test_pose_roundtrips_with_half_turn_and_gimbal_lock(self):
        for angles in ((0, 0, pi), (0.7, pi / 2, -0.9), (-1.2, -pi / 2, -2.8), (0, 0, 0)):
            with self.subTest(angles=angles):
                pose = pose_from_rpy((0.1, 0.2, 0.3), angles)
                self.assertLess(max_error(pose, pose_from_rpy(*pose_to_rpy(pose))), 1e-12)
                self.assertLess(max_error(pose, pose_from_quaternion(*pose_to_quaternion(pose))), 1e-12)
        with self.assertRaises(ValueError):
            pose_from_quaternion((0, 0, 0), (0, 0, 0, 0))

    def test_jacobian_fd_and_wrist_sweep(self):
        jacobian = self.model.jacobian(EIGHT_BRANCH_SEED)
        for step, tolerance in ((1e-3, 1e-6), (1e-5, 1e-9), (1e-7, 1e-8)):
            self.assertLess(max_error(jacobian, self.model.jacobian_fd(EIGHT_BRANCH_SEED, step)),
                            tolerance)
        sweep = [self.model.singular_values(EIGHT_BRANCH_SEED[:4] + (q5,) + EIGHT_BRANCH_SEED[5:])
                 for q5 in (0.1, 0.01, 0.001, 0)]
        self.assertTrue(all(left[-1] > right[-1] for left, right in zip(sweep, sweep[1:])))
        self.assertEqual(sum(value > 1e-8 for value in sweep[-1]), 5)

    def test_single_link_dynamics(self):
        dynamics = SingleLinkDynamics(0.25, 2.0, 0.2)
        self.assertAlmostEqual(dynamics.torque(pi / 2, 2.0), 0.5 + 3.924)
        self.assertEqual(dynamics.components(0, -4.0), (-1.0, 0.0))
        with self.assertRaises(ValueError):
            SingleLinkDynamics(0, 1, 1)


if __name__ == "__main__":
    unittest.main()
