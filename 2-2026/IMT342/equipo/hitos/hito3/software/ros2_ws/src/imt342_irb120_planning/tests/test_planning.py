from math import inf, nan, sqrt
from types import SimpleNamespace
import unittest

from imt342_irb120_planning import (AUTO_REFERENCE_VELOCITIES, JOINT_LIMITS, plan,
                                     plan_candidates, plan_sequence, planner_for_profile, tcp_samples)


ZERO = (0.0,) * 6
GOAL = (0.4, -0.5, 0.2, 0.6, 0.0, -0.3)


class PlanningTest(unittest.TestCase):
    def test_lspb_is_continuous_at_phase_boundaries_and_allows_triangular_motion(self):
        for blend in (.25, .5):
            trajectory = plan(ZERO, GOAL, duration=4, blend_fraction=blend)
            for knot in (blend * trajectory.duration, (1 - blend) * trajectory.duration):
                left = trajectory.sample(knot - 1e-8)
                right = trajectory.sample(knot + 1e-8)
                for a, b in zip(left.positions + left.velocities, right.positions + right.velocities):
                    self.assertAlmostEqual(a, b, places=7)
            self.assertAlmostEqual(trajectory.sample(2).velocities[1], GOAL[1] / (4 * (1 - blend)))
            self.assertEqual(trajectory.sample(2).accelerations[1] == 0, blend < .5)
        for invalid in (True, "bad", float("nan"), 0, 5e-324, .51):
            with self.subTest(blend_fraction=invalid), self.assertRaisesRegex(ValueError, "LSPB"):
                plan(ZERO, GOAL, blend_fraction=invalid)

    def test_mixed_profiles_preserve_duration_and_exact_waypoint_stops(self):
        trajectory = plan_sequence(ZERO, (GOAL, ZERO), durations=(3., 3.5), profiles=("lspb", "quintic"))
        self.assertEqual(trajectory.profiles, ("lspb", "quintic"))
        self.assertEqual(trajectory.waypoint_times, (3., 6.5))
        self.assertEqual(trajectory.sample(3.).positions, GOAL)
        self.assertEqual(trajectory.sample(3.).velocities, ZERO)
        self.assertEqual(trajectory.sample(6.5).positions, ZERO)
        for profiles in (("lspb",), ("lspb", "unknown")):
            with self.subTest(profiles=profiles), self.assertRaises(ValueError):
                plan_sequence(ZERO, (GOAL, ZERO), profiles=profiles)

    def test_analytic_lspb_reference(self):
        # a = 1 rad/s², t_blend = 1 s, duration = 4 s, dq = 3 rad.
        goal = (0.0,) * 5 + (3.0,)
        trajectory = plan(ZERO, goal, duration=4.0)
        expected = [(0.5, 0.125, 0.5, 1), (2, 1.5, 1, 0), (3.5, 2.875, 0.5, -1)]
        for time, position, velocity, acceleration in expected:
            with self.subTest(time=time):
                sample = trajectory.sample(time)
                self.assertAlmostEqual(sample.positions[5], position)
                self.assertAlmostEqual(sample.velocities[5], velocity)
                self.assertAlmostEqual(sample.accelerations[5], acceleration)

    def test_automatic_reference_is_moderate_and_synchronized(self):
        for profile in ("lspb", "quintic"):
            with self.subTest(profile=profile):
                planner = planner_for_profile(profile)
                reference = planner(ZERO, GOAL, max_velocity=AUTO_REFERENCE_VELOCITIES)
                self.assertGreater(reference.duration, 0.1)
                self.assertLess(reference.duration, 10.0)
                self.assertEqual(reference.sample(0).positions, ZERO)
                self.assertEqual(reference.sample(reference.duration).positions, GOAL)
                self.assertEqual(reference.sample(reference.duration).velocities, ZERO)
                for sample in reference.samples(.01):
                    self.assertLessEqual(max(map(abs, sample.velocities)), .6 + 1e-9)
                    self.assertLessEqual(max(map(abs, sample.accelerations)), 1.0 + 1e-9)

    def test_automatic_lspb_short_move_has_no_cruise(self):
        goal = (.04, -.02, *ZERO[2:])
        trajectory = plan(ZERO, goal, max_velocity=AUTO_REFERENCE_VELOCITIES)
        self.assertAlmostEqual(trajectory.duration, .4)
        self.assertEqual(trajectory.blend_fraction, .5)
        for time, expected_speed in ((.1, .1), (.2, .2), (.3, .1)):
            state = trajectory.sample(time)
            self.assertAlmostEqual(state.velocities[0], expected_speed)
            self.assertAlmostEqual(state.positions[1], -.5 * state.positions[0])
            self.assertAlmostEqual(state.velocities[1], -.5 * expected_speed)
            self.assertAlmostEqual(abs(state.accelerations[0]), 1.0)
        explicit = plan(ZERO, goal, duration=.4, max_velocity=AUTO_REFERENCE_VELOCITIES)
        self.assertEqual(explicit, trajectory)
        with self.assertRaisesRegex(ValueError, "factible"):
            plan(ZERO, goal, duration=.39, max_velocity=AUTO_REFERENCE_VELOCITIES)

    def test_automatic_trapezoid_respects_different_limiting_joints(self):
        goal = (.9, -.3, .2, *ZERO[3:])
        velocity = (.6, .18, .6, .6, .6, .6)
        acceleration = (1., 1., .2, 1., 1., 1.)
        trajectory = plan(ZERO, goal, max_velocity=velocity, max_acceleration=acceleration)
        self.assertAlmostEqual(trajectory.duration, .3 / .18 + .6)
        self.assertAlmostEqual(trajectory.blend_fraction * trajectory.duration, .6)
        self.assertAlmostEqual(trajectory.sample(.3).accelerations[2], .2)
        cruise = trajectory.sample(trajectory.duration / 2)
        self.assertAlmostEqual(cruise.velocities[1], -.18)
        self.assertEqual(cruise.accelerations, ZERO)
        for state in trajectory.samples(.01):
            for q, dq, ddq, target, v_max, a_max in zip(
                    state.positions, state.velocities, state.accelerations, goal, velocity, acceleration):
                self.assertAlmostEqual(q, target * state.positions[0] / goal[0])
                self.assertLessEqual(abs(dq), v_max + 1e-12)
                self.assertLessEqual(abs(ddq), a_max + 1e-12)

    def test_lspb_velocity_remains_bounded_at_the_end_of_a_long_cruise(self):
        cap = .005
        trajectory = plan(ZERO, (*ZERO[:5], 4.), max_velocity=(cap,) * 6, max_acceleration=(10.,) * 6)
        end_of_cruise = trajectory.duration * (1.0 - trajectory.blend_fraction)
        speed = trajectory.sample(end_of_cruise).velocities[5]
        self.assertAlmostEqual(speed, cap, places=11)
        self.assertLessEqual(speed, cap * (1.0 + 1e-12))

    def test_synchronized_bounds_and_endpoints(self):
        trajectory = plan(ZERO, GOAL)
        samples = trajectory.samples(period=0.031)
        self.assertEqual(samples[0].positions, ZERO)
        self.assertEqual(samples[-1].positions, GOAL)
        self.assertEqual(samples[0].velocities, ZERO)
        self.assertEqual(samples[-1].velocities, ZERO)
        self.assertEqual(samples[-1].accelerations, ZERO)
        self.assertTrue(any(abs(v) > 0 for v in trajectory.sample(
            trajectory.duration - 1e-5).accelerations))
        self.assertEqual(samples[-1].time, trajectory.duration)
        for previous, sample in zip(samples, samples[1:]):
            self.assertLess(sample.time - previous.time, 0.031 + 1e-12)
            self.assertGreater(sample.time, previous.time)
            self.assertLessEqual(max(map(abs, sample.velocities)), 1.0 + 1e-12)
            self.assertLessEqual(max(map(abs, sample.accelerations)), 1.0 + 1e-12)
            self.assertAlmostEqual(sample.positions[1] / GOAL[1], sample.positions[3] / GOAL[3])
            self.assertEqual(sample.positions[4], 0.0)
        stationary = plan(GOAL, GOAL).sample(0.05)
        self.assertEqual(stationary.positions, GOAL)
        self.assertEqual(stationary.accelerations, ZERO)

    def test_sequence_contains_exact_stop_waypoints(self):
        alternate = (0.1, 0.2, -0.3, 0.4, -0.5, 0.6)
        trajectory = plan_sequence(ZERO, (GOAL, alternate, ZERO))
        waypoint_samples = [
            sample for sample in trajectory.samples(0.19)
            if sample.time in trajectory.waypoint_times
        ]
        self.assertEqual([sample.positions for sample in waypoint_samples], [GOAL, alternate, ZERO])
        self.assertTrue(all(sample.velocities == ZERO for sample in waypoint_samples))
        self.assertEqual(waypoint_samples[-1].accelerations, ZERO)
        self.assertEqual(trajectory, plan_sequence(ZERO, (GOAL, alternate, ZERO)))

    def test_virtual_board_three_cycles(self):
        goals = ((.2, -.3, .7, .25, -.7, .3), (.35, -.4, .6, .2, -.6, .2),
                 (.18, -.25, .55, .12, -.42, .17), ZERO)
        trajectory = plan_sequence(ZERO, goals * 3)
        self.assertEqual(len(trajectory.waypoint_times), 12)
        one_cycle = sum(2.0 * sqrt(max(abs(a - b) for a, b in zip(previous, goal)))
                        for previous, goal in zip((ZERO, *goals[:-1]), goals, strict=True))
        self.assertAlmostEqual(trajectory.duration, 3.0 * one_cycle, places=10)
        previous_time = 0.0
        for index, (time, goal) in enumerate(zip(trajectory.waypoint_times, goals * 3)):
            self.assertGreater(time, previous_time)
            at = trajectory.sample(time)
            self.assertEqual(at.positions, goal)
            self.assertEqual(at.velocities, ZERO)
            for side in (-1, 1) if index < 11 else (-1,):
                near = trajectory.sample(time + side * 1e-5)
                self.assertLess(max(abs(a - b) for a, b in zip(near.positions, goal)), 1e-9)
                self.assertLessEqual(max(map(abs, near.velocities)), 1.1e-5)
                self.assertLessEqual(max(map(abs, near.accelerations)), 1.0 + 1e-12)
            previous_time = time
        for index in range(4):
            first, second, third = trajectory.waypoint_times[index::4]
            self.assertAlmostEqual(second - first, third - second)
        self.assertEqual(trajectory.sample(trajectory.duration).accelerations, ZERO)

    def test_invalid_input(self):
        with self.assertRaisesRegex(ValueError, "posición fuera"):
            plan(ZERO, ZERO[:2] + (JOINT_LIMITS[2][1] + 0.001,) + ZERO[3:])
        with self.assertRaises(ValueError):
            plan(ZERO, ZERO[:5] + (nan,))
        with self.assertRaises(ValueError):
            plan(ZERO, GOAL, duration=0.01)
        with self.assertRaises(ValueError):
            plan(ZERO, GOAL, max_acceleration=ZERO)
        with self.assertRaises(ValueError):
            plan_sequence(ZERO, ())
        trajectory = plan(ZERO, GOAL)
        for time in (-0.01, inf, trajectory.duration + 0.01):
            with self.assertRaises(ValueError):
                trajectory.sample(time)
        with self.assertRaises(ValueError):
            trajectory.samples(0)
        with self.assertRaises(ValueError):
            trajectory.samples(1e-20)


class CandidatePlanningTest(unittest.TestCase):
    class Model:
        @staticmethod
        def singular_values(q, characteristic_length_m):
            return (1.0, abs(q[0] - 0.2) / characteristic_length_m)

        @staticmethod
        def fk(q):
            return ((1, 0, 0, q[0]), (0, 1, 0, q[1]), (0, 0, 1, q[2]), (0, 0, 0, 1))

    @staticmethod
    def branch(name, j1):
        return SimpleNamespace(
            branch=name, source="fixture", q=(j1,) + ZERO[1:],
            position_error_m=0.0, orientation_error_rad=0.0,
        )

    def test_rank_preserves_rejections_and_sampling(self):
        far, near, invalid = self.branch("far", 0.4), self.branch("near", 0.1), self.branch("bad", 5)
        paths = plan_candidates(self.Model(), ZERO, (far, invalid, near))
        self.assertEqual([path.branch for path in paths], ["near", "far", "bad"])
        self.assertLess(paths[0].score, paths[1].score)
        self.assertTrue(all(path.collision == "NO_VERIFICADA" for path in paths))
        self.assertIn("fuera del límite", paths[2].reason)
        self.assertIsNone(paths[2].trajectory)
        samples = tcp_samples(self.Model(), paths[0].trajectory)
        self.assertEqual(samples[0][0].positions, ZERO)
        self.assertEqual(samples[-1][0].positions, near.q)
        self.assertEqual(samples[-1][1][0][3], 0.1)

    def test_singularity_and_joint_displacement_filters(self):
        far = self.branch("far", 0.4)
        singular, = plan_candidates(self.Model(), ZERO, (far,), min_sigma=0.01)
        self.assertIn("Sigma", singular.reason)
        self.assertAlmostEqual(singular.min_sigma, 0.0)
        discontinuous, = plan_candidates(self.Model(), ZERO, (far,), max_joint_delta_rad=0.3)
        self.assertIn("Desplazamiento", discontinuous.reason)
        self.assertEqual(plan_candidates(self.Model(), ZERO, ()), ())

    def test_global_and_axis_scale_multiply_nominal(self):
        goal = self.branch("far", 0.4)
        scales = (0.1,) + (1.0,) * 5
        fast, = plan_candidates(self.Model(), ZERO, (goal,))
        slow, = plan_candidates(self.Model(), ZERO, (goal,), global_scale=0.1, axis_scales=scales)
        self.assertGreater(slow.trajectory.duration, fast.trajectory.duration)
        peak = max(abs(sample.velocities[0]) for sample in slow.trajectory.samples())
        self.assertAlmostEqual(peak, 4.363323 * 0.1 * 0.1)
        with self.assertRaises(ValueError):
            plan_candidates(self.Model(), ZERO, (), global_scale=0)


if __name__ == "__main__":
    unittest.main()
