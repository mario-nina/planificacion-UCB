from math import cbrt
import unittest

from imt342_irb120_planning.smooth import plan_quintic


class SmoothTest(unittest.TestCase):
    def test_analytic_jerk_boundary_and_invalid_durations(self):
        start = (0.,) * 6
        goal = (.2, *start[1:])
        minimum = cbrt(60 * goal[0] / 0.1)
        limits = dict(max_velocity=(20.,) * 6, max_acceleration=(20.,) * 6, max_jerk=(.1,) * 6)
        segment = plan_quintic(start, goal, duration=minimum, **limits)
        self.assertAlmostEqual(segment.jerk(0)[0], .1)
        self.assertAlmostEqual(segment.jerk(minimum)[0], .1)
        for duration in (True, float("inf"), "invalid", minimum * .999):
            with self.subTest(duration=duration), self.assertRaisesRegex(ValueError, "[Dd]uración"):
                plan_quintic(start, goal, duration=duration, **limits)

    def test_bounds_endpoints_and_signed_synchronization(self):
        start = (0.0,) * 6
        goal = (.7, -.4, 1e-9, 0, 0, 0)
        plan = plan_quintic(start, goal, (0.6,) * 6, (.8,) * 6, (2,) * 6)
        self.assertEqual(plan.sample(0).positions, start)
        self.assertEqual(plan.sample(plan.duration).positions, goal)
        for t in (0, plan.duration):
            self.assertEqual(plan.sample(t).velocities, start)
            self.assertEqual(plan.sample(t).accelerations, start)
        for i in range(1001):
            t = plan.duration * (i / 1000)
            state = plan.sample(t)
            self.assertLessEqual(max(map(abs, state.velocities)), .6 + 1e-12)
            self.assertLessEqual(max(map(abs, state.accelerations)), .8 + 1e-12)
            self.assertLessEqual(max(map(abs, plan.jerk(t))), 2 + 1e-12)
            self.assertAlmostEqual(state.positions[0] / .7, state.positions[1] / -.4)
            self.assertEqual(state.positions[3:], (0., 0., 0.))
        epsilon = plan.duration * 1e-8
        self.assertLess(max(map(abs, plan.sample(epsilon).accelerations)), 1e-6)
        self.assertLess(max(map(abs, plan.sample(plan.duration - epsilon).accelerations)), 1e-6)

    def test_zero_small_and_infeasible(self):
        zero = (0,) * 6
        self.assertEqual(plan_quintic(zero, zero).sample(.05).positions, zero)
        self.assertEqual(plan_quintic(zero, (1e-15, *zero[1:])).duration, .1)
        with self.assertRaisesRegex(ValueError, "Duración"):
            plan_quintic(zero, (1, *zero[1:]), duration=.1)
        with self.assertRaisesRegex(ValueError, "límites"):
            plan_quintic(zero, (9, *zero[1:]))
