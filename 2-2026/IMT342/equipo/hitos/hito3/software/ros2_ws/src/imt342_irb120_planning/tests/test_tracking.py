from math import isclose, radians
from unittest import TestCase
from imt342_irb120_planning import AUTO_REFERENCE_VELOCITIES, plan
from imt342_irb120_planning.smooth import plan_quintic
from imt342_irb120_planning.tracking import TrackingMetrics, reference_limits, local_derivatives, reference_at


class TrackingTests(TestCase):
    def test_automatic_limits_and_profile_shape(self):
        for displacement, shape in ((3, 'triangular'), (60, 'trapezoidal')):
            goal = (radians(displacement), radians(displacement)/2, 0, 0, 0, 0)
            s = plan((0,)*6, goal, max_velocity=AUTO_REFERENCE_VELOCITIES)
            limits = reference_limits(s)
            self.assertEqual(limits['shape'], shape)
            self.assertLessEqual(max(limits['peak_velocity_rad_s']), .6 + 1e-12)
            self.assertLessEqual(max(limits['peak_acceleration_rad_s2']), 1 + 1e-12)
            self.assertAlmostEqual(2*limits['ramp_s'] + limits['cruise_s'], s.duration)
            for sample in s.samples(.005):
                for i in range(6):
                    self.assertLessEqual(abs(sample.velocities[i]), limits['peak_velocity_rad_s'][i]+1e-12)
                    self.assertLessEqual(abs(sample.accelerations[i]), limits['peak_acceleration_rad_s2'][i]+1e-12)

    def test_quintic_extrema(self):
        s = plan_quintic((0,)*6, (.4,.2,0,0,0,0), max_velocity=AUTO_REFERENCE_VELOCITIES)
        data = reference_limits(s)
        self.assertEqual(data['shape'], 'quintic')
        self.assertIsNone(data['cruise_s'])
        self.assertAlmostEqual(data['peak_velocity_rad_s'][0], s.sample(s.duration/2).velocities[0])

    def test_unique_measurements_and_separate_legs(self):
        m = TrackingMetrics()
        self.assertEqual(m.report(0,1)['leg']['samples'], 0)
        self.assertIsNone(m.report(0,1)['leg']['rmse_rad'])
        self.assertTrue(m.add(0,1,10.,(1.,0.,0.,0.,0.,0.)))
        self.assertFalse(m.add(0,1,10.,(2.,)*6))
        self.assertFalse(m.add(0,1,9.,(2.,)*6))
        self.assertTrue(m.add(0,1,11.,(3.,0.,0.,0.,0.,0.)))
        self.assertTrue(m.add(1,2,12.,(2.,)*6))
        data = m.report(0,1)
        self.assertEqual(data['leg']['samples'], 2)
        self.assertAlmostEqual(data['leg']['rmse_by_joint_rad'][0], 5**.5)
        self.assertEqual(data['leg']['max_error_rad'],3.)
        self.assertEqual(data['run']['samples'], 3)
        self.assertEqual(m.report(1,2)['leg']['rmse_rad'],2.)
        self.assertFalse(m.add(1,None,13.,(0.,)*6))
        self.assertFalse(m.add(1,2,13.,(float('nan'),)*6))


    def test_derivative_fit_nonuniform_quadratic(self):
        times = (0., .09, .19, .31, .45)
        samples = [(round(t*1e9), tuple(0.5*j*t*t + .2*t for j in range(6))) for t in times]
        velocity, acceleration = local_derivatives(samples)
        for j in range(6):
            self.assertAlmostEqual(velocity[j], j*times[-1]+.2, places=10)
            self.assertAlmostEqual(acceleration[j], j, places=10)

    def test_derivative_rejects_missing_or_duplicate_samples(self):
        q = (0.,)*6
        self.assertIsNone(local_derivatives([(0,q),(100_000_000,q),(100_000_000,q)]))
        self.assertIsNone(local_derivatives([(0,q),(100_000_000,q),(700_000_000,q)]))
        self.assertIsNone(local_derivatives([(0,q),(100_000_000,q)]))

    def test_derivative_five_samples_reduces_alternating_noise(self):
        from math import sqrt
        samples = [(i*100_000_000, (.02*i + .0003*(-1)**i,)*6) for i in range(5)]
        v5,a5 = local_derivatives(samples)
        v3,a3 = local_derivatives(samples[-3:])
        self.assertLess(abs(v5[0]-.2), abs(v3[0]-.2))
        self.assertLess(abs(a5[0]), abs(a3[0]))

    def test_reference_is_unscaled_and_stays_at_rest_outside_interval(self):
        segment = plan((0.,)*6, (.4,.2,0,0,0,0), max_velocity=AUTO_REFERENCE_VELOCITIES)
        origin = 15.
        before = reference_at(segment, origin-.2, origin)
        after = reference_at(segment, origin+segment.duration*2, origin)
        self.assertEqual(before.positions, segment.start)
        self.assertEqual(before.accelerations, (0.,)*6)
        self.assertEqual(after.positions, segment.goal)
        self.assertEqual(after.velocities, (0.,)*6)
        self.assertEqual(after.accelerations, (0.,)*6)
        middle = reference_at(segment, origin+segment.duration/2, origin)
        self.assertAlmostEqual(middle.positions[0], .2)
        self.assertAlmostEqual(middle.velocities[0], segment.sample(segment.duration/2).velocities[0])
