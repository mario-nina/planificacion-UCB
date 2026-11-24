from json import loads
from math import degrees, radians
import unittest

from imt342_irb120_control.rws_mock import GOAL, FakeHttp, RwsRapidMock
from imt342_irb120_control.state import Result


ZERO = (0.0,) * 6
GOAL_RAD = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class RwsTest(unittest.TestCase):
    def setUp(self):
        self.http = FakeHttp()
        self.backend = RwsRapidMock(self.http, Clock())

    def test_discrete_goal_acceptance_and_completion(self):
        self.assertEqual(self.backend.execute(GOAL_RAD), Result.ACCEPTED)
        method, route, body = self.http.requests[-1]
        self.assertEqual((method, route), ("POST", GOAL))
        self.assertEqual(loads(body)["position_deg"], list(map(degrees, GOAL_RAD)))
        self.assertEqual(self.backend.read_state().positions, ZERO)
        self.assertEqual(self.backend.result(), Result.ACCEPTED)
        with self.assertRaisesRegex(RuntimeError, "409"):
            self.backend.execute(GOAL_RAD)
        self.http.complete()
        self.assertEqual(self.backend.result(), Result.SIMULATED_COMPLETE)
        for actual, expected in zip(self.backend.read_state().positions, GOAL_RAD):
            self.assertAlmostEqual(actual, expected)

    def test_cancel_and_http_errors(self):
        self.backend.execute(GOAL_RAD)
        self.assertEqual(self.backend.cancel(), Result.CANCELLED)
        self.assertEqual(self.backend.result(), Result.CANCELLED)
        self.assertEqual(self.backend.read_state().positions, ZERO)
        with self.assertRaises(RuntimeError):
            self.http.complete()
        self.http.forced_status = 403
        with self.assertRaisesRegex(RuntimeError, "HTTP 403"):
            self.backend.read_state()

    def test_invalid_read_goal_and_transport(self):
        self.http.positions_deg = (float("nan"),) + ZERO[1:]
        with self.assertRaises(ValueError):
            self.backend.read_state()
        with self.assertRaises(ValueError):
            self.backend.execute((100.0,) + ZERO[1:])
        with self.assertRaises(TypeError):
            RwsRapidMock("http://localhost", Clock())


if __name__ == "__main__":
    unittest.main()
