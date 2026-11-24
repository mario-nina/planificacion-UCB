import unittest
from time import monotonic, sleep

from imt342_irb120_control.state import Feedback, Result
from imt342_irb120_control.supervisor import Supervisor


class Transport:
    def __init__(self, name):
        self.name = name
        self.calls = []
        self.result = Result.IDLE
        self.available = True
        self.safe = True
        self.ready = False
        self.age = 0

    def execute(self, goal, trajectory, authorized):
        if not authorized():
            self.result = Result.FAILED
            return
        self.calls.append(("execute", goal, trajectory))
        self.result = Result.EXECUTING

    def cancel(self):
        self.calls.append(("cancel",))
        self.result = Result.UNKNOWN

    def poll(self, *, check_ready=False):
        self.ready = check_ready and self.available and self.safe
        feedback = Feedback((0.0,) * 6, monotonic() - self.age, self.name)
        return feedback if self.available else None, self.result, self.result.value

    def close(self):
        pass


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.rws = Transport("rws")
        self.trajectory = Transport("rws_trajectory")
        self.manager = Supervisor({"rws": lambda: self.rws, "rws_trajectory": lambda: self.trajectory},
                                  "rws_trajectory")
        self.addCleanup(self.manager.close)

    def drive(self, predicate, online=True):
        deadline = monotonic() + 2.0
        while not predicate() and monotonic() < deadline:
            self.manager.tick(online)
            sleep(0.001)
        self.assertTrue(predicate())

    def authorize(self):
        self.assertTrue(self.manager.set_mode("control"))
        self.drive(lambda: self.manager.can_arm)
        self.assertTrue(self.manager.arm())

    def test_observe_does_not_send_and_cancellation_holds_ownership(self):
        self.manager.tick(False)
        self.assertEqual(self.manager.clients, {})
        self.assertFalse(self.manager.execute("offline", "trajectory"))
        self.assertTrue(self.manager.set_mode("observe"))
        self.drive(lambda: len(self.manager.clients) == 1 and self.manager.feedback is not None)
        self.assertFalse(self.manager.arm())
        self.assertFalse(self.manager.execute("observe", "trajectory"))
        self.assertEqual(self.trajectory.calls, [])
        self.assertEqual(self.rws.calls, [])
        self.authorize()
        self.assertTrue(self.manager.execute("goal", "trajectory"))
        self.drive(lambda: self.trajectory.result is Result.EXECUTING)
        self.assertFalse(self.manager.select("rws"))
        self.assertFalse(self.manager.set_mode("observe"))
        self.assertEqual(self.manager.mode, "control")
        self.assertFalse(self.manager.execute("second", "trajectory"))
        self.manager.cancel()
        self.drive(lambda: self.trajectory.result is Result.UNKNOWN)
        self.assertFalse(self.manager.select("rws"))
        self.assertEqual(self.rws.calls, [])
        self.trajectory.result = Result.CANCELLED
        self.drive(lambda: self.manager.owner is None)
        self.assertTrue(self.manager.select("rws"))

    def test_prior_terminal_poll_cannot_clear_new_pending_request(self):
        self.authorize()
        self.trajectory.result = Result.VERIFIED
        self.manager.next_poll["rws_trajectory"] = 0
        self.manager.tick(True)
        self.assertIn("rws_trajectory", self.manager.jobs)
        self.assertTrue(self.manager.execute("goal", "trajectory"))
        self.drive(lambda: self.trajectory.result is Result.EXECUTING)
        self.assertEqual(self.manager.owner, "rws_trajectory")
        self.assertEqual(self.trajectory.calls, [("execute", "goal", "trajectory")])

    def test_pending_request_can_be_cancelled_without_transport(self):
        self.authorize()
        self.assertTrue(self.manager.execute("goal", "trajectory"))
        self.manager.cancel()
        self.assertIsNone(self.manager.owner)
        self.manager.tick(False)
        self.assertEqual(self.trajectory.calls, [])

    def test_mode_transition_discards_pending_request_and_permission(self):
        self.authorize()
        self.assertTrue(self.manager.execute("goal", "trajectory"))
        self.assertTrue(self.manager.set_mode("offline"))
        self.assertIsNone(self.manager.owner)
        self.authorize()
        self.assertEqual(self.trajectory.calls, [])

    def test_control_without_authorization_or_fresh_feedback(self):
        self.manager.set_mode("control")
        self.assertFalse(self.manager.arm())
        self.drive(lambda: self.manager.can_arm)
        self.assertFalse(self.manager.execute("goal", "trajectory"))
        self.assertTrue(self.manager.arm())
        self.trajectory.age = 2
        self.drive(lambda: not self.manager.armed)
        self.assertFalse(self.manager.execute("goal", "trajectory"))
        self.assertEqual(self.trajectory.calls, [])

    def test_outage_drops_pending_and_reconnect_does_not_rearm(self):
        self.authorize()
        # Arrange for a read failure to finish before the accepted request is dispatched.
        self.trajectory.available = False
        self.manager.next_poll["rws_trajectory"] = 0
        self.manager.tick(True)
        self.assertIn("rws_trajectory", self.manager.jobs)
        self.assertTrue(self.manager.execute("goal", "trajectory"))
        self.drive(lambda: self.manager.owner is None)
        self.trajectory.available = True
        self.drive(lambda: self.manager.can_arm)
        self.assertFalse(self.manager.armed)
        self.assertFalse(self.manager.execute("goal", "trajectory"))
        self.assertEqual(self.trajectory.calls, [])

    def test_controller_interlock_revokes_without_outage(self):
        self.authorize()
        self.trajectory.safe = False
        self.drive(lambda: not self.manager.armed)
        self.assertFalse(self.manager.arm())
        self.assertEqual(self.trajectory.calls, [])


class AdapterTest(unittest.TestCase):
    def test_rws_poll_outage_keeps_unknown_and_surfaces_error(self):
        from unittest.mock import Mock
        from imt342_irb120_control.adapters import RwsAdapter
        from imt342_irb120_control.rws_backend import RwsError

        backend = Mock(last_error="", final_error_rad=None, motion_evidence={}, final_feedback=None)
        backend.result.return_value = Result.UNKNOWN
        backend.read_state.side_effect = RwsError("HTTP 503")
        feedback, result, detail = RwsAdapter(backend).poll()
        self.assertIsNone(feedback)
        self.assertEqual(result, Result.UNKNOWN)
        self.assertIn("HTTP 503", detail)
