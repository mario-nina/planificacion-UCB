"""Validate telemetry provenance, freshness, and terminal output redaction."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import runpy
import unittest


status = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/diagnostico.py"))
START = 2_000_000_000
SENT = START + 100_000_000


class StatusTest(unittest.TestCase):
    def setUp(self):
        self.packet = dict(
            mode="observe", backend="rws", source="rws", actual=[0.0] * 6,
            timestamp_ns=SENT, actual_stamp_ns=SENT - 10_000_000,
            feedback_age_s=0.01, stale=False, backend_connected=True, physical_armed=False,
        )

    def test_ignores_old_reports(self):
        self.assertEqual(status["fresh_packet"](self.packet, START, SENT), self.packet)
        for started, now in ((SENT + 1, SENT + 1), (START, SENT - 1),
                             (START, SENT + 1_000_000_001)):
            with self.subTest(started=started, now=now):
                self.assertIsNone(status["fresh_packet"](self.packet, started, now))

    def test_checks_steady_age_and_packet_delay(self):
        age, stamp, fresh = status["telemetry"](self.packet, SENT + 100_000_000)
        self.assertAlmostEqual(age, 0.11)
        self.assertEqual(stamp, self.packet["actual_stamp_ns"])
        self.assertTrue(fresh)
        self.assertFalse(status["telemetry"](self.packet, SENT + 999_999_999)[2])

    def test_declines_virtual_mock_stale_missing_and_invalid(self):
        for replacement in (dict(mode="offline"), dict(backend="external"),
                            dict(source="MEDIDO (SIMULADO/MOCK)"), dict(stale=True),
                            dict(actual=None), dict(actual=[False] * 6), dict(actual=[10**400] * 6),
                            dict(actual=[float("nan")] * 6), dict(actual_stamp_ns=0),
                            dict(backend_connected=False), dict(feedback_age_s=None)):
            with self.subTest(replacement=replacement):
                self.assertFalse(status["telemetry"]({**self.packet, **replacement}, SENT)[2])

    def test_observe_signal_requires_mode_and_blocked_execution(self):
        self.assertTrue(status["observe_ready"](self.packet, SENT))
        self.assertFalse(status["observe_ready"]({**self.packet, "physical_armed": True}, SENT))
        self.assertFalse(status["observe_ready"]({**self.packet, "mode": "control"}, SENT))

    def test_only_whitelisted_diagnostics_are_printed(self):
        self.packet.update(actual=None, status="secret", transport={
            "http_status": 401, "mode": "secret", "motors": "motoroff", "error": "secret",
        })
        out = StringIO()
        with redirect_stdout(out):
            status["describe"](self.packet, SENT)
        self.assertIn("HTTP 401", out.getvalue())
        self.assertIn("motoroff", out.getvalue())
        self.assertNotIn("secret", out.getvalue())
        self.assertNotIn("telemetría física reciente", out.getvalue())


if __name__ == "__main__":
    unittest.main()
