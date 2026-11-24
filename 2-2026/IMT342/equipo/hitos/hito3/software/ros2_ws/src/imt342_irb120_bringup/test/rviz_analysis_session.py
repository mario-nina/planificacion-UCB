"""Observe the production analysis ROS contract with no physical transport."""
import json
import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from imt342_irb120_kinematics import Model
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import session_offline as fixture


class AnalysisSessionTest(fixture.SessionTest):
    def setUp(self):
        super().setUp()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.samples = []
        self.node.create_subscription(String, "/imt342/analysis", self.on_sample, latched)
        self.node.create_subscription(TFMessage, "/tf", self.on_transforms, 50)
        share = get_package_share_directory("imt342_irb120_description")
        self.model = Model.from_file(Path(share) / "urdf/irb120.urdf")

    def on_sample(self, msg):
        self.samples.append(json.loads(msg.data))

    def on_transforms(self, msg):
        self.tf.update((transform.header.frame_id, transform.child_frame_id)
                       for transform in msg.transforms)

    def latest(self, condition):
        self.wait_for(lambda: bool(self.samples) and condition(self.samples[-1]))
        return self.samples[-1]

    def plan(self, request_id, matrix, scale=1):
        self.send("pose_plan_request", String(data=json.dumps({
            "request_id": request_id, "matrix": matrix, "global_scale": scale, "point": "tcp",
        })))
        record = self.latest(lambda data: data.get("pose_plan_id") == request_id)["route_result"]
        return record

    def test_rws_backends_share_offline_profiles_and_task_analysis(self):
        zero = [0.] * 6
        self.latest(lambda data: data["mode"] == "offline" and data["actual"] == zero)
        self.send("analysis_command", String(data='{"trajectory_duration_s":0.8}'))
        self.latest(lambda data: data["trajectory_duration_s"] == 0.8)
        for backend in ("rws", "rws_trajectory"):
            self.send("backend_command", String(data=backend))
            selected = self.latest(lambda data: data["backend"] == backend)
            self.assertFalse(selected["backend_connected"])
            self.assertFalse(selected["physical_can_arm"])
            for profile in ("lspb", "quintic"):
                self.send("analysis_command", String(data=json.dumps({"trajectory_profile": profile})))
                self.latest(lambda data: data["trajectory_profile"] == profile)
                self.send("preview_target", self.joints([.03, 0., 0., 0., 0., 0.]))
                sample = self.latest(lambda data: (data["plan"] or {}).get("profile_id") == profile)
                self.assertEqual(sample["actual"], zero)
                self.assertEqual(sample["source"], "SIMULACIÓN OFFLINE")
                self.assertTrue(all(math.isfinite(a) for a in sample["plan"]["profile_qddot"]))
                self.assertIsNone(sample["measured_qddot"])
                if profile == "quintic":
                    self.assertAlmostEqual(sample["plan"]["duration_s"], .8)
                self.send("preview_command", String(data="cancel"))
                self.latest(lambda data: data["plan"] is None)

        goal = [.025, 0., 0., 0., 0., 0.]
        task = dict(kind="custom", cycles=2, order=["Ida", "Retorno"], waypoints={
            "Ida": dict(q_rad=goal, duration_s=.8, profile="quintic"),
            "Retorno": dict(q_rad=zero, duration_s=.8, profile="lspb"),
        })
        self.send("task_command", String(data=json.dumps({
            "action": "plan", "request_id": "rws-cycle", "task": task,
        })))
        planned = self.latest(lambda data: data["task"].get("request_id") == "rws-cycle")["task"]
        self.assertEqual(planned["state"], "planned", planned.get("error"))
        self.assertEqual(planned["reference_profile"], "mixed")
        self.assertAlmostEqual(planned["planned_duration_s"], 3.2)
        for action in ("preview", "simulate"):
            self.send("task_command", String(data=json.dumps({"action": action, "plan_id": planned["plan_id"]})))
            if action == "preview":
                self.latest(lambda data: data["task"].get("previewed") is True)
        finished = self.latest(lambda data: data["task"].get("state") == "simulated")
        self.assertEqual(finished["task"]["verified_waypoints"], 4)
        self.assertEqual(finished["actual"], zero)
        self.assertFalse(finished["physical_armed"])
        self.assertFalse(finished["backend_connected"])

    def test_points_singularity_and_tf(self):
        base = self.latest(lambda data: bool(data.get("jacobian")))
        self.assertEqual(base["source"], "SIMULACIÓN OFFLINE")
        self.assertEqual(base["joint_order"], list(fixture.JOINT_NAMES))
        self.assertEqual(len(base["model_sha256"]), 64)
        self.wait_for(lambda: {"dh_frame_6", "preview_dh_frame_6"} <= {
            parent for parent, child in self.tf if child in {"tcp_active", "preview_tcp_active"}
        })
        self.assertIn(("dh_frame_6", "tcp_active"), self.tf)
        self.assertIn(("preview_dh_frame_6", "preview_tcp_active"), self.tf)
        self.send("analysis_command", String(data=json.dumps({
            "point": "both", "offset_xyz": [.10, .03, .08], "offset_rpy": [.30, 0, 0],
        })))
        changed = self.latest(lambda data: data.get("tool_revision", 0) > base["tool_revision"])
        self.assertEqual(changed["actual_fk"]["flange"], base["actual_fk"]["flange"])
        self.assertNotEqual(changed["actual_fk"]["tcp"]["xyz"], changed["actual_fk"]["flange"]["xyz"])
        self.assertNotEqual(changed["jacobian"]["tcp"]["jv"], changed["jacobian"]["flange"]["jv"])
        self.assertEqual(changed["jacobian"]["tcp"]["jw"], changed["jacobian"]["flange"]["jw"])
        self.assertLess(changed["jacobian"]["flange"]["rank"], 6)
        self.assertGreaterEqual(len(changed["jacobian"]["flange"]["fd"]), 3)

    def test_profile_progress_pause_scrub_and_error(self):
        matrix = self.model.fk((.2, -.3, .7, .25, -.7, .3))
        self.latest(lambda data: data.get("actual") == [0] * 6)
        normal = self.plan("normal", matrix)
        slow = self.plan("slow", matrix, .05)
        self.assertFalse(normal["error"])
        self.assertFalse(slow["error"])
        self.assertGreaterEqual(len(slow["routes"]), 2)
        best = slow["routes"][0]
        equivalent = next(route for route in normal["routes"] if route["branch"] == best["branch"])
        self.assertGreater(best["duration"], equivalent["duration"])
        request = {"request_id": "slow", "branch": best["branch"]}
        self.send("pose_plan_preview", String(data=json.dumps(request)))
        first = self.latest(lambda data: bool(data.get("plan")) and data["plan"]["time_s"] > .1)
        moving = self.latest(lambda data: bool(data.get("plan")) and
                             data["plan"]["time_s"] > first["plan"]["time_s"] + .1)
        self.assertEqual(moving["actual"], first["actual"])
        self.assertNotEqual(moving["preview"], first["preview"])
        self.assertNotEqual(moving["preview_fk"]["tcp"]["xyz"], first["preview_fk"]["tcp"]["xyz"])
        self.assertTrue(any(abs(value) > 0 for value in moving["plan"]["qdot"]))
        # At 5% of the reference limit, the narrow ramp can precede the first
        # published cursor; this sample is in cruise, with zero acceleration.
        self.assertGreater(moving["plan"]["time_s"], moving["plan"]["blend_s"])
        self.assertEqual(moving["plan"]["qddot"], [0.0] * 6)
        self.assertTrue(math.isfinite(moving["plan"]["torque_1r_nm"]))
        self.send("pose_plan_preview", String(data="pause"))
        frozen = self.latest(lambda data: bool(data.get("plan")) and data["plan"]["paused"])
        self.assertEqual(frozen["plan"]["qdot"], [0] * 6)
        self.assertEqual(frozen["plan"]["qddot"], [0] * 6)
        self.assertEqual(frozen["plan"]["phase"], "paused")
        self.assertIn("profile_qddot", frozen["plan"])
        after = self.latest(lambda data: data["elapsed_s"] > frozen["elapsed_s"] + .18)
        self.assertEqual(after["plan"]["time_s"], frozen["plan"]["time_s"])
        self.assertEqual(after["preview"], frozen["preview"])
        at = best["duration"] * .6
        self.send("pose_plan_preview", String(data=json.dumps({**request, "time_s": at})))
        self.latest(lambda data: bool(data.get("plan")) and data["plan"]["paused"] and
                    abs(data["plan"]["time_s"] - at) < 1e-8)
        self.send("pose_plan_preview", String(data="resume"))
        self.latest(lambda data: bool(data.get("plan")) and data["plan"]["time_s"] > at + .06)
        self.send("pose_plan_preview", String(data="cancel"))
        self.latest(lambda data: data.get("plan") is None)
        self.send("analysis_command", String(data=json.dumps({"invalidate_route": "slow"})))
        invalidated = self.latest(lambda data: not data.get("pose_plan_id"))
        self.assertEqual(invalidated["incident"]["code"], "RUTA_OBSOLETA")
        self.assertIsNone(invalidated["route_result"])
        invalid = self.plan("invalid", [[0] * 4 for _ in range(4)])
        self.assertTrue(invalid["error"])
        self.assertEqual(self.samples[-1]["incident"]["code"], "PLAN_RECHAZADO")
        self.send("preview_waypoints", JointTrajectory(
            joint_names=list(fixture.JOINT_NAMES),
            points=[JointTrajectoryPoint(positions=[.2, -.3, .7, .25, .5, .3]),
                    JointTrajectoryPoint(positions=[.2, -.3, .7, .25, 0.0, .3])],
        ))
        self.latest(lambda data: data.get("plan") and data["plan"]["time_s"] > .12)
        self.send("pose_plan_preview", String(data="pause"))
        sequence = self.latest(lambda data: data.get("plan") and data["plan"]["paused"])
        self.assertIsNone(sequence["plan"]["branch"])
        self.assertEqual(sequence["actual"], [0] * 6)
        self.send("pose_plan_preview", String(data="resume"))
        self.latest(lambda data: data.get("plan") and
                    data["plan"]["time_s"] > sequence["plan"]["time_s"] + .1)
        self.send("pose_plan_preview", String(data="cancel"))
        self.latest(lambda data: data.get("plan") is None)

    def test_preview_run_boundaries_terminal_acceleration_and_cycles(self):
        self.latest(lambda data: data.get("actual") == [0] * 6)
        goals = ([.2, -.3, .7, .25, -.7, .3], [.35, -.4, .6, .2, -.6, .2],
                 [.18, -.25, .55, .12, -.42, .17], [0.0] * 6)
        self.send("target", self.joints([0, .34906585, 0, 0, 0, 0]))
        self.latest(lambda data: data["target"][1] == .34906585)
        repeated = JointTrajectory(
            joint_names=list(fixture.JOINT_NAMES),
            points=[JointTrajectoryPoint(positions=q) for q in goals * 3],
        )
        repeated.header.frame_id = "imt342/offline_cycles/3/4"
        invalid = JointTrajectory(joint_names=repeated.joint_names, header=repeated.header,
                                  points=repeated.points[:-1])
        self.send("preview_waypoints", invalid)
        failure = self.latest(lambda data: (data["incident"] or {}).get("code") == "CICLO_INVALIDO")
        self.send("preview_waypoints", repeated)
        first = self.latest(lambda data: bool(data.get("plan")) and
                            data["plan"].get("running") and
                            data["plan"].get("time_s", 0) > .08)
        run = first["preview_run_id"]
        self.assertTrue(run)
        self.assertEqual(first["plan"]["run_id"], run)
        self.assertEqual(first["plan"]["cycles_total"], 3)
        self.assertEqual(first["plan"]["waypoints_per_cycle"], 4)
        self.assertEqual(len(first["plan"]["waypoint_times_s"]), 12)
        self.assertEqual(first["plan"]["cycle_index"], 1)
        self.assertIsNone(first["incident"])
        self.assertEqual(first["incident_resolution"]["timestamp_ns"], failure["incident"]["timestamp_ns"])
        self.assertEqual(first["incident_history"][-1]["state"], "resolved")
        self.wait_for(lambda: any(sample.get("plan") and
                       sample["plan"].get("cycle_index") == 2 and
                       sample["plan"].get("run_id") == run for sample in self.samples), timeout=25)
        self.wait_for(lambda: any(sample.get("plan") and
                       sample["plan"].get("phase") == "completed" and
                       sample["plan"].get("run_id") == run for sample in self.samples), timeout=25)
        ended = next(sample for sample in reversed(self.samples)
                     if sample.get("plan") and sample["plan"].get("phase") == "completed")
        self.assertEqual(ended["preview_end"]["phase"], "completed")
        self.assertEqual(ended["preview_end"]["run_id"], run)
        self.assertEqual(ended["plan"]["cycle_index"], 3)
        self.assertEqual(ended["plan"]["final_q"], [0] * 6)
        self.assertEqual(ended["actual"], ended["preview"])
        self.assertEqual(ended["target"][1], .34906585)
        self.assertEqual(ended["error"]["comparison"], "comparable_model")
        self.assertEqual(ended["plan"]["qdot"], [0] * 6)
        self.assertEqual(ended["plan"]["qddot"], [0] * 6)
        self.send("preview_waypoints", JointTrajectory(
            joint_names=list(fixture.JOINT_NAMES),
            points=[JointTrajectoryPoint(positions=q) for q in goals],
        ))
        second = self.latest(lambda data: data.get("plan") and
                             data["plan"].get("running") and
                             data["preview_run_id"] != run)
        self.assertGreater(second["preview_epoch"], first["preview_epoch"])
        self.send("preview_command", String(data="cancel"))
        cancelled = self.latest(lambda data: data.get("plan") is None)
        self.assertEqual(cancelled["preview_end"]["phase"], "cancelled")
        self.assertEqual(cancelled["preview_end"]["run_id"], second["preview_run_id"])

    def test_observation_mock_staleness(self):
        self.send("mode_command", String(data="observe"))
        self.latest(lambda data: data.get("mode") == "observe" and data.get("actual") is None)
        for i in range(5):
            mock = self.joints([.05 + .02 * i, 0, .3, 0, -.2, 0])
            mock.header.frame_id = "imt342_mock"
            mock.header.stamp = self.node.get_clock().now().to_msg()
            self.send("measured", mock)
            self.latest(lambda data: data.get("actual") == list(mock.position))
            time.sleep(.05)
        fresh = self.latest(lambda data: data.get("source") == "MEDIDO (SIMULADO/MOCK)")
        self.assertFalse(fresh["stale"])
        expired = self.latest(lambda data: data.get("stale"))
        self.assertIsNone(expired["actual"])
        self.assertIsNone(expired["actual_fk"])
        self.assertEqual(fresh["actual_stamp_ns"], expired["actual_stamp_ns"])
        self.assertEqual(expired["error"]["comparison"], "INCOMPARABLE")


if __name__ == "__main__":
    suite = unittest.TestSuite(AnalysisSessionTest(name) for name in (
        "test_rws_backends_share_offline_profiles_and_task_analysis",
        "test_points_singularity_and_tf",
        "test_profile_progress_pause_scrub_and_error",
        "test_preview_run_boundaries_terminal_acceleration_and_cycles",
        "test_observation_mock_staleness",
    ))
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
