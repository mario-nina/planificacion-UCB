"""Jazzy integration test with DDS localhost and no configured transport."""

import json
import math
import os
import signal
import subprocess
import tempfile
from pathlib import Path
from time import monotonic
import unittest

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory
from imt342_irb120_kinematics import JOINT_NAMES, Model


class SessionTest(unittest.TestCase):
    def setUp(self):
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 30)
        self.log = tempfile.TemporaryFile(mode="w+")
        self.addCleanup(self.log.close)
        self.launch = subprocess.Popen(
            ["ros2", "launch", "imt342_irb120_bringup", "session.launch.py",
             "gui:=false", "mode:=offline", "backend:=external"],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.addCleanup(self.stop_launch)
        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        self.node = rclpy.create_node("imt342_session_test")
        self.addCleanup(self.node.destroy_node)
        self.received = {}
        self.tf = set()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, msg_type, qos in (
            ("mode_state", String, latched), ("status", String, latched),
            ("actual", JointState, 10), ("preview", JointState, 10),
            ("trajectory", JointTrajectory, latched),
            ("pose_plan_result", String, latched), ("cell_markers", MarkerArray, latched),
        ):
            self.node.create_subscription(
                msg_type, "/imt342/" + topic,
                lambda msg, key=topic: self.received.update({key: msg}), qos,
            )
        # Match tf2's static listener history, with independent writers.
        static_tf = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, qos in (("/tf", 100), ("/tf_static", static_tf)):
            self.node.create_subscription(
                TFMessage, topic,
                lambda msg: self.tf.update((tf.header.frame_id, tf.child_frame_id)
                                           for tf in msg.transforms), qos,
            )

    def stop_launch(self):
        if self.launch.poll() is None:
            os.killpg(self.launch.pid, signal.SIGINT)
            try:
                self.launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.launch.pid, signal.SIGKILL)
                self.launch.wait()

    def wait_for(self, condition, timeout=10.0):
        deadline = monotonic() + timeout
        while monotonic() < deadline and self.launch.poll() is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return
        self.log.seek(0)
        self.fail(f"Transición ROS no observada. TF: {sorted(self.tf)}\n{self.log.read()[-4000:]}")

    def send(self, topic, msg):
        pub = self.node.create_publisher(type(msg), "/imt342/" + topic, 10)
        self.wait_for(lambda: pub.get_subscription_count() > 0)
        pub.publish(msg)

    def state(self, key, expected):
        msg = self.received.get(key)
        return msg is not None and len(msg.position) == len(expected) and all(
            math.isclose(x, y, abs_tol=1e-6) for x, y in zip(msg.position, expected)
        )

    def status(self, fragment):
        return fragment in self.received.get("status", String()).data

    @staticmethod
    def joints(q):
        return JointState(name=list(JOINT_NAMES), position=list(q))

    def test_analytical_route_preview_keeps_virtual_actual_fixed(self):
        share = Path(get_package_share_directory("imt342_irb120_description"))
        model = Model.from_file(share / "urdf/irb120.urdf")
        matrix = model.fk((0.2, -0.3, 0.7, 0.25, -0.7, 0.3))
        zero = [0.0] * 6
        self.wait_for(lambda: self.state("actual", zero))
        self.send("pose_plan_request", String(data=json.dumps(
            dict(matrix=matrix, request_id="offline-test", global_scale=0.5),
        )))
        self.wait_for(lambda: "pose_plan_result" in self.received)
        result = json.loads(self.received["pose_plan_result"].data)
        self.assertEqual(result["request_id"], "offline-test")
        self.assertFalse(result["error"], result["error"])
        self.assertGreaterEqual(len(result["routes"]), 2)
        route = result["routes"][0]
        self.assertEqual(route["collision"], "NO_VERIFICADA")
        self.assertEqual(route["cell_collision"], "clear")
        self.send("pose_plan_preview", String(data=json.dumps(
            dict(branch=route["branch"], request_id="offline-test"),
        )))
        self.wait_for(lambda: self.status("Previsualizando") and "trajectory" in self.received)
        self.assertEqual(list(self.received["trajectory"].points[-1].positions), route["q"])
        self.assertTrue(self.state("actual", zero))
        self.send("pose_plan_preview", String(data="cancel"))
        self.wait_for(lambda: self.status("Vista de ruta cancelada") and self.state("preview", zero))
        self.assertTrue(self.state("actual", zero))

    def test_modes_preview_cancel_and_tf(self):
        zero = [0.0] * 6
        measured = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
        first = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        last = [0.4, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.wait_for(lambda: self.state("actual", zero) and self.state("preview", zero))
        self.wait_for(lambda: ("base_link", "preview_base_link") in self.tf
                      and ("base_link", "cell_frame") in self.tf
                      and any(child == "link_1" for _, child in self.tf)
                      and any(child == "preview_link_1" for _, child in self.tf))
        self.wait_for(lambda: "cell_markers" in self.received)
        mesh, = self.received["cell_markers"].markers
        self.assertEqual(mesh.header.frame_id, "cell_frame")
        self.assertEqual(mesh.type, Marker.MESH_RESOURCE)
        self.assertAlmostEqual(mesh.scale.x, .001)
        self.send("execute_target", self.joints(last))
        self.wait_for(lambda: self.status("Ejecución bloqueada"))
        self.assertTrue(self.state("actual", zero))
        self.assertNotIn("trajectory", self.received)
        self.send("mode_command", String(data="observe"))
        self.wait_for(lambda: self.status("sin telemetría"))
        self.send("measured", self.joints(measured))
        self.wait_for(lambda: self.state("actual", measured))
        self.send("execute_target", self.joints(last))
        self.wait_for(lambda: self.status("Ejecución bloqueada"))
        self.assertNotIn("trajectory", self.received)
        self.send("preview_waypoints", JointTrajectory(
            joint_names=list(JOINT_NAMES), points=[
                JointTrajectoryPoint(positions=first), JointTrajectoryPoint(positions=last),
            ],
        ))
        self.wait_for(lambda: "trajectory" in self.received)
        points = [list(point.positions) for point in self.received["trajectory"].points]
        self.assertEqual(points[0], measured)
        self.assertIn(first, points)
        self.assertEqual(points[-1], last)
        self.send("preview_command", String(data="cancel"))
        self.wait_for(lambda: self.status("cancelada") and self.state("preview", zero))
        self.assertTrue(self.state("actual", measured))
        self.send("mode_command", String(data="control"))
        self.wait_for(lambda: self.received.get("mode_state", String()).data == "control")
        self.send("physical_command", String(data=f"arm:{self.node.get_clock().now().nanoseconds}:fk"))
        self.wait_for(lambda: self.status("Habilitación rechazada"))
        self.send("execute_target", self.joints(last))
        self.wait_for(lambda: self.status("Ejecución bloqueada"))
        self.send("apply_target", self.joints(last))  # Virtual command cannot execute online.
        self.send("mode_command", String(data="offline"))
        self.wait_for(lambda: self.status("virtual") and self.state("actual", zero))


if __name__ == "__main__":
    unittest.main()
