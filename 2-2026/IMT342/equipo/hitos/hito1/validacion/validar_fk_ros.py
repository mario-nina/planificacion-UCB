import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


FK_DIR = (
    Path(__file__).resolve().parent.parent
    / "modelo_cinematico"
    / "cinematica_directa"
)
sys.path.insert(0, str(FK_DIR))

from irb120_fk import forward_kinematics, forward_kinematics_tcp


JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

CONFIGURATIONS_DEG = {
    "A": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "B": [20.0, -30.0, 25.0, 15.0, -20.0, 35.0],
}

POSITION_TOLERANCE = 1e-6


def quaternion_to_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)

    if norm == 0.0:
        raise ValueError("El cuaternion recibido tiene norma cero.")

    x, y, z, w = q / norm

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def transform_to_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation

    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]

    return T


class FKROSValidator(Node):
    def __init__(self):
        super().__init__("validar_fk_ros")

        self.joint_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

    def wait_for_robot_state_publisher(self, timeout=5.0):
        deadline = time.monotonic() + timeout

        while (
            self.joint_publisher.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.joint_publisher.get_subscription_count() == 0:
            raise RuntimeError(
                "No se encontro un suscriptor para /joint_states."
            )

    def publish_configuration(self, q):
        stamp = self.get_clock().now()

        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = JOINT_NAMES
        msg.position = q.tolist()

        for _ in range(3):
            self.joint_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        return stamp

    def lookup_matrix(self, target_frame, stamp, timeout=5.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    target_frame,
                    stamp,
                )
                return transform_to_matrix(transform)

            except TransformException:
                pass

        raise RuntimeError(
            f"No se pudo obtener TF base_link -> {target_frame}."
        )


def compare_transforms(label, T_fk, T_ros):
    p_fk = T_fk[:3, 3]
    p_ros = T_ros[:3, 3]

    delta_e = np.linalg.norm(p_fk - p_ros)
    rotation_max_error = np.max(
        np.abs(T_fk[:3, :3] - T_ros[:3, :3])
    )

    passed = delta_e < POSITION_TOLERANCE

    print()
    print(f"--- {label} ---")
    print("T_FK:")
    print(T_fk)
    print()
    print("T_ROS:")
    print(T_ros)
    print()
    print(f"p_FK  = {p_fk}")
    print(f"p_ROS = {p_ros}")
    print(f"Delta E = {delta_e:.15e} m")
    print(
        "Error maximo de orientacion "
        f"|R_FK - R_ROS| = {rotation_max_error:.15e}"
    )
    print(f"Resultado: {'PASS' if passed else 'FAIL'}")

    return passed


def validate_configuration(node, name, q_deg):
    q = np.deg2rad(np.asarray(q_deg, dtype=float))

    stamp = node.publish_configuration(q)

    T_6_fk = forward_kinematics(q)
    T_6_ros = node.lookup_matrix("dh_frame_6", stamp)

    T_tcp_fk = forward_kinematics_tcp(q)
    T_tcp_ros = node.lookup_matrix("tcp", stamp)

    print()
    print(f"========== CONFIGURACION {name} ==========")
    print(f"q [deg] = {np.asarray(q_deg, dtype=float)}")

    flange_passed = compare_transforms(
        "BRIDA {6} - VALIDACION OFICIAL",
        T_6_fk,
        T_6_ros,
    )

    tcp_passed = compare_transforms(
        "TCP - VALIDACION COMPLEMENTARIA",
        T_tcp_fk,
        T_tcp_ros,
    )

    return flange_passed and tcp_passed


def main():
    np.set_printoptions(
        precision=15,
        suppress=True,
    )

    rclpy.init()
    node = FKROSValidator()

    try:
        node.wait_for_robot_state_publisher()

        results = []

        for name, q_deg in CONFIGURATIONS_DEG.items():
            results.append(
                validate_configuration(
                    node,
                    name,
                    q_deg,
                )
            )

        print()
        print("========== CRITERIO ==========")
        print(f"Delta E < {POSITION_TOLERANCE:.0e} m")

        print()
        print("========== RESULTADO GLOBAL ==========")
        print("PASS" if all(results) else "FAIL")

        return 0 if all(results) else 1

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
