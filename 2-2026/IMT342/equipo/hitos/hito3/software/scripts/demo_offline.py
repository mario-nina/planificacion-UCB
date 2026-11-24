#!/usr/bin/env python3
"""Emit reproducible evidence from the model and a synthetic joint sequence."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws/src"
for package in ("imt342_irb120_kinematics", "imt342_irb120_planning"):
    sys.path.insert(0, str(SRC / package))

from imt342_irb120_kinematics import JOINT_NAMES, Model
from imt342_irb120_planning import plan_sequence


def main():
    urdf = SRC / "imt342_irb120_description/urdf/irb120.urdf"
    model = Model.from_file(urdf)
    start = (0.1, -0.2, 0.6, 0.0, -0.5, 0.1)
    target = (0.2, -0.3, 0.7, 0.25, -0.7, 0.3)
    pose = model.fk(target)
    ik = model.ik(pose, start)
    if not ik.converged:
        raise SystemExit("La IK de referencia no convergió")
    trajectory = plan_sequence(start, [ik.q, start])
    output = {
        "scope": "synthetic_offline",
        "joint_names": JOINT_NAMES,
        "angle_unit": "rad",
        "frames": {"base": "base_link", "tcp": "tcp"},
        "model_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "gate_2": {
            "fk_target": pose,
            "analytic_jacobian": model.jacobian(target),
            "scaled_singular_values": model.singular_values(target),
            "ik_seed": start,
            "ik_solution": ik.q,
            "ik_iterations": ik.iterations,
            "ik_position_error_m": ik.position_error_m,
            "ik_orientation_error_rad": ik.orientation_error_rad,
        },
        "gate_4_synthetic_sequence": [
            {"time_s": sample.time, "q_rad": sample.positions}
            for sample in trajectory.samples(period=0.1)
        ],
    }
    print(json.dumps(output, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
