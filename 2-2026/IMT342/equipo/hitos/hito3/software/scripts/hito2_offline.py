#!/usr/bin/env python3
"""Run the synthetic Hito 2 experiment without ROS or controller access."""

import argparse
import csv
from dataclasses import asdict
from hashlib import sha256
import json
from math import hypot
from pathlib import Path
from xml.sax.saxutils import escape

# Reuse workspace discovery from the original standalone demo.
from demo_offline import SRC

from imt342_irb120_kinematics import JOINT_NAMES, Model, SingleLinkDynamics
from imt342_irb120_planning import BLEND, plan_candidates, tcp_samples

LENGTH_M = 0.5
RANK_TOL = 1e-8
START = (0.1, -0.2, 0.6, 0.0, -0.5, 0.1)
TARGETS = ((-0.5, 0.4, -0.7, 0.3, 0.8, -0.4), (0.2, -0.3, 0.7, 0.25, -0.7, 0.3))
PARAMETERS_1R = dict(inertia_kg_m2=0.08, mass_kg=1.2, com_radius_m=0.16, gravity_m_s2=9.81)


def experiment():
    urdf = SRC / "imt342_irb120_description/urdf/irb120.urdf"
    model = Model.from_file(urdf)
    link = SingleLinkDynamics(**PARAMETERS_1R)
    cases = []
    for index, q_ref in enumerate(TARGETS, start=1):
        matrix = model.fk(q_ref)
        candidates = model.ik_pieper(matrix, seed=START)
        paths = plan_candidates(model, START, candidates, global_scale=0.5,
                                axis_scales=(1, 1, 1, 1, 1, 0.65))
        selected = next(path for path in paths if path.trajectory is not None)
        profiles = []
        for sample, pose in tcp_samples(model, selected.trajectory, period=0.05):
            xyz = tuple(row[3] for row in pose[:3])
            inertia, gravity = link.components(sample.positions[1], sample.accelerations[1])
            profiles.append(dict(
                t_s=sample.time, q=sample.positions, qdot=sample.velocities,
                qddot=sample.accelerations, tcp_xyz_m=xyz,
                torque_inertia_nm=inertia, torque_gravity_nm=gravity, torque_nm=inertia + gravity,
            ))
        jacobian = model.jacobian(q_ref)
        fd = []
        for h in (1e-2, 1e-3, 1e-4, 1e-5, 1e-6):
            finite_difference = model.jacobian_fd(q_ref, h)
            errors = [[a - b for a, b in zip(row, diff)]
                      for row, diff in zip(jacobian, finite_difference)]
            fd.append(dict(h_rad=h, linear_error=hypot(*(x for row in errors[:3] for x in row)),
                           angular_error=hypot(*(x for row in errors[3:] for x in row))))
        cases.append(dict(
            id=f"synthetic-{index}", q_reference=q_ref, T_base_tcp=matrix,
            candidates=[asdict(candidate) for candidate in candidates],
            routes=[asdict(path) for path in paths], selected=selected.branch,
            blend_s=BLEND * selected.trajectory.duration, jacobian=jacobian, fd=fd,
            profiles=profiles,
        ))

    sweep = []
    base_q = list(TARGETS[1])
    for q5 in (0.3, 0.15, 0.05, 0.01, 0.003, 0.001, 0):
        base_q[4] = q5
        sigma = model.singular_values(base_q, LENGTH_M)
        rank = sum(value > RANK_TOL for value in sigma)
        sweep.append(dict(q=base_q.copy(), sigma=sigma, rank=rank,
                          condition=sigma[0] / sigma[-1] if rank == 6 else None,
                          inverse_gain=1 / sigma[-1] if rank == 6 else None))
    return dict(
        source="synthetic_offline", joint_names=JOINT_NAMES, q_start=START,
        angular_unit="rad", linear_unit="m", frames=["base_link", "tcp"],
        urdf_sha256=sha256(urdf.read_bytes()).hexdigest(),
        approved_tolerances="PENDIENTE", reference_independence="INCONCLUSO",
        characteristic_length_m=LENGTH_M, rank_threshold=RANK_TOL,
        parameters_1r_synthetic=PARAMETERS_1R, cases=cases, wrist_sweep=sweep,
    )


def chart(title, x_label, xs, series):
    """Render a small SVG chart using only the standard library."""
    palette = ("#1462a3", "#bd460f", "#14805c", "#6a43a6", "#0f788c", "#a03867")
    width, height = 720, 236
    xmin, xmax = min(xs), max(xs)
    ys = [y for _, values in series for y in values]
    ymin, ymax = min(ys), max(ys)
    dx, dy = xmax - xmin or 1, ymax - ymin or 1
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title)}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<g fill="#152b3c" font-family="sans-serif" font-size="13">',
        f'<text x="62" y="22" font-weight="bold">{escape(title)}</text>',
    ]
    for step in range(5):
        fraction = step / 4
        x, y = 62 + 620 * fraction, 170 - 130 * fraction
        parts += [
            f'<path d="M62 {y}H682 M{x} 40V170" stroke="#e1e8ef" fill="none"/>',
            f'<text x="55" y="{y+4}" text-anchor="end">{ymin+fraction*(ymax-ymin):.3g}</text>',
            f'<text x="{x}" y="188" text-anchor="middle">{xmin+fraction*dx:.3g}</text>',
        ]
    for i, (label, values) in enumerate(series):
        color = palette[i]
        points = " ".join(f"{62 + 620*(x-xmin)/dx:.2f},{170 - 130*(y-ymin)/dy:.2f}"
                          for x, y in zip(xs, values))
        parts.extend([
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>',
            f'<text x="{62 + i*105}" y="225" fill="{color}">{escape(label)}</text>',
        ])
    parts.append(f'<text x="372" y="207" text-anchor="middle">{escape(x_label)}</text></g></svg>')
    return "".join(parts)


def figures(data):
    profiles = data["cases"][0]["profiles"]
    t = [row["t_s"] for row in profiles]
    for field, label in (("q", "Posición [rad]"), ("qdot", "Velocidad [rad/s]"),
                         ("qddot", "Aceleración [rad/s²]"), ("tcp_xyz_m", "TCP [m]")):
        axes = "XYZ" if field == "tcp_xyz_m" else [f"J{i}" for i in range(1, 7)]
        series = [(name, [row[field][i] for row in profiles]) for i, name in enumerate(axes)]
        yield field, chart(label, "t [s]", t, series)
    yield "torque_1r", chart("Torque 1R sintético [N m]", "t [s]", t, [
        (label, [row[field] for row in profiles])
        for field, label in (("torque_nm", "Total"), ("torque_inertia_nm", "Inercia"),
                             ("torque_gravity_nm", "Gravedad"))
    ])
    sweep = data["wrist_sweep"]
    yield "wrist_sigma", chart("Aproximación a singularidad de muñeca", "J5 [rad]",
                              [row["q"][4] for row in sweep],
                              [("σ mínima", [row["sigma"][-1] for row in sweep])])


def export(data, output):
    output.mkdir(parents=True, exist_ok=True)
    (output / "hito2.json").write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    for case in data["cases"]:
        with (output / f"{case['id']}.csv").open("w", encoding="utf-8", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["t_s", *[f"{key}_{joint}_{unit}" for key, unit in
                            (("q", "rad"), ("qdot", "rad_s"), ("qddot", "rad_s2"))
                            for joint in JOINT_NAMES],
                            "tcp_x_m", "tcp_y_m", "tcp_z_m", "torque_1r_nm"])
            for row in case["profiles"]:
                writer.writerow([row["t_s"], *row["q"], *row["qdot"], *row["qddot"],
                                 *row["tcp_xyz_m"], row["torque_nm"]])
    for name, figure in figures(data):
        (output / f"{name}.svg").write_text(figure, encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Directorio de exportación JSON, CSV y SVG")
    args = parser.parse_args()
    export(experiment(), args.output)
