"""Verify the task presets against the installed IRB 120 kinematic model."""

from pathlib import Path
import unittest

from imt342_irb120_kinematics import Model
from imt342_irb120_kinematics.analysis import PointModels
from imt342_irb120_kinematics.cell import Cell
from imt342_irb120_planning import AUTO_REFERENCE_VELOCITIES
from imt342_irb120_planning.cell_task import perimeter_available, plan_cell_task
from imt342_irb120_planning.task import TaskInputError


HOME = (0.,) * 6


class CartesianCellTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.description = Path(__file__).resolve().parents[2] / "imt342_irb120_description"
        cls.xml = (cls.description / "urdf" / "irb120.urdf").read_text(encoding="utf-8")
        cls.points = PointModels.from_model(Model.from_xml(cls.xml))
        config = dict(
            frame=dict(xyz_m=(.040, 0., -.200), rpy_rad=(0.,) * 3),
            walls=dict(outer_xy_m=(.800, .800), height_m=.200, thickness_m=.050),
            collision=dict(clearance_m=.002, joint_margin_rad=.00523598775598),
        )
        cls.cell = Cell.from_urdf(config, cls.xml, cls.description)

    def test_longitudinal_tracks_a_wall_and_returns_to_fixed_home(self):
        task = dict(kind="cell_wall", cycles=2, point="tcp")
        plan, evidence = plan_cell_task(
            self.points, HOME, task, AUTO_REFERENCE_VELOCITIES, self.cell, max_duration=30.)
        self.assertEqual(plan.kind, "cell_wall")
        self.assertEqual(plan.required_start, HOME)
        self.assertEqual(evidence["home_q_rad"], HOME)
        self.assertEqual(evidence["variant"], "over_wall")
        self.assertEqual(evidence["wall"], "x_max")
        self.assertEqual(plan.order[0:2], ("APROX", "INICIO"))
        self.assertEqual(plan.order[-2:], ("SALIDA", "HOME"))
        self.assertEqual(plan.trajectory.profiles, ("lspb",) * len(plan.trajectory.segments))
        count = evidence["command_segments"]
        self.assertLessEqual(count, 14)
        self.assertEqual(evidence["total_command_segments"], 2 * count)
        first = plan.trajectory.segments[:count]
        self.assertEqual(first, plan.trajectory.segments[count:])
        self.assertEqual(first[0].start, HOME)
        self.assertEqual(first[-1].goal, HOME)

        reference = evidence["reference_xyz_m"]
        self.assertEqual(reference[0], reference[-1])
        self.assertAlmostEqual(max(y for _, y, _ in reference) - min(y for _, y, _ in reference), .500)
        for point in reference:
            self.assertAlmostEqual(point[0], .415)
            self.assertAlmostEqual(point[2], .100)
        self.assertGreater(evidence["sampled_configurations"], count)
        self.assertGreater(evidence["collision_configurations"], count)
        self.assertLess(evidence["max_tcp_deviation_m"], .008)
        self.assertLess(evidence["max_tcp_orientation_error_rad"], .14)
        self.assertGreater(evidence["min_sigma_scaled"], .025)
        self.assertTrue(self.cell.check_trajectory(self.points, plan.trajectory).ok)

        wall_leg_count = 0
        for segment, record in zip(first, plan.waypoints, strict=True):
            if record["wall"] is None:
                continue
            wall_leg_count += 1
            self.assertIsNotNone(record["branch"])
            self.assertLess(max(abs(a - b) for a, b in zip(segment.start, segment.goal)), .6)
            for sample in segment.samples(.007):
                position = tuple(row[3] for row in self.points.tcp.fk(sample.positions)[:3])
                self.assertAlmostEqual(self.cell.wall_distance(record["wall"], position), .100, delta=.010)
        self.assertGreaterEqual(wall_leg_count, 8)

    def test_wall_preset_keeps_quintic_alternative(self):
        task = dict(kind="cell_wall", cycles=1, profile="quintic", wall="x_max", point="tcp")
        plan, evidence = plan_cell_task(self.points, HOME, task, AUTO_REFERENCE_VELOCITIES, self.cell)
        self.assertEqual(plan.trajectory.profiles, ("quintic",) * evidence["command_segments"])
        self.assertEqual(plan.trajectory.segments[-1].goal, HOME)

    def test_nominal_perimeters_require_more_than_j1_can_turn(self):
        self.assertFalse(perimeter_available(self.points, self.cell, "tcp"))
        task = dict(kind="cell_perimeter", cycles=1, point="tcp")
        with self.assertRaisesRegex(TaskInputError, r"over_perimeter:.*vuelta completa de J1.*inner_perimeter:"):
            plan_cell_task(self.points, HOME, task, AUTO_REFERENCE_VELOCITIES, self.cell)

    def test_perimeter_can_close_for_a_different_synthetic_cell(self):
        for center_x, size, variant in ((.36, (.32, .32), "over_perimeter"),
                                       (.43, (.40, .34), "inner_perimeter")):
            with self.subTest(variant=variant):
                config = dict(
                    frame=dict(xyz_m=(center_x, 0., -.20), rpy_rad=(0.,) * 3),
                    walls=dict(outer_xy_m=size, height_m=.20, thickness_m=.025),
                )
                cell = Cell.from_urdf(config, self.xml, self.description)
                self.assertTrue(perimeter_available(self.points, cell, "tcp"))
                plan, evidence = plan_cell_task(self.points, HOME, dict(kind="cell_perimeter", cycles=1, point="tcp"),
                                                AUTO_REFERENCE_VELOCITIES, cell)
                self.assertEqual(evidence["variant"], variant)
                self.assertEqual(evidence["reference_xyz_m"][0], evidence["reference_xyz_m"][-1])
                self.assertEqual(plan.trajectory.segments[1].goal, plan.trajectory.segments[-3].goal)
                self.assertEqual(plan.trajectory.segments[-1].goal, HOME)
                self.assertTrue(cell.check_trajectory(self.points, plan.trajectory).ok)

    def test_invalid_home_repetitions_and_wall_are_rejected(self):
        task = dict(kind="cell_wall", cycles=1)
        for invalid in (dict(task, cycles=True), dict(task, cycles=9), dict(task, wall="center")):
            with self.subTest(task=invalid), self.assertRaises(TaskInputError):
                plan_cell_task(self.points, HOME, invalid, AUTO_REFERENCE_VELOCITIES, self.cell)
        with self.assertRaisesRegex(TaskInputError, "HOME fuera"):
            plan_cell_task(self.points, (3.0, *HOME[1:]), task, AUTO_REFERENCE_VELOCITIES, self.cell)

    def test_corrected_installed_geometry_flange_default_and_repetition(self):
        # Read the production YAML so the regression detects a future wrong Z.
        import yaml
        config_path = self.description.parent / "imt342_irb120_bringup/config/cell.yaml"
        config = yaml.safe_load(config_path.read_text())
        cell = Cell.from_urdf(config, self.xml, self.description)
        self.assertEqual(cell.xyz_m, (.128, 0., 0.))
        self.assertAlmostEqual(cell.to_base((0., 0., cell.wall_top_z))[2], .2)
        self.assertTrue(cell.check_pose(HOME, self.points.tool_offset).ok)
        plan, evidence = plan_cell_task(self.points, HOME, dict(kind="cell_wall", cycles=2),
                                        AUTO_REFERENCE_VELOCITIES, cell, max_duration=30.)
        self.assertEqual(evidence["reference_point"], "flange")
        self.assertEqual(evidence["wall"], "x_max")
        self.assertTrue(all(wp["point"] == "flange" for wp in plan.waypoints))
        self.assertTrue(all(abs(p[2] - .30) < 1e-9 for p in evidence["reference_xyz_m"]))
        self.assertEqual(plan.trajectory.segments[-1].goal, HOME)
        count = evidence["command_segments"]
        self.assertEqual(plan.trajectory.segments[:count], plan.trajectory.segments[count:])
        self.assertLess(evidence["max_tcp_deviation_m"], .008)
        self.assertTrue(cell.check_trajectory(self.points, plan.trajectory).ok)
        self.assertTrue(all(.1 <= seg.duration <= 30. for seg in plan.trajectory.segments))
        self.assertFalse(perimeter_available(self.points, cell))
