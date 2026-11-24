from copy import deepcopy
from math import pi, radians
from pathlib import Path
from types import SimpleNamespace
from unittest import TestCase

from imt342_irb120_kinematics.analysis import PointModels
from imt342_irb120_kinematics.cell import Cell
from imt342_irb120_kinematics.model import Model
from imt342_irb120_kinematics.pose import pose_from_rpy


DESCRIPTION = Path(__file__).resolve().parents[2] / "imt342_irb120_description"
ROBOT_XML = (DESCRIPTION / "urdf/irb120.urdf").read_text(encoding="utf-8")
HOME = (0.0,) * 6
CONFIG = {
    "geometry_verified": False,
    "frame": {"xyz_m": [0.04, 0., -0.2], "rpy_rad": [0., 0., 0.]},
    "walls": {"outer_xy_m": [0.8, 0.8], "height_m": 0.2, "thickness_m": 0.05},
    "collision": {"clearance_m": 0.002, "joint_margin_rad": radians(0.3)},
}


def cell_from(config=CONFIG):
    return Cell.from_urdf(config, ROBOT_XML, DESCRIPTION)


def segment(start, goal, profile="lspb"):
    return SimpleNamespace(start=start, goal=goal, profile=profile)


class CellTests(TestCase):
    def setUp(self):
        self.cell = cell_from()
        self.points = PointModels.from_model(Model.from_xml(ROBOT_XML))

    def test_nominal_dimensions_offset_and_cell_transform(self):
        self.assertFalse(self.cell.geometry_verified)
        self.assertEqual(self.cell.xyz_m, (0.04, 0.0, -0.2))
        self.assertEqual(self.cell.wall_top_z, 0.2)
        self.assertEqual(self.cell.wall_thickness, 0.05)
        axes = self.cell.inner_bounds
        for lo, hi in axes:
            self.assertAlmostEqual(hi - lo, 0.7)
        self.assertAlmostEqual(self.cell.to_base((axes[0][0], 0., 0.2))[0], -0.310)
        self.assertAlmostEqual(self.cell.to_base((axes[0][1], 0., 0.2))[0], 0.390)
        self.assertAlmostEqual(self.cell.to_base((0., 0., 0.2))[2], 0.0)
        self.assertEqual([wall.name for wall in self.cell.walls], ["x_min", "x_max", "y_min", "y_max"])
        self.assertAlmostEqual(min(wall.lower[0] for wall in self.cell.walls), -0.4)
        self.assertAlmostEqual(max(wall.upper[1] for wall in self.cell.walls), 0.4)
        self.assertAlmostEqual(self.cell.wall_distance("x_max", (0.415, 0., 0.1)), 0.1)
        self.assertAlmostEqual(self.cell.wrist_tcp_bound(self.points.tool_offset), 0.192)
        self.assertEqual(self.cell.first_joint_axis(), ((0., 0., 1.), (0., 0., 0.)))

        rotated = deepcopy(CONFIG)
        rotated["frame"]["rpy_rad"][2] = pi / 2
        rotated_cell = cell_from(rotated)
        point = (0.2, -0.1, 0.05)
        base = rotated_cell.to_base(point)
        for measured, expected in zip(base, (0.14, 0.2, -0.15)):
            self.assertAlmostEqual(measured, expected)
        for measured, expected in zip(rotated_cell.to_cell(base), point):
            self.assertAlmostEqual(measured, expected)

    def test_clear_pose_and_segment_keep_original_limits(self):
        self.assertTrue(self.cell.check_pose(HOME).ok)
        target = (0.06, 0.08, 0.005, 0., 0., 0.)
        report = self.cell.check_segment(HOME, target)
        self.assertTrue(report.ok, report.reason)
        self.assertGreaterEqual(report.configurations_checked, 3)
        outside = self.cell.check_segment(HOME, (pi, 0., 0., 0., 0., 0.))
        self.assertFalse(outside.ok)
        self.assertIn("límites articulares", outside.reason)

    def test_detects_base_arm_and_gripper_from_collision_geometry(self):
        narrowed = deepcopy(CONFIG)
        narrowed["frame"]["xyz_m"] = [0.35, 0., -0.1]
        blocked_base = cell_from(narrowed).check_pose(HOME)
        self.assertEqual((blocked_base.ok, blocked_base.link, blocked_base.wall), (False, "base_link", "x_min"))
        for angles, link, wall in (
            ((-2.78, 1.42, -0.08, 1.98, 0.34, -3.58), "link_4", "x_min"),
            ((2.75, 1.43, 0.44, 2.72, 1.69, -1.26), "link_5", "x_min"),
            ((-0.21, 1.58, -0.72, -0.12, 1.31, 6.08), "gripper_base", "x_max"),
        ):
            with self.subTest(link=link):
                report = self.cell.check_pose(angles)
                self.assertEqual((report.ok, report.link, report.wall), (False, link, wall))

    def test_continuous_check_catches_collision_with_endpoints_and_midpoint_clear(self):
        start = (0.82, 1.54, 0.52, -1.73, 0.37, -4.65)
        end = (2.71, -0.98, -0.08, -0.88, -1.79, -4.94)
        midpoint = tuple((a + b) / 2 for a, b in zip(start, end))
        for q in (start, midpoint, end):
            self.assertTrue(self.cell.check_pose(q).ok)
        for profile in ("lspb", "quintic"):
            with self.subTest(profile=profile):
                travel = SimpleNamespace(segments=(segment(start, start, profile),
                                                   segment(start, end, profile)))
                report = self.cell.check_trajectory(self.points, travel)
                self.assertEqual((report.ok, report.wall, report.segment_index), (False, "y_max", 1))
                self.assertEqual(report.fraction, 0.25)

    def test_custom_tcp_is_checked_and_calibration_margin_is_applied(self):
        flange = self.points.flange.fk(HOME)
        near_wall = (0.415, 0., 0.001)
        # Translate the desired base-link point into flange coordinates.
        world_delta = tuple(p - row[3] for p, row in zip(near_wall, flange[:3]))
        tool_xyz = tuple(sum(flange[i][j] * world_delta[i] for i in range(3)) for j in range(3))
        tool = pose_from_rpy(tool_xyz, (0., 0., 0.))
        with_margin = self.cell.check_pose(HOME, tool)
        self.assertEqual((with_margin.ok, with_margin.link, with_margin.wall), (False, "tcp_active", "x_max"))

        nominal = deepcopy(CONFIG)
        nominal["collision"] = {}
        self.assertTrue(cell_from(nominal).check_pose(HOME, tool).ok)

    def test_discontinuous_or_invalid_geometry_is_rejected(self):
        goal = (0.08, 0., 0., 0., 0., 0.)
        travel = SimpleNamespace(segments=(segment(HOME, goal), segment(HOME, HOME)))
        report = self.cell.check_trajectory(self.points, travel)
        self.assertEqual((report.ok, report.segment_index), (False, 1))
        self.assertIn("discontinuidad", report.reason)
        invalid = deepcopy(CONFIG)
        invalid["walls"]["thickness_m"] = 0.45
        with self.assertRaisesRegex(ValueError, "Dimensiones"):
            cell_from(invalid)
