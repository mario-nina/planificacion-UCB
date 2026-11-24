"""Point conventions, differential checks and analytic IK of an editable tool."""

from math import hypot
from pathlib import Path
import unittest

from imt342_irb120_kinematics import Model, pose_from_rpy
from imt342_irb120_kinematics.analysis import PointModels, jacobian_record, pose_records, pose_residual
from imt342_irb120_kinematics.model import _cross, _multiply


URDF = Path(__file__).resolve().parents[2] / "imt342_irb120_description/urdf/irb120.urdf"
REGULAR = (0.3, -0.2, 0.7, 0.4, -0.5, 1.0)


class PointAnalysisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = Model.from_file(URDF)
        cls.points = PointModels.from_model(cls.nominal)
        cls.manual = cls.points.with_tool(pose_from_rpy((0.04, -0.02, 0.18), (0.1, -0.18, 0.2)))

    def test_flange_and_nominal_tcp_reconstruct_the_urdf(self):
        flange, tcp = (self.points.at(point).fk(REGULAR) for point in ("flange", "tcp"))
        self.assertEqual(self.points.model_tool_offset[2][3], 0.120)
        self.assertEqual(tcp, self.nominal.fk(REGULAR))
        composed = _multiply(flange, self.points.model_tool_offset)
        self.assertLess(pose_residual(composed, tcp)["position_error_m"], 1e-14)

    def test_manual_tcp_changes_position_and_orientation_but_not_flange(self):
        baseline = pose_records(self.points, REGULAR)
        modified = pose_records(self.manual, REGULAR)
        self.assertEqual(modified["flange"], baseline["flange"])
        change = pose_residual(modified["tcp"]["matrix"], baseline["tcp"]["matrix"])
        self.assertGreater(change["position_error_m"], 0.05)
        self.assertGreater(change["orientation_error_rad"], 0.1)
        self.assertEqual(self.manual.tool_offset[0][3], 0.04)

    def test_jacobian_column_shift_and_fd_at_both_points(self):
        transforms = pose_records(self.manual, REGULAR)
        displacement = tuple(t - f for t, f in zip(transforms["tcp"]["xyz"],
                                                     transforms["flange"]["xyz"]))
        flange, tcp = (jacobian_record(self.manual.at(point), REGULAR)
                       for point in ("flange", "tcp"))
        self.assertEqual(flange["jw"], tcp["jw"])
        delta_norm = []
        for base_jv, active_jv, omega in zip(zip(*flange["jv"]), zip(*tcp["jv"]), zip(*tcp["jw"])):
            shift = _cross(omega, displacement)
            residual = tuple(end - start - change
                             for start, end, change in zip(base_jv, active_jv, shift))
            self.assertLess(hypot(*residual), 1e-14)
            delta_norm.append(hypot(*shift))
        self.assertGreater(max(delta_norm), 0.05)
        for point in (flange, tcp):
            self.assertEqual(point["rank"], 6)
            self.assertGreater(len(point["fd"]), 3)
            self.assertLess(min(item["error_jv"] for item in point["fd"]), 1e-8)
            self.assertLess(min(item["error_jw"] for item in point["fd"]), 1e-8)

    def test_singular_direction_is_orthogonal_to_all_scaled_columns(self):
        for point in ("flange", "tcp"):
            result = jacobian_record(self.manual.at(point), (0.0,) * 6)
            self.assertEqual(result["rank"], 5)
            self.assertIsNone(result["kappa"])
            self.assertIsNone(result["inverse_gain"])
            direction = result["weakest_direction_scaled"]
            self.assertAlmostEqual(hypot(*direction), 1.0)
            rows = [tuple(value / result["L"] for value in row) for row in result["jv"]]
            for col in zip(*rows, *result["jw"]):
                self.assertLess(abs(sum(a * b for a, b in zip(col, direction))), 1e-12)

    def test_pieper_round_trip_for_custom_tcp_and_flange(self):
        for point in ("flange", "tcp"):
            model = self.manual.at(point)
            goal = model.fk(REGULAR)
            candidates = model.ik_pieper(goal, seed=REGULAR)
            self.assertGreater(len(candidates), 1)
            self.assertLess(hypot(*(a - b for a, b in zip(candidates[0].q, REGULAR))), 1e-9)
            for candidate in candidates:
                residual = pose_residual(goal, model.fk(candidate.q))
                self.assertLess(residual["position_error_m"], 1e-8)
                self.assertLess(residual["orientation_error_rad"], 1e-8)

    def test_invalid_tool_transform_reports_the_model_constraint(self):
        bad = [list(row) for row in self.manual.tool_offset]
        bad[3][2] = 0.1
        with self.assertRaisesRegex(ValueError, "homogénea"):
            self.points.with_tool(bad)
        with self.assertRaisesRegex(ValueError, "Brida o TCP"):
            self.points.at("both")


if __name__ == "__main__":
    unittest.main()
