"""Task planning uses the real model without a controller."""
from math import radians
from pathlib import Path
import unittest

from imt342_irb120_kinematics import Model
from imt342_irb120_kinematics.analysis import PointModels
from imt342_irb120_planning import AUTO_REFERENCE_VELOCITIES, MODEL_VELOCITIES, planner_for_profile
from imt342_irb120_planning.smooth import QuinticSegment, plan_quintic
from imt342_irb120_planning.task import TaskInputError, WAYPOINTS, plan_task, preprogrammed_demo


class TaskPlanningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        urdf = Path(__file__).resolve().parents[2] / "imt342_irb120_description/urdf/irb120.urdf"
        cls.points = PointModels.from_model(Model.from_file(urdf))

    def setUp(self):
        self.start = (0.3, -0.2, 0.7, 0.4, -0.5, 1.0)
        self.task = dict(cycles=2, waypoints={
            name: dict(q_rad=[0.5 + i / 10, *self.start[1:]]) for i, name in enumerate(WAYPOINTS)
        })

    def plan(self, velocity=MODEL_VELOCITIES):
        return plan_task(self.points, self.start, self.task, velocity)

    def test_preprogrammed_demo_resolves_four_destinations_for_both_profiles(self):
        d = radians(3)
        expected = ((self.start[0] + d, *self.start[1:]),
                    (self.start[0] + d, self.start[1] + d, *self.start[2:]),
                    (self.start[0], self.start[1] + d, *self.start[2:]), self.start)
        for profile in ("lspb", "quintic"):
            with self.subTest(profile=profile):
                config = preprogrammed_demo(self.start, dict(kind="demo_cycle_live", cycles=1, profile=profile))
                planned = plan_task(self.points, self.start, config, AUTO_REFERENCE_VELOCITIES)
                self.assertEqual(planned.order, WAYPOINTS)
                self.assertEqual(planned.required_start, self.start)
                self.assertEqual(tuple(s.goal for s in planned.trajectory.segments), expected)
                self.assertEqual(planned.trajectory.profiles, (profile,) * 4)
                expected_duration = sum(planner_for_profile(profile)(
                    seg.start, seg.goal, max_velocity=AUTO_REFERENCE_VELOCITIES).duration
                    for seg in planned.trajectory.segments)
                self.assertAlmostEqual(planned.trajectory.duration, expected_duration)
                self.assertTrue(all("duration_s" not in item for item in config["waypoints"].values()))
                self.assertTrue(all(max(map(abs, sample.velocities)) <= .6 + 1e-9
                                    for seg in planned.trajectory.segments for sample in seg.samples(.01)))
                self.assertTrue(all(s.start != s.goal for s in planned.trajectory.segments))
                self.assertNotIn("physical_reviewed", config)

    def test_preprogrammed_demo_rejects_missing_origin_limits_and_unreviewed_repeats(self):
        task = dict(kind="demo_cycle_live", cycles=1)
        for invalid in (dict(task, cycles=2), dict(task, waypoints={}), dict(task, offline_example=True)):
            with self.assertRaises(TaskInputError):
                preprogrammed_demo(self.start, invalid)
        with self.assertRaises(TaskInputError):
            preprogrammed_demo((0,) * 5, task)
        with self.assertRaises(TaskInputError):
            preprogrammed_demo(None, task)
        near_limit = (self.points.flange.limits[0][1] - radians(1), *self.start[1:])
        with self.assertRaises(TaskInputError):
            plan_task(self.points, near_limit, preprogrammed_demo(near_limit, task), MODEL_VELOCITIES)

    def test_saved_relative_route_repeats_with_one_fixed_home(self):
        route = preprogrammed_demo(self.start, dict(kind="demo_cycle_live", cycles=1))
        measured = (self.start[0] + radians(.06), *self.start[1:])
        repeat = plan_task(self.points, measured, dict(route, kind="repeat", cycles=3),
                           AUTO_REFERENCE_VELOCITIES)
        self.assertEqual(repeat.required_start, self.start)
        self.assertEqual(repeat.trajectory.segments[0].start, measured)
        goals = tuple(tuple(route["waypoints"][name]["q_rad"]) for name in WAYPOINTS)
        self.assertEqual(tuple(segment.goal for segment in repeat.trajectory.segments), goals * 3)
        for segment in repeat.trajectory.segments[4::4]:
            self.assertEqual(segment.start, self.start)

    def test_cycles_close_to_the_explicit_home_and_scale_time(self):
        fast = self.plan()
        slow = self.plan((0.1,) * 6)
        self.assertEqual((fast.kind, fast.order), ("legacy", WAYPOINTS))
        self.assertEqual(len(fast.trajectory.segments), 8)
        self.assertEqual(fast.trajectory.segments[4].start, fast.trajectory.segments[3].goal)
        self.assertEqual(fast.trajectory.segments[-1].goal, tuple(self.task["waypoints"]["HOME"]["q_rad"]))
        self.assertGreater(slow.trajectory.duration, fast.trajectory.duration)
        self.assertTrue(all(rec["min_sigma_scaled"] >= 0 for rec in fast.leg_checks))

    def test_missing_home_and_joint_limit_are_rejected(self):
        self.task["waypoints"].pop("HOME")
        with self.assertRaisesRegex(TaskInputError, "HOME"):
            self.plan()
        self.task["waypoints"]["HOME"] = dict(q_rad=[10] * 6)
        with self.assertRaisesRegex(TaskInputError, "fuera del límite"):
            self.plan()

    def test_pose_requires_explicit_branch_and_records_fk_residual(self):
        matrix = self.points.tcp.fk(self.start)
        self.task["waypoints"]["A"] = dict(matrix=matrix, point="tcp")
        with self.assertRaises(TaskInputError) as rejected:
            self.plan()
        candidates = rejected.exception.candidates
        self.assertTrue(candidates)
        self.assertEqual(rejected.exception.waypoint, "A")
        self.task["waypoints"]["A"]["branch"] = "missing"
        with self.assertRaises(TaskInputError):
            self.plan()
        self.task["waypoints"]["A"]["branch"] = candidates[0]["branch"]
        record = self.plan().waypoints[0]
        self.assertEqual(record["requested_matrix"], matrix)
        self.assertLess(record["position_error_m"], 1e-6)
        self.assertLess(record["orientation_error_rad"], 1e-6)

    def test_joint_test_preserves_untested_live_axes_and_returns_to_measured_origin(self):
        initial = (0.301, -0.202, *self.start[2:])
        nominal_target = (*initial[:5], 1.1)
        self.task = dict(kind="joint_test", cycles=1, waypoints={
            "INICIO": dict(q_rad=list(initial)),
            "OBJETIVO": dict(q_rad=list(nominal_target)),
        })
        planned = self.plan()
        target = (*self.start[:5], nominal_target[-1])
        self.assertEqual(planned.kind, "joint_test")
        self.assertEqual(planned.order, ("OBJETIVO", "INICIO"))
        self.assertEqual(planned.required_start, initial)
        self.assertEqual(planned.trajectory.segments[0].start, self.start)
        self.assertEqual(tuple(segment.goal for segment in planned.trajectory.segments), (target, self.start))
        for record, segment, configured in zip(planned.waypoints, planned.trajectory.segments,
                                               (nominal_target, initial)):
            self.assertEqual(record["q_rad"], segment.goal)
            self.assertEqual(record["configured_q_rad"], configured)
            self.assertEqual(record["calculated_matrix"], self.points.flange.fk(segment.goal))
            self.assertEqual(record["first_lspb_s"], segment.duration)
            self.assertEqual(record["limit_margin_rad"], tuple(
                min(q - low, high - q) for q, (low, high) in zip(segment.goal, self.points.flange.limits)
            ))
        self.assertEqual(tuple(check["waypoint"] for check in planned.leg_checks), planned.order)
        self.start = target
        with self.assertRaisesRegex(TaskInputError, "ya está en OBJETIVO"):
            self.plan()

    def test_joint_test_rejects_unspecified_multiaxis_and_out_of_bounds_goals(self):
        self.task = dict(kind="joint_test", cycles=1, waypoints={
            "INICIO": dict(q_rad=list(self.start)),
            "OBJETIVO": {},
        })
        with self.assertRaises(TaskInputError) as blank:
            self.plan()
        self.assertEqual(blank.exception.waypoint, "OBJETIVO")
        for goal, message in (
            (self.start, "exactamente una articulación"),
            ((0.4, -0.3, *self.start[2:]), "exactamente una articulación"),
            ((*self.start[:5], 8), "fuera del límite"),
        ):
            with self.subTest(goal=goal):
                self.task["waypoints"]["OBJETIVO"] = dict(q_rad=list(goal))
                with self.assertRaisesRegex(TaskInputError, message):
                    self.plan()
        self.task["waypoints"]["OBJETIVO"] = dict(matrix=self.points.flange.fk(self.start))
        with self.assertRaisesRegex(TaskInputError, "requiere seis valores q_rad"):
            self.plan()

    def test_single_cycle_and_repeat_use_the_same_explicit_points(self):
        self.task.update(kind="cycle", cycles=1)
        single = self.plan()
        self.task.update(kind="repeat", cycles=3)
        repeat = self.plan()
        self.assertEqual(single.order, repeat.order)
        self.assertEqual(single.required_start, tuple(self.task["waypoints"]["HOME"]["q_rad"]))
        self.assertEqual(repeat.required_start, single.required_start)
        self.assertEqual((single.kind, repeat.kind), ("cycle", "repeat"))
        self.assertEqual(tuple(segment.goal for segment in repeat.trajectory.segments),
                         tuple(segment.goal for segment in single.trajectory.segments) * 3)
        self.assertEqual(repeat.leg_checks[-1]["cycle"], 2)
        for kind, cycles in (("cycle", 2), ("repeat", 1), ("repeat", 9), ("joint_test", True)):
            with self.subTest(kind=kind, cycles=cycles):
                self.task.update(kind=kind, cycles=cycles)
                with self.assertRaisesRegex(TaskInputError, "ciclo"):
                    self.plan()

    def test_custom_order_and_quintic_duration_are_preserved(self):
        self.task = dict(kind="custom", cycles=1, order=["P2", "P1"], waypoints={
            "P1": dict(q_rad=list(self.start), duration_s=2),
            "P2": dict(q_rad=[0.4, *self.start[1:]], duration_s=1.5),
        })
        planned = plan_task(self.points, self.start, self.task, MODEL_VELOCITIES,
                            segment_planner=plan_quintic)
        self.assertIsNone(planned.required_start)
        self.assertEqual(planned.order, ("P2", "P1"))
        self.assertEqual(planned.trajectory.waypoint_times, (1.5, 3.5))
        self.assertTrue(all(isinstance(leg, QuinticSegment) for leg in planned.trajectory.segments))
        self.assertNotIn("lspb_s", planned.leg_checks[0])
        self.task["waypoints"]["P2"]["duration_s"] = 0.1
        with self.assertRaisesRegex(TaskInputError, "P2: .*Mínimo"):
            plan_task(self.points, self.start, self.task, MODEL_VELOCITIES,
                      segment_planner=plan_quintic)

    def test_custom_one_leg_and_duplicate_order(self):
        self.task = dict(kind="custom", cycles=1, order=["P1"], waypoints={
            "P1": dict(q_rad=list(self.start)),
        })
        self.assertEqual(len(self.plan().trajectory.segments), 1)
        self.task["order"] = ["P1", "P1"]
        with self.assertRaisesRegex(TaskInputError, "únicos"):
            self.plan()

    def test_custom_repetition_applies_default_and_local_profiles(self):
        self.task = dict(kind="custom", cycles=3, order=["P2", "P1"], profile="quintic", waypoints={
            "P1": dict(q_rad=list(self.start), duration_s=2),
            "P2": dict(q_rad=[0.4, *self.start[1:]], duration_s=1.5, profile="lspb"),
        })
        planned = self.plan()
        self.assertEqual(planned.trajectory.profiles, ("lspb", "quintic") * 3)
        self.assertEqual(planned.trajectory.waypoint_times, (1.5, 3.5, 5.0, 7.0, 8.5, 10.5))
        self.assertEqual(tuple(record["profile"] for record in planned.waypoints), ("lspb", "quintic"))
        self.assertEqual(planned.leg_checks[-1]["cycle"], 2)
        self.task["waypoints"]["P2"]["profile"] = "unknown"
        with self.assertRaisesRegex(TaskInputError, "P2: Perfil desconocido"):
            self.plan()

    def test_cycle_permutation_keeps_explicit_home_as_terminal_pose(self):
        self.task.update(kind="cycle", cycles=1, order=["C", "A", "B", "HOME"])
        planned = self.plan()
        home = tuple(self.task["waypoints"]["HOME"]["q_rad"])
        self.assertEqual(planned.order, ("C", "A", "B", "HOME"))
        self.assertEqual(planned.required_start, home)
        self.assertEqual(planned.trajectory.segments[-1].goal, home)
        self.task["order"] = ["C", "HOME", "A", "B"]
        with self.assertRaisesRegex(TaskInputError, "seguido de HOME"):
            self.plan()
