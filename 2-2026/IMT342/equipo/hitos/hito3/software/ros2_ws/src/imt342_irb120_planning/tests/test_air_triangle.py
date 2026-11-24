from pathlib import Path
from math import dist
from unittest import TestCase
import yaml
from imt342_irb120_kinematics import Model
from imt342_irb120_kinematics.analysis import PointModels
from imt342_irb120_kinematics.cell import Cell
from imt342_irb120_planning import AUTO_REFERENCE_VELOCITIES
from imt342_irb120_planning.air_triangle import plan_air_triangle, MAX_DEVIATION_M
from imt342_irb120_planning.task import TaskInputError

class AirTriangleTest(TestCase):
    @classmethod
    def setUpClass(cls):
        p=Path(__file__).resolve().parents[2]
        d=p/'imt342_irb120_description'
        xml=(d/'urdf/irb120.urdf').read_text()
        cls.points=PointModels.from_model(Model.from_xml(xml))
        cls.cell=Cell.from_urdf(yaml.safe_load((p/'imt342_irb120_bringup/config/cell.yaml').read_text()),xml,d)

    def test_both_profiles_repeat_with_one_destination_per_side(self):
        for profile in ('lspb','quintic'):
            with self.subTest(profile=profile):
                home=(0.,)*6
                plan,e=plan_air_triangle(self.points,home,dict(kind='air_triangle',cycles=2,profile=profile),
                                        AUTO_REFERENCE_VELOCITIES,self.cell,max_duration=30.)
                self.assertEqual(plan.order,('APROX','A','B','C','CIERRE_A','SALIDA','HOME'))
                self.assertEqual(plan.trajectory.segments[:7],plan.trajectory.segments[7:])
                self.assertEqual(plan.required_start,home)
                self.assertEqual(plan.trajectory.segments[-1].goal,home)
                self.assertEqual(plan.trajectory.profiles,(profile,)*14)
                v=e['reference_xyz_m']; self.assertEqual(v[0],v[-1])
                for a,b in zip(v,v[1:]): self.assertAlmostEqual(dist(a,b),.1)
                self.assertLessEqual(e['max_tcp_deviation_m'],MAX_DEVIATION_M)
                self.assertTrue(self.cell.check_trajectory(self.points,plan.trajectory).ok)
                self.assertEqual(plan.trajectory.segments[1].goal,plan.trajectory.segments[4].goal)
                self.assertGreater(e['height_above_wall_m'],.15)
                self.assertTrue(all(.1<=s.duration<=30 for s in plan.trajectory.segments))

    def test_invalid_inputs_rejected(self):
        for task in (dict(kind='air_triangle',cycles=0),dict(kind='air_triangle',cycles=True),
                     dict(kind='air_triangle',cycles=1,point='tcp'),dict(kind='air_triangle',cycles=1,order=['A'])):
            with self.subTest(task=task), self.assertRaises(TaskInputError):
                plan_air_triangle(self.points,(0.,)*6,task,AUTO_REFERENCE_VELOCITIES,self.cell)
        with self.assertRaises(TaskInputError):
            plan_air_triangle(self.points,(3.,0.,0.,0.,0.,0.),dict(kind='air_triangle',cycles=1),AUTO_REFERENCE_VELOCITIES,self.cell)
