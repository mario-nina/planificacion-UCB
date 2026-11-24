from pathlib import Path
from unittest import TestCase
import yaml
from imt342_irb120_kinematics import Model
from imt342_irb120_kinematics.analysis import PointModels
from imt342_irb120_kinematics.cell import Cell
from imt342_irb120_planning import AUTO_REFERENCE_VELOCITIES
from imt342_irb120_planning.cell_continuous import plan_continuous_wall
from imt342_irb120_planning.tracking import reference_limits
from imt342_irb120_planning.task import TaskInputError


class ContinuousWallTests(TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parents[2]
        desc=root/'imt342_irb120_description'
        xml=(desc/'urdf/irb120.urdf').read_text()
        cls.points=PointModels.from_model(Model.from_xml(xml))
        config=yaml.safe_load((root/'imt342_irb120_bringup/config/cell.yaml').read_text())
        cls.cell=Cell.from_urdf(config,xml,desc)
        cls.home=(0.,)*6
        cls.plan,cls.evidence=plan_continuous_wall(cls.points,cls.home,dict(kind='cell_wall',cycles=2),
                                                  AUTO_REFERENCE_VELOCITIES,cls.cell,max_duration=30.)

    def test_grouped_linear_passes_repeat_and_return_home(self):
        legs=self.plan.trajectory.segments
        self.assertEqual(len(legs),12)
        self.assertEqual(legs[:6],legs[6:])
        self.assertEqual(self.plan.order,('APROX','INICIO','IDA_CONTINUA','VUELTA_CONTINUA','SALIDA','HOME'))
        self.assertEqual(legs[-1].goal,self.home)
        self.assertEqual([getattr(s,'linear_cartesian',False) for s in legs[:6]], [False,False,True,True,False,False])
        self.assertTrue(self.cell.check_trajectory(self.points,self.plan.trajectory).ok)
        self.assertLess(self.evidence['max_tcp_deviation_m'],.001)
        self.assertAlmostEqual(self.evidence['wall_clearance_min_m'],.1,delta=.01)

    def test_no_internal_stop_and_bounded_reference(self):
        for leg in self.plan.trajectory.segments[2:4]:
            self.assertEqual(leg.sample(0).velocities,(0.,)*6)
            self.assertEqual(leg.sample(leg.duration).velocities,(0.,)*6)
            for t in range(1,100):
                s=leg.sample(leg.duration*t/100)
                self.assertGreater(max(map(abs,s.velocities)),1e-5)
                self.assertLessEqual(max(map(abs,s.velocities)),.6+1e-8)
                self.assertLessEqual(max(map(abs,s.accelerations)),1.+1e-8)
            self.assertEqual(reference_limits(leg)['shape'],'LSPB cartesiano')

    def test_non_flange_path_not_mislabeled(self):
        with self.assertRaises(TaskInputError):
            plan_continuous_wall(self.points,self.home,dict(kind='cell_wall',cycles=1,point='tcp'),
                                 AUTO_REFERENCE_VELOCITIES,self.cell)
