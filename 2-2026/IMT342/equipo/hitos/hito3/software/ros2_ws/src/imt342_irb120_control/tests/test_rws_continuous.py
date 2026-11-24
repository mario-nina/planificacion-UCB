from unittest import TestCase
from pathlib import Path
from os import environ
from imt342_irb120_control.rws_continuous_revision import CONTINUOUS_SOURCE, CONTINUOUS_SHA256
from imt342_irb120_control.rws_trajectory import RwsTrajectoryBackend
from imt342_irb120_control.rws_backend import RwsError
from imt342_irb120_control.state import Result
from test_lspb_calibration import CandidateHttp
from test_rws_trajectory import CONFIG, START, GOAL, ORDER

LINEAR=ORDER.replace('trjOrder','trjLinear')

class LinearHttp(CandidateHttp):
    def __init__(self,source):
        super().__init__(source)
        self.linear=None
        self.fail_linear=False
    def request(self,method,url,body):
        if method=='POST' and url==LINEAR:
            self.calls.append((method,url,body))
            if not self.mastership or self.fail_linear: return 403,''
            self.linear=body['value'];return 204,''
        return super().request(method,url,body)

class ContinuousRevisionTests(TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[4]
        self.source=Path(environ.get('IMT342_RWS_TRAJECTORY_SOURCE',root/'experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod')).read_bytes()
        self.http=LinearHttp(CONTINUOUS_SOURCE.decode('ascii'))
        self.backend=RwsTrajectoryBackend(CONFIG,self.source,self.http,lambda:1.,allow_candidate=True)
    def execute(self,linear):
        return self.backend.execute(GOAL,duration=3.,authorized=lambda:True,expected_start=START,
                                    reference_blend=.25,linear_cartesian=linear)
    def test_mode_correlated_before_order_and_not_calibrated_as_joint_chord(self):
        self.http.source=self.http.source.replace('trjLinear := [0,0];','trjLinear := [42,1];')
        self.assertEqual(self.backend.prepare(),0)
        self.assertEqual(self.backend.controller_metadata['module_sha256'],CONTINUOUS_SHA256)
        self.assertEqual(self.execute(True),Result.TRANSMITTED)
        self.assertEqual(self.http.linear,'[1,1]')
        posts=[url for method,url,_ in self.http.calls if method=='POST']
        self.assertLess(posts.index(LINEAR),posts.index(ORDER))
        self.assertEqual(self.backend.motion_evidence['motion_instruction'],'MoveL')
        self.assertIsNone(self.backend._shape_key)
    def test_joint_order_keeps_moveabsj(self):
        self.assertEqual(self.execute(False),Result.TRANSMITTED)
        self.assertEqual(self.http.linear,'[1,0]')
        self.assertEqual(self.backend.motion_evidence['motion_instruction'],'MoveAbsJ')
    def test_failed_metadata_does_not_dispatch(self):
        self.http.fail_linear=True
        with self.assertRaises(RwsError): self.execute(True)
        self.assertFalse(any(method=='POST' and url==ORDER for method,url,_ in self.http.calls))
        self.assertFalse(self.http.mastership)
    def test_original_never_silently_executes_linear_as_joint(self):
        self.http.source=self.source.decode('ascii')
        with self.assertRaises(RwsError): self.execute(True)
        self.assertFalse(any(method=='POST' and url==ORDER for method,url,_ in self.http.calls))
    def test_source_contains_one_movel_and_original_joint_fallback(self):
        self.assertEqual(CONTINUOUS_SOURCE.count(b'MoveL linearTarget'),1)
        self.assertIn(b'MoveAbsJ requested.target',CONTINUOUS_SOURCE)
        self.assertIn(b'ConfL \\On;',CONTINUOUS_SOURCE)
        self.assertIn(b'IF trjLinear.seq <> requested.seq',CONTINUOUS_SOURCE)
        # Source packaged with host revision must be byte identical.
        original=Path(environ.get('IMT342_RWS_TRAJECTORY_SOURCE',Path(__file__).resolve().parents[4]/'experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod'))
        self.assertEqual(original.with_name('Imt342TrajectoryRapidContinuous.mod').read_bytes(),CONTINUOUS_SOURCE)
