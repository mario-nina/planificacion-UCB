from math import degrees, radians
from pathlib import Path
from unittest import TestCase

from imt342_irb120_control.lspb_calibration import (
    AccelerationCalibration, fit_segment, latest_derivatives, progress,
)
from imt342_irb120_control.rws_lspb_revision import CANDIDATE_SOURCE, CANDIDATE_SHA256
from imt342_irb120_control.rws_backend import RwsError
from imt342_irb120_control.state import Result
from test_rws_trajectory import CONFIG, START, GOAL, ORDER, TrajectoryHttp
from imt342_irb120_control.rws_trajectory import RwsTrajectoryBackend

TARGET = (.8, -.4, .3, 0., 0., 0.)
KEY = (START, TARGET, 3., .4)
TUNE = ORDER.replace('trjOrder','trjTune')


def synthetic(f=.22, dt=.04, duration=3., onset=.12):
    return [(i*dt, tuple(x*progress((i*dt-onset)/duration, f) for x in TARGET))
            for i in range(round((onset+duration+.16)/dt)+1)]


def observed_fit(f=.22,dt=.04,duration=3.):
    samples=synthetic(f,dt,duration)
    return fit_segment(*KEY,samples,0.,.14,samples[-1][0])


class CalibrationTests(TestCase):
    def test_identifies_equivalent_profile_from_positions(self):
        fit=observed_fit()
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit['ramp_s'],.66,delta=.09)
        self.assertAlmostEqual(fit['fitted_duration_s'],3.,delta=.12)
        self.assertGreater(fit['acceleration_ratio'],1.15)
        self.assertLess(fit['rmse_progress'],.012)
        self.assertEqual(fit['timing'],'host_fit_not_servo')

    def test_rejects_undersampled_noise_and_cross_axis_motion(self):
        self.assertIsNone(observed_fit(dt=.3))
        self.assertIsNone(observed_fit(dt=.15,f=.09))
        samples=synthetic()
        samples[20]=(samples[20][0], (*samples[20][1][:5],.4))
        self.assertIsNone(fit_segment(*KEY,samples,0.,.14,samples[-1][0]))
        self.assertIsNone(fit_segment(START,(.01,)*6,3.,.4,synthetic(),0.,.14,3.5))

    def test_reduction_requires_two_consistent_trials_and_exact_direction(self):
        c=AccelerationCalibration(); fit=observed_fit()
        self.assertEqual(c.ratio(KEY,1)[0],1.)
        c.record(KEY,fit,1); self.assertEqual(c.ratio(KEY,2)[0],1.)
        c.record(KEY,fit,2); ratio=c.ratio(KEY,3)[0]
        self.assertGreaterEqual(ratio,.8); self.assertLess(ratio,1.)
        self.assertEqual(c.ratio((TARGET,START,3.,.4),3)[0],1.)
        self.assertEqual(c.ratio(KEY,700)[0],1.)
        c.record(KEY,None,4,ratio=ratio)
        self.assertEqual(c.ratio(KEY,5)[0],1.)

    def test_successful_reduction_reused_without_cumulative_adjustment(self):
        c=AccelerationCalibration(); f=observed_fit()
        c.record(KEY,f,1); c.record(KEY,f,2); ratio=c.ratio(KEY,3)[0]
        better=observed_fit(f=.4)
        c.record(KEY,better,4,ratio=ratio)
        self.assertEqual(c.ratio(KEY,5)[0],ratio)
        self.assertLess(c.score(better),c.score(f))

    def test_nonuniform_derivatives_at_last_actual_time(self):
        stamps=(.1,.23,.4)
        samples=[(round(t*1e9),(1+2*t+3*t*t,)*6) for t in stamps]
        v,a=latest_derivatives(samples)
        for x,y in zip(v,a):
            self.assertAlmostEqual(x,4.4); self.assertAlmostEqual(y,6.)
        self.assertIsNone(latest_derivatives(samples[:2]))
        self.assertIsNone(latest_derivatives([(0,START),(1000000,START),(300000000,START)]))


class CandidateHttp(TrajectoryHttp):
    def __init__(self, source):
        super().__init__(source)
        self.fail_tune=False
        self.tune=None
        self.timing="[0,0,0,0,0]"

    def request(self,method,url,body):
        if method=='POST' and url==TUNE:
            self.calls.append((method,url,body))
            if not self.mastership or self.fail_tune:
                return 403,''
            self.tune=body['value']
            return 204,''
        if method == 'GET' and url.split('?')[0].endswith('/trjTiming'):
            from test_rws_trajectory import state
            self.calls.append((method,url,body))
            return state(value=self.timing)
        return super().request(method,url,body)


class RevisionTests(TestCase):
    def setUp(self):
        import os
        root=Path(__file__).resolve().parents[4]
        self.original=Path(os.environ.get('IMT342_RWS_TRAJECTORY_SOURCE',root/'experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod')).read_bytes()
        self.now=1.
        self.http=CandidateHttp(CANDIDATE_SOURCE.decode('ascii'))
        self.backend=RwsTrajectoryBackend(CONFIG,self.original,self.http,lambda:self.now,allow_candidate=True)

    def execute(self,blend=.4):
        return self.backend.execute(TARGET,duration=3.,authorized=lambda:True,expected_start=START,reference_blend=blend)

    def test_recognizes_pinned_candidate_read_only_and_normalizes_persistent_tune(self):
        self.http.source=self.http.source.replace('trjTune := [0,1];','trjTune := [4,0.8];')
        self.assertEqual(self.backend.prepare(),0)
        self.assertEqual(self.backend.module_revision,'lspb_candidate')
        self.assertEqual(self.backend.controller_metadata['module_sha256'],CANDIDATE_SHA256)
        self.assertFalse(any(m=='POST' for m,_,_ in self.http.calls))

    def test_original_contract_gets_no_tuning_write(self):
        self.http.source=self.original.decode('ascii')
        self.assertEqual(self.execute(),Result.TRANSMITTED)
        self.assertEqual(self.backend.module_revision,'original')
        self.assertIsNone(self.http.tune)
        self.assertEqual(len([p for m,p,b in self.http.calls if m=='POST' and p==ORDER]),1)

    def test_calibrated_candidate_sends_sequence_bound_metadata_before_order(self):
        self.backend.prepare()
        f=observed_fit()
        self.backend._calibration.record(KEY,f,.5);self.backend._calibration.record(KEY,f,.8)
        self.assertEqual(self.execute(),Result.TRANSMITTED)
        self.assertEqual(self.http.tune,'[1,0.80000000]')
        posts=[p for m,p,b in self.http.calls if m=='POST']
        self.assertLess(posts.index(TUNE),posts.index(ORDER))
        self.assertEqual(self.backend.motion_evidence['lspb_acceleration_ratio_requested'],.8)
        self.http.reply='[1,3]';self.http.q_deg=tuple(map(degrees,TARGET));self.now=4.1
        self.http.timing='[1,3.0,0.02,80,64]' 
        self.assertEqual(self.backend.result(),Result.VERIFIED)
        # An undersampled adjusted trial cannot be treated as a good fit.
        self.assertIsNone(self.backend.motion_evidence['lspb_equivalent_fit'])
        self.assertEqual(self.backend.motion_evidence['controller_timing_s'],3.)
        self.assertEqual(self.backend.motion_evidence['controller_applied_acc_percent'],64.)
        self.assertEqual(self.backend._calibration.ratio(KEY,self.now)[0],1.)

    def test_no_calibration_or_quintic_is_neutral_and_failed_write_does_not_move(self):
        self.http.fail_tune=True
        with self.assertRaises(RwsError): self.execute(None)
        self.assertFalse(any(m=='POST' and p==ORDER for m,p,b in self.http.calls))
        self.assertFalse(self.http.mastership)
        self.http.fail_tune=False
        self.assertEqual(self.execute(None),Result.TRANSMITTED)
        self.assertEqual(self.http.tune,'[1,1.00000000]')

    def test_modified_candidate_rejected(self):
        self.http.source=self.http.source.replace('limitedAcc := 20;','limitedAcc := 100;')
        with self.assertRaises(RwsError): self.execute()
        self.assertFalse(any(m=='POST' for m,_,_ in self.http.calls))

    def test_installer_backend_accepts_only_original_source(self):
        plain=RwsTrajectoryBackend(CONFIG,self.original,self.http,lambda:self.now)
        with self.assertRaises(RwsError): plain.prepare()
        self.assertFalse(any(m=='POST' for m,_,_ in self.http.calls))

    def test_embedded_candidate_matches_reviewable_module(self):
        import os
        root=Path(__file__).resolve().parents[4]
        original=Path(os.environ.get('IMT342_RWS_TRAJECTORY_SOURCE',root/'experimental/trajectory_control/rws_rapid/Imt342TrajectoryRapid.mod'))
        self.assertEqual(original.with_name('Imt342TrajectoryRapidLspb.mod').read_bytes(),CANDIDATE_SOURCE)

    def test_two_measured_orders_learn_without_synthetic_seeding(self):
        # The HTTP double feeds real-backend code with a known synthetic trace;
        # this is not evidence of a physical ABB motion.
        for seq in (1,2):
            self.http.q_deg=START
            self.assertEqual(self.execute(),Result.TRANSMITTED)
            self.assertEqual(self.http.tune,f'[{seq},1.00000000]')
            sent=self.now
            self.http.reply=f'[{seq},2]';self.now=sent+.12
            self.assertEqual(self.backend.result(),Result.EXECUTING)
            for t,q in synthetic():
                if t >= .12:
                    self.now=sent+t;self.http.q_deg=tuple(map(degrees,q));self.backend.read_state()
            self.http.q_deg=tuple(map(degrees,TARGET));self.now+=.01
            self.http.timing=f'[{seq},3,0.01,80,80]';self.http.reply=f'[{seq},3]'
            self.assertEqual(self.backend.result(),Result.VERIFIED)
            self.assertIsNotNone(self.backend.motion_evidence['lspb_equivalent_fit'])
            self.now+=.1
        self.http.q_deg=START
        self.assertEqual(self.execute(),Result.TRANSMITTED)
        self.assertEqual(self.http.tune,'[3,0.80000000]')

    def test_stale_controller_clock_never_invalidates_terminal_arrival(self):
        self.assertEqual(self.execute(),Result.TRANSMITTED)
        self.http.q_deg=tuple(map(degrees,TARGET));self.http.reply='[1,3]';self.now=4.1
        self.http.timing='[0,3,0.01,80,80]'
        self.assertEqual(self.backend.result(),Result.VERIFIED)
        self.assertIsNone(self.backend.motion_evidence['controller_timing_s'])
        self.assertEqual(self.backend.motion_evidence['controller_timing_unavailable'],'ValueError')
