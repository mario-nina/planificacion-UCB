import unittest
import numpy as np
from src.irb120_fk import forward_kinematics,forward_kinematics_tcp
from src.pieper import solve_pieper

class TestPieper(unittest.TestCase):
    def check_solutions(self, q, frame='flange', expect_eight=False):
        T=forward_kinematics(q) if frame=='flange' else forward_kinematics_tcp(q)
        result=solve_pieper(T,target_frame=frame)
        if expect_eight:self.assertEqual(len(result.solutions),8)
        self.assertTrue(len(result.solutions)>0,result.diagnostics)
        for sol in result.solutions:
            T2=forward_kinematics(sol.q) if frame=='flange' else forward_kinematics_tcp(sol.q)
            np.testing.assert_allclose(T2,T,atol=1e-8,rtol=0)
        return result

    def test_eight_branches(self):
        from math import atan2,sqrt,acos
        rho=np.hypot(.4,.1); h=.138; L=np.hypot(.07,.302);beta=atan2(.302,.07)
        d=acos((rho*rho+h*h-.270**2-L**2)/(2*.270*L))
        q=[atan2(.1,.4),atan2(rho,h)-atan2(L*np.sin(d),.270+L*np.cos(d)),d-beta,.5,-.6,.7]
        s=self.check_solutions(q,expect_eight=True)
        np.testing.assert_allclose(s.wrist_center_m,[.4,.1,.428],atol=1e-12)
        self.assertTrue(any(np.linalg.norm((sol.q-q+np.pi)%(2*np.pi)-np.pi)<1e-8 for sol in s.solutions))

    def test_tcp(self):self.check_solutions([.3,-.4,.2,.3,-.2,.8],frame='tcp')
    def test_zero_wrist(self):
        res=self.check_solutions([0,0,0,0,0,0])
        self.assertTrue(any(s.singular for s in res.solutions))
    def test_pi_wrist(self):
        res=self.check_solutions([.2,-.2,.1,1.,np.pi,.3])
        self.assertTrue(any(s.singular for s in res.solutions))
    def test_unreachable(self):
        T=np.eye(4);T[:3,3]=[3.,0.,0.5]
        self.assertEqual(len(solve_pieper(T).solutions),0)
    def test_invalid(self):
        with self.assertRaises(ValueError):solve_pieper(np.ones((4,4)))
    def test_many_random(self):
        rng=np.random.default_rng(777)
        for _ in range(100):
            q=rng.uniform(-2,2,6)
            self.check_solutions(q)

if __name__=='__main__':unittest.main()
