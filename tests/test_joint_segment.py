import copy, math, unittest
from unilab_robot_runtime import JointSegmentExecutor as Executor

class TestExecutor(unittest.TestCase):
    def setUp(self):
        self.e=Executor(['j1'],[dict(lower=-1,upper=1,velocity=1)],'rev','session')
        self.e.observe([0.],[0.],0.,0)
        self.c=dict(command_id='a',model_revision='rev',session='session',joint_names=['j1'],observation_sequence=0,positions=[.2],duration_s=2.)
    def test_actual_feedback_required(self):
        self.e.submit(self.c)
        for i in range(1,61):self.e.observe([0.],[0.],i/10,i)
        self.assertEqual(self.e.events[-1]['state'],'ABORTED')
        self.assertFalse(any(x['state']=='SUCCEEDED' for x in self.e.events))
    def test_settle_required(self):
        self.e.submit(self.c);self.e.observe([.2],[0.],2.,1)
        self.assertIsNotNone(self.e.active)
        self.e.observe([.2],[0.],2.2,2);self.assertEqual(self.e.events[-1]['state'],'SUCCEEDED')
    def test_cancel_preserves_partial_position(self):
        self.e.submit(self.c);self.e.observe([.08],[.05],1.,1);self.e.cancel('a')
        self.assertEqual(self.e.setpoint(2),[.08]);self.assertIsNotNone(self.e.active)
        self.e.observe([.081],[0.],1.1,2);self.e.observe([.081],[0.],1.3,3)
        self.assertEqual(self.e.events[-1]['state'],'CANCELED')
        self.assertAlmostEqual(self.e.events[-1]['actual_positions'][0],.081)
    def test_reject_bad_commands(self):
        mutations=[('positions',[math.nan]),('positions',[True]),('positions',[2.]),('positions',[]),
                   ('joint_names',['wrong']),('joint_names',['j1','j1']),('model_revision','old'),('session','old'),
                   ('observation_sequence',False),('observation_sequence',-1),('duration_s',0),('duration_s',True),('duration_s',.01),('command_id','')]
        for k,v in mutations:
            with self.subTest(k=k,v=v):
                c=copy.deepcopy(self.c);c[k]=v
                with self.assertRaises(ValueError):self.e.submit(c)
                self.assertIsNone(self.e.active)
    def test_one_owner(self):
        self.e.submit(self.c)
        with self.assertRaisesRegex(ValueError,'busy'):self.e.submit(self.c)
        with self.assertRaises(ValueError):self.e.cancel('another')
        self.assertEqual(self.e.active['command_id'],'a')
    def test_stale_observation(self):
        with self.assertRaises(ValueError):self.e.observe([0],[0],0,0)
    def test_replay_rejected_after_complete(self):
        self.e.submit(self.c);self.e.observe([.2],[0],2,1);self.e.observe([.2],[0],2.2,2)
        self.c['observation_sequence']=2
        with self.assertRaisesRegex(ValueError,'reused'):self.e.submit(self.c)
    def test_terminal_target_is_held(self):
        self.e.submit(self.c);self.e.observe([.2],[0],2,1);self.e.observe([.2],[0],2.2,2)
        self.e.observe([.199],[0],2.3,3)
        self.assertEqual(self.e.setpoint(2.4),[.2])
    def test_smooth_midpoint(self):
        self.e.submit(self.c)
        self.assertAlmostEqual(self.e.setpoint(1)[0],.1)
        self.assertEqual(self.e.setpoint(2),[.2])

if __name__=='__main__':unittest.main()
