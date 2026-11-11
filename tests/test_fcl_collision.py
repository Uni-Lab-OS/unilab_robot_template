import unittest
import numpy as np
from unilab_robot_contracts.planning import CollisionGeometry,CollisionObject
from unilab_robot_runtime.fcl_collision import FCLSceneChecker,joint_samples
I=tuple(map(tuple,np.eye(4)))
def pose(x=0,y=0,z=0):
 p=np.eye(4);p[:3,3]=[x,y,z];return tuple(map(tuple,p))
def box(name,xyz=(0,0,0),size=(.1,.1,.1)):
 return CollisionObject(name,pose(*xyz),(CollisionGeometry('box',size_m=size),))
class CollisionTests(unittest.TestCase):
 def test_thin_wall_and_allowed_pair_reset(self):
  c=FCLSceneChecker([box('tool',(-.2,0,0)),box('wall',size=(.001,1.,1.))])
  self.assertFalse(c.check({'tool':pose(-.2)},moving_ids=['tool'])['collision'])
  self.assertTrue(c.check({'tool':pose()},moving_ids=['tool'])['collision'])
  self.assertFalse(c.check({'tool':pose()},moving_ids=['tool'],allowed_pairs=[('tool','wall')])['collision'])
  self.assertTrue(c.check({'tool':pose()},moving_ids=['tool'])['collision'])
 def test_sibling_attachments_are_checked(self):
  c=FCLSceneChecker([box('finger'),box('payload')]);r=c.check({'finger':I,'payload':I},moving_ids=['finger','payload'])
  self.assertTrue(r['collision']);self.assertEqual(r['eligible_pairs'],1)
 def test_native_solid_proxy_detects_contained_triangle(self):
  mesh=CollisionGeometry('mesh',vertices=((-0.01,0.,0.),(.01,0.,0.),(0.,.01,0.)),triangles=((0,1,2),))
  c=FCLSceneChecker([box('solid'),CollisionObject('thin',I,(mesh,))])
  self.assertTrue(c.check({'thin':I},moving_ids=['thin'])['collision'])
 def test_opening_is_preserved(self):
  walls=CollisionObject('rack',I,(CollisionGeometry('box',size_m=(.01,.2,.2)),CollisionGeometry('box',size_m=(.01,.2,.2))),geometry_poses=(pose(-.08),pose(.08)))
  c=FCLSceneChecker([walls,box('probe',size=(.02,.02,.02))])
  self.assertFalse(c.check({'probe':I},moving_ids=['probe'])['collision'])
  self.assertTrue(c.check({'probe':pose(.08)},moving_ids=['probe'])['collision'])
 def test_invalid_scope_fails_closed(self):
  c=FCLSceneChecker([box('a'),box('b',(2,0,0))])
  for kwargs in [dict(poses={},moving_ids=['a']),dict(poses={'a':I},moving_ids=['ghost']),dict(poses={'a':I},moving_ids=['a'],allowed_pairs=[('a','ghost')]),dict(poses={'a':I},moving_ids=['a'],allowed_pairs=[('a','b'),('b','a')])]:
   with self.assertRaises(ValueError):c.check(**kwargs)
 def test_static_pairs_are_omitted_but_moving_pairs_remain(self):
  c=FCLSceneChecker([box('a'),box('b'),box('tool',(3,0,0))]);r=c.check({'tool':pose(3)},moving_ids=['tool'])
  self.assertFalse(r['collision']);self.assertEqual(r['eligible_pairs'],2);self.assertEqual(r['narrow_phase_calls'],0)
 def test_subdivision_keeps_internal_points_and_endpoints(self):
  path=[[0.,0.],[.003,0.],[.003,.005]];samples=list(joint_samples(path,max_step_rad=.002))
  self.assertEqual(samples[0],path[0]);self.assertEqual(samples[-1],path[-1]);self.assertIn(path[1],samples)
  self.assertLessEqual(np.max(abs(np.diff(samples,axis=0))),.002)
  with self.assertRaises(ValueError):list(joint_samples(path,max_samples=2))
if __name__=='__main__':unittest.main()


class CompoundTransformTests(unittest.TestCase):
 def test_rotated_local_shape_and_stale_native_pose(self):
  local=np.eye(4);local[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];local[0,3]=1.
  obj=CollisionObject('arm',I,(CollisionGeometry('box',size_m=(.6,.02,.02)),),geometry_poses=(tuple(map(tuple,local)),))
  c=FCLSceneChecker([obj,box('probe',size=(.01,.01,.01))])
  self.assertTrue(c.check({'arm':I,'probe':pose(1,.25)},moving_ids=['arm','probe'])['collision'])
  self.assertFalse(c.check({'arm':pose(5),'probe':pose()},moving_ids=['arm','probe'])['collision'])
  rotated=np.eye(4);rotated[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
  self.assertTrue(c.check({'arm':rotated,'probe':pose(-.25,1)},moving_ids=['arm','probe'])['collision'])
  self.assertFalse(c.check({'arm':rotated,'probe':pose(.25,1.2)},moving_ids=['arm','probe'])['collision'])


def test_overflowing_path_fails_before_yielding_any_point():
 import pytest
 with pytest.raises(ValueError,match='budget'):
  next(joint_samples([[-1e308],[1e308]]))

def test_string_pair_cannot_silently_disable_collision():
 import pytest
 checker=FCLSceneChecker([box('a'),box('b')])
 with pytest.raises(ValueError,match='pair'):
  checker.check({'a':I},moving_ids=['a'],allowed_pairs=['ab'])


def test_generator_input_retains_all_shapes():
 checker=FCLSceneChecker(obj for obj in [box('a'),box('b')])
 assert checker.check({'a':I},moving_ids=['a'])['collision']
