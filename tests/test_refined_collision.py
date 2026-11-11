import unittest
from collections.abc import Mapping
import numpy as np
from unilab_robot_contracts.planning import CollisionGeometry,CollisionObject
from unilab_robot_runtime.refined_collision import RefinedCollisionChecker


def pose(x=0,y=0,z=0):
 p=np.eye(4);p[:3,3]=[x,y,z];return tuple(map(tuple,p))

def box(name,size,at=None):
 return CollisionObject(name,at or pose(),(CollisionGeometry('box',size_m=size),))

class Counted(Mapping):
 def __init__(self,objects):self.objects={o.object_id:o for o in objects};self.reads=[]
 def __iter__(self):return iter(self.objects)
 def __len__(self):return len(self.objects)
 def __getitem__(self,k):self.reads.append(k);return self.objects[k]

class RefinedTests(unittest.TestCase):
 def test_opening_refined_and_solid_wall_retained(self):
  outer=box('fixture',(2.,2.,1.));probe=box('probe',(.1,.1,.1))
  exact=CollisionObject('fixture',pose(),tuple(CollisionGeometry('box',size_m=size) for size in [(0.2,2.,1.),(.2,2.,1.),(2.,.2,1.)]),geometry_poses=(pose(-.9),pose(.9),pose(y=-.9)))
  source=Counted([exact,probe]);checker=RefinedCollisionChecker([outer,probe],source)
  self.assertEqual(source.reads,[])
  free=checker.check({'probe':pose()},moving_ids=['probe'])
  self.assertFalse(free['collision']);self.assertEqual(free['outer_hit_pairs'],1)
  self.assertEqual(len(source.reads),2)
  hit=checker.check({'probe':pose(.9)},moving_ids=['probe'])
  self.assertTrue(hit['collision']);self.assertEqual(hit['new_exact_pair_caches'],0)
  self.assertFalse(checker.check({'probe':pose(.9)},moving_ids=['probe'],allowed_pairs=[('fixture','probe')])['collision'])
  self.assertTrue(checker.check({'probe':pose(.9)},moving_ids=['probe'])['collision'])
 def test_free_broadphase_does_not_load_exact(self):
  a=box('a',(1.,1.,1.));b=box('b',(.1,.1,.1),pose(10))
  source=Counted([a,b]);checker=RefinedCollisionChecker([a,b],source)
  self.assertFalse(checker.check({'b':pose(10)},moving_ids=['b'])['collision'])
  self.assertEqual(source.reads,[])
 def test_missing_exact_source_fails_closed(self):
  a=box('a',(1.,1.,1.));b=box('b',(1.,1.,1.))
  with self.assertRaisesRegex(ValueError,'identities'):RefinedCollisionChecker([a,b],{'a':a})

if __name__=='__main__':unittest.main()
