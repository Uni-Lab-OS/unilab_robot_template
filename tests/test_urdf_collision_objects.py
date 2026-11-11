import unittest
import numpy as np
from unilab_robot_runtime.urdf_assets import collision_objects

class AuthoredCollisionTests(unittest.TestCase):
 def test_visual_only_is_not_a_collider(self):
  self.assertEqual(collision_objects('<robot><link name="visual"><visual><geometry><box size="9 9 9"/></geometry></visual></link></robot>',{},root='.'),())
 def test_collision_origin_and_live_link_transform_are_separate(self):
  xml='<robot><link name="moving"><collision><origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/><geometry><box size="1 2 3"/></geometry></collision><collision><geometry><cylinder radius="0.04" length="0.3"/></geometry></collision></link></robot>'
  frame=np.eye(4);frame[0,3]=2.
  obj,=collision_objects(xml,{'moving':frame},root='.')
  self.assertEqual(obj.object_id,'moving')
  np.testing.assert_allclose(obj.world_pose,frame)
  np.testing.assert_allclose(np.asarray(obj.geometry_poses[0])[:3,3],[.1,.2,.3])
  np.testing.assert_allclose(np.asarray(obj.geometry_poses[0])[:2,:2],[[0,-1],[1,0]],atol=1e-12)
  self.assertEqual(obj.geometry[0].size_m,(1.,2.,3.))
  self.assertEqual(obj.geometry[1].radius_m,.04)
 def test_unsupported_shape_fails_closed(self):
  with self.assertRaises(ValueError):collision_objects('<robot><link name="a"><collision><geometry><unknown/></geometry></collision></link></robot>',{'a':np.eye(4)},root='.')

if __name__=='__main__':unittest.main()
