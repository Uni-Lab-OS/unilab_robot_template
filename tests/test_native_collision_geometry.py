"""真实 ROS 消息中的复合几何局部位姿，两个规划后端须一致。"""
import math
import unittest
from unilab_robot_contracts.planning import CollisionObject, CollisionGeometry
from unilab_robot_runtime.moveit_planner import _object_to_native as moveitpy_object
from unilab_robot_runtime.moveit_move_group import _object_to_native as movegroup_object

IDENTITY=((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))
WORLD=((1.,0.,0.,4.),(0.,1.,0.,5.),(0.,0.,1.,6.),(0.,0.,0.,1.))
LOCAL=((0.,-1.,0.,1.),(1.,0.,0.,2.),(0.,0.,1.,3.),(0.,0.,0.,1.))


class NativeCollisionGeometryTest(unittest.TestCase):
    def test_compound_local_poses_and_mesh_scale_survive(self):
        obj=CollisionObject('fixture',WORLD,(
            CollisionGeometry('box',size_m=(.16,.16,.05)),
            CollisionGeometry('mesh',vertices=((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)),
                triangles=((0,1,2),),mesh_scale=(.5,2.,3.))),geometry_poses=(LOCAL,LOCAL))
        results=[]
        for converter in (moveitpy_object,movegroup_object):
            native=converter(obj);results.append(native)
            self.assertEqual(native.pose.position.x,4.)
            self.assertEqual(native.pose.position.y,5.)
            self.assertEqual(native.pose.position.z,6.)
            for pose in (native.primitive_poses[0],native.mesh_poses[0]):
                self.assertEqual((pose.position.x,pose.position.y,pose.position.z),(1.,2.,3.))
                self.assertAlmostEqual(abs(pose.orientation.z),math.sqrt(.5))
                self.assertAlmostEqual(abs(pose.orientation.w),math.sqrt(.5))
            self.assertEqual(native.meshes[0].vertices[1].x,.5)
            self.assertEqual(native.meshes[0].vertices[2].y,2.)
            self.assertEqual(list(native.primitives[0].dimensions),[.16,.16,.05])
        self.assertEqual(results[0],results[1])

    def test_omitted_local_pose_stays_identity(self):
        obj=CollisionObject('fixture',WORLD,(CollisionGeometry('box',size_m=(1.,2.,3.)),))
        for converter in (moveitpy_object,movegroup_object):
            pose=converter(obj).primitive_poses[0]
            self.assertEqual((pose.position.x,pose.position.y,pose.position.z),(0.,0.,0.))
            self.assertEqual(pose.orientation.w,1.)


if __name__=='__main__':unittest.main(verbosity=2)
