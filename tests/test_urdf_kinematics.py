import numpy as np
from unilab_robot_runtime.urdf_kinematics import UrdfKinematics
from test_planning_contract import _model

def test_one_joint_chain_fixed_origin_and_external_axis():
    from dataclasses import replace
    robot=replace(_model(),urdf_xml='<robot name="r"><link name="world"/><link name="a"/><link name="link6"/><joint name="rail" type="prismatic"><parent link="world"/><child link="a"/><axis xyz="1 0 0"/></joint><joint name="j" type="revolute"><parent link="a"/><child link="link6"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/></joint></robot>',
        base_frame="world",joint_names=("j",),joint_limits=((-3.14,3.14),))
    kin=UrdfKinematics(robot)
    pose=kin.forward((.4,),{"rail":.2})
    assert np.allclose(pose[:3,3],[.2,0,1])
    assert np.allclose(pose[:2,:2],[[np.cos(.4),-np.sin(.4)],[np.sin(.4),np.cos(.4)]])
    assert np.allclose(kin.inverse(pose,(.1,),{"rail":.2}),(.4,),atol=1e-5)
