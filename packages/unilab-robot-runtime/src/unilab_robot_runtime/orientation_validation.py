"""Reference-frame rotation-vector constraints, independent of MoveIt version semantics."""
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
from .urdf_kinematics import UrdfKinematics
from unilab_robot_contracts.planning import RobotModel

def native_local_tolerances(quaternion,reference_tolerances):
    """Conservative inscribed axis box for MoveIt 2.5 desired-link error axes."""
    rotation=abs(Rotation.from_quat(quaternion).as_matrix())
    bounds=np.asarray(reference_tolerances,dtype=float)
    local=bounds[np.argmax(rotation,axis=0)]
    extent=rotation@local
    scale=min([1.]+[bounds[i]/extent[i] for i in range(3) if extent[i]>0])
    return tuple(float(v) for v in local*scale)

@lru_cache(maxsize=64)
def _compiled_chain(urdf_xml, base_frame, link_name, joint_names, joint_limits):
    # 完整运动学输入作键：同名型号变更 URDF 时自动失效；关节状态始终重新计算。
    model = RobotModel(model_id="orientation-chain", urdf_xml=urdf_xml,
        srdf_xml='<robot name="orientation-chain"/>', base_frame=base_frame,
        flange_link=link_name, planning_group="orientation-check",
        joint_names=joint_names, joint_limits=joint_limits)
    return UrdfKinematics(model)


def _chain(robot, link):
    return _compiled_chain(robot.urdf_xml, robot.base_frame, link,
                           robot.joint_names, robot.joint_limits)


def check_orientation_constraints(robot,q,external,constraints):
    rows=[]
    for spec in constraints:
        actual=_chain(robot,spec.link_name).forward(q,external)[:3,:3]
        if spec.frame_ref!=robot.base_frame:
            reference=_chain(robot,spec.frame_ref).forward(q,external)
            actual=reference[:3,:3].T@actual
        desired=Rotation.from_quat(spec.orientation_xyzw).as_matrix()
        error=abs(Rotation.from_matrix(actual@desired.T).as_rotvec())
        bounds=np.array([spec.absolute_x_axis_tolerance_rad,spec.absolute_y_axis_tolerance_rad,
                         spec.absolute_z_axis_tolerance_rad])
        rows.append(dict(link=spec.link_name,frame=spec.frame_ref,error_vector_rad=error.tolist(),
            tolerance_rad=bounds.tolist(),satisfied=bool(np.all(error<=bounds+1e-9))))
    return rows
