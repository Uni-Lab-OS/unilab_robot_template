"""原生 MoveItPy 状态接口回归；未安装 MoveIt 时明确跳过。"""
import contextlib
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    from moveit.core.robot_model import RobotModel as NativeModel
    from moveit.core.robot_state import RobotState
except ImportError:
    NativeModel = None

from unilab_robot_contracts.planning import RobotModel, RailAxis, PlanRequest, PlanGoal
from unilab_robot_runtime.moveit_planner import MoveItPlanner

URDF = """<robot name="state_probe">
<link name="world"/><link name="carriage"/><link name="tip"/><link name="finger"/>
<joint name="rail" type="prismatic"><parent link="world"/><child link="carriage"/><axis xyz="1 0 0"/><limit lower="0" upper="2" effort="1" velocity="1"/></joint>
<joint name="arm" type="revolute"><parent link="carriage"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
<joint name="finger" type="prismatic"><parent link="tip"/><child link="finger"/><axis xyz="0 1 0"/><limit lower="0" upper="0.1" effort="1" velocity="1"/></joint>
</robot>"""
SRDF = '<robot name="state_probe"><group name="arm_group"><joint name="arm"/></group></robot>'


@unittest.skipIf(NativeModel is None, 'native MoveItPy unavailable')
class NativeCompleteStateTest(unittest.TestCase):
    def test_actual_native_state_has_rail_and_passive_joint(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);urdf=root/'robot.urdf';srdf=root/'robot.srdf'
            urdf.write_text(URDF);srdf.write_text(SRDF)
            native=RobotState(NativeModel(str(urdf),str(srdf)))
            native.set_to_default_values();native.update()
            planner=MoveItPlanner()
            planner._monitor=SimpleNamespace(read_only=lambda: contextlib.nullcontext(SimpleNamespace(current_state=native)))
            robot=RobotModel('state-probe',URDF,SRDF,'world','tip','arm_group',('arm',),((-3.,3.),))
            request=PlanRequest(robot,(.3,),PlanGoal.from_joint((.4,)),
                rail_axis=RailAxis('rail','world',(0.,2.)),start_rail_m=.7,
                options={'passive_joint_positions':{'finger':.04}})
            with patch.object(planner,'_require_native',return_value=(None,None,RobotState)):
                state=planner._state(request,(.3,),.7)
                self.assertAlmostEqual(state.joint_positions['arm'],.3)
                self.assertAlmostEqual(state.joint_positions['rail'],.7)
                self.assertAlmostEqual(state.joint_positions['finger'],.04)
                self.assertAlmostEqual(native.joint_positions['rail'],0.)
                missing=PlanRequest(robot,(.3,),PlanGoal.from_joint((.4,)),
                    rail_axis=RailAxis('rail','world',(0.,2.)),start_rail_m=.7)
                with self.assertRaises(ValueError):planner._state(missing,(.3,),.7)


if __name__ == '__main__':
    unittest.main(verbosity=2)
