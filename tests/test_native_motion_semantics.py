from types import SimpleNamespace
import pytest
pytest.importorskip("moveit_msgs")
from unilab_robot_contracts.planning import PlanRequest,PlanGoal,PlanConstraints,OrientationConstraintSpec
from unilab_robot_runtime.moveit_move_group import MoveGroupPlanner
from test_planning_contract import _model

def test_joint_goal_path_orientation_and_validity_constraint_are_not_dropped():
    planner=MoveGroupPlanner();planner._import_native()
    spec=OrientationConstraintSpec("base","link6",(0,0,0,1),.001,.001,3.14)
    req=PlanRequest(robot=_model(),start_q=(0.,)*6,goal=PlanGoal.from_joint((.1,)*6),
        constraints=PlanConstraints(orientation=(spec,)))
    seen={}
    def call(name,value,**kw):
        seen[name]=value
        if name=="scene":return SimpleNamespace(success=True)
        if name=="get_scene":
            return SimpleNamespace(scene=seen["scene"].scene)
        if name=="validity":return SimpleNamespace(valid=False,contacts=[])
        return SimpleNamespace(motion_plan_response=SimpleNamespace(
            error_code=SimpleNamespace(val=-1),planning_time=.01))
    planner._call=call
    planner._native_segment(req,group=req.robot.planning_group,start_q=req.start_q,
        joint_goal=dict(zip(req.robot.joint_names,(.1,)*6)),pose=None,rail_start=0.,rail_goal=0.)
    path=seen["plan"].motion_plan_request.path_constraints.orientation_constraints
    assert len(path)==1 and path[0].parameterization==path[0].ROTATION_VECTOR
    assert path[0].absolute_x_axis_tolerance==.001
    assert not planner._state_validity(req,req.start_q,whole_robot=True)["valid"]
    assert len(seen["validity"].constraints.orientation_constraints)==1

def test_cartesian_primitive_never_falls_back_to_ompl():
    planner=MoveGroupPlanner()
    req=PlanRequest(robot=_model(),start_q=(0.,)*6,goal=PlanGoal.from_joint((.1,)*6),
        options={"primitive":"cartesian_linear"})
    result=planner.plan(req)
    assert result.status=="invalid"
    assert "requires_fixed_rail" in result.diagnostics["failure_category"]
