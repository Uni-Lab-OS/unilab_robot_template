"""性能改动保留模型变更、错误传播与最小状态回读的约束。"""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from unilab_robot_runtime.planning_timing import PlanningTiming
from unilab_robot_runtime.moveit_move_group import MoveGroupPlanner
from unilab_robot_runtime.orientation_validation import check_orientation_constraints, _compiled_chain
from unilab_robot_contracts.planning import OrientationConstraintSpec
from test_move_group_planner_contract import robot_model, request


def test_timing_counts_failed_calls_without_swallowing_errors():
    timing = PlanningTiming()
    before = timing.snapshot()
    with pytest.raises(ValueError):
        with timing.measure("rpc.validity"):
            raise ValueError("native failure")
    row = timing.since(before)["rpc.validity"]
    assert row["calls"] == row["failures"] == 1
    assert row["wall_s"] >= 0
    before["unused"] = {}
    assert "unused" not in timing.snapshot()


def test_minimal_robot_readback_excludes_attachment_geometry_and_acm():
    planner = MoveGroupPlanner()
    planner._import_native()
    observed = []
    planner._call = lambda name, query: (observed.append(query.components.components) or SimpleNamespace(scene="read"))
    assert planner._read_scene(state_only=True) == "read"
    flags = planner._session.messages["PlanningSceneComponents"]
    assert observed[-1] == (flags.ROBOT_STATE | flags.SCENE_SETTINGS)
    planner._read_scene()
    assert observed[-1] & flags.ROBOT_STATE_ATTACHED_OBJECTS
    assert observed[-1] & flags.ALLOWED_COLLISION_MATRIX


def test_cached_orientation_recomputes_q_and_invalidates_changed_urdf():
    _compiled_chain.cache_clear()
    robot = robot_model()
    spec = OrientationConstraintSpec("world", "link_1", (0,0,0,1), .01,.01,.01)
    assert check_orientation_constraints(robot,(0,),{},(spec,))[0]["satisfied"]
    assert not check_orientation_constraints(robot,(.2,),{},(spec,))[0]["satisfied"]
    assert _compiled_chain.cache_info().misses == 1
    changed = replace(robot,urdf_xml=robot.urdf_xml.replace('rpy="0 0 0"', 'rpy="0.2 0 0"'))
    assert not check_orientation_constraints(changed,(0,),{},(spec,))[0]["satisfied"]
    assert _compiled_chain.cache_info().misses == 2


def test_failed_plan_keeps_performance_and_integrity_digest():
    planner = MoveGroupPlanner()
    planner._import_native = lambda: (_ for _ in ()).throw(RuntimeError("unavailable"))
    result = planner.plan(request())
    assert result.status == "invalid"
    assert result.diagnostics["performance"]["pure_fcl_s"] is None
    assert result.diagnostics["performance"]["wall_s"] >= 0
    assert replace(result,digest="").digest == result.digest
