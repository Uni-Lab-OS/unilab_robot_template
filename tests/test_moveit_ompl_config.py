"""OMPL 管线配置守卫（不需要 ROS/MoveIt 运行时）。

MoveIt ≤2.5.x 的 ``planning_pipeline`` 只读**单数** ``planning_plugin``；
只给复数 ``planning_plugins`` 或整个键缺失时，pluginlib 会按声明顺序取第一个
插件，本机顺序恰好是 CHOMP。CHOMP 不支持位姿目标，失败信息又像普通不可达，
所以这里把"必须显式给出单数 OMPL 插件"做成配置守卫，两个后端构造参数时都要
调用它，禁止发出会静默回落 CHOMP 的配置。
"""

from __future__ import annotations

import pytest

from unilab_robot_contracts.planning import PlanGoal, PlanRequest, RobotModel
from unilab_robot_runtime import moveit_planner
from unilab_robot_runtime.moveit_move_group import build_move_group_parameters
from unilab_robot_runtime.moveit_ompl_config import (
    OMPL_PLANNING_PLUGIN,
    OMPLConfigurationError,
    require_ompl_planning_plugin,
)

URDF = """<?xml version="1.0"?><robot name="stub">
  <link name="world"/><link name="base_link"/><link name="link_1"/>
  <joint name="world_to_base" type="fixed"><origin xyz="0 0 0.1" rpy="0 0 0"/>
    <parent link="world"/><child link="base_link"/></joint>
  <joint name="joint_1" type="revolute"><origin xyz="0 0 0"/><parent link="base_link"/>
    <child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="3.14"/></joint>
</robot>"""

SRDF = """<?xml version="1.0"?><robot name="stub">
  <group name="arm"><chain base_link="base_link" tip_link="link_1"/></group>
</robot>"""

CHOMP_PLANNING_PLUGIN = "chomp_interface/CHOMPPlanner"


def robot_model() -> RobotModel:
    return RobotModel(
        model_id="stub",
        urdf_xml=URDF,
        srdf_xml=SRDF,
        base_frame="world",
        flange_link="link_1",
        planning_group="arm",
        joint_names=("joint_1",),
        joint_limits=((-3.0, 3.0),),
    )


def plan_request() -> PlanRequest:
    return PlanRequest(
        robot=robot_model(),
        start_q=(0.0,),
        goal=PlanGoal.from_joint([0.5]),
        timeout_s=1.0,
    )


def test_both_config_builders_emit_the_singular_ompl_plugin() -> None:
    """两个后端构造出的 ompl 块都必须通过守卫，且不含 CHOMP 管线。"""

    params = build_move_group_parameters(robot_model())
    assert require_ompl_planning_plugin(params["ompl"]) == OMPL_PLANNING_PLUGIN
    assert params["ompl"]["planning_plugin"] == OMPL_PLANNING_PLUGIN
    assert params["ompl"]["planning_plugins"] == [OMPL_PLANNING_PLUGIN]
    assert params["planning_pipelines"] == ["ompl"]
    assert params["default_planning_pipeline"] == "ompl"
    assert "chomp" not in params

    config = moveit_planner.MoveItPlanner()._build_config(plan_request())
    assert require_ompl_planning_plugin(config["ompl"]) == OMPL_PLANNING_PLUGIN
    assert config["ompl"]["planning_plugin"] == OMPL_PLANNING_PLUGIN
    assert config["ompl"]["planning_plugins"] == [OMPL_PLANNING_PLUGIN]


def test_plural_only_config_is_rejected_not_silently_accepted() -> None:
    """只有复数键时 MoveIt 2.5.9 会静默回落 CHOMP，必须显式报错。"""

    with pytest.raises(OMPLConfigurationError, match="planning_plugin"):
        require_ompl_planning_plugin(
            {"planning_plugins": [OMPL_PLANNING_PLUGIN]}
        )


def test_missing_plugin_is_rejected_instead_of_chomp() -> None:
    """两个键都缺省时 pluginlib 顺序决定插件，等于不可控，必须报错。"""

    with pytest.raises(OMPLConfigurationError, match="CHOMP"):
        require_ompl_planning_plugin({})


def test_chomp_singular_plugin_is_rejected() -> None:
    """显式 CHOMP 不是本层允许的几何规划器。"""

    with pytest.raises(OMPLConfigurationError, match="CHOMP|ompl_interface"):
        require_ompl_planning_plugin({"planning_plugin": CHOMP_PLANNING_PLUGIN})


def test_plural_mismatch_with_singular_is_rejected() -> None:
    """MoveIt 2.10+ 只读复数；两个键指向不同插件时不能装作 OMPL 已生效。"""

    with pytest.raises(OMPLConfigurationError, match="planning_plugins"):
        require_ompl_planning_plugin(
            {
                "planning_plugin": OMPL_PLANNING_PLUGIN,
                "planning_plugins": [CHOMP_PLANNING_PLUGIN],
            }
        )


def test_non_mapping_config_is_a_type_error() -> None:
    with pytest.raises(TypeError):
        require_ompl_planning_plugin(["ompl_interface/OMPLPlanner"])


def test_move_group_builder_runs_the_guard(monkeypatch: pytest.MonkeyPatch) -> None:
    """守卫必须真的接进构造路径，而不是只存在于测试里。"""

    import unilab_robot_runtime.moveit_move_group as move_group_module

    def explode(*args: object, **kwargs: object) -> str:
        raise OMPLConfigurationError("guard reached")

    monkeypatch.setattr(
        move_group_module, "require_ompl_planning_plugin", explode
    )
    with pytest.raises(OMPLConfigurationError, match="guard reached"):
        build_move_group_parameters(robot_model())


def test_moveitpy_builder_runs_the_guard(monkeypatch: pytest.MonkeyPatch) -> None:
    def explode(*args: object, **kwargs: object) -> str:
        raise OMPLConfigurationError("guard reached")

    monkeypatch.setattr(
        moveit_planner, "require_ompl_planning_plugin", explode
    )
    with pytest.raises(OMPLConfigurationError, match="guard reached"):
        moveit_planner.MoveItPlanner()._build_config(plan_request())

def test_simplification_flag_reaches_humble_planner_configuration():
    # Humble ignores arbitrary group keys; it enumerates all per-planner keys.
    from unilab_robot_runtime.moveit_move_group import MoveGroupPlannerConfig
    for enabled in (False, True):
        config = MoveGroupPlannerConfig(simplify_solutions=enabled)
        ompl = build_move_group_parameters(robot_model(), config)["ompl"]
        assert ompl["planner_configs"][config.planner_id]["simplify_solutions"] == int(enabled)
        assert "simplify_solutions" not in ompl["arm"]
