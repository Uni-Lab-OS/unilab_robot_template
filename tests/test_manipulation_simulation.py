"""机械臂、夹爪和快换工具的无 RViz 组合仿真测试。"""

from __future__ import annotations

from test_moveit_commissioning import FakeCommissioningMoveGroup
from unilab_arm_cr7 import MODEL_DESCRIPTOR
from unilab_arm_cr7.adapters import MoveItCommissioningAdapter
from unilab_end_effector_sim import SimulatedGripper, SimulatedToolChanger
from unilab_robot_contracts import ResolvedJointTarget, RigidTransform, ToolDefinition
from unilab_robot_runtime import ManipulationSequence, ManipulationSequenceRunner


def test_pick_sequence_changes_tool_then_moves_and_grips() -> None:
    """快换确认和 PlanningScene 更新必须先于 ready→抓取→回撤序列。"""

    move_group = FakeCommissioningMoveGroup()
    tool_changer = SimulatedToolChanger(
        tools={
            "parallel-gripper": ToolDefinition(
                tool_ref="parallel-gripper",
                model_digest="b" * 64,
                mount_to_tcp=RigidTransform((0.0, 0.0, 0.12), (0.0, 0.0, 0.0, 1.0)),
                collision_asset_ref="package://gripper/model.stl",
            )
        }
    )
    gripper = SimulatedGripper(tool_changer=tool_changer)
    targets = {
        name: ResolvedJointTarget(name, values, (name,))
        for name, values in {
            "pick.ready": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "pick.approach": (0.1, -0.2, 0.2, 0.0, 0.1, 0.0),
            "pick.interaction": (0.1, -0.3, 0.3, 0.0, 0.1, 0.0),
            "pick.retract": (0.1, -0.2, 0.2, 0.0, 0.1, 0.0),
        }.items()
    }
    adapter = MoveItCommissioningAdapter(
        port=move_group,
        model=MODEL_DESCRIPTOR,
        targets=targets,
        point_set_revision="pick-demo@1.0.0",
        hardware_profile_digest="profile-digest",
        tool_context_digest="detached-tool-context",
    )
    runner = ManipulationSequenceRunner(
        commissioning=adapter,
        end_effector=gripper,
        tool_changer=tool_changer,
        tool_context_activator=adapter,
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
    )

    result = runner.pick(
        ManipulationSequence(
            sequence_id="pick-beaker-1",
            point_set_revision="pick-demo@1.0.0",
            ready_target_ref="pick.ready",
            approach_target_ref="pick.approach",
            interaction_target_ref="pick.interaction",
            retract_target_ref="pick.retract",
            tool_ref="parallel-gripper",
            payload_profile="beaker-100ml",
        )
    )

    assert result.success
    assert gripper.observe().holding_payload is True
    assert tool_changer.observe().tool_ref == "parallel-gripper"
    assert adapter.tool_context_digest == tool_changer.active_tool_context.digest
    assert len(result.output["steps"]) == 7


def test_simulated_gripper_restores_confirmed_attachment_state() -> None:
    """仿真重启时可恢复已由 PlanningScene 对账确认的持料状态。"""

    tool_changer = SimulatedToolChanger(
        tools={
            "parallel-gripper": ToolDefinition(
                tool_ref="parallel-gripper",
                model_digest="b" * 64,
                mount_to_tcp=RigidTransform(
                    (0.0, 0.0, 0.12),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                collision_asset_ref="package://gripper/model.stl",
            )
        }
    )
    tool_changer.change_tool("activate", tool_ref="parallel-gripper")

    gripper = SimulatedGripper(
        tool_changer=tool_changer,
        initial_holding_payload=True,
    )

    assert gripper.observe().closed is True
    assert gripper.observe().holding_payload is True
