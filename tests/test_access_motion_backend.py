"""D9-1S 机械臂、夹爪、快换和负载观测的单命令执行测试。"""

from __future__ import annotations

import time
from dataclasses import replace

import pytest
from unilab_robot_contracts import (
    ActionKind,
    BackendStatus,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    EndEffectorObservation,
    GripperObservation,
    MotionSegment,
    ObservationState,
    RigidTransform,
    RobotCommand,
    ToolAttachmentObservation,
    ToolContext,
)
from unilab_robot_runtime import AccessMotionBackend


class _ArmBackend:
    """记录每个机械臂阶段并返回确定完成。"""

    endpoint_ids = frozenset({"arm:test"})

    def __init__(self, events: list[str] | None = None) -> None:
        """创建空阶段记录。"""

        self.targets: list[str] = []
        self.command_ids: list[str] = []
        self.events = events

    def status(self) -> BackendStatus:
        """返回在线空闲。"""

        return BackendStatus(True, True)

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录唯一机械臂段。"""

        self.targets.append(command.segments[0].target_ref)
        self.command_ids.append(command.command_id)
        if self.events is not None:
            self.events.append(f"arm:{command.segments[0].segment_id}")
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "arm done")

    def reconcile(self, command_id: str) -> CommandResult:
        """返回测试完成见证。"""

        return CommandResult(command_id, CommandState.SUCCEEDED, "arm done")

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """返回不确定停止；理由只保持接口一致。"""

        del reason
        return CommandResult(command_id, CommandState.EXECUTION_UNKNOWN, "stopping")

    def end_effector_observation(self) -> EndEffectorObservation:
        """返回兼容的已知末端观测。"""

        return EndEffectorObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "arm-test",
            None,
        )


class _UnknownOnSelectedSegmentArmBackend(_ArmBackend):
    """只让显式选中的 Arm 段产生派发不明。"""

    def __init__(self, events: list[str]) -> None:
        """默认全部成功，由测试在完成 pick 后开启故障。"""

        super().__init__(events)
        self.unknown_segment_id: str | None = None

    def execute(self, command: RobotCommand) -> CommandResult:
        """记录真实调用顺序，并在目标段模拟 MoveIt 结果不明。"""

        result = super().execute(command)
        if command.segments[0].segment_id == self.unknown_segment_id:
            raise DispatchUnknownError("MoveIt retreat result unknown")
        return result


class _ToolChanger:
    """持有一份与规划完全一致的快换上下文。"""

    def __init__(
        self,
        context: ToolContext,
        *,
        locked: bool = True,
        observed_offset_s: float = 0.0,
    ) -> None:
        """冻结上下文和锁紧测试状态。"""

        self.context = context
        self.locked = locked
        self.observed_offset_s = float(observed_offset_s)

    def observe(self) -> ToolAttachmentObservation:
        """返回最新工具身份与附着代次。"""

        return ToolAttachmentObservation(
            ObservationState.KNOWN,
            time.time() + self.observed_offset_s,
            1.0,
            "tool-test",
            self.context.context_id,
            self.context.attachment_generation,
            self.locked,
        )

    def change_tool(self, command_id: str, *, tool_ref: str) -> CommandResult:
        """该测试不执行换装。"""

        del tool_ref
        return CommandResult(command_id, CommandState.SUCCEEDED, "unchanged")

    @property
    def active_tool_context(self) -> ToolContext:
        """返回规划器当前使用的同一上下文。"""

        return self.context


class _Gripper:
    """模拟抓取成功或失败，并提供负载观测。"""

    def __init__(
        self,
        *,
        fail_grip: bool = False,
        observed_offset_s: float = 0.0,
    ) -> None:
        """从无负载状态启动。"""

        self.holding = False
        self.fail_grip = fail_grip
        self.observed_offset_s = float(observed_offset_s)

    def observe(self) -> GripperObservation:
        """返回最新负载状态。"""

        return GripperObservation(
            ObservationState.KNOWN,
            time.time() + self.observed_offset_s,
            1.0,
            "gripper-test",
            self.holding,
            self.holding,
        )

    def grip(self, command_id: str, *, payload_profile: str) -> CommandResult:
        """按配置成功持有负载或返回确定失败。"""

        del payload_profile
        if self.fail_grip:
            return CommandResult(command_id, CommandState.FAILED, "grip failed")
        self.holding = True
        return CommandResult(command_id, CommandState.SUCCEEDED, "gripped")

    def release(self, command_id: str) -> CommandResult:
        """释放测试负载。"""

        self.holding = False
        return CommandResult(command_id, CommandState.SUCCEEDED, "released")


class _PayloadScene:
    """记录负载碰撞体的附着代次和严格调用顺序。"""

    def __init__(self, events: list[str]) -> None:
        """从空 PlanningScene 启动。"""

        self.events = events
        self.active: dict[str, object] | None = None

    def snapshot(self) -> list[dict[str, object]]:
        """返回当前持料碰撞体副本。"""

        return [dict(self.active)] if self.active is not None else []

    def attach_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> dict[str, object]:
        """记录附着并返回确定本地位姿。"""

        self.events.append("scene:attach")
        self.active = {
            "command_id": command_id,
            "payload_instance_ref": payload_instance_ref,
            "payload_profile_ref": payload_profile_ref,
            "attachment_generation": attachment_generation,
        }
        return {
            **self.active,
            "payload_profile_digest": "c" * 64,
            "parent_link": "robot_cr7_tool0",
            "state": "attached",
            "local_pose": {
                "xyz_m": [0.0, 0.0, 0.12],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        }

    def detach_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> dict[str, object]:
        """只解除同一 profile 和代次的碰撞体。"""

        assert self.active is not None
        assert self.active["payload_instance_ref"] == payload_instance_ref
        assert self.active["payload_profile_ref"] == payload_profile_ref
        assert self.active["attachment_generation"] == attachment_generation
        self.events.append("scene:detach")
        self.active = None
        return {
            "command_id": command_id,
            "payload_instance_ref": payload_instance_ref,
            "payload_profile_ref": payload_profile_ref,
            "attachment_generation": attachment_generation,
            "parent_link": "robot_cr7_tool0",
            "state": "detached",
            "detached": True,
        }

    def finalize_detached_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> dict[str, object]:
        """记录机械臂撤离完成后撤销临时碰撞许可。"""

        self.events.append("scene:finalize-detach")
        return {
            "command_id": command_id,
            "payload_instance_ref": payload_instance_ref,
            "payload_profile_ref": payload_profile_ref,
            "payload_profile_digest": "c" * 64,
            "attachment_generation": attachment_generation,
            "parent_link": "robot_cr7_tool0",
            "state": "detached_clearance_confirmed",
            "world_body_present": True,
            "contact_allowance_revoked": True,
        }


class _DeferredPayloadScene(_PayloadScene):
    """证明运行时构造不访问硬件，首条命令才执行场景对账。"""

    def __init__(self, events: list[str]) -> None:
        super().__init__(events)
        self.snapshot_calls = 0

    def snapshot(self) -> list[dict[str, object]]:
        self.snapshot_calls += 1
        self.events.append("scene:snapshot")
        return super().snapshot()


class _WrongAnchorPayloadScene(_PayloadScene):
    """模拟 MoveIt 把负载挂到错误末端 link 的异常回读。"""

    def attach_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> dict[str, object]:
        """返回与工具上下文不一致的 parent_link。"""

        receipt = super().attach_payload(
            command_id=command_id,
            payload_instance_ref=payload_instance_ref,
            payload_profile_ref=payload_profile_ref,
            attachment_generation=attachment_generation,
        )
        receipt["parent_link"] = "wrong_tool0"
        return receipt


class _RecoverablePayloadScene(_PayloadScene):
    """模拟 detach 已物理完成、只待 UNKNOWN 结算提交的场景。"""

    def settle_unknown_payload_state(
        self,
        *,
        command_id: str,
        holding_payload: bool,
    ) -> dict[str, object] | None:
        """只在夹爪空载时提交已有 attached 记录为 detached。"""

        self.events.append("scene:settle-unknown")
        if holding_payload:
            return None
        assert self.active is not None
        active = dict(self.active)
        self.active = None
        return {
            **active,
            "command_id": command_id,
            "payload_profile_digest": "c" * 64,
            "parent_link": "robot_cr7_tool0",
            "local_pose": {
                "xyz_m": [0.0, 0.0, 0.12],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "state": "detached",
            "recovery_evidence": "moveit_scene_absent",
        }


def _command() -> RobotCommand:
    """返回最小但完整的 pick 接近运动块。"""

    return RobotCommand(
        "access-1",
        ActionKind.PICK,
        "a" * 64,
        "beaker@v1",
        "boot-1",
        1,
        (
            MotionSegment(
                "approach",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "empty"},
            ),
            MotionSegment(
                "pick",
                "end_effector.grip",
                {"phase_kind": "end_effector", "payload_state": "loaded"},
            ),
            MotionSegment(
                "observe",
                "end_effector.payload.loaded",
                {"phase_kind": "observe", "payload_state": "loaded"},
            ),
            MotionSegment(
                "retract",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
        ),
        metadata={"payload_instance_ref": "payload-a"},
    )


def _place_command() -> RobotCommand:
    """返回与抓取共享负载 profile 的完整 place 运动块。"""

    return RobotCommand(
        "access-2",
        ActionKind.PLACE,
        "a" * 64,
        "beaker@v1",
        "boot-1",
        2,
        (
            MotionSegment(
                "approach",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
            MotionSegment(
                "place",
                "end_effector.release",
                {"phase_kind": "end_effector", "payload_state": "empty"},
            ),
            MotionSegment(
                "observe",
                "end_effector.payload.empty",
                {"phase_kind": "observe", "payload_state": "empty"},
            ),
            MotionSegment(
                "retract",
                "rack.r1c1.approach",
                {"phase_kind": "arm_move", "payload_state": "empty"},
            ),
        ),
        metadata={"payload_instance_ref": "payload-a"},
    )


def _pour_command() -> RobotCommand:
    """返回保持同一负载的 ready→tilt→ready 倒液运动块。"""

    return RobotCommand(
        "access-pour",
        ActionKind.POUR,
        "a" * 64,
        "beaker@v1",
        "boot-1",
        2,
        (
            MotionSegment(
                "pour-ready",
                "s08.S081.ready",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
            MotionSegment(
                "pour-tilt",
                "s08.S081.pour-tilt",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
            MotionSegment(
                "pour-return",
                "s08.S081.ready",
                {"phase_kind": "arm_move", "payload_state": "loaded"},
            ),
        ),
        metadata={"payload_instance_ref": "payload-a"},
    )


def _backend(
    *,
    fail_grip: bool = False,
    locked: bool = True,
    tool_observed_offset_s: float = 0.0,
    gripper_observed_offset_s: float = 0.0,
) -> AccessMotionBackend:
    """装配固定工具上下文的组合后端。"""

    context = ToolContext("gripper@1", "b" * 64, RigidTransform.identity(), 1)
    return AccessMotionBackend(
        arm_backend=_ArmBackend(),
        end_effector=_Gripper(
            fail_grip=fail_grip,
            observed_offset_s=gripper_observed_offset_s,
        ),
        tool_changer=_ToolChanger(
            context,
            locked=locked,
            observed_offset_s=tool_observed_offset_s,
        ),
        expected_tool_context=context,
    )


def test_access_motion_executes_arm_gripper_observation_and_retract() -> None:
    """成功必须包含抓取后的负载观测和撤离。"""

    backend = _backend()
    result = backend.execute(_command())

    assert result.state is CommandState.SUCCEEDED
    assert backend.arm_backend.targets == [  # type: ignore[attr-defined]
        "rack.r1c1.approach",
        "rack.r1c1.approach",
    ]
    assert backend.arm_backend.command_ids == [  # type: ignore[attr-defined]
        "access-1:arm:approach",
        "access-1:arm:retract",
    ]


def test_access_motion_defers_scene_reconciliation_until_first_dispatch() -> None:
    """构造期不得碰 ROS 服务；对账必须发生在第一段 Arm 运动之前。"""

    events: list[str] = []
    scene = _DeferredPayloadScene(events)
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=scene,
        require_payload_collision=True,
    )

    assert scene.snapshot_calls == 0
    result = backend.execute(_command())

    assert result.state is CommandState.SUCCEEDED
    assert scene.snapshot_calls == 1
    assert events.index("scene:snapshot") < events.index("arm:approach")


def test_access_motion_rejects_unlocked_tool_before_first_arm_move() -> None:
    """快换未锁紧时不得产生任何机械臂物理作用。"""

    backend = _backend(locked=False)
    with pytest.raises(CommandRejectedError, match="快换工具"):
        backend.execute(_command())
    assert backend.arm_backend.targets == []  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    "backend",
    (
        _backend(tool_observed_offset_s=10.0),
        _backend(gripper_observed_offset_s=10.0),
    ),
)
def test_access_motion_rejects_future_observation_before_motion(
    backend: AccessMotionBackend,
) -> None:
    """快换或夹爪未来时间戳不得被负时延误判为新鲜观测。"""

    with pytest.raises(CommandRejectedError, match="观测|快换工具"):
        backend.validate_before_dispatch(_command())
    assert backend.arm_backend.targets == []  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    "segments",
    (
        _command().segments[1:3],
        _command().segments[:3],
        (
            _command().segments[0],
            _command().segments[2],
            _command().segments[1],
            _command().segments[3],
        ),
    ),
)
def test_access_motion_rejects_incomplete_or_reordered_block_before_motion(
    segments: tuple[MotionSegment, ...],
) -> None:
    """缺少接近/撤离或夹爪观测颠倒时必须在任何物理作用前拒绝。"""

    backend = _backend()
    with pytest.raises(CommandRejectedError, match="严格按"):
        backend.execute(replace(_command(), segments=segments))
    assert backend.arm_backend.targets == []  # type: ignore[attr-defined]


def test_access_motion_keeps_unknown_after_gripper_failure_post_move() -> None:
    """机械臂已动后夹爪失败不能降级成可自动重试的普通失败。"""

    result = _backend(fail_grip=True).execute(_command())
    assert result.state is CommandState.EXECUTION_UNKNOWN


def test_access_motion_updates_payload_scene_between_observe_and_retract() -> None:
    """pick/place 必须在末端观测后、撤离前同步同一碰撞代次。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    scene = _PayloadScene(events)
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=scene,
        require_payload_collision=True,
    )

    picked = backend.execute(_command())
    assert picked.state is CommandState.SUCCEEDED
    assert picked.output["payload_attachment"]["state"] == "attached"
    generation = picked.output["payload_attachment"]["attachment_generation"]
    assert events == ["arm:approach", "scene:attach", "arm:retract"]

    events.clear()
    placed = backend.execute(_place_command())
    assert placed.state is CommandState.SUCCEEDED
    assert placed.output["payload_attachment"] == {
        "state": "detached",
        "evidence": "observed",
        "payload_profile_ref": "beaker@v1",
        "payload_instance_ref": "payload-a",
        "attachment_generation": generation,
        "parent_link": "robot_cr7_tool0",
        "observed_at": placed.output["payload_attachment"]["observed_at"],
        "stale_after_s": placed.output["payload_attachment"]["stale_after_s"],
            "scene_receipt": {
            "command_id": "access-2",
            "payload_instance_ref": "payload-a",
            "payload_profile_ref": "beaker@v1",
            "attachment_generation": generation,
            "parent_link": "robot_cr7_tool0",
            "state": "detached",
                "detached": True,
            },
            "clearance_receipt": {
                "command_id": "access-2",
                "payload_instance_ref": "payload-a",
                "payload_profile_ref": "beaker@v1",
                "payload_profile_digest": "c" * 64,
                "attachment_generation": generation,
                "parent_link": "robot_cr7_tool0",
                "state": "detached_clearance_confirmed",
                "world_body_present": True,
                "contact_allowance_revoked": True,
            },
        }
    assert events == [
        "arm:approach",
        "scene:detach",
        "arm:retract",
        "scene:finalize-detach",
    ]
    assert (
        placed.output["payload_attachment"]["clearance_receipt"]
        ["contact_allowance_revoked"]
        is True
    )
    assert scene.snapshot() == []


def test_place_preserves_confirmed_detach_when_retract_becomes_unknown() -> None:
    """释放和场景解除已回读后，撤离 UNKNOWN 不得丢掉 detached 转移。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    arm = _UnknownOnSelectedSegmentArmBackend(events)
    scene = _PayloadScene(events)
    backend = AccessMotionBackend(
        arm_backend=arm,
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=scene,
        require_payload_collision=True,
    )
    assert backend.execute(_command()).state is CommandState.SUCCEEDED
    events.clear()
    arm.unknown_segment_id = "retract"

    result = backend.execute(_place_command())

    assert result.state is CommandState.EXECUTION_UNKNOWN
    assert result.output["payload_attachment"]["state"] == "detached"
    assert scene.snapshot() == []
    assert events == ["arm:approach", "scene:detach", "arm:retract"]


def test_unknown_settlement_projects_detach_confirmed_by_gripper_and_scene() -> None:
    """人工结算应把空夹爪与缺失 mesh 的共同见证投影为 detached。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    scene = _RecoverablePayloadScene(events)
    scene.active = {
        "payload_instance_ref": "payload-a",
        "payload_profile_ref": "beaker@v1",
        "attachment_generation": 3,
    }
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=scene,
        require_payload_collision=True,
    )

    output = backend.prepare_unknown_settlement("access-2:arm")

    assert output["payload_attachment"]["state"] == "detached"
    assert output["payload_attachment"]["payload_instance_ref"] == "payload-a"
    assert output["payload_attachment"]["attachment_generation"] == 3
    assert scene.snapshot() == []
    assert events == ["scene:settle-unknown"]


def test_pour_reuses_active_payload_without_gripper_or_scene_transition() -> None:
    """倒液只允许在同一持料实例上执行闭合 Arm 块，不得释放或重挂负载。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    scene = _PayloadScene(events)
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=scene,
        require_payload_collision=True,
    )
    backend.execute(_command())
    events.clear()

    result = backend.execute(_pour_command())

    assert result.state is CommandState.SUCCEEDED
    assert events == ["arm:pour-ready", "arm:pour-tilt", "arm:pour-return"]
    assert scene.snapshot()[0]["payload_instance_ref"] == "payload-a"
    assert "payload_attachment" not in result.output


def test_pour_rejects_without_active_payload_before_motion() -> None:
    """夹爪或 PlanningScene 未持有指定烧杯时，倒液必须在首段运动前拒绝。"""

    backend = _backend()

    with pytest.raises(CommandRejectedError, match="负载状态|持料碰撞体"):
        backend.execute(_pour_command())

    assert backend.arm_backend.targets == []  # type: ignore[attr-defined]


def test_access_motion_rejects_place_for_different_payload_instance_before_motion() -> None:
    """同 Profile 的另一物料不得解除当前持料，且拒绝必须发生在移动前。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=_PayloadScene(events),
        require_payload_collision=True,
    )
    backend.execute(_command())
    events.clear()

    with pytest.raises(CommandRejectedError, match="负载实例"):
        backend.execute(
            replace(
                _place_command(),
                metadata={"payload_instance_ref": "payload-b"},
            )
        )

    assert events == []


def test_access_motion_rejects_scene_receipt_from_wrong_tool_link() -> None:
    """碰撞体锚点与 ToolContext 不一致时不得撤离或报告成功。"""

    events: list[str] = []
    context = ToolContext(
        "gripper@1",
        "b" * 64,
        RigidTransform.identity(),
        1,
        {"parent_link": "robot_cr7_tool0"},
    )
    backend = AccessMotionBackend(
        arm_backend=_ArmBackend(events),
        end_effector=_Gripper(),
        tool_changer=_ToolChanger(context),
        expected_tool_context=context,
        payload_planning_scene=_WrongAnchorPayloadScene(events),
        require_payload_collision=True,
    )

    with pytest.raises(DispatchUnknownError, match="parent_link"):
        backend.execute(_command())
    assert events == ["arm:approach", "scene:attach"]
