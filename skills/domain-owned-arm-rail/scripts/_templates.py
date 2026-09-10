"""领域机械臂 Module API 文件模板。"""

from __future__ import annotations


def adapters_init_py(robot_device: str) -> str:
    return f'''"""{robot_device} 执行后端适配器公开导出。"""

from .moveit import MoveGroupPort, MoveItBackend
from .moveit_client import MoveIt2ClientPort
from .moveit_commissioning import (
    MoveItCommissioningAdapter,
    MoveItCommissioningClientPort,
)

__all__ = [
    "MoveGroupPort",
    "MoveIt2ClientPort",
    "MoveItBackend",
    "MoveItCommissioningAdapter",
    "MoveItCommissioningClientPort",
]
'''


def adapters_moveit_py(robot_device: str) -> str:
    return f'''"""{robot_device} 六轴 MoveIt 后端；关节名与规划组来自当前 MODEL_DESCRIPTOR。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any, Protocol

from unilab_robot_contracts import (
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    ResolvedMotionTarget,
    RobotCommand,
)

from ..moveit_model import MODEL_DESCRIPTOR
from ._support import BackendObservationMixin, validate_completion_receipt

_ARM_DOF = len(MODEL_DESCRIPTOR.joint_names)


class MoveGroupPort(Protocol):
    """move_group 的无 RViz 执行端口。"""

    def execute_joint_target(
        self,
        *,
        group_name: str,
        joint_names: Sequence[str],
        target: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """规划并执行六轴目标，返回 move_group 完成见证。"""

    def execute_cartesian_target(
        self,
        *,
        group_name: str,
        frame_ref: str,
        xyz_m: Sequence[float],
        orientation_xyzw: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """规划并执行绝对 TCP 目标，返回 move_group 完成见证。"""

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """查询 move_group 执行见证。"""

    def cancel(self, command_id: str) -> bool:
        """请求取消并返回是否已确认停止。"""


class MoveItBackend(BackendObservationMixin):
    """headless MoveIt 后端；构造和执行均不 import、启动或探测 RViz。"""

    def __init__(
        self,
        *,
        port: MoveGroupPort,
        endpoint_ids: frozenset[str],
        targets: Mapping[str, ResolvedMotionTarget],
        group_name: str | None = None,
    ) -> None:
        normalized: dict[str, ResolvedMotionTarget] = {{}}
        for name, target in targets.items():
            if name != target.target_ref:
                raise ValueError(f"MoveIt target_ref 索引漂移: {{name}}")
            if isinstance(target, ResolvedJointTarget):
                if len(target.joint_positions) != _ARM_DOF:
                    raise ValueError(f"MoveIt 关节目标必须为 {{_ARM_DOF}} 轴: {{name}}")
            elif not isinstance(target, ResolvedCartesianTarget):
                raise TypeError(f"MoveIt target 必须由 ArmTargetResolver 解析: {{name}}")
            normalized[name] = target
        self.port = port
        self.endpoint_ids = endpoint_ids
        self.targets = normalized
        self.group_name = group_name or MODEL_DESCRIPTOR.planning_group
        self._results: dict[str, CommandResult] = {{}}
        self._initialize_observation()

    @staticmethod
    def required_launch_components() -> tuple[str, ...]:
        return (
            "robot_state_publisher",
            "controller_manager",
            "joint_state_broadcaster",
            "move_group",
        )

    def execute(self, command: RobotCommand) -> CommandResult:
        missing = [
            segment.target_ref
            for segment in command.segments
            if segment.target_ref not in self.targets
        ]
        if missing:
            raise CommandRejectedError(f"MoveIt point-set 缺少 target_ref: {{missing}}")
        self._active_command_id = command.command_id
        outputs: list[Mapping[str, Any]] = []
        try:
            for segment in command.segments:
                target = self.targets[segment.target_ref]
                outputs.append(
                    validate_completion_receipt(
                        self._execute_target(
                            target,
                            command_id=command.command_id,
                            parameters=segment.parameters,
                        ),
                        command_id=command.command_id,
                        source="MoveIt",
                    )
                )
        except Exception as exc:
            raise DispatchUnknownError(f"MoveIt 执行结果不明: {{exc}}") from exc
        finally:
            self._active_command_id = None
        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "move_group 返回完成见证",
            {{"segments": outputs}},
        )
        self._results[command.command_id] = result
        return result

    def _execute_target(
        self,
        target: ResolvedMotionTarget,
        *,
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        if isinstance(target, ResolvedJointTarget):
            return self.port.execute_joint_target(
                group_name=self.group_name,
                joint_names=MODEL_DESCRIPTOR.joint_names,
                target=target.joint_positions,
                command_id=command_id,
                parameters=parameters,
            )
        return self.port.execute_cartesian_target(
            group_name=self.group_name,
            frame_ref=target.pose.frame_ref,
            xyz_m=target.pose.xyz_m,
            orientation_xyzw=target.pose.orientation_xyzw,
            command_id=command_id,
            parameters=parameters,
        )

    def reconcile(self, command_id: str) -> CommandResult:
        observed = self.port.query_command(command_id)
        if observed is None:
            return CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, "move_group 无完成见证"
            )
        state = CommandState(str(observed.get("state", CommandState.EXECUTION_UNKNOWN.value)))
        if state.terminal:
            try:
                validate_completion_receipt(
                    observed, command_id=command_id, source="MoveIt"
                )
            except ValueError as exc:
                return CommandResult(
                    command_id, CommandState.EXECUTION_UNKNOWN, str(exc), observed
                )
        return CommandResult(
            command_id, state, str(observed.get("message", "MoveIt 对账")), observed
        )

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        confirmed = self.port.cancel(command_id)
        state = CommandState.CANCELED if confirmed else CommandState.EXECUTION_UNKNOWN
        return CommandResult(command_id, state, f"MoveIt cancel: {{reason}}")
'''


def arm_module_py(robot_device: str) -> str:
    return f'''"""{robot_device} 后端生命周期深模块。"""

from __future__ import annotations

from collections.abc import Callable, Mapping

from unilab_robot_contracts import (
    CommandJournal,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DeploymentMode,
    DispatchUnknownError,
    HardwareProfile,
    PhysicalSettlementEvidence,
    RobotCommand,
    RobotExecutionBackend,
    SafetyInterlockObservation,
    settle_unknown_as_canceled,
)


class ArmModule:
    """机械臂执行模块；standalone 与 WorkCell 复用同一个实例接口。"""

    def __init__(
        self,
        *,
        backend: RobotExecutionBackend,
        profile: HardwareProfile,
        journal: CommandJournal,
        safety_observation: Callable[[], SafetyInterlockObservation],
    ) -> None:
        if not backend.endpoint_ids:
            raise ValueError("机械臂后端必须声明物理 endpoint_ids")
        if not backend.endpoint_ids.issubset(profile.endpoint_ids):
            raise ValueError("后端 endpoint_ids 未被 HardwareProfile 原子锁定")
        self.backend = backend
        self.profile = profile
        self.journal = journal
        self._safety_observation = safety_observation

    @property
    def endpoint_ids(self) -> frozenset[str]:
        return self.backend.endpoint_ids

    @property
    def has_unsettled_fence(self) -> bool:
        return bool(self.journal.fenced_command_ids())

    def fenced_command_ids(self) -> tuple[str, ...]:
        return self.journal.fenced_command_ids()

    def get_command(self, command_id: str) -> CommandResult | None:
        return self.journal.get(command_id)

    def resolve_unknown_as_canceled(
        self,
        command_id: str,
        *,
        witness_id: str,
        reason: str,
        source: str,
    ) -> CommandResult:
        return settle_unknown_as_canceled(
            self.journal,
            command_id,
            witness_id=witness_id,
            reason=reason,
            source=source,
        )

    def execute(self, command: RobotCommand) -> CommandResult:
        if command.hardware_profile_digest != self.profile.digest:
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                "HardwareProfile digest 不匹配",
            )
        if self.journal.get(command.command_id) is None:
            fenced = self.journal.fenced_command_ids()
            if fenced:
                return CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    f"机械臂存在未物理结算命令，禁止新派发: {{fenced}}",
                )
        try:
            created, existing = self.journal.accept(command)
        except CommandRejectedError as exc:
            return CommandResult(command.command_id, CommandState.REJECTED, str(exc))
        if not created:
            return existing

        rejection = self._pre_dispatch_rejection()
        if rejection:
            return self.journal.update(
                CommandResult(command.command_id, CommandState.REJECTED, rejection)
            )
        self.journal.update(
            CommandResult(command.command_id, CommandState.RUNNING, "机械臂命令已派发")
        )
        try:
            result = self.backend.execute(command)
        except DispatchUnknownError as exc:
            result = CommandResult(
                command.command_id, CommandState.EXECUTION_UNKNOWN, str(exc)
            )
        except CommandRejectedError as exc:
            result = CommandResult(command.command_id, CommandState.FAILED, str(exc))
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"后端异常且派发结果不明: {{exc}}",
            )
        if result.command_id != command.command_id:
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                "后端返回了不同 command_id",
            )
        interrupted = self.journal.get(command.command_id)
        if (
            interrupted is not None
            and interrupted.state is CommandState.EXECUTION_UNKNOWN
            and self.journal.is_fenced(command.command_id)
        ):
            return interrupted
        return self.journal.update(result)

    def validate_before_dispatch(self, command: RobotCommand) -> None:
        if command.hardware_profile_digest != self.profile.digest:
            raise CommandRejectedError("HardwareProfile digest 不匹配")
        if self.has_unsettled_fence:
            raise CommandRejectedError("机械臂存在未物理结算 Fence")
        validator = getattr(self.backend, "validate_before_dispatch", None)
        if callable(validator):
            validator(command)

    def request_controlled_stop(self, command_id: str, reason: str) -> CommandResult:
        existing = self.journal.get(command_id)
        if existing is not None and not existing.state.terminal:
            self.journal.update(
                CommandResult(
                    command_id,
                    CommandState.EXECUTION_UNKNOWN,
                    f"已请求机械臂受控停止，等待物理结算: {{reason}}",
                ),
                fenced=True,
            )
        try:
            result = self.backend.request_stop(command_id, reason)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"机械臂受控停止请求失败: {{exc}}",
            )
        if result.command_id != command_id:
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "机械臂停止结果 command_id 不匹配",
            )
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        existing = self.journal.get(command_id)
        if existing is None:
            raise KeyError(f"命令不存在: {{command_id}}")
        if existing.state.terminal and not self.journal.is_fenced(command_id):
            return existing
        try:
            result = self.backend.reconcile(command_id)
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, f"对账失败: {{exc}}"
            )
        if (
            existing.state is CommandState.EXECUTION_UNKNOWN
            and result.state.terminal
        ):
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "已观测到候选终态，必须提供精确物理结算见证才能解除阻断",
                {{"candidate_state": result.state.value, "candidate": dict(result.output)}},
            )
        return self.journal.update(result)

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
    ) -> CommandResult:
        preparer = getattr(self.backend, "prepare_unknown_settlement", None)
        if callable(preparer):
            prepared = preparer(result.command_id)
            if not isinstance(prepared, Mapping):
                raise CommandRejectedError("后端 UNKNOWN 结算输出必须是对象")
            conflicts = {{
                key
                for key in prepared
                if key in result.output and result.output[key] != prepared[key]
            }}
            if conflicts:
                raise CommandRejectedError(
                    f"后端 UNKNOWN 结算输出与控制面冲突: {{sorted(conflicts)}}"
                )
            result = CommandResult(
                result.command_id,
                result.state,
                result.message,
                {{**dict(result.output), **dict(prepared)}},
            )
        return self.journal.settle(result, evidence)

    def _pre_dispatch_rejection(self) -> str | None:
        status = self.backend.status()
        if not status.online:
            return "机械臂后端不在线"
        if not status.idle:
            return "机械臂后端非空闲"
        safety = self._safety_observation()
        if not safety.is_fresh():
            return "硬件安全许可未知或过期"
        if not safety.granted or not safety.arm_motion_permitted:
            return "硬件安全链未许可机械臂运动"
        if not safety.concurrent_motion_blocked:
            return "硬件安全链不能证明导轨与机械臂互斥"
        if (
            self.profile.mode is DeploymentMode.PRODUCTION
            and not safety.hardware_enforced
        ):
            return "production profile 缺少经验证硬件互锁"
        return None
'''


def robot_factory_py(robot_device: str, module_version: str) -> str:
    return f'''"""{robot_device} 的 Robot Module API v1 工厂。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

from unilab_robot_contracts import (
    CommandJournal,
    HardwareProfile,
    ResolvedMotionTarget,
    RobotExecutionBackend,
    SafetyInterlockObservation,
)

from .adapters import MoveIt2ClientPort, MoveItBackend, MoveItCommissioningAdapter
from .arm_module import ArmModule
from .moveit_model import (
    MODEL_DESCRIPTOR,
    build_joint_state_name_map,
    build_moveit_model,
)

MODULE_API_VERSION = 1
MODULE_KIND = "arm"
MODULE_VERSION = "{module_version}"


def create_arm_module(
    *,
    backend: RobotExecutionBackend,
    profile: HardwareProfile,
    journal: CommandJournal,
    safety_observation: Callable[[], SafetyInterlockObservation],
) -> ArmModule:
    return ArmModule(
        backend=backend,
        profile=profile,
        journal=journal,
        safety_observation=safety_observation,
    )


def create_moveit_backend(
    *,
    moveit_client: Any,
    endpoint_ids: frozenset[str],
    targets: Mapping[str, ResolvedMotionTarget],
    qualified_joint_names: Sequence[str],
    collision_asset_resolver: Any = None,
) -> MoveItBackend:
    return MoveItBackend(
        port=MoveIt2ClientPort(
            moveit_client,
            qualified_joint_names=qualified_joint_names,
            collision_asset_resolver=collision_asset_resolver,
        ),
        endpoint_ids=endpoint_ids,
        targets=targets,
        group_name=MODEL_DESCRIPTOR.planning_group,
    )


def create_moveit_commissioning_adapter(
    *,
    moveit_client: Any,
    targets: Mapping[str, ResolvedMotionTarget],
    qualified_joint_names: Sequence[str],
    point_set_revision: str,
    hardware_profile_digest: str,
    tool_context_digest: str,
    commissioning_velocity_limit: float,
    commissioning_acceleration_limit: float,
    joint_completion_tolerance_si: float = 0.002,
    collision_asset_resolver: Any = None,
) -> MoveItCommissioningAdapter:
    """创建与生产后端共享 MoveIt2 客户端的统一维护调试 Adapter。"""

    return MoveItCommissioningAdapter(
        port=MoveIt2ClientPort(
            moveit_client,
            qualified_joint_names=qualified_joint_names,
            collision_asset_resolver=collision_asset_resolver,
        ),
        model=MODEL_DESCRIPTOR,
        targets=targets,
        point_set_revision=point_set_revision,
        hardware_profile_digest=hardware_profile_digest,
        tool_context_digest=tool_context_digest,
        commissioning_velocity_limit=commissioning_velocity_limit,
        commissioning_acceleration_limit=commissioning_acceleration_limit,
        joint_completion_tolerance_si=joint_completion_tolerance_si,
    )


__all__ = [
    "MODULE_API_VERSION",
    "MODULE_KIND",
    "MODULE_VERSION",
    "MODEL_DESCRIPTOR",
    "build_joint_state_name_map",
    "build_moveit_model",
    "create_arm_module",
    "create_moveit_backend",
    "create_moveit_commissioning_adapter",
]
'''


def robot_module_py(robot_device: str) -> str:
    return f'''"""{robot_device} Robot Module API v1 入口。"""

from .arm_module import ArmModule
from .moveit_model import MoveItModelBundle, build_joint_state_name_map, build_moveit_model
from .robot_factory import (
    MODEL_DESCRIPTOR,
    MODULE_API_VERSION,
    MODULE_KIND,
    MODULE_VERSION,
    create_arm_module,
    create_moveit_backend,
    create_moveit_commissioning_adapter,
)

__version__ = MODULE_VERSION

__all__ = [
    "ArmModule",
    "MODEL_DESCRIPTOR",
    "MODULE_API_VERSION",
    "MODULE_KIND",
    "MODULE_VERSION",
    "MoveItModelBundle",
    "build_joint_state_name_map",
    "build_moveit_model",
    "create_arm_module",
    "create_moveit_backend",
    "create_moveit_commissioning_adapter",
]
'''


def test_robot_module_py(domain_pkg: str, robot_device: str, module_version: str) -> str:
    module_path = f"{domain_pkg}.devices.{robot_device}.robot_module"
    return f'''"""Robot Module API v1 门禁。"""

from __future__ import annotations

import importlib


def test_robot_module_exports_module_api_v1() -> None:
    module = importlib.import_module("{module_path}")
    assert module.MODULE_API_VERSION == 1
    assert module.MODULE_KIND == "arm"
    assert module.__version__ == "{module_version}"
    assert callable(module.create_arm_module)
    assert callable(module.create_moveit_backend)
    assert callable(module.build_moveit_model)
    assert module.MODEL_DESCRIPTOR.planning_group
'''
