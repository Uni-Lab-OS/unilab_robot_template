"""CR5 distribution 对部署组合层暴露的稳定工厂。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from unilab_robot_contracts import (
    CartesianPose,
    CommandJournal,
    HardwareProfile,
    JointSpecification,
    PLCAdapterProfile,
    PLCProgramSet,
    ResolvedMotionTarget,
    RobotExecutionBackend,
    SafetyInterlockObservation,
    ToolContext,
)

from .adapters import (
    MoveIt2ClientPort,
    MoveItBackend,
    MoveItCommissioningAdapter,
    PLCBackend,
    PLCProgramBinding,
    TcpSdkBackend,
)
from .arm_module import ArmModule
from .kinematics import forward_kinematics, load_joint_specifications

MODULE_API_VERSION = 1
MODULE_KIND = "arm"
MODULE_VERSION = "0.1.0"


@dataclass(frozen=True)
class ArmModelDescriptor:
    """组合、点位解析和 MoveIt 启动所需的型号固有事实。"""

    model_ref: str
    joint_specs: tuple[JointSpecification, ...]
    base_frame: str
    base_link: str
    tip_link: str
    planning_group: str

    @property
    def joint_names(self) -> tuple[str, ...]:
        """返回型号拥有的规范关节顺序。"""

        return tuple(specification.name for specification in self.joint_specs)

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        tool_context: ToolContext,
    ) -> CartesianPose:
        """用精确 CR5 模型和活动 TCP 解析绝对位姿。"""

        return forward_kinematics(joint_positions, tool_context)


MODEL_DESCRIPTOR = ArmModelDescriptor(
    model_ref="package://unilab_arm_cr5/models/model.yaml",
    joint_specs=load_joint_specifications(),
    base_frame="arm_base",
    base_link="device_link",
    tip_link="cr5_link_6",
    planning_group="cr5_arm",
)


def create_arm_module(
    *,
    backend: RobotExecutionBackend,
    profile: HardwareProfile,
    journal: CommandJournal,
    safety_observation: Callable[[], SafetyInterlockObservation],
) -> ArmModule:
    """创建可供 standalone 或 WorkCell 复用的 CR5 模块。"""

    return ArmModule(
        backend=backend,
        profile=profile,
        journal=journal,
        safety_observation=safety_observation,
    )


def create_plc_backend(
    *,
    port: Any,
    endpoint_ids: frozenset[str],
    program_data: Mapping[str, Any],
    adapter_data: Mapping[str, Any],
) -> PLCBackend:
    """把领域部署资产组合成 CR5 PLC Backend。"""

    common = adapter_data["arm"]
    program_set = PLCProgramSet.from_mapping(program_data)
    adapter_profile = PLCAdapterProfile.from_mapping(
        adapter_data,
        program_set=program_set,
    )
    programs = {
        str(target_ref): PLCProgramBinding(
            program_number=adapter_profile.programs[str(target_ref)].selector,
            command_variable=str(common["command_variable"]),
            write_done_variable=str(common["write_done_variable"]),
            completion_variable=str(common["completion_variable"]),
            home_variable=str(common["home_variable"]),
            write_allowed_variable=str(common["write_allowed_variable"]),
            parameter_variables=adapter_profile.programs[
                str(target_ref)
            ].parameter_variables,
            timeout_s=float(common.get("timeout_s", 300.0)),
        )
        for target_ref in program_set.programs
    }
    return PLCBackend(
        port=port,
        endpoint_ids=endpoint_ids,
        programs=programs,
    )


def create_moveit_backend(
    *,
    moveit_client: Any,
    endpoint_ids: frozenset[str],
    targets: Mapping[str, ResolvedMotionTarget],
    qualified_joint_names: Sequence[str],
) -> MoveItBackend:
    """创建不启动 RViz 的 CR5 MoveIt Backend。"""

    return MoveItBackend(
        port=MoveIt2ClientPort(
            moveit_client, qualified_joint_names=qualified_joint_names
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
) -> MoveItCommissioningAdapter:
    """创建与生产后端共享 MoveIt2 客户端的统一维护调试 Adapter。"""

    return MoveItCommissioningAdapter(
        port=MoveIt2ClientPort(
            moveit_client,
            qualified_joint_names=qualified_joint_names,
        ),
        model=MODEL_DESCRIPTOR,
        targets=targets,
        point_set_revision=point_set_revision,
        hardware_profile_digest=hardware_profile_digest,
        tool_context_digest=tool_context_digest,
        commissioning_velocity_limit=commissioning_velocity_limit,
        commissioning_acceleration_limit=commissioning_acceleration_limit,
    )


def create_tcp_sdk_backend(
    *,
    port: Any,
    endpoint_ids: frozenset[str],
    targets: Mapping[str, ResolvedMotionTarget],
) -> TcpSdkBackend:
    """创建受限的 CR5 TCP/SDK 执行后端。

    参数：实现 ``RobotSDKPort`` 的厂家客户端和唯一物理端点。
    返回：只接受已解析目标引用的 ``TcpSdkBackend``。
    异常：端口通信和完成见证错误由后端在执行时按 UNKNOWN 语义处理。
    """

    return TcpSdkBackend(port=port, endpoint_ids=endpoint_ids, targets=targets)
