"""仅暴露珞石CR12型号事实；不提供越疆PLC/TCP执行工厂。"""
from __future__ import annotations
from collections.abc import Sequence
from dataclasses import dataclass
from unilab_robot_contracts import CartesianPose, JointSpecification, ToolContext
from .kinematics import forward_kinematics, load_joint_specifications

MODULE_API_VERSION = 1
MODULE_KIND = "arm-model"
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
        """用精确 珞石CR12 模型和活动 TCP 解析绝对位姿。"""

        return forward_kinematics(joint_positions, tool_context)

MODEL_DESCRIPTOR = ArmModelDescriptor(
    model_ref="package://unilab_arm_rokae_cr12/models/model.yaml",
    joint_specs=load_joint_specifications(),
    base_frame="arm_base",
    base_link="device_link",
    tip_link="rokae_cr12_link_6",
    planning_group="rokae_cr12_arm",
)
