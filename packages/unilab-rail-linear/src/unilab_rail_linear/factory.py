"""单轴导轨 distribution 对部署组合层暴露的稳定工厂。"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from unilab_robot_contracts import JointStateNameMap

from .adapters import PLCRailAxisPort, PLCRailBinding, SimulationRailAxisPort
from .rail_module import RailModule

MODULE_API_VERSION = 1
MODULE_KIND = "rail"
MODULE_VERSION = "0.1.0"


@dataclass(frozen=True)
class RailModelDescriptor:
    """组合模型所需的型号固有轴和挂载点名称。"""

    model_ref: str
    axis_joint: str
    base_link: str
    carriage_link: str
    travel_m: tuple[float, float]
    velocity_limit_m_s: float


MODEL_DESCRIPTOR = RailModelDescriptor(
    model_ref="package://unilab_rail_linear/models/model.yaml",
    axis_joint="rail_joint",
    base_link="rail_base",
    carriage_link="rail_carriage",
    travel_m=(0.0, 2.25),
    velocity_limit_m_s=0.25,
)


def build_joint_state_name_map(*, device_id: str) -> JointStateNameMap:
    """按 Graph Device id 构造与机械臂相同规则的完全限定关节名。"""

    axis = MODEL_DESCRIPTOR.axis_joint
    return JointStateNameMap(
        device_id=str(device_id).strip(),
        canonical_joint_names=(axis,),
        raw_to_canonical={axis: axis},
    )


def create_plc_module(
    *,
    port: Any,
    endpoint_ids: frozenset[str],
    target_data: Mapping[str, Any],
    adapter_data: Mapping[str, Any],
) -> RailModule:
    """把部署目标集和原始节点绑定组合成导轨模块。"""

    rail_data = adapter_data["rail"]
    target_values = _validated_target_values(target_data)
    binding = PLCRailBinding(
        target_variable=str(rail_data["target_variable"]),
        command_id_variable=str(rail_data["command_id_variable"]),
        accepted_command_id_variable=str(rail_data["accepted_command_id_variable"]),
        completed_command_id_variable=str(
            rail_data["completed_command_id_variable"]
        ),
        start_variable=str(rail_data["start_variable"]),
        moving_variable=str(rail_data["moving_variable"]),
        settled_variable=str(rail_data["settled_variable"]),
        position_variable=str(rail_data["position_variable"]),
        target_values=target_values,
        timeout_s=float(rail_data.get("timeout_s", 60.0)),
        tolerance=float(rail_data.get("tolerance", 0.001)),
    )
    return RailModule(
        port=PLCRailAxisPort(
            port=port,
            binding=binding,
            endpoint_ids=endpoint_ids,
        ),
        allowed_targets=frozenset(target_values),
    )


def create_simulation_module(
    *,
    endpoint_ids: frozenset[str],
    target_data: Mapping[str, Any],
    on_settled: Callable[[str], None],
) -> RailModule:
    """创建不访问 PLC 节点的确定性单轴仿真模块。

    参数：唯一仿真端点、部署目标集与到位通知。返回：标准 RailModule。
    异常：端点数量不为一或目标超出型号行程时拒绝装配。
    """

    if len(endpoint_ids) != 1:
        raise ValueError("仿真 RailModule 必须声明且只声明一个端点")
    target_values = _validated_target_values(target_data)
    return RailModule(
        port=SimulationRailAxisPort(
            endpoint_id=next(iter(endpoint_ids)),
            on_settled=on_settled,
            target_values=target_values,
        ),
        allowed_targets=frozenset(target_values),
    )


def create_module_from_port(
    *,
    port: Any,
    target_data: Mapping[str, Any],
) -> RailModule:
    """把领域组合根注入的标准 RailAxisPort 装配为通用单轴模块。"""

    target_values = _validated_target_values(target_data)
    return RailModule(port=port, allowed_targets=frozenset(target_values))


def _validated_target_values(target_data: Mapping[str, Any]) -> dict[str, float]:
    """解析导轨目标并验证全部数值位于型号固有行程内。"""

    target_values = {
        str(name): float(value) for name, value in target_data["targets"].items()
    }
    lower, upper = MODEL_DESCRIPTOR.travel_m
    outside = {
        name: value
        for name, value in target_values.items()
        if not lower <= value <= upper
    }
    if outside:
        raise ValueError(f"导轨目标超出型号行程 {MODEL_DESCRIPTOR.travel_m}: {outside}")
    return target_values
