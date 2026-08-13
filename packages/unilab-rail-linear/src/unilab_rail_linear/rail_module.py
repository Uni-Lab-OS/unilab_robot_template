"""导轨型号与传输解耦的单轴模块。"""

from __future__ import annotations

from typing import Protocol

from unilab_robot_contracts import (
    DispatchUnknownError,
    ObservationState,
    RailStateObservation,
)


class RailAxisPort(Protocol):
    """PLC、厂家 SDK 和仿真都必须实现的单轴端口。"""

    endpoint_ids: frozenset[str]

    def move(self, command_id: str, target_ref: str) -> None:
        """开始移动到部署 target-set 中的目标。"""

    def observe(self) -> RailStateObservation:
        """读取位置、零速和稳定见证。"""

    def request_stop(self, command_id: str) -> bool:
        """请求普通停止并返回是否已确认。"""


class RailModule:
    """在一个接口内封装导轨移动、稳定见证和 UNKNOWN 语义。"""

    def __init__(self, *, port: RailAxisPort, allowed_targets: frozenset[str]) -> None:
        """注入轴端口与部署允许目标集合。"""

        if not port.endpoint_ids or not allowed_targets:
            raise ValueError("RailModule 必须声明 endpoint_ids 与 allowed_targets")
        self.port = port
        self.allowed_targets = allowed_targets

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回导轨模块占用的物理端点。"""

        return self.port.endpoint_ids

    def move_and_settle(self, command_id: str, target_ref: str) -> RailStateObservation:
        """移动导轨并要求已知、最新、零速且到达同一目标。

        参数：命令身份和部署 target ref。返回：稳定观测。异常：派发前错误为 ``ValueError``，派发后不确定为 ``DispatchUnknownError``。
        """

        if target_ref not in self.allowed_targets:
            raise ValueError(f"导轨 target-set 不包含: {target_ref}")
        try:
            self.port.move(command_id, target_ref)
            observed = self.port.observe()
        except Exception as exc:
            raise DispatchUnknownError(f"导轨派发后观测失败: {exc}") from exc
        if (
            observed.state is not ObservationState.KNOWN
            or not observed.is_fresh()
            or observed.moving is not False
            or observed.settled is not True
            or observed.target_ref != target_ref
            or observed.completed_command_id != command_id
        ):
            raise DispatchUnknownError(
                "导轨无法证明本次命令已到目标且零速稳定"
            )
        return observed

    def observe(self) -> RailStateObservation:
        """透出规范 RailStateObservation，不解析业务 Site。"""

        return self.port.observe()

    def request_controlled_stop(self, command_id: str, reason: str) -> bool:
        """向单轴端口请求普通停止；确认值只作诊断。

        参数：组合命令身份与诊断原因。返回：端口是否确认收到或完成停止请求。
        注意：该布尔值不是物理结算见证，不能据此恢复派发。
        """

        del reason
        return self.port.request_stop(command_id)
