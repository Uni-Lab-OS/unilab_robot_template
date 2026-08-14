"""公共 Device 依赖的唯一机械臂运行时表面。"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import RLock
from typing import Any

from unilab_rail_mounted_arm import EndpointLease, EndpointLeaseRegistry
from unilab_robot_contracts import (
    CommandResult,
    CommissioningCommand,
    CommissioningSnapshot,
    DeploymentMode,
    PhysicalSettlementEvidence,
    RobotCommand,
    RobotCommissioningPort,
)

_ENDPOINTS = EndpointLeaseRegistry()


@dataclass
class RuntimeBinding:
    """隐藏 standalone/WorkCell 差异并持有唯一物理端点租约。"""

    runtime: Any
    lease: EndpointLease
    rail_mounted: bool
    commissioning_port: RobotCommissioningPort | None = None
    deployment_mode: DeploymentMode | None = None
    _lock: RLock = field(default_factory=RLock, init=False, repr=False)
    _closed: bool = field(default=False, init=False, repr=False)
    _maintenance_owner: str | None = field(default=None, init=False, repr=False)

    def execute(
        self,
        command: RobotCommand,
        rail_target_ref: str | None = None,
    ) -> CommandResult:
        """执行一个已解析机器人指令（RobotCommand）。

        参数：指令与 WorkCell 可选导轨目标。返回：权威执行结果。
        异常：运行时关闭、WorkCell 缺少导轨目标或底层执行失败时向上抛出。
        """

        with self._lock:
            if self._closed:
                raise RuntimeError("Robot runtime 已关闭")
            if self._maintenance_owner is not None:
                raise RuntimeError("维护会话占用物理端点，拒绝生产动作")
            if self.rail_mounted:
                if not rail_target_ref:
                    raise ValueError("RailMountedArm 命令必须解析出 rail_target_ref")
                return self.runtime.execute(command, rail_target_ref=rail_target_ref)
            return self.runtime.execute(command)

    def request_controlled_stop(
        self,
        command_id: str,
        reason: str,
    ) -> CommandResult:
        """请求普通受控停止，不把请求成功等同于物理结算。

        参数：命令标识和停止原因。返回：底层停止结果。
        异常：运行时关闭或不支持停止时抛出 ``RuntimeError``。
        """

        if self._closed:
            raise RuntimeError("Robot runtime 已关闭")
        stopper = getattr(self.runtime, "request_controlled_stop", None)
        if stopper is None:
            raise RuntimeError("Robot runtime 不支持受控停止请求")
        if self.rail_mounted:
            return stopper(command_id, reason=reason)
        return stopper(command_id, reason)

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
        *,
        arm_result: CommandResult | None = None,
        arm_evidence: PhysicalSettlementEvidence | None = None,
    ) -> CommandResult:
        """用精确物理见证结算 execution_unknown，且绝不重放运动。

        参数：公共结果/见证，以及 WorkCell 可选的机械臂私有结果/见证。
        返回：结算后的权威结果。异常：见证不匹配或运行时关闭时向上抛出。
        """

        with self._lock:
            if self._closed:
                raise RuntimeError("Robot runtime 已关闭")
            if self.rail_mounted:
                return self.runtime.settle_unknown(
                    result,
                    evidence,
                    arm_result=arm_result,
                    arm_evidence=arm_evidence,
                )
            if arm_result is not None or arm_evidence is not None:
                raise ValueError("standalone Arm 结算不得传入 WorkCell 私有见证")
            return self.runtime.settle_unknown(result, evidence)

    def close(self) -> None:
        """仅在没有未物理结算 Fence 时释放物理端点租约。"""

        with self._lock:
            if self._closed:
                return
            if self._maintenance_owner is not None:
                raise RuntimeError("维护会话尚未关闭，拒绝释放物理端点")
            if bool(getattr(self.runtime, "has_unsettled_fence", False)):
                raise RuntimeError("存在未物理结算命令，拒绝释放物理端点")
            self.lease.close()
            self._closed = True

    def open_maintenance_session(self, owner_id: str) -> MaintenanceSession:
        """取得当前绑定内的独占维护会话。"""

        normalized_owner = str(owner_id).strip()
        if not normalized_owner:
            raise ValueError("维护会话 owner_id 不能为空")
        with self._lock:
            if self._closed:
                raise RuntimeError("Robot runtime 已关闭")
            if self.commissioning_port is None:
                raise RuntimeError("当前运行时未提供机械臂调试端口")
            if self.deployment_mode not in {
                DeploymentMode.MAINTENANCE,
                DeploymentMode.SIMULATION,
            }:
                raise RuntimeError("production profile 禁止打开维护调试会话")
            if self._maintenance_owner is not None:
                raise RuntimeError("维护会话已被占用")
            if bool(getattr(self.runtime, "has_unsettled_fence", False)):
                raise RuntimeError("存在未物理结算 Fence，禁止打开维护会话")
            prepare = getattr(self.commissioning_port, "prepare_commissioning", None)
            if callable(prepare):
                prepare()
            snapshot = self.commissioning_port.commissioning_snapshot()
            if (
                not snapshot.is_fresh()
                or snapshot.online is not True
                or snapshot.idle is not True
                or snapshot.execution_fenced
            ):
                raise RuntimeError("调试快照未知、过期、离线、忙碌或存在 Fence")
            self._maintenance_owner = normalized_owner
            return MaintenanceSession(self, normalized_owner)

    def _commissioning_snapshot(self, owner_id: str) -> CommissioningSnapshot:
        """为当前维护所有者读取快照。"""

        with self._lock:
            self._require_maintenance_owner(owner_id)
            assert self.commissioning_port is not None
            return self.commissioning_port.commissioning_snapshot()

    def _execute_commissioning(
        self,
        owner_id: str,
        command: CommissioningCommand,
    ) -> CommandResult:
        """为当前维护所有者串行派发封闭调试命令。"""

        with self._lock:
            self._require_maintenance_owner(owner_id)
            assert self.commissioning_port is not None
            return self.commissioning_port.execute_commissioning(command)

    def _close_maintenance(self, owner_id: str) -> None:
        """仅由持有者释放维护会话。"""

        with self._lock:
            self._require_maintenance_owner(owner_id)
            snapshot = self.commissioning_port.commissioning_snapshot()  # type: ignore[union-attr]
            if snapshot.execution_fenced:
                raise RuntimeError("调试命令结果不明，保留维护会话与 Fence")
            self._maintenance_owner = None

    def _require_maintenance_owner(self, owner_id: str) -> None:
        """校验运行时和维护会话所有权。"""

        if self._closed:
            raise RuntimeError("Robot runtime 已关闭")
        if self._maintenance_owner != owner_id:
            raise RuntimeError("调用方不拥有当前维护会话")


@dataclass
class MaintenanceSession:
    """维护人员持有的最小独占调试表面。"""

    _binding: RuntimeBinding
    owner_id: str
    _closed: bool = field(default=False, init=False, repr=False)

    @property
    def capabilities(self) -> Any:
        """返回 Adapter 真实声明的调试能力。"""

        if self._closed:
            raise RuntimeError("维护会话已关闭")
        return self._binding.commissioning_port.commissioning_capabilities  # type: ignore[union-attr]

    @property
    def target_revision(self) -> str | None:
        """返回当前维护 Adapter 活动的 PointSet/程序集版本。"""

        if self._closed:
            raise RuntimeError("维护会话已关闭")
        return self._binding.commissioning_port.commissioning_target_revision  # type: ignore[union-attr]

    def snapshot(self) -> CommissioningSnapshot:
        """读取当前机械臂调试快照。"""

        if self._closed:
            raise RuntimeError("维护会话已关闭")
        return self._binding._commissioning_snapshot(self.owner_id)

    def execute(self, command: CommissioningCommand) -> CommandResult:
        """执行一个封闭机械臂调试命令。"""

        if self._closed:
            raise RuntimeError("维护会话已关闭")
        return self._binding._execute_commissioning(self.owner_id, command)

    def close(self) -> None:
        """在没有调试 Fence 时释放维护占用。"""

        if self._closed:
            return
        self._binding._close_maintenance(self.owner_id)
        self._closed = True

def bind_runtime(
    runtime: Any,
    endpoint_ids: frozenset[str],
    *,
    owner_id: str,
    rail_mounted: bool,
    commissioning_port: RobotCommissioningPort | None = None,
    deployment_mode: DeploymentMode | None = None,
) -> RuntimeBinding:
    """取得进程内端点租约并创建公共绑定。

    参数：底层运行时、端点集合、所有者和是否为 WorkCell。
    返回：持有租约的绑定。异常：端点重复时由租约注册表拒绝。
    """

    return RuntimeBinding(
        runtime,
        _ENDPOINTS.acquire(owner_id, endpoint_ids),
        rail_mounted,
        commissioning_port,
        deployment_mode,
    )


def build_test_runtime(
    runtime: Any,
    endpoint_ids: frozenset[str],
    *,
    owner_id: str,
    commissioning_port: RobotCommissioningPort | None = None,
    deployment_mode: DeploymentMode = DeploymentMode.SIMULATION,
) -> RuntimeBinding:
    """为测试注入内存运行时，同时保留端点唯一性门禁。"""

    return bind_runtime(
        runtime,
        endpoint_ids,
        owner_id=owner_id,
        rail_mounted=hasattr(runtime, "rail"),
        commissioning_port=commissioning_port,
        deployment_mode=deployment_mode,
    )


@dataclass(frozen=True)
class _CommissioningOnlyRuntime:
    """只承载维护端口的运行时占位，不伪装成生产执行后端。"""

    commissioning_port: RobotCommissioningPort

    @property
    def has_unsettled_fence(self) -> bool:
        """把 Adapter 的未知执行状态投影给端点租约关闭门禁。"""

        return self.commissioning_port.commissioning_snapshot().execution_fenced

    def execute(self, *_args: Any, **_kwargs: Any) -> CommandResult:
        """拒绝从维护专用绑定派发生产机器人指令。"""

        raise RuntimeError("维护专用 Robot runtime 不提供生产动作入口")


def bind_commissioning_runtime(
    commissioning_port: RobotCommissioningPort,
    endpoint_ids: frozenset[str],
    *,
    owner_id: str,
    deployment_mode: DeploymentMode,
) -> RuntimeBinding:
    """为领域设备包已有的调试 Adapter 建立正式独占维护绑定。

    参数：统一调试端口、物理端点集合、部署所有者和维护/仿真模式。
    返回：只能打开维护会话的 ``RuntimeBinding``。异常：生产模式、空端点或
    端点重复时关闭失败。安全：该绑定没有生产 ``execute`` 能力，避免把为了
    调试而创建的 MoveIt 客户端变成第二个生产执行入口。
    """

    if deployment_mode not in {
        DeploymentMode.MAINTENANCE,
        DeploymentMode.SIMULATION,
    }:
        raise ValueError("维护专用 Robot runtime 禁止使用 production 模式")
    if not endpoint_ids:
        raise ValueError("维护专用 Robot runtime 必须声明物理端点")
    runtime = _CommissioningOnlyRuntime(commissioning_port)
    return bind_runtime(
        runtime,
        endpoint_ids,
        owner_id=owner_id,
        rail_mounted=False,
        commissioning_port=commissioning_port,
        deployment_mode=deployment_mode,
    )
