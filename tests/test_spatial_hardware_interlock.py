"""空间硬件互锁默认关闭和故障注入测试。"""

from __future__ import annotations

import time
from typing import Any

import pytest
from unilab_rail_mounted_arm import (
    ConfiguredInterlockProvider,
    InterlockBinding,
    SpatialInterlockBinding,
    SpatiallyGuardedInterlockProvider,
)
from unilab_robot_contracts import ObservationState, SafetyInterlockObservation
from unilab_robot_runtime.factory import _interlock_binding


class _Port:
    """按节点返回固定值并记录读取次数的 PLC 端口替身。"""

    def __init__(self, values: dict[str, Any]) -> None:
        """注入节点值；异常对象会在读取时抛出。"""

        self.values = values
        self.calls: list[str] = []

    def read(self, variable: str) -> Any:
        """记录节点并返回或抛出配置的结果。"""

        self.calls.append(variable)
        value = self.values[variable]
        if isinstance(value, BaseException):
            raise value
        return value


def _base_observation(
    *,
    granted: bool = True,
    hardware_enforced: bool = True,
    observed_at: float | None = None,
) -> SafetyInterlockObservation:
    """构造新鲜或指定时间的既有运动互斥硬件观测。"""

    return SafetyInterlockObservation(
        ObservationState.KNOWN,
        time.time() if observed_at is None else observed_at,
        0.5,
        "plc:motion-interlock",
        granted,
        hardware_enforced,
        True,
        False,
        True,
    )


def _enabled_binding() -> SpatialInterlockBinding:
    """构造显式启用且绑定资格摘要的空间硬件互锁。"""

    return SpatialInterlockBinding(
        granted_variable="plc.safety.spatial_granted",
        source="plc:spatial-interlock",
        enabled=True,
        hardware_enforced=True,
        qualified_evidence_digest="a" * 64,
        max_age_s=0.25,
    )


def test_spatial_interlock_is_disabled_without_any_plc_read() -> None:
    """默认配置不得读取节点，更不得继承基础链的 grant。"""

    port = _Port({})
    provider = SpatiallyGuardedInterlockProvider(
        base_observation=lambda: _base_observation(),
        port=port,
        binding=SpatialInterlockBinding(
            granted_variable="plc.safety.spatial_granted",
            source="plc:spatial-interlock",
        ),
    )

    observed = provider.read()

    assert observed.state is ObservationState.UNKNOWN
    assert observed.granted is False
    assert observed.hardware_enforced is False
    assert observed.rail_motion_permitted is False
    assert observed.arm_motion_permitted is False
    assert port.calls == []


@pytest.mark.parametrize(
    ("hardware_enforced", "digest"),
    ((False, "a" * 64), (True, None), (True, "NOT-A-DIGEST")),
)
def test_enabled_spatial_interlock_requires_hardware_chain_and_qualification(
    hardware_enforced: bool,
    digest: str | None,
) -> None:
    """软件开关、空摘要或非硬件来源都不能启用空间许可。"""

    with pytest.raises(ValueError):
        SpatialInterlockBinding(
            granted_variable="plc.safety.spatial_granted",
            source="plc:spatial-interlock",
            enabled=True,
            hardware_enforced=hardware_enforced,
            qualified_evidence_digest=digest,
        )


@pytest.mark.parametrize("raw_value", ("false", 0, 1, None))
def test_non_boolean_spatial_plc_value_fails_closed(raw_value: Any) -> None:
    """PLC 映射类型漂移不能经过 Python truthiness 变成许可。"""

    provider = SpatiallyGuardedInterlockProvider(
        base_observation=lambda: _base_observation(),
        port=_Port({"plc.safety.spatial_granted": raw_value}),
        binding=_enabled_binding(),
    )

    observed = provider.read()

    assert observed.state is ObservationState.UNKNOWN
    assert observed.granted is False
    assert observed.hardware_enforced is False


def test_explicit_spatial_denial_strips_both_motion_permissions() -> None:
    """已知空间拒绝要保留诊断已知性，同时撤销 rail/arm 两种运动许可。"""

    provider = SpatiallyGuardedInterlockProvider(
        base_observation=lambda: _base_observation(),
        port=_Port({"plc.safety.spatial_granted": False}),
        binding=_enabled_binding(),
    )

    observed = provider.read()

    assert observed.state is ObservationState.KNOWN
    assert observed.granted is False
    assert observed.hardware_enforced is True
    assert observed.rail_motion_permitted is False
    assert observed.arm_motion_permitted is False


def test_only_two_validated_hardware_grants_preserve_motion_phase() -> None:
    """基础互斥链和空间链同时为真时才允许协调器继续判定当前相位。"""

    provider = SpatiallyGuardedInterlockProvider(
        base_observation=lambda: _base_observation(),
        port=_Port({"plc.safety.spatial_granted": True}),
        binding=_enabled_binding(),
    )

    observed = provider.read()

    assert observed.state is ObservationState.KNOWN
    assert observed.granted is True
    assert observed.hardware_enforced is True
    assert observed.rail_motion_permitted is True
    assert observed.arm_motion_permitted is False
    assert observed.concurrent_motion_blocked is True
    assert observed.max_age_s == 0.25


def test_stale_base_interlock_cannot_be_rescued_by_spatial_true() -> None:
    """空间节点为真不能覆盖基础安全链过期。"""

    provider = SpatiallyGuardedInterlockProvider(
        base_observation=lambda: _base_observation(observed_at=time.time() - 2.0),
        port=_Port({"plc.safety.spatial_granted": True}),
        binding=_enabled_binding(),
    )

    observed = provider.read()

    assert observed.state is ObservationState.UNKNOWN
    assert observed.granted is False


def test_existing_interlock_provider_rejects_string_boolean() -> None:
    """旧四节点适配器也必须拒绝 ``"false"`` 这类危险配置值。"""

    port = _Port(
        {
            "grant": "false",
            "rail": True,
            "arm": False,
            "blocked": True,
        }
    )
    provider = ConfiguredInterlockProvider(
        port=port,
        binding=InterlockBinding(
            granted_variable="grant",
            rail_permitted_variable="rail",
            arm_permitted_variable="arm",
            concurrent_blocked_variable="blocked",
            hardware_enforced=True,
            source="plc:test",
        ),
    )

    observed = provider.read()

    assert observed.state is ObservationState.UNKNOWN
    assert observed.granted is False


def test_runtime_factory_rejects_string_hardware_enforced() -> None:
    """YAML 字符串不能在运行时组合根被 ``bool(value)`` 悄悄升级。"""

    with pytest.raises(TypeError, match="必须是 bool"):
        _interlock_binding(
            {
                "granted_variable": "grant",
                "rail_permitted_variable": "rail",
                "arm_permitted_variable": "arm",
                "concurrent_blocked_variable": "blocked",
                "hardware_enforced": "false",
                "source": "plc:test",
            }
        )
