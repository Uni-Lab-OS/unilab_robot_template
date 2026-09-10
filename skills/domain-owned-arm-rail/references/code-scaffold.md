# 领域机械臂移植：可复制代码 scaffold

**优先用 CLI**（一条命令，exit 0 即完成）：

```bash
python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <robot_device> --apply
```

手工粘贴时按本节模板；Agent 默认应跑 CLI，不要只改 skill 文档。

| 占位符 | 读哪里 |
|---|---|
| `<domain_pkg>` | 领域 Python 包名，如 `szlab_poly_studio` |
| `<robot_device>` | `@device` id，如 `szlab_mixer_robot` |
| `<distribution>` | `moveit_runtime_assembly._manifest()` 里 arm 的 distribution，如 `szlab-poly-studio` |
| `<module_version>` | 与 manifest `_ModuleRef` version 一致，如 `0.1.0` |

路径根：`<domain_pkg>/devices/<robot_device>/`

---

## 1. `adapters/__init__.py`（若为空或缺导出则整文件替换）

```python
"""<robot_device> 执行后端适配器公开导出。"""

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
```

若还有 `plc.py` / `tcp_sdk.py`，按 template `unilab_arm_cr7/adapters/__init__.py` 追加导出。

---

## 2. `arm_module.py`（新建；全文如下，仅改首行 docstring）

```python
"""<robot_device> 后端生命周期深模块（与 template ArmModule 相同）。"""

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
                    f"机械臂存在未物理结算命令，禁止新派发: {fenced}",
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
                f"后端异常且派发结果不明: {exc}",
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

    def request_controlled_stop(
        self, command_id: str, reason: str
    ) -> CommandResult:
        existing = self.journal.get(command_id)
        if existing is not None and not existing.state.terminal:
            self.journal.update(
                CommandResult(
                    command_id,
                    CommandState.EXECUTION_UNKNOWN,
                    f"已请求机械臂受控停止，等待物理结算: {reason}",
                ),
                fenced=True,
            )
        try:
            result = self.backend.request_stop(command_id, reason)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"机械臂受控停止请求失败: {exc}",
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
            raise KeyError(f"命令不存在: {command_id}")
        if existing.state.terminal and not self.journal.is_fenced(command_id):
            return existing
        try:
            result = self.backend.reconcile(command_id)
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, f"对账失败: {exc}"
            )
        if (
            existing.state is CommandState.EXECUTION_UNKNOWN
            and result.state.terminal
        ):
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "已观测到候选终态，必须提供精确物理结算见证才能解除阻断",
                {"candidate_state": result.state.value, "candidate": dict(result.output)},
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
            conflicts = {
                key
                for key in prepared
                if key in result.output and result.output[key] != prepared[key]
            }
            if conflicts:
                raise CommandRejectedError(
                    f"后端 UNKNOWN 结算输出与控制面冲突: {sorted(conflicts)}"
                )
            result = CommandResult(
                result.command_id,
                result.state,
                result.message,
                {**dict(result.output), **dict(prepared)},
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
```

---

## 3. `robot_factory.py`（新建）

```python
"""<robot_device> 的 Robot Module API v1 工厂。"""

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

from .adapters import MoveIt2ClientPort, MoveItBackend
from .arm_module import ArmModule
from .moveit_model import (
    MODEL_DESCRIPTOR,
    build_joint_state_name_map,
    build_moveit_model,
    create_moveit_commissioning_adapter,
)

MODULE_API_VERSION = 1
MODULE_KIND = "arm"
MODULE_VERSION = "<module_version>"


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
```

**若** `moveit_model.py` 尚无 `create_moveit_commissioning_adapter`：从 template `unilab_arm_cr7/factory.py` 的 `create_moveit_commissioning_adapter` 抄入 `moveit_model.py`（SZLab 已有则跳过）。

**若** 走 PLC 后端：在 `robot_factory.py` 追加 template 的 `create_plc_backend` / `create_tcp_sdk_backend`，并扩展 `adapters/__init__.py`。

---

## 4. `robot_module.py`（新建；runtime 加载此模块）

```python
"""<robot_device> Robot Module API v1 入口。"""

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
```

---

## 5. `tests/test_robot_module.py`（新建）

```python
"""Robot Module API v1 门禁。"""

from __future__ import annotations

import importlib


def test_robot_module_exports_module_api_v1() -> None:
    module = importlib.import_module(
        "<domain_pkg>.devices.<robot_device>.robot_module"
    )
    assert module.MODULE_API_VERSION == 1
    assert module.MODULE_KIND == "arm"
    assert module.__version__ == "<module_version>"
    assert callable(module.create_arm_module)
    assert callable(module.create_moveit_backend)
    assert callable(module.build_moveit_model)
    assert module.MODEL_DESCRIPTOR.planning_group
```

---

## 6. `moveit_runtime_assembly.py`（或 `*runtime_assembly.py`）manifest 补丁

grep `_ModuleRef` / `python_package`，**只改 arm 最后一项**：

```python
        arm=_ModuleRef(
            "<distribution>",
            "<module_version>",
            frozenset({arm_endpoint}),
            "<domain_pkg>.devices.<robot_device>.robot_module",
        ),
```

**禁止**仍写 `...moveit_model`。

`@device` 的 provider **保持**：

```python
"provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model",
```

---

## 7. 型号符号 grep（换 slug 时同 PR 执行）

```bash
# 在领域仓根目录；把 OLD_SLUG 换成 grep 到的旧名（如 cr7）
rg -n "OLD_SLUG" <domain_pkg>/ deployment/ frontend/ tests/
```

**必须整批一致**的字段：

- `models/model.yaml` → `kinematic_joints` / `moveit_group`
- `moveit_model.py` → `canonical_joint_names` / `MODEL_DESCRIPTOR.tip_link`
- `deployment/robot_cell/assets/point_sets/*.yaml` → revision + 关节名
- `robot_cell/*_site_ik.py` → `HOME_SEED_DEG`
- `moveit_runtime_assembly.py` → 硬编码 point_set / calibration 路径
- 卡片 `pointSetRevision` / `calibrationRevision`

---

## 8. 阶段 4 命令

```bash
python domain-owned-arm-rail-skill/scripts/check_domain_robot_module.py \
  --domain <领域仓根> --device <robot_device> --import-test
pytest <domain_pkg>/devices/<robot_device>/tests/ -q
python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain <领域仓根>
```

---

## 9. 阶段 5 Edge 日志 grep

```bash
rg "不支持 Robot Module API v1|初始工具附着延迟发布失败，统一运行时已关闭" <领域仓>/.unilabos/logs/
```

**零匹配**才能进入阶段 6。

---

## 10. 阶段 6 FE（完整迁移）

读 `.cursor/skills/fe-must-match-rviz/SKILL.md`，并核对：

- `uni-lab-fe/packages/services/src/materialBackendGraphCodec.ts` 解析 `rendering.parent_link`
- `uni-lab-fe/packages/pascal-lab-plugin/src/materialPlacementProjection.ts` 子设备挂 `{rail_id}_rail_carriage`

Workbench **桌面版**：导轨棱柱关节变化 → 臂基座在 3D 里跟动。

---

## SZLab 实例（填好占位符后的值）

| 占位符 | 值 |
|---|---|
| `<domain_pkg>` | `szlab_poly_studio` |
| `<robot_device>` | `szlab_mixer_robot` |
| `<distribution>` | `szlab-poly-studio` |
| `<module_version>` | `0.1.0` |
| manifest 文件 | `devices/szlab_mixer_robot/moveit_runtime_assembly.py` 约 L705–710 |
| 待改 | L709：`moveit_model` → `robot_module` |
