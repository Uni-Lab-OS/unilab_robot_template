---
name: domain-owned-arm-rail
description: >-
  领域机械臂一键移植：命名只绑 device_id；robot_module、MoveIt attach 链、RViz
  夹爪 attach 必须在 migrate --apply 一次完成。详见 references/skill-scope.md。
---

# 领域自有机械臂 / 导轨完整移植（一次 `--apply` 完成）

人类可读使用说明：[`docs/DOMAIN_OWNED_ARM_RAIL.md`](../../docs/DOMAIN_OWNED_ARM_RAIL.md)

本目录与仓根 `domain-owned-arm-rail-skill/` **镜像同步**（scripts / references / tests 须一致）。

## 本 skill 不做什么

→ [`references/skill-scope.md`](references/skill-scope.md)

- **不会**在 OS / move_group 启动时运行  
- **不会**向 controller 配置写入任何关节名  
- 除非你手动 `migrate ... --apply`，否则**不写**领域仓  

## 入口

```bash
python skills/domain-owned-arm-rail/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <机械臂 id> --apply
```

在 Uni-Lab-Core 仓根也可：

```bash
python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <机械臂 id> --apply
```

`--apply` 顺序（**缺任一步 = 移植未完成**）：

1. `fix_device_joint_names.py` — 剔除 `{device}_{slug}_joint_*`
2. `scaffold_domain_robot_module.py` — `robot_module` / `robot_factory`
3. `fix_runtime_manifest.py` — manifest → `robot_module`
4. `scaffold_domain_adapters.py` — MoveIt adapters
5. **`fix_moveit_runtime_attach.py`** — catalog → 领域模块；**写对 `moveit_model.model_ref`**
6. 四门禁：`check_domain_robot_module` / joint alignment / **attach contract** / domain arm assembly

## 领域命名（只绑 device_id）

- endpoint：`moveit:{device_id}:arm`
- 规划组：`{device_id}_arm`
- 关节 qualified：`{device_id}_joint_{n}`
- **禁止** vendor 型号 slug 进入领域运行时命名

## `model_ref`（RViz 夹爪 attach 硬门禁）

| 用途 | 格式 |
|---|---|
| PointSet `components.arm.model_ref` | `package://{domain_pkg}/devices/{device_id}/models/model.yaml` |
| `moveit_model.py` → `MODEL_DESCRIPTOR.model_ref` | **必须与 PointSet 逐字相同** |

## 关节命名

→ [`references/joint-naming.md`](references/joint-naming.md)

| 层 | 格式 |
|---|---|
| canonical | `joint_1` … `joint_6` |
| qualified | `{device_id}_joint_{n}` |

## 单独门禁

```bash
python skills/domain-owned-arm-rail/scripts/check_moveit_joint_alignment.py \
  --domain <领域仓根> --device <机械臂 id>
python skills/domain-owned-arm-rail/scripts/check_moveit_attach_contract.py \
  --domain <领域仓根> --device <机械臂 id>
```

## 其它规则

- ToolContext 用 `moveit.end_effector` 符号引用。
- RViz 权威；FE 对齐见 `fe-must-match-rviz`。

Runbook：[`references/complete-domain-moveit-migration.md`](references/complete-domain-moveit-migration.md)
