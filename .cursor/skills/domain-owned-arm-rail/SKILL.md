---
name: domain-owned-arm-rail
description: >-
  Full domain-owned MoveIt migration in L3: assets, moveit_model.py, device provider,
  tests, RViz E2E, then FE via fe-must-match-rviz. Follow references/complete-domain-moveit-migration.md.
  Generic—read slug/URDF from disk; never hardcode vendor or lab.
---

# 领域自有机械臂 / 导轨型号包

## Agent：完整迁移

**按顺序执行** [`references/complete-domain-moveit-migration.md`](references/complete-domain-moveit-migration.md) **阶段 0–6**，全部完成才算迁移结束。

| 阶段 | 内容 |
|------|------|
| 0 | 范围：use-unilab-arm-package 不变量、device id |
| 1 | 资产：`models/model.yaml` + URDF + meshes |
| 2 | `moveit_model.py`（映射来自当前 URDF） |
| 3 | `device.py` provider + digest **与阶段 1–2 同批** |
| 4 | 测试 + check_domain_arm_assembly |
| 5 | 重启 OS → RViz 有 mesh、零位正常 |
| 6 | `fe-must-match-rviz` 对齐 FE |

**禁止**拆分只做 URDF 或只做 provider；禁止写死厂商/实验室路径。

## 默认原则

型号资产在 `devices/<device>/models/`；template 提供 model-kit + runtime + 组合逻辑。

## 目录结构

```text
<domain_pkg>/devices/<robot_device>/
  device.py
  moveit_model.py
  robot_module.py          # 可选，Module API v1
  models/model.yaml
  models/<vendor>.urdf
  models/meshes/<mesh_slug>/
  tests/test_*_moveit_model.py
```

## 双管道（阶段 5 vs 6）

| 管道 | 阶段 | 入口 |
|------|------|------|
| RViz / MoveIt | 5 | `package_moveit` → `moveit_model` |
| FE 3D | 6 | `shape` / xacro + 物理图 |

## moveit_model 模板

```python
_MODELS = Path(__file__).resolve().parent / "models"

_ARM_SPEC = SixAxisArmModelSpec(
    model_slug="<slug>",
    source_urdf=_MODELS / "<vendor>.urdf",
    expected_source_digest="<models/model.yaml source.sha256>",
    mesh_paths=(...),
    link_names={...},
    joint_names={...},
    canonical_joint_names=(...),
    flange_frame="<tool_link>",
    last_link="<last_link>",
    disabled_collisions=(...),
    mock_system_suffix="<slug>",
    model_descriptor_path=_MODELS / "model.yaml",
)
```

## 依赖

`unilab-robot-contracts`, `unilab-robot-model-kit`, `unilab-robot-runtime`, `unilab-rail-mounted-arm`

## 禁止

- 不完整迁移（缺资产、缺 provider、缺 RViz、缺 FE 却报完成）  
- 映射照搬 catalog 另一 URDF  
- 改 OS/template 代替补领域资产  

## 可选：留在 template catalog

跨实验室复用且**不迁 L3** 时用 `unilab_arm_<slug>`；见 `use-unilab-arm-package`。
