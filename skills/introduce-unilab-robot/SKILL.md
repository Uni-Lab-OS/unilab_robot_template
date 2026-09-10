---
name: introduce-unilab-robot
description: >-
  一键把 unilab_robot_template 机械臂引用进领域仓：写 pyproject 依赖、@device
  provider 一句话、source_digest；catalog 或 domain-owned 两种模式。最小成本接入。
---

# 一键引入 unilab_robot_template 机械臂

**一条命令**完成：依赖声明 + `@device model.provider` + `source_digest`（+ 可选安装 / migrate）。

## 情况 A：引用 template 内 catalog 型号（cr5 / cr7）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> \
  --device <robot_device> \
  --catalog cr5 \
  --apply --install
```

等价于在 `@device` 里写一句话：

```python
"provider": "unilab_arm_cr5:build_moveit_model"
```

脚本还会：
- 向 `pyproject.toml` 追加 `unilab-arm-cr5`、`unilab-robot-runtime`
- 创建或更新 `devices/<robot_device>/device.py` 的 `model={...}`
- `--install` 时 `pip install -e` template 对应 L1 包
- 默认跑 `check_domain_arm_assembly.py` 验收

## 情况 B：引用领域自有模型（L3）

前提：`devices/<robot_device>/models/model.yaml` 与 `moveit_model.py` 已就绪。

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> \
  --device <robot_device> \
  --domain-owned \
  --apply --migrate
```

等价于：

```python
"provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model"
```

`--migrate` 会继续跑 `domain-owned-arm-rail` 的 `migrate_domain_arm.py --apply`。

## 参数

| 参数 | 含义 |
|------|------|
| `--domain` | 领域仓根 |
| `--device` | 图里机械臂 `@device` id |
| `--catalog SLUG` | cr5 / cr7 |
| `--domain-owned` | 领域 L3 自有 URDF |
| `--apply` | 真正写入（默认 dry-run 只打印 JSON） |
| `--install` | catalog：editable 安装 L1 包 |
| `--migrate` | domain-owned：继续 migrate |
| `--skip-check` | 跳过装配检查器 |
| `--domain-pkg` | 覆盖自动探测的包名 |

## 之后

- catalog 整机（导轨 + 臂）：读 [use-unilab-arm-package](../use-unilab-arm-package/SKILL.md)
- domain 完整移植：读 [domain-owned-arm-rail](../domain-owned-arm-rail/SKILL.md)
- 人类说明：[docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md)
