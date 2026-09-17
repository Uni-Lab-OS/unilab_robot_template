---
name: introduce-unilab-robot
description: >-
  一键把 unilab_robot_template 机械臂引用进领域仓。Agent 必须先读本 skill，再跑
  introduce_arm.py；领域需改部分见 docs/demo。支持 MoveIt catalog、Preview catalog、
  领域自有 MoveIt、设备卡片 scaffold。
---

# 一键引入机械臂（introduce-unilab-robot）

**硬性流程**：先读 [docs/demo/README.md](./docs/demo/README.md) 确认模式 → dry-run → `--apply`。

## 模式选择

| 场景 | 命令模式 | demo 目录 |
|------|----------|-----------|
| template L1 MoveIt（cr5/cr7） | `--catalog SLUG` | [catalog-moveit-cr5](./docs/demo/catalog-moveit-cr5/) |
| template L1 Preview（elite-cs66） | `--preview-catalog SLUG` | [catalog-preview-elite-cs66](./docs/demo/catalog-preview-elite-cs66/) |
| 领域自有 URDF MoveIt | `--domain-owned` | [domain-owned-moveit](./docs/demo/domain-owned-moveit/) |

卡片薄包装参照：[domain-card-moveit-szlab](./docs/demo/domain-card-moveit-szlab/) · [domain-card-preview-hydration](./docs/demo/domain-card-preview-hydration/)

## 命令模板

在 **Uni-Lab-Core 根**或 **unilab_robot_template 根**：

```bash
python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> \
  --device <graph_device_id> \
  <模式标志> \
  [--with-card] [--with-preview-scaffold] \
  [--apply] [--install] [--migrate] [--skip-check]
```

### A. MoveIt catalog（cr5 / cr7）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device my_robot \
  --catalog cr5 \
  --with-card --apply --install
```

等价 provider：`unilab_arm_cr5:build_moveit_model`

### B. Preview catalog（elite-cs66）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device elite_preview \
  --preview-catalog elite-cs66 \
  --with-preview-scaffold --with-card --apply --install
```

等价 model：`package_static` + 领域 `model.py` / `mounts.py`。引入后 **必须**改 `mounts.py` 与 `_arm_card_context()`（见 demo）。

### C. 领域自有 MoveIt

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device szlab_mixer_robot \
  --domain-owned --with-card --apply --migrate
```

## 参数

| 参数 | 含义 |
|------|------|
| `--domain` | 领域仓根 |
| `--device` | 图（Graph）里 `@device` id |
| `--catalog` | MoveIt L1：cr5 / cr7 |
| `--preview-catalog` | Preview L1：elite-cs66 |
| `--domain-owned` | 领域 L3 moveit_model |
| `--apply` | 写入（默认 dry-run JSON） |
| `--install` | catalog / preview-catalog：`pip install -e` L1 包 |
| `--migrate` | domain-owned：链式 `migrate_domain_arm.py --apply` |
| `--with-card` | 生成 `frontend/cards/<device>-card` manifest |
| `--with-preview-scaffold` | preview-catalog：从 demo 模板生成设备骨架 |
| `--card-title` / `--device-title` | 卡片 / Preview 显示名 |
| `--skip-check` | 跳过 `check_domain_arm_assembly.py` |
| `--domain-pkg` | 覆盖 Python 包名 |

## 脚本 stdout

JSON 含 `demo_index`、`demo_reference`、`preview_scaffold`（若启用）。`ok: false` 时读 `error` / `pip_install` / `migrate` / `checker`。

## 之后读哪个 skill

| 下一步 | Skill |
|--------|-------|
| catalog 臂 + 导轨整机 | [use-unilab-arm-package](../use-unilab-arm-package/SKILL.md) |
| 领域 MoveIt 完整移植 | [domain-owned-arm-rail](../domain-owned-arm-rail/SKILL.md) |
| 新厂商 URDF 打 L1 包 | [package-arm-moveit](../package-arm-moveit/SKILL.md) |
| 人类长文 | [docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md) |
