---
name: introduce-unilab-robot
description: >-
  一键把 unilab_robot_template 机械臂引用进领域仓。Agent 必须先读本 skill，再跑
  introduce_arm.py；领域需改部分见 docs/demo。支持 greenfield 最小导轨+机械臂领域包+graph、
  MoveIt catalog、Preview catalog、领域自有 MoveIt、设备卡片 scaffold。Greenfield 在
  Workbench 验 FE 3D 前必读 docs/demo/workbench-fe-3d-gate.md。
---

# 一键引入机械臂（introduce-unilab-robot）

**硬性流程**：先读 [docs/demo/README.md](./docs/demo/README.md) 确认模式 → dry-run → `--apply`。

## 默认模型（未显式指定时）

| 部件 | 默认 | 说明 |
|------|------|------|
| 机械臂（Arm） | **`cr5`**（`unilab-arm-cr5` / `package_moveit`） | `--new-domain` 可省略 `--catalog` |
| 导轨（Rail） | **L1 `unilab-rail-linear`** | mesh：`packages/unilab-rail-linear/.../models/meshes/arm_slideway.stl`；provider：`unilab_rail_linear.static_layout:build_default_rail`（SZLab 同源视觉 + 包围盒碰撞） |
| 机械臂卡片（Arm Device Card） | **每次运行自动检测并补齐** | 缺 manifest / `*_arm_card.py` / mixin / **commissioning 资产** 即创建；仅 `--skip-card` 可跳过 |

覆盖：`--catalog cr7` / `--domain-owned` / `--preview-catalog`；导轨 mesh 用 `--rail-stl <path>`。

**STL 门禁**：写入前校验 80 字节头 + 三角面数 × 50 + 84 = 文件长；坏 STL（含 184B 空占位、82B 头）会导致 FE **API 200 但 3D 无导轨**（见 [workbench-fe-3d-gate.md](./docs/demo/workbench-fe-3d-gate.md)）。

## 模式选择

| 场景 | 命令模式 | demo 目录 |
|------|----------|-----------|
| **greenfield 最小导轨+臂+catalog** | `--new-domain` + `--catalog` | [minimal-rail-arm-catalog](./docs/demo/minimal-rail-arm-catalog/) |
| **greenfield domain-owned + vendor** | `--new-domain` + `--domain-owned` + vendor | [minimal-rail-arm-domain-owned](./docs/demo/minimal-rail-arm-domain-owned/) |
| template L1 MoveIt（cr5/cr7） | `--catalog SLUG` | [catalog-moveit-cr5](./docs/demo/catalog-moveit-cr5/) |
| template L1 Preview（elite-cs66） | `--preview-catalog SLUG` | [catalog-preview-elite-cs66](./docs/demo/catalog-preview-elite-cs66/) |
| 领域自有 URDF MoveIt | `--domain-owned` | [domain-owned-moveit](./docs/demo/domain-owned-moveit/) |

graph 片段模板：[graph-rail-arm.template.json](./docs/demo/graph-rail-arm.template.json)

卡片薄包装参照：[domain-card-moveit-szlab](./docs/demo/domain-card-moveit-szlab/) · [domain-card-preview-hydration](./docs/demo/domain-card-preview-hydration/)

## 命令模板

在 **Uni-Lab-Core 根**或 **unilab_robot_template 根**：

```bash
python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  (--domain <已有领域仓> | --new-domain <空目录>) \
  --device <graph 机械臂 id> \
  [--rail-device rail] \
  <模式标志> \
  [--scaffold-graph] [--skip-graph] \
  [--vendor-urdf ...] [--vendor-mesh-dir ...] \
  [--skip-card] [--with-preview-scaffold] \
  [--apply] [--install] [--migrate] [--skip-check]
```

### A. Greenfield catalog（一键：cr5 + SZLab 导轨 + 机械臂卡片）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain F:/path/to/MyLab \
  --rail-device rail --device robot \
  --scaffold-graph --apply --install
```

**硬性**：脚本 stdout 须含 `card_assessment`；缺卡片则 `card_ensure.created_or_patched: true`（apply 后磁盘必有 manifest + `*_arm_card.py` + mixin 继承）。不要卡片才用 `--skip-card`。

### B. Greenfield domain-owned + vendor

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain F:/path/to/MyLab \
  --device szlab_mixer_robot --rail-device szlab_mixer_rail \
  --domain-owned \
  --vendor-urdf F:/path/to/cr20_robot.urdf \
  --vendor-mesh-dir F:/path/to/meshes/cr20 \
  --scaffold-graph --apply --migrate
```

### C. 已有领域包只导入 catalog 臂（不改 graph，自动补卡片）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device my_robot \
  --catalog cr5 \
  --apply --install
```

### D. Preview catalog（elite-cs66）

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device elite_preview \
  --preview-catalog elite-cs66 \
  --with-preview-scaffold --apply --install
```

### E. 已有领域包 domain-owned

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/path/to/domain \
  --device szlab_mixer_robot \
  --domain-owned --apply --migrate
```

## 参数

| 参数 | 含义 |
|------|------|
| `--domain` | 已存在的领域仓根 |
| `--new-domain` | 新建最小导轨+机械臂领域包路径 |
| `--device` / `--arm-device` | graph 里机械臂 `@device` id |
| `--rail-device` | graph / 导轨 `@device` id（默认 `rail`） |
| `--catalog` | MoveIt L1：cr5 / cr7（**greenfield 默认 cr5**） |
| `--rail-stl` | 覆盖默认 SZLab 导轨 mesh |
| `--preview-catalog` | Preview L1：elite-cs66 |
| `--domain-owned` | 领域 L3 moveit_model |
| `--scaffold-graph` | 写/合并 graph（`--new-domain` 时默认开启） |
| `--skip-graph` | 不修改 graph |
| `--graph-file` | graph 相对路径（默认 `deployment/graphs/local-debug.json`） |
| `--vendor-urdf` / `--vendor-mesh-dir` | domain-owned greenfield vendor 资产 |
| `--apply` | 写入（默认 dry-run JSON） |
| `--install` | catalog / preview-catalog：`pip install -e` L1 包 |
| `--migrate` | domain-owned：链式 `migrate_domain_arm.py --apply` |
| `--skip-card` | 跳过机械臂卡片检测与补齐（默认**不跳过**） |
| `--with-preview-scaffold` | preview-catalog：从 demo 模板生成设备骨架 |
| `--card-title` / `--device-title` | 卡片 / Preview 显示名 |
| `--skip-check` | 跳过 `check_domain_arm_assembly.py` |
| `--domain-pkg` | 覆盖 Python 包名 |

## 编排顺序

1. domain / rail / graph scaffold（greenfield 或 `--scaffold-graph`）
2. domain-owned vendor 资产（若 `--vendor-urdf`）
3. **卡片门禁**：`assess_arm_card` + `assess_moveit_commissioning` 检测 manifest / `*_arm_card.py` / mixin / PointSet / `moveit_commissioning.py` / `rail_simulation.py`；缺则补齐（`--skip-card` 除外）
4. **MoveIt commissioning scaffold**（catalog 模式）：PointSet v3、`robot_cell/point_set_v3.py`、`moveit_commissioning.py`、可点击 `build_*_arm_card_context()`；导轨 `post_init` 登记仿真轴
5. 机械臂 provider / pyproject / device.py（缺 mixin / `post_init` 时自动 patch）
6. 前端卡片 manifest（`frontend/cards/<device>-card/`）
7. `--install` 或 `--migrate`
8. `check_domain_arm_assembly.py`（非 preview、非 `--skip-check`）
9. **Greenfield + Workbench**：读 [workbench-fe-3d-gate.md](./docs/demo/workbench-fe-3d-gate.md)，全量清理后桌面版启动，跑 HTTP + Edge 日志验收（**checker exit 0 ≠ FE 3D 可用**）；**卡片按钮**不应再报 `NotImplementedError` / 「MoveIt 调试端口未就绪」

## Greenfield Workbench 启动文件（skill 默认写入）

greenfield 除导轨/臂/graph/卡片外，**一并 scaffold**：

| 文件 | 作用 |
|------|------|
| `deployment/local_config.py` | Backend 本地配置 |
| `pyproject.toml` → `[tool.unilabos.startup].graph` | CLI / Host 默认启动图 |
| `package.yaml` + `{domain_pkg}/workflows/__init__.py` | 可编辑包工作流（Workflow）源码身份（允许空 `workflows: []`） |
| `.unilabos/environment.local.json` | `externalDevicesOnly: false` + `graphPath` |
| `deployment/robot_cell/assets/point_sets/{domain_pkg}-rail-arm.v3.yaml` | 卡片可点的最小 PointSet v3 |
| `{domain_pkg}/robot_cell/point_set_v3.py` | PointSet 解析器 |
| `{domain_pkg}/devices/<device>/moveit_commissioning.py` | MoveIt 调试绑定（`_moveit_split_binding`） |
| `{domain_pkg}/devices/rail_simulation.py` + `rail.py#post_init` | mock 导轨仿真轴（卡片 rail 快照/移动） |

缺任一项会导致 `workspace start` 报 `graph_not_configured` / `config_not_found` / `source_target_unavailable`。

## Greenfield Workbench FE 3D（硬性门禁）

`--new-domain` 完成后若用户要看 **物料（Material）3D**：

1. **OS worktree** 须含 v2 设备网格管线（`prepare_device_mesh_runtime`、`device_nodes`、device-telemetry、kinematic-models）。缺则合入 scratch/OS 补丁后再验，勿只改领域包。
2. **启动**：`start-workbench.mjs --desktop`，`--workspace` 指向**新领域包**；全量清理旧 SZLab / 3100 占用（见 workbench-desktop-default 规则）。
3. **Backend ready 后** 若无 Edge：`unilab workspace start --component all`（或 UI os.start）。
4. **inventory 陈旧**（graph 无 `rendering.kinematics`）：`workspace reset-local --yes` 再 start。
5. **必验**：Edge 同时 init `rail` + `robot`；`robot.urdf` 200；materials graph 含 `model.path` + `kinematics`。

**导轨 mesh**：Greenfield 默认 **SZLab arm_slideway.stl**（`models/rail_szlab/`）；与机械臂同走 `package_static` + `joint_state_provider` → `kinematic-models/rail.urdf`。验收须同时查 rail 与 robot（gate §3b）；`collision.stl`/mesh 响应须 **>500B 且 STL 结构合法**。库存陈旧时 **重启 Backend** 触发 rendering 增量合并，仍不对再 `reset-local`。禁止为对齐 FE 改 RViz。

## 脚本 stdout

JSON 含 `demo_index`、`demo_reference`、`domain_scaffold`、`rail_scaffold`、`graph_scaffold`、`moveit_commissioning`、`commissioning_assessment`、`card_assessment.commissioning_complete`、`vendor_assets`（若启用）。`ok: false` 时读 `error` / `pip_install` / `migrate` / `checker`。

## 之后读哪个 skill

| 下一步 | Skill |
|--------|-------|
| Greenfield Workbench FE 3D 验收 | [workbench-fe-3d-gate.md](./docs/demo/workbench-fe-3d-gate.md) |
| catalog 臂 + 导轨整机 | [use-unilab-arm-package](../use-unilab-arm-package/SKILL.md) |
| 领域 MoveIt 完整移植 | [domain-owned-arm-rail](../domain-owned-arm-rail/SKILL.md) |
| 新厂商 URDF 打 L1 包 | [package-arm-moveit](../package-arm-moveit/SKILL.md) |
| 人类长文 | [docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md) |
