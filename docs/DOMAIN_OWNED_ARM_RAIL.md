# 领域自有机械臂 / 导轨（Domain-Owned Arm/Rail）使用说明

把领域仓里 **`model_ownership: domain`** 的机械臂设备，从 catalog 型号包（`unilab_arm_*`）迁到 **L3 领域自有 MoveIt 模型**（`devices/<robot_device>/moveit_model.py`），并完成 **RViz 显示**、**MoveIt attach**、**夹爪（Gripper）附着** 验收。

| 读者 | 入口 |
|------|------|
| 人类操作 | 本文 |
| Agent 细则 | `skills/domain-owned-arm-rail/SKILL.md` |
| 脚本实现 | `skills/domain-owned-arm-rail/`（与仓根 `domain-owned-arm-rail-skill/` 镜像同步） |

---

## 快速上手（7 步）

```
1. 确认 @device 为 model_ownership: domain
2. 阶段 1：vendor URDF + STL 只读拷贝到 devices/<device>/models/
3. 阶段 2：手写 moveit_model.py 映射（只改 Python，不改 URDF）
4. migrate_domain_arm.py --apply（一键 scaffold + attach 链 + 四门禁）
5. 完全停 OS → 清 .unilabos/runtime/ 缓存 → 冷启动（带 RViz）
6. RViz：全 0 位 mesh 连续 + 夹爪 attach 可见
7. FE 3D 对齐 RViz（fe-must-match-rviz）
```

**SZLab 示例**（在 Uni-Lab-Core 仓根）：

```bash
e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab \
  --device szlab_mixer_robot \
  --apply
```

---

## 何时用本流程

| 场景 | 用哪个 |
|------|--------|
| 新型号 URDF 由领域仓自己维护（`devices/.../models/`） | **本说明** |
| 继续用 template 里现成的 `unilab_arm_cr7` 等 catalog 包 | [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) |
| 把厂商 URDF 首次打成 L1 发行包 | `skills/package-arm-moveit/SKILL.md` |

同一领域仓可以 **A 臂 domain、B 臂 catalog 并存**。每次只迁 **一个** `@device` id。

---

## 本 skill 做什么 / 不做什么

| 会做 | 不会做 |
|------|--------|
| 生成 / 修补 `robot_module`、manifest、adapters | 在 OS 启动时自动运行 |
| 修正关节命名、`model_ref`、attach 执行链 | 向 move_group / controller 注入关节名 |
| 跑静态门禁（关节、attach 合同、整机装配） | 未经 `--apply` 时写领域仓 |
| 输出 JSON 步骤报告 | 代替 RViz / FE 人工目视验收 |

**重要**：日志里 `cr7` / `cr16` 出现在测试或禁止模式样例中，**不代表** skill 往系统写入两个型号。若 move_group **同时**找 `{device}_cr7_joint_*` 和 `{device}_cr16_joint_*`，说明 OS **叠了多套历史 controller 缓存**，需冷启动清缓存（见故障速查）。

---

## 目录布局

```text
devices/<robot_device>/
  moveit_model.py          # L3 映射与 build_moveit_model
  moveit_runtime_assembly.py
  robot_module.py          # migrate --apply 生成
  robot_factory.py
  adapters/
  models/
    model.yaml             # 禁止放在 device 根目录
    <vendor>.urdf          # 厂商只读副本
    meshes/<mesh_slug>/*.STL
  tests/
```

endpoint 后缀固定 `arm`；规划组 `{device_id}_arm`；mesh 目录路径可保留 vendor 文件夹名，**领域运行时命名不得含 vendor 型号 slug**。

**完整路径示例（SZLab）**：

```text
Uni-Lab-SZLab/szlab_poly_studio/devices/szlab_mixer_robot/
```

`<robot_device>` = 文件夹名 = 图里 `"id"` 字段；**领域专属模型就在这个文件夹里**，不在 `unilab_robot_template/packages/`。

---

## 文件还不存在时怎么办

**migrate 不是「从零建一台机械臂」**，而是：**在 `devices/<robot_device>/` 已具备 L3 骨架的前提下**，自动补 Module API、manifest、attach 链并跑门禁。

若目录里什么都没有，直接 `--apply` 会报 `未找到 package_moveit device` 或 import 失败——这是预期行为。

### 必须先手工准备（migrate 不会生成）

| 文件 / 目录 | 作用 | 没有会怎样 |
|-------------|------|------------|
| `device.py` | `@device` 装饰器；`model.type=package_moveit`；`provider` 指向本目录 `moveit_model:build_moveit_model` | `find_arm_device` 找不到设备 |
| `models/model.yaml` | 型号身份、`source.sha256`、`kinematic_joints` | attach 合同 / digest 门禁失败 |
| `models/<vendor>.urdf` | 厂商 URDF 只读拷贝 | 无 robot_description |
| `models/meshes/<slug>/*.STL` | 与 URDF 配套的 mesh | RViz 无 STL |
| `moveit_model.py` | `build_moveit_model()`；link/joint 映射 | provider import 失败 |
| `moveit_runtime_assembly.py` | 运行时 manifest；arm 模块引用 | attach 链 / manifest 修复无目标 |

图（Graph）里还必须有对应 `"id": "<robot_device>"` 节点，且 `parent` 指向导轨。

### migrate `--apply` 会自动生成 / 修补

| 文件 | 动作 |
|------|------|
| `robot_module.py` | 新建（或 `--force` 覆盖） |
| `arm_module.py` | 新建 |
| `robot_factory.py` | 新建 |
| `adapters/__init__.py` | 补导出 |
| `moveit_runtime_assembly.py` 内 arm `_ModuleRef` | 改指向 `robot_module` |
| attach / commissioning 里的 catalog 引用 | 改绑领域模块 |
| `moveit_model.py` 内 `MODEL_DESCRIPTOR.model_ref` | 写对领域路径 |
| 全仓 `{device}_{slug}_joint_*` 模板 | `fix_device_joint_names` 剔除 |

### 推荐顺序（从 catalog 迁到 domain，或新建 domain 臂）

```
① 在领域仓建目录  <domain_pkg>/devices/<robot_device>/
② 写 device.py（provider 先指向即将创建的 moveit_model）
③ 阶段 1：拷贝 vendor URDF + STL → models/
④ 写 models/model.yaml（上游 digest、joint_1..joint_6）
⑤ 写 moveit_model.py（参考 unilab_arm_cr7 的 Python 结构，映射来自当前 URDF）
⑥ 从 pTLC / 现有 SZLab 臂复制 moveit_runtime_assembly.py 骨架并改 device id
⑦ migrate_domain_arm.py --apply   ← 此时才跑
⑧ 冷启动 OS + RViz 验收
```

**抄样本时**：

- 目录结构 / 装配：`Uni-Lab-pTLC` 或 `Uni-Lab-SZLab/.../szlab_mixer_robot/`
- `moveit_model.py` Python 结构：`unilab_robot_template/packages/unilab-arm-cr7/.../moveit_model.py`（**只抄 Python，不抄 URDF 映射**）
- scaffold 模板兜底：`domain-owned-arm-rail-skill/references/code-scaffold.md`

### 两种起点

| 起点 | 你要做的 |
|------|----------|
| 已有 catalog 臂（`unilab_arm_cr7`）要改 domain | 在 **同一** `devices/<robot_device>/` 下落 URDF + 写 `moveit_model.py`，改 `device.py` 的 provider / digest，再 `--apply` |
| 全新机械臂设备 | 先按 [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) 把图 + 导轨 + `@device` 搭好，再在本目录补 models + `moveit_model.py`，最后 `--apply` |

---

## 前置条件

1. Python 环境：`e:/miniforge3/envs/unilab`。
2. 已读 [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) 的装配不变量（导轨 / 机械臂分设备、禁止第七轴合一体 MoveIt 等）。
3. 目标设备在图（Graph）里已是 `package_moveit`，且 `@device` 声明 `model_ownership: domain`。
4. **vendor URDF 只读拷贝**已落盘；未通过 digest 门禁前 **不要** 跑 `--apply`。

---

## 一键移植（主入口）

```bash
# 推荐：Uni-Lab-Core 仓根
e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain <领域仓根> \
  --device <机械臂 @device id> \
  --apply

# 或经模板 skill 转发（逻辑相同）
e:/miniforge3/envs/unilab/python skills/domain-owned-arm-rail/scripts/migrate_domain_arm.py \
  --domain <领域仓根> \
  --device <机械臂 @device id> \
  --apply
```

| 参数 | 说明 |
|------|------|
| 无 `--apply` | 仅预览 JSON，**不算移植** |
| `--apply` | 写入领域仓并跑门禁 |
| `--force` | 覆盖已有 scaffold（慎用） |
| exit 0 + `"ok": true` | 脚本侧完成；仍需 OS 冷启动 + RViz 验收 |

### `--apply` 执行顺序（缺任一步 = 未完成）

1. `fix_device_joint_names.py` — 剔除 `{device}_{slug}_joint_*`
2. `scaffold_domain_robot_module.py` — `robot_module` / `robot_factory`
3. `fix_runtime_manifest.py` — manifest → `robot_module`
4. `scaffold_domain_adapters.py` — MoveIt commissioning adapters
5. `fix_moveit_runtime_attach.py` — catalog → 领域模块；写对 `model_ref`
6. 四门禁：`check_domain_robot_module` / 关节对齐 / attach 合同 / 整机装配

### 读懂 migrate JSON 输出

失败时看 `steps[].result.errors`（或 `steps.verify.result.errors`）。常见 step 名：

| step | 含义 |
|------|------|
| `fix_device_joint_names` | 关节名仍含型号 slug |
| `scaffold` / `fix_manifest` / `scaffold_adapters` | 模块或 manifest 生成失败 |
| `fix_moveit_runtime_attach` | attach 链 / `model_ref` 未对齐 |
| `verify` | `robot_module` import 或 Module API v1 失败 |
| `verify_moveit_joint_alignment` | canonical 不是 `joint_N` |
| `verify_moveit_attach_contract` | ToolContext / PointSet attach 合同违规 |
| `verify_domain_arm_assembly` | 整机导轨 + 臂装配检查失败 |

---

## 阶段清单（人工 + 脚本）

### 阶段 0：范围

- 确认目标 `@device` id 与本 PR **不会误改** 的 catalog 设备列表。
- 建议维护 `devices/ROBOT_MODEL_REGISTRY.md` 记录各臂 `model_ownership`。

### 阶段 1：资产落盘（vendor 只读拷贝）

1. 在 `model.yaml` 写清 `source.repository`、`source.path`、`source.sha256`。
2. 从厂商 **精确版本路径** 整文件拷贝 URDF 与配套 STL。
3. 本地 SHA256 必须等于上游与 `model.yaml`：

```bash
python -c "
import hashlib
from pathlib import Path
p = Path('devices/<robot_device>/models/<vendor>.urdf')
print(hashlib.sha256(p.read_bytes()).hexdigest())
"
```

**禁止**：手改 URDF 后再算 digest 当作 vendor 验收。

### 阶段 2：`moveit_model.py`

- 只改 **Python 映射**（`link_names` / `joint_names`），**不改** vendor URDF 几何。
- key 必须来自 **当前** URDF 原名；结构可参考 `unilab_arm_cr7` 的 Python 包装，**禁止**照搬 CR7 映射表到新型号。

### 阶段 3：provider 与 manifest

```python
model={
    "type": "package_moveit",
    "model_ownership": "domain",
    "provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model",
    "source_digest": "<model.yaml source.sha256>",
}
```

### 阶段 4：静态门禁

```bash
pytest devices/<robot_device>/tests/ -q

e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/check_moveit_joint_alignment.py \
  --domain <领域仓根> --device <robot_device>

e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/check_moveit_attach_contract.py \
  --domain <领域仓根> --device <robot_device>

e:/miniforge3/envs/unilab/python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py \
  --domain <领域仓根>
```

以上 **exit 0** 才能进入运行时验收。

### 阶段 5：OS + RViz + 夹爪 attach

#### 5.1 冷启动（必做）

1. **完全停止** 旧 OS / Workbench / RViz / move_group。
2. 删除该 lab 的运行时缓存（常见路径：`.unilabos/runtime/` 下 `ros2_controllers.yaml`、`moveit_controllers.yaml` 或整目录）。
3. **冷启动**，不要热重载。

#### 5.2 本地调试启动（Selective 线 + RViz）

当前 SZLab 调试默认用 selective worktree：

| 组件 | 路径 |
|------|------|
| 领域仓（Workspace） | `F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab` |
| OS（Selective） | `F:/GitHub/new/_merge-worktrees/os-selective-20260907` |
| FE Workbench（Selective） | `F:/GitHub/new/_merge-worktrees/fe-selective-20260907/apps/workbench` |

**Workbench 桌面版**（在 FE workbench 目录）：

```bash
node scripts/start-workbench.mjs --desktop \
  --workspace F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab \
  --os-project F:/GitHub/new/_merge-worktrees/os-selective-20260907 \
  --python-env e:/miniforge3/envs/unilab \
  --port 3100
```

启动前先停掉已有 Workbench / Electron / 3100 端口占用。

**RViz**：Edge 启动计划默认带 `--visual rviz`（`Uni-Lab-OS/unilabos/workspace_host/launch.py`）。若 RViz 空白，先确认 OS edge 已起来、`robot_state_publisher` 与 `move_group` 存活，再查 TF 树（见故障速查）。

#### 5.3 全零位门禁

`/joint_states` 全 0 时，相邻 link 的 STL **必须视觉接上**。断节 → 先 diff 上游 URDF，**不要**先改 home / PointSet。

#### 5.4 夹爪 attach 门禁

- MoveIt attach 父 link = 当前 `end_effector_name`（运行时解析，**禁止**写死 `*_cr7_tool0`）
- ToolContext 用 `moveit.end_effector` / `moveit.end_effector_parent`
- PlanningScene 回读含 attached body
- RViz 无 `Unable to attach a body to link`；夹爪 STL 可见
- 夹爪 STL 仍由 `attachment_profile` / ToolContext 指定，**禁止**为解决 attach 把夹爪并入 vendor URDF

### 阶段 6：前端（FE）

RViz 是权威。3D 位姿对齐见 `.cursor/skills/fe-must-match-rviz/SKILL.md` — **只改 FE，不改 RViz / ToolContext / collision_asset_pose**。

---

## 关节命名（硬规则）

```
vendor URDF          canonical (model.yaml)     qualified (MoveIt / /joint_states)
───────────          ──────────────────────     ──────────────────────────────────
joint1      ──map──► joint_1          ──prefix──► {device_id}_joint_1
joint2      ──map──► joint_2          ──prefix──► {device_id}_joint_2
…                    …                              …
```

| 层 | 格式 | 说明 |
|----|------|------|
| canonical | `joint_1` … `joint_6` | **不含**型号 slug |
| qualified | `{device_id}_joint_{n}` | **只**绑 device_id |

**禁止**：`cr7_joint_1`、`{device}_cr30h_joint_1` 等含 slug 的关节名。

单独修复（migrate 已包含，也可单跑）：

```bash
e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/fix_device_joint_names.py \
  --domain <领域仓根> --device <robot_device> --apply
```

---

## `model_ref`（RViz 夹爪 attach 关键）

| 字段 | 合法格式 |
|------|----------|
| PointSet `components.arm.model_ref` | `package://{domain_pkg}/devices/{device_id}/models/model.yaml` |
| `moveit_model.py` → `MODEL_DESCRIPTOR.model_ref` | 与 PointSet **逐字相同** |

**禁止**把 ROS share 前缀写进 `model_ref`（例如 `package://szlab/szlab_poly_studio/...`）——那是 mesh / ToolContext 路径，不是 arm 型号身份。

**RViz 无夹爪、FE 正常** → 单独跑：

```bash
e:/miniforge3/envs/unilab/python domain-owned-arm-rail-skill/scripts/fix_moveit_runtime_attach.py \
  --domain <领域仓根> --device <robot_device> --apply
```

或重新 `migrate_domain_arm.py --apply`。**不要**手改 PointSet 凑合。

---

## 脚本索引

| 脚本 | 用途 |
|------|------|
| `migrate_domain_arm.py` | **一键入口**（预览 / `--apply`） |
| `fix_device_joint_names.py` | 剔除 slug 关节名 |
| `fix_moveit_runtime_attach.py` | attach 链 + `model_ref` |
| `scaffold_domain_robot_module.py` | 生成 `robot_module` |
| `fix_runtime_manifest.py` | manifest 指向 `robot_module` |
| `scaffold_domain_adapters.py` | MoveIt adapters |
| `check_moveit_joint_alignment.py` | 关节命名门禁 |
| `check_moveit_attach_contract.py` | attach / ToolContext 合同门禁 |
| `check_domain_robot_module.py` | Module API v1 import 门禁 |

路径前缀：`skills/domain-owned-arm-rail/scripts/`（模板仓内）或仓根 `domain-owned-arm-rail-skill/scripts/`。

---

## 故障速查

| 现象 | 处理 |
|------|------|
| RViz 无夹爪，FE 正常 | 查 `model_ref`；`fix_moveit_runtime_attach.py --apply` |
| `components.arm.model_ref 与精确 Arm 型号不一致` | 同上 |
| move_group：`Joint '{device}_*_joint_N' not found`（多种 slug 同时出现） | 停 OS → 清 `.unilabos/runtime/` → 冷启动 → `check_moveit_joint_alignment.py` |
| `ConnectivityException`：TF 多棵树 / `world` 连不上设备 link | 查物理图 parent、静态布局 `root_link` 撞名（如 `{id}_rail_base`）；跑 `check_domain_arm_assembly.py`；确认 `robot_state_publisher` 已发布完整树 |
| RViz 只有 TF、无 STL | 迁移未完成、OS 未冷启动、或 `move_group` 已崩溃退出 |
| 全关节 0 仍 mesh 断缝 / 浮空 | 重拷 vendor URDF；diff 上游 joint origin/axis |
| host_executor_thread + TF lookup 崩溃 | 多为 TF 树未连通；先修装配与 RSP，勿在 OS 侧硬 bypass |
| FE 与 RViz 不一致 | `fe-must-match-rviz`，只调 FE |
| migrate exit 非 0 | 读 JSON `steps.*.result.errors`，修完再跑直到 `ok: true` |

更完整对照：`domain-owned-arm-rail-skill/references/troubleshooting.md`

---

## 禁止事项（摘要）

- 编辑 vendor URDF 的 joint `<origin>` / `<axis>` / link 名
- 用 CR7（或其它型号）关节模板手搓新型号 URDF
- 改 URDF 后重算 digest 冒充 vendor 验收
- 全 0 位断节却去改 home / PointSet / mapping
- 在 ToolContext / attach 里写死型号限定末端 link
- 迁 A 臂时改动 B 臂的 catalog 配置
- 用 `--action_mode simulate` 冒充 MoveIt 验收

---

## 完成标准

- [ ] vendor URDF 与上游字节一致（或书面记录 fork 原因）
- [ ] `migrate_domain_arm.py --apply` exit 0
- [ ] 四门禁脚本 + `check_domain_arm_assembly.py` exit 0
- [ ] 冷启动 OS 后 RViz：机械臂 mesh 正常、全 0 位连续
- [ ] 夹爪 attach 成功，PlanningScene 与 RViz 一致
- [ ] FE 3D 与 RViz 对齐

---

## 相关路径

| 路径 | 内容 |
|------|------|
| `skills/domain-owned-arm-rail/` | Agent 入口 skill（模板仓） |
| `domain-owned-arm-rail-skill/` | 仓根权威实现（须与上表镜像同步） |
| `skills/use-unilab-arm-package/` | catalog 臂 / 导轨装配 |
| `Uni-Lab-pTLC` | catalog 装配权威样本 |
