# 领域自有机械臂 / 导轨：完整迁移 runbook（Agent 写代码版）

从 catalog（`unilab_arm_*`）或空白，**一次性**迁到领域 L3 自有型号并在 RViz + runtime + FE 全部可用。

`<robot_device>`、`<vendor>.urdf>`、`<mesh_slug>`、`<slug>` 从用户领域仓**磁盘与 @device 声明**读取，禁止写死厂商或实验室。

**Agent：每阶段有「交付物」列；全部打勾才能说「移植完成」。**

## vendor URDF 铁律（贯穿阶段 1–5）

- `models/<vendor>.urdf` = 厂商 **只读拷贝**，**禁止**手改 `<origin>` / `<axis>` / link 名  
- **禁止**按 CR7（或其它型号）关节习惯「生成」新型号 URDF  
- `source.sha256` 必须等于 **上游 vendor 文件** SHA256（改完再算 = 无效）  
- 全关节 **0** 时 RViz 仍断节 → **先** diff URDF vs 上游，**不要**先改 home / mapping  

细则：[`asset-layout-checklist.md`](asset-layout-checklist.md)

---

## 阶段 0：范围确认

**读**：`use-unilab-arm-package` 不变量（两 device、图 parent/child、禁止合一体 MoveIt）。

**确认**：

- [ ] 机械臂 `@device` id = 物理图 child
- [ ] 导轨 `@device` id = 物理图 parent
- [ ] 机械臂走 `robot_template` / `moveit_runtime_assembly`（若是，阶段 2.2 **强制**）

**交付物**：写下 `<domain_pkg>`、`<robot_device>`、`<rail_device>`、`<slug>`（从 `model.yaml` / URDF 读）。

---

## 阶段 1：资产落盘（vendor 只读拷贝）

路径：`devices/<robot_device>/models/`

```text
model.yaml
<vendor>.urdf
meshes/<mesh_slug>/*.STL
```

**做**：

1. 从 `model.yaml` → `source.repository` + `source.path`（或 template inventory）**原样复制** URDF → `models/<vendor>.urdf`  
2. **同包/同版本**复制 mesh → `meshes/<mesh_slug>/`  
3. 对 **上游 vendor 文件** 算 SHA256 → `model.yaml` → `source.sha256`（禁止改 URDF 后再算）  
4. 门禁：本地 URDF 字节 SHA256 = 上游官方同路径文件  
5. `kinematic_joints` / `moveit_group` / `model_id` 与新型号一致  
6. **禁止** `model.yaml` 在 device 根目录  

**禁止**：手搓 URDF；用 CR7 joint rpy/axis 模板写 CR30H；digest 只锁本地改过的文件。

**交付物**：

- [ ] URDF + mesh 同源 vendor 包  
- [ ] 本地 URDF 与上游 **字节一致**（或书面记录 fork）  
- [ ] `source.sha256` = `@device source_digest` = `expected_source_digest`  
- [ ] 三件套在磁盘上存在  

---

## 阶段 2：MoveIt 型号 + Module API（同 PR）

### 2.1 `moveit_model.py`

**关节名铁律（与型号 slug 解耦）**

| 层 | 必须 |
|---|---|
| `model.yaml` → `kinematic_joints` | `joint_1` … `joint_6`（**无** `cr7`/`cr16` 等型号前缀） |
| MoveIt qualified | `{device_id}_joint_{n}`，由 `JointStateNameMap` 自动加 device 前缀 |
| 两台相同型号臂 | canonical 相同；qualified 靠不同 `device_id` 区分，**不会冲突** |

1. 只读 `models/<vendor>.urdf`，列出全部 link / joint 名
2. `SixAxisArmModelSpec.link_names` / `joint_names`：**key = URDF 原名**，value = canonical（旋转轴 → `joint_1`…`joint_6`）
3. `canonical_joint_names` = `model.yaml` → `kinematic_joints`（**逐字一致**）
4. `model_slug`、`flange_frame`、`last_link`、`mesh_paths`、`disabled_collisions` 按**当前** URDF 重填
5. 实现 `build_moveit_model`、`build_joint_state_name_map`、`MODEL_DESCRIPTOR`
6. **`MODEL_DESCRIPTOR.model_ref`** = `package://{domain_pkg}/devices/{robot_device}/models/model.yaml`
   （与 PointSet `components.arm.model_ref` **逐字相同**）

**禁止**：

- `model_ref` 写成 `package://szlab/{domain_pkg}/...` 等 ROS share 前缀（那是 mesh URI，不是型号身份）

- 只换 URDF 路径，映射表仍来自另一型号（如 CR7 表配 CR30H URDF）  
- 为「修 RViz」去改 vendor URDF（应在阶段 1 重拷上游）  
- 把 `unilab_arm_cr7` 的 URDF 几何当模板  

细则：[`moveit-model-thin-wrapper.md`](moveit-model-thin-wrapper.md)

### 2.2 `robot_module.py`（**新建，强制**）

走 `robot_template` / `unilab_robot_runtime` 的领域臂**没有此文件就不能移植完成**。

**按 [`code-scaffold.md`](code-scaffold.md) 或 CLI 写入：**

```bash
python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <robot_device> --apply
```

或分步（与 `migrate_domain_arm.py --apply` 等价）：

```bash
python domain-owned-arm-rail-skill/scripts/scaffold_domain_robot_module.py \
  --domain <领域仓根> --device <robot_device> --apply
python domain-owned-arm-rail-skill/scripts/fix_runtime_manifest.py \
  --domain <领域仓根> --device <robot_device> --apply
python domain-owned-arm-rail-skill/scripts/scaffold_domain_adapters.py \
  --domain <领域仓根> --device <robot_device> --apply
python domain-owned-arm-rail-skill/scripts/fix_moveit_runtime_attach.py \
  --domain <领域仓根> --device <robot_device> --apply
```

细则：[`module-api-v1-arm.md`](module-api-v1-arm.md)

**交付物**：

- [ ] `robot_module.py` 在磁盘上存在
- [ ] 上述 `python -c` 通过

### 2.3 型号符号全仓对齐（换 slug / URDF 时**强制**）

在**整个领域仓** grep 旧 slug（例：`cr7`、`unilab_arm_cr7`、旧 revision）。

**必须同步**（与 `model.yaml` → `kinematic_joints` / `moveit_group` 一致）：

| 类型 | 典型路径 |
|---|---|
| 点位 PointSet v3 | `deployment/robot_cell/assets/point_sets/*.yaml` |
| 标定 | `deployment/robot_cell/assets/calibrations/*.yaml` |
| runtime 硬编码路径 | `moveit_runtime_assembly.py`、`*commissioning*.py` |
| HOME / IK | `robot_cell/*_site_ik.py`、`*_tcp_policy.py` |
| 卡片 revision | `frontend/cards/*/`、`CARD_SPEC.md` |

**规则**：

- canonical 关节名 **只能是** `joint_1..joint_6`（见 [`joint-naming.md`](joint-naming.md)）
- 点位 / HOME / 卡片里的 `joint_ref` 与 `cr7_joint_*` 等旧写法必须整批改为 `joint_*`
- `fix_device_joint_names.py --apply` 只改 `devices/<robot>/`；领域仓其它路径须手动或另 PR 同步

**禁止**：在关节名（含 qualified）里保留任何型号 slug（`cr7_joint_*`、`{device}_cr7_joint_*`）。

**交付物**：

- [ ] grep 旧 slug 无「型号体与符号不一致」的残留（或用户书面确认债务）

---

## 阶段 3：provider 与 runtime manifest（与阶段 2 同 PR）

### 3.1 `@device` provider（通常已存在）

```python
model={
    "type": "package_moveit",
    "provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model",
    "source_digest": "<models/model.yaml source.sha256>",
}
```

**不要**把 provider 改成 `robot_module`。

### 3.2 runtime manifest（**必改代码**）

grep 领域仓：`_ModuleRef`、`python_package`、`moveit_runtime_assembly`。

**改** `arm=_ModuleRef(...)` 的最后一项：

```python
# 错（会导致「不支持 Robot Module API v1」）
"...moveit_model"

# 对
"<domain_pkg>.devices.<robot_device>.robot_module"
```

**改 MoveIt attach 执行链**（`fix_moveit_runtime_attach.py --apply`）：

| 残留 | 应改为 |
|---|---|
| `from unilab_arm_cr7 import ...` | `from <domain_pkg>.devices.<robot>.robot_module import ...` |
| `arm_endpoint = f"moveit:{normalized_device}:cr7"` | `...:{model_slug}`（从 `moveit_model.py` 读） |
| manifest `distribution` = `unilab-arm-cr7` | 领域包 distribution（`<domain_pkg>-<device>`） |

同时核对：

| 项 | 要求 |
|---|---|
| `distribution` / `version` | 与领域 Python 包一致 |
| `qualified_joint_names` | = `model.yaml` → `kinematic_joints` 加 device 前缀 |
| ToolContext / `attachment_profile` | `moveit.end_effector` → `{device_id}_<slug>_tool0` |
| `moveit_commissioning.py` | 不再 import `unilab_arm_*`；endpoint 后缀 = 当前 `model_slug` |

**交付物**：

- [ ] manifest 里 `arm.python_package` 指向 `robot_module`
- [ ] grep 无「manifest 仍指 moveit_model 作 arm 模块」

---

## 阶段 4：测试与检查器

**新建 / 更新**：

- `tests/test_moveit_model.py` — `build_moveit_model(device_id=...)`
- `tests/test_robot_module.py` — Module API 常量 + `create_arm_module` 可 import

**运行**（loop 直到全绿）：

```bash
pytest <domain_pkg>/devices/<robot_device>/tests/ -q
python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain <领域根>
```

**交付物**：

- [ ] pytest 通过
- [ ] checker 无 error

---

## 阶段 5：OS + RViz + Edge runtime（loop）

1. 重启 OS（真实 ROS2 后端，非 browser mock）
2. 查 Edge 日志（`*-edge.log`）

### 5.1 全零位 mesh 连续（硬门禁）

`/joint_states` **全部为 0** 时：

- [ ] 相邻 link STL **无断缝/浮空**（肩、底座优先看）  
- [ ] 若断节 → **阶段 1**：diff 本地 URDF vs 上游（joint1–3 `origin`/`axis`），重拷 vendor 文件  
- [ ] **禁止**未查 URDF 就改 home / PointSet / mapping  

### 5.2 常规

- [ ] Edge **无** `不支持 Robot Module API v1`
- [ ] Edge **无** `初始工具附着延迟发布失败，统一运行时已关闭`
- [ ] RViz 有 STL
- [ ] 臂基座在导轨 **carriage（滑座）** 上
- [ ] 夹爪 attach 可见
- [ ] `move_group` 存活

未满足 → 全 0 断节回阶段 **1**；其余回阶段 2–4。**禁止**未过 5.1 进入阶段 6 或报完成。

---

## 阶段 6：FE（完整迁移**必做**）

RViz 通过后 **仍须改 FE**，否则 3D 与 commissioning 不一致。

1. **运动学**：OS 写 `rendering.parent_link` → `{rail_id}_rail_carriage`；FE `projectPlacement` 须跟 parent，**禁止**臂长期挂 `__root__` 导致滑座动、臂不动  
   - `uni-lab-fe/packages/services/src/materialBackendGraphCodec.ts`  
   - `uni-lab-fe/packages/pascal-lab-plugin/src/materialPlacementProjection.ts`
2. **视觉**：读并执行 `.cursor/skills/fe-must-match-rviz/SKILL.md`（夹爪 xacro、`collision_asset_pose`、禁止为对齐 FE 改 RViz）

**验证**：Workbench **桌面版** — 拖导轨关节 → 臂基座跟动；夹爪与 RViz 一致。

**交付物**：

- [ ] FE PR 或同任务内 FE 改动已合并
- [ ] Workbench 3D 与 RViz 对齐

---

## 移植完成：总验收表

| # | 验收项 | 通过 |
|---|---|---|
| 1 | vendor URDF 与上游字节一致；digest 对上游 | ☐ |
| 2 | 全 0 位 RViz mesh 连续 | ☐ |
| 3 | `robot_module.py` 存在且 Module API v1 门禁通过 | ☐ |
| 4 | manifest `arm` → `robot_module` | ☐ |
| 5 | 型号符号（点位/HOME/revision）与 `model.yaml` 一致 | ☐ |
| 6 | pytest + checker 绿 | ☐ |
| 7 | Edge runtime 日志 + RViz carriage + 夹爪 | ☐ |
| 8 | FE：carriage 跟动 + fe-must-match-rviz | ☐ |

**8 项全 ☐→☑ 才能对用户说「移植完成」。**
+
## MoveIt attach 与可更换夹爪门禁

这一步与机械臂 robot_description 的几何装配不同。夹爪可视化和碰撞几何可以继续是独立 STL；MoveIt 通过 ToolContext / PlanningScene attach 使用同一 STL。两者都必须引用当前机械臂设备的动态末端：

    mount_link_ref: moveit.end_effector
    planning_scene:
      parent_link_ref: moveit.end_effector
      allowed_touch_link_refs:
        - moveit.end_effector
        - moveit.end_effector_parent

禁止在 ToolContext、附着投影、PointSet 生成器或运行时状态中写入 szlab_mixer_robot_cr7_tool0、szlab_mixer_robot_cr30h_tool0 等型号限定 link。package_moveit_client_spec() 从当前 URDF/SRDF 产生 end_effector_name，运行时解析这些符号；若 PlanningScene 日志出现 link not found，移植失败，必须定位生成源，不得把旧名称直接替换成新名称。

夹爪 STL 仍由 attachment_profile.visual_asset / planning_scene.collision_asset_ref 指定，保留其摘要、比例、位姿和可替换能力；禁止为解决 attach 问题把夹爪并入 vendor URDF。

完成阶段 5 前必须同时验证：

- MoveIt attach 请求的父 link 等于当前 end_effector_name；
- PlanningScene 回读包含 attached body；
- RViz 无 Unable to attach a body to link；
- 夹爪 STL 在 RViz 可见；
- ToolContext 的符号引用没有被冻结为某一型号名称。
