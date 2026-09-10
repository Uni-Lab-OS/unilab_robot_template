# 消费机械臂 / 导轨型号包（Use Uni-Lab Arm Package）使用说明

领域仓如何按 **pTLC 方式** 引用 template 里的 L1 型号包（`unilab-arm-*`、`unilab-rail-linear`）。适用于 **`model_ownership: catalog`** 或未做 L3 自有 URDF 的机械臂 / 导轨装配。

Agent 细则：`skills/use-unilab-arm-package/SKILL.md`  
权威样本：`Uni-Lab-pTLC`

---

## 何时用本流程

| 场景 | 用哪个 |
|------|--------|
| 直接用 `unilab_arm_cr7` 等 catalog 包 | **本说明** |
| 领域仓自维护 URDF（`devices/.../models/`） | [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md) |
| 把厂商 URDF 首次打成 L1 发行包 | `skills/package-arm-moveit/SKILL.md` |

---

## 必做装配（复制 pTLC，只换 id / 型号）

| 角色 | `@device model.type` | Provider | 物理图 |
|------|---------------------|----------|--------|
| 导轨 | `package_static` | 领域 `static_layout:build_*` + `joint_state_provider: unilab_rail_linear:build_kinematic_model` | 父设备；backend 为 `mock` / `simulation` / PLC，**禁止** `moveit` |
| 机械臂 | `package_moveit` | `unilab_arm_<slug>:build_moveit_model` + 锁定 `source_digest` | `parent` = 导轨 id；MoveIt 图用 `moveit` / `moveit_sim` |

静态外壳 `root_link` = `{member_id}_base_link`。运动学根 = `{device_id}_rail_base`，滑座 `mount_link` = `{device_id}_rail_carriage`。**两边名字不得相交**。

---

## 硬禁止（摘要）

- 导轨做成机械臂第七轴，或 `arm_base_joint` 进 MoveIt 规划组 / SRDF / ros2_control
- 抄 OS 遗留 `arm_slider` xacro
- 一个 `@device` 同时拥有六轴规划组和导轨棱柱轴
- 静态 `root_link` 与导轨运动学根同名（→ `link is not unique`，`move_group` 立刻退出）
- 物理图已有 `rotation.z=180` 时再叠 `mount_yaw_deg=180`
- FE 3D 订阅 `/tf` 驱动机械臂（只跟 Host 投影的 `/joint_states`）
- ToolContext / attach 写死 CR5/CR7 末端 link（必须用 `moveit.end_effector` 符号）

---

## 安装偏航（Mount Yaw）只选一条来源

- **默认（pTLC）**：物理图 `position.rotation.z = 180`，**不设** `mount_yaw_deg`
- 仅当图上该轴为 0 且领域明确拥有固定安装角时，才用 `mount_yaw_deg`

---

## 工作流

```
1. 读 invariants + pTLC 样本
2. 拆成两个 @device（导轨 / 机械臂）
3. 静态外壳 root_link = {id}_base_link
4. 配齐物理图 parent / backend / pose
5. 安装偏航只留一条来源
6. 跑 check_domain_arm_assembly.py
7. 领域仓回归测试
8. ToolContext / MoveIt attach 动态末端门禁
```

---

## 检查器（必跑）

```bash
e:/miniforge3/envs/unilab/python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py \
  --domain <领域仓根>
```

**exit 0** 才能宣称装配完成。pTLC 必须保持绿灯；其它领域仓按同一规则验收。

---

## 批量复用

对每个领域仓只改：设备 id、class、型号 slug（cr5/cr7）、digest、图上位姿毫米值、以及**一条**安装偏航来源。不要重写 Provider 合同、不要合并 URDF、不要从 SZLab 旧 `move_group.json` 抄第七轴。

---

## 相关文档

- [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md) — 领域自有臂完整移植
- `skills/use-unilab-arm-package/references/invariants.md` — 不变量全文
- `skills/use-unilab-arm-package/references/ptlc-canonical-assembly.md` — pTLC 装配细节
