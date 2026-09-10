---
name: use-unilab-arm-package
description: >-
  Wire Uni-Lab L1 arm and linear-rail packages into a domain repository the pTLC
  way: separate devices, package_moveit vs package_static+joint_state_provider,
  unique URDF links, graph parent=rail, no combined MoveIt, no arm_base_joint.
  Use whenever adding, migrating, reviewing, or debugging a domain 机械臂,
  导轨, MoveIt, package_moveit, package_static, SZLab, pTLC, CR5, CR7, or
  robot_description assembly.
---

# 领域包如何消费机械臂 / 导轨型号包

本技能管 **L3 领域仓怎么引用** `unilab-arm-*` 与 `unilab-rail-linear`。
包装新六轴型号走 `package-arm-moveit`，不要混用。

**权威样本**：`Uni-Lab-pTLC` 的 `ptlc_station/devices/robot.py`、
`ptlc_station/devices/rail.py`、`ptlc_station/devices/static_layout.py`、
`deployment/graphs/local-debug.json`。有冲突时以 pTLC 当前代码为准，不以记忆或 SZLab 旧 xacro 为准。

开始前必须读完：

1. [references/invariants.md](references/invariants.md)
2. [references/ptlc-canonical-assembly.md](references/ptlc-canonical-assembly.md)

然后按下面清单改领域仓。改完立刻跑检查器，红了不准宣称完成。

## 硬禁止

- 把导轨做成机械臂第七轴，或把 `arm_base_joint` 放进 MoveIt 规划组 / SRDF / ros2_control。
- 用 OS 遗留 `unilabos/device_mesh/devices/arm_slider` xacro 当新产品路径。
- 一个 `@device` 同时拥有六轴规划组和导轨棱柱轴。
- 静态外壳 `root_link` 与导轨运动学根同名（典型撞名 `{id}_rail_base` → MoveIt 报 `link is not unique` 后 `move_group` 立刻退出）。
- 把 `mount_yaw_deg=180` 叠在物理图已经是 `rotation.z=180` 的安装上。
- 让前端订阅 `/tf` 驱动机械臂 3D；3D 只跟 Host 投影的 `/joint_states`。
- 用 `--action_mode simulate` 冒充 MoveIt（环境管理 Dry-run 除外）。
- 即兴发明新的中文导轨关节名、新的安装角字段、或第二套 MoveIt 启动路径。
- 工具上下文、MoveIt attach 和夹爪 STL 的挂载父 link 不得写死 CR5/CR7/其它型号；
  必须使用 `moveit.end_effector` / `moveit.end_effector_parent`，由当前设备模型
  的 `end_effector_name` 运行时解析。

## 必做装配（复制 pTLC 结构，只换 id / 型号）

| 角色 | `@device model.type` | Provider | 物理图 |
|---|---|---|---|
| 导轨 | `package_static` | 领域 `static_layout:build_*` + `joint_state_provider: unilab_rail_linear:build_kinematic_model` | 父设备；`standard_execution_backend` 为 `mock` / `simulation` / PLC，**禁止** `moveit` |
| 机械臂 | `package_moveit` | `unilab_arm_<slug>:build_moveit_model` + 锁定 `source_digest` | `parent` = 导轨 id；MoveIt 图用 `moveit` / `moveit_sim`，PLC 图用领域后端 |

静态外壳 `root_link` 必须是 `{member_id}_base_link`。运动学根是 `{device_id}_rail_base`，滑座 `mount_link` 是 `{device_id}_rail_carriage`。两边名字不得相交。

OS 会：把静态树和关节树并入同一 `robot_description`；把子机械臂的唯一 `world` 固定关节改挂到父 `mount_link`；**不会**把导轨棱柱轴并入机械臂 `qualified_joint_names`。

## 安装偏航只选一条来源

物理图局部位姿的 `rotation` 单位是**度**；OS `apply_graph_world_mount` 会把它合成进 Provider 的 `rotation`（弧度）。`mount_yaw_deg` 是机械臂包里额外的 `type=fixed` 关节，运行时不变，不是第七轴。

- pTLC 做法（默认）：图上 `position.rotation.z = 180`，**不要**再设 `mount_yaw_deg`。
- 仅当图上该轴旋转为 0、且领域配置明确拥有固定安装角时，才用 `mount_yaw_deg`。

## 工作流

```
Progress:
- [ ] 1. 读 invariants + pTLC 样本
- [ ] 2. 拆成两个 @device（导轨 / 机械臂）
- [ ] 3. 静态外壳 root_link = {id}_base_link
- [ ] 4. 全部物理图 parent/backend/pose
- [ ] 5. 安装偏航只留一条来源
- [ ] 6. 跑 check_domain_arm_assembly.py，红则修
- [ ] 7. 领域仓回归测试（catalog / graph / unique links）
- [ ] 8. ToolContext / MoveIt attach 动态末端门禁（STL 保持可更换）
```

检查器（从本技能目录或模板仓根执行）：

```bash
python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain <领域仓根>
```

退出码 0 才能把装配任务标完成。pTLC 必须保持绿灯；其他领域仓按同一规则验收。

## 批量复用

对每个领域仓只改：设备 id、class、型号 slug（cr5/cr7）、digest、图上位姿毫米值、以及**一条**安装偏航来源。不要重写 Provider 合同、不要合并 URDF、不要从 SZLab 旧 `models/config/move_group.json` 抄第七轴。
