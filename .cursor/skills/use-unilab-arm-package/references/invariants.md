# 领域消费不变量

本文件是机械臂 / 导轨型号包的 **L3 消费合同**。代码标识符保持英文原样。

## 分层

| 层 | 仓 / 包 | 允许拥有 | 禁止拥有 |
|---|---|---|---|
| L1 机械臂 | `unilab-arm-cr5` / `unilab-arm-cr7` | 六轴关节、限位、mesh、`build_moveit_model`、固定 `mount_yaw_joint` | 现场点位、库位（Site）、导轨棱柱轴、RViz 启动 |
| L1 导轨 | `unilab-rail-linear` | 一根棱柱轴、`rail_base` / `rail_carriage`、`build_kinematic_model` | 六轴规划组、ros2_control、把臂当第七轴 |
| L2 | `unilab-rail-mounted-arm` / runtime | 先导轨后臂的协调 | 领域工位编号 |
| L3 领域仓 | pTLC / SZLab / 其它实验室包 | 物理图、静态外壳 STL、点位 digest、HardwareProfile | 改写 L1 URDF 拓扑、合一体 MoveIt |

## `@device` 机械臂

```python
model={
    "type": "package_moveit",
    "provider": "unilab_arm_cr5:build_moveit_model",  # 或 unilab_arm_cr7
    "source_digest": "<model.yaml source.sha256，64 位小写 hex>",
}
```

- `provider` 必须是 `unilab_arm_<slug>:build_moveit_model`。不要写 `unilab_arm_<slug>.moveit_model:build_moveit_model` 以外的私有路径；公开导出以包根 `:` 符号为准（pTLC / CR7 均如此）。
- `source_digest` 必须等于该型号 `models/model.yaml` 的 `source.sha256`。漂移则 OS 启动失败关闭。
- 禁止 `type: xacro` / `workspace_xacro`。禁止模型字典出现 `arm_base_joint`。
- 可选 `format`/`entry`：仅当领域仓**真有**可投影的本地 URDF 时再写；没有本地文件就不要写 `format: urdf` 空入口（否则工作区物料目录编译会要求 POSIX 相对路径）。
- 可选 `mount_yaw_deg`：见「安装偏航」。未设置则机械臂包按 `0.0` 插入固定关节（零偏航），不影响规划组。

MoveIt 规划组只含 `{device_id}_<slug>_joint_1` … `_joint_6`。`mount_yaw_joint` 与 `world_mount_joint` 都是 `fixed`。

## `@device` 导轨

```python
model={
    "type": "package_static",
    "provider": "<领域包>.static_layout:build_<rail>",
    "source_digest": "<外壳 STL 的 SHA-256>",
    "joint_state_provider": "unilab_rail_linear:build_kinematic_model",
    "joint_state_source_digest": "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d",
}
```

`joint_state_source_digest` 必须等于 `unilab-rail-linear` 的 `models/model.yaml` 源摘要（当前实现锁定值见上，漂移以该 yaml 为准）。

静态 Provider 必须：

- 只返回一个 `root_link`，名字为 `{member_id}_base_link`
- 只含 `fixed` 关节或无关节；必须有 `collision`
- **禁止**使用 `{member_id}_rail_base` 作为外壳根（与运动学根撞名）

运动学 Provider 产生：

- `{device_id}_rail_base`
- `{device_id}_rail_carriage`（`mount_link`，子机械臂挂这里）
- `{device_id}_rail_joint`（`prismatic`，完全限定）

OS 把两棵树都并进 `robot_description`。重复 link 名会让 `robot_state_publisher` / `move_group` 解析失败后退出。

## 物理图

- 导轨 `parent` 为空（或场地球）；`children` 含机械臂 id。
- 机械臂 `parent` 必须是导轨 id。
- 导轨 `config.standard_execution_backend` ∈ {`mock`, `simulation`, 领域 PLC 名}，**不是** `moveit` / `moveit_sim`。
- 机械臂 `config.standard_execution_backend`：MoveIt 图用 `moveit` / `moveit_sim`；PLC / TCP 图用领域执行后端。拓扑不随后端改变。
- 仅当机械臂选择 MoveIt 时，OS 才启动 `move_group`（`node_requests_moveit`）。
- 位姿：`position` 毫米；嵌套 `position.rotation` 为度。机械臂相对导轨滑座的偏移写在子节点局部位姿，不要把导轨世界位姿再抄进臂的世界坐标。
- 图 JSON 禁止出现 `arm_base_joint`。

OS 在存在父 `mount_link` 时：用子节点**局部**位姿生成机械臂安装，再把唯一 `world` 固定关节的 parent 改成 `{rail_id}_rail_carriage`。

## 安装偏航

三条来源最多生效一条非零 Z 偏航：

1. 物理图子节点 `position.rotation.z`（度）→ OS 合成进 `world_mount_joint` 的 `rpy`
2. 领域 `@device model.mount_yaw_deg` 或 `config.mount_yaw_deg`（度）→ 机械臂包 `mount_yaw_joint`
3. 已废弃：不要再手写一份 `config.rotation` 指望它叠加；`apply_graph_world_mount` 会覆盖 `config.rotation`

pTLC：只用来源 1（`rotation.z = 180`）。  
若用来源 2，图上该轴必须为 0。

## 前端与执行

- Workbench 3D：Host 投影 `/joint_states`，不订 `/tf`。
- 调试运动：`RobotCommissioningPort` / Host `robotCommissioning`，卡片禁止 `fetch` 文件系统。
- MoveIt 无头：`rviz_required` 必须为 false。RViz 不是完成判定。
- 导轨仿真：领域自己发布完全限定关节名（pTLC `rail_rail_joint`）；不要让机械臂 Adapter 冒充导轨轴。

## 失败信号（修装配时先复现这些）

| 日志 | 含义 |
|---|---|
| `link '…_rail_base' is not unique` | 静态根与运动学根同名 |
| `Failed to parse robot description` + `move_group process has died` | URDF 非法，不是「还在加载网格」 |
| `NO PLANNING LIBRARY LOADED` | 模型已解析但规划器未起来；与撞名不同 |
| 工作区 `工作区模型资产入口必须是非空 POSIX 相对路径` | `format: urdf` 却没有本地 `entry` |
