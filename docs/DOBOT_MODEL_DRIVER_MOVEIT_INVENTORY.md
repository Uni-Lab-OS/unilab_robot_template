# DOBOT 当前驱动、模型与 MoveIt 资产盘点

盘点日期：2026-07-29

## 1. 范围与证据边界

本报告只检查工作区内 DOBOT 官方仓库的固定提交，不根据产品宣传页或型号名称相似性补充推断：

| 资产 | 固定提交 | 上游 |
|---|---|---|
| `uni-lab-assets` | `ca20b88bc41761b45d398ce5f7b7c5d0500e6da9` | Uni-Lab-OS 组织资产仓库 |
| Python/TCP V4 SDK | `55ec1ec82201aaf2e6d54aa47b6272bbaa14c815` | `Dobot-Arm/TCP-IP-Python-V4`，本地固定在 `feature/v4-optimization` |
| ROS2 V4 | `37730d08b08c74061ae10d4fa5565b4c4c914885` | `Dobot-Arm/DOBOT_6Axis_ROS2_V4` |

对应本地根目录：

- `uni-lab-assets/robots/dobot/python_v4`
- `uni-lab-assets/robots/dobot/ros2_v4`

证据等级定义如下：

- **S — 字符串声明**：型号只出现在 README、协议表或源码注释中。
- **D — 驱动实现**：存在可读的 TCP/ROS 驱动实现；若没有按型号分支，只能证明协议驱动是通用实现。
- **M — 完整静态模型**：有六轴 URDF/xacro、visual/collision/inertial 和实际存在的 base + 6 link mesh。
- **P — 完整 MoveIt 配置包**：有 URDF、SRDF、kinematics、joint limits、controller 配置和 launch 套件。
- **R — 已验证可启动**：在目标 ROS 环境中实际构建并启动，验证 TF、模型、规划和控制器。

本轮达到 D/M/P 的静态证据，**没有任何型号达到 R**。当前检查环境没有 `ros2`、`colcon` 或 `xacro`，也未连接真机或启动 Gazebo。

## 2. 结论摘要

1. 当前有两个驱动实现：Python/TCP V4 SDK 和 ROS2 V4 驱动；两者都是一套通用 V4 TCP 驱动，不是每个机械臂型号一套驱动。
2. ROS2 仓库有 **16 个完整静态模型**和对应的 **16 个 MoveIt 配置包**：
   - CR：CR3、CR5、CR7、CR10、CR12、CR16、CR20、CR30H；
   - CRAF：CR3AF、CR5AF、CR10AF、CR20AF；
   - E：ME6；
   - Nova：Nova2、Nova2s、Nova5。
3. 协议文档出现了 **CR5A**，但仓库中没有 `cr5a` URDF、mesh 或 MoveIt 包。现有 CR5 和 CR5AF 各有独立几何，不能把 pTLC 的 CR5A 自动当作 CR5 或 CR5AF。
4. 16 个 MoveIt 包的文件结构完整，但它们默认使用 `mock_components/GenericSystem`；真机执行另由一个通用 `FollowJointTrajectory → ServoJ` 桥接节点完成。
5. 所有 MoveIt `joint_limits.yaml` 都关闭了六个关节的速度和加速度限制，当前配置不能直接作为生产运动限制依据。
6. Gazebo 接线覆盖 16 型，但仍存在环境变量、文件名、控制器启动和限位配置问题，不能据此宣称逐型可启动。

## 3. Python/TCP V4 驱动

### 3.1 是否按型号区分

**不按型号区分。**

`DobotRobot` 构造参数只有 IP、Dashboard/feedback 端口和超时，没有 `model` 或型号选择参数：

- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/api/robot.py`
- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/protocol/feedback.py`
- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/protocol/data_types.py`

同一个对象通过 TCP 29999 发送控制命令，通过 30004/30005/30006 接收反馈。反馈中的 `CRRobotType` 被解析为整数 `robot_type`，但 SDK 没有根据该值选择不同的运动学模型或不同驱动类。因此它是 **D 级通用协议驱动**，不是逐型号驱动。

SDK 版本为 2.0.0：

- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/version.py`
- `uni-lab-assets/robots/dobot/python_v4/pyproject.toml`

### 3.2 README 声明的覆盖范围

本地固定提交的 README 声明：

| 系列 | README 中的型号写法 | 证据等级 |
|---|---|---|
| CRA | CR3A、CR5、CR10、CR16 等 | S + 通用 D |
| E6 | E6、E6 Pro 等 | S + 通用 D |
| CRAF | CRAF5 等 | S + 通用 D |
| NovaLite | NovaLite 等 | S + 通用 D |
| 其他 | 支持 TCP/IP V4 协议的机器人 | S + 通用 D |

证据：

- `uni-lab-assets/robots/dobot/python_v4/README.zh.md`
- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/__init__.py`
- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/version.py`

这里的“支持”表示共用 V4 TCP API，不表示仓库包含这些型号的 URDF、运动学或 MoveIt 配置。

### 3.3 型号特有 API

通用 SDK 中存在少量“仅适用某系列”的 API，例如力传感器碰撞检测注释列出 CR5AF、CR10AF、CR20AF，但仍使用同一个 `Plugins` 类，没有独立型号驱动：

- `uni-lab-assets/robots/dobot/python_v4/dobot_sdk/api/plugins.py`

这类注释只能作为 S 级适用范围证据；调用前仍需根据控制器返回型号和固件能力显式校验。

## 4. ROS2 真机驱动

### 4.1 驱动结构

ROS2 侧同样只有一套通用驱动包：

- package：`cr_robot_ros2`
- executable：`cr_robot_ros2_node`
- TCP：29999 + 30004
- launch：`uni-lab-assets/robots/dobot/ros2_v4/dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py`
- 实现：
  - `uni-lab-assets/robots/dobot/ros2_v4/dobot_bringup_v4/src/main.cpp`
  - `uni-lab-assets/robots/dobot/ros2_v4/dobot_bringup_v4/src/cr_robot_ros2.cpp`
  - `uni-lab-assets/robots/dobot/ros2_v4/dobot_bringup_v4/src/command.cpp`

它暴露通用 Dashboard/运动/IO ROS services，并发布：

- `joint_states_robot`
- `dobot_msgs_v4/msg/RobotStatus`
- `dobot_msgs_v4/msg/ToolVectorActual`

### 4.2 型号参数的实际作用

节点声明 `robot_type`，默认值为 `cr5`。但是 `CRRobotRos2::init()` 读取后只记录日志，没有型号 allow-list，也没有根据型号选择数据结构或协议分支：

- `uni-lab-assets/robots/dobot/ros2_v4/dobot_bringup_v4/src/cr_robot_ros2.cpp`

`main.cpp` 读取环境变量 `DOBOT_TYPE`，用于拼接计划中的 controller action 名称；当前拼出的局部变量并未用于创建驱动实例。实际 MoveIt action server 在 Python 桥接节点中再次读取 `DOBOT_TYPE`。

因此应表述为：

> ROS2 README 声明支持 16 型；真机驱动本身是通用 V4 TCP 驱动，型号字符串主要用于选择模型包和命名 controller，而不是选择不同的底层驱动。

### 4.3 README 声明的真机覆盖范围

ROS2 README 的支持型号表列出：

- CR3、CR5、CR7、CR10、CR12、CR16、CR20、CR30H；
- CR3AF、CR5AF、CR10AF、CR20AF；
- E6 / ME6；
- Nova2、Nova5、Nova2s。

证据：`uni-lab-assets/robots/dobot/ros2_v4/README_ZH.md`。

其中 E6 / ME6 的等价关系只有 README 声明；本地实际模型、mesh 和 MoveIt 包都只使用 `me6` 名称。

## 5. 型号、模型、MoveIt 与 Gazebo 总表

下表中的“完整”是 M/P 静态结论，不是 R 级启动结论。每个 `cra_description` mesh 目录均包含 7 个 STL；每个模型有 6 个 revolute joint。

| 系列 | 型号 | `DOBOT_TYPE`/包前缀 | description/xacro + mesh | RViz 原始 URDF | MoveIt 包 | Gazebo 接线 |
|---|---|---|---|---|---|---|
| CR | CR3 | `cr3` | `ros2_v4/cra_description/urdf/cr3_robot.xacro` + `meshes/cr3/` | `ros2_v4/dobot_rviz/urdf/cr3_robot.urdf` | `ros2_v4/cr3_moveit/` | 有 |
| CR | CR5 | `cr5` | `ros2_v4/cra_description/urdf/cr5_robot.xacro` + `meshes/cr5/` | `ros2_v4/dobot_rviz/urdf/cr5_robot.urdf` | `ros2_v4/cr5_moveit/` | 有 |
| CR | CR7 | `cr7` | `ros2_v4/cra_description/urdf/cr7_robot.xacro` + `meshes/cr7/` | `ros2_v4/dobot_rviz/urdf/cr7_robot.urdf` | `ros2_v4/cr7_moveit/` | 有 |
| CR | CR10 | `cr10` | `ros2_v4/cra_description/urdf/cr10_robot.xacro` + `meshes/cr10/` | `ros2_v4/dobot_rviz/urdf/cr10_robot.urdf` | `ros2_v4/cr10_moveit/` | 有 |
| CR | CR12 | `cr12` | `ros2_v4/cra_description/urdf/cr12_robot.xacro` + `meshes/cr12/` | `ros2_v4/dobot_rviz/urdf/cr12_robot.urdf` | `ros2_v4/cr12_moveit/` | 有 |
| CR | CR16 | `cr16` | `ros2_v4/cra_description/urdf/cr16_robot.xacro` + `meshes/cr16/` | `ros2_v4/dobot_rviz/urdf/cr16_robot.urdf` | `ros2_v4/cr16_moveit/` | 有 |
| CR | CR20 | `cr20` | `ros2_v4/cra_description/urdf/cr20_robot.xacro` + `meshes/cr20/` | `ros2_v4/dobot_rviz/urdf/cr20_robot.urdf` | `ros2_v4/cr20_moveit/` | 有 |
| CR | CR30H | `cr30h` | `ros2_v4/cra_description/urdf/cr30h_robot.xacro` + `meshes/cr30h/` | `ros2_v4/dobot_rviz/urdf/cr30h_robot.urdf` | `ros2_v4/cr30h_moveit/` | 有 |
| CRAF | CR3AF | `cr3af` | `ros2_v4/cra_description/urdf/cr3af_robot.xacro` + `meshes/cr3af/` | `ros2_v4/dobot_rviz/urdf/cr3af_urdf.urdf` | `ros2_v4/cr3af_moveit/` | 有 |
| CRAF | CR5AF | `cr5af` | `ros2_v4/cra_description/urdf/cr5af_robot.xacro` + `meshes/cr5af/` | `ros2_v4/dobot_rviz/urdf/cr5af_urdf.urdf` | `ros2_v4/cr5af_moveit/` | 有 |
| CRAF | CR10AF | `cr10af` | `ros2_v4/cra_description/urdf/cr10af_robot.xacro` + `meshes/cr10af/` | `ros2_v4/dobot_rviz/urdf/cr10af_robot.urdf` | `ros2_v4/cr10af_moveit/` | 有 |
| CRAF | CR20AF | `cr20af` | `ros2_v4/cra_description/urdf/cr20af_robot.xacro` + `meshes/cr20af/` | `ros2_v4/dobot_rviz/urdf/cr20af_urdf.urdf` | `ros2_v4/cr20af_moveit/` | 有 |
| E | ME6 | `me6` | `ros2_v4/cra_description/urdf/me6_robot.xacro` + `meshes/me6/` | `ros2_v4/dobot_rviz/urdf/me6_robot.urdf` | `ros2_v4/me6_moveit/` | 有 |
| Nova | Nova2 | `nova2` | `ros2_v4/cra_description/urdf/nova2_robot.xacro` + `meshes/nova2/` | `ros2_v4/dobot_rviz/urdf/nova2_robot.urdf` | `ros2_v4/nova2_moveit/` | 有 |
| Nova | Nova2s | `nova2s` | `ros2_v4/cra_description/urdf/nova2s_robot.xacro` + `meshes/nova2s/` | `ros2_v4/dobot_rviz/urdf/nova2s_robot.urdf` | `ros2_v4/nova2s_moveit/` | 有 |
| Nova | Nova5 | `nova5` | `ros2_v4/cra_description/urdf/nova5_robot.xacro` + `meshes/nova5/` | `ros2_v4/dobot_rviz/urdf/nova5_robot.urdf` | `ros2_v4/nova5_moveit/` | 有 |

表中 `ros2_v4/...` 均相对于 `uni-lab-assets/robots/dobot/`。

当前固定提交 `37730d0` 的提交说明为 `add: cr3af cr5af cr20af nova2s`。实际目录包含这四个新增型号；README 的“项目结构”代码块没有全部列出它们，但同一 README 的“支持型号”表已列出。因此覆盖判断应以固定提交中的 package/model 文件为准，不能只看 README 目录树。

## 6. Description、URDF 和 mesh

### 6.1 `cra_description`

`cra_description` 对 16 型均提供：

- `<model>_robot.xacro`；
- base + 6 links；
- 6 个 revolute joints；
- inertial、visual 和 collision；
- 7 个 STL mesh；
- Gazebo material；
- `gazebo_ros2_control/GazeboSystem`；
- 对应 `<model>_moveit/config/ros2_controllers.yaml` 引用。

根路径：

- `uni-lab-assets/robots/dobot/ros2_v4/cra_description/urdf`
- `uni-lab-assets/robots/dobot/ros2_v4/cra_description/meshes`

静态检查结果：

- 16/16 xacro XML 结构可解析；
- 共 112 个 STL，16/16 型的 14 个 visual/collision mesh 引用均能解析到实际文件；
- mesh 是实际二进制 STL，不是 Git LFS pointer。

### 6.2 `dobot_rviz`

`dobot_rviz` 另复制了一套 16 型 URDF 和 112 个 STL：

- `uni-lab-assets/robots/dobot/ros2_v4/dobot_rviz/urdf`
- `uni-lab-assets/robots/dobot/ros2_v4/dobot_rviz/meshes`

这套重复资产存在漂移风险，应在 UniLab 适配层选定单一模型权威源。建议首选 `cra_description`，因为 Gazebo 通用 launch 直接使用它。

### 6.3 不应误称完整模型的名称

以下名称只达到 S 级：

- **CR5A**：协议 `RobotType=115`，但无 `cr5a` 模型；
- **CR3A、CR7A、CR10A、CR12A、CR16A、CR20A**：出现在协议 RobotType 表，当前无同名模型/MoveIt package；
- **E6**：README 写 E6 / ME6，但只有 `me6` 资产；
- **NovaLite**：Python README 声明通用协议支持，但 ROS2 仓库无 `novalite` 模型；
- **Magician E6**：协议 RobotType 表的写法，与 ROS 模型 `me6` 命名不一致。

协议型号表证据：

- `uni-lab-assets/robots/dobot/ros2_v4/V4新增指令/README.md`

## 7. MoveIt 覆盖范围

### 7.1 16 个完整配置包

每个表中型号都有一个版本为 0.3.0 的 `<model>_moveit` package。每包静态包含 10 个 config 文件和 10 个 launch 文件，核心内容包括：

- `<model>_robot.urdf.xacro`
- `<model>_robot.ros2_control.xacro`
- `<model>_robot.srdf`
- `kinematics.yaml`
- `joint_limits.yaml`
- `initial_positions.yaml`
- `moveit_controllers.yaml`
- `ros2_controllers.yaml`
- `pilz_cartesian_limits.yaml`
- `moveit.rviz`
- `demo.launch.py`
- `dobot_moveit.launch.py`
- `move_group.launch.py`
- `moveit_gazebo.launch.py`
- `moveit_rviz.launch.py`
- `rsp.launch.py`
- `spawn_controllers.launch.py`

16/16 型的下列名称静态一致：

- URDF robot：`<model>_robot`
- SRDF robot：`<model>_robot`
- planning group：`<model>_group`
- KDL key：`<model>_group`
- MoveIt controller：`<model>_group_controller`
- joints：`joint1` 至 `joint6`

Kinematics 全部使用：

```yaml
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

以首轮重点型号为例：

- CR5：`uni-lab-assets/robots/dobot/ros2_v4/cr5_moveit`
- CR5AF：`uni-lab-assets/robots/dobot/ros2_v4/cr5af_moveit`

**没有 `cr5a_moveit`。**

### 7.2 MoveIt demo 与真机不是同一硬件后端

所有 16 个 `<model>_robot.ros2_control.xacro` 都使用：

```xml
<plugin>mock_components/GenericSystem</plugin>
```

因此 `demo.launch.py` 主要是 fake hardware 规划/显示环境。真机链路不是一个 DOBOT ros2_control hardware plugin，而是：

```text
MoveIt
  → /<model>_group_controller/follow_joint_trajectory
  → dobot_moveit/action_move_server.py
  → /dobot_bringup_ros2/srv/ServoJ
  → 通用 TCP 驱动
```

证据：

- `uni-lab-assets/robots/dobot/ros2_v4/dobot_moveit/dobot_moveit/action_move_server.py`
- `uni-lab-assets/robots/dobot/ros2_v4/dobot_moveit/launch/dobot_joint.launch.py`
- `uni-lab-assets/robots/dobot/ros2_v4/dobot_moveit/launch/dobot_moveit.launch.py`

这说明仓库有 MoveIt 到真机的桥接代码，但不能把“完整 MoveIt 配置包”直接等同于“完整且可靠的真机 MoveIt 驱动”。

### 7.3 运动限制缺口

16/16 个 `joint_limits.yaml` 均对六轴设置：

```yaml
has_velocity_limits: false
max_velocity: 0
has_acceleration_limits: false
max_acceleration: 0
```

虽然全局 scaling factor 是 0.1，但禁用基础速度/加速度约束后，不能把它作为经型号验证的运动限制。生产接入前必须从对应型号手册/控制器导出值，形成版本化的限位和缩放配置。

例：

- `uni-lab-assets/robots/dobot/ros2_v4/cr5_moveit/config/joint_limits.yaml`
- `uni-lab-assets/robots/dobot/ros2_v4/cr5af_moveit/config/joint_limits.yaml`

## 8. Gazebo 与仿真资产

### 8.1 通用入口

Gazebo 使用单一通用包：

- `uni-lab-assets/robots/dobot/ros2_v4/dobot_gazebo/launch/dobot_gazebo.launch.py`
- `uni-lab-assets/robots/dobot/ros2_v4/dobot_gazebo/launch/gazebo_moveit.launch.py`

它读取 `DOBOT_TYPE`，拼出：

```text
cra_description/urdf/<DOBOT_TYPE>_robot.xacro
<DOBOT_TYPE>_group_controller
```

由于 16 型都有对应 xacro 和 controller YAML，静态上覆盖全部 16 型。

### 8.2 仿真能力边界

- `dobot_gazebo.launch.py` 启动 classic Gazebo、robot_state_publisher 和 `spawn_entity.py`，但不激活控制器。
- `gazebo_moveit.launch.py` 在 spawn 后加载 `joint_state_broadcaster` 和 `<model>_group_controller`，但它本身不启动 move_group/RViz。
- `dobot_moveit/launch/moveit_gazebo.launch.py` 只转到型号 MoveIt launch，不启动 Gazebo。
- 实际“Gazebo + MoveIt”需要组合入口；README 中的一条命令不能证明控制闭环完整。
- `worlds/cr.world` 只有 ground plane 和 sun，且当前通用 launch 没有引用这个 world。
- visual 和 collision 使用相同 STL，可用于首轮视觉/碰撞检查，但不是经简化和性能验证的生产碰撞模型。

### 8.3 Gazebo 限位风险

16 型的 Gazebo ros2_control position command interface 都硬编码：

```xml
<param name="min">-1</param>
<param name="max">1</param>
```

这与各型号 URDF 中的关节范围并不等价。仿真验收前必须消除这套硬编码并与型号限位的单一权威数据对齐。

例：

- `uni-lab-assets/robots/dobot/ros2_v4/cra_description/urdf/cr5_robot.xacro`
- `uni-lab-assets/robots/dobot/ros2_v4/cra_description/urdf/cr5af_robot.xacro`

## 9. 明确缺口、命名不一致和启动风险

### 9.1 CR5A 是首轮 blocker

协议文档明确列出：

| RobotType | 型号 |
|---:|---|
| 5 | CR5 |
| 115 | CR5A |

但是本地模型只有：

- `cr5`
- `cr5af`

CR5AF 是一套独立模型和 MoveIt 包，名称和几何证据都不能证明它等于 CR5A。

对 pTLC 的处理要求：

1. 读取真机反馈 `RobotType`；
2. 核对铭牌完整型号、控制器固件、连杆尺寸、法兰和关节限位；
3. 让 DOBOT/集成商书面确认 CR5A 对应哪套官方 description；
4. 在确认前允许复用通用 Python/TCP 协议驱动，但禁止绑定 CR5 或 CR5AF MoveIt 模型进入真机规划。

### 9.2 型号命名至少有四套口径

| 层 | 示例 |
|---|---|
| Python README | CRA、CR3A、CRAF5、NovaLite |
| ROS README/model package | CR5、CR5AF、Nova2、ME6 |
| 协议 RobotType | CR5、CR5A、CR20A、Magician E6 |
| UniLab/pTLC 项目 | DOBOT CR5A |

UniLab 设备配置不应只存自由文本 `model`。建议至少分开：

```yaml
marketing_model: CR5A
controller_robot_type: 115
driver_family: dobot_tcp_v4
description_id: unresolved
moveit_package: unresolved
```

### 9.3 三个 RViz 默认文件名不匹配

通用 RViz launch 固定寻找：

```text
dobot_rviz/urdf/<DOBOT_TYPE>_robot.urdf
```

但以下实际文件为：

- `cr3af_urdf.urdf`
- `cr5af_urdf.urdf`
- `cr20af_urdf.urdf`

因此 CR3AF、CR5AF、CR20AF 直接使用默认 `dobot_rviz.launch.py` 会得到错误路径；CR10AF 文件名正常。

证据：

- `uni-lab-assets/robots/dobot/ros2_v4/dobot_rviz/launch/dobot_rviz.launch.py`
- `uni-lab-assets/robots/dobot/ros2_v4/dobot_rviz/urdf`

### 9.4 通用 launch 依赖未校验的环境变量

以下入口直接读取环境变量，没有 ROS launch argument、choices 或可靠默认值：

- `DOBOT_TYPE`
- `IP_address`

涉及：

- `dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py`
- `dobot_moveit/launch/dobot_moveit.launch.py`
- `dobot_moveit/launch/moveit_demo.launch.py`
- `dobot_gazebo/launch/dobot_gazebo.launch.py`
- `dobot_rviz/launch/dobot_rviz.launch.py`

未设置时会拼出 `None_moveit`、`None_robot.*`，或给节点传入空 IP。README 的示例命令没有先展示环境变量设置，不能按原样作为 UniLabOS 的稳定 launch 合同。

### 9.5 Headless 和依赖声明风险

- 各型号 `dobot_moveit.launch.py` 直接读取 `os.environ["DISPLAY"]`，在无图形环境的 OS 服务中可能抛出 `KeyError`。
- `dobot_moveit/package.xml` 没有声明其 Python 代码实际导入的 `rclpy`、`control_msgs`、`trajectory_msgs`、`sensor_msgs` 和 `dobot_msgs_v4` 运行依赖。
- `cr_robot_ros2/package.xml` 未声明 CMake/源码使用的 `sensor_msgs` 依赖。

这些缺口意味着 `rosdep` 成功不能单独证明工作区可构建或 launch 可运行。

### 9.6 真机 MoveIt 完成语义不足

`action_move_server.py` 对每个轨迹点异步调用 `ServoJ`，没有等待或检查 service response，也没有用 DOBOT `CurrentCommandId` 对账；随后把 `FollowJointTrajectory` goal 标记成功。

因此当前桥接层的“成功”不能作为 UniLab 物料搬运的完成见证。生产接入必须补充：

- service response 检查；
- controller command ID；
- feedback 中的执行终态；
- 轨迹误差/超时；
- cancel、pause、fault 和断线对账。

## 10. 静态验证记录

本轮对固定源代码执行了下列只读检查：

- 16/16 `cra_description` xacro XML 解析成功；
- 16/16 `dobot_rviz` URDF XML 解析成功；
- 16/16 SRDF XML 解析成功；
- 32/32 MoveIt URDF/ros2_control xacro XML 解析成功；
- 16 型 description mesh 引用无缺失；
- 16 型 RViz mesh 引用无缺失；
- 16 个 MoveIt package 的 YAML 均可解析；
- 16 个 MoveIt package 的 URDF/SRDF/group/kinematics/controller 名称静态对齐；
- ROS2 仓库 190 个 Python 文件 AST 语法解析无错误。

未执行：

- `colcon build`
- `ros2 launch`
- xacro 展开后的 URDF 校验
- MoveIt planning smoke test
- Gazebo spawn/controller smoke test
- 真机 TCP/ROS2 联调

因此当前可以对外表述：

> DOBOT 官方本地固定提交具备一套通用 Python/TCP V4 驱动、一套通用 ROS2 V4 驱动，以及 16 型完整静态 description、MoveIt 和 Gazebo 接线；尚未在 UniLab 运行环境逐型验证构建和启动。pTLC 所称 CR5A 没有同名模型资产，必须先解决型号映射，不能直接选择 CR5AF。

## 11. 对第一轮 DOBOT 测试的建议

### 阶段 A：不依赖模型的真机协议测试

使用通用 Python/TCP V4 驱动测试 CR5A：

- 连接、反馈长度、RobotType；
- enable/error/running/safety 状态；
- CurrentCommandId；
- 低速已示教点动作；
- stop/pause/断线对账。

这一阶段不加载 CR5/CR5AF MoveIt 模型。

### 阶段 B：型号身份和模型验收

拿到 CR5A 的官方模型映射或独立模型后验证：

- 零位和六轴正方向；
- 关节上下限、速度、加速度；
- 基座到法兰的 FK；
- 实际 TCP 与模型；
- mesh 尺寸和碰撞几何；
- 同一组关节角下真机与 RViz 姿态。

### 阶段 C：MoveIt/Gazebo 接入

先修复 launch 参数、依赖、joint limits 和完成语义，再进入：

- xacro/URDF smoke test；
- MoveIt fake controller；
- Gazebo controller；
- MoveIt → ServoJ 仿真；
- 隔离单元、低速真机；
- pTLC 物料搬运技能。
