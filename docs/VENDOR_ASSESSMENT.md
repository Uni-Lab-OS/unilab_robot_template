# 厂家资产评估与首批选择

## 1. 本地证据范围

`uni-lab-assets/robots` 中的厂家仓库以 git submodule 固定。此次初始化并检查了 7 个国产候选的 Python/ROS 2 仓库，以下提交是评估快照，不表示未来版本自动兼容：

| 厂家 | Python/API 固定提交 | ROS 2 固定提交 | 仿真/规划资产 | 主要约束 |
|---|---|---|---|---|
| Elite | `451631a` | `3e9e8ca` | description、MoveIt、Gazebo、新旧 fake hardware | Python SDK 需 pybind/CMake；生产成功需额外完成见证 |
| FAIRINO | `563ae32` | `60755d4` | description、多个 MoveIt 配置 | ROS 包按控制器版本拆分较多，部分 manifest 许可证仍为 TODO |
| JAKA | `76d48ca` | `aadcf50` | description、大量 MoveIt 配置 | Python SDK 更新较慢，部分 ROS manifest 许可证为 TODO |
| Dobot | `55ec1ec` | `37730d0` | MoveIt、Gazebo | 根许可证清楚，但部分 ROS package manifest 元数据仍为 TODO |
| RealMan | `9d75cc9` | `c941b56` | description、MoveIt、Gazebo | API2 语言绑定/部署结构较重，需单独完成许可证审计 |
| ROKAE | `a135059` | `d6a508c` | description、MoveIt、Gazebo | 能力完整，但当前 UniLabOS 没有既有落地点 |
| UFACTORY | `d911319` | `d0b9511` | description、MoveIt、Gazebo、双臂、移动底盘示例 | ROS/SDK API 面很大；部分 ROS 子包 manifest 许可证仍为 TODO |

所有结论均来自本地固定提交的 README、package manifest、消息/服务定义和示例，而不是仅看目录名。

## 2. 第一轮现场验证：DOBOT CR5A

DOBOT 被提升为第一轮现场验证厂家，原因不是其 ROS 接口最完整，而是 pTLC 已有
CR5A 上位机直连、点位序列、夹爪语义和滑轨/工站流程，可直接验证迁移等价性。

第一轮采用：

- Python V4 TCP/SDK：29999 命令、30004 feedback；
- pTLC 四个具名 `robot_group_*` 技能；
- host runner 逐条校验 DOBOT `CurrentCommandId`，上层再形成 OS `command_id` 终态见证；
- 原点/锚点检查、工具动作 allowlist、断线 `UNKNOWN` 和禁止自动清故障语义。

ROS2 V4 驱动用于 launch、状态、MoveIt/仿真联调。其当前 `RobotStatus` 只有连接和使能，
不足以成为生产完成见证。`CR5A`、`cr5`、`cr5af` 的上游命名差异必须在现场部署前核实。

限制：

- pTLC 有滑轨与动态工站，首轮“直连”只是受控迁移验证，不自动获得 L1 结论；
- SDK/ROS 上电、使能、清错和急停 API 不注册为自动生产动作；
- 上游 ROS package manifest 中仍有 TODO 许可证字段，分发前必须审计。

## 3. 为什么首批模板保留 Elite CS

1. UniLabOS 已有 `unilabos/devices/arm/elite_robot.py`，但当前实现把 rclpy Node、socket 和手写 Modbus 混在一个类中，缺少超时、事务号并发、幂等、完成见证和安全边界。首批改造收益最高。
2. 官方 Python SDK 提供 Dashboard、RTSI、Primary、任务加载/启动/停止和安全/机器人模式诊断。控制器最低版本要求写得明确。[Elite Python SDK](https://github.com/Elite-Robots/Elite_Robots_CS_SDK_Python)
3. 官方 ROS 2 Humble 驱动包含 ros2_control、Dashboard/Primary 服务、标定、MoveIt、Gazebo 和 fake hardware；也明确要求 ExternalControl 或 remote/headless 配置。[Elite ROS 2 driver](https://github.com/Elite-Robots/Elite_Robots_CS_ROS2_Driver)
4. 控制器驻留 `.task` 与 Dashboard `loadTask/playProgram` 很适合落实“上位机只执行已验证技能”。

限制：

- `taskIsRunning` 从 true 变 false 只能证明任务停止，不能证明业务成功。必须由 PLC/机器人程序提供带 `command_id` 的完成序列；模板缺少该见证时返回 `UNKNOWN`。
- 不允许复用 SDK 的上电、松闸、解除保护停止或安全重启 API 做自动恢复。
- ExternalControl/FollowJointTrajectory 适合研发、标定和仿真，不作为 UniLabOS 默认生产动作。

## 4. 为什么首批模板保留 UFACTORY xArm

1. Python SDK 是纯 Python 可安装包，状态回调、错误码、控制器驻留轨迹录制/加载/回放、GPIO/Modbus 和滑轨能力齐全。[xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
2. 官方 ROS 2 仓库覆盖 Humble/Jazzy 等分支，提供 `xarm_api` 服务、`robot_states`、ros2_control、MoveIt、Gazebo、双臂和移动底盘示例。[xArm ROS 2](https://github.com/xArm-Developer/xarm_ros2)
3. `playback_trajectory(wait=true)` 提供厂家完成反馈，适合第二种控制器驻留技能机制，也能验证统一契约没有绑死 Elite。
4. xArm ROS 服务数量很多，恰好验证模板能把厂家大 API 面收敛到少数稳定业务动作。

限制：

- `robot_states` 的 `state/mode/error` 不是完整安全状态，模板保持 `SafetyState.UNKNOWN`，运动许可必须来自 Cell Controller 镜像。
- `mode=0/1/...` 是运动控制模式，不等于 `MANUAL/REMOTE_AUTO` 控制权，禁止混淆。
- 模板不会自动 `motion_enable`、`set_mode(0)`、`set_state(0)` 或清故障。
- 控制器驻留 `.traj` 适合固定技能；带堆栈参数的技能仍需经验证的 Blockly/PLC/Modbus 参数通路和命令级完成见证。
- 上游仓库根许可证不能替代逐包审计；部分 ROS 子包 `package.xml` 仍含 `TODO` 许可证，重新分发前必须闭环。

## 5. 三种路径的作用

DOBOT、Elite 与 xArm 分别代表：

- 已投用站点的上位机直连点位序列迁移；
- Dashboard 任务加载/运行 + ROS ros2_control；
- 控制器驻留轨迹回放 + 大量厂家 ROS 服务。

两条路径能验证统一协议的稳定部分是 `command_id/skill/version/permit/witness`，而不是某个厂家的函数名。后续 FAIRINO、JAKA、Dobot、RealMan 或 ROKAE 只需要实现 `VendorAdapter`，不得复制一套新的业务状态机。

## 6. 新厂家准入评分

新厂家至少按以下项目打分并留证：

| 项目 | 必须条件 |
|---|---|
| 官方性 | 厂家官方 SDK/驱动仓库或盖章协议文档 |
| 版本 | 控制器固件、SDK、ROS 发行版兼容表 |
| 状态 | 连接、模式、错误、运行和重启检测 |
| 高层执行 | 控制器驻留程序/轨迹或可验证的技能入口 |
| 完成见证 | 能关联 `command_id` 的接受、终态和序列 |
| 普通停止 | 明确停止/暂停语义；不冒充安全停止 |
| 仿真 | description + fake/MoveIt/Gazebo 至少一条 |
| 点位/配置 | 导出、哈希、恢复和版本绑定 |
| 许可证 | 根 LICENSE 与所有 ROS package manifest 一致、可分发 |
| 测试 | 无硬件单测、SIL、HIL、SAT 与故障注入 |
