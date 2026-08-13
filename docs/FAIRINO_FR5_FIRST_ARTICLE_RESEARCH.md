# FAIRINO FR5 首件接入与验证研究

更新日期：2026-07-31

## 结论

FAIRINO FR5 适合作为 UniLab 机械臂规范的首件真机，但第一轮应采用以下边界：

- Python SDK 适配器承担真机的单点运动、状态采集、受控停止和结果见证。
- ROS2/MoveIt2 第一轮承担模型、TF、可达性、关节限制和离线规划验证。
- 厂家 `ros2_control` 真机插件在完成版本匹配、缺陷修正和故障注入验证前，不作为生产动作的权威执行链路。
- 首轮真机只开放固定底座、隔离工作区、低速、两个直接示教锚点的无料往返；通过停机、断线和结果见证用例后，再开放基于 `site` 的 `pick/place`。

这是一套首件验证方案，不构成机械安全认证。急停、安全停止、安全门和协作安全功能仍由机器人安全系统或经验证的安全控制系统承担。

## 1. 硬件身份必须先锁定

销售页名称“FR5 6DOF 922 mm 5 kg”与标准 FR5 的公开规格相符，但不能仅凭销售链接确定控制器和机械版本。开发前必须形成一份不可变的 `hardware_identity`：

- 机械臂准确型号与铭牌照片，确认是 FR5、FR5 V6，还是其他变型；FR5-C 不能套用 FR5 模型。
- 机器人序列号。
- 控制箱型号及 QX/LA 软件包类型。
- WebApp “系统设置/关于”中的机器人软件、控制器和固件版本。
- 机器人 IP、SDK 协议版本。
- 安装朝向、基坐标、工具型号、TCP、质量和重心。
- 急停、安全停止、安全门等现场回路。

控制器版本必须决定 SDK 和 ROS 硬件插件版本，不能反过来为迁就仓库代码而先升级真机。

### 1.1 现场照片已确认的基线（2026-07-31）

机器人本体铭牌已确认：

- 厂家：FAIRINO 法奥机器人。
- 产品型号：FR5，不是 FR5-C。
- 额定负载：5 kg。
- 工作半径：922 mm。
- 本体质量：22 kg。
- 额定输入：48 VDC，最大短路电流标注为 42 A。
- 本体序列号：`SN.125025-0014862-04`。
- 校方固定资产验收日期：2025-06-08。

这组信息与公开的标准 FR5 参数一致，可以把 `fairino5_v6` 作为首轮 ROS 模型候选，但“V6”、控制器 QX/LA 类型和软件版本仍须由 WebApp About/控制器信息确认后才能冻结。

控制箱照片能确认是 FAIRINO 机器人控制箱及其 I/O 端子布局，但铭牌拍摄角度和反光不足以可靠识别完整型号；铭牌也没有可直接采用的网络 IP。默认地址只能用于发现，必须通过 WebApp、示教器/网络设置或实际连通性核验后写入设备配置。

## 2. 当前资产基线

本地已有两套官方资产：

- `uni-lab-assets/robots/fairino/python_sdk`：固定于官方 Python SDK，仓库提交标记为 Robot V3.9.8。
- `uni-lab-assets/robots/fairino/ros2`：固定于 `frcobot_ros2` Robot V3.9.7。

官方当前 ROS2 仓库已出现 V3.9.8 版本包，但本地 Python 与 ROS2 资产目前并不完全一致。Python SDK 的 `GetSDKVersion()` 还包含静态版本字符串，不能用它独立确认控制器版本；应组合使用 `GetSoftwareVersion()`、`GetRobotSN()` 和 WebApp About。

FR5 的公开 ROS2 资产包括：

- FR5 V6 URDF 和 STL 网格。
- SRDF、KDL、关节限制、Pilz 限制。
- MoveIt2 配置和 `demo.launch.py`。
- 多个按机器人软件版本拆分的硬件插件。

缺口包括：

- 默认 MoveIt2 配置使用 `mock_components/GenericSystem`，不是立即可用的真机 bringup。
- SRDF 的末端为 `wrist3_link`，没有实际夹爪、TCP、物料和工装模型。
- 官方公开仓库没有 Gazebo/Ignition world、接触或动力学仿真交付。
- UniLab 当前 Humble 环境有 ROS2 基础，但缺少 MoveIt2、ros2_control、xacro、RViz2、robot_state_publisher 和 colcon 等完整工作空间依赖。

因此需要给 FR5 建立与 UniLab Humble 配套的设备 overlay 或容器，不能把“系统已有 ROS2”等同于“厂家 launch 已满足真机条件”。

## 3. 接入模块与接口

首件实现应增加 `FairinoSdkAdapter`，放在现有 `VendorAdapter` 这一适配 seam 后：

- 对外继续使用 UniLab 的标准 skill、命令状态、幂等、许可、journal、`UNKNOWN` 和 reconcile 语义。
- 对内隔离厂家协议、状态流、单位、错误码和版本差异。
- 不增加 FAIRINO 专属的上层动作名称。

建议内部模块：

1. `FairinoIdentity`：锁定型号、序列号、软件版本和控制器 epoch。
2. `FairinoTransport`：一次性 XML-RPC 下发和显式超时。
3. `FairinoStateStream`：独立消费 CNDE/实时状态，计算 freshness、帧序号和停稳状态。
4. `FairinoSdkAdapter`：把 UniLab skill binding 翻译成厂家调用，并输出标准状态与证据。
5. `FairinoPointResolver`：把 `site + role` 或 `point_ref` 解析为经批准的点位包。

这条 seam 的目标是：厂家能力变化留在适配器内，Gateway 的业务动作和可靠性语义不随厂家变化。

## 4. Python SDK 的关键风险

官方 Python SDK 已提供首件需要的能力：

- `MoveJ`、`MoveL`、`StopMotion`。
- 实际关节和 TCP、机器人模式、错误码、碰撞、急停、安全停止。
- `motion_done`、运动队列长度、当前工具/工件坐标。
- 软件版本、机器人序列号和控制器时钟。

但其多个运动方法在 `socket.error` 后使用循环重试。通信异常时，上一条动作可能已经到达控制器，再次调用会造成重复运动。这与 UniLab 的 `DispatchUnknownError` 语义冲突。

生产适配不得直接调用带自动重试的运动包装器。正确语义是：

- 可确认未下发：`REJECTED` 或 `DISCONNECTED`。
- 已可能下发但回复丢失：立即记为 `UNKNOWN`，保持运动栅栏，查询和对账，禁止自动重发。
- 仅当能用新鲜状态独立证明终态，才由 reconcile 收敛为 `SUCCEEDED` 或 `FAILED`。

## 5. 完成、停机和归因证据

厂家返回码 `0` 仅表示调用被接受，不能直接等于 UniLab 动作成功。

单点动作的最小完成见证应同时满足：

- 收到晚于下发时刻的新鲜状态帧。
- 机器人由运行状态回到停止状态。
- `motion_done = 1`，运动队列为空。
- 无主/子错误、碰撞、急停和安全停止。
- 实际关节和 TCP 均落入该锚点的批准容差。
- 当前工具号、工件坐标号和该点位版本一致。
- 适配器为该命令写入单调递增的终态事件。

受控停止的成功也不能只看 `StopMotion()` 返回码；必须连续 N 个新鲜状态帧证明：

- 机器人已停止。
- 关节速度和 TCP 速度低于项目阈值。
- 队列已清空或进入经验证的停止状态。

无法观察到停稳就返回未确认，命令保持 `UNKNOWN`，不得解开运动栅栏。`StopMotion` 是普通受控停止，不是安全停机。

对于 `pick/place`，到达位姿仍不足以证明物料转移成功，还必须加入：

- 夹具开闭/真空达到的传感器见证。
- 源 site 物料离位见证。
- 目标 site 物料到位见证。
- 只有这些证据完成后，UniLab warehouse/site 的资源归属才提交变更。

## 6. 点位与动作

首轮公开动作保持厂家无关：

- 上层：`pick(site=...)`、`place(site=...)`、`transfer(...)`。
- 维护调试：`robot.move_to_point(point_ref=...)` 或 `robot.move_to_point(site=..., role=...)`。
- 控制面：`command.get`、`command.cancel`、`reconcile`、`controlled_stop`。

生产环境不开放任意裸关节角或裸笛卡尔位姿作为上层参数。

一个 warehouse `site` 对应的是物料语义位置；机械臂执行时应解析为版本化的点位包，而不是一个位姿：

- `approach`
- `contact` 或 `pick/place`
- `retreat`
- 可选的 `verify`
- 工具/TCP、工件坐标、负载、速度配置和见证配方

首轮使用两个直接示教的测试 site，不启用仿射阵列。基础动作通过后，再使用一个小型 2×2 阵列验证“基准点 + 有界仿射衍生 + site 索引”的规范。

## 7. 分阶段验证

### P0：身份与只读基线

- 收集 `hardware_identity` 和控制器数据备份，计算哈希。
- 连续采集 10–30 分钟状态，不发运动命令。
- 验证帧新鲜度、断线检测、控制器重启 epoch、模式/错误/安全状态映射。

通过条件：版本可复现、状态无陈旧值冒充在线、控制器重启可识别。

### P1：S0 合同测试

使用 fake adapter 验证：

- 相同 `command_id` 和相同请求不会二次执行。
- 相同 ID 和不同请求被拒绝。
- permit 过期后 fail-closed。
- 模拟回复丢失进入 `UNKNOWN`，不会自动重发。
- stop、journal 和重启 reconcile。

通过条件：外部动作合同不依赖 FAIRINO 代码也能完整验收。

### P2：两类仿真

1. FAIRINO SimMachine：验证控制器协议、错误码、状态流、程序启动和控制器重启；不用于证明物理碰撞安全。
2. MoveIt2 fake hardware：验证 URDF、TF、关节限位、自碰撞、工位可达性和路径规划；必须补上真实工具和主要工装碰撞体。

通过条件：同一动作 schema、点位版本和错误模型可在 fake、SimMachine 和 MoveIt2 中复用。

### P3：真机只读

- 适配器连接真机，只允许 identity/status。
- 校验工具、TCP、负载、基坐标和安全状态。
- 验证 MotionPermit 缺失时所有运动 fail-closed。

### P4：首次运动

前提是假设固定底座、隔离清场、无人共享工作空间；模式切换和使能由现场操作员手动完成。

- 无物料或已知轻质假负载。
- 使用风险评估批准的 commissioning 低速配置；初始建议不高于 5%。
- 仅在两个直接示教、已人工复核的锚点之间执行 A→B→A，并回 `park`。
- 记录命令、状态帧、实际关节/TCP、工具/负载、permit、原始返回码和终态见证。

通过条件：每次运动可预测、终态可证明，日志可重建一次完整执行。

### P5：受控停止与异常

- 运动中取消，验证 `StopMotion` 加连续停稳证据。
- 重复 `command_id`，验证没有第二次运动。
- 在代理层注入回复丢失，验证 `UNKNOWN` 和禁止重发。
- 杀掉 Gateway、重启并 reconcile。
- 网络物理拔线应放在 stop-on-disconnect 语义经确认之后，并由现场人员持有急停。
- 急停和安全停止只由有资格的现场人员触发，OS 仅镜像和记录。

通过条件：能被叫停、停稳可证明、通信不确定不会演变为重复动作。

### P6：site 搬运

- 建立两个有独立物料见证的测试夹具。
- 使用假物料执行 `pick(source_site)`、`place(target_site)` 和 `transfer`。
- 验证动作终态与 warehouse/site 资源归属提交顺序。
- 最后再验证一个小型仿射阵列。

## 8. 首件验收假设

1. FR5 接入不产生厂家专属上层动作。
2. 相同命令 ID 永远不会产生重复物理运动。
3. 丢失回复进入 `UNKNOWN`，不会自动重发。
4. “已叫停”必须有新鲜状态证明停稳。
5. “已完成”必须有位置、控制器和工具/物料见证。
6. 任何失败都能关联到命令、控制器 epoch、帧序号、版本、点位、工具、负载、permit 和原始错误。
7. `site` 解析为版本化点位包，warehouse 资源归属只在物理见证后更新。
8. fake、SimMachine、MoveIt2 和真机使用同一外部动作与状态合同。

## 9. 进入开发前仍需现场提供

- 机械臂及控制箱铭牌照片。
- WebApp About 页面截图。
- 当前软件/固件版本、控制器类型和序列号。
- 末端工具、TCP、质量、重心及夹具传感器。
- 安装朝向、现场总图、是否有滑轨/共享工作区。
- 急停、安全停止和围栏/安全门配置。
- 当前网络地址，以及是否允许升级控制器软件。

## 10. 主要来源

- FAIRINO FR5 产品页：https://www.fairino.com/FR/4.html
- FAIRINO Python SDK 手册：https://manual.fairino.support/latest/SDKManual/python_intro.html
- FAIRINO ROS2 指南：https://manual.fairino.support/latest/ROSGuide/ros2guide.html
- FAIRINO MoveIt2 指南：https://manual.fairino.support/latest/ROSGuide/moveIt2.html
- FAIRINO SimMachine：https://manual.fairino.support/latest/VMMachine/controller_docker_machine.html
- FAIRINO 安全说明：https://manual.fairino.support/latest/CobotsManual/safety.html
- FAIRINO Python SDK：https://github.com/FAIR-INNOVATION/fairino-python-sdk
- FAIRINO ROS2：https://github.com/FAIR-INNOVATION/frcobot_ros2
