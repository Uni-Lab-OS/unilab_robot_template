# 机械臂标准动作规范（物料搬运）

版本：1.1
适用范围：UniLabOS 接入的工业机械臂、协作机械臂及其 Cell Controller
关键词：**必须**表示强制要求，**应**表示默认要求，**不得**表示禁止要求。

## 1. 边界和总原则

UniLabOS 只下发业务意图，不下发任意关节角、TCP 位姿、路径、速度或厂家脚本。

- UniLabOS 是任务、配方、物料身份和资源预约的权威。
- Cell Controller 是滑轨、多机器人、干涉区和运动许可的权威。
- 机器人控制器是已验证程序、轨迹、生产点位及实时运动的权威。
- 安全 PLC/安全控制器是急停、安全门、安全区和安全速度的权威。
- 软件状态、ROS 话题和普通 PLC 位不得宣称为安全功能。

生产 API 必须引用已批准的 `program_version`、`point_set_version`、
`tool_profile` 和 `payload_profile`。原始 `move_to_point`、`move_joint`、
`move_pose` 只允许出现在有控制权切换、限速和审计的维护/示教模式，不得注册成
UniLabOS 生产动作。

## 2. 标准动作集合

### 2.1 生产公开动作与工作流宏

| 层级/动作 ID | 用途 | 必须的完成见证 |
|---|---|---|
| 工作流宏 `material.transfer` | 展开物理取放和 OS 记账；默认业务入口 | 取料、放料、退出和物料归属提交 |
| 设备动作 `<robot>.pick` | 只取料并让机械臂持有物料 | 夹持/吸附、源位释放、机器人到达持有锚点 |
| 设备动作 `<robot>.place` | 把机械臂已持有物料放到目标位 | 释放、目标在位、机器人退出目标区 |
| Host 动作 `host.transfer_resource` | 物理放料成功后提交 OS 资源树；不运动 | 前序 `place` 已成功且物料句柄未变 |
| `material.handoff` | 机器人、输送线或人工交接 | 双方交接协议完成、物料唯一归属提交 |
| `robot.park` | 移动到已验证停放点并释放可释放干涉区 | 停放点锚定见证、区域释放 |
| `robot.verify_anchor` | 不改点位，仅验证机器人是否位于指定锚点 | 关节、位置和姿态均在批准容差内 |
| `robot.recover` | 执行已验证的具名恢复程序 | 恢复终点锚定、故障清除条件、人工/控制器授权 |

默认向编排层提供 `material.transfer` 工作流模板、`robot.park` 和查询类动作。
`pick`/`place` 应作为同一单元内的受控组合动作；如果单独对外公开，
必须显式处理“机械臂持料”的长期资源占用和人工恢复。

`material.transfer` 不得注册成机器人设备的 `transfer_resource` 方法。UniLabOS
当前 `host.transfer_resource(resource,target_device,mount_resource,site)` 是纯记账
动作，标准物理流程固定为：

```text
robot.pick → robot.place → host.transfer_resource
```

动作名和参数的完整对齐表见
[OS 动作对齐与工作区干涉协调结论](OS_ACTION_ALIGNMENT_AND_ZONE_COORDINATION.md)。

### 2.2 控制和查询动作

| 动作 ID | 语义 |
|---|---|
| `command.get` | 查询已有 `command_id`，绝不触发重发 |
| `command.reconcile` | 依据机器人/PLC 完成见证对账，绝不重发运动 |
| `command.controlled_stop` | 请求普通受控停止；结果不确定时返回 `UNKNOWN` |
| `command.pause` / `command.resume` | 仅在厂家和项目已验证暂停语义时启用 |

急停不是 UniLabOS 动作。取消 ROS Action 也只等于普通停止请求，不等于安全停止完成。

## 3. `material.transfer` 工作流宏契约

### 3.1 请求

```yaml
protocol_version: unilab.robot/v1
command_id: 018f...
source: unilabos
source_boot_id: scheduler-boot-uuid
monotonic_sequence: 1042
skill_id: material.transfer
program_version: sha256:...
point_set_version: tube_cell@v12
material:
  material_id: tube-000184
  material_type: centrifuge_tube_50ml
source_location:
  resource_id: rack_A
  slot: r2c3
destination_location:
  resource_id: centrifuge_01
  slot: bucket_4
tool_profile: parallel_gripper@v3
payload_profile: tube_50ml@v2
handling_profile: sealed_liquid_tube@v2
parameters:
  source_grid: tube_rack_4x3
  destination_variant: bucket
```

强制规则：

- `command_id` 在调用方一次启动周期内和跨重试都必须不变。
- 相同 `command_id` 和不同请求内容冲突时必须拒绝。
- `monotonic_sequence` 必须递增；调用方重启后更换 `source_boot_id`。
- `source_location` 和 `destination_location` 必须是已预约的具名资源/槽位。
- `point_set_version` 必须与机器人侧已激活部署清单完全一致。
- `parameters` 只允许技能清单中声明、限定范围的业务参数；不得包含 pose、
  joints、trajectory 或 waypoints。
- 液体、开口容器、易碎件等必须指定匹配的 `handling_profile`。

### 3.2 执行阶段

状态反馈必须至少能区分：

1. `VALIDATING`：校验版本、控制权、许可、工具、负载和物料/槽位状态。
2. `RESERVING`：获取机器人、滑轨、源、目标和干涉区资源。
3. `APPROACHING_SOURCE`：执行已验证的源位接近程序。
4. `ACQUIRING`：抓取/吸附物料。
5. `VERIFYING_ACQUIRE`：读取夹爪、真空、视觉或 PLC 的取料见证。
6. `RETREATING_SOURCE`：退出源设备限制区。
7. `TRANSITING`：经过已验证的中间/避让路径。
8. `APPROACHING_DESTINATION`：执行已验证的目标接近程序。
9. `RELEASING`：释放物料。
10. `VERIFYING_RELEASE`：确认工具空、目标在位和目标设备可接收。
11. `RETREATING_DESTINATION`：退出目标设备限制区。
12. `COMMITTING`：调用 `host.transfer_resource` 原子提交物料归属，再释放业务资源。

这些阶段可以由机器人控制器内部实现，但对 UniLabOS 至少应上报当前阶段和最新见证。

### 3.3 物料归属

物料在任意时刻必须只有一个权威位置：

```text
SOURCE_SLOT → IN_TRANSIT_ON_TOOL → DESTINATION_SLOT
```

- 只有取得有效取料见证后，才能把归属从源槽位改为 `IN_TRANSIT_ON_TOOL`。
- 只有取得有效放料见证后，才能改为目标槽位。
- 见证缺失、掉线或人工介入后，物料位置必须变为 `UNKNOWN`，冻结相关源、目标和工具，
  待人工或传感器对账。
- 不得因为机器人程序返回“结束”就推断物料已转移。

推荐见证按强到弱组合：目标设备握手、独立在位传感器、夹爪/真空反馈、视觉识别、
机器人程序位。生产项目应至少使用两个相互独立的信息源；若受设备限制只能使用一个，
必须在风险分析和验收记录中说明。

### 3.4 结果

```yaml
command_id: 018f...
state: SUCCEEDED
success: true
boot_id: robot-gateway-boot-uuid
phase: COMMITTING
material_location:
  resource_id: centrifuge_01
  slot: bucket_4
witnesses:
  acquired: {type: gripper_and_source_handshake, sequence: 82}
  released: {type: destination_and_tool_empty, sequence: 96}
  retreated: {anchor: centrifuge_01_exit, verified: true}
program_version: sha256:...
point_set_version: tube_cell@v12
```

终态只有 `SUCCEEDED`、`FAILED`、`CANCELED`、`REJECTED`。`UNKNOWN` 是非终态：
它表示不能确定动作是否执行或物料在哪里，必须先对账，不得自动重发。

## 4. 前置条件和 fail-closed 规则

每个可能引起运动的动作必须同时满足：

- 连接为 `ONLINE`，设备为 `IDLE`；
- 控制权为 `REMOTE_AUTO`；
- 安全系统上报允许运动，且许可未过期；
- 没有未对账的 `UNKNOWN` 命令；
- 工具、TCP、负载、基坐标、标定、程序和点位版本匹配；
- 源物料身份、源在位、目标空闲/可接收；
- 机器人、滑轨、源设备、目标设备和全部经过的干涉区已预约；
- 当前点位满足技能声明的 `require_anchor`，或技能入口包含已验证的恢复段。

任何信息缺失都必须拒绝动作。软件不得把 `UNKNOWN` 当作 `NORMAL`、`IDLE` 或空槽位。

## 5. 搬运轨迹约束

每个搬运技能必须显式定义：

- `source_approach`、`source_pick`、`source_retreat`；
- `transit_anchor[]` 或机器人控制器内的具名路径；
- `destination_approach`、`destination_place`、`destination_retreat`；
- 允许的工具、TCP、负载和物料姿态；
- 各阶段速度/加速度上限及装液物料的倾角、角速度、等待时间上限；
- 夹持力/真空阈值、取放见证和超时；
- 可占用的干涉区及申请/释放时刻；
- 失败后允许执行的具名恢复路径。

阵列槽位只改变已批准技能的具名目标，不能绕过工作空间、轴限位、碰撞、工具和负载检查。

## 6. 资源和死锁

`material.transfer` 默认以固定顺序预约：

```text
robot → rail → source → destination → zones（按 ID 排序）
```

- 预约必须带租约、所有者 `command_id` 和 fencing token。
- 机器人进入区域前必须获得区域许可，退出并验证锚点后才能释放。
- 多机器人项目必须由 Cell Controller 做区域仲裁，UniLabOS 的资源锁不能替代它。
- 避让必须调用已验证的 `robot.park`/`robot.yield_zone` 技能，不得在线生成任意避让方向。
- 超时后只能撤销尚未进入的预约；已经进入或持料时必须进入对账/恢复流程。

## 7. 幂等、断线和恢复

- “请求已发送”不等于“机器人已接受”，“程序结束”不等于“物料已搬完”。
- 网关必须在下发前持久化请求和指纹。
- 下发后通信中断必须记录为 `UNKNOWN`。
- 相同 `command_id` 的重试只返回已记录状态，不能再次下发。
- 机器人、PLC 或网关重启必须更换 `boot_id`，此前在途命令全部待对账。
- 自动恢复运动默认禁止。只有具名、版本化、通过仿真和现场验证的恢复程序可自动执行。
- 急停、安全门打开、工具/物料人工移动后必须重新验证锚点及物料位置。

## 8. 动作清单交付要求

每个厂家/项目必须交付一份技能清单，至少包含：

- 动作 ID、版本、厂家程序/轨迹路径及 SHA-256；
- 点位集、工具/TCP、负载和 handling profile 版本；
- 参数类型、枚举和数值上下限；
- 前置锚点、终点锚点、占用资源和干涉区；
- 正常完成、取料、放料和退出见证；
- 超时、暂停、取消、断线和重启语义；
- 允许的恢复技能和需要人工介入的情形；
- 仿真、SIL/HIL、SAT 用例编号和批准记录。

没有上述清单的厂家动作不得加入生产 Profile。
