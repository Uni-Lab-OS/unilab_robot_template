# 机械臂本地维护与点位发布

## 运行边界

生产动作继续使用 `RobotCommand`。维护人员通过 `RuntimeBinding` 打开独占维护会话，
再使用同一个 `RobotCommissioningPort` 执行 `move_target`、`move_pose`、
`tcp_jog`、`joint_jog` 或 `controlled_stop`。维护会话存在时生产动作被拒绝；
调试结果为 `execution_unknown` 时保留 Fence，不允许关闭会话或自动重放。

## MoveIt

`MoveItCommissioningAdapter` 读取完整关节状态和当前 TCP，校验活动
HardwareProfile、PointSet revision、exact Arm 关节限位与 ToolContext digest 后，
只调用 MoveGroupPort。启动依赖为 robot_state_publisher、controller_manager、
joint_state_broadcaster 和 move_group；RViz 仍是可选可视化客户端。

## 点位发布

`PointMaintenanceService` 的唯一顺序是：

1. `validate_draft()`：冻结源文件 SHA-256，并解析全部目标和相对引用。
2. 使用锁定该候选文件 exact digest 的 maintenance/simulation manifest 启动运行时；
   维护会话报告的活动 revision 必须和草稿一致。
3. `test_target()`：通过独占维护会话逐点低速执行，并持久保存命令见证。
4. `qualify()`：要求全部目标都有成功见证，生成资格确认记录。
5. `publish()`：只发布与资格记录相同摘要的源字节；已存在不同内容时拒绝覆盖。

## 快换和夹爪

快换 Adapter 拥有工具身份、锁紧观测和单调附着代次。`ToolDefinition` 用工具模型、
TCP 和碰撞资产生成新的 ToolContext。规划器通过 `ToolContextActivator` 确认同一
digest/attachment_generation 后，组合运行时才执行 ready→approach→interaction→
grip/release→retract→ready。夹爪和快换不属于 CR7 型号包，因此可独立替换。
