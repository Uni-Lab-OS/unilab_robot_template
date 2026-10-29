# 关节段参考执行器（C30l）

`unilab_robot_runtime.JointSegmentExecutor` 从冻结 CR12/CTAG 实验迁移，拥有一个关节组的一条在执行命令。构造时传入该组关节顺序、SI限位、模型版本和会话；调用者在同一仿真线程按显式模型时钟送入实际位置、速度及递增序号。

```python
from unilab_robot_runtime import JointSegmentExecutor

executor = JointSegmentExecutor(joint_names, limits, model_revision, session_id,
    velocity=velocity_rad_s, acceleration=acceleration_rad_s2)
executor.observe(actual_positions, actual_velocities, sim_s, sequence)
executor.submit(command)
target = executor.setpoint(next_sim_s)
```

命令继续使用已有字段：command_id、joint_names、positions、model_revision、session、observation_sequence、duration_s。已有所有字段校验、五次插值、速度/加速度预算、单写者/重复命令拒绝、稳定时间及反馈超时行为保持；类名改为公开的JointSegmentExecutor，其余函数/方法AST相同。

实际位置误差和速度都满足容差并持续settle_s后才成功。取消记录当前测量位置作为hold，后续实际反馈稳定才发出CANCELED；超时为ABORTED。取消不是紧急停机、制动、碰撞保护或厂家停止协议。

构造参数仍沿用原实验的受控输入前提：调用者提供非空唯一关节组、等长有效限位、有限正运动参数，以及可靠且单调的反馈时钟。本次不把旧参考实现改称完整输入校验器或生产控制器；没有新增时钟、线程、自动重试、暂停/epoch或PLC扫描。实际反馈停更时，调用者仍需独立检测断线；此类不会自行唤醒计时器。

此模块属于机器人包的参考行为。OS可选Isaac桥接位于 `unilabos.sim.backends.isaac.joints.IsaacJointBackend`；它以已初始化articulation读回/下发，不初始化世界。两边互不复制实现，型号、安装、采样周期和命令范围仍由已有设备包输入决定。CTAG旧实验的[0.2,0.9]弧度范围由调用者传入，不能自动从原URDF[0,1]推导为全行程可用。

没有登记新的device_pair、OS启动入口或设备图，不将该参考执行器直接接到OS动作来替代电气闭环。以后PLC联合路径的目标/状态应由所属设备包的电气与行为接线使用，原驱动→PLC→物理反馈仍单独验收。本批没有领域装配，不运行领域图检查器；机械臂/导轨仍按仓内技能保持独立设备与独立规划组。

验证：原9项执行器用例保持；OS桥接8项SDK/数组替身检查、主仓2项跨包关节组检查通过。历史2690条反馈、7命令、1次取消逐项重放，5378组目标值、全部状态和最终事件与原实现及历史记录精确一致。历史输入摘要/大小/mtime保持。未运行ROS、Isaac、GPU或硬件，未构建或安装发行包。来源见 `migration/joint-segment-source.json`。
