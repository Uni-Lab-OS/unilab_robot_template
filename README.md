# Uni-Lab Robotics

这是机械臂、导轨和组合工作单元（WorkCell）的共享 monorepo。根项目只提供开发、测试和统一治理，不发布一个兼容性总包；各能力作为独立 Python distribution 发布。

根目录作为独立 Git 仓库维护。`scripts/build_distributions.sh` 分别构建
`packages/` 中的 wheel/sdist，CI 在 Python 3.11/3.12 上执行统一测试和制品构建。

## 分层

```text
unilab-robot-contracts     L0：指令、后端、观测、硬件配置和点位解析合同
unilab-arm-cr7             L1：CR7 型号、限位、运动学及 PLC/TCP-SDK/MoveIt Adapter
unilab-arm-cr5             L1：CR5 型号、限位、运动学及 PLC/TCP-SDK/MoveIt Adapter
unilab-end-effector-sim    L1：夹爪与快换工具的可替换仿真 Adapter
unilab-rail-linear         L1：单轴导轨型号、行程及 PLC/仿真 Adapter
unilab-rail-mounted-arm    L2：厂商无关 rail-then-arm 协调、互锁和端点租约
unilab-robot-runtime       L2：依据 HardwareProfile 选择并装配上述模块
领域部署包                 L3：exact manifest、SiteAccessBinding、点位、标定和安全参数
```

领域仓库不应复制通用 Adapter、运行时路由或 WorkCell 状态机。机械臂型号包不保存现场点位、Warehouse 传感器地址或 Site 绑定。

## 公共边界

- FE/Workflow 只使用厂商无关的 `pick`、`place`、`pour`。
- `RobotCommand` 只携带已解析目标，不携带 Site、Material 或原始地址。
- PLC、TCP/SDK、MoveIt 都实现相同的 `RobotExecutionBackend` 生命周期。
- MoveIt 只规划机械臂关节；RViz 是可选客户端，不参与启动和完成判定。
- 真实导轨机械臂由一个 WorkCell 公开动作，导轨到位并获得硬件许可后才派发机械臂。
- `execution_unknown` 不得自动重放；生产仍依赖 OS 持久作业执行占用（JobExecutionClaim）和栅栏（Fence）。

## 点位

型号包拥有规范关节顺序、类型和限位。L3 点位资产使用 `unilab.arm-point-set/v2`，不重复保存 `joint_names` 或 target-level `unit`。`joint_positions` 的每个值按型号关节类型解释为 SI 单位；`cartesian_delta` 可相对关节点（先通过 FK 转换）或另一个 delta 点递归展开。

示例见 `config/arm_point_set.v2.example.yaml`。生产点位、工具上下文和安装标定必须由 exact digest 原子锁定。

## 本地维护闭环

- `RobotCommissioningPort` 是 PLC、TCP/SDK、MoveIt 共用的唯一调试 Interface。
- 当前 MoveIt Adapter 已实现版本化目标、临时 TCP 位姿、TCP 点动、单关节点动和受控停止；全程只依赖 `move_group`/控制器，不依赖 RViz。
- `RuntimeBinding.open_maintenance_session()` 取得物理端点独占维护会话；会话期间生产 `RobotCommand` 失败关闭。
- `unilab-robot-maintenance` 提供本地 snapshot/运动命令入口，不经过 Backend 或 FE。
- `PointMaintenanceService` 强制执行草稿解析、逐点低速试运行、资格确认和按 digest 不可变发布。
- `EndEffectorPort`、`ToolChangerPort` 与 `ToolContextActivator` 保持独立 seam；工具附着代次改变后，MoveIt 必须确认 PlanningScene 使用新的 ToolContext 才能继续。

`tests/test_headless_moveit_e2e.py` 覆盖 Manifest→PointSet→Runtime→维护会话→MoveGroup 的无 RViz 链路；`tests/test_manipulation_simulation.py` 覆盖快换、抓取和回撤序列。

## 测试

```bash
pytest
```

真机上线还必须分别完成厂家完成见证、硬件互锁、末端执行器/快换观测、碰撞环境和断线/重启 UNKNOWN 测试。仿真许可不得替代生产安全链。
