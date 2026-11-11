# 导轨、机械臂与末端工具的工站组合

本轮采用一套机械装配、按部署绑定执行能力。品牌包提供裸臂型号及固定资产绑定；导轨是公共外部轴接口加领域选定的参数/真实几何；末端工具由领域安装声明选择。`unilab-rail-mounted-arm` 是组合与执行边界，不能替代三者的资产，也不复制工站任务与 OS 库存。

## 外部工站入口

工站继续调用领域 Actions；领域包选择 `Composition.execution(mode)` 并将调用交给既有执行器。OS graph 是实例身份、父子关系和安装位姿的权威，`assemble_robot_model` 连接 provider；组合 JSON 只声明部件、关节、工具身份/TCP 来源和控制能力，不是第二份布局图。

| 部分 | postproc | SZLab |
|---|---|---|
| 裸臂 | 固定库 CR5，兼容 `unilab_arm_cr5` provider | 固定库 CR7，兼容 `unilab_arm_cr7` provider |
| 导轨 | 0–1 m、X 方向、仿真 0.05 m/s | 0–2.25 m、X 方向、仿真 0.25 m/s |
| 已安装工具 | 既有 CAD 夹爪本体与两手指；沿用标定 | 选定 v2 URDF 工具子树；最大垫面开口 0.09 m，执行器合计行程 0.08 m |
| 仿真 | 现有 OS 场景、领域序列与保存轨迹 | 现有 USD/Blender 状态、领域动作与保存轨迹 |
| 真机 | 没有已接通的真机路径 | 既有 PLC 程序/任务号/传感器与 journal 持有整个组合 |

上述速度是沿用的仿真配置，不增加厂家动力学资格。SZLab 选定的 v2 embodied URDF 不是仓内旧 `device.xacro`；后者的静态手指和单网格导轨不能自动替换该绑定。

## 配置与使用

- `RailConfiguration` 校验行程、速度和单位轴向；`build_configured_model(device_id=..., configuration=...)` 生成仅含单轴的 URDF/SRDF。真实床身/滑座由领域资产提供，避免重复几何；OS graph 安装该轴。旧 `build_kinematic_model()` 输出及摘要保持兼容。
- `Composition.read(path).validate_model(urdf, external_geometry=...)` 校验活动关节归属、挂载帧及工具几何身份。postproc 的 `cad:` 选择器在实际场景装载后解析，不能带未解析项放行；SZLab 的 `link:` 选择器在编译 URDF 中解析。
- 内嵌工具用 `binding: embedded`，只绑定已有子树；禁止再次实例化资产。独立换装工具用 `binding: asset`，声明 `asset_lock`，仍由既有资产解析器验证锁与几何，并由 OS graph 安装。组合声明本身不下载资产或证明几何等价。
- 每部件恰好一个声明控制所有者。`plc_program` 的单一 owner 负责 rail/arm/tool，不能直接提交 trajectory、point 或独立 tool 命令。`ExecutionBinding.dispatch()` 只调用既有执行器一次，不重发、不写第二份 journal。
- `independent_drives` 仅为可选接入模式；两示例未启用、未授予 SDK 真机资格。现场控制权切换仍经现有部署/端点租约与 OS 资源占用，不由 JSON 声明替代。

默认六轴臂、导轨、工具分组分阶段执行。MoveItPy 当前只规划固定导轨的六轴路径；其导轨变化请求明确拒绝。需要导轨规划时使用已有独立导轨检查或 `MoveGroupPlanner` 的导轨组，不给六轴适配器伪造第七轴。更新模型/安装/TCP/碰撞体后重验保存轨迹，不静默改写旧摘要。

## 安装与验收边界

运行需要品牌发行包、公共 robotics 包、领域包、OS 以及锁定标准资产；`uni-lab-assets` 不是运行依赖。构建和验收在隔离远端完成，wheel 安装验证禁止读取源码快照。

本轮证据位于 `/home/tyzuo/robot-workcell-composition-20261108/evidence`；根仓计划记录最终矩阵。区分合同回归、原生 MoveIt/FCL、USD/Blender 运动学读回、物理接触和真机验证。合成载荷负例只验证附着碰撞机制；八动作运动学回放不等于带整站与真实载荷的工艺路径资格。没有连接真机，没有新增控制器或物料账本。

## 2026-11-09 两工站完整场景验证

已用 postproc 的载架/样品/瓶盖和 SZLab 的原始 500 mL 烧杯完成导轨、臂、原装夹爪的整段搬运及独立 USD 读回。细项见两领域 `docs/robot-composition.md` 和根仓组合计划，未连接真机。

`native_planning_session` 通过实际 ament 安装位置定位 move_group，并按安装的 adapter 描述区分旧 request TOTG 与新版 response TOTG。原生 OMPL 简化启用后消除单轴 RRT 折返，仍对返回轨迹做完整场景密集检查。`rail_goal_tolerance_m` 与 `joint_goal_tolerance_rad` 分开；领域必须采用返回的实际导轨终点，不能隐式补接。没有轨迹的原生失败响应保留错误类别和空轨迹原因。

本例通过路径/碰撞/运动学执行不等于物理或真机验收；首次验收曾出现 MoveIt 2.12.4 owner 关闭段错误，旧失败日志保留；后续修复见下节。


## 2026-11-09 原生 MoveIt 关闭修复

已在 Linux x86-64 / ROS Jazzy / MoveIt 2.12.4 / rclcpp 28.1.21 复现并修复关闭时的 `-11`。不加载工站、不调用 Python 规划客户端的单关节原生进程也会失败。`LD_DEBUG=files` 记录默认能力库先卸载；随后 CallbackGroup 析构的故障地址恰落在该库原来的只读映射内。具体 weak_ptr 控制块机制由堆栈和上游源码推断，没有取得 GDB 对象级读回。

`move_group_environment` 仅对 Linux 且安装 manifest 精确为 2.12.4 的原生子进程，通过 `LD_PRELOAD` 保持默认能力库映射到进程结束。原来的 preload 保留，父 Python/ROS 环境不加这个库，其他版本不自动套用。它是已验证的启动器兼容修复，没有修改上游 MoveIt 二进制或跳过析构；升级 MoveIt 后须重跑原生验收。

关闭仍先释放客户端，再向本次拥有的服务发送 SIGTERM 并 wait。`native-exit.json` 记录真实退出码、是否超时强杀及 `clean_shutdown`。非零退出或强杀会导致会话失败；若规划本身已抛异常，关闭失败作为 note 附加，保留原异常。不能用成功规划覆盖失败的资源关闭。

验收：96 项自动化测试通过；维护入口负例 2/2 复现 -11，修复后 5/5 正常退出；另有最小模型和两套真实机器人参数共 12 次正常退出。postproc 与 SZLab 的完整 CAD/真实载荷搬运均重新规划、碰撞检查、仿真回放并正常关闭（exit 0，未强杀），两套参数文件 SHA-256 与上次完全相同。Windows、其他 MoveIt 版本及真机没有获得本次资格。

原生复现入口为 `tests/engines/native_shutdown.py`。在已安装 robotics 和 ROS 的环境中运行 `python -B tests/engines/native_shutdown.py --output <新目录> --runs 3`；显式加 `--unpatched` 仅用于负面对照，可用 `--robot-config` 输入现有工站参数。测试保存配置、二进制摘要、日志、加载映射和实际退出码。

完整证据与最终场景：`/home/tyzuo/robot-workcell-composition-20261108/shutdown-fix-20261109/delivery`。本地维护回执：`D:/UniLab_maintenance/20261109/moveit-shutdown`；本地交付目录：`D:/UniLab_asset_evidence/robot-workcell-shutdown-20261109`。原始资产与转换脚本继续独立保留，不把诊断日志放入资产库。
