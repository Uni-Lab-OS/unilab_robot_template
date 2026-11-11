# 原生驱动接入与离线验收（2026-11-08）

这轮把实际厂家 SDK / 已安装 RoboDK 驱动接到了品牌模块工厂。结果须分成三层：代码入口可调用、实际厂家代码在受控端点运行、具体型号/控制器实机支持。最后一层仍为 **0**，没有连接或移动真实机器人。

## 五个优先型号

| 品牌/型号 | 实际执行对象 | 结果 | 边界或阻塞 |
|---|---|---|---|
| Dobot CR5 | 已安装 `DobotV3Driver.py` + `v311/DobotV3Driver.pyc`，v1.0.0 | connect、GetAngle、JointMovJ、非零关节回读通过 | 29999/30003 loopback 模拟端点；驱动 CONNECT 会发送 EnableRobot，真机部署须明确授权此副作用 |
| KUKA KR6R900sixx | 已安装 `kukabridge.exe`，1.0.8.0 | C3 PING/信息、CJNT、COM_E6AXIS/COM_ACTION 写入、Ready 与后续关节回读通过 | 使用公开 C3 协议端点，不是 KUKA 控制器仿真；反馈含 12 轴，显式选择前 6 轴，未控制外部轴 |
| Elite EC66 | 已安装 `EliteRobotsDriverEC.py` + v311，v1.0.2 | connect 与真实 JSON-RPC 关节读取通过；moveByJoint 序列化已观察 | 当前响应 fixture 下 MOVJ 不产生对应 Ready，严格适配器超时隔离；不借后续 CJNT 的 Ready 证明 MOVJ 完成 |
| Elite EC66 原生入口 | 官方 ROS README 指定的 `elirobots 0.0.7` EC SDK | connect / get_joint / move_joint / 非零回读通过，控制器错误和超时拒绝 | 与 RoboDK 入口分开；未自动开伺服，显式声明两个外部轴保持值；ACK 不算运动完成 |
| ABB IRB1200-5/0.9 | 已安装 `ABBDriver.exe`，v3.0.1 | 实际进程、TCP 请求编码及超时隔离已执行 | 尚无与驱动相符的 RDK_DriverSocket.mod 协议端点；官网包位于需公司和授权码的 Customer Area，不算完整离线通过 |
| FANUC LR Mate200iC | 已安装 `FanucSM.exe`，v5.3.2 | 实际进程、TCP CONNECT 请求及超时隔离已执行 | 尚无匹配的 RDK_S3.PC/GO_MJ 等控制器侧程序和协议端点；不算完整离线通过 |

本机 api/Robot 目录相邻 Qt5Core.dll 是 x86，而上述原生驱动是 x64。现支持显式库外启动目录中的可校验硬链接，PATH 指向已安装 RoboDK/bin 的 x64 Qt。未修改安装目录，未把专有 exe/DLL 放进 wheel。启动目录需支持与安装源的硬链接；执行文件、编译入口、Qt 依赖 SHA 均留在回执。

官方依据：[ABB 驱动要求](https://robodk.com/doc/en/Robots-ABB.html)、[FANUC 选件与服务端程序](https://robodk.com/doc/en/Robots-Fanuc.html)、[KUKA C3 驱动](https://robodk.com/doc/en/Robots-KUKA.html)、[C3 公开实现与协议](https://github.com/ulsu-tech/c3bridge-server)。

## 原 53 个 SDK-only 型号，分母保持不变

| SDK 族 | 原型号数 | 已实现接口 | 真实 SDK 离线结果 |
|---|---:|---|---|
| Agilebot A | 1 | connect / read_joints / joint request | 官方已安装 SDK，经 HTTP loopback 完成非零目标编码和读回；断响应 deadline 通过。GBT-P7A 的真实控制器固件仍待确认 |
| Elephant | 2 | 280 Socket connect / read / joint request；Pro600 本地只读入口 | myCobot280M5 接口通过；`mycobotpro630` 内明确 `Robots.PRO_600=502`，对应 Phoenix/LinuxCNC，而非远程630类等价。Pro600 `do_init=False` 只读入口已接工厂；当前非 Debian aarch64 平台真实 SDK import 报 Enum 未定义，目标 LinuxCNC/HAL/CAN 环境未具备 |
| Igus CRI | 11 | connect / fresh status / joint request | 官方 CRI-Python-Lib 实际 TCP 编码/解析、非零读回、超时通过；这不是 11 个物理型号逐一确认 |
| HIWIN | 8 | connect / read / joint request | 官方 Commander C++ 实际序列化与读回通过；断连、短包、错误命令、控制器错误、超时拒绝。RA6/RS4 文档范围不能直接扩张为 RT605 已验证 |
| Inovo | 1 | connect / read_joints / move_cartesian | ROS Noetic 官方 Commander，经真实 ROS 服务/action/topic 闭环；仅挂载 catkin **安装前缀**再次通过，没有挂 SDK 源码/devel。原 SDK 没有直接 joint-target 命令 |
| Estun ERI | 25 | connect / read / 显式 native-units movJ | Linux 真库 ERI 1.0.0.17 已加载并调用。SDK 要求 SCHED_RR priority 51，当前权限返回 EPERM；configure=1 而三通道仍 0。状态/运动正例未完成，不能假连通 |
| Flexiv RDK | 4 | connect / read / explicit joint position request | 官方 2.1 实际拒绝 Rizon4s；改用 1.9.4.1 后构造器实际运行，隔离环境无法连接 RDK server。状态/运动未验证 |
| Franka Panda FCI | 1 | connect / read（只读） | 官方 libfranka **0.9.2** C ABI 已真实编译执行，FCIv5 synthetic peer 非零7轴读取和断流拒绝通过；拒绝运动。需匹配 Panda system >=4.2.1；现代 pylibfranka 0.21 属FR3，不计为Panda支持 |

当前原 **53 个型号均有原生族入口**；其中 **23 个型号所对应的6个族入口**有真实 SDK 正/负例证据（Agilebot、280、Igus、HIWIN、Inovo、Panda）。它们仍全部是 `model_controller=not_verified`，没有改变模块 `hardware=not_qualified`。剩余 30 条明确保留阻塞：Pro600 1、Estun 25、Flexiv 4。53 不等于53个型号已适配完成，23不等于逐型号实测；Elite 新入口另计，不改变原53分母。

公共 `ControllerJointMap` 已支持逐轴 rad/deg/m/mm 与 SI 仿射映射；Igus/Estun 原生边界可接显式混合单位，RoboDK degree-only入口继续拒绝直线轴。实际 CRI SDK 对“5旋转+1直线”的合成配置已完成 .06m→60mm wire 编码、回读及超时验证，依据[官方 CRI 单位说明](https://cpr-robots.com/download/CRI/CPR_RobotInterfaceCRI.pdf)。这不是任何具体SCARA型号的标定或规划验收。Estun 两个已知SCARA仍缺ERI控制器逐轴单位及RT权限证据；缺少源轴类型的型号记为 `axis_types_unknown`。工厂必须收到显式 `joint_types`、`joint_type_evidence`、native units与单位证据，并与已知来源一致，不从缺字段推断全旋转。

HIWIN 上游 `getActualPosition` 存在忽略 read/write 返回值、EOF 后以零缓冲报告成功的问题。构建脚本在**库外构建副本**加入完整帧、命令号和传输错误检查，保存原始源码 SHA、精确补丁、原版假零负例和修复后回执；没有修改厂家算法或伪造数据。低层 Commander 建连接不自动启用 Auto/Servo，运动返回只解释为 ACK。

主要来源：[CRI 官方库](https://github.com/CommonplaceRobotics/CRI-Python-Lib)、[HIWIN 官方库](https://github.com/HIWINCorporation/hiwin_robot_client_library)、[Flexiv 官方安装和版本环境](https://www.flexiv.com/software/rdk/api/v2.1/index.html)、[Franka 版本兼容记录](https://github.com/frankarobotics/libfranka/blob/main/CHANGELOG.md)。Agilebot、Elephant、Inovo、Estun 的本地源码版本、license、调用入口和哈希在逐项回执中固定。

## 运行接口

品牌 `create_hardware_driver` 工厂仍为入口，选择精确 candidate ID。原生候选 ID 为 `native:<family>`，需要显式 `python_executable`、`native_config`、controller、deployment_evidence、已校准的 mapping。SDK 与原生库单独安装，品牌 wheel 不持有 SDK/资产大文件。

`native_config` 固定 DOF、SDK 模块 SHA、原生扩展 SHA、安装搜索路径，以及族特定的控制器参数。HIWIN/Estun 固定库外 bridge SHA；Estun 还要明确单位、16 轴剩余值和已获控制权的部署证据，不能补默认零。Flexiv 固定 1.9 接口、序列号和接口白名单。Inovo 指定 ROS master、action namespace、关节名称、最大反馈时龄。

所有阻塞调用经独立 SDK worker，具有整体 deadline、命令 ID 去重、严格参数白名单与失败隔离。原生请求 ACK 只生成 `submitted/completed=false`；驱动进程被终止不等于机器人停止。无通用已验收的 STOP、轨迹/插补或复合动作恢复，不授予这些能力。

Igus 与 Agilebot 的9轴请求不再补默认零；与 Elite/Estun 一样，调用者必须明确未归属轴保持值及来源证据。只有合成端点测试显式声明未使用轴为零。Elite原SDK自身固定JSONRPC id、单次recv等限制仍被披露，未取得生产执行资格。

Noetic Python3.8 与品牌宿主 Python3.11 分开运行。可从发行内容复制 `native_sdk.py` 到库外，以 SHA 固定 `worker_script`，并显式提供安装态 `sdk_search_paths`；没有隐式查找 uni-lab-assets。Python SDK 的离线测试通过 socket audit 限定 loopback；原生 Linux SDK 测试使用 `podman --network none`。

HIWIN、Panda、Inovo进一步完成了已安装品牌工厂→独立worker→真实SDK的整条调用链。HIWIN与Panda使用安装后的品牌wheel及外部共享库；Inovo以Python3.11品牌宿主调用Python3.8 Noetic SDK，SDK只有catkin安装前缀。Panda初次缺Poco依赖的失败回执保留，补齐匹配运行镜像后通过；没有借源仓导入或替换SDK方法。

维护命令：`bind_native_hardware_adapters.py` 对原53清单更新品牌定义，`record_vendor_driver_validation.py` 写入五个真实驱动及Elite原生回执；`build_estun_eri_bridge.py` / `build_hiwin_commander_bridge.py` / `build_franka_panda_bridge.py` 固定库外构建。后续重新编目保留原生入口，代码或来源变化将证据标为待重验，不自动继承。

## 证据位置

所有原始日志、wire 请求、模块/库 SHA、失败输出、构建产物和测试环境都在仓库外：

`C:/Users/DP/files/asset-pipeline-work/robot-hardware-drivers-20261108/`

- `vendor-offline/*-adapter-receipt.json`：5 个实际 RoboDK 驱动及完整 stdio/wire。
- `vendor-offline/{elephant280,igus_cri,agilebot_a,elite_ec,elephant_pro600_local}-receipt.json`：实际 Python SDK 调用；CRI `-mixed`与Elite `-controller-error`保留额外边界证据。
- `vendor-offline/{estun-native,hiwin-native,inovo-native,flexiv-native,franka-panda}/`：真实库/ROS 构建、调用、正负例、阻塞原始日志。`franka-native/` 保留误用现代 SDK 的排除依据，不用于 Panda 支持声明。
- `native-bindings/native-sdk-53.json`：原 53 逐型号来源、入口、操作、缺项和证据。
- `vendor-driver-five.json`：五个型号的当前真实驱动状态。
- `driver-completion-summary.json`、`driver-models-current.json`：1941个型号的最终资产pin、驱动候选和逐项状态快照。
- `vendor-installed-final-261.json`保留前次全量安装审核；最后74项更新后，`vendor-installed-final-74.json`再次通过130品牌发行包/1941型号/175来源配置/3 provider完整隔离验证。`vendor-installed-final-driver-modules.json`另核对全部最终型号定义及驱动字段，禁止读取仓库源代码。

本轮共25项针对性回归通过。完整SDK/原生库验证单独记录，不用这25个逻辑测试代替真实厂家代码证据。261个新发布资产pin已更新并保留驱动配置，旧asset_adapter规划资格不转移到新资产。

随后增加7项正式资产版本（Fanuc2、Rokae2、Kawasaki3），对应三个品牌wheel已增量重建；7项驱动字段逐字典一致，全部1941个安装态模块再次审核通过。回执为 `binding-refresh-additional-7/receipt.json`、`driver-preservation.json` 和最新 `vendor-installed-final-driver-modules.json`；运行时与真实驱动证据未改变。

最终追加74项已正式发布版本（70项无几何Robot World辅助frame恢复、2项ABB完整VCMX重转，以及MH12/M710iC50E），更新9个品牌发行包。`binding-refresh-final-74/driver-preservation.json`核对全部1941型号driver字段完全不变，74项不继承旧asset_adapter资格，worker SHA仍为 `9828810741d337a350bdb5aed1626b4bc3e6806f86eb55ff319fed1265f1497a`。

统计区分版本修订与型号：原生驱动实施阶段261+7+74共342次已发布版本pin更新，涉及341个去重型号；若包含本次恢复任务初期4个Delta、27组合并及2项事实修订，则累计375次pin更新、369个去重型号。这些不是新增型号数，活动型号仍为1941。

### 全量活动版本核对补充

主独立验收发现，上述增量更新与安装审核只保证安装包等于仓库，没有保证每个固定pin等于资产库活动版本。重新检查全部1941项后，发现另外571处历史旧pin：562机械臂、9工具（OnRobot8、Robotiq1），涉及38品牌。它们是已存在活动版本的锁迁移，不是571次新发布；原版immutable包仍保留。

维护入口 `scripts/reconcile_active_robot_bindings.py` 按相同asset_id取唯一活动包，拒绝缺失或歧义，逐包完整校验摘要，并比较仓库、安装态及活动索引的精确 `asset_id/package_digest/package_path`。使用显式 `verified_local_active_revision_mapping`，不将本地独有资产描述成OSS已发布。正常运行仍按固定锁读取，不自动跟随latest。

对应571项已更新，全部1941型号driver字段精确保持，旧asset_adapter资格不继承。新的证据目录为 `D:/UniLab_asset_evidence/robot-hardware-drivers-20261108/active-reconciliation/`：`before/audit.json`保留遗漏，`applied/receipt.json`与`driver-preservation.json`记录迁移，`after/audit.json`的`--require-current`验收要求三方零差异，`installed-full-audit.json`复核全部发行包与资产。最终结论以此全量核对为准，不再用旧增量审核替代活动版本一致性。

最终验收已通过：1941个活动包完整SHA校验，仓库与活动版本差异0、安装态与仓库差异0；38个变更品牌wheel重新构建安装，130品牌发行包/1941型号/175来源配置/3 provider完整安装审核再次通过。`vendor-effective-wheels.json`固定完整138个有效wheel。新增不完整scope负例在写入前退出，4012个目录/模块/锁文件保持不变。

最后补充已正式发布的 `unbranded_vc/gripper2finger` r002（asset_id `2712a7ff-f287-5933-aa5f-043ab1263198`）。只更新该工具pin及对应品牌wheel，1941项驱动字段全部保持，旧资产资格不继承。最新权威证据为 `D:/UniLab_asset_evidence/robot-hardware-drivers-20261108/final-tool-pin/`：`after/audit.json`再次完整校验1941活动包并确认三方零差异；`installed-full-audit.json`再次通过130品牌/1941型号/175来源配置/3 provider；`driver-completion-summary.json`及`vendor-effective-wheels.json`记录最终状态。活动索引SHA为 `b02ab6ae9656517c37590a7e7950b30daf5d4b681b6ac88f5f8c49fd15c8c7ad`。

包含最后夹爪后，原生驱动实施阶段实际发布pin修订为343次、342个不同资产；连同恢复任务初期33项则376次、370个去重型号。另571次是将既有旧锁对齐已存在活动版本，不计作新增发布。型号总数仍1941，实机资格仍0，worker仍982881版本。

旧的五品牌公共 stdio fixture 验收保留为单元测试证据；不能替代本文件中的实际 SDK/驱动验收。
