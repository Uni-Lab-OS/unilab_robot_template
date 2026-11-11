# 品牌型号的在线驱动候选与离线适配

2026-11-08。初阶段给当前每个品牌型号写入可追溯驱动候选，并接通品牌工厂与共享 RoboDK 控制台协议适配器。随后已实际执行厂家 SDK 与已安装 RoboDK 驱动，当前结果和阻塞以 [原生驱动离线验收](vendor-native-adapters-20261108.md) 为准。**型号名称、驱动文件存在、离线测试均不授予实机资格。** 未连接真实机器人、扫描设备地址或运行 RoboDK GUI。

## 范围与当前结果

资产侧的 4 个 Delta 新型号、27 组规范身份合并已接入品牌锁，当前为 **1,941 个模块：1,769 个机械臂、172 个工具**。固定资产的更新使用正式发布回执与独立范围分类，保留已有 provider 和驱动字段；旧资产摘要绑定的规划资格失效，不能自动转移给新资产。

随后 MPK2、MPL300 两份厂家事实元数据修订也已更新品牌锁；型号数量不变，旧定义和资格处理保留在 `binding-refresh-round7/receipt.json`。

| 当前模块的互斥类别 | 数量 | 含义 |
|---|---:|---|
| RoboDK 在线驱动与 SDK/ROS 候选均有 | 509 | 两条候选路线，未证明具体控制器兼容 |
| 只有 RoboDK 在线驱动候选 | 716 | 外部安装依赖，品牌包不分发其文件 |
| 只有含实际代码的 SDK/ROS 候选 | 53 | 原名单保留；53 条已有原生族入口，23 条对应实际 SDK 正负例已跑的族，30 条仍有阻塞；不是逐型号实机支持 |
| 只发现离线后处理器 | 248 | 不计入在线驱动覆盖 |
| 未匹配 | 415 | 包括没有专用驱动证据的工具，逐项保留原因 |

在线驱动候选并集为 1,278：RoboDK 1,225，SDK/ROS 562，交集 509。这些不是可直接执行的型号数量。初阶段五个选定绑定通过公共协议 fixture；后续实际厂家代码的受控端点闭环为 Dobot、KUKA 的已安装驱动及 Elite 官方EC SDK，共3个。Elite RoboDK入口仍保留MOVJ无Ready问题，ABB/FANUC缺匹配控制器服务端；所有型号的 `model_controller`、`live_connection`、`hardware_execution` 仍未验证。

来源盘点的 121 个仓库中，50 个有实际文件、71 个为空或仅 Git 元数据；43 个含代码、46 个发现许可证文件。这些统计包含模型/配置仓库，不能直接作为 SDK 数量。匹配脚本只取明确的 SDK/driver 仓库集合，空仓不计入实际代码覆盖。

## 与原 964/1,468 口径的对应

读取同一份 RoboDK 1,468 条机器人 manifest，在本机 Windows 安装 `C:/Program Files/RoboDK/api/Robot` 中，宽泛品牌族文件对应为 **982**。与旧 macOS 记录 964 的差额是本机发现的 Doosan 驱动族 18 条。

进一步按系列过滤为 **971**：移出 9 个 Omron 非 TM 型号，以及不能凭当前证据对应的 Dobot M1、Magician Lite。Techman 型号本身的 `Techman TM...` 前缀被显式识别；不会把这些 30 条误排除。两种口径都只是驱动族候选，均不是实机验证。`robodk-1468-comparison.json` 保留每一条来源及选择结果。

## 五品牌离线验证

| 品牌 / 型号 | 选定外部候选 | 必须由部署确认的条件 | 本轮资格 |
|---|---|---|---|
| Dobot CR5 | `DobotV3Driver.py` | V3 控制器；本地 V4 SDK 不适用于它 | 协议适配通过 |
| Elite EC66 | `EliteRobotsDriverEC.py` | EC 控制器；CS SDK 不适用于它 | 协议适配通过 |
| ABB IRB1200-5/0.9 | `ABBDriver.exe` | RAPID socket 路线，PC Interface 与对应机器人端程序 | 协议适配通过 |
| FANUC LR Mate 200iC | `FanucSM.exe` | 控制器匹配的 socket、PC Interface、KAREL 选项及机器人端程序 | 协议适配通过 |
| KUKA KR6 R900 sixx | `kukabridge.exe` | 明确 KRC 版本及对应控制器端 bridge；不能套 IIWA/Sunrise | 协议适配通过 |

测试使用独立子进程作为离线协议对端，每个品牌工厂均实际完成：初始化握手、当前关节读取、SI 弧度转换、六轴及位姿参数编码、Working→Ready 状态转换、完成后关节回读、控制器错误、超时隔离、Ready 但位置不符的拒绝、重复命令拒绝与未知对账。另有仿射耦合、轴交换/符号/偏移、非法数值、奇异映射、版本摘要及路径边界测试。

测试坐标是显式合成数据，**没有证明五个型号的实际零点、轴方向、耦合标定或厂家驱动内部行为**。这里的“五个通过”是同一公共适配器在五个精确品牌绑定上的测试，不是五套厂家驱动或控制器仿真器分别通过。测试记录保存 `vendor_driver_executed=false`、`hardware_connected=false`、`hardware_motion=false`。

## 实现与接口

`unilab_robot_runtime.hardware_driver` 提供共享适配器，不按品牌复制协议代码。品牌模块导出：

```python
import unilab_robot_dobot as dobot

candidates = dobot.hardware_candidates("cr5")
driver = dobot.create_hardware_driver(
    "cr5", "robodk:DobotV3Driver",
    mapping=commissioned_joint_map,         # 显式标定矩阵、偏移、限位与证据
    controller="Dobot V3",
    deployment_evidence=deployment_record,
    robodk_root=installed_robodk_root,
    python_executable=compatible_driver_python,
    targets=deployment_allowlisted_targets,
)
# 构造不启动进程、不开网络。connect 是调用者另行选择的部署动作。
```

`ControllerJointMap` 不按品牌默填 identity；调用者提供模型到控制器SI坐标的可逆矩阵及偏移。原生边界支持显式逐轴rad/deg/m/mm映射；mixed配置只在有来源的Igus/Estun单位声明下启用，缺证型号保持阻塞。RoboDK degree-only边界仍拒绝直线轴。`MOVJ` 位姿参数须由已标定部署模型按驱动约定提供，不填零掩盖TCP/基座缺口。Igus/Agilebot/Elite/Estun未归属轴也必须显式给保持值或未使用轴证据。

运行只解析明确安装目录中的驱动，校验已记录文件 SHA。Python 包装器还校验实际解释器版本对应的编译入口 SHA；依赖的全部厂家 DLL 闭包尚未验证。无安装目录、版本变化、SDK 候选尚无实现、控制器选择不符、轴数不符均拒绝。品牌 wheel 不复制 RoboDK/厂家二进制，不查找 `uni-lab-assets` 源仓。

`execute_target/query_command/request_stop` 可接入已有受限 `RobotSDKPort`。目标必须出现在部署白名单且与解析的关节目标一致。支持的命令是关节目标，未实现线性/圆弧路径、实时轨迹流、IO、工具协议和通用停止确认；`request_stop` 返回未确认，不能当成急停。轨迹时序、场景校验及硬件互锁仍由既有运行层承担。本接口不自动把 MoveIt 路径转换成真机执行。

超时或协议错误后关闭并隔离进程，不自动重发；关闭进程不表示机器人停止。适配器只保存进程内单段回执，不能据此证明重启前或整个多段 OS 命令已完成。

## 维护和证据

- `scripts/match_hardware_drivers.py`：读取显式来源目录和 RoboDK 安装，维护每个 `module.json` 的 `drivers.hardware_candidates`、控制器条件、依赖摘要、来源许可证文件摘要及排除原因。重新匹配不提升硬件资格。
- `scripts/validate_hardware_adapters.py`：生成五品牌逐例协议场景、收发记录与结果，可在已安装 wheel 的环境中复验；只有 `--record` 才写入五个具体模块的离线资格。
- `scripts/update_published_robot_bindings.py`：消费精确正式发布映射，更新资产锁，库外保存旧定义与失效资格。
- `tests/test_hardware_driver.py`、`test_driver_matching_rules.py`：协议失败路径和系列排除的回归检查。

库外证据根：`C:/Users/DP/files/asset-pipeline-work/robot-hardware-drivers-20261108`。逐型号结果在 `matching-final/model-driver-matches.json`；逐品牌/来源数量在 `matching-final/matching-summary.json`；SDK 文件与许可证摘要在 `matching-final/sdk-source-evidence.json`；五品牌完整收发记录在 `offline-five/`；资产锁迁移在 `binding-refresh/receipt.json`。详细报告与缓存不进入标准资产包或品牌源码。

20 项协议/系列/目录回归检查通过，原始结果在 `tests.xml`。一次复跑漏设显式资产库环境变量造成两个旧 provider 测试失败，保留于 `tests-missing-library-env.xml`；设置原要求的库路径后复验通过，未改成隐式回退。138 个发行包构建完成；新建的隔离环境验证了 130 个品牌目录、1,941 个模块、175 个来源配置及 3 个旧 provider，检查时禁止读取整个源码 repos。安装后的五品牌协议复验也全部通过，见 `installed-offline-five-final/summary.json`；它仍是离线协议测试。最终 wheel 摘要和无品牌几何检查在 `effective-wheels.json`。

## 依据与边界

RoboDK 在线驱动通过控制器接口读写真实机器人；后处理器是离线代码生成，两者分开统计。[官方说明](https://robodk.com/doc/en/Robot-Drivers.html)

控制台适配依据官方 stdin/stdout 协议，包含角度、关节反馈和命令状态约定。[协议文档](https://robodk.com/doc/en/PythonAPI/driver.html)

Dobot V3/V4/MG400 与控制器系列的对应依据官方驱动表。[Dobot 驱动](https://robodk.com/doc/en/Robots-Dobot.html)

ABB、FANUC、KUKA 的机器人端组件和选项需分别部署，不能由机械臂模型名推定已经安装。[ABB](https://robodk.com/doc/en/Robots-ABB.html)、[FANUC](https://robodk.com/doc/en/Robots-Fanuc.html)、[KUKA](https://robodk.com/doc/en/Robots-KUKA.html)

Elite 本地开源 Python SDK 的 README 明确面向 CS，且对 2.13.x/2.14.x 有最低版本要求；本轮保存该 README 和许可证 SHA，未套用于 EC。[Elite CS SDK](https://github.com/Elite-Robots/Elite_Robots_CS_SDK_Python)

发现许可证文件不等于批准再分发全部依赖。本轮不复制这些 SDK 或专有驱动进品牌包，保留来源和外部依赖边界；真正部署时仍需匹配控制器、固件、选项、标定及现有 HardwareProfile 的验收要求。
