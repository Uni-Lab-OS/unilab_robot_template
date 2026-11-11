# 统一资产目录与品牌包

2026-11-11 使用 `unilab.public-entry/2` 的 13 字段合同关联品牌包。公共业务 UUID 不代替工程 `asset_id`，运行仍锁定 `asset_id + package_digest + package_path`，不自动追随最新版本。

## 字段与使用

公共 `module` 保存真实定义工厂，例如 `unilab_robot_dobot:module_definition`。`model.robot_module` 保存精确 `model` 与发行名 `distribution`，`model.targets` 完整保留工程引用、入口、配置和成员信息。品牌 `module.json.public_catalog` 反向保存业务 UUID 和合同版本，品牌目录更新模块 SHA。分类、厂家、来源、关系、资产锁、组合、驱动候选与规划资格不变。

```python
from unilab_robot_contracts.public_catalog import resolve_public_robot
catalog, model = resolve_public_robot(business_uuid, library_root)
definition = catalog.module_definition(model)
asset = catalog.asset(model, library_root)
compiled = catalog.compile_asset_model(model, library_root, external_empty_cache)
```

解析器检查公共条目 SHA、合同、反向身份、品牌模块 SHA、发行包/型号、固定 pin 和资产全包。旧项目仍可独立使用原 BrandCatalog API，不强制读取公共目录；缺包、歧义、错误摘要和旧 pin 拒绝，不模糊匹配、不下载或换版本。

定义工厂不是已获实机资格的 Device 类。图中仍须明确选择执行驱动、安装与标定；工具通过安装接口使用，不作为独立机械臂规划。资产端不复制品牌实现，品牌 wheel 不复制资产几何。

## OS 仿真驱动

`unilab_robot_runtime.public_robot_device:PublicRobotSimulationDevice` 可由 OS DeviceClassCreator / ROS2DeviceNode 创建。参数为 `catalog_uuid`、`library_root`、`compiled_model`、`model_sha256`、库外 `output_root` 和显式 `initial_q`。编译模型必须与目录目标的固定 pin 一致。

`move_joint(q_goal)` 实际调用 MoveIt/FCL，检查轨迹后按源关节 FK 生成带时间采样的 USD，并更新 `joint_positions`；角度为弧度。碰撞、越限和并发请求拒绝。文件写入库外，原资产不变。执行是仿真时钟下的运动学回放，不是动力学、硬件控制或工站工作流验收。领域图持有安装、初始状态、现场碰撞环境和控制器参数；真机仍使用品牌已登记的硬件候选、SDK 和经核验的控制器映射。

## 维护与验收

1. `scripts/link_public_robot_catalog.py --library <库> --output <库外新目录>` 全量预检；`--apply` 保存修改前原文和 SHA 后写入。仅唯一且 pin 相同的目标可绑定，既有不同 module、歧义或重复归属中止。
2. Asset_pipeline 的 `tools/asset-square/unified.py reindex --root <库>` 重建派生索引；不要重新 migrate/backfill 覆盖整理结果。
3. `scripts/build_brand_distributions.py` 库外构建，更新运行环境品牌 wheel 与 contracts/runtime。
4. `scripts/verify_public_robot_bindings.py` 安装态逐项校验，生成五品牌模型并验证错误 UUID、摘要、路径和 asset_id 的拒绝路径。
5. ROS 环境运行 `scripts/validate_asset_moveit.py` 验证关节/位姿规划、FK、FCL 阻挡；`tests/engines/public_robot_os.py` 经真实 OS 创建和 JSON 驱动分发执行往返运动，重开 USD 对照 MoveIt FK，验证阻挡与越限不改变状态。

构建、模型缓存和验收证据均在库外。公共目录本地修订不等于 OSS 发布，硬件资格不因本轮绑定或仿真验收自动升级。

## 导轨机械臂规划入口

```python
from unilab_robot_runtime.ros_environment import native_planning_session
from unilab_robot_contracts.planning import PlanGoal

# provider 属于领域包：从 OS 部署图和资产锁装配整机模型、工具与 CAD 碰撞环境。
with native_planning_session(provider.robot, provider.rail_axis,
                           external_run / 'native', domain_id=94) as planner:
    request = provider.builder.build(
        current_state, PlanGoal.from_joint(q_goal), goal_rail_m=rail_target_m)
    result = planner.plan(request)
    # 由既有领域设备动作处理结果并调用同一执行器；不再创建第二个硬件控制所有者。
```

以上是既有入口，本轮不另建规划实现。请求必须含当前导轨坐标、目标导轨坐标和完整环境/工具/载荷；SceneRequestBuilder 保留这些信息。当前语义为导轨段后机械臂段，整机密集碰撞检查，不是七轴同步冗余优化。无需移入领域点位、现场标定或工艺策略。

生命周期由领域启动器在 OS 初始化 ROS 前建立；OS 设备动作使用该规划器。已运行 ROS 的 OS 进程使用已经启动的 MoveGroupPlanner 服务，不在设备动作中重复启动 native_planning_session。规划、关节模型和碰撞实现归 robot-template；OS 调度与驱动分发、领域装配与执行职责保持原边界。

两个工站此前已通过该链路，本轮验收目的只是资产迁移后的回归。工站测试保留原规划入口，仅支持显式 `--package-root` 以在库外复验原部署快照。CR5/CR7 原项目的 r001 资产锁保持不变，检查 D 盘包及网格摘要，从 D 盘复制到新验收根，并禁止读取旧资产缓存，再复跑原有规划、运动和读回。这个结果不能自动证明另一修订的几何可替换原项目。

## 验收结果

证据根：`D:/UniLab_asset_evidence/robot-public-binding-20261111`，汇总为 `summary.json`。构建与安装环境在 D 盘，未发布 OSS。

- 1,941 项关联通过：1,769 个机械臂、172 个工具，129 个非空品牌发行包；完整包摘要、公共身份、固定引用及仓库/安装态一致。20 个错误绑定负例和 8 项原品牌接口测试通过。关联脚本重复运行改动数为 0。
- Dobot CR5、Elite EC66、ABB IRB1200-5/0.9、FANUC LR Mate 200iC、KUKA KR6 R900 sixx：实际关节/位姿规划、FCL 阻挡、OS 设备创建与 JSON 驱动分发、USD 往返运动及读回通过。共 10 段运动，本地重开通过；USD/原生 FK 最大误差约 1.33e-15。碰撞和越限请求不改变状态。
- 后处理工站 CR5＋导轨、苏州工站 CR7＋导轨：从 D 盘取出的 r001 包在全新远端根目录通过原工站规划、转运、放件与运动读回，两个原生进程均 exit 0、无强杀。旧资产库 Python 文件访问被禁止，生成的 MoveIt 参数中旧资产根引用为 0；两个模型归一化根路径后的 URDF、SRDF 与旧验收完全一致。14 个源网格摘要未变。本地另在禁止 C/E 旧资产入口的条件下完成两 provider 构建。

五个公共资产版本的规划/OS 运动资格已写回品牌模块并更新安装态；旧工站保留原版本锁。上述是运动学仿真与迁移回归，不授予真机、动力学或完整 OS 调度工作流资格。场景大文件保留在远端，摘要与位置记录于 `migration/retained-remote-artifacts.json`，未复制进资产库。
