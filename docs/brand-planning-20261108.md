# 品牌并集与资产驱动的实际 MoveIt 接入

后续发布接入：4 个 Delta 新模块及 27 组厂家别名合并已完成品牌锁更新，活动目录增至 **1,769 个机械臂、172 个工具**。下文 1,765/172 与别名待审是此前阶段记录；当前驱动匹配与新绑定证据见 [驱动接入说明](hardware-driver-matching-20261108.md)。已有七型号真实 MoveIt 资格与五型号离线驱动协议资格相互独立。

本轮后续范围收敛：品牌目录只允许机械臂与末端工具。已移出 Haas UMC500、KUKA KL1000-2 / KL1500-3 / KL2000 / KL250-3 / KL3000 导轨及 KP2-HV500 变位机，标准资产库中的这些资产保持可用。厂家别名归并后，27 个同型号不同资产绑定保留为合并待审项，未强行替换旧锁。当前活动目录逐项范围审计为 **1,765 个机械臂模块、172 个工具模块**，没有未分类条目、重复 asset_id 绑定或品牌几何文件。无厂家工具按发布来源分为 `unbranded_vc` / `unbranded_robodk`，不虚构硬件厂家、不按通用同名合并。

CR5、CR7、CR12 的 21 个 mesh 已移出品牌源码，改用 `asset-files.json` 的逐文件 SHA 在固定资产包中解析。旧 provider 接受 `library_root`，也可显式设置 `UNILAB_ASSET_LIBRARY`。CR12 使用正式发布的同 ID `cr12__r002`，保留旧运动模型的专有几何表示；主 USD 与旧 provider 的几何等价仍未被推定。

本轮将品牌范围改为 VC、RoboDK、历史 uni-lab-assets 的型号并集。共同型号只保留一个身份；单源型号同样接入。上游 URDF/Xacro 的宏、包装文件和配置变体不能直接算作独立型号。uni-lab-assets 只在维护导入时读取，不参与运行与构建。

## 已执行的五型号闭环

| 品牌 | 精确型号键 | OSS 冷下载 | 关节目标规划 | KDL 位姿目标规划 | FCL 阻挡测试 | 源关节 FK 对照 |
|---|---|---|---|---|---|---|
| Dobot | `cr5` | 通过 | 通过 | 通过 | 检出并拒绝规划 | 通过 |
| Elite | `ec66` | 通过 | 通过 | 通过 | 检出并拒绝规划 | 通过 |
| ABB | `irb1200:5:0.9` | 通过 | 通过 | 通过 | 检出并拒绝规划 | 通过 |
| FANUC | `lrmate200ic` | 通过 | 通过 | 通过 | 检出并拒绝规划 | 通过 |
| KUKA | `kr6r900sixx` | 通过 | 通过 | 通过 | 检出并拒绝规划 | 通过 |

补充三方并集的 UniLab-only 实例：Dobot `cr20`、`me6` 已由原生 URDF 转成同一标准库结构并正式发布；两者也在同一远端实际通过关节规划、KDL 位姿规划、FCL 阻挡拒绝及 FK 对照。通用编译器仅将无几何的固定根坐标系折入基座安装，保留原变换证据；有几何的固定祖先或复杂固定分支继续拒绝。因此本轮实际引擎通过数为 **7**，五个最初型号另完成真实 OSS 冷下载。

实际引擎运行于远端隔离 Podman 容器：ROS Jazzy、MoveItPy 2.12.4、KDL、OMPL RRTConnect、FCL。每个型号实际生成两条 310–311 路点轨迹，每条做 619–621 次 FCL 查询；碰撞检查采用最大 0.02 rad 的关节空间离散采样，不是连续碰撞证明。阻挡测试使用包围机器人工作区域的实体障碍，证实碰撞和无效起点拒绝；尚未把“狭窄通道绕障”列为已验收能力。

源 USD 的 `A · R(q) · inverse(B)` 与实际 MoveIt FK 共检查每型号 12 个连杆姿态，最大矩阵元素误差约 1.34e-15；位姿规划终点矩阵最大误差约 1.11e-6。角度、米制坐标与四元数均从锁定资产恢复。五份冷下载后重新生成的 URDF/SRDF 和全部碰撞网格 SHA 与首次引擎测试一致。

这证明选定资产的运动学、规划及离散碰撞闭环；不授予真机、动力学、厂家 TCP 标定或完整 NVIDIA SimReady 资格。规划速度 0.5 rad/s、加速度 0.5 rad/s² 是显式仿真假设；URDF effort=1 是不用作动力学的占位值。规划末端使用最后一个源刚体坐标系，工站工具 TCP 必须显式绑定。

## 通用实现

- `BrandCatalog.compile_asset_model()`：先校验固定资产清单、BOM、入口和依赖摘要，再从 USD 构建模型；输出必须是库外空目录。
- `asset_moveit.py`：保留两个关节局部坐标系，采用虚拟转轴连杆与固定变换表达 `localRot1`；直接导出每个刚体局部坐标下的完整三角面碰撞网格，不按近似厂家名称套用配置。
- 只默认排除源关节的相邻刚体碰撞，不套用上游任意 `Never` 排除对。闭链、分支、缺失几何、非法关节域等显式拒绝，不生成静态替代品。
- `AssetMoveItPlanner`：复用现有 MoveItPy 规划器，绑定编译元数据 SHA；请求模型必须与缓存一致。FANUC 的控制器耦合矩阵逆变换用于输入与全轨迹原始限位复核。当前是规划后域过滤，不声称 OMPL 已原生支持耦合域搜索。
- Elite 已保存的 CS 系列上游配置不直接套给 EC66；KUKA 即使缺少对应上游 MoveIt 包，也由实际资产关节和几何生成明确的 KDL/OMPL 配置。冻结上游配置继续作为审查资料。
- `fetch_asset()` / `scripts/fetch_robot_asset.py`：只下载锁定修订，校验每个文件及全包后才移入活动缓存。失败留在库外暂存目录并记录原因；没有 latest 回退、凭据硬编码或隐式上传。

## 使用

安装品牌包和 `unilab-robot-runtime[asset-model]`。编译环境需要 OpenUSD；规划环境另需真实 ROS/MoveItPy。生成的缓存可以复制到规划机器，不依赖 USD 或来源 Git 仓。

```python
import unilab_robot_elite as elite
from unilab_robot_runtime.asset_moveit import AssetMoveItPlanner
from unilab_robot_contracts.planning import PlanRequest, PlanGoal

build = elite.compile_asset_model(
    "ec66", library_root="/srv/assets", output="/srv/model-cache/ec66-v1"
)
planner = AssetMoveItPlanner(
    "/srv/model-cache/ec66-v1", expected_sha256=build["model_file_sha256"]
)
# q_start / q_goal 是该模型的弧度坐标，须来自当前场景状态与任务。
request = PlanRequest(planner.robot, tuple(q_start), PlanGoal.from_joint(q_goal))
result = planner.plan(request)
collision = planner.check_collision(request, q_start)
```

可用安装后的 `unilab-robot-asset-model --brand elite --model ec66 --library <库根> --output <外部空缓存>` 编译；库缺失时先用 `scripts/fetch_robot_asset.py --brand elite --model ec66 --cache <库根> --store <OSS配置>`。OSS 配置只保存地址、桶、区域和前缀，凭据由环境变量提供。

五型号资格保存在各自 `module.json` 的 `planning.asset_adapter`，绑定精确资产与默认实例 `robot` 的编译元数据 SHA。改变关节、几何、实例或编译配置后要重新构建并按用途复验，不能借用旧摘要。

## 并集维护与失败

`scripts/import_brand_catalogs.py --crosswalk <三方清单> --scope-report <机械臂/工具逐来源分类> --library <正式库> --receipt <库外回执>` 走增量并集导入；不再覆盖已有品牌模块。旧 bootstrap 导入在已有目录时拒绝，避免抹掉手工接通状态或旧锁。来源交叉表只用于维护，品牌发行本身不读三方仓库。

第一轮补入 1,183 个正式库中已有可动关节的单源条目，品牌目录暂为 112 品牌、1,784 条模块（含原有工具/轴条目，并非全是六轴机械臂）。新增条目仅验证包完整性与可动 USD 关节，不直接授予规划资格。三方源清单中的 399 个 UniLab 单源键仍需去除别名、包装和模板后转换；不把未发布原件编成虚假的可执行型号。

以上是范围收敛前的阶段计数，不能当作当前型号总量。来源韦恩统计现仅包含 VC `Robots` / `Robot Tools`、RoboDK `robots` / `tools` 与实际 URDF 结构确认的机械臂/末端工具；厂家显式别名先归一。未展开宏、测试包装、不完整 URDF 单列待核。报告脚本 `Asset_pipeline/tools/asset-library/report_robot_source_union.py` 输出七个互斥区域和逐品牌表，按正式库 BOM 中的原件 SHA 重新计算覆盖，不能使用候选 staging ID 冒充已发布。

`scripts/compile_brand_models.py` 将通用适配推广到全部已绑定条目。编译与引擎资格分列；每型号独立进程，120 秒超时，保存失败日志并支持续跑。库外缓存允许保留失败诊断，品牌包不携带这些中间产物。

范围收敛前的 1,784 条编译批次全部完成：1,348 通过、436 拒绝。该数字保留在原回执中，不将后来移除/归并的条目重复计作当前型号，更不等同于五型号以外的引擎资格。

## 本轮证据

工作根：`C:/Users/DP/files/asset-pipeline-work/robot-brand-planning-20261108`。

- `oss-cold-cache-receipt.json`：五型号真实 OSS 下载文件数、字节数与固定绑定。
- `cold-compile-receipt.json`：冷下载模型与首次引擎输入一致性。
- `engine-final/summary.json`、五个 JSON/原生日志：实际关节/位姿规划、轨迹、FCL 拒绝与 FK 误差。
- `union-01.json`：三方并集逐项新增、未发布、别名候选和拒绝状态。
- `union-compile-01/progress.jsonl`、`receipt.json`（结束后生成）：逐型号编译与缺口；不能等同于引擎通过数。

所有缓存、源快照和运行日志位于仓库外。品牌包保留小型运动模型声明、限位、配置、资产锁和兼容代码；几何从固定资产库读取。
