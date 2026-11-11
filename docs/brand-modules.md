# 品牌机械臂模块与标准资产库

2026-11-11：与 13 字段统一目录采用固定 pin 双向关联；字段、解析 API 和 OS 仿真驱动接入见 [统一目录绑定](public-catalog-binding.md)。

实际厂家 SDK/驱动接入、原53个SDK-only型号的原生入口及未通过项见 [原生驱动离线验收](vendor-native-adapters-20261108.md)。族接口验收、具体型号控制器兼容和实机资格分别记录。

2026-11-08 后续落实：型号范围采用 **VC、RoboDK、uni-lab-assets 三方并集**，不是仅重合型号。单源型号同样转换入标准资产库；同身份的多来源合并，别名和宏包装另行归一。当前实际规划接入、逐型号状态和新 API 见 [品牌规划闭环](brand-planning-20261108.md)。

用户于 2026-11-08 确认：本仓作为去重后机器人模块的维护入口，每品牌一个发行包，内部维护所有已接入型号及其组合资产、驱动、运动模型和 MoveIt 配置。标准资产库承担固定机械表示的发布、缓存与设计使用。`uni-lab-assets` 退出运行和构建依赖，只用于明确调用的历史来源导入。旧技能目录的包结构规则不再约束此方案。

## 维护边界

- 品牌包：型号模块定义、固定机械资产引用、运动参数、控制器映射、驱动适配、MoveIt 配置和逐项能力状态。
- 公共 robotics 包：合同、通用规划/碰撞查询、轨迹和导轨组合协调；不按品牌复制。
- 标准资产库：原 CAD/VCMX/robot 等必要来源、NVIDIA USD 分层、几何材质与物理表示、机械接口与逐项验收摘要。
- 领域包：本工站安装/标定、导轨与工具实例、工位点、控制器地址及工艺。OS 继续持有物料、调度、占用与结算。

资产大文件由 `asset_id + package_digest + package_path` 精确绑定，按本地库/缓存提供；品牌 wheel 不复制整套模型大文件，也不扫描上游 Git 树。一个运行安装只需要品牌 wheel、公共依赖和锁定资产包。品牌发行可与固定资产包一同离线分发。不能仅凭文件夹或型号名称替换旧绑定。

批量维护后须运行 `scripts/reconcile_active_robot_bindings.py --require-current`，核对全部型号的仓库锁、安装态锁与本地活动索引三方一致，并逐包校验完整摘要。此前仅按发布增量更新漏掉571个历史旧pin（562臂、9工具），所以安装包与仓库一致不足以证明已对齐活动资产。活动版本迁移是显式维护动作；既有项目锁和immutable旧包保留，运行时不自动跟随latest。本地独有资产不能被标记为OSS已发布。

## 当前结构

```text
packages/unilab-robot-dobot/
  pyproject.toml
  src/unilab_robot_dobot/
    catalog.json
    models/cr5/module.json
    models/cr5/assets.lock.json
    models/cr7/...
    source_profiles/<原配置包及来源标识>/
      source.lock.json
      config/...
      launch/...
      package.xml
  src/unilab_arm_cr5/...  # 旧导入名，只有这一份实现
  src/unilab_arm_cr7/...
```

CR5、CR7 实现已移入 Dobot distribution；Rokae CR12 移入 Rokae distribution。旧三个 `unilab-arm-*` distribution 只依赖对应品牌包，不再拥有源码。旧 Python 模块名保留以兼容 provider 字符串、package URI 和已有领域包；新代码通过品牌 API 选择型号。不能在旧目录维护第二份实现。

`module.json` 的 `composition.root_asset` 是可复用装配引用，内部连杆/关节仍由选定资产定义；它不是新 OS 工站图，不包含虚构的现场导轨或工具安装。模块还保存源运动参数、原行为绑定、驱动实现状态、规划 provider、候选源配置以及对应关系。品牌目录只收机械臂与末端工具，`asset_role` 明确为 `arm` / `tool`；独立导轨、变位机、底座、底盘、机床和场景留在标准资产库/领域装配。外部轴组合的通用代码不因此删除。

## 使用

```python
import unilab_robot_dobot as dobot

definition = dobot.module_definition("cr5")
asset_dir = dobot.resolve_asset("cr5", library_root)  # 显式库或缓存根
bundle = dobot.build_moveit_model("cr5", device_id="transfer_arm")
```

`resolve_asset` 验证固定 NVIDIA 包和文件摘要，不自动跟随 latest、不查询上游仓。新增显式 `fetch_asset` 和 OSS 下载脚本用于冷缓存，所有文件校验后才激活。`build_moveit_model` 保留旧 provider 入口；新的 `compile_asset_model` 从锁定 USD 的实际几何和关节生成库外缓存，经 `AssetMoveItPlanner` 接入真实 MoveIt。两套运动表示不因同型号而自动视为等价。

当前 CR5/CR7 在包内的活动关节声明与合并资产保存的 UniLab 原 URDF 一致，此对应在模块中逐项记录；选定 RoboDK USD 与 provider 的全几何/坐标对应仍是 `not_verified`。Rokae CR12 只绑定明确的 CR12 原资产，不与 Dobot CR12 或带不同后缀的 ROKAE 型号强合并。

## MoveIt 与驱动接入

2026-11-08 驱动侧新增逐型号候选、外部 RoboDK 控制台适配与五品牌离线验证，见 [驱动匹配与证据边界](hardware-driver-matching-20261108.md)。品牌 API 提供 `hardware_candidates()` 与 `create_hardware_driver()`；候选匹配、离线协议测试与实机资格分别记录，SDK/ROS 只有源码候选时不提供虚假的可执行工厂。

`source_profiles` 保存上游配置和包内相关源码的冻结副本、许可证及逐文件 SHA，允许没有上游检出时检查和进一步适配。它们是候选输入，不自动执行。相同型号名仅建立候选关联；ROS 版本、外部插件、package URI、运行依赖和控制器须分别解决。

接通一个型号按以下顺序完成：

1. 固定资产/原工程配置和候选来源，确定实际模型变体、关节与坐标对应；对应不完整则保留缺项。
2. 在品牌内维护模型 provider、SRDF/IK/限位与型号专属差异；通用规划器设置使用共享运行时。
3. 解析所有执行期依赖，只能使用本仓发行包、明确的第三方安装依赖或锁定资产；候选源 launch 不直接变成产品启动入口。
4. 校验多实例命名、FK、完整碰撞模型、轨迹与故障返回；更换碰撞几何后重验 SRDF 排除对。
5. 分别记录无引擎合同验证、真实规划器、物理执行和控制器/硬件资格。无驱动或无模型映射时保持不可执行，不返回模拟成功。

完整型号维护不等于全部硬件已经接通：本轮保留已有 CR5/CR7 adapter 和 CR12 模型 provider；其他条目明确记录尚无可执行 provider/driver。原始 VC 行为保存不等于全部回调已移植。

## 维护工具与复验

- `scripts/import_brand_catalogs.py --crosswalk <三方并集清单> --scope-report <机械臂与工具分类报告> --library <固定库> --receipt <库外回执>`：增量补充已发布的范围内资产，保留已有型号、配置和锁；活动机构缺关节时拒绝，源定义为固定几何的工具可绑定但不声明运动能力。已有品牌目录时旧 bootstrap 全量写入被拒绝。
- `scripts/build_brand_distributions.py --work-root <库外新目录>`：外部暂存、构建所有 wheel，记录摘要，不在源码树堆放构建产物。
- `scripts/verify_brand_installation.py`：用独立环境 `python -I -B` 运行，`--deny-root` 指向整个 repos，实际禁止读取源码仓；验证品牌/模块/源配置、三个 provider、多实例、旧入口等价及固定资产解析。
- `tests/test_brand_catalog.py`：未知型号、未验证 provider、篡改源配置、越界资产路径等拒绝路径。

正式库存在其他本地独有资产；品牌导入只读取库索引，不上传、删除或改写 LH05 等资产。后台通用转换发布的新修订不自动改变已锁定品牌模块，更新需重新导入、审查对应变化并复验。

便携 NVIDIA 验证器位于 contracts 的 `_asset_package.py`，来自 Asset_pipeline 同名独立读取器的逐字节副本；执行不依赖该仓。更新该读取器时同步源摘要并重新跑包篡改/冷缓存验证。
