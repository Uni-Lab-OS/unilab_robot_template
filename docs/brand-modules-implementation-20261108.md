# 品牌机械臂模块落地回执

本轮落实用户批准的 C 方案，并按补充要求把 `unilab_robot_template` 作为去重机器人模块的维护入口。运行/构建不依赖 `uni-lab-assets` 检出；它只用于本轮显式导入历史资料。

## 完成内容

- 47 个品牌发行包，601 份型号模块定义：对应本轮锁定的 600 个已合并资产条目（含机械臂、工具、外部轴等源类型），加独立的 Rokae xMateCR12 provider 条目。此数量不是 601 个已完成真机接入的机械臂。
- 每型号包含 `module.json` 和 `assets.lock.json`，保存组合资产引用、源运动参数、行为绑定、驱动/规划状态、表示对应及来源；不只是名称目录。
- 175 套上游 MoveIt 配置收进品牌包，连同相关包内文本、许可证及 SHA 清单。已知型号名可建立候选关联；原 launch、SRDF、控制器配置不会自动被执行或授予能力。
- CR5、CR7 实现归入 `unilab-robot-dobot`；CR12 归入 `unilab-robot-rokae`。旧三个发行包成为依赖兼容壳，旧 Python 模块名仍能使用。64 个原有跟踪文件与 Git 原字节一致，没有重新生成 URDF 或改变算法。
- 品牌 API 提供 `models`、`model`、`module_definition`、`build_moveit_model`、`resolve_asset`。未知型号、无 provider、篡改模块/配置、包身份不符和路径越界均拒绝；无隐式 latest 或上游检出回退。
- Asset_pipeline 的依赖及 uv 锁改为品牌包；postproc 仿真依赖改为品牌包，URDF 转换入口从已安装模块解析模型路径，保留显式维护覆盖参数。
- 当前 `AGENTS.md` 与 README 已更新，旧技能的一型号一发行包结构不再作为约束。

## 实际验收

工作与构建根：`C:/Users/DP/files/asset-pipeline-work/robot-brand-packages-20261108`。

| 检查 | 结果 | 证据 |
|---|---|---|
| 原 provider 代码与模型迁移 | 64 个跟踪文件逐字节相同 | `provider-migration.json` |
| 品牌/型号/旧接口/组合规划合同 | 80 项通过 | `final-contract-tests.log` |
| 原型所有发行包构建 | 55 个 wheel 成功 | `build-01/receipt.json` |
| 原型独立安装，禁止读整个 repos | 47 品牌、601 型号、175 配置、3 provider 通过 | `prototype-isolation.json` |
| 包含完整 module.json 的最终重建 | 55 个 wheel 成功 | `build-02/receipt.json` |
| 最终独立安装与资产解析 | 601 模块定义、175 配置摘要、三型号双实例/旧入口等价与固定资产包校验通过 | `final-isolation.json` |

独立环境仅安装 wheel 和 PyYAML，用 `python -I -B` 执行，并以 Python audit hook 拒绝 `E:/Design_projects/Lab-OS-Testbed/repos` 下所有文件读取。可执行模型来源于安装目录，不来自品牌源码或 `uni-lab-assets`。模型调用不启动 RViz、商业软件或硬件。该验收不是实际 MoveIt 规划、Isaac 动力学或真机验收。

CR5/CR7 与资产内 UniLab 原 URDF 的活动关节声明一致；选定 RoboDK USD 与 provider 全几何/坐标对应仍未验证。Rokae CR12 不冒充其他后缀型号的对应关系。

## 尚未完成的能力

- 除现有三个 provider 外，其他模块仍须把候选运动/MoveIt/控制器资料适配成可执行实现；已有条目中没有驱动就明确记为 `not_implemented`。
- 175 套源配置已经归品牌持有，但其 ROS 版本、外部插件、几何闭包和具体型号对应尚未全部验证，不能把“已保存”当成“可规划”。
- 库侧 subagent 正进行 600 个旧资产分层和新增资产修复，本轮品牌锁仍指向已经发布的旧固定修订。新修订正式发布后更新锁、审查变化并复验，不从候选目录悄悄切换。
- 新的导轨组合安装和工站标定继续归领域包。本轮保留现有组合协调合同，未把轨道改成机械臂第七轴。

## 既有环境问题

默认 pytest 自动加载的 `launch_testing` 与本地 pytest hook 不兼容；本轮合同测试使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，没有宣称运行 ROS launch 测试。扩展检查发现既有 dirty 导轨实现与 `test_rail_carriage_visual_sits_above_the_bed_section` 的 visual 预期不一致，未为品牌迁移修改该实现。根仓来源检查仍报告既有 `repos/Uni-Lab-SZLab` 非法活动仓记录，见 `repository-check.log`。

本轮未提交或推送 Git，也未切换现有工站运行服务；不回滚其他并行任务的 dirty 文件。所有构建暂存和安装验收环境位于库外工作目录。
