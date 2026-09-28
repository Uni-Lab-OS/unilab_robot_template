# 珞石 xMateCR12 型号包

`unilab-arm-rokae-cr12` 使用现有 CR7 包的目录、正解及静态 MoveIt 模型接口结构，独立保存珞石型号事实。它与越疆 CR5／CR7 不共用厂家型号参数，也没有复制越疆的 PLC、TCP 或 MoveIt 执行工厂。

```python
from unilab_arm_rokae_cr12 import MODEL_DESCRIPTOR, build_moveit_model

bundle = build_moveit_model(device_id="robot_a")
```

公开 Provider 为 `unilab_arm_rokae_cr12:build_moveit_model`。参数与既有接口一致：世界位置为毫米、旋转为弧度，可选安装偏航为度且只生成固定关节。六个规范关节名为 `rokae_cr12_joint_1` 至 `_6`，所有实例 link、joint、控制器由 `device_id` 限定。设备图不在本包；本批没有创建或修改任何部署图。

`MODEL_DESCRIPTOR` 提供关节顺序、SI限位及基座到末端模型 link6 的正解。身份工具变换只表示源模型端点，不证明厂家法兰、实际 TCP 或安装标定。反馈映射仅声明 `canonical`；未声明厂家 SDK/PLC 字段对应关系。

## 来源与派生

包内保留13份输入：旧实验冻结模型、7个网格、几何/惯量两份派生源、厂家SRDF、厂家xacro与原许可证。7个网格和3份厂家资料逐字对照从备份恢复的上游 Git `d6a508c38513d4c39b2f10f14275d0e26df7fb43`。数据使用该来源保留的许可，见 `models/source/ROKAE-LICENSE`，不将厂家资料改称本项目原创。

主URDF是旧实验 `rokae_xmate_cr12` 的冻结派生文件，摘要为 `091fbc1fa23963e01c5a68f5aba7b6b1bf63b493530403371d771407d104b7ea`；不是直接声称它等于厂家xacro。`model.yaml` 分别记录派生文件摘要和厂家Git版本，`source-lock.json` 保存逐文件来源。原URDF、网格与原始资料均保留字节。只重放旧 `assets.py::prepare` 的 CR12 循环体，确认主URDF和7网格相同，未重放CR7或场景复制。

Provider沿用既有双URDF接口：执行模型使用包内 `file://` 网格路径，渲染模型使用实例相对URL。它仅添加标准固定安装根、命名空间和 `mock_components/GenericSystem` 声明。六轴几何、惯量、effort及velocity保持珞石源值，MoveIt速度上限从源URDF逐关节读取；12条自碰撞排除规则来自该型号的厂家SRDF，没有套用越疆的 `Never` 列表。

## 已验证与未验证

新包与CR5／CR7共27项CPU合同测试通过，包含双实例隔离、源/网格/SRDF漂移拒绝、固定安装不增加运动轴、64组姿态与冻结URDF参考正解的一致性。包内13份原始输入及旧实验保持原样。记录见 `migration/rokae-cr12-model.json`。

本包当前是**型号与静态模型入口**，不是完整机器人执行模块。没有运行MoveIt、ROS或Isaac，没有加载厂家硬件插件、调用硬件、构建或安装distribution；不能据此宣称规划、碰撞安全、实物限位、动力学、负载、接触或厂家控制器资格。CI登记安装路径只供以后执行，本轮未触发CI。

已有CTAG夹爪另有独立型号来源。机械臂与夹爪、导轨和工站的实际装配仍由领域包现有部署图负责，不能把旧组合URDF直接作为新的合一体MoveIt设备。本批未修改领域包，领域装配检查器不适用。
