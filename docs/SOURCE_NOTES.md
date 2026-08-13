# 来源与限制

## 一般设备接入规范

用户提供的飞书文档：

`https://dptechnology.feishu.cn/docx/YRZtdUKkJo1zu4xgjohcc498neh`

本环境访问时跳转企业登录，无法读取正文。没有对不可见内容作猜测。仓库中已有
`Uni-Lab-SZLab/docs/CONFORMANCE.md` 记录相同限制，并采用以下可验证基线：

- `Uni-Lab-OS/.cursor/skills/add-device/SKILL.md` 的自包含设备接入规则；
- `Uni-Lab-SZLab/schemas/device-template-v2.schema.json`；
- `Uni-Lab-SZLab/schemas/profile-v1.schema.json`。

本模板继承的通用要求包括：

- 外部设备包；
- `@device/@action/@property + @topic_config`；
- 稳定英文 action 名；
- 用户物理单位；
- Profile、资源拓扑、连接引用和输入绑定；
- 默认不连真机、不含凭据；
- schema、注册表和工作流检查。

获得飞书权限后，应逐条对照正文并记录差异，不得把当前 fallback 描述成飞书原文。

## 厂家仓库

本地厂家证据来自 `uni-lab-assets/robots/*` 的固定 gitlink。模板没有复制厂家源码，只引用其公共 API。

DOBOT 第一轮重点核对：

- `robots/dobot/python_v4` 固定提交 `55ec1ec` 的 `DobotRobot`、V4 feedback、
  `RunScript/MovJ/MovL/Stop`；
- `robots/dobot/ros2_v4` 固定提交 `37730d0` 的 `cr_robot_ros2`、
  `dobot_msgs_v4` 和 `cr5af_moveit`；
- ROS bringup 从 `IP_address/DOBOT_TYPE` 环境变量取值，状态消息只有
  `is_connected/is_enable`，模板已按该局限保守处理。

## pTLC 点位模型

本地 `Uni-Lab-pTLC` 仓库用于确认已有项目的点位组织方式：

- `packages/ptlc_station/ptlc_station/config/points/robot/robot_points_meta.json`
  同时保存直接点、`group-rack` 三锚点 4×3 阵列、每槽六维修正和基点偏移派生点；
- `MIGRATION_TREES.md` 记录 74 个直接示教点、12 个阵列槽位和 153 个补充派生点；
- workflow 入口/出口使用 `require_anchor`，默认示例容差包含 2° 关节、5 mm 位置和
  5° 姿态。

本模板继承“直接点 + 仿射阵列 + 基点偏移 + 锚点验证”的模型，但没有把 pTLC 的
`move_to_point` 直接作为生产 UniLabOS 动作。新增了四元数姿态约束、修正上限、部署前
物化、内容哈希和审批/回读要求，并保留 DOBOT 直接示教点的可选 `joint_deg`。

pTLC DOBOT 直连实现来自其冻结迁移源提交
`c65b34a8839ebb13fc86701e420b2734d6c4cfa6`：29999 命令、30004 feedback、22000
报警，运动按 `CurrentCommandId` 等待并区分断线/暂停/其他控制端。当前
`packages/ptlc_station` 中的 `robot` 动作是迁移合同，函数体仍为
`NotImplementedError`；因此模板把真实点位序列执行明确留在站点
`dobot_host_action_runner`，未注入时拒绝运动。

## UniLabOS ROS 2 模式

ROS 2 接入判断来自本地 `Uni-Lab-OS`：

- `unilabos/device_mesh/resource_visalization.py` 使用 Python `LaunchService`；
- `unilabos/ros/main_slave_run.py` 统一初始化 ROS context 和 executor；
- `unilabos/ros/nodes/base_device_node.py` 向设备注入 ROS node 并调用 `post_init`；
- `unilabos/devices/ros_dev/moveit_interface.py` 展示如何复用注入节点创建 ROS
  client；
- `ResourceMeshManager` 和 `JointRepublisher` 展示 Action/属性与状态转发。

因此模板不调用 shell `ros2 launch`，也不在 driver 内重复 `rclpy.init()`。

## UniLabOS 转运和 Scheduler

- `unilabos/ros/nodes/presets/host_node.py` 明确
  `transfer_resource` 是系统记账，不含物理搬运，并给出机器人
  `pick → place → transfer_resource` 顺序；
- `unilabos/scheduler/resource_lock.py` 的 live lease 默认只支持 exact device；
- `unilabos/scheduler/python_fallback.py` 声明
  `resourceKinds=["device"]` 且 Layer B/PlannedOccupancy 不支持；
- 独立 `uni-lab-scheduler` 的 `TransferEntry/TransferScheduler` 只形成排程，不提供
  实时区域控制或执行许可。

据此模板没有把 OS Scheduler 描述为 Cell Controller。

## 标准

标准页面只用于确定范围、版本和责任边界；完整项目仍需购买/获取适用标准全文并由合格人员执行风险评估和验证。
