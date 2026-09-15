# SZLab Mixer CR7 调试卡片

该设备卡片只属于 SZLab Mixer，用于本地后端（Local Backend）机械臂与独立导轨调试。

- 设备属性与关节状态（JointState）通过通用设备遥测（DeviceTelemetry）SSE
  进入 Host Bridge；卡片不自行联网。
- 手动独占（Exclusive）通过 `manual-exclusive` Host 能力调用，不伪装成动作（Action）。
- PointSet 列表、工具中心点（TCP）、关节监视、无单步最大值 Jog、点位设置、`home`、
  `move_to_anchor` 与 `read_debug_snapshot` 都通过设备卡片动作桥调用；危险动作由 Host 强制确认。
  live 卡片挂载时只订阅 Host 状态，不会自动调用 Action。`read_debug_snapshot` 仅在用户点击“刷新”后显式执行，避免打开 FE 就创建隐藏 WorkflowTask。
- 正式后端（Backend）未开放对应能力时，Live 模式关闭失败。
- 生产取放仍走 PLC；本卡片只驱动 MoveIt 资格通道。

一次移动、Jog 或示教开始前必须先显式取得调试控制。点击动作后卡片先释放
手动独占（Exclusive），把设备准入交给正式动作任务；任务到达终态后再尝试恢复
调试控制。

在 Uni-Lab Electron 中把本目录作为设备卡片工作区打开，等待诊断为 `ready`，
再由用户确认安装或仅使用工作区预览。
