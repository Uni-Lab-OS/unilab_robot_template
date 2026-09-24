# 卡片规格

## 双模式（Preview / commissioning）

同一张 template 卡服务 Preview 臂、MoveIt 臂与 pTLC 式调试：

| 模式 | 判定 | UI |
|---|---|---|
| `preview` | Host 注入的 `allowedActions` **不含** `read_debug_snapshot` | `read_preview_snapshot` 加载库位目录；Jog（`set_joint`）/ MoveJ / home / stop / pick / place |
| `commissioning` | **含** `read_debug_snapshot` | MoveIt/pTLC 调试台；Jog / 视觉等按 `allowedActions` 再隐藏 |

- template `card.manifest.json` 的 `permissions.actions` 为 MoveIt + Preview **并集**；Host 按 Graph 设备 `@action` 收窄后写入 `getContext().config.allowedActions`。
- 领域仓只需 `templateCard` 引用 + 可选 `branding.ts` overlay；**不写**模式字段。
- Preview 设备**不需要** `RailMountedArmCardMixin`；MoveIt 设备仍需要。

## SZLab MoveIt 实例

- 设备类型：`community.szlab_poly_studio.szlab_mixer_robot`
- 调试实例：`szlab_mixer_robot`
- Host Protocol：`1`
- Host 能力：`core`、`manual-exclusive`
- 实时状态：`online`、`actionBusy`、`moveit_online`、`jointState`
- 只读动作（Action）：`read_debug_snapshot`
- 启动约束：授权 `read_debug_snapshot` / `read_point_catalog` 时，挂载即读取领域 PointSet；用户仍可通过“刷新”重新拉取。
- 维护动作（Action）：`home`、`move_to_anchor`、`jog_joint_once`、
  `jog_tcp_once`、`move_rail_to_position`、`record_current_point`、
  `calibrate_camera_extrinsic`、`calibrate_tcp`、`record_marker`

导轨位置只作为快照与组合点位记录显示。在独立导轨维护端口能返回与命令
精确绑定的零速、到位与 settled 见证之前，卡片失败关闭，不提供导轨运动按钮。

卡片不得直接建立 HTTP、WebSocket 或 SSE 连接。`moveit_online` 与
`jointState` 必须来自 Host 的统一设备遥测（DeviceTelemetry）SSE 投影，手动
独占（Exclusive）必须通过 Host Bridge。Jog 是一步一命令的动作，卡片不设置
单步最大值，只要求输入正的有限数；最终目标仍由 MoveIt、型号关节限位和碰撞
检查约束。移动到锚点时必须传 PointSet 稳定 `target_ref`，因为 Mixer 作者层没有
pTLC 的 P 点别名。

## 视觉校准与数据落盘

页签：

| 页签 | 功能 |
|---|---|
| 点位与姿态 | PointSet 目标列表与目标/实时 TCP |
| Jog 与示教 | 关节/TCP Jog、记录当前位置（可勾选视觉） |
| 视觉校准 | 摄像头外参标定、TCP 校准、marker 记录 |

视觉数据按《使用说明》v1.8 两层落盘（不建 `assets/vision/`）：

| 层 | 路径 |
|---|---|
| 机器读取层 | `szlab_poly_studio/deployment/robot_cell/assets/calibrations/szlab-mixer-cr7-rail.v1.yaml` |
| 部署登记层 | `szlab_poly_studio/deployment/robot_cell/assets/qualifications/szlab-mixer-vision-registry.v1.yaml` |

`read_debug_snapshot` 返回 `vision` 块（markers、point_bindings、标定状态）。
记录点位时 `include_vision=true` 与可选 `marker_ref` 写入 `point_bindings`；
几何仍由 `record_current_point` 写回 PointSet。

仓位粒度：**process_warehouse** = PointSet `group_ref`；默认 marker 为 `{warehouse}/default`。
