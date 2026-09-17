# 参照：Preview 卡片 + PointSet v3（Hydration）

Canonical 实现：

| 文件 | 路径 |
|------|------|
| Preview 设备 | `hydration-cad-workspace/hydration_station/devices/hydration_elite_arm/device.py` |
| 卡片 mixin | `.../hydration_elite_arm_card.py` |
| mounts / 运动学 | `mounts.py`, `preview_kinematics.py`, `model.py` |
| PointSet v3 | `hydration_station/robot_cell/hydration_point_set_v3.py` |
| 卡片 manifest | `hydration-cad-workspace/frontend/cards/elite-preview-card/card.manifest.json` |

引入后必改：

1. `mounts.py` — 基座 `base_xyz`、导轨 `rail_limits`
2. `_arm_card_context()` — `point_set_path`、`catalog_entries`、`on_record_with_vision`
3. `robot_cell/` — 生成或切换 PointSet v3 YAML

卡片能力（Preview）：`set_joint`、`jog_tcp_once`、`read_point_catalog`、`record_current_point`；UI 自适应 [rail-mounted-arm-card](../../../../frontend/cards/rail-mounted-arm-card)。
