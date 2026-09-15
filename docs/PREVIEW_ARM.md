# Preview 机械臂运行时（PreviewArm）

无 MoveIt、无真机 I/O 的本地 preview 机械臂控制，与 PLC / TCP-SDK / MoveIt 共用同一套 **BackendKind** 切换机制。

## 分层

| 层 | 包 | 职责 |
|----|-----|------|
| L0 | `unilab-robot-runtime/preview/` | `PreviewArmDevice`、关节插值、六段路径、`LOCAL_ARMS` 注册表 |
| L1 | `unilab-arm-elite-cs66` 等 | 型号 DH、mesh、FK/IK、`urdf_providers` |
| L3 | 领域仓 | `ArmMount` 基座/导轨、工位坐标、路由与 visual |

## Backend 切换

与 SZLab `standard_execution_backend` 相同：**改图 → 冷重启 OS**，非运行时热切换。

| 字段 | Preview 取值 |
|------|--------------|
| 图节点 `config.standard_execution_backend` | `"preview"` |
| `HardwareProfile.backend` | `BackendKind.PREVIEW` |
| `HardwareProfile.mode` | 固定 `DeploymentMode.SIMULATION` |

### 四种 backend 对照

| BackendKind | 图里典型值 | 执行体 |
|-------------|-----------|--------|
| `plc` | `"plc"` | PLC 变量端口 |
| `moveit` | `"moveit"` / `"moveit_sim"` | MoveIt move_group |
| `tcp_sdk` | `"tcp_sdk"` | 厂商 SDK |
| **`preview`** | **`"preview"`** | L0 + L1 解析 IK |

### hydration 图示例

```json
"config": {
  "device_id": "elite_left",
  "standard_execution_backend": "preview",
  "robot_execution_path": "robot_template",
  "preview_kinematics_package": "unilab_arm_elite_cs66"
}
```

## L1 实现 checklist

实现 `PreviewKinematics` 协议：

- `home_deg`
- `fk_tcp(mount, q_deg, rail_y)`
- `plate_pose(...)`
- `ik_tcp(mount, xyz, seed=..., yaw_deg=..., rail_y=...)`
- `linear_path(...)` / `carry_path(...)`

提供 `urdf_providers.build_base` / `build_kinematics`（接受 `base_xyz` 与 `rail_limits`）。

## 领域接入

1. 新建 `devices/mounts.py`：`ArmMount` 字典（基座世界坐标、可选导轨限位）。
2. `devices/arm.py`：`@device` 子类化 `PreviewArmDevice`，注入 L1 kinematics + mount。
3. `devices/model.py`：薄 re-export L1 URDF provider。
4. `pyproject.toml` 依赖 `unilab-arm-*` 与 `unilab-robot-runtime`。

## 开发安装

```bash
cd unilab_robot_template
pip install -e packages/unilab-robot-contracts \
  -e packages/unilab-robot-runtime \
  -e packages/unilab-arm-elite-cs66
pytest tests/ -q
```
