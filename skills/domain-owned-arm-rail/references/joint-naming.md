# 关节命名（Joint Naming）：只绑 device_id，不绑型号

领域自有机械臂 **必须** 遵守本规则。完整迁移门禁：`check_moveit_joint_alignment.py`。

## 两层命名

```
vendor URDF          canonical (model.yaml)     qualified (MoveIt / /joint_states)
───────────          ──────────────────────     ──────────────────────────────────
joint1      ──map──► joint_1          ──prefix──► {device_id}_joint_1
joint2      ──map──► joint_2          ──prefix──► {device_id}_joint_2
…                    …                              …
```

| 层 | 格式 | 谁区分两台相同型号的臂 |
|---|---|---|
| canonical | `joint_1` … `joint_6` | **不区分**（设备无关） |
| qualified | `{device_id}_joint_{n}` | **device_id** |

示例：同一 CR30H 装两台

- `mixer_a` → `mixer_a_joint_1`
- `mixer_b` → `mixer_b_joint_1`

换 vendor 型号（CR7→CR30H→CR16）时 **只改** URDF 映射与 link/mesh；**关节 canonical 名不变**。

## 禁止

| 错误写法 | 为何错 |
|---|---|
| `cr7_joint_1` 作 canonical | 换型号要改全仓点位/控制器 |
| `szlab_mixer_robot_cr7_joint_1` 手写 qualified | 型号 slug 不应出现在关节名里 |
| endpoint / 规划组 / mesh 目录用 `joint_*` | 那些层用 `{device_id}` 或固定角色名 `arm` |

## model.yaml

```yaml
kinematic_joints:
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6
joint_state_sources:
  canonical: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  dobot_sdk_v1: [J1, J2, J3, J4, J5, J6]   # 厂商反馈名，按实际填写
moveit_group: szlab_mixer_robot_arm   # 只绑 device_id，不出现在 joint
```

## moveit_model.py

```python
_CANONICAL_JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
_JOINT_NAMES = {
    "dummy_joint": "base_mount_joint",  # 按 URDF 填
    **{f"joint{i}": f"joint_{i}" for i in range(1, 7)},
}
```

`build_joint_state_name_map(device_id=...)` 与 `assemble_six_axis_moveit_model` 会生成
qualified 名，**不要**手搓 `{device_id}_cr7_joint_*`。

## 门禁

```bash
python domain-owned-arm-rail-skill/scripts/check_moveit_joint_alignment.py \
  --domain <领域仓根> --device <robot_device>
```

失败时 JSON 里的 `remediation` 字段给出应对齐的目标命名（只读，不自动改领域仓）。

## move_group 报 Joint not found

若日志出现 `{device_id}_cr16_joint_*` 而 URDF 里是 `{device_id}_joint_*`（或反之）：

1. 停 OS，不要热重载
2. 对照 `remediation.qualified_joint_names` 与 `moveit_model` / `model.yaml`
3. 清 runtime 缓存后重启

根因几乎都是 **canonical 里混入了型号 slug**，不是 MoveIt 本身 bug。
