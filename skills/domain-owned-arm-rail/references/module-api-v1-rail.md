# 导轨 Module API v1（领域包内实现）

## 默认拆分

| 部分 | 领域文件 | 说明 |
|------|----------|------|
| 外壳 STL / 碰撞盒 | `devices/<rail>/static_layout.py` | `@device provider`（package_static） |
| 棱柱运动学 | `devices/<rail>/kinematic_model.py` + `models/model.yaml` | `joint_state_provider` |

## 模块常量

```python
MODULE_API_VERSION = 1
MODULE_KIND = "rail"
```

## 必须导出

| 符号 | 文件 |
|------|------|
| `MODEL_DESCRIPTOR` | `rail_module.py` 或 `kinematic_model.py` |
| `build_kinematic_model` | `kinematic_model.py` |
| `create_simulation_module` / `create_plc_module` | `rail_module.py` |

## `@device`（默认领域路径）

```python
model={
    "type": "package_static",
    "provider": "<domain>.devices.my_rail.static_layout:build_rail",
    "source_digest": "<外壳 STL digest>",
    "joint_state_provider": "<domain>.devices.my_rail.kinematic_model:build_kinematic_model",
    "joint_state_source_digest": "<devices/my_rail/models/model.yaml digest>",
}
```

## 运动学选项

- **A（默认）**：领域 `models/model.yaml` 定义 `travel_m` / 轴名，用 model-kit `build_prismatic_rail_kinematic_model`
- **B（例外）**：行程与 `unilab-rail-linear` 完全一致 → 可引用 `unilab_rail_linear:build_kinematic_model`，**外壳仍在领域**
