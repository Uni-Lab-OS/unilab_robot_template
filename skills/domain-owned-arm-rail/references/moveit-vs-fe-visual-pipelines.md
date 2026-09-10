# MoveIt（RViz）与 FE 3D 双管道

领域 `@device model` 里常同时存在两块配置，**互不串联**。

## 管道对照

```text
                    ┌─────────────────────────────────────┐
  package_moveit    │ moveit_model.py → model-kit         │
  provider          │ models/vendor.urdf + meshes         │
                    └──────────────┬──────────────────────┘
                                   ▼
                         OS device_mesh / full_dev
                                   ▼
                              RViz / MoveIt

                    ┌─────────────────────────────────────┐
  shape / xacro     │ models/shape.yml 或 workspace_xacro │
  + 物理图位姿       │ Graph position / rotation           │
                    └──────────────┬──────────────────────┘
                                   ▼
                         Workbench FE 3D 场景
```

## 迁移时常见误解

| 误解 | 事实 |
|------|------|
| 改了 provider，FE 应该跟着变 | FE 不读 `moveit_model.py` |
| RViz 错了，改 FE xacro 就能修 RViz | 违反 fe-must-match-rviz：RViz 是权威 |
| shape.yml 是机械臂真实 mesh | 多为参数化占位；MoveIt 才用 URDF mesh |

## 完整迁移中的顺序（阶段 5 → 6）

1. **domain-owned-arm-rail** 阶段 5：RViz 资产 + 映射 + provider → RViz 通过  
2. **fe-must-match-rviz** 阶段 6：改 FE shape/xacro/图挂载，跟 RViz 对齐  

仅改 provider 时 RViz 变、FE 不变，属于阶段 5 未完成完整迁移，不是 FE 故障。
