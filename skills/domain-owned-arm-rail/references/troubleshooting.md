# 故障对照（通用）

| 现象 | 原因 | 处理 |
|------|------|------|
| **RViz 无夹爪，FE 正常** | `moveit_model.model_ref` ≠ PointSet；runtime 装配失败，attach 未发 | `fix_moveit_runtime_attach.py --apply`（**不改 PointSet**）；或 `migrate_domain_arm.py --apply` 一次完成 |
| edge：`components.arm.model_ref 与精确 Arm 型号不一致` | 同上 | 同上 |
| edge：`初始工具附着延迟发布失败，统一运行时已关闭` | runtime 装配异常（常见 model_ref） | 查 edge traceback；跑 attach contract 门禁 |
| move_group：`Joint '{device}_*_joint_N' not found`（**多种 slug 同时出现**） | OS **叠了多套历史** ros2/moveit controller，URDF 已是 `{device}_joint_*` | ① 停 OS ② 删 `.unilabos/runtime/` mesh 缓存 ③ 冷启动 ④ 领域仓跑 joint 门禁 |
| **全关节 0，RViz 两节 mesh 断缝/浮空** | **vendor URDF 被手改**或与上游不一致；或 URDF/mesh 不同包 | 阶段 1：diff 本地 vs 上游 URDF（查 joint1–3 `origin`/`axis`）；**重拷** vendor 文件；**不要**先改 home/mapping |
| digest 本地自洽但与 Dobot 官方不同 | 改 URDF 后重算 digest | 用上游 SHA256；重拷 URDF |
| `FileNotFoundError: .../models/model.yaml` | yaml 不在 `models/` | 阶段 1 |
| `mesh 资产缺失` | 缺 STL | 阶段 1：与 URDF 同包拷贝 mesh |
| RViz 只有 TF | 未完整迁移或未重启 OS | 阶段 5 |
| mesh 散架（**非全 0**） | 映射与 URDF 不一致，或 joint 角错 | 阶段 2 重填映射；查 joint_states |
| FE 与 RViz 不一致 | 未完成阶段 6 | fe-must-match-rviz |
| Edge：`不支持 Robot Module API v1` | 缺 `robot_module` 或 manifest 指错 | 阶段 2.2 + 3.2 |
| `model.yaml` 新型号，点位仍旧 slug | 未做阶段 2.3 grep | 整批同步或保留旧 canonical 并文档化 |

**全 0 仍断节** → 99% 是 URDF 资产问题，不是 home、不是 PointSet。

完整流程：[`complete-domain-moveit-migration.md`](complete-domain-moveit-migration.md)
