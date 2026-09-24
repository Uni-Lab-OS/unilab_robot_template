# 参照：MoveIt commissioning 卡片薄包装（SZLab）

Canonical 实现（勿复制进 demo，直接读源文件）：

| 文件 | 路径 |
|------|------|
| 卡片 mixin + `@action` | `Uni-Lab-SZLab/szlab_poly_studio/devices/szlab_mixer_robot/szlab_mixer_arm_card.py` |
| MoveIt commissioning | `.../moveit_commissioning.py` |
| 卡片 manifest | `Uni-Lab-SZLab/frontend/cards/szlab-mixer-robot-card/card.manifest.json` |

要点：

- 设备类继承 `RailMountedArmCardMixin`；mixin 在 **领域包** 重复 `@action` 供 OS AST 扫描
- `build_*_arm_card_context()` 注入 `ArmCardContext`：`catalog_entries`、`project_target`、`point_set_path`
- `moveit_commissioning.py` + `device.post_init` → `_moveit_split_binding`（卡片按钮必需）
- 前端单壳：`templateCard: unilab_robot_template/frontend/cards/rail-mounted-arm-card`

**Greenfield 最小可复制集**（无需抄 SZLab 全量）：[catalog-moveit-cr5](../catalog-moveit-cr5/README.md) 模板 + `introduce_arm.py --new-domain` 自动 scaffold。

引入命令示例：

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab \
  --device szlab_mixer_robot --domain-owned --with-card --apply --migrate
```
