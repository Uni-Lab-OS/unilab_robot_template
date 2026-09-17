# 机械臂引入 Demo（领域仓需改部分）

本目录存放 **introduce-unilab-robot** 一键引入后，领域仓仍须人工核对或补全的文件示例。Agent 与用户均应先读对应子目录，再 `--apply`。

| 子目录 | 模式 | 说明 |
|--------|------|------|
| [catalog-moveit-cr5](./catalog-moveit-cr5/) | `--catalog cr5` | MoveIt L1 catalog 最小 `@device` |
| [catalog-preview-elite-cs66](./catalog-preview-elite-cs66/) | `--preview-catalog elite-cs66` | Preview L1 + 领域 mounts / 卡片薄包装 |
| [domain-owned-moveit](./domain-owned-moveit/) | `--domain-owned` | 领域自有 URDF + migrate 清单 |
| [domain-card-moveit-szlab](./domain-card-moveit-szlab/) | MoveIt  commissioning 卡片薄包装（SZLab 参照） |
| [domain-card-preview-hydration](./domain-card-preview-hydration/) | Preview 卡片 + PointSet v3（Hydration 参照） |

## 一键命令（在 Uni-Lab-Core 或 unilab_robot_template 根）

```bash
# MoveIt catalog
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --catalog cr5 \
  --with-card --apply --install

# Preview catalog（Elite CS66）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --preview-catalog elite-cs66 \
  --with-preview-scaffold --with-card --apply --install

# 领域自有 MoveIt
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --domain-owned \
  --with-card --apply --migrate
```

引入完成后对照本目录相应子文件夹，补全 `TODO` 与领域资产路径。
