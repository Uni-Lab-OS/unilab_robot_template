# 机械臂引入 Demo（领域仓需改部分）

本目录存放 **introduce-unilab-robot** 一键引入后，领域仓仍须人工核对或补全的文件示例。Agent 与用户均应先读对应子目录，再 `--apply`。

## 完整 greenfield 从这里开始

| 子目录 | 模式 | 说明 |
|--------|------|------|
| [**minimal-rail-arm-catalog**](./minimal-rail-arm-catalog/) | `--new-domain` + `--catalog` | 最小领域包 + 导轨 + graph + MoveIt catalog |
| [**minimal-rail-arm-domain-owned**](./minimal-rail-arm-domain-owned/) | `--new-domain` + `--domain-owned` + vendor | 最小领域包 + 领域自有 MoveIt |

graph 片段模板：[graph-rail-arm.template.json](./graph-rail-arm.template.json)

Greenfield 在 Workbench 验 **FE 3D** 前必读：[workbench-fe-3d-gate.md](./workbench-fe-3d-gate.md)（OS v2 管线、inventory reset、HTTP 验收；默认 **SZLab 导轨 mesh + cr5**）。

## 已有领域包 / 单设备模式

| 子目录 | 模式 | 说明 |
|--------|------|------|
| [catalog-moveit-cr5](./catalog-moveit-cr5/) | `--catalog cr5` | MoveIt L1 catalog + **commissioning 卡片**（PointSet / moveit_commissioning / rail_simulation） |
| [catalog-preview-elite-cs66](./catalog-preview-elite-cs66/) | `--preview-catalog elite-cs66` | Preview L1 + 领域 mounts / 卡片薄包装 |
| [domain-owned-moveit](./domain-owned-moveit/) | `--domain-owned` | 领域自有 URDF + migrate 清单 |
| [domain-card-moveit-szlab](./domain-card-moveit-szlab/) | MoveIt commissioning 卡片薄包装（SZLab 参照） |
| [domain-card-preview-hydration](./domain-card-preview-hydration/) | Preview 卡片 + PointSet v3（Hydration 参照） |

## 一键命令（在 Uni-Lab-Core 或 unilab_robot_template 根）

```bash
# Greenfield：默认 cr5 + SZLab 导轨 + graph（可省略 --catalog）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain <空目录> --rail-device rail --device robot \
  --scaffold-graph --apply --install

# Greenfield：domain-owned + vendor 资产
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain <空目录> --device my_robot --rail-device rail \
  --domain-owned --vendor-urdf <vendor.urdf> --vendor-mesh-dir <meshes> \
  --scaffold-graph --apply --migrate

# 已有领域仓：只导入 catalog 机械臂（不改 graph）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --catalog cr5 \
  --with-card --apply --install

# Preview catalog（Elite CS66）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --preview-catalog elite-cs66 \
  --with-preview-scaffold --with-card --apply --install

# 已有领域仓：领域自有 MoveIt
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <device_id> --domain-owned \
  --with-card --apply --migrate
```

引入完成后对照本目录相应子文件夹，补全 `TODO` 与领域资产路径。
