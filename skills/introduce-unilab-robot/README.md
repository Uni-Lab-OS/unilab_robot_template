# introduce-unilab-robot

**机械臂引入唯一入口 skill**：Agent 必须先读 [SKILL.md](./SKILL.md) 与 [docs/demo](./docs/demo/README.md)，再执行脚本。

```bash
# Greenfield：默认 cr5 机械臂 + SZLab 导轨 + graph
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain <空目录> --rail-device rail --device robot \
  --scaffold-graph --apply --install

# Greenfield：domain-owned + vendor
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain <空目录> --device my_robot --rail-device rail \
  --domain-owned --vendor-urdf <vendor.urdf> --vendor-mesh-dir <meshes> \
  --scaffold-graph --apply --migrate

# 已有领域仓：MoveIt catalog
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓> --device <id> --catalog cr5 --with-card --apply --install

# 已有领域仓：领域自有 MoveIt
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓> --device <id> --domain-owned --with-card --apply --migrate
```

- 领域需改部分：**[docs/demo/](./docs/demo/)**（greenfield 从 `minimal-rail-arm-*` 开始；MoveIt 卡片 commissioning 见 [catalog-moveit-cr5](./docs/demo/catalog-moveit-cr5/)）
- Workbench FE 3D 门禁：**[workbench-fe-3d-gate.md](./docs/demo/workbench-fe-3d-gate.md)**
- 人类说明：[docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md)
