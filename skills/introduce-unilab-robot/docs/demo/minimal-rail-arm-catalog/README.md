# 最小导轨 + 机械臂领域包（catalog）

**完整 greenfield 从这里开始。** 一条命令 scaffold 最小领域包、导轨 `@device`、graph，并引入 MoveIt catalog 机械臂 + **可点击 commissioning 卡片**。

## 命令

在 **Uni-Lab-Core** 或 **unilab_robot_template** 根：

```bash
python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain F:/path/to/MyLab \
  --rail-device rail \
  --device robot \
  --scaffold-graph \
  --apply \
  --install
```

未写 `--catalog` 时默认 **cr5**（L1 `unilab-arm-cr5`）；未写 `--rail-stl` 时校验 template 包内 **SZLab arm_slideway.stl**，领域仓只写 `rail.py` 引用 L1 provider。

**每次运行**自动检测机械臂卡片与 commissioning 是否齐全，缺则补齐（manifest + `robot_arm_card.py` + mixin + PointSet + `moveit_commissioning.py` + `rail_simulation.py`）；不要卡片时加 `--skip-card`。

## 期望磁盘结构

```text
MyLab/
├── pyproject.toml
├── package.yaml
├── deployment/local_config.py
├── deployment/robot_cell/assets/point_sets/<domain_pkg>-rail-arm.v3.yaml
├── .unilabos/environment.local.json
├── my_lab/
│   ├── robot_cell/
│   │   ├── __init__.py
│   │   └── point_set_v3.py
│   ├── devices/
│   │   ├── rail.py                 # post_init → rail_simulation
│   │   ├── rail_simulation.py
│   │   └── robot/
│   │       ├── device.py           # post_init → _moveit_split_binding
│   │       ├── robot_arm_card.py   # build_*_arm_card_context()
│   │       └── moveit_commissioning.py
│   └── workflows/__init__.py
├── frontend/cards/robot-card/
└── deployment/graphs/local-debug.json
```

模板索引见 [catalog-moveit-cr5](../catalog-moveit-cr5/README.md)。

## graph 要点

- 导轨 `parent: null`，`children` 含机械臂 id
- 导轨 `standard_execution_backend: mock`
- 机械臂 `parent` = 导轨 id，`standard_execution_backend: moveit_sim`
- 机械臂 `rotation.z: 180`（不设 `mount_yaw_deg`，避免 checker 叠加报错）

参照 [graph-rail-arm.template.json](../graph-rail-arm.template.json)。

## 验收

```bash
python unilab_robot_template/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py \
  --domain F:/path/to/MyLab
```

期望 **exit 0**。

## Workbench FE 3D + 卡片

checker **不能**代替 3D 验收。桌面 Workbench + [workbench-fe-3d-gate.md](../workbench-fe-3d-gate.md)（Edge init `rail` + `robot`，URDF / kinematics）。

**卡片按钮**：Backend + Edge ready 后，刷新快照 / 读取状态 / 回 home 不应报 `NotImplementedError` 或「MoveIt 调试端口未就绪」。参照 [catalog-moveit-cr5](../catalog-moveit-cr5/README.md) commissioning 清单。
