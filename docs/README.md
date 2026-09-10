# Uni-Lab Robotics 使用说明

本目录只保留**面向使用者的操作说明**。Agent 技能细则在 `skills/` 下。

## 文档索引

| 文档 | 适用场景 |
|------|----------|
| **[INTRODUCE_UNILAB_ROBOT.md](./INTRODUCE_UNILAB_ROBOT.md)** | **一键**把 template 机械臂引用进领域仓（最小成本入口，推荐先看） |
| [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) | catalog 型号包（`unilab_arm_*`）pTLC 式导轨 + 机械臂整机装配 |
| [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md) | 领域自有机械臂 / 导轨（`model_ownership: domain`）MoveIt 完整移植、RViz / attach 验收 |

## 最快路径（一条命令）

```bash
# catalog 型号（cr5 / cr7）
e:/miniforge3/envs/unilab/python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --catalog cr5 --apply --install

# 领域自有 URDF
e:/miniforge3/envs/unilab/python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --domain-owned --apply --migrate
```

详见 [INTRODUCE_UNILAB_ROBOT.md](./INTRODUCE_UNILAB_ROBOT.md)。

## 其它技能

| 技能 | 入口 |
|------|------|
| 包装新六轴 L1 型号 | `skills/package-arm-moveit/SKILL.md` |
| FE 3D 对齐 RViz | `.cursor/skills/fe-must-match-rviz/SKILL.md`（仓根 `.cursor/skills/`） |

## 参数说明

- `<领域仓根>`：领域实验室仓库根目录，如 `F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab`
- `<robot_device>`：图（Graph）里机械臂节点的 `"id"`，也是 `devices/<robot_device>/` 文件夹名（SZLab 示例：`szlab_mixer_robot`）

## 常用命令速查

```bash
# 一键引入（见 INTRODUCE_UNILAB_ROBOT.md）
e:/miniforge3/envs/unilab/python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --catalog cr5 --apply --install

# 领域自有臂 migrate（introduce --migrate 也会链式调用）
e:/miniforge3/envs/unilab/python skills/domain-owned-arm-rail/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <robot_device> --apply

# catalog 臂 / 导轨整机装配检查
e:/miniforge3/envs/unilab/python skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py \
  --domain <领域仓根>
```
