# 本 skill 改了什么 / 没改什么

## 没改（重要）

| 范围 | 说明 |
|---|---|
| **OS / move_group 运行时** | skill 脚本**不会**在 OS 启动时执行；不会向 move_group 注入关节名 |
| **Uni-Lab-OS** | 未改 `resource_visalization.py`、`package_moveit_model.py` 等 |
| **领域仓** | 除非你**手动**跑 `migrate_domain_arm.py --apply`，否则 skill 不写磁盘 |

## 改了什么（仅 `domain-owned-arm-rail-skill/` 目录）

| 文件 | 作用 |
|---|---|
| `scripts/migrate_domain_arm.py` | 一键迁移入口；`--apply` 时**先**跑 joint 名修复 |
| `scripts/fix_device_joint_names.py` | 剔除 device 目录内 `{device}_{slug}_joint_*` 写法 |
| `scripts/check_moveit_joint_alignment.py` | 门禁：canonical 必须是 `joint_N`；全仓扫禁止模式 |
| `scripts/_joint_naming.py` | 关节命名规则（纯逻辑，不写运行时） |
| `scripts/fix_moveit_runtime_attach.py` | catalog → 领域模块；**写对 `moveit_model.model_ref`**；endpoint / manifest / import |
| `scripts/scaffold_*.py` | 生成 `robot_module`、adapters |
| `references/joint-naming.md` | 关节命名规范 |
| `SKILL.md` / `README.md` | Agent 入口说明 |

文档和测试里出现的 `cr7` / `cr16` 字符串 = **「禁止模式」样例**，不是 skill 往系统里写这两个型号。

## 日志里 cr7 和 cr16 **同时**出现说明什么

当前 URDF 关节已是 `{device_id}_joint_*` 时，move_group 仍找 `{device}_cr7_joint_*` **和** `{device}_cr16_joint_*`：

→ **OS 叠了多套历史 controller / moveit_controllers 配置**（多次迁移、热重启、未清缓存），不是 skill 一次写进两个型号。

处理（与 skill 无关，在 OS 侧重做）：

1. **完全停** OS / move_group / controller_manager  
2. 删 Workbench 运行时 mesh 缓存（常见：`.unilabos/runtime/` 下该 lab 的 `ros2_controllers.yaml` / `moveit_controllers.yaml` 或整目录）  
3. 确认 `moveit_model.py` + `model.yaml` 已是 `joint_N`  
4. **冷启动** OS  

## 领域仓应对齐的目标（skill 门禁）

```
canonical:  joint_1 .. joint_6
qualified:  {device_id}_joint_1 .. {device_id}_joint_6
```

检查：

```bash
python domain-owned-arm-rail-skill/scripts/check_moveit_joint_alignment.py \
  --domain <领域仓> --device szlab_mixer_robot
```
