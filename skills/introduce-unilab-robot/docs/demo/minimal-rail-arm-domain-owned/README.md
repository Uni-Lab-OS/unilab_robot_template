# 最小导轨 + 机械臂领域包（domain-owned）

greenfield 引入领域自有 MoveIt 机械臂：scaffold 领域包 + 导轨 + graph，只读拷贝 vendor URDF/mesh，再链式 `migrate_domain_arm.py`。

## 命令

```bash
python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --new-domain F:/path/to/MyLab \
  --device szlab_mixer_robot \
  --rail-device szlab_mixer_rail \
  --domain-owned \
  --vendor-urdf F:/path/to/vendor/cr20_robot.urdf \
  --vendor-mesh-dir F:/path/to/vendor/meshes/cr20 \
  --scaffold-graph \
  --apply \
  --migrate
```

## vendor 资产写入位置

```text
<domain_pkg>/devices/<device_id>/
├── kinematics.py          # introduce 生成
├── moveit_model.py        # introduce 生成
└── models/
    ├── model.yaml
    ├── <vendor>.urdf
    └── meshes/<slug>/*.stl
```

`--migrate` 会调用 [domain-owned-arm-rail](../../../domain-owned-arm-rail/SKILL.md) 完成 L3 移植细节；introduce 只负责薄封装与 provider 写入。

## 已有领域包只导入臂

不加 `--new-domain` / `--scaffold-graph`：

```bash
python .../introduce_arm.py \
  --domain F:/path/to/Uni-Lab-SZLab \
  --device szlab_mixer_robot \
  --domain-owned \
  --apply \
  --migrate
```

## 验收

```bash
python unilab_robot_template/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py \
  --domain F:/path/to/MyLab
```

期望 **exit 0**。
