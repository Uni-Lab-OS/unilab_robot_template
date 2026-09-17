# Demo：`--domain-owned`（领域 L3 MoveIt）

## 前提目录

```text
devices/<device_id>/
  models/model.yaml          # source.sha256
  models/<vendor>.urdf
  moveit_model.py            # build_moveit_model()
  device.py
```

## 脚本行为

1. 写 `@device(model_ownership=domain, provider=<domain>.devices.<id>.moveit_model:build_moveit_model)`
2. `--migrate` → `domain-owned-arm-rail/scripts/migrate_domain_arm.py --apply`
3. `--with-card` → manifest 指向 `rail-mounted-arm-card`

## 验收

- [domain-owned-arm-rail SKILL](../../../domain-owned-arm-rail/SKILL.md)
- RViz attach / 关节名 / robot_module 四门禁

参照：[domain-card-moveit-szlab](../domain-card-moveit-szlab/README.md)
