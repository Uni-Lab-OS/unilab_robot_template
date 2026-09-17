# Demo：`--preview-catalog elite-cs66`（Preview L1）

脚本 `--with-preview-scaffold` 会生成下列骨架；**必须**按领域改 `mounts.py`（基座坐标、导轨限位）与可选 `point_set` / vision 路径。

```text
devices/<device_id>/
  device.py                 # PreviewArmDevice + CardMixin
  model.py                  # L1 URDF provider 薄包装
  mounts.py                 # TODO: 领域 mount / 导轨
  preview_kinematics.py     # L1 FK/IK 门面
  <device_id>_arm_card.py   # PreviewArmCardMixin + @action 声明
frontend/cards/<device>-card/
  card.manifest.json        # templateCard → rail-mounted-arm-card
```

参照完整领域实现：[domain-card-preview-hydration](../domain-card-preview-hydration/README.md)（Hydration `hydration_elite_arm`）。
