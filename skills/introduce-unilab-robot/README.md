# introduce-unilab-robot

一键把 `unilab_robot_template` 机械臂型号写进领域仓。

```bash
# catalog（最小成本）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --catalog cr5 --apply --install

# 领域自有
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --domain-owned --apply --migrate
```

- 人类说明：[docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md)
- Agent 细则：[SKILL.md](SKILL.md)
