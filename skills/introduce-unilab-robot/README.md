# introduce-unilab-robot

**机械臂引入唯一入口 skill**：Agent 必须先读 [SKILL.md](./SKILL.md) 与 [docs/demo](./docs/demo/README.md)，再执行脚本。

```bash
# MoveIt catalog
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓> --device <id> --catalog cr5 --with-card --apply --install

# Preview catalog（Elite CS66 + 领域骨架）
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓> --device <id> --preview-catalog elite-cs66 \
  --with-preview-scaffold --with-card --apply --install

# 领域自有 MoveIt
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓> --device <id> --domain-owned --with-card --apply --migrate
```

- 领域需改部分：**[docs/demo/](./docs/demo/)**
- 人类说明：[docs/INTRODUCE_UNILAB_ROBOT.md](../../docs/INTRODUCE_UNILAB_ROBOT.md)
