# Uni-Lab Robotics 代理须知

消费本仓机械臂 / 导轨型号包时，**先读再用，禁止即兴拼装**。

1. 打开并完整遵循 `.cursor/skills/use-unilab-arm-package/SKILL.md`。
2. 以 `Uni-Lab-pTLC` 为唯一已接受的领域装配样本，不要抄 OS 遗留 `arm_slider` xacro，也不要发明合一体 MoveIt。
3. 改完领域包后运行：

```text
python .cursor/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain <领域仓根>
```

包装新的六轴型号（L1 distribution）走另一条技能：`.cursor/skills/package-arm-moveit/SKILL.md`。
