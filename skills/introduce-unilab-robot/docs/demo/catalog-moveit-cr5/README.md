# Demo：`--catalog cr5`（MoveIt L1）

脚本写入后，领域仓至少应有：

```text
<domain_pkg>/devices/<device_id>/device.py   # @device model.provider 一行
pyproject.toml                               # unilab-arm-cr5、unilab-robot-runtime
frontend/cards/<device_id>-card/             # 仅 --with-card 时
```

## 等价 provider 一行

```python
"provider": "unilab_arm_cr5:build_moveit_model"
```

## 整机装配（导轨 + 臂）

catalog 只解决 **臂型号引用**。pTLC 式双 `@device`、物理图 parent、禁止第七轴合并 MoveIt 模型 → 读 [use-unilab-arm-package](../../../use-unilab-arm-package/SKILL.md) 并跑 `check_domain_arm_assembly.py`。

示例文件：[device.py.example](./device.py.example)
