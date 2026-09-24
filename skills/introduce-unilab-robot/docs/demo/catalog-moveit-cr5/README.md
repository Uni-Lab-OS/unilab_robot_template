# Demo：`--catalog cr5`（MoveIt L1）

脚本写入后，领域仓至少应有：

```text
<domain_pkg>/devices/<device_id>/device.py              # 继承 *ArmCardMixin + post_init MoveIt 绑定
<domain_pkg>/devices/<device_id>/<device_id>_arm_card.py # build_*_arm_card_context()，禁止 NotImplementedError
<domain_pkg>/devices/<device_id>/moveit_commissioning.py # CR5 commissioning 客户端 + bind_commissioning_runtime
<domain_pkg>/robot_cell/point_set_v3.py                 # PointSet v3 解析器
<domain_pkg>/robot_cell/__init__.py
<domain_pkg>/devices/rail_simulation.py                   # mock/simulation 导轨轴（卡片 rail 快照/移动）
<domain_pkg>/devices/<rail_id>.py                         # 含 post_init 登记仿真轴
deployment/robot_cell/assets/point_sets/<domain_pkg>-rail-arm.v3.yaml
pyproject.toml                                            # unilab-arm-cr5、unilab-robot-runtime
frontend/cards/<device_id>-card/                          # greenfield 默认
```

## 等价 provider 一行

```python
"provider": "unilab_arm_cr5:build_moveit_model"
```

## 卡片可点击（Commissioning）最小集

| 组件 | 作用 |
|------|------|
| `*_arm_card.py` | `ArmCardContext`：catalog、导轨快照、`point_set_path` |
| `moveit_commissioning.py` | `post_init` 创建 `_moveit_split_binding` |
| PointSet v3 | `read_debug_snapshot` / `home` / jog 的 target 目录 |
| `rail_simulation.py` | graph `mock` 导轨的 `/joint_states` + 卡片导轨移动 |

参照实现（生产级）：[domain-card-moveit-szlab](../domain-card-moveit-szlab/README.md)（SZLab Mixer）。

## Demo 模板索引

| 文件 | 说明 |
|------|------|
| [arm_card.py.template](./arm_card.py.template) | 卡片 mixin + `build_*_arm_card_context()` |
| [moveit_commissioning.py.template](./moveit_commissioning.py.template) | MoveIt 调试绑定 |
| [point_set_v3.py.template](./point_set_v3.py.template) | PointSet 解析 |
| [point_set.v3.example.yaml](./point_set.v3.example.yaml) | 作者层 YAML 形状 |
| [rail_simulation.py.template](./rail_simulation.py.template) | 导轨仿真轴 |
| [device.py.example](./device.py.example) | 机械臂 `@device` + `post_init` |
| [rail.py.example](./rail.py.example) | 导轨 `@device` + `post_init` |

## 整机装配（导轨 + 臂）

catalog 只解决 **臂型号引用**。pTLC 式双 `@device`、物理图 parent、禁止第七轴合并 MoveIt 模型 → 读 [use-unilab-arm-package](../../../use-unilab-arm-package/SKILL.md) 并跑 `check_domain_arm_assembly.py`。

## Workbench 卡片验收

Backend + Edge ready 后，机械臂卡片按钮（刷新快照、读取状态、回 home）不应再报 `NotImplementedError` 或「MoveIt 调试端口未就绪」。仍失败时查 Edge 日志里 `robot` 设备 `post_init` 与 move_group 是否在线。
