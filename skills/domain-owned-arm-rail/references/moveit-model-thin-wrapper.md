# model-kit 薄封装 + Module API（通用）

完整迁移 **阶段 2** 使用本模式。slug / URDF / 映射从**领域 device 磁盘**读取。

## vendor URDF vs Python 包装（必读）

| 层 | 谁负责 | Agent 能改吗 |
|----|--------|--------------|
| `models/<vendor>.urdf` + STL | 厂商发布物 | **否** — 只读拷贝 |
| `moveit_model.py` 映射 | 领域 L3 | **是** — link/joint 重命名、碰撞对 |
| model-kit 运行时 | prefix、mount_yaw、SRDF | 自动 — 不改磁盘 URDF |

**`unilab_arm_cr7` 只是 Python/adapters 的只读参考**，不是 URDF 几何模板。  
禁止把 CR7 的 joint rpy/axis（如 joint1 `axis 0 0 1`）套到 CR30H 等其它 vendor URDF 上。

## 落点（阶段 2 结束时应存在）

```text
<domain_pkg>/devices/<robot_device>/
  moveit_model.py
  robot_module.py
  models/
    model.yaml
    <vendor>.urdf
    meshes/<mesh_slug>/
  tests/
    test_moveit_model.py
```

## 关节命名（阶段 2 硬规则）

→ 全文 [`joint-naming.md`](joint-naming.md)

- `model.yaml` → `kinematic_joints` = `joint_1` … `joint_6`（**禁止** `cr7_joint_*`）
- vendor `joint1` → canonical `joint_1`；qualified 由 kit 生成 `{device_id}_joint_1`
- 换 vendor 型号时 **不改** canonical 关节名

## moveit_model.py 步骤

1. **阶段 1 已完成**：vendor URDF 为上游只读拷贝（见 [`asset-layout-checklist.md`](asset-layout-checklist.md)）  
2. 只读 URDF，列出 link/joint **原名** → 填 `link_names` / `joint_names`（旋转轴 → `joint_N`）  
3. `model.yaml` 锁 **上游** digest；`kinematic_joints` = `joint_1..joint_6`  
4. `build_moveit_model` → `assemble_six_axis_moveit_model(_ARM_SPEC, ...)`  
5. `disabled_collisions` / `mesh_paths` 按**当前** URDF 重填，禁止照搬其它型号表  

Python 结构只读参考：`unilab_robot_template/packages/unilab-arm-cr7/src/unilab_arm_cr7/moveit_model.py`

## SixAxisArmModelSpec 字段

| 字段 | 说明 |
|------|------|
| `source_urdf` | `models/<vendor>.urdf`（只读 vendor 副本） |
| `expected_source_digest` | = 上游 vendor SHA256 = `model.yaml` → `source.sha256` |
| `mesh_paths` | 与 URDF 同包的 STL 文件名 |
| `link_names` / `joint_names` | vendor 原名 → canonical |
