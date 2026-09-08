# 领域自有机械臂 / 导轨：完整迁移清单（通用）

从 catalog 引用（`unilab_arm_*`）或空白，**一次性**迁到领域 L3 自有型号。禁止分阶段只改 URDF 或只改 provider。

`<robot_device>`、`<vendor>.urdf>`、`<mesh_slug>`、`<slug>` 一律从用户领域仓**磁盘与 @device 声明**读取，禁止写死厂商。

---

## 阶段 0：范围确认

读 [`use-unilab-arm-package`](../../use-unilab-arm-package/SKILL.md) 不变量：两 device（导轨+臂）、图 parent/child、禁止合一体 MoveIt。

确认要迁的 `@device` id（与物理图一致）。

---

## 阶段 1：资产落盘（L3）

在 `devices/<robot_device>/models/` 齐备：

```text
model.yaml              # schema: unilab.robot-model/v1
<vendor>.urdf
meshes/<mesh_slug>/*.STL   # basename 与 URDF mesh 引用一致
```

- 算 `vendor.urdf` SHA256 → 写入 `model.yaml` → `source.sha256`
- **禁止** `model.yaml` 放在 device 根目录

---

## 阶段 2：`moveit_model.py`（从 vendor URDF 建映射）

1. 读 URDF，列出全部 link / joint 名  
2. 定 `canonical_joint_names`（与将写入 `model.yaml` → `kinematic_joints` 相同）  
   - 若已有 PointSet v3：grep 点位与 manifest，**保持关节名**或同步改点位（二选一，须完整）  
3. 填 `link_names` / `joint_names` / `disabled_collisions` / `mesh_paths`  
4. 实现 `build_moveit_model` → `assemble_six_axis_moveit_model`  
5. 实现 `build_joint_state_name_map`  
6. 可选：同目录 `robot_module.py`（Module API v1，结构只读参考 template catalog 六轴包）

**禁止**：映射表来自另一个型号的 catalog 拷贝而未对照当前 URDF。

---

## 阶段 3：`device.py` 与 provider（与资产同 PR）

```python
model={
    "type": "package_moveit",
    "provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model",
    "source_digest": "<models/model.yaml source.sha256>",
    # mount_yaw_deg 等保留原图/install 约定
}
```

- provider 与 `moveit_model.py` **同时**切换，不得长期 catalog + 领域 URDF 混用  
- `shape` / xacro 块本阶段**不改**（FE 在阶段 6）

导轨（若有）：`static_layout` + `kinematic_model` 或 catalog `unilab_rail_linear`，见 invariants。

---

## 阶段 4：测试与检查器

```text
devices/<robot_device>/tests/test_*_moveit_model.py   # 无则新建
```

```bash
# 领域仓根
python <unilab_robot_template>/.cursor/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain .
pytest devices/<robot_device>/tests/ -q
```

本地：

```python
build_moveit_model(device_id="<graph device id>")
# execution_urdf 含 visual；mesh_paths 文件均存在
```

---

## 阶段 5：OS + RViz 端到端（loop 直到通过）

1. 重启 OS（非只刷新 RViz）  
2. Fixed Frame = `world`  
3. 有 STL（非仅 TF）；mock 零位臂体不散架  
4. MoveIt 规划组关节与 `kinematic_joints` 一致  

HTTP 自检（可选）：

- `GET /api/v1/kinematic-models/<device_id>.urdf`  
- `GET /api/v1/kinematic-models/<device_id>/meshes/<basename>`

---

## 阶段 6：FE 对齐（完整迁移必做）

RViz 权威。另执行 [`fe-must-match-rviz`](../../../../.cursor/skills/fe-must-match-rviz/SKILL.md)：

- 更新 `shape` / xacro / 图 `parent_link`、旋转  
- **禁止**在 RViz 未通过前改 FE 猜姿态  

完整迁移 = 阶段 1–6 全部完成，不是「只改 MoveIt provider」。

---

## 完成标准（全部勾选）

- [ ] `models/` 三件套 + digest 锁定  
- [ ] 映射 100% 来自当前 vendor URDF  
- [ ] `device.py` provider + digest 与 model.yaml 一致  
- [ ] checker + pytest 通过  
- [ ] RViz 有 mesh、零位正常  
- [ ] FE 已与 RViz 对齐（或用户明确暂缓 FE 并记录债务）  

---

## 禁止

- 写死某实验室/厂商到 skill 或代码  
- 只迁 URDF 或只迁 provider（不完整迁移）  
- 改 OS / template 规避领域侧缺失  
- 未验证 RViz 即宣称迁移完成  
