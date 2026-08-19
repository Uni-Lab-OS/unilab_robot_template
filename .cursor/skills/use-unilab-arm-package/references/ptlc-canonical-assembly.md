# pTLC 权威装配样本

以下路径相对 `Uni-Lab-pTLC`。改领域仓时对照这些文件复制结构，只替换实例身份和型号。

## 机械臂设备

`ptlc_station/devices/robot.py` `@device(id='robot', model=…)`：

- `type`: `package_moveit`
- `provider`: `unilab_arm_cr5:build_moveit_model`
- `source_digest`: `8c8b9ea935fd83122b19b572c84d107e81b4864d4310c94d0906cc361e7631c2`
- 另有本地 `format`/`entry` 供工作区物料目录投影；执行仍走 Provider
- **没有** `mount_yaw_deg`
- `post_init` 仅当 `standard_execution_backend` ∈ {`moveit`, `moveit_sim`} 时绑定 commissioning

测试：`tests/test_registry.py::test_robot_catalog_uses_exact_cr5_package_moveit_provider`

## 导轨设备

`ptlc_station/devices/rail.py` `@device(id='rail', model=…)`：

- `type`: `package_static`
- `provider`: `ptlc_station.devices.static_layout:build_rail_11y`
- `joint_state_provider`: `unilab_rail_linear:build_kinematic_model`
- `joint_state_source_digest`: `9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d`
- `post_init` 仅在 `mock`/`simulation` 时挂仿真轴发布，PLC 不会误开第二套轴

测试：`tests/test_registry.py::test_rail_catalog_keeps_static_shell_and_joint_state_provider`

## 静态外壳

`ptlc_station/devices/static_layout.py` 的 `_bundle`：

```python
root_link = f"{member_id}_base_link"
```

导轨与货架共用这个根命名。FE 合成时把外壳视觉拷到运动学 `{id}_rail_base`；MoveIt 的 `robot_description` 里外壳根与运动学根是**两个不同 link**。

测试：`tests/test_rail_simulation.py::test_rail_kinematic_urdf_is_world_mounted_for_rviz`  
断言静态只有 `rail_layout_world_joint`（fixed），运动学有 `rail_kinematic_world_joint` + `rail_rail_joint`（prismatic）。

## 物理图（local-debug）

`deployment/graphs/local-debug.json`：

- `rail`：`parent` null，`children: ["robot"]`，世界位姿毫米，`standard_execution_backend: mock`
- `robot`：`parent: "rail"`，局部位姿 `z: 200` mm，`position.rotation.z: 180`（度），`standard_execution_backend: moveit_sim`
- `config.robot_cell_manifest` 锁定 L3 digest（点位、标定、HardwareProfile）

`config.rotation.z = π` 会被 `apply_graph_world_mount` 覆盖成图合成结果，不要把它当成第二份安装角。

## 单元清单（WorkCell）

`ptlc_station/deployment/robot_cell/manifest.v1.yaml` 把臂与导轨写成两个 `components`，各锁 `model_ref` + digest。领域仓若有同等资产，保持双组件，不要合成一个 `arm_base_joint` 组件。

## 检查器金样

```bash
python unilab_robot_template/.cursor/skills/use-unilab-arm-package/scripts/check_domain_arm_assembly.py --domain Uni-Lab-pTLC
```

必须退出 0。若 pTLC 变红，先修样本或更新本参考，禁止把检查器改松来迁就错误装配。
