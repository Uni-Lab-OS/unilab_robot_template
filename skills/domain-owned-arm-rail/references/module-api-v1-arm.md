# 机械臂 Module API v1（领域包内实现）

实现入口：**`<domain_pkg>/devices/<device>/robot_module.py`**

只读参考 template：`unilab_robot_template/packages/unilab-arm-cr7/src/unilab_arm_cr7/`

## Agent：直接写文件

**一键命令（等同下面全部手工步骤）：**

```bash
python domain-owned-arm-rail-skill/scripts/migrate_domain_arm.py \
  --domain <领域仓根> --device <robot_device> --apply
```

exit 0 = 移植完成。手工模板见 [`code-scaffold.md`](code-scaffold.md)。

## 模块常量

```python
MODULE_API_VERSION = 1
MODULE_KIND = "arm"
MODULE_VERSION = "0.1.0"  # 须与 _ModuleRef version 一致
```

## 必须导出的符号

| 符号 | 实现位置 |
|------|----------|
| `MODULE_API_VERSION` / `MODULE_KIND` / `MODULE_VERSION` | `robot_factory` 或 `robot_module` |
| `MODEL_DESCRIPTOR` | **导入**自 `moveit_model` |
| `build_moveit_model` | **导入**自 `moveit_model` |
| `build_joint_state_name_map` | **导入**自 `moveit_model` |
| `create_arm_module` | `robot_factory` |
| `create_moveit_backend` / `create_plc_backend` / … | `robot_factory` + 本 device `adapters/` |

runtime 加载时检查（`unilab_robot_runtime.factory._module_impl`）：

- `MODULE_API_VERSION == 1`
- `MODULE_KIND == "arm"`
- `__version__ == manifest 里 arm 的 version`

缺任一项 → `不支持 Robot Module API v1`。

## 资产位置

```text
devices/<device>/models/
  model.yaml
  vendor.urdf
  meshes/<slug>/
```

`@device source_digest` = `models/model.yaml` → `source.sha256`（= **上游 vendor URDF** SHA256，禁止手改 URDF 后再算）。

`vendor.urdf`：**只读拷贝**，禁止改 joint origin/axis。见 [`asset-layout-checklist.md`](asset-layout-checklist.md)。

## `@device` provider（不要改指 robot_module）

```python
"provider": "<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model"
```

## 与 moveit_model.py 分工

| 文件 | 职责 |
|---|---|
| `moveit_model.py` | 读**只读** vendor URDF → 映射/SRDF、`build_moveit_model`（不改磁盘 URDF 几何） |
| `robot_module.py` | Module API v1、`create_arm_module`（runtime 装配） |

`moveit_runtime_assembly` 里 `arm=_ModuleRef(...).python_package` **必须**指向 `robot_module`。

只改 moveit_model、不建 robot_module → **不能移植完成**。

## 例外：template catalog

跨实验室复用 catalog 时 URDF 在 template，不走本 skill 默认路径：

```python
"provider": "unilab_arm_<slug>:build_moveit_model"
```
