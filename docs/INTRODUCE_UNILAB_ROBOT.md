# 一键引入 unilab_robot_template 机械臂（Introduce Uni-Lab Robot）

把 `unilab_robot_template` 里的机械臂型号**以最小成本**挂进领域仓：一条命令写 `pyproject.toml` 依赖、`@device model.provider`、`source_digest`；可选安装 L1 包或继续 domain migrate。

| 读者 | 入口 |
|------|------|
| 人类操作 | 本文 |
| Agent 细则 | `skills/introduce-unilab-robot/SKILL.md` |
| 脚本 | `skills/introduce-unilab-robot/scripts/introduce_arm.py` |

---

## 30 秒上手

在 **Uni-Lab-Core 仓根**或 **unilab_robot_template 仓根**执行（路径按你当前 cwd 调整）：

```bash
# 情况 A：引用 template 内 catalog 型号（cr5 / cr7）—— 推荐新领域仓
e:/miniforge3/envs/unilab/python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/GitHub/new/Uni-Lab-Core/Uni-Lab-pTLC \
  --device robot \
  --catalog cr5 \
  --apply --install

# 情况 B：引用领域仓自有 URDF（须已有 models/ + moveit_model.py）
e:/miniforge3/envs/unilab/python unilab_robot_template/skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab \
  --device szlab_mixer_robot \
  --domain-owned \
  --apply --migrate
```

不加 `--apply` 时为 **dry-run**：只打印 JSON 报告，不写磁盘。

---

## 两种模式怎么选

| 你的 URDF 在哪 | 模式 | 一句话 provider |
|----------------|------|-----------------|
| template `packages/unilab-arm-*`（L1 catalog） | `--catalog cr5` / `cr7` | `unilab_arm_<slug>:build_moveit_model` |
| 领域仓 `devices/<robot_device>/models/`（L3 自有） | `--domain-owned` | `<domain_pkg>.devices.<robot_device>.moveit_model:build_moveit_model` |

同一领域仓可以 **catalog 臂 + domain 臂并存**（不同 `@device` 各跑各的命令）。

---

## 情况 A：catalog 型号（L1）

### 一条命令做什么

| 步骤 | 动作 |
|------|------|
| 1 | 从 template `packages/unilab-arm-<slug>/.../model.yaml` 读取 `source.sha256` |
| 2 | 向领域仓 `pyproject.toml` 追加 `unilab-arm-<slug>`、`unilab-robot-runtime`（若尚未声明） |
| 3 | 创建或更新 `devices/<robot_device>/device.py` 里 `@device(model={...})` |
| 4 | `--install` 时 `pip install -e` template 对应 L1 包 |
| 5 | 默认跑 `check_domain_arm_assembly.py`（可用 `--skip-check` 跳过） |

### 等价手写（了解即可，不必手改）

```python
@device(
    id="<robot_device>",
    model={
        "type": "package_moveit",
        "provider": "unilab_arm_cr5:build_moveit_model",
        "source_digest": "<packages/unilab-arm-cr5/.../model.yaml 里的 source.sha256>",
    },
)
```

### 当前可选 slug

| slug | L1 发行包 | provider 一句话 |
|------|-----------|-----------------|
| cr5 | `unilab-arm-cr5` | `unilab_arm_cr5:build_moveit_model` |
| cr7 | `unilab-arm-cr7` | `unilab_arm_cr7:build_moveit_model` |

### 之后还要做什么

catalog **只解决机械臂型号引用**。导轨 + 臂整机 pTLC 式装配（双 `@device`、物理图 parent、禁止第七轴）见 [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md)。

---

## 情况 B：领域自有模型（L3，`model_ownership: domain`）

### 前提（脚本不会从零创建）

```text
devices/<robot_device>/
  models/model.yaml          # 含 source.sha256
  models/<vendor>.urdf       # 厂商只读副本
  moveit_model.py            # Python 映射（不改 URDF 几何）
  device.py                  # 可无；脚本可 scaffold 最小壳
```

### 一条命令做什么

| 步骤 | 动作 |
|------|------|
| 1 | 从 `models/model.yaml` 读 `source.sha256` |
| 2 | 写 `@device` 的 `model_ownership: domain` + 领域 provider |
| 3 | `--migrate` 时继续跑 `migrate_domain_arm.py --apply`（robot_module / attach 链 / 四门禁） |

### 等价手写

```python
"provider": "szlab_poly_studio.devices.szlab_mixer_robot.moveit_model:build_moveit_model"
```

完整移植、RViz / attach 验收见 [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md)。

---

## 参数一览

| 参数 | 必填 | 含义 |
|------|------|------|
| `--domain` | 是 | 领域仓根目录 |
| `--device` | 是 | 图（Graph）里机械臂 `@device` 的 `"id"` |
| `--catalog SLUG` | 二选一 | catalog 模式：`cr5` / `cr7` |
| `--domain-owned` | 二选一 | 领域 L3 自有 URDF |
| `--apply` | 否 | 真正写入；默认 dry-run |
| `--install` | 否 | catalog：`pip install -e` template L1 包 |
| `--migrate` | 否 | domain-owned：链式 migrate |
| `--skip-check` | 否 | 跳过 `check_domain_arm_assembly.py` |
| `--domain-pkg` | 否 | 覆盖自动探测的 Python 包名（默认读 `pyproject.toml`） |

### 参数说明

- `<领域仓根>`：如 `F:/GitHub/new/Uni-Lab-Core/Uni-Lab-SZLab`
- `<robot_device>`：图里节点 `"id"` = `devices/<robot_device>/` 文件夹名（SZLab 示例：`szlab_mixer_robot`）

---

## 输出与验收

脚本 stdout 为 JSON，关键字段：

```json
{
  "ok": true,
  "dry_run": false,
  "mode": "catalog",
  "provider_one_liner": "unilab_arm_cr5:build_moveit_model",
  "dependencies_added": ["unilab-arm-cr5>=0.1,<0.2"],
  "device_py": "demo_lab/devices/my_robot/device.py",
  "checker": { "exit_code": 0 }
}
```

- `ok: false` → 看 `error` 或 `checker` / `migrate` / `pip_install`
- catalog 装配全量规则仍以 [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) 为准；本脚本只保证**型号引用链**写对

---

## 常见问题

| 现象 | 处理 |
|------|------|
| `找不到 unilab_robot_template 根目录` | 在 Uni-Lab-Core  monorepo 内执行，或保证 cwd 能向上找到 `packages/unilab-arm-cr5` |
| `无法推断 Python 包名` | 加 `--domain-pkg szlab_poly_studio`，或补全 `pyproject.toml` 的 `[tool.setuptools.packages.find]` |
| domain-owned 报缺 `model.yaml` | 先把 vendor URDF 拷到 `devices/<device>/models/`，算好 digest 写入 `model.yaml` |
| checker 红 | catalog 整机未配齐（导轨 / 图 parent / 双 device）→ [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) |
| 已有 `device.py` 但 provider 不对 | 再跑 `--apply`；脚本会 patch 现有 `@device(model=...)` 块 |

---

## 相关文档

| 文档 | 何时读 |
|------|--------|
| [USE_UNILAB_ARM_PACKAGE.md](./USE_UNILAB_ARM_PACKAGE.md) | catalog 臂 + 导轨整机装配 |
| [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md) | 领域自有 MoveIt 完整移植 |
| `skills/package-arm-moveit/SKILL.md` | 把**新**厂商 URDF 打成 L1 发行包 |
