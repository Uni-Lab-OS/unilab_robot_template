# Greenfield Workbench FE 3D 门禁（E2E Gate）

**何时读**：`introduce-unilab-robot` 用 `--new-domain` + `--scaffold-graph` 建好最小导轨+机械臂领域包后，在 Workbench **桌面版**验证物料（Material）3D 视图前，Agent **必须**读本页并按清单验收。

本页来自 `Uni-Lab-DemoRailCr5` 首次联调踩坑；不要假设「checker exit 0 = FE 能出 3D」。

---

## 1. OS 运行时前置（Uni-Lab-OS）

Greenfield graph 的 **机械臂** 依赖 OS 侧 v2 **设备网格（Device Mesh）** 管线，不是旧版 `ResourceVisualization` 直启路径。OS worktree 至少要有：

| 能力 | 作用 |
|------|------|
| `prepare_device_mesh_runtime` / `compile_kinematic_runtime` | 编译 URDF、往资源树写入 `rendering.model` + `rendering.kinematics` |
| `ResourceTreeSet.device_nodes` + `HostNode` 遍历 `device_nodes` | 地轨子节点 **机械臂（robot）** 也会被 Edge 初始化（只 init 根节点会漏臂） |
| `install_host_joint_state_projection` + `joint_state_owners` | Host → Edge 关节状态（JointState）投影 |
| `/api/v1/kinematic-models/{device}.urdf` | FE 拉 URDF / mesh |
| `/api/v1/device-telemetry/events` | FE 订阅关节 SSE |
| 库存启动投影指纹剥离运行时 kinematics | 避免 `compile_kinematic_runtime` 后 fingerprint 冲突；旧库无 kinematics 时需 `reset-local` |

**判断**：若 Edge 日志只有 `Initializing device: rail`、没有 `robot`，或 `/api/v1/device-telemetry/events` 404、或 `/api/v1/materials/graph` 里机械臂只有 `dimensionsMm` —— 先修 OS 合入/重启，不要先改 FE 对齐。

---

## 2. Workbench 启动（桌面版）

在 FE worktree `apps/workbench`：

```bash
node scripts/start-workbench.mjs --desktop \
  --workspace <领域包根，例如 Uni-Lab-DemoRailCr5> \
  --os-project <OS worktree，含上述 v2 能力> \
  --python-env e:/miniforge3/envs/unilab \
  --port 3100
```

**全量清理后再拉**（禁止只杀 3100）：

- 释放 **3100**
- 杀掉 **Electron**、**start-workbench** / workbench **node**
- 杀掉本 workspace 下 **UniLab OS**（`unilabos.app.main`、`workspace_host.host` 等）
- 确认 **3100 未被其它工作区（例如 Uni-Lab-SZLab）占用**

Backend `ready` 后若 Edge 仍为 `idle`，在同一 workspace 执行：

```bash
python -m unilabos.app.main workspace start \
  --workspace <领域包根> --component all --wait 180
```

（或 Workbench UI 里等价的 **os.start**。）

---

## 3. 库存（Inventory）陈旧快照

若 OS 管线已合入但 `/api/v1/materials/graph` 里设备仍只有 `rendering.dimensionsMm`、无 `model.path` / `kinematics`，多为 **bootstrap 在合入前已写入**、指纹匹配时整图跳过更新。

**优先**：只 **重启 Backend**（或 `workspace restart --component backend`）。OS 会在指纹不变时 **增量合并** `rendering.model` / `kinematics` / `parent_link`（来自 `compile_kinematic_runtime`），导轨与机械臂都应补齐。

**仍不对时再 reset-local**（会删可重建 Local Domain / Edge 状态，不删领域仓源码）：

```bash
python -m unilabos.app.main workspace reset-local \
  --workspace <领域包根> --yes --wait 120
python -m unilabos.app.main workspace start \
  --workspace <领域包根> --component all --wait 180
```

---

## 4. Agent 验收清单（必须跑，不能只看 UI）

Backend 地址见 `.unilabos/runtime/workbench/session.json` → `components.backend.address`（例如 `http://127.0.0.1:54907`）。

| # | 检查 | 期望 |
|---|------|------|
| 1 | Edge 日志 | `Initializing device: rail` **且** `Initializing device: robot` |
| 2 | `GET /api/v1/kinematic-models/robot.urdf` | HTTP 200，合法 URDF |
| 3 | `GET /api/v1/materials/graph` → **rail** 与 **robot** 物料 | 二者均有 `rendering.model.path`（`/api/v1/kinematic-models/{id}.urdf`）；有 `rendering.kinematics`；robot 有 `rendering.parent_link`（如 `rail_rail_carriage`） |
| 3b | `GET /api/v1/kinematic-models/rail.urdf` 与 `.../rail/meshes/arm_slideway.stl`（或 `collision.stl`） | HTTP 200；**mesh > 500B** 且 STL 头 **80B**、三角面数合法（坏 STL → FE 静默空 mesh） |
| 4 | `GET /api/v1/device-telemetry/events` | 非 404（SSE 长连接，用短 timeout 探测路由即可） |
| 5 | Workbench `session.json` | `workspacePath` = 目标领域包，不是旧 SZLab |

全部通过后再看 Workbench 3D；RViz 为准时只改 FE 对齐（见 `fe-must-match-rviz` skill）。

---

## 5. 常见失败速查

| 现象 | 根因 | 动作 |
|------|------|------|
| Edge 只 init rail | `HostNode` 只遍历 `root_nodes` | OS：`device_nodes` 补丁 |
| 机械臂无 URDF / kinematics | 未走 `prepare_device_mesh_runtime` | OS：`main.py` 合入 v2 启动 |
| device-telemetry 404 | 路由未挂 | OS：`create_device_telemetry_router` |
| Backend 启动 fingerprint 冲突 | 运行时 kinematics 写入指纹 | OS：`_config_without_runtime_kinematics` |
| graph 无 kinematics、API 正常 | 陈旧 inventory.db | `workspace reset-local` |
| 3100 被顶、Workbench 52s 退出 | 端口 / 旧工作区进程 | 全量清理 + 确认 workspace 参数 |
| API 200 但 FE 只见机械臂不见导轨 | 导轨 mesh 无效（184B 空占位、**STL 头非 80B**）或未用默认 SZLab 资产 | Greenfield 应用 skill 默认 **SZLab arm_slideway.stl**；或 `--rail-stl` 指向合法 STL 并更新 `source_digest`，重启 Backend |

---

## 6. 导轨（rail）package_static mesh

Greenfield scaffold 的导轨注册表为 **`package_static` + `joint_state_provider`**（`unilab_rail_linear`），默认视觉 mesh 为 **SZLab `arm_slideway.stl`**（与 pTLC 导轨一致）。OS `collect_package_joint_state_owners` 会把导轨与机械臂一并编译进 `kinematic-models` 目录；`compile_kinematic_runtime` 给 **owner 设备**（含 `rail`）写入 `rendering.model` + `kinematics`。

**Agent 必须同时验收 rail 与 robot**（见清单 #3 / #3b）。若 rail 只有 `dimensionsMm`：

1. 确认 OS worktree 含 v2 管线且 Backend 已重启（触发运行时 rendering 增量合并）；
2. 仍失败再 `reset-local`；
3. **禁止**为凑 FE 改 RViz / `collision_asset_pose`（`fe-must-match-rviz`）。

**依赖**：领域包 `pyproject.toml` 须含 `unilab-rail-linear`（scaffold 已写）。缺包时 Backend 启动会在编译导轨模型时失败关闭。

---

## 7. 相关 skill

| 下一步 | Skill |
|--------|-------|
| pTLC 装配 / checker | [use-unilab-arm-package](../../../use-unilab-arm-package/SKILL.md) |
| FE 对齐 RViz | 仓库 `.cursor/skills/fe-must-match-rviz/SKILL.md` |
| 说要重启就重启 | `restart-when-needed` |
