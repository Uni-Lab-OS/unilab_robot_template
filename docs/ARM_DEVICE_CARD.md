# 机械臂设备卡片（Arm Device Card）

template 提供 **导轨 + 机械臂调试卡片** 的通用 Python mixin 与前端 Web Component；领域仓只注入 binding、PointSet、vision 资产路径与 branding。

| 层 | 路径 |
|----|------|
| Python mixin | `unilab_robot_runtime.device_card.RailMountedArmCardMixin` |
| Vision 读写 | `unilab_robot_runtime.vision_calibration` |
| 前端 canonical | `frontend/cards/rail-mounted-arm-card/` |
| 一键 scaffold | `skills/introduce-unilab-robot/scripts/introduce_arm.py --with-card` |

## 领域仓接入（Python）

1. 设备类继承 `RailMountedArmCardMixin`（或领域薄包装，如 SZLab `SzlabMixerArmCardMixin`）。
2. 在**领域包内**的薄包装 mixin 上重新声明全部 card `@action`（template runtime 包不在社区包 AST 扫描范围内）。
3. 实现 `_arm_card_context()` → `ArmCardContext`（catalog / rail / vision 钩子）。
4. 可选 `_vision_calibration_paths()` → `VisionCalibrationPaths`（标定 YAML 仍在领域 `deployment/`）。
5. 覆盖 `_ensure_card_binding()` / `_point_set_resolver()` 若需懒绑定 MoveIt。

### OS 注册表（Registry）静态扫描约定

Workbench 以 **Host Authoring Context** 校验 `card.manifest.json` 权限；Context 来自 OS 对 `@device` / `@action` 的 AST 扫描。领域 mixin 文件须满足：

| 规则 | 原因 |
|------|------|
| 领域 import 用**绝对路径**（`szlab_poly_studio.devices...`），不要用 `from .foo import` | 相对 import 会导致动作合同（Action Contract）`invalid_module_scope` |
| card `@action` 返回注解用 `dict[str, JSONValue]` 或 `None` | 外部 TypedDict（如 `RobotDebugSnapshot`）无法被 Workflow 静态解析 |
| mixin 继承链需能被 OS `_lookup_class_record` 合并进 `@device` 类 | import 模块前缀须与扫描根（`devices.*`）一致 |

离线开发：保留 `authoring-context.json`；Backend 未就绪时 Workbench 可回退 `project-preview` 仅构建卡片 UI，**接 Host 调动作**仍须 Backend 启动。

SZLab 参考：[`szlab_mixer_arm_card.py`](../../Uni-Lab-SZLab/szlab_poly_studio/devices/szlab_mixer_robot/szlab_mixer_arm_card.py)

## 领域仓接入（前端）

1. Canonical UI 在 `frontend/cards/rail-mounted-arm-card/`。
2. 领域仓 `frontend/cards/<device>-card/` 保留：
   - `card.manifest.json`（`deviceTypes`、permissions）
   - `src/branding.ts`（标题文案）
3. 或用 introduce 自动生成：

```bash
python skills/introduce-unilab-robot/scripts/introduce_arm.py \
  --domain <领域仓根> --device <robot_device> --catalog cr5 \
  --with-card --apply
```

## Vision 分层

- **template**：登记读写 API、`calibrate_*` / `record_marker` 逻辑。
- **领域仓**：`deployment/robot_cell/assets/` 下 YAML 资产与 revision。

## 相关

- [INTRODUCE_UNILAB_ROBOT.md](./INTRODUCE_UNILAB_ROBOT.md) — 机械臂型号引用
- [DOMAIN_OWNED_ARM_RAIL.md](./DOMAIN_OWNED_ARM_RAIL.md) — L3 MoveIt 移植
