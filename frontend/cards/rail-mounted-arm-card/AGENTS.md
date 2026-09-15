# SZLab Mixer 机械臂设备卡片约束

- 只使用 `@unilab/device-card-sdk` 与 Host 提供的 `u-*` 元素。
- 禁止 `fetch`、`WebSocket`、`EventSource`、`XMLHttpRequest`、Node.js 和动态 import。
- 手动独占（Exclusive）只用 Host Bridge 的 `read/acquire/releaseManualExclusive`。
- 动作（Action）只用 `callAction` 或 `u-action-button`，不得绕过风险确认。
- 权限变化必须同步修改 `card.manifest.json` 与 `authoring-context.json`。
- Mixer 点位用 PointSet `target_ref`（例如 `s07_process_warehouse.S0722.interaction_seed`），不要传 pTLC 的 `P9` 别名。
