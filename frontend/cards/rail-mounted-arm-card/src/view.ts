import type {
  CardTab,
  ExclusiveState,
  PointTarget,
  RobotDebugSnapshot,
  TcpPose,
  VisionMarker
} from './model'
import { defaultMarkerRefForWarehouse, markerOptionsForSnapshot } from './model'
import { cardBranding } from './branding'

export interface RobotCardViewState {
  actionBusy: boolean
  exclusiveState: ExclusiveState
  hostOnline: boolean
  includeVisionOnRecord: boolean
  jointStep: number
  message: string
  moving: boolean
  selectedMarkerRef: string
  selectedTargetRef: string
  snapshot: RobotDebugSnapshot | null
  tab: CardTab
  tcpFrame: 'arm_base' | 'tool'
  tcpStep: number
}

/** 生成接近 pTLC 既有调试工作台的信息密度和视觉层次。 */
export function renderRobotCard(state: RobotCardViewState): string {
  const snapshot = state.snapshot
  const selected = snapshot?.pointTargets.find(
    (point) => point.targetRef === state.selectedTargetRef
  ) ?? snapshot?.pointTargets[0] ?? null
  return `
    <style>${styles}</style>
    <article class="robot-card" data-card-ready="${snapshot ? 'true' : 'false'}">
      <header class="hero">
        <div>
          <p class="eyebrow">${cardBranding.eyebrow}</p>
          <h1>${cardBranding.title}</h1>
          <p class="subtitle">${cardBranding.subtitle}</p>
        </div>
        <div class="hero-actions">
          ${badge(state.hostOnline ? 'OS 在线' : 'OS 离线', state.hostOnline ? 'ok' : 'danger')}
          ${badge(exclusiveLabel(state.exclusiveState), state.exclusiveState === 'exclusive' ? 'accent' : 'muted')}
          <button class="button ghost compact" data-refresh type="button">刷新</button>
        </div>
      </header>

      <nav class="tabs" aria-label="机械臂调试功能">
        ${tabButton('points', '点位与姿态', state.tab)}
        ${tabButton('jog', 'Jog 与示教', state.tab)}
        ${tabButton('vision', '视觉校准', state.tab)}
        <span class="revision">PointSet · ${escapeHtml(snapshot?.pointSetRevision ?? '读取中')}</span>
      </nav>

      <main class="content">
        ${state.tab === 'points'
          ? renderPointPanel(snapshot, selected)
          : state.tab === 'jog'
            ? renderJogPanel(state, snapshot, selected)
            : renderVisionPanel(state, snapshot, selected)}
      </main>

      <footer class="command-bar">
        <div class="exclusive-controls">
          <button class="button ghost" data-exclusive="acquire" type="button"
            ${state.exclusiveState === 'exclusive' ? 'disabled' : ''}>取得调试控制</button>
          <button class="button ghost" data-exclusive="release" type="button"
            ${state.exclusiveState !== 'exclusive' ? 'disabled' : ''}>释放</button>
        </div>
        <div class="command-actions">
          <button class="button ghost" data-home type="button"
            ${motionDisabled(state) ? 'disabled' : ''}>回原点</button>
          <button class="button primary" data-move-to-point type="button"
            ${motionDisabled(state) || !selected ? 'disabled' : ''}>
            ${state.moving ? '<span class="spinner"></span>规划执行中' : `移动到 ${escapeHtml(selected?.sourcePoint ?? '目标点')}`}
          </button>
        </div>
      </footer>
      <p class="message ${messageTone(state.message)}" aria-live="polite">${escapeHtml(state.message || readyMessage(state))}</p>
    </article>
  `
}

function renderPointPanel(
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null
): string {
  return `
    <section class="point-panel">
      <div class="section-heading">
        <div><span class="kicker">动作步骤</span><h2>PointSet 目标</h2></div>
        <span class="count">${snapshot?.pointTargets.length ?? 0} 点</span>
      </div>
      <div class="point-list" role="listbox" aria-label="目标点位">
        ${(snapshot?.pointTargets ?? []).map((point, index) => `
          <button class="point-row ${point.targetRef === selected?.targetRef ? 'selected' : ''}"
            data-point-target="${escapeHtml(point.targetRef)}" type="button" role="option"
            data-source-point="${escapeHtml(point.sourcePoint)}"
            aria-selected="${point.targetRef === selected?.targetRef}">
            <span class="point-index">${index + 1}</span>
            <span class="point-copy">
              <strong>${escapeHtml(point.sourcePoint)}</strong>
              <small>${escapeHtml(point.groupRef)} · ${escapeHtml(point.targetRef)}</small>
            </span>
            <span class="point-meta">${point.editable ? '可写回' : '只读'}</span>
          </button>`).join('') || '<div class="empty">正在读取活动 PointSet…</div>'}
      </div>
    </section>
    ${renderPoseMonitor(snapshot, selected, 'target')}
  `
}

function renderPoseMonitor(
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null,
  source: 'target' | 'live'
): string {
  const targetMode = source === 'target'
  const pose = targetMode ? selected?.tcpPose : snapshot?.tcpPose
  const joints = targetMode
    ? (selected?.jointPositionsDeg ?? []).map((positionDeg, index) => ({
        jointRef: `target_joint_${index + 1}`,
        positionDeg
      }))
    : (snapshot?.jointPositions ?? [])
  const railPosition = targetMode ? selected?.railPositionMm : snapshot?.rail?.positionMm
  const poseValues = [
    ['X', pose?.xyzMm[0], 'mm'], ['Y', pose?.xyzMm[1], 'mm'], ['Z', pose?.xyzMm[2], 'mm'],
    ['Rx', pose?.rotationXyzDeg[0], '°'], ['Ry', pose?.rotationXyzDeg[1], '°'], ['Rz', pose?.rotationXyzDeg[2], '°']
  ] as const
  return `
    <section class="monitor">
      <div class="monitor-head">
        <span>${targetMode ? '目标位置' : '实时姿态'} · TCP / 关节</span>
        <span>目标选择：<strong>${escapeHtml(selected?.sourcePoint ?? '—')}</strong></span>
      </div>
      ${targetMode
        ? `<div class="target-rail">目标导轨 ${formatNumber(railPosition ?? undefined)} mm</div>`
        : ''}
      <h3>TCP 位姿 <small>${escapeHtml(pose?.frameRef ?? '')}</small></h3>
      <div class="pose-grid">
        ${poseValues.map(([label, value, unit]) => `
          <div class="pose-value"><small>${label}</small><strong>${formatNumber(value)}</strong><span>${unit}</span></div>
        `).join('')}
      </div>
      <div class="joint-heading"><h3>关节位置</h3><small>角度</small></div>
      <div class="joint-bars">
        ${joints.map((joint, index) => `
          <div class="joint-bar">
            <span>J${index + 1}</span>
            <i><b style="width:${jointPercent(joint.positionDeg)}%"></b></i>
            <strong>${formatNumber(joint.positionDeg)}°</strong>
          </div>
        `).join('') || '<div class="empty compact-empty">等待关节状态 SSE…</div>'}
      </div>
    </section>
  `
}

function renderJogPanel(
  state: RobotCardViewState,
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null
): string {
  const hasRail = snapshot?.capabilities.railMove === true && snapshot.rail !== null
  const railMinimum = snapshot?.rail?.travelMinMm
  const railMaximum = snapshot?.rail?.travelMaxMm
  return `
    <section class="jog-panel">
      <div class="section-heading">
        <div><span class="kicker">一步一命令</span><h2>Jog</h2></div>
        ${badge(snapshot?.idle ? 'MoveIt 空闲' : 'MoveIt 忙碌', snapshot?.idle ? 'ok' : 'muted')}
      </div>
      <div class="jog-settings">
        <label>关节步长 <input data-joint-step type="number" min="0.1" step="0.1" value="${escapeHtml(formatStep(state.jointStep, 1))}"> °</label>
        <label>TCP 步长 <input data-tcp-step type="number" min="0.1" step="0.1" value="${escapeHtml(formatStep(state.tcpStep, 2))}"> mm / °</label>
        <label>坐标系 <select data-tcp-frame>
          <option value="arm_base" ${state.tcpFrame === 'tool' ? '' : 'selected'}>基座</option>
          <option value="tool" ${state.tcpFrame === 'tool' ? 'selected' : ''}>工具</option>
        </select></label>
      </div>
      ${hasRail ? `
        <div class="rail-control">
          <div><span class="kicker">绝对导轨位置</span><strong>当前 ${formatNumber(snapshot?.rail?.positionMm)} mm</strong>
            <small>限位 ${formatNumber(railMinimum ?? undefined)}–${formatNumber(railMaximum ?? undefined)} mm</small></div>
          <label><input data-rail-position type="number" step="1"
            ${railMinimum === undefined ? '' : `min="${railMinimum}"`}
            ${railMaximum === undefined ? '' : `max="${railMaximum}"`}
            value="${snapshot?.rail?.positionMm ?? 0}"> mm</label>
          <button class="button rail" data-move-rail type="button"
            ${motionDisabled(state) ? 'disabled' : ''}>移动导轨</button>
        </div>` : ''}
      <div class="jog-columns">
        <div class="jog-group">
          <h3>关节 Jog</h3>
          ${(snapshot?.jointPositions ?? []).map((joint, index) => jogRow(
            `J${index + 1}`,
            joint.jointRef,
            formatNumber(joint.positionDeg) + '°',
            'joint'
          )).join('') || '<div class="empty">等待关节目录…</div>'}
        </div>
        <div class="jog-group">
          <h3>TCP Jog</h3>
          ${tcpJogRows(snapshot?.tcpPose).map((row) => jogRow(
            row.label, row.axis, row.value, 'tcp'
          )).join('')}
        </div>
      </div>
      <div class="teach-card">
        <div><span class="kicker">点位设置</span><h3>${escapeHtml(selected?.sourcePoint ?? '请选择点位')}</h3>
          <p>把当前新鲜完整机械臂${hasRail ? '与导轨' : ''}位置记录到作者目录中的目标；执行前需要再次确认。</p>
          <label class="vision-toggle">
            <input data-include-vision type="checkbox" ${state.includeVisionOnRecord ? 'checked' : ''}
              ${!selected?.editable ? 'disabled' : ''}>
            加入视觉校准的参数
          </label>
          ${state.includeVisionOnRecord ? renderMarkerSelect(state, selected) : ''}
        </div>
        <button class="button teach" data-record-point type="button"
          ${!selected?.editable || snapshot?.capabilities.compositePointRecord !== true || motionDisabled(state) ? 'disabled' : ''}>记录当前机械臂${hasRail ? '与导轨' : ''}位置</button>
      </div>
    </section>
    ${renderPoseMonitor(snapshot, selected, 'live')}
  `
}

function renderVisionPanel(
  state: RobotCardViewState,
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null
): string {
  const vision = snapshot?.vision
  const warehouseRef = selected?.groupRef ?? '—'
  const markerOptions = markerOptionsForSnapshot(snapshot, selected?.groupRef ?? '')
  return `
    <section class="vision-panel">
      <div class="section-heading">
        <div><span class="kicker">部署登记层</span><h2>视觉校准</h2></div>
        <span class="count">${escapeHtml(vision?.revision ?? '未读取')}</span>
      </div>
      <div class="vision-summary">
        <div><small>当前仓</small><strong>${escapeHtml(warehouseRef)}</strong></div>
        <div><small>摄像头外参</small><strong>${escapeHtml(vision?.cameraExtrinsicState ?? 'pending')}</strong></div>
        <div><small>TCP 标定</small><strong>${escapeHtml(vision?.tcpCalibrationState ?? 'pending')}</strong></div>
        <div><small>标定 revision</small><strong>${escapeHtml(vision?.calibrationRevision ?? '—')}</strong></div>
      </div>
      <div class="vision-actions">
        <button class="button primary" data-calibrate-camera type="button"
          ${motionDisabled(state) ? 'disabled' : ''}>摄像头外参标定</button>
        <button class="button primary" data-calibrate-tcp type="button"
          ${motionDisabled(state) ? 'disabled' : ''}>TCP 校准</button>
        <button class="button teach" data-record-marker type="button"
          ${motionDisabled(state) || !selected?.groupRef ? 'disabled' : ''}>marker 记录</button>
      </div>
      <div class="marker-list">
        <div class="joint-heading"><h3>已登记 marker</h3><small>${markerOptions.length} 项</small></div>
        ${markerOptions.length > 0
          ? markerOptions.map((marker) => renderMarkerRow(marker)).join('')
          : '<div class="empty compact-empty">尚无 marker 登记；可对当前仓执行 marker 记录。</div>'}
      </div>
    </section>
  `
}

function renderMarkerSelect(
  state: RobotCardViewState,
  selected: PointTarget | null
): string {
  const options = markerOptionsForSnapshot(state.snapshot, selected?.groupRef ?? '')
  const selectedRef = state.selectedMarkerRef
    || defaultMarkerRefForWarehouse(selected?.groupRef ?? '')
  return `
    <label class="marker-select">关联 marker
      <select data-marker-ref>
        ${options.map((marker) => `
          <option value="${escapeHtml(marker.markerRef)}"
            ${marker.markerRef === selectedRef ? 'selected' : ''}>
            ${escapeHtml(marker.label)} (${escapeHtml(marker.markerRef)})
          </option>`).join('')}
      </select>
    </label>`
}

function renderMarkerRow(marker: VisionMarker): string {
  const recorded = marker.recordedAt ? `已记录 ${escapeHtml(marker.recordedAt)}` : '未记录'
  return `<div class="marker-row">
    <strong>${escapeHtml(marker.label)}</strong>
    <small>${escapeHtml(marker.markerRef)} · ${escapeHtml(marker.deviceFrameRef)}</small>
    <span>${recorded}</span>
  </div>`
}

function jogRow(label: string, ref: string, value: string, kind: 'joint' | 'tcp'): string {
  return `<div class="jog-row"><strong>${label}</strong><small>${escapeHtml(value)}</small>
    <button data-jog-${kind}="${escapeHtml(ref)}" data-direction="negative" type="button" aria-label="${label} 负向点动">−</button>
    <button data-jog-${kind}="${escapeHtml(ref)}" data-direction="positive" type="button" aria-label="${label} 正向点动">＋</button></div>`
}

/** 把快照里的工具中心点（TCP）投影成 Jog 行当前读数。 */
function tcpJogRows(pose: TcpPose | null | undefined): Array<{
  axis: string
  label: string
  value: string
}> {
  const xyz = pose?.xyzMm ?? []
  const rotation = pose?.rotationXyzDeg ?? []
  return [
    { label: 'X', axis: 'x', value: `${formatNumber(xyz[0])} mm` },
    { label: 'Y', axis: 'y', value: `${formatNumber(xyz[1])} mm` },
    { label: 'Z', axis: 'z', value: `${formatNumber(xyz[2])} mm` },
    { label: 'RX', axis: 'rx', value: `${formatNumber(rotation[0])}°` },
    { label: 'RY', axis: 'ry', value: `${formatNumber(rotation[1])}°` },
    { label: 'RZ', axis: 'rz', value: `${formatNumber(rotation[2])}°` }
  ]
}

/** 把卡片记住的步长写成 input value；非法时回落到默认。 */
function formatStep(value: number, fallback: number): string {
  return Number.isFinite(value) && value > 0 ? String(value) : String(fallback)
}

function tabButton(value: CardTab, label: string, current: CardTab): string {
  return `<button class="tab ${value === current ? 'active' : ''}" data-tab="${value}" type="button" aria-selected="${value === current}">${label}</button>`
}

function badge(label: string, tone: 'ok' | 'accent' | 'muted' | 'danger'): string {
  return `<span class="badge ${tone}"><i></i>${escapeHtml(label)}</span>`
}

function exclusiveLabel(value: ExclusiveState): string {
  return value === 'exclusive' ? '调试控制已取得' : value === 'busy' ? '设备被占用' : '未取得调试控制'
}

function motionDisabled(state: RobotCardViewState): boolean {
  return state.actionBusy || state.moving || state.exclusiveState !== 'exclusive' || !state.snapshot?.idle
}

function readyMessage(state: RobotCardViewState): string {
  if (!state.hostOnline) return 'OS 设备离线，调试动作不可用。'
  if (!state.snapshot) return '正在读取 MoveIt 调试快照…'
  if (state.snapshot.stale) return 'MoveIt 快照已过期，请刷新后再操作。'
  if (state.exclusiveState !== 'exclusive') return '取得调试控制后可以执行 Jog、点位设置和目标移动。'
  return 'MoveIt 调试通道已就绪。'
}

function messageTone(message: string): string {
  return /失败|错误|不可用|离线/u.test(message) ? 'error' : ''
}

function jointPercent(degrees: number): number {
  return Math.max(4, Math.min(96, (degrees + 180) / 3.6))
}

function formatNumber(value: number | undefined): string {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(1) : '—'
}

function escapeHtml(value: string): string {
  return value.replace(/[&<>"']/gu, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  })[character] ?? character)
}

const styles = `
  :host { display:block; height:100%; color:#172033; font:13px/1.45 Inter,"PingFang SC","Microsoft YaHei",sans-serif; }
  * { box-sizing:border-box; }
  button,input,select { font:inherit; }
  button { cursor:pointer; }
  button:disabled { cursor:not-allowed; opacity:.42; }
  .robot-card { --teal:#009c91; --blue:#1687ee; height:100%; min-height:640px; display:grid; grid-template-rows:auto auto minmax(0,1fr) auto auto; background:linear-gradient(180deg,#f9fcfd 0,#f4f7fa 100%); border:1px solid #dce5ec; border-radius:15px; overflow:hidden; box-shadow:0 18px 45px rgba(42,66,88,.12); }
  .hero { display:flex; align-items:center; justify-content:space-between; gap:18px; padding:20px 22px 16px; background:rgba(255,255,255,.92); border-bottom:1px solid #e6edf2; }
  .eyebrow,.kicker { margin:0; color:#78909f; font-size:10px; font-weight:800; letter-spacing:.12em; text-transform:uppercase; }
  h1 { margin:3px 0 2px; font-size:17px; letter-spacing:-.01em; }
  .subtitle { margin:0; color:#7c8996; font-size:11px; }
  .hero-actions { display:flex; align-items:center; justify-content:flex-end; flex-wrap:wrap; gap:7px; }
  .badge { display:inline-flex; align-items:center; gap:6px; padding:5px 9px; border:1px solid #dce6ec; border-radius:999px; background:#f6f9fa; color:#63717d; font-size:10px; font-weight:700; white-space:nowrap; }
  .badge i { width:6px; height:6px; border-radius:50%; background:#96a6b2; }
  .badge.ok { color:#15876f; background:#edfbf5; border-color:#c9ecdf; }.badge.ok i{background:#1db58c;box-shadow:0 0 0 3px rgba(29,181,140,.12)}
  .badge.accent { color:#087e78; background:#e9fbf9; border-color:#bfe8e3; }.badge.accent i{background:var(--teal)}
  .badge.danger { color:#b44b4b; background:#fff2f2; border-color:#f4d2d2; }.badge.danger i{background:#d95d5d}
  .tabs { min-height:46px; display:flex; align-items:end; padding:0 20px; background:#fff; border-bottom:1px solid #e2e9ef; }
  .tab { height:46px; padding:0 15px; border:0; border-bottom:2px solid transparent; background:transparent; color:#768592; font-weight:700; }
  .tab.active { color:#087f78; border-color:var(--teal); }
  .revision { margin:0 0 12px auto; color:#6995bd; font-size:10px; background:#eef7ff; border-radius:999px; padding:3px 8px; }
  .content { min-height:0; overflow:auto; padding:16px 18px 18px; scrollbar-width:thin; scrollbar-color:#b8c8d3 transparent; }
  .point-panel,.jog-panel,.monitor { background:#fff; border:1px solid #e1e8ee; border-radius:11px; box-shadow:0 7px 22px rgba(39,65,87,.055); }
  .section-heading { height:58px; display:flex; align-items:center; justify-content:space-between; padding:0 15px; border-bottom:1px solid #edf1f4; }
  h2,h3 { margin:0; } h2 { font-size:14px; margin-top:2px; } h3 { font-size:12px; }
  .count { color:#2989d8; background:#edf7ff; padding:4px 8px; border-radius:999px; font-size:10px; }
  .point-list { max-height:275px; overflow:auto; padding:9px; background:#f8fafb; }
  .point-row { width:100%; display:grid; grid-template-columns:32px minmax(0,1fr) auto; align-items:center; gap:10px; margin-bottom:6px; padding:9px 11px; text-align:left; border:1px solid #e2e9ee; border-radius:8px; background:#fff; color:inherit; box-shadow:0 2px 7px rgba(54,75,91,.035); }
  .point-row:hover { border-color:#a7cee9; }.point-row.selected { border-color:#48a7ed; background:linear-gradient(90deg,#eff8ff,#f9fcff); box-shadow:inset 3px 0 #1687ee; }
  .point-index { display:grid; place-items:center; width:27px; height:27px; border-radius:50%; color:#84919b; background:#f0f3f5; font-size:11px; }.selected .point-index{color:#fff;background:#1687ee}
  .point-copy { min-width:0; }.point-copy strong,.point-copy small{display:block}.point-copy small{overflow:hidden;color:#95a2ac;font-size:9px;text-overflow:ellipsis;white-space:nowrap}.point-meta{color:#21a37e;font-size:9px;background:#ebfaf5;padding:3px 6px;border-radius:5px}
  .monitor { margin-top:12px; padding:13px 14px 15px; }.monitor-head,.joint-heading{display:flex;justify-content:space-between;color:#71818e;font-size:10px}.monitor h3{margin:10px 0 7px}.monitor h3 small{color:#9aa7b0;font-weight:500;margin-left:6px}.target-rail{margin-top:9px;padding:7px 9px;border:1px solid #d9e8f2;border-radius:7px;background:#f3f9fd;color:#4f7692;font-size:10px;font-weight:700}
  .pose-grid { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:7px; }.pose-value{position:relative;padding:8px 9px;border:1px solid #e2e8ed;border-radius:7px;background:#fbfcfd}.pose-value small{display:block;color:#8e9ba5;font-size:9px}.pose-value strong{font-size:13px}.pose-value span{margin-left:4px;color:#9ca7af;font-size:9px}
  .joint-heading { align-items:center;margin-top:12px;padding-top:10px;border-top:1px solid #edf1f4}.joint-bars{display:grid;gap:5px;margin-top:8px}.joint-bar{display:grid;grid-template-columns:22px minmax(0,1fr) 58px;align-items:center;gap:7px;font-size:9px;color:#75838f}.joint-bar i{height:5px;overflow:hidden;background:#e8eef2;border-radius:99px}.joint-bar b{display:block;height:100%;background:linear-gradient(90deg,#178eea,#28b3d1);border-radius:99px}.joint-bar strong{text-align:right;font-weight:600;color:#5c6d79}
  .jog-settings { display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:12px 14px;border-bottom:1px solid #edf1f4}.jog-settings label{color:#6f7f8b;font-size:10px}.jog-settings input,.jog-settings select{width:72px;margin-left:5px;padding:5px;border:1px solid #d8e2e8;border-radius:6px;background:#fff}
  .rail-control{display:grid;grid-template-columns:minmax(0,1fr) auto auto;align-items:center;gap:10px;margin:12px 12px 0;padding:10px 12px;border:1px solid #cfe3ee;border-radius:8px;background:#f5fafc}.rail-control>div{display:flex;align-items:center;gap:10px}.rail-control strong{font-size:11px}.rail-control small{color:#8a99a5;font-size:9px}.rail-control label{color:#6f7f8b;font-size:10px}.rail-control input{width:90px;margin-right:4px;padding:5px;border:1px solid #cfdde6;border-radius:6px;background:#fff}
  .jog-columns{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:12px}.jog-group{padding:10px;border:1px solid #e3eaef;border-radius:8px}.jog-group h3{margin-bottom:8px}.jog-row{display:grid;grid-template-columns:28px minmax(0,1fr) 29px 29px;gap:5px;align-items:center;margin:5px 0}.jog-row small{color:#8e9aa4}.jog-row button{width:29px;height:27px;border:1px solid #cfdce4;border-radius:6px;background:#f6fafc;color:#267baa;font-weight:800}.jog-row button:hover{border-color:#49a9da;background:#eaf7fd}
  .teach-card{display:flex;align-items:flex-start;justify-content:space-between;gap:15px;margin:0 12px 12px;padding:12px;border:1px solid #cde7e2;border-radius:9px;background:#f1fbf9}.teach-card p{margin:3px 0 0;color:#758b89;font-size:9px}.vision-toggle{display:flex;align-items:center;gap:6px;margin-top:8px;color:#4d6d69;font-size:10px}.marker-select{display:block;margin-top:8px;color:#4d6d69;font-size:10px}.marker-select select{margin-left:6px;min-width:220px;padding:4px 6px;border:1px solid #bfd8d3;border-radius:6px;background:#fff}.teach{color:#087d74!important;border-color:#9dd8cf!important;background:#fff!important}
  .vision-panel{background:#fff;border:1px solid #e1e8ee;border-radius:11px;box-shadow:0 7px 22px rgba(39,65,87,.055);padding:0 0 12px}.vision-summary{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;padding:12px 14px;border-bottom:1px solid #edf1f4}.vision-summary div{padding:8px 9px;border:1px solid #e2e8ed;border-radius:7px;background:#fbfcfd}.vision-summary small{display:block;color:#8e9ba5;font-size:9px}.vision-summary strong{font-size:11px}.vision-actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 14px;border-bottom:1px solid #edf1f4}.marker-list{padding:10px 14px 4px}.marker-row{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:4px 10px;padding:8px 0;border-bottom:1px solid #edf1f4}.marker-row strong{font-size:11px}.marker-row small{color:#8e9aa4}.marker-row span{color:#21a37e;font-size:9px;white-space:nowrap}
  .command-bar{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:11px 18px;background:#fff;border-top:1px solid #e1e9ee}.exclusive-controls,.command-actions{display:flex;gap:7px}.button{min-height:32px;padding:0 12px;border-radius:7px;border:1px solid #d3dee5;background:#fff;color:#536674;font-weight:700;font-size:11px}.button.compact{min-height:27px;padding:0 9px;font-size:10px}.button.primary{min-width:128px;color:#fff;border-color:#008e84;background:linear-gradient(135deg,#00aa9d,#008b83);box-shadow:0 5px 13px rgba(0,150,139,.22)}.button.ghost:hover{border-color:#99bbc9;background:#f5fafb}
  .message{min-height:28px;margin:0;padding:7px 18px;color:#58807d;background:#eff9f7;border-top:1px solid #dcefeb;font-size:10px}.message.error{color:#af4949;background:#fff1f1;border-color:#f2d1d1}.spinner{display:inline-block;width:10px;height:10px;margin-right:6px;border:2px solid rgba(255,255,255,.4);border-top-color:#fff;border-radius:50%;animation:spin .8s linear infinite;vertical-align:-1px}
  .empty{padding:28px;text-align:center;color:#97a5af}.compact-empty{padding:12px}@keyframes spin{to{transform:rotate(360deg)}}
  @media(max-width:760px){.hero{align-items:flex-start}.hero-actions{max-width:230px}.jog-columns{grid-template-columns:1fr}.jog-settings{grid-template-columns:1fr 1fr}.command-bar{align-items:stretch;flex-direction:column}.exclusive-controls,.command-actions{justify-content:flex-end}.content{padding:12px}.robot-card{min-height:600px}}
`
