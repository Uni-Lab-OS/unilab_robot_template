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
import type { PreviewCalibrationInfo } from './previewModel'
import {
  canCallAction,
  canReadPointCatalog,
  canShowJogTab,
  canShowVisionTab,
  usesMoveItExclusiveJog
} from './runtimeMode'
import { cardStyles } from './styles'

export interface RobotCardViewState {
  actionBusy: boolean
  allowedActions?: string[]
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
  const showRefresh = canReadPointCatalog(state.allowedActions)
  const showJogTab = canShowJogTab(state.allowedActions)
  const showVisionTab = canShowVisionTab(state.allowedActions)
  const showCalibrationTab = snapshot?.calibration != null
  const moveItExclusive = usesMoveItExclusiveJog(state.allowedActions)
  const activeTab = state.tab === 'jog' && !showJogTab
    ? 'points'
    : state.tab === 'vision' && !showVisionTab
      ? 'points'
      : state.tab === 'calibration' && !showCalibrationTab
        ? 'points'
        : state.tab
  return `
    <style>${cardStyles}</style>
    <article class="robot-card" data-card-ready="${snapshot ? 'true' : 'false'}" data-runtime-mode="${moveItExclusive ? 'commissioning' : 'preview'}">
      <header class="hero">
        <div>
          <p class="eyebrow">${cardBranding.eyebrow}</p>
          <h1>${cardBranding.title}</h1>
          <p class="subtitle">${cardBranding.subtitle}</p>
        </div>
        <div class="hero-actions">
          ${badge(state.hostOnline ? 'OS 在线' : 'OS 离线', state.hostOnline ? 'ok' : 'danger')}
          ${badge(exclusiveLabel(state.exclusiveState), state.exclusiveState === 'exclusive' ? 'accent' : 'muted')}
          ${showRefresh ? '<button class="button ghost compact" data-refresh type="button">刷新</button>' : ''}
        </div>
      </header>

      <nav class="tabs" aria-label="机械臂调试功能">
        ${tabButton('points', '点位与姿态', activeTab)}
        ${showCalibrationTab ? tabButton('calibration', '校准点', activeTab) : ''}
        ${showJogTab ? tabButton('jog', 'Jog 与示教', activeTab) : ''}
        ${showVisionTab ? tabButton('vision', '视觉校准', activeTab) : ''}
        <span class="revision">PointSet · ${escapeHtml(snapshot?.pointSetRevision ?? '读取中')}</span>
      </nav>

      <main class="content">
        ${activeTab === 'points'
          ? renderPointPanel(snapshot, selected, state.message, state.hostOnline)
          : activeTab === 'calibration'
            ? renderCalibrationPanel(snapshot, selected)
            : activeTab === 'jog'
              ? renderJogPanel(state, snapshot, selected)
              : renderVisionPanel(state, snapshot, selected)}
      </main>

      <footer class="command-bar">
        ${requiresExclusiveJog(state) ? `
        <div class="exclusive-controls">
          <button class="button ghost" data-exclusive="acquire" type="button"
            ${state.exclusiveState === 'exclusive' ? 'disabled' : ''}>取得调试控制</button>
          <button class="button ghost" data-exclusive="release" type="button"
            ${state.exclusiveState !== 'exclusive' ? 'disabled' : ''}>释放</button>
        </div>` : ''}
        <div class="command-actions">
          ${canCallAction(state.allowedActions, 'home') ? `
            <button class="button ghost" data-home type="button"
              ${motionDisabled(state) ? 'disabled' : ''}>回原点</button>` : ''}
          ${canCallAction(state.allowedActions, 'move_to_anchor') ? `
            <button class="button primary" data-move-to-point type="button"
              ${motionDisabled(state) || !selected ? 'disabled' : ''}>
              ${state.moving ? '<span class="spinner"></span>规划执行中' : `移动到 ${escapeHtml(selected?.sourcePoint ?? '目标点')}`}
            </button>` : ''}
        </div>
      </footer>
      <p class="message ${messageTone(state.message)}" aria-live="polite">${escapeHtml(state.message || readyMessage(state))}</p>
    </article>
  `
}

function renderPointPanel(
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null,
  message = '',
  hostOnline = true
): string {
  const emptyLabel = snapshot
    ? ''
    : (!hostOnline
      ? '设备离线，请启动 OS/Edge 后点击「刷新」。'
      : (message.trim() || '正在读取活动 PointSet…'))
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
          </button>`).join('') || `<div class="empty">${escapeHtml(emptyLabel)}</div>`}
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
  const showRailMove = canCallAction(state.allowedActions, 'move_rail_to_position')
  const showJointJog = canCallAction(state.allowedActions, 'jog_joint_once')
  const showPreviewJog = canCallAction(state.allowedActions, 'set_joint')
  const showTcpJog = canCallAction(state.allowedActions, 'jog_tcp_once')
  const showRecord = canCallAction(state.allowedActions, 'record_current_point')
    || canCallAction(state.allowedActions, 'teach_point_from_current')
  const hasRail = showRailMove && snapshot?.capabilities.railMove === true && snapshot.rail !== null
  const railMinimum = snapshot?.rail?.travelMinMm
  const railMaximum = snapshot?.rail?.travelMaxMm
  return `
    <section class="jog-panel">
      <div class="section-heading">
        <div><span class="kicker">一步一命令</span><h2>Jog</h2></div>
        ${badge(snapshot?.idle ? '执行器空闲' : '执行器忙碌', snapshot?.idle ? 'ok' : 'muted')}
      </div>
      <div class="jog-settings">
        <label>关节步长 <input data-joint-step type="number" min="0.1" step="0.1" value="${escapeHtml(formatStep(state.jointStep, 1))}"> °</label>
        ${showTcpJog ? `
        <label>TCP 步长 <input data-tcp-step type="number" min="0.1" step="0.1" value="${escapeHtml(formatStep(state.tcpStep, 2))}"> mm / °</label>
        <label>坐标系 <select data-tcp-frame>
          <option value="arm_base" ${state.tcpFrame === 'tool' ? '' : 'selected'}>基座</option>
          <option value="tool" ${state.tcpFrame === 'tool' ? 'selected' : ''}>工具</option>
        </select></label>` : ''}
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
        ${showJointJog ? `
        <div class="jog-group">
          <h3>关节 Jog</h3>
          ${(snapshot?.jointPositions ?? []).map((joint, index) => jogRow(
            `J${index + 1}`,
            joint.jointRef,
            formatNumber(joint.positionDeg) + '°',
            'joint'
          )).join('') || '<div class="empty">等待关节目录…</div>'}
        </div>` : ''}
        ${showPreviewJog && !showJointJog ? `
        <div class="jog-group">
          <h3>Preview 关节 Jog</h3>
          ${(snapshot?.jointPositions ?? []).map((joint, index) => previewJogRow(
            index + 1,
            joint.jointRef,
            formatNumber(joint.positionDeg) + '°'
          )).join('') || '<div class="empty">等待关节目录…</div>'}
        </div>` : ''}
        ${showTcpJog ? `
        <div class="jog-group">
          <h3>TCP Jog</h3>
          ${tcpJogRows(snapshot?.tcpPose).map((row) => jogRow(
            row.label, row.axis, row.value, 'tcp'
          )).join('')}
        </div>` : ''}
      </div>
      ${showRecord ? `
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
          ${!selected?.editable || snapshot?.capabilities.pointRecord !== true || motionDisabled(state) ? 'disabled' : ''}>记录当前机械臂${hasRail ? '与导轨' : ''}位置</button>
      </div>` : ''}
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
        ${canCallAction(state.allowedActions, 'calibrate_camera_extrinsic') ? `
          <button class="button primary" data-calibrate-camera type="button"
            ${motionDisabled(state) ? 'disabled' : ''}>摄像头外参标定</button>` : ''}
        ${canCallAction(state.allowedActions, 'calibrate_tcp') ? `
          <button class="button primary" data-calibrate-tcp type="button"
            ${motionDisabled(state) ? 'disabled' : ''}>TCP 校准</button>` : ''}
        ${canCallAction(state.allowedActions, 'record_marker') ? `
          <button class="button teach" data-record-marker type="button"
            ${motionDisabled(state) || !selected?.groupRef ? 'disabled' : ''}>marker 记录</button>` : ''}
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

function renderCalibrationPanel(
  snapshot: RobotDebugSnapshot | null,
  selected: PointTarget | null
): string {
  const calibration = snapshot?.calibration
  return `
    <section class="vision-panel">
      <div class="section-heading">
        <div><span class="kicker">CAD 点位包</span><h2>校准点</h2></div>
        <span class="count">${escapeHtml(calibration?.version ?? '未读取')}</span>
      </div>
      ${calibration ? `
        <div class="vision-summary">
          <div><small>版本</small><strong>${escapeHtml(calibration.version)}</strong></div>
          <div><small>状态</small><strong>${escapeHtml(calibration.status)}</strong></div>
          <div><small>人工确认</small><strong>${calibration.humanConfirmed ? '已确认' : '待确认'}</strong></div>
          <div><small>点位数量</small><strong>${snapshot?.pointTargets.length ?? 0}</strong></div>
        </div>
        ${calibration.qualification ? `<p class="calibration-note">${escapeHtml(calibration.qualification)}</p>` : ''}
      ` : '<div class="empty">点击右上角「刷新」读取校准点与点位包状态…</div>'}
      ${selected ? `<div class="calibration-meta"><small>当前点</small><strong>${escapeHtml(selected.sourcePoint)}</strong></div>` : ''}
    </section>
  `
}

function previewJogRow(jointIndex: number, jointRef: string, value: string): string {
  return `<div class="jog-row"><strong>J${jointIndex}</strong><small>${escapeHtml(value)}</small>
    <button data-preview-jog-joint="${jointIndex}" data-direction="negative" type="button" aria-label="J${jointIndex} 负向点动">−</button>
    <button data-preview-jog-joint="${jointIndex}" data-direction="positive" type="button" aria-label="J${jointIndex} 正向点动">＋</button></div>`
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

function requiresExclusiveJog(state: RobotCardViewState): boolean {
  return usesMoveItExclusiveJog(state.allowedActions)
}

function motionDisabled(state: RobotCardViewState): boolean {
  const exclusiveBlocked = requiresExclusiveJog(state) && state.exclusiveState !== 'exclusive'
  return state.actionBusy || state.moving || exclusiveBlocked || !state.snapshot?.idle
}

function readyMessage(state: RobotCardViewState): string {
  if (!state.hostOnline) return 'OS 设备离线，调试动作不可用。'
  if (!state.snapshot) {
    return requiresExclusiveJog(state)
      ? '正在读取 MoveIt 调试快照…'
      : '正在读取 PointSet 目录与关节状态…'
  }
  if (state.snapshot.stale) {
    return state.snapshot.source === 'point_set'
      ? 'PointSet 目录已就绪；MoveIt 调试通道暂不可用，仍可执行锚点移动。'
      : 'MoveIt 快照已过期，请刷新后再操作。'
  }
  if (requiresExclusiveJog(state) && state.exclusiveState !== 'exclusive') {
    return '取得调试控制后可以执行 Jog、点位设置和目标移动。'
  }
  if (requiresExclusiveJog(state)) return 'MoveIt 调试通道已就绪。'
  return 'Preview 执行器已就绪，可直接 Jog 与示教落盘。'
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
