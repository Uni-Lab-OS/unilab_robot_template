import { cardBranding } from './branding'
import type {
  PreviewCalibrationInfo,
  PreviewSiteTarget,
  PreviewSnapshot,
  PreviewTab
} from './previewModel'
import { canCallAction } from './runtimeMode'
import { cardStyles } from './styles'

export interface PreviewCardViewState {
  actionBusy: boolean
  allowedActions?: string[]
  hostOnline: boolean
  jointStep: number
  jointTargets: number[]
  message: string
  moving: boolean
  selectedSiteRef: string
  snapshot: PreviewSnapshot | null
  tab: PreviewTab
}

export function renderPreviewCard(state: PreviewCardViewState): string {
  const snapshot = state.snapshot
  const selected = snapshot?.sites.find((site) => site.siteRef === state.selectedSiteRef)
    ?? snapshot?.sites[0]
    ?? null
  const joints = previewJointRows(state, snapshot)
  const showJogTab = canCallAction(state.allowedActions, 'set_joint')
    || canCallAction(state.allowedActions, 'moveJ')
  const activeTab = state.tab === 'jog' && !showJogTab
    ? 'points'
    : state.tab === 'calibration'
      ? 'calibration'
      : state.tab
  return `
    <style>${cardStyles}</style>
    <article class="robot-card" data-card-ready="${snapshot ? 'true' : 'false'}" data-runtime-mode="preview">
      <header class="hero">
        <div>
          <p class="eyebrow">${cardBranding.eyebrow}</p>
          <h1>${cardBranding.title}</h1>
          <p class="subtitle">${cardBranding.subtitle}</p>
        </div>
        <div class="hero-actions">
          ${badge(state.hostOnline ? 'OS 在线' : 'OS 离线', state.hostOnline ? 'ok' : 'danger')}
          ${badge('Preview 模式', 'accent')}
          ${canCallAction(state.allowedActions, 'read_preview_snapshot')
            ? '<button class="button ghost compact" data-preview-refresh type="button">刷新</button>'
            : ''}
        </div>
      </header>

      <nav class="tabs" aria-label="Preview 机械臂调试">
        ${tabButton('points', '点位与姿态', activeTab)}
        ${tabButton('calibration', '校准点', activeTab)}
        ${showJogTab ? tabButton('jog', 'Jog 与示教', activeTab) : ''}
        <span class="revision">PointSet · ${escapeHtml(snapshot?.pointSetRevision ?? '点击刷新')}</span>
      </nav>

      <main class="content">
        ${activeTab === 'points'
          ? renderPointsPanel(snapshot, selected)
          : activeTab === 'calibration'
            ? renderCalibrationPanel(snapshot, selected)
            : renderJogPanel(state, snapshot, selected, joints)}
      </main>

      <footer class="command-bar">
        <div class="command-actions">
          ${canCallAction(state.allowedActions, 'home') ? `
            <button class="button ghost" data-preview-home type="button"
              ${previewMotionDisabled(state) ? 'disabled' : ''}>回原点</button>` : ''}
          ${canCallAction(state.allowedActions, 'stop') ? `
            <button class="button danger" data-preview-stop type="button"
              ${state.actionBusy ? 'disabled' : ''}>停止</button>` : ''}
          ${canCallAction(state.allowedActions, 'pick') && selected ? `
            <button class="button ghost" data-preview-pick type="button"
              ${previewMotionDisabled(state) ? 'disabled' : ''}>Pick</button>` : ''}
          ${canCallAction(state.allowedActions, 'place') && selected ? `
            <button class="button ghost" data-preview-place type="button"
              ${previewMotionDisabled(state) ? 'disabled' : ''}>Place</button>` : ''}
          ${canCallAction(state.allowedActions, 'moveJ') && activeTab === 'jog' ? `
            <button class="button primary" data-preview-movej type="button"
              ${previewMotionDisabled(state) ? 'disabled' : ''}>
              ${state.moving ? '<span class="spinner"></span>执行中' : 'MoveJ'}
            </button>` : ''}
        </div>
      </footer>
      <p class="message ${messageTone(state.message)}" aria-live="polite">${escapeHtml(state.message || readyMessage(state))}</p>
    </article>
  `
}

function renderCalibrationPanel(
  snapshot: PreviewSnapshot | null,
  selected: PreviewSiteTarget | null
): string {
  const calibration = snapshot?.calibration
  return `
    <section class="vision-panel">
      <div class="section-heading">
        <div><span class="kicker">CAD 点位包</span><h2>校准点</h2></div>
        <span class="count">${escapeHtml(calibration?.version ?? '未读取')}</span>
      </div>
      ${calibration
        ? renderCalibrationSummary(calibration, snapshot)
        : '<div class="empty">点击右上角「刷新」读取 CAD 校准点与点位包状态…</div>'}
      ${selected && snapshot ? renderCalibrationSiteDetail(selected) : ''}
    </section>
  `
}

function renderCalibrationSummary(
  calibration: PreviewCalibrationInfo,
  snapshot: PreviewSnapshot
): string {
  const confirmedLabel = calibration.humanConfirmed ? '已人工确认' : '待人工确认'
  const confirmedTone = calibration.humanConfirmed ? 'ok' : 'muted'
  return `
    <div class="vision-summary">
      <div><small>版本</small><strong>${escapeHtml(calibration.version)}</strong></div>
      <div><small>状态</small><strong>${escapeHtml(calibration.status)}</strong></div>
      <div><small>人工确认</small><strong>${escapeHtml(confirmedLabel)}</strong></div>
      <div><small>库位数量</small><strong>${snapshot.sites.length}</strong></div>
    </div>
    <div class="calibration-meta">
      <div><small>digest</small><strong>${escapeHtml(shortDigest(calibration.digest))}</strong></div>
      <div><small>PointSet revision</small><strong>${escapeHtml(snapshot.pointSetRevision)}</strong></div>
    </div>
    ${calibration.qualification ? `
      <p class="calibration-note">${escapeHtml(calibration.qualification)}</p>` : ''}
    ${calibration.reviewAssets ? `
      <div class="calibration-assets">
        <span class="kicker">确认图</span>
        ${calibration.reviewAssets.slotReviewSvg
          ? `<div><small>槽位图</small><strong>${escapeHtml(calibration.reviewAssets.slotReviewSvg)}</strong></div>`
          : ''}
        ${calibration.reviewAssets.wellReviewSvg
          ? `<div><small>孔位图</small><strong>${escapeHtml(calibration.reviewAssets.wellReviewSvg)}</strong></div>`
          : ''}
      </div>` : ''}
    <div class="marker-list">
      <div class="joint-heading"><h3>校准点目录</h3><small>${snapshot.sites.length} 项</small></div>
      ${snapshot.sites.map((site, index) => `
        <div class="marker-row">
          <strong>${index + 1}. ${escapeHtml(site.label)}</strong>
          <small>${escapeHtml(site.groupRef)} · ${escapeHtml(site.siteRef)}</small>
          <span>${formatCalibrationXyz(site.xyzM)}</span>
        </div>`).join('') || '<div class="empty compact-empty">点位包为空。</div>'}
    </div>
    <p class="calibration-readonly">${badge(confirmedLabel, confirmedTone)} Preview 模式仅展示 CAD 校准点，不含实机视觉标定动作。</p>
  `
}

function renderCalibrationSiteDetail(selected: PreviewSiteTarget): string {
  return `
    <section class="monitor">
      <div class="monitor-head">
        <span>选中校准点 · 世界坐标</span>
        <span>目标：<strong>${escapeHtml(selected.siteRef)}</strong></span>
      </div>
      <div class="pose-grid">
        ${[
          ['X', selected.xyzM[0], 'm'], ['Y', selected.xyzM[1], 'm'], ['Z', selected.xyzM[2], 'm']
        ].map(([label, value, unit]) => `
          <div class="pose-value"><small>${label}</small><strong>${formatNumber(value)}</strong><span>${unit}</span></div>
        `).join('')}
      </div>
    </section>
  `
}

function renderPointsPanel(
  snapshot: PreviewSnapshot | null,
  selected: PreviewSiteTarget | null
): string {
  return `
    <section class="point-panel">
      <div class="section-heading">
        <div><span class="kicker">库位目录</span><h2>点位包目标</h2></div>
        <span class="count">${snapshot?.sites.length ?? 0} 点</span>
      </div>
      <div class="point-list" role="listbox" aria-label="库位点位">
        ${(snapshot?.sites ?? []).map((site, index) => `
          <button class="point-row ${site.siteRef === selected?.siteRef ? 'selected' : ''}"
            data-preview-site="${escapeHtml(site.siteRef)}" type="button" role="option"
            aria-selected="${site.siteRef === selected?.siteRef}">
            <span class="point-index">${index + 1}</span>
            <span class="point-copy">
              <strong>${escapeHtml(site.label)}</strong>
              <small>${escapeHtml(site.groupRef)} · ${escapeHtml(site.siteRef)}</small>
            </span>
            <span class="point-meta">库位</span>
          </button>`).join('') || '<div class="empty">点击右上角「刷新」读取点位包…</div>'}
      </div>
    </section>
    ${renderSiteMonitor(snapshot, selected)}
  `
}

function renderSiteMonitor(
  snapshot: PreviewSnapshot | null,
  selected: PreviewSiteTarget | null
): string {
  const joints = snapshot?.jointPositions ?? []
  const xyz = selected?.xyzM ?? []
  return `
    <section class="monitor">
      <div class="monitor-head">
        <span>选中库位 · 世界坐标</span>
        <span>目标：<strong>${escapeHtml(selected?.siteRef ?? '—')}</strong></span>
      </div>
      <div class="pose-grid">
        ${[
          ['X', xyz[0], 'm'], ['Y', xyz[1], 'm'], ['Z', xyz[2], 'm']
        ].map(([label, value, unit]) => `
          <div class="pose-value"><small>${label}</small><strong>${formatNumber(value)}</strong><span>${unit}</span></div>
        `).join('')}
      </div>
      ${snapshot?.railPositionMm !== null && snapshot?.railPositionMm !== undefined ? `
        <div class="target-rail">导轨 ${formatNumber(snapshot.railPositionMm)} mm</div>` : ''}
      <div class="joint-heading"><h3>关节位置</h3><small>角度</small></div>
      <div class="joint-bars">
        ${joints.map((joint, index) => `
          <div class="joint-bar">
            <span>J${index + 1}</span>
            <i><b style="width:${jointPercent(joint.positionDeg)}%"></b></i>
            <strong>${formatNumber(joint.positionDeg)}°</strong>
          </div>
        `).join('') || '<div class="empty compact-empty">等待关节状态…</div>'}
      </div>
    </section>
  `
}

function renderJogPanel(
  state: PreviewCardViewState,
  snapshot: PreviewSnapshot | null,
  selected: PreviewSiteTarget | null,
  joints: Array<{ index: number; positionDeg: number }>
): string {
  const showJointJog = canCallAction(state.allowedActions, 'set_joint')
    && snapshot?.capabilities.jointJog !== false
  return `
    <section class="jog-panel">
      <div class="section-heading">
        <div><span class="kicker">一步一命令</span><h2>Jog</h2></div>
        ${badge(snapshot?.idle ? 'Preview 空闲' : 'Preview 忙碌', snapshot?.idle ? 'ok' : 'muted')}
      </div>
      <div class="jog-settings">
        <label>关节步长 <input data-joint-step type="number" min="0.1" step="0.1" value="${escapeHtml(formatStep(state.jointStep, 1))}"> °</label>
      </div>
      ${showJointJog ? `
      <div class="jog-columns">
        <div class="jog-group">
          <h3>关节 Jog</h3>
          ${joints.map((joint) => jogRow(
            `J${joint.index}`,
            String(joint.index),
            formatNumber(joint.positionDeg) + '°'
          )).join('') || '<div class="empty">等待关节状态…</div>'}
        </div>
      </div>` : ''}
      ${canCallAction(state.allowedActions, 'moveJ') ? `
      <div class="preview-joints">
        ${state.jointTargets.map((value, index) => `
          <label>J${index + 1} 目标
            <input data-preview-joint="${index + 1}" type="number" step="0.1"
              value="${formatNumber(value)}">
          </label>`).join('')}
      </div>` : ''}
    </section>
    ${renderSiteMonitor(snapshot, selected)}
  `
}

function previewJointRows(
  state: PreviewCardViewState,
  snapshot: PreviewSnapshot | null
): Array<{ index: number; positionDeg: number }> {
  if (snapshot?.jointPositions.length) {
    return snapshot.jointPositions.map((joint, index) => ({
      index: index + 1,
      positionDeg: joint.positionDeg
    }))
  }
  return state.jointTargets.map((positionDeg, index) => ({
    index: index + 1,
    positionDeg
  }))
}

function jogRow(label: string, jointIndex: string, value: string): string {
  return `<div class="jog-row"><strong>${label}</strong><small>${escapeHtml(value)}</small>
    <button data-preview-jog-joint="${escapeHtml(jointIndex)}" data-direction="negative" type="button" aria-label="${label} 负向点动">−</button>
    <button data-preview-jog-joint="${escapeHtml(jointIndex)}" data-direction="positive" type="button" aria-label="${label} 正向点动">＋</button></div>`
}

function tabButton(value: PreviewTab, label: string, current: PreviewTab): string {
  return `<button class="tab ${value === current ? 'active' : ''}" data-preview-tab="${value}" type="button" aria-selected="${value === current}">${label}</button>`
}

function previewMotionDisabled(state: PreviewCardViewState): boolean {
  return state.actionBusy || state.moving || !state.hostOnline
}

function readyMessage(state: PreviewCardViewState): string {
  if (!state.hostOnline) return 'OS 设备离线，Preview 动作不可用。'
  if (!state.snapshot) return '点击「刷新」加载点位包与关节状态；Jog 使用 set_joint。'
  if (state.actionBusy) return '设备动作进行中…'
  return `已加载 ${state.snapshot.sites.length} 个库位；Jog 与 MoveJ 已就绪。`
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

function formatStep(value: number, fallback: number): string {
  return Number.isFinite(value) && value > 0 ? String(value) : String(fallback)
}

function formatCalibrationXyz(xyzM: number[]): string {
  if (xyzM.length < 3) return '—'
  return `${formatNumber(xyzM[0])}, ${formatNumber(xyzM[1])}, ${formatNumber(xyzM[2])} m`
}

function shortDigest(value: string): string {
  if (!value) return '—'
  return value.length <= 16 ? value : `${value.slice(0, 12)}…`
}

function badge(label: string, tone: 'ok' | 'accent' | 'muted' | 'danger'): string {
  return `<span class="badge ${tone}"><i></i>${escapeHtml(label)}</span>`
}

function escapeHtml(value: string): string {
  return value.replace(/[&<>"']/gu, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  })[character] ?? character)
}
