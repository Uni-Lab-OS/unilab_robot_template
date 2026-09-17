import { getDeviceCardBridge } from '@unilab/device-card-sdk'

import {
  asRecord,
  defaultMarkerRefForWarehouse,
  mergeJointState,
  mockRobotDebugSnapshot,
  type CardTab,
  type ExclusiveState,
  type RobotDebugSnapshot
} from './model'
import { parsePointCatalogSnapshot } from './cardView'
import {
  canCallAction,
  readAllowedActions,
  readAllowedState,
  resolveCatalogReadAction,
  usesMoveItExclusiveJog
} from './runtimeMode'
import { renderRobotCard } from './view'

const PREVIEW_HOME_JOINTS = [90, -65, -70, -135, 90, 0]

/** 导轨 + 机械臂调试设备卡片（Device Card）Web Component。 */
export default class RailMountedArmCardElement extends HTMLElement {
  private readonly root = this.attachShadow({ mode: 'open' })
  private unsubscribe: (() => void) | null = null
  private state: Record<string, unknown> = {}
  private snapshot: RobotDebugSnapshot | null = null
  private exclusiveState: ExclusiveState = 'idle'
  private message = ''
  private selectedTargetRef = ''
  private tab: CardTab = 'points'
  private includeVisionOnRecord = false
  private selectedMarkerRef = ''
  private actionPending = false
  private alignSelectedPoint = true
  private jointStep = 1
  private tcpStep = 2
  private tcpFrame: 'arm_base' | 'tool' = 'arm_base'
  private allowedActions: string[] | undefined
  private allowedState: string[] = []
  private previewJointTargets = [...PREVIEW_HOME_JOINTS]

  /** 连接 Host Bridge；live 模式只订阅状态，不隐式派发设备动作。 */
  async connectedCallback(): Promise<void> {
    const bridge = getDeviceCardBridge()
    this.render()
    try {
      const context = await bridge.getContext()
      this.state = context.state
      this.allowedActions = readAllowedActions(context.config)
      this.allowedState = readAllowedState(context.config)
      if (context.mode === 'mock') {
        this.acceptSnapshot(mockRobotDebugSnapshot())
      } else if (resolveCatalogReadAction(this.allowedActions)) {
        this.message = '点击「刷新」加载 PointSet 目录与关节状态。'
      } else {
        this.message = '机械臂卡片已就绪；当前未授权目录读取动作。'
      }
      this.render()
      if (usesMoveItExclusiveJog(this.allowedActions)) {
        try {
          this.exclusiveState = (await bridge.readManualExclusive()).state
        } catch (error) {
          this.message = errorMessage(error, '手动独占状态不可用。')
        }
      }
      const stateKeys = this.subscriptionKeys()
      this.unsubscribe = bridge.subscribeState(stateKeys, (state) => {
        this.state = state
        this.snapshot = mergeJointState(this.snapshot, state)
        this.render()
      })
      this.render()
    } catch (error) {
      this.message = `卡片宿主连接失败：${errorMessage(error, '未知错误')}`
      this.render()
    }
  }

  /** 断开时释放状态订阅。 */
  disconnectedCallback(): void {
    this.unsubscribe?.()
    this.unsubscribe = null
  }

  private subscriptionKeys(): string[] {
    if (this.allowedState.length > 0) return [...this.allowedState]
    return ['actionBusy', 'jointState', 'moveit_online', 'online']
  }

  /** 渲染当前完整状态并绑定只通过 Host Bridge 的交互。 */
  private render(): void {
    this.captureJogSettings()
    const scrollTop = this.root.querySelector<HTMLElement>('.content')?.scrollTop ?? 0
    const pointScrollTop = this.root.querySelector<HTMLElement>('.point-list')?.scrollTop ?? 0
    this.root.innerHTML = renderRobotCard({
      actionBusy: Object.values(asRecord(this.state.actionBusy)).some(Boolean),
      allowedActions: this.allowedActions,
      exclusiveState: this.exclusiveState,
      hostOnline: this.state.online === true,
      includeVisionOnRecord: this.includeVisionOnRecord,
      message: this.message,
      moving: this.actionPending,
      selectedMarkerRef: this.selectedMarkerRef,
      selectedTargetRef: this.selectedTargetRef,
      snapshot: this.snapshot,
      tab: this.tab,
      jointStep: this.jointStep,
      tcpStep: this.tcpStep,
      tcpFrame: this.tcpFrame
    })
    const content = this.root.querySelector<HTMLElement>('.content')
    if (content) content.scrollTop = scrollTop
    this.bindCardInteractions()
    const pointList = this.root.querySelector<HTMLElement>('.point-list')
    if (pointList) {
      if (this.alignSelectedPoint) {
        const selected = [...this.root.querySelectorAll<HTMLElement>('[data-point-target]')]
          .find((element) => element.dataset.pointTarget === this.selectedTargetRef)
        if (selected) {
          pointList.scrollTop = Math.max(
            0,
            selected.offsetTop - pointList.offsetTop
              - ((pointList.clientHeight - selected.clientHeight) / 2)
          )
        }
        this.alignSelectedPoint = false
      } else {
        pointList.scrollTop = pointScrollTop
      }
    }
  }

  /** 绑定能力驱动单壳交互。 */
  private bindCardInteractions(): void {
    this.root.querySelector('[data-refresh]')
      ?.addEventListener('click', () => void this.refreshSnapshot())
    this.root.querySelector('[data-exclusive="acquire"]')
      ?.addEventListener('click', () => void this.changeExclusive(true))
    this.root.querySelector('[data-exclusive="release"]')
      ?.addEventListener('click', () => void this.changeExclusive(false))
    this.root.querySelectorAll<HTMLElement>('[data-tab]').forEach((element) => {
      element.addEventListener('click', () => {
        const value = element.dataset.tab
        this.tab = value === 'jog' || value === 'vision' || value === 'calibration'
          ? value
          : 'points'
        this.render()
      })
    })
    this.root.querySelectorAll<HTMLElement>('[data-preview-jog-joint]').forEach((element) => {
      element.addEventListener('click', () => void this.previewJogJoint(element))
    })
    this.root.querySelectorAll<HTMLElement>('[data-point-target]').forEach((element) => {
      element.addEventListener('click', () => {
        this.selectedTargetRef = element.dataset.pointTarget ?? ''
        this.alignSelectedPoint = true
        this.syncMarkerSelection()
        this.render()
      })
    })
    this.root.querySelector('[data-move-to-point]')
      ?.addEventListener('click', () => void this.moveToSelectedPoint())
    this.root.querySelector('[data-home]')
      ?.addEventListener('click', () => {
        if (usesMoveItExclusiveJog(this.allowedActions)) {
          void this.performMotion('home', {}, '已回到 PointSet 原点。')
        } else {
          void this.performDirectAction('home', {}, '已回原点。')
        }
      })
    this.root.querySelector('[data-stop]')
      ?.addEventListener('click', () => void this.performDirectAction('stop', {}, '已发送停止。'))
    this.root.querySelectorAll<HTMLElement>('[data-jog-joint]').forEach((element) => {
      element.addEventListener('click', () => void this.jogJoint(element))
    })
    this.root.querySelectorAll<HTMLElement>('[data-jog-tcp]').forEach((element) => {
      element.addEventListener('click', () => void this.jogTcp(element))
    })
    this.root.querySelector('[data-move-rail]')
      ?.addEventListener('click', () => void this.moveRailToPosition())
    this.root.querySelector('[data-record-point]')
      ?.addEventListener('click', () => void this.recordCurrentPoint())
    this.root.querySelector('[data-include-vision]')
      ?.addEventListener('change', () => this.toggleIncludeVision())
    this.root.querySelector('[data-marker-ref]')
      ?.addEventListener('change', () => this.captureMarkerSelection())
    this.root.querySelector('[data-calibrate-camera]')
      ?.addEventListener('click', () => void this.calibrateCameraExtrinsic())
    this.root.querySelector('[data-calibrate-tcp]')
      ?.addEventListener('click', () => void this.calibrateTcp())
    this.root.querySelector('[data-record-marker]')
      ?.addEventListener('click', () => void this.recordMarker())
    this.root.querySelector('[data-joint-step]')
      ?.addEventListener('input', () => this.captureJogSettings())
    this.root.querySelector('[data-tcp-step]')
      ?.addEventListener('input', () => this.captureJogSettings())
    this.root.querySelector('[data-tcp-frame]')
      ?.addEventListener('change', () => this.captureJogSettings())
  }

  private async previewJogJoint(element: HTMLElement): Promise<void> {
    this.captureJogSettings()
    const joint = Number(element.dataset.previewJogJoint)
    const direction = element.dataset.direction === 'negative' ? -1 : 1
    if (!Number.isFinite(joint) || joint < 1 || joint > 6) return
    const current = this.snapshot?.jointPositions[joint - 1]?.positionDeg ?? 0
    const angle = current + direction * this.jointStep
    await this.performDirectAction(
      'set_joint',
      { joint, angle, duration: 1.0 },
      `J${joint} Jog 已完成。`
    )
  }

  private async performDirectAction(
    action: string,
    params: Record<string, unknown>,
    successMessage: string
  ): Promise<void> {
    if (!canCallAction(this.allowedActions, action)) {
      this.message = '动作未在卡片 manifest 中授权。'
      this.render()
      return
    }
    const bridge = getDeviceCardBridge()
    const restoreExclusive = this.exclusiveState === 'exclusive'
    try {
      this.actionPending = true
      this.message = '正在执行…'
      this.render()
      const released = await bridge.releaseManualExclusive()
      this.exclusiveState = released.state
      const run = await bridge.callAction(action, params)
      if (run.status !== 'DONE') throw new Error(run.error ?? `动作状态：${run.status}`)
      if (resolveCatalogReadAction(this.allowedActions)) {
        await this.refreshSnapshot(successMessage)
        return
      }
      this.message = successMessage
    } catch (error) {
      this.message = errorMessage(error, '机械臂动作失败。')
    } finally {
      if (restoreExclusive) {
        await this.restoreManualExclusiveAfterAction()
      }
      this.actionPending = false
      this.render()
    }
  }

  /** 把 Jog 步长和坐标系收进卡片状态，避免 innerHTML 重绘写回默认值。 */
  private captureJogSettings(): void {
    const joint = Number(this.root.querySelector<HTMLInputElement>('[data-joint-step]')?.value)
    if (Number.isFinite(joint) && joint > 0) this.jointStep = joint
    const tcp = Number(this.root.querySelector<HTMLInputElement>('[data-tcp-step]')?.value)
    if (Number.isFinite(tcp) && tcp > 0) this.tcpStep = tcp
    const frame = this.root.querySelector<HTMLSelectElement>('[data-tcp-frame]')?.value
    if (frame === 'arm_base' || frame === 'tool') this.tcpFrame = frame
  }

  /** 读取领域设备公开的只读调试快照。 */
  private async refreshSnapshot(finalMessage?: string): Promise<void> {
    const action = resolveCatalogReadAction(this.allowedActions)
    if (!action) {
      this.message = '点位目录读取未授权。'
      this.render()
      return
    }
    const bridge = getDeviceCardBridge()
    try {
      this.message = '正在读取 PointSet 目录与执行器状态…'
      this.render()
      const released = await bridge.releaseManualExclusive()
      this.exclusiveState = released.state
      const run = await bridge.callAction(action, {})
      if (run.status !== 'DONE') throw new Error(run.error ?? `动作状态：${run.status}`)
      const snapshot = parsePointCatalogSnapshot(run.result)
      if (!snapshot) throw new Error('点位目录合同无效')
      this.acceptSnapshot(snapshot)
      this.message = finalMessage
        ?? `已读取 ${snapshot.pointTargets.length} 个 PointSet 目标。`
    } catch (error) {
      this.message = errorMessage(error, '点位目录读取失败。')
    }
    this.render()
  }

  /** 接受快照并维持稳定的点位选择。 */
  private acceptSnapshot(snapshot: RobotDebugSnapshot): void {
    this.snapshot = mergeJointState(snapshot, this.state)
    if (!snapshot.pointTargets.some((point) => point.targetRef === this.selectedTargetRef)) {
      this.selectedTargetRef = snapshot.pointTargets.find(
        (point) => point.sourcePoint === 'S0722'
      )?.targetRef ?? snapshot.pointTargets[0]?.targetRef ?? ''
      this.alignSelectedPoint = true
    }
    this.syncMarkerSelection()
  }

  private syncMarkerSelection(): void {
    const selected = this.selectedPoint()
    if (!selected?.groupRef) {
      this.selectedMarkerRef = ''
      return
    }
    const binding = this.snapshot?.vision.pointBindings[selected.targetRef]
    this.selectedMarkerRef = binding || defaultMarkerRefForWarehouse(selected.groupRef)
  }

  private toggleIncludeVision(): void {
    const checked = this.root.querySelector<HTMLInputElement>('[data-include-vision]')?.checked
    this.includeVisionOnRecord = checked === true
    if (this.includeVisionOnRecord) this.syncMarkerSelection()
    this.render()
  }

  private captureMarkerSelection(): void {
    const value = this.root.querySelector<HTMLSelectElement>('[data-marker-ref]')?.value ?? ''
    if (value) this.selectedMarkerRef = value
  }

  private captureTeachOptions(): void {
    const includeVision = this.root.querySelector<HTMLInputElement>('[data-include-vision]')
    if (includeVision) this.includeVisionOnRecord = includeVision.checked
    this.captureMarkerSelection()
  }

  /** 通过活动 PointSet 稳定引用移动到当前选择目标。 */
  private async moveToSelectedPoint(): Promise<void> {
    const selected = this.selectedPoint()
    if (!selected) return
    await this.performMotion(
      'move_to_anchor',
      { point_id: selected.targetRef },
      `已移动到 ${selected.sourcePoint}。`
    )
  }

  /** 执行一次有限关节点动（Joint Jog）。 */
  private async jogJoint(element: HTMLElement): Promise<void> {
    this.captureJogSettings()
    await this.performMotion('jog_joint_once', {
      joint_ref: element.dataset.jogJoint ?? '',
      direction: element.dataset.direction ?? 'positive',
      step_deg: this.jointStep
    }, '关节 Jog 已完成。')
  }

  /** 执行一次有限工具中心点点动（TCP Jog）。 */
  private async jogTcp(element: HTMLElement): Promise<void> {
    this.captureJogSettings()
    const axis = element.dataset.jogTcp ?? ''
    const params = {
      axis,
      direction: element.dataset.direction ?? 'positive',
      frame_ref: this.tcpFrame,
      step: this.tcpStep
    }
    const success = 'TCP Jog 已完成。'
    if (usesMoveItExclusiveJog(this.allowedActions)) {
      await this.performMotion('jog_tcp_once', params, success)
    } else {
      await this.performDirectAction('jog_tcp_once', params, success)
    }
  }

  /** 执行一次绝对导轨位置移动。 */
  private async moveRailToPosition(): Promise<void> {
    const snapshot = this.snapshot
    if (!snapshot?.capabilities.railMove || !snapshot.rail) {
      this.message = '当前机械臂未配置可用导轨。'
      this.render()
      return
    }
    const positionMm = inputNumber(this.root, '[data-rail-position]', Number.NaN, true)
    const minimum = snapshot.rail.travelMinMm
    const maximum = snapshot.rail.travelMaxMm
    if (!Number.isFinite(positionMm)) {
      this.message = '请输入有效的绝对导轨位置。'
      this.render()
      return
    }
    if ((minimum !== null && positionMm < minimum) || (maximum !== null && positionMm > maximum)) {
      this.message = `导轨位置超出限位 ${formatLimit(minimum)}–${formatLimit(maximum)} mm。`
      this.render()
      return
    }
    if (!globalThis.confirm(`确认把导轨移动到 ${positionMm.toFixed(1)} mm？`)) return
    await this.performMotion(
      'move_rail_to_position',
      { position_mm: positionMm },
      `导轨已移动到 ${positionMm.toFixed(1)} mm。`
    )
  }

  /** 显式确认后把当前机械臂与可选导轨位置记录到作者目录目标。 */
  private async recordCurrentPoint(): Promise<void> {
    const selected = this.selectedPoint()
    if (!selected?.editable) return
    if (this.snapshot?.capabilities.pointRecord !== true) {
      this.message = '当前执行器不支持 PointSet v3 点位记录。'
      this.render()
      return
    }
    const includesRail = this.snapshot.rail !== null
    const visionNote = this.includeVisionOnRecord ? '，并关联视觉 marker' : ''
    if (!globalThis.confirm(
      `确认把当前机械臂${includesRail ? '与导轨' : ''}位置记录到 ${selected.sourcePoint}${visionNote}？`
    )) return
    this.captureTeachOptions()
    const params: Record<string, unknown> = {
      target_ref: selected.targetRef,
      expected_revision: this.snapshot.pointSetRevision,
      confirm: true
    }
    if (this.includeVisionOnRecord) {
      params.include_vision = true
      if (this.selectedMarkerRef) params.marker_ref = this.selectedMarkerRef
    }
    const success = `${selected.sourcePoint} 已记录并生成新的 PointSet 修订。`
    if (usesMoveItExclusiveJog(this.allowedActions)) {
      await this.performMotion('record_current_point', params, success)
    } else {
      await this.performDirectAction('record_current_point', params, success)
    }
  }

  private async calibrateCameraExtrinsic(): Promise<void> {
    if (!globalThis.confirm('确认执行摄像头外参标定占位登记？')) return
    const params = { confirm: true }
    const success = '摄像头外参标定已登记。'
    if (usesMoveItExclusiveJog(this.allowedActions)) {
      await this.performMotion('calibrate_camera_extrinsic', params, success)
    } else {
      await this.performDirectAction('calibrate_camera_extrinsic', params, success)
    }
  }

  private async calibrateTcp(): Promise<void> {
    if (!globalThis.confirm('确认执行 TCP 校准占位登记？')) return
    const params = { confirm: true }
    const success = 'TCP 校准已登记。'
    if (usesMoveItExclusiveJog(this.allowedActions)) {
      await this.performMotion('calibrate_tcp', params, success)
    } else {
      await this.performDirectAction('calibrate_tcp', params, success)
    }
  }

  private async recordMarker(): Promise<void> {
    const selected = this.selectedPoint()
    if (!selected?.groupRef) return
    if (!globalThis.confirm(`确认为仓 ${selected.groupRef} 记录 marker？`)) return
    const params = { warehouse_ref: selected.groupRef, confirm: true }
    const success = `${selected.groupRef} 的 marker 已登记。`
    if (usesMoveItExclusiveJog(this.allowedActions)) {
      await this.performMotion('record_marker', params, success)
    } else {
      await this.performDirectAction('record_marker', params, success)
    }
  }

  /** 在显式手动独占（Exclusive）下执行一个设备动作（Action）。 */
  private async performMotion(
    action: string,
    params: Record<string, unknown>,
    successMessage: string
  ): Promise<void> {
    if (!canCallAction(this.allowedActions, action)) {
      this.message = '动作未在卡片 manifest 中授权。'
      this.render()
      return
    }
    if (usesMoveItExclusiveJog(this.allowedActions) && this.exclusiveState !== 'exclusive') {
      this.message = '请先取得调试控制。'
      this.render()
      return
    }
    const bridge = getDeviceCardBridge()
    const moveItExclusive = usesMoveItExclusiveJog(this.allowedActions)
    let restoreExclusive = false
    try {
      this.actionPending = true
      if (moveItExclusive) {
        this.message = '正在把调试控制转交 MoveIt 动作任务…'
        this.render()
        const released = await bridge.releaseManualExclusive()
        this.exclusiveState = released.state
        if (released.state !== 'idle') {
          throw new Error(`调试控制释放后设备状态为 ${released.state}`)
        }
        restoreExclusive = true
        this.message = 'MoveIt 正在规划并执行…'
      } else {
        this.message = '正在执行…'
      }
      this.render()
      const run = await bridge.callAction(action, params)
      if (run.status !== 'DONE') throw new Error(run.error ?? `动作状态：${run.status}`)
      await this.refreshSnapshot(successMessage)
    } catch (error) {
      this.message = errorMessage(error, '机械臂动作失败。')
    } finally {
      if (restoreExclusive) {
        await this.restoreManualExclusiveAfterAction()
      }
      this.actionPending = false
      this.render()
    }
  }

  /** MoveIt 动作结束后尝试恢复手动独占；失败时保留 busy 并追加提示。 */
  private async restoreManualExclusiveAfterAction(): Promise<void> {
    const bridge = getDeviceCardBridge()
    try {
      const restored = await withTimeout(
        bridge.acquireManualExclusive(),
        5_000,
        '恢复调试控制等待超时'
      ).catch(async () => withTimeout(
        bridge.readManualExclusive(),
        5_000,
        '调试控制状态对账超时'
      ))
      this.exclusiveState = restored.state
      if (restored.state !== 'exclusive') {
        this.message = `${this.message}；设备已被其他任务占用，未恢复调试控制。`
      }
    } catch (error) {
      this.exclusiveState = 'busy'
      this.message = `${this.message}；${errorMessage(error, '未恢复调试控制。')}`
    }
  }

  /** 经 Host Bridge 取得或释放设备级手动独占（Exclusive）。 */
  private async changeExclusive(acquire: boolean): Promise<void> {
    const bridge = getDeviceCardBridge()
    try {
      const snapshot = acquire
        ? await bridge.acquireManualExclusive()
        : await bridge.releaseManualExclusive()
      this.exclusiveState = snapshot.state
      this.message = acquire ? '已取得调试控制。' : '已释放调试控制。'
    } catch (error) {
      this.message = errorMessage(error, '手动独占请求失败。')
    }
    this.render()
  }

  private selectedPoint() {
    return this.snapshot?.pointTargets.find(
      (point) => point.targetRef === this.selectedTargetRef
    ) ?? null
  }
}

function inputNumber(root: ShadowRoot, selector: string, fallback: number, allowZero = false): number {
  const value = Number(root.querySelector<HTMLInputElement>(selector)?.value)
  return Number.isFinite(value) && (allowZero ? value >= 0 : value > 0) ? value : fallback
}

/** 把可空导轨限位格式化为用户可识别的边界。 */
function formatLimit(value: number | null): string {
  return value === null ? '未知' : value.toFixed(1)
}

function errorMessage(error: unknown, fallback: string): string {
  return error instanceof Error ? error.message : fallback
}

function withTimeout<T>(
  pending: Promise<T>,
  timeoutMs: number,
  message: string
): Promise<T> {
  return new Promise<T>((resolve, reject) => {
    const timer = globalThis.setTimeout(() => reject(new Error(message)), timeoutMs)
    pending.then(
      (value) => {
        globalThis.clearTimeout(timer)
        resolve(value)
      },
      (error) => {
        globalThis.clearTimeout(timer)
        reject(error)
      }
    )
  })
}
