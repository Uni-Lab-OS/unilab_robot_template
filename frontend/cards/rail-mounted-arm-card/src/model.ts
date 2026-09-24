/** SZLab Mixer 机械臂设备卡片（Device Card）的运行时值模型。 */

export type ExclusiveState = 'idle' | 'busy' | 'exclusive'
export type CardTab = 'points' | 'jog' | 'vision' | 'calibration'

export interface VisionMarker {
  warehouseRef: string
  markerRef: string
  label: string
  deviceFrameRef: string
  recordedAt: string | null
}

export interface VisionSnapshot {
  revision: string
  calibrationRevision: string
  markers: VisionMarker[]
  pointBindings: Record<string, string>
  cameraExtrinsicState: string
  tcpCalibrationState: string
  capabilities: {
    cameraExtrinsic: boolean
    tcpCalibration: boolean
    markerRecord: boolean
  }
}

export interface PointTarget {
  targetRef: string
  sourcePoint: string
  kind: string
  editable: boolean
  jointPositionsDeg: number[]
  railPositionMm: number | null
  groupRef: string
  tcpPose: TcpPose | null
}

export interface JointPosition {
  jointRef: string
  positionDeg: number
}

export interface TcpPose {
  frameRef: string
  xyzMm: number[]
  rotationXyzDeg: number[]
}

export interface PreviewCalibrationReviewAssets {
  slotReviewSvg: string
  wellReviewSvg: string
}

export interface PreviewCalibrationInfo {
  version: string
  digest: string
  status: string
  humanConfirmed: boolean
  qualification: string
  reviewAssets: PreviewCalibrationReviewAssets | null
}

/** Host 注入的 online 可能滞后；目录/Joint SSE 成功即视为 OS 可达。 */
export function resolveHostOnline(
  state: Record<string, unknown>,
  snapshot: RobotDebugSnapshot | null
): boolean {
  if (state.online === true || state.moveit_online === true) return true
  if (snapshot?.online === true) return true
  if ((snapshot?.pointTargets.length ?? 0) > 0) return true
  const jointState = asRecord(state.jointState)
  if (jointState.stale !== true && asArray(jointState.positions).length > 0) return true
  return false
}

export interface RobotDebugSnapshot {
  pointSetRevision: string
  source: string
  online: boolean
  idle: boolean
  stale: boolean
  tcpPose: TcpPose | null
  jointPositions: JointPosition[]
  rail: {
    positionMm: number
    travelMinMm: number
    travelMaxMm: number
  } | null
  pointTargets: PointTarget[]
  capabilities: {
    jointJog: boolean
    tcpJog: boolean
    railMove: boolean
    compositePointRecord: boolean
    pointRecord?: boolean
  }
  vision: VisionSnapshot
  calibration?: PreviewCalibrationInfo | null
}

/** 把动作（Action）结果中的未知 JSON 值收窄为卡片快照。 */
export function parseRobotDebugSnapshot(value: unknown): RobotDebugSnapshot | null {
  const actionResult = asRecord(value)
  // 通用卡片宿主（Host）把后端作业的 `return_info` 保留在动作任务
  // 结果的 `output` 字段；OS 再把领域返回值放进 `return_value`。
  const output = asRecord(actionResult.output)
  const nested = Object.keys(output).length > 0 ? output : actionResult
  const returnValue = asRecord(nested.return_value)
  const source = stringValue(returnValue.point_set_revision) ? returnValue : nested
  const capabilities = asRecord(source.capabilities)
  const tcp = asRecord(source.tcp_pose)
  const rail = asRecord(source.rail)
  const pointTargets = asArray(source.point_targets)
    .map(parsePointTarget)
    .filter((item): item is PointTarget => item !== null)
  const jointPositions = asArray(source.joint_positions)
    .map(parseJointPosition)
    .filter((item): item is JointPosition => item !== null)
  if (!stringValue(source.point_set_revision) || pointTargets.length === 0) {
    return null
  }
  const calibrationRaw = source.calibration
  let calibration: RobotDebugSnapshot['calibration'] = null
  if (calibrationRaw && typeof calibrationRaw === 'object') {
    const item = asRecord(calibrationRaw)
    const reviewAssets = asRecord(item.review_assets)
    calibration = {
      version: stringValue(item.version),
      digest: stringValue(item.digest),
      status: stringValue(item.status) || 'unknown',
      humanConfirmed: item.human_confirmed === true,
      qualification: stringValue(item.qualification),
      reviewAssets: stringValue(reviewAssets.slot_review_svg) || stringValue(reviewAssets.well_review_svg)
        ? {
            slotReviewSvg: stringValue(reviewAssets.slot_review_svg),
            wellReviewSvg: stringValue(reviewAssets.well_review_svg)
          }
        : null
    }
  }
  return {
    pointSetRevision: stringValue(source.point_set_revision),
    source: stringValue(source.source),
    online: source.online === true,
    idle: source.idle === true,
    stale: source.stale === true,
    tcpPose: tcp.frame_ref
      ? {
          frameRef: stringValue(tcp.frame_ref),
          xyzMm: numberArray(tcp.xyz_mm, 3),
          rotationXyzDeg: numberArray(tcp.rotation_xyz_deg, 3)
        }
      : null,
    jointPositions,
    rail: finiteNumber(rail.position_mm) !== null
      && finiteNumber(rail.travel_min_mm) !== null
      && finiteNumber(rail.travel_max_mm) !== null
      ? {
          positionMm: finiteNumber(rail.position_mm) ?? 0,
          travelMinMm: finiteNumber(rail.travel_min_mm) ?? 0,
          travelMaxMm: finiteNumber(rail.travel_max_mm) ?? 0
        }
      : null,
    pointTargets,
    capabilities: {
      jointJog: capabilities.joint_jog === true,
      tcpJog: capabilities.tcp_jog === true,
      railMove: capabilities.rail_move === true,
      compositePointRecord: capabilities.composite_point_record === true,
      pointRecord: capabilities.point_record === true || capabilities.composite_point_record === true
    },
    vision: parseVisionSnapshot(source.vision),
    calibration
  }
}

const PREVIEW_JOINT_COUNT = 6

/** 从 jointState SSE 解析 Preview 模式下的六轴读数（弧度转角度）。 */
export function parsePreviewJointPositions(
  runtimeState: Record<string, unknown>,
  jointCount = PREVIEW_JOINT_COUNT
): number[] {
  const jointState = asRecord(runtimeState.jointState)
  if (jointState.stale === true) return []
  const values = asRecord(jointState.jointStates)
  const entries = Object.entries(values)
    .map(([ref, value]) => ({ ref, radians: finiteNumber(value) }))
    .filter((item): item is { ref: string; radians: number } => item.radians !== null)
    .sort((left, right) => left.ref.localeCompare(right.ref))
  if (entries.length === 0) return []
  return entries.slice(0, jointCount).map((item) => item.radians * 180 / Math.PI)
}

/** 把 SSE 里的 qualified 名（如 robot_joint_1）规范成维护命令用的 canonical 名（joint_1）。 */
export function canonicalArmJointRef(jointRef: string): string {
  const match = jointRef.match(/_(joint_\d+)$/)
  return match ? match[1] : jointRef
}

function jointCatalogFromTelemetry(
  values: Readonly<Record<string, unknown>>
): JointPosition[] {
  return Object.entries(values)
    .map(([jointRef, value]) => ({
      jointRef: canonicalArmJointRef(jointRef),
      radians: finiteNumber(value)
    }))
    .filter((item): item is { jointRef: string; radians: number } => item.radians !== null)
    .sort((left, right) => left.jointRef.localeCompare(right.jointRef))
    .map(({ jointRef, radians }) => ({
      jointRef,
      positionDeg: radians * 180 / Math.PI
    }))
}

/** 用通用设备遥测（DeviceTelemetry）SSE 的最新帧覆盖关节读数。 */
export function mergeJointState(
  snapshot: RobotDebugSnapshot | null,
  runtimeState: Record<string, unknown>
): RobotDebugSnapshot | null {
  if (!snapshot) return null
  const jointState = asRecord(runtimeState.jointState)
  if (jointState.stale === true) return snapshot
  const values = asRecord(jointState.jointStates)
  if (Object.keys(values).length === 0) return snapshot
  const catalog = (snapshot.jointPositions.length > 0
    ? snapshot.jointPositions
    : jointCatalogFromTelemetry(values)
  ).map((joint) => ({
    ...joint,
    jointRef: canonicalArmJointRef(joint.jointRef)
  }))
  if (catalog.length === 0) return snapshot
  return {
    ...snapshot,
    jointPositions: catalog.map((joint) => {
      const match = Object.entries(values).find(([qualifiedRef, value]) => (
        (qualifiedRef === joint.jointRef ||
          qualifiedRef.endsWith(`_${joint.jointRef}`)) &&
        finiteNumber(value) !== null
      ))
      return match
        ? { ...joint, positionDeg: (finiteNumber(match[1]) ?? 0) * 180 / Math.PI }
        : joint
    })
  }
}

function parseVisionSnapshot(value: unknown): VisionSnapshot {
  const raw = asRecord(value)
  const capabilities = asRecord(raw.capabilities)
  const markers = asArray(raw.markers)
    .map(parseVisionMarker)
    .filter((item): item is VisionMarker => item !== null)
  const bindingsRaw = asRecord(raw.point_bindings)
  const pointBindings: Record<string, string> = {}
  for (const [key, binding] of Object.entries(bindingsRaw)) {
    if (typeof binding === 'string' && binding) pointBindings[key] = binding
  }
  return {
    revision: stringValue(raw.revision) || 'preview@unknown',
    calibrationRevision: stringValue(raw.calibration_revision),
    markers,
    pointBindings,
    cameraExtrinsicState: stringValue(raw.camera_extrinsic_state) || 'pending',
    tcpCalibrationState: stringValue(raw.tcp_calibration_state) || 'pending',
    capabilities: {
      cameraExtrinsic: capabilities.camera_extrinsic === true,
      tcpCalibration: capabilities.tcp_calibration === true,
      markerRecord: capabilities.marker_record === true
    }
  }
}

function parseVisionMarker(value: unknown): VisionMarker | null {
  const item = asRecord(value)
  const markerRef = stringValue(item.marker_ref)
  if (!markerRef) return null
  return {
    warehouseRef: stringValue(item.warehouse_ref),
    markerRef,
    label: stringValue(item.label) || markerRef,
    deviceFrameRef: stringValue(item.device_frame_ref),
    recordedAt: stringValue(item.recorded_at) || null
  }
}

export function defaultMarkerRefForWarehouse(warehouseRef: string): string {
  const normalized = warehouseRef.trim()
  return normalized ? `${normalized}/default` : ''
}

export function markerOptionsForSnapshot(
  snapshot: RobotDebugSnapshot | null,
  selectedGroupRef: string
): VisionMarker[] {
  if (!snapshot) return []
  const known = new Map(snapshot.vision.markers.map((item) => [item.markerRef, item]))
  const defaultRef = defaultMarkerRefForWarehouse(selectedGroupRef)
  if (defaultRef && !known.has(defaultRef)) {
    known.set(defaultRef, {
      warehouseRef: selectedGroupRef,
      markerRef: defaultRef,
      label: `${selectedGroupRef} 默认 marker`,
      deviceFrameRef: `device:${selectedGroupRef}`,
      recordedAt: null
    })
  }
  return [...known.values()].sort((left, right) => (
    left.markerRef.localeCompare(right.markerRef)
  ))
}

function parsePointTarget(value: unknown): PointTarget | null {
  const item = asRecord(value)
  const targetRef = stringValue(item.target_ref)
  const tcp = asRecord(item.tcp_pose)
  if (!targetRef) return null
  return {
    targetRef,
    sourcePoint: stringValue(item.source_point) || displaySourcePoint(targetRef),
    kind: stringValue(item.kind),
    editable: item.editable === true,
    jointPositionsDeg: numberArray(item.joint_positions_deg),
    railPositionMm: finiteNumber(item.rail_position_mm),
    groupRef: stringValue(item.group_ref),
    tcpPose: tcp.frame_ref
      ? {
          frameRef: stringValue(tcp.frame_ref),
          xyzMm: numberArray(tcp.xyz_mm, 3),
          rotationXyzDeg: numberArray(tcp.rotation_xyz_deg, 3)
        }
      : null
  }
}

function displaySourcePoint(targetRef: string): string {
  const parts = targetRef.split('.').filter(Boolean)
  const last = parts[parts.length - 1] ?? targetRef
  if (
    parts.length >= 2
    && (last === 'interaction_seed' || last === 'interaction' || last === 'entry')
  ) {
    return parts[parts.length - 2] ?? last
  }
  return last
}

function parseJointPosition(value: unknown): JointPosition | null {
  const item = asRecord(value)
  const jointRef = stringValue(item.joint_ref)
  const positionDeg = finiteNumber(item.position_deg)
  return jointRef && positionDeg !== null ? { jointRef, positionDeg } : null
}

export function asRecord(value: unknown): Record<string, unknown> {
  return value && typeof value === 'object' && !Array.isArray(value)
    ? value as Record<string, unknown>
    : {}
}

function asArray(value: unknown): unknown[] {
  return Array.isArray(value) ? value : []
}

function numberArray(value: unknown, expected?: number): number[] {
  const numbers = asArray(value).map(finiteNumber).filter((item): item is number => item !== null)
  return expected === undefined || numbers.length === expected ? numbers : []
}

function finiteNumber(value: unknown): number | null {
  const number = typeof value === 'number' ? value : Number.NaN
  return Number.isFinite(number) ? number : null
}

function stringValue(value: unknown): string {
  return typeof value === 'string' ? value : ''
}
