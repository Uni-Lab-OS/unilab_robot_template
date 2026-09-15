/** SZLab Mixer 机械臂设备卡片（Device Card）的运行时值模型。 */

export type ExclusiveState = 'idle' | 'busy' | 'exclusive'
export type CardTab = 'points' | 'jog' | 'vision'

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
  }
  vision: VisionSnapshot
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
      compositePointRecord: capabilities.composite_point_record === true
    },
    vision: parseVisionSnapshot(source.vision)
  }
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
  return {
    ...snapshot,
    jointPositions: snapshot.jointPositions.map((joint) => {
      const match = Object.entries(values).find(([qualifiedRef, value]) => (
        qualifiedRef.endsWith(joint.jointRef) && finiteNumber(value) !== null
      ))
      return match
        ? { ...joint, positionDeg: (finiteNumber(match[1]) ?? 0) * 180 / Math.PI }
        : joint
    })
  }
}

/** 提供卡片开发检查使用的非实时视觉样本；实时模式绝不使用。 */
export function mockRobotDebugSnapshot(): RobotDebugSnapshot {
  const points = ['home', 'S0722', 'P01', 'S061', 'S04', 'S05', 'S081', 'L1B1']
  return {
    pointSetRevision: 'szlab-mixer-cr7-rail@3.0.0',
    source: 'mock:moveit',
    online: true,
    idle: true,
    stale: false,
    tcpPose: {
      frameRef: 'arm_base',
      xyzMm: [-137.0, 226.0, 549.0],
      rotationXyzDeg: [-90.0, 0.0, 0.0]
    },
    jointPositions: Array.from({ length: 6 }, (_, index) => ({
      jointRef: `cr7_joint_${index + 1}`,
      positionDeg: [-93.0, -30.0, 128.0, -99.0, -86.0, 0.0][index] ?? 0
    })),
    rail: { positionMm: 586.1, travelMinMm: 0, travelMaxMm: 2250 },
    pointTargets: points.map((point, index) => ({
      targetRef: point === 'home'
        ? 'szlab.arm.home'
        : `mock.targets.${point}.interaction_seed`,
      sourcePoint: point,
      kind: 'joint_positions',
      editable: point !== 'home',
      jointPositionsDeg: [],
      railPositionMm: index * 100,
      groupRef: point === 'home' ? 'szlab.arm' : 's07_process_warehouse',
      tcpPose: {
        frameRef: 'arm_base',
        xyzMm: [index * 10, 226.0, 549.0],
        rotationXyzDeg: [-90.0, 0.0, 0.0]
      }
    })),
    capabilities: {
      jointJog: true,
      tcpJog: true,
      railMove: false,
      compositePointRecord: true
    },
    vision: {
      revision: 'szlab-mixer-vision-registry@1.0.0',
      calibrationRevision: 'szlab-mixer-cr7-rail@1.0.0',
      markers: [
        {
          warehouseRef: 's07_process_warehouse',
          markerRef: 's07_process_warehouse/default',
          label: 'S07 默认 marker',
          deviceFrameRef: 'device:s07_process_warehouse',
          recordedAt: null
        },
        {
          warehouseRef: 's3_unused_beaker',
          markerRef: 's3_unused_beaker/default',
          label: 'S3 默认 marker',
          deviceFrameRef: 'device:s3_unused_beaker',
          recordedAt: null
        }
      ],
      pointBindings: {},
      cameraExtrinsicState: 'pending',
      tcpCalibrationState: 'pending',
      capabilities: {
        cameraExtrinsic: true,
        tcpCalibration: true,
        markerRecord: true
      }
    }
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
    revision: stringValue(raw.revision) || 'szlab-mixer-vision-registry@1.0.0',
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
