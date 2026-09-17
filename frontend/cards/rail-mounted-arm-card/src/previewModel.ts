import type {
  PreviewCalibrationInfo,
  PreviewCalibrationReviewAssets,
  RobotDebugSnapshot
} from './model'
import { parseRobotDebugSnapshot } from './model'

export type { PreviewCalibrationInfo, PreviewCalibrationReviewAssets }

export type PreviewTab = 'points' | 'jog' | 'calibration'

export interface PreviewJointPosition {
  jointRef: string
  positionDeg: number
}

export interface PreviewSiteTarget {
  siteRef: string
  label: string
  groupRef: string
  xyzM: number[]
}

export interface PreviewSnapshot {
  pointSetRevision: string
  sites: PreviewSiteTarget[]
  jointPositions: PreviewJointPosition[]
  calibration: PreviewCalibrationInfo | null
  capabilities: {
    jointJog: boolean
    tcpJog: boolean
    railMove: boolean
  }
  idle: boolean
  railPositionMm: number | null
}

/** 解析 Preview 设备 read_preview_snapshot 返回值。 */
export function parsePreviewSnapshot(value: unknown): PreviewSnapshot | null {
  const actionResult = asRecord(value)
  const output = asRecord(actionResult.output)
  const nested = Object.keys(output).length > 0 ? output : actionResult
  const returnValue = asRecord(nested.return_value)
  const source = resolvePreviewSnapshotPayload(returnValue, nested)
  const sitesRaw = Array.isArray(source.sites) ? source.sites : []
  const sites = sitesRaw
    .map(parsePreviewSite)
    .filter((item): item is PreviewSiteTarget => item !== null)
    .sort((left, right) => left.siteRef.localeCompare(right.siteRef))
  const jointsRaw = Array.isArray(source.joint_positions) ? source.joint_positions : []
  const jointPositions = jointsRaw
    .map(parseJointPosition)
    .filter((item): item is PreviewJointPosition => item !== null)
  const capabilities = asRecord(source.capabilities)
  const rail = asRecord(source.rail)
  const railPositionMm = finiteNumber(rail.position_mm)
  return {
    pointSetRevision: stringValue(source.point_set_revision) || 'preview@unknown',
    sites,
    jointPositions,
    calibration: parsePreviewCalibration(source.calibration),
    capabilities: {
      jointJog: capabilities.joint_jog !== false,
      tcpJog: capabilities.tcp_jog === true,
      railMove: capabilities.rail_move === true
    },
    idle: source.idle !== false,
    railPositionMm
  }
}

/** 统一解析 read_point_catalog / read_debug_snapshot / read_preview_snapshot。 */
export function parsePointCatalogSnapshot(value: unknown): RobotDebugSnapshot | null {
  const debug = parseRobotDebugSnapshot(value)
  if (debug) return debug
  const preview = parsePreviewSnapshot(value)
  if (!preview) return null
  return previewSnapshotToDebugSnapshot(preview)
}

function previewSnapshotToDebugSnapshot(preview: PreviewSnapshot): RobotDebugSnapshot {
  return {
    pointSetRevision: preview.pointSetRevision,
    source: 'preview',
    online: true,
    idle: preview.idle,
    stale: false,
    tcpPose: null,
    jointPositions: preview.jointPositions,
    rail: preview.railPositionMm === null
      ? null
      : {
          positionMm: preview.railPositionMm,
          travelMinMm: 0,
          travelMaxMm: 0
        },
    pointTargets: preview.sites.map((site) => ({
      targetRef: site.siteRef,
      sourcePoint: site.label || site.siteRef,
      kind: 'joint_positions',
      editable: true,
      jointPositionsDeg: [],
      railPositionMm: null,
      groupRef: site.groupRef,
      tcpPose: site.xyzM.length === 3
        ? {
            frameRef: 'world',
            xyzMm: site.xyzM.map((value) => value * 1000),
            rotationXyzDeg: [0, 0, 0]
          }
        : null
    })),
    capabilities: {
      jointJog: preview.capabilities.jointJog,
      tcpJog: preview.capabilities.tcpJog,
      railMove: preview.capabilities.railMove,
      compositePointRecord: false,
      pointRecord: false
    },
    vision: {
      revision: 'preview@unknown',
      calibrationRevision: preview.calibration?.digest ?? '',
      markers: [],
      pointBindings: {},
      cameraExtrinsicState: 'pending',
      tcpCalibrationState: 'pending',
      capabilities: {
        cameraExtrinsic: false,
        tcpCalibration: false,
        markerRecord: false
      }
    },
    calibration: preview.calibration
  }
}

function parsePreviewCalibration(value: unknown): PreviewCalibrationInfo | null {
  const item = asRecord(value)
  const version = stringValue(item.version)
  if (!version) return null
  const reviewAssets = asRecord(item.review_assets)
  const slotReviewSvg = stringValue(reviewAssets.slot_review_svg)
  const wellReviewSvg = stringValue(reviewAssets.well_review_svg)
  return {
    version,
    digest: stringValue(item.digest),
    status: stringValue(item.status) || 'unknown',
    humanConfirmed: item.human_confirmed === true,
    qualification: stringValue(item.qualification),
    reviewAssets: slotReviewSvg || wellReviewSvg
      ? {
          slotReviewSvg,
          wellReviewSvg
        }
      : null
  }
}

function parsePreviewSite(value: unknown): PreviewSiteTarget | null {
  const item = asRecord(value)
  const siteRef = stringValue(item.site_ref)
  if (!siteRef) return null
  return {
    siteRef,
    label: stringValue(item.label) || siteRef,
    groupRef: stringValue(item.group_ref),
    xyzM: numberArray(item.xyz_m, 3)
  }
}

function parseJointPosition(value: unknown): PreviewJointPosition | null {
  const item = asRecord(value)
  const jointRef = stringValue(item.joint_ref)
  const positionDeg = finiteNumber(item.position_deg)
  return jointRef && positionDeg !== null ? { jointRef, positionDeg } : null
}

function resolvePreviewSnapshotPayload(
  returnValue: Record<string, unknown>,
  nested: Record<string, unknown>
): Record<string, unknown> {
  const envelope = Object.keys(returnValue).length > 0 ? returnValue : nested
  const snapshotJson = envelope.snapshot_json
  if (typeof snapshotJson === 'string' && snapshotJson.trim()) {
    try {
      const parsed = JSON.parse(snapshotJson)
      return {
        ...asRecord(parsed),
        point_set_revision: envelope.point_set_revision,
        idle: envelope.idle
      }
    } catch {
      return envelope
    }
  }
  return envelope
}

function asRecord(value: unknown): Record<string, unknown> {
  return value && typeof value === 'object' && !Array.isArray(value)
    ? value as Record<string, unknown>
    : {}
}

function numberArray(value: unknown, expected?: number): number[] {
  const numbers = (Array.isArray(value) ? value : [])
    .map(finiteNumber)
    .filter((item): item is number => item !== null)
  return expected === undefined || numbers.length === expected ? numbers : []
}

function finiteNumber(value: unknown): number | null {
  const number = typeof value === 'number' ? value : Number.NaN
  return Number.isFinite(number) ? number : null
}

function stringValue(value: unknown): string {
  return typeof value === 'string' ? value : ''
}
