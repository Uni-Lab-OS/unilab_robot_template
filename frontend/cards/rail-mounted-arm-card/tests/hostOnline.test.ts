import assert from 'node:assert/strict'
import test from 'node:test'

import { resolveHostOnline, type RobotDebugSnapshot } from '../src/model.ts'

const snapshot = (overrides: Partial<RobotDebugSnapshot> = {}): RobotDebugSnapshot => ({
  pointSetRevision: 'demo@1',
  source: 'point_set',
  online: false,
  idle: true,
  stale: true,
  tcpPose: null,
  jointPositions: [],
  rail: null,
  pointTargets: [{
    targetRef: 'robot_cell.arm.standby',
    sourcePoint: 'standby',
    kind: 'anchor',
    editable: false,
    jointPositionsDeg: [0, 0, 0, 0, 0, 0],
    railPositionMm: null,
    groupRef: 'arm',
    tcpPose: null
  }],
  capabilities: {
    jointJog: false,
    tcpJog: false,
    railMove: false,
    compositePointRecord: false
  },
  vision: {
    revision: '',
    calibrationRevision: '',
    markers: [],
    pointBindings: {},
    cameraExtrinsicState: '',
    tcpCalibrationState: '',
    capabilities: {
      cameraExtrinsic: false,
      tcpCalibration: false,
      markerRecord: false
    }
  },
  ...overrides
})

test('resolveHostOnline 在 Host online 滞后时仍可从 PointSet 目录判定 OS 可达', () => {
  assert.equal(resolveHostOnline({ online: false }, snapshot()), true)
  assert.equal(resolveHostOnline({ online: false }, null), false)
  assert.equal(resolveHostOnline({ online: true }, null), true)
})
