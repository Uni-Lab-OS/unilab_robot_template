import assert from 'node:assert/strict'
import test from 'node:test'

import { parsePreviewSnapshot } from '../src/previewModel.ts'

test('parsePreviewSnapshot 解析 snapshot_json 库位与关节', () => {
  const snapshot = parsePreviewSnapshot({
    return_value: {
      point_set_revision: 'cad-v0.1-candidate@abc',
      idle: true,
      snapshot_json: JSON.stringify({
        joint_positions: [
          { joint_ref: 'elite_left_joint_1', position_deg: 10 },
          { joint_ref: 'elite_left_joint_2', position_deg: 20 }
        ],
        sites: [
          { site_ref: 'loading_1', label: 'loading_1', group_ref: 'loading', xyz_m: [1, 2, 3] }
        ],
        calibration: {
          version: 'cad-v0.1-candidate',
          digest: 'abc123',
          status: 'candidate_simulation_only',
          human_confirmed: false,
          qualification: 'CAD-derived candidate'
        },
        capabilities: { joint_jog: true, tcp_jog: false, rail_move: true },
        rail: { position_mm: 12.3 }
      })
    }
  })

  assert.ok(snapshot)
  assert.equal(snapshot?.sites[0]?.siteRef, 'loading_1')
  assert.equal(snapshot?.jointPositions[0]?.positionDeg, 10)
  assert.equal(snapshot?.capabilities.jointJog, true)
  assert.equal(snapshot?.railPositionMm, 12.3)
  assert.equal(snapshot?.calibration?.version, 'cad-v0.1-candidate')
  assert.equal(snapshot?.calibration?.humanConfirmed, false)
})
