import assert from 'node:assert/strict'
import test from 'node:test'

import { renderPreviewCard } from '../src/previewView.ts'

test('Preview UI 展示点位页与 Jog 页签', () => {
  const html = renderPreviewCard({
    actionBusy: false,
    allowedActions: ['read_preview_snapshot', 'moveJ', 'home', 'stop', 'set_joint'],
    hostOnline: true,
    jointStep: 1,
    jointTargets: [90, -65, -70, -135, 90, 0],
    message: '',
    moving: false,
    selectedSiteRef: 'loading_1',
    snapshot: {
      pointSetRevision: 'cad-v0.1-candidate@test',
      sites: [{
        siteRef: 'loading_1',
        label: 'loading_1',
        groupRef: 'loading',
        xyzM: [1, 2, 3]
      }],
      jointPositions: [{ jointRef: 'j1', positionDeg: 10 }],
      calibration: {
        version: 'cad-v0.1-candidate',
        digest: '3dea4c2dbfd9',
        status: 'candidate_simulation_only',
        humanConfirmed: false,
        qualification: 'CAD-derived candidate',
        reviewAssets: {
          slotReviewSvg: 'points/cad-v0.1-candidate/review.svg',
          wellReviewSvg: 'points/cad-v0.1-candidate/well-review.svg'
        }
      },
      capabilities: { jointJog: true, tcpJog: false, railMove: false },
      idle: true,
      railPositionMm: null
    },
    tab: 'points'
  })

  assert.match(html, /data-runtime-mode="preview"/u)
  assert.match(html, /点位与姿态/u)
  assert.match(html, /校准点/u)
  assert.match(html, /Jog 与示教/u)
  assert.match(html, /data-preview-site="loading_1"/u)
  assert.match(html, /data-preview-refresh/u)
})

test('Preview 校准点页签展示 CAD 点位包摘要', () => {
  const html = renderPreviewCard({
    actionBusy: false,
    allowedActions: ['read_preview_snapshot'],
    hostOnline: true,
    jointStep: 1,
    jointTargets: [0, 0, 0, 0, 0, 0],
    message: '',
    moving: false,
    selectedSiteRef: 'loading_1',
    snapshot: {
      pointSetRevision: 'cad-v0.1-candidate@abc',
      sites: [{
        siteRef: 'loading_1',
        label: 'loading_1',
        groupRef: 'loading',
        xyzM: [1.1, 2.2, 3.3]
      }],
      jointPositions: [],
      calibration: {
        version: 'cad-v0.1-candidate',
        digest: '3dea4c2dbfd9ff4e7502b47ee3e80362c5e86e8122c69bbc7e7136a949b90647',
        status: 'candidate_simulation_only',
        humanConfirmed: false,
        qualification: 'CAD-derived candidate',
        reviewAssets: null
      },
      capabilities: { jointJog: true, tcpJog: false, railMove: false },
      idle: true,
      railPositionMm: null
    },
    tab: 'calibration'
  })

  assert.match(html, /CAD 点位包/u)
  assert.match(html, /candidate_simulation_only/u)
  assert.match(html, /1\.1, 2\.2, 3\.3 m/u)
})

test('Preview UI 按 allowedActions 隐藏未授权按钮', () => {
  const html = renderPreviewCard({
    actionBusy: false,
    allowedActions: ['home'],
    hostOnline: true,
    jointStep: 1,
    jointTargets: [0, 0, 0, 0, 0, 0],
    message: '',
    moving: false,
    selectedSiteRef: '',
    snapshot: null,
    tab: 'points'
  })

  assert.match(html, /data-preview-home/u)
  assert.doesNotMatch(html, /data-preview-movej/u)
  assert.doesNotMatch(html, /data-preview-stop/u)
})
