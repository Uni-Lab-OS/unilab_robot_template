import assert from 'node:assert/strict'
import test from 'node:test'

import type { RobotDebugSnapshot } from '../src/model.ts'
import { mockRobotDebugSnapshot } from '../src/model.ts'
import { renderRobotCard } from '../src/view.ts'

function baseState(snapshot: RobotDebugSnapshot) {
  return {
    actionBusy: false,
    exclusiveState: 'exclusive' as const,
    hostOnline: true,
    includeVisionOnRecord: false,
    jointStep: 1,
    message: '',
    moving: false,
    selectedMarkerRef: 'szlab_s08_cap_station/default',
    selectedTargetRef: 'szlab_s08_cap_station.S081_pour_tilt',
    snapshot,
    tab: 'points' as const,
    tcpFrame: 'arm_base' as const,
    tcpStep: 2
  }
}

/** 构造同时包含实时状态和不同 PointSet 目标的卡片快照。 */
function snapshotWithRail(): RobotDebugSnapshot {
  return {
    pointSetRevision: 'szlab-mixer-cr7-rail@3.0.1',
    source: 'test:moveit',
    online: true,
    idle: true,
    stale: false,
    tcpPose: {
      frameRef: 'arm_base',
      xyzMm: [1, 2, 3],
      rotationXyzDeg: [4, 5, 6]
    },
    jointPositions: Array.from({ length: 6 }, (_, index) => ({
      jointRef: `cr7_joint_${index + 1}`,
      positionDeg: index + 1
    })),
    rail: { positionMm: 12.3, travelMinMm: 0, travelMaxMm: 2250 },
    pointTargets: [{
      targetRef: 'szlab_s08_cap_station.S081_pour_tilt',
      sourcePoint: 'S081_pour_tilt',
      kind: 'joint_positions',
      editable: true,
      jointPositionsDeg: [10, 20, 30, 40, 50, 60],
      railPositionMm: 432.1,
      groupRef: 'szlab_s08_cap_station',
      tcpPose: {
        frameRef: 'arm_base',
        xyzMm: [101, 202, 303],
        rotationXyzDeg: [11, 22, 33]
      }
    }],
    capabilities: {
      jointJog: true,
      tcpJog: true,
      railMove: false,
      compositePointRecord: true
    },
    vision: mockRobotDebugSnapshot().vision
  }
}

test('点位页展示选中目标的 TCP、六轴和导轨位置', () => {
  const snapshot = snapshotWithRail()
  const html = renderRobotCard({
    ...baseState(snapshot),
    tab: 'points'
  })

  assert.match(html, /目标位置 · TCP \/ 关节/u)
  assert.match(html, /101\.0/u)
  assert.match(html, /10\.0°/u)
  assert.match(html, /目标导轨[^<]*432\.1 mm/u)
})

test('有导轨移动能力时 Jog 页显示绝对位置输入和移动按钮', () => {
  const snapshot = snapshotWithRail()
  snapshot.capabilities.railMove = true
  const html = renderRobotCard({
    ...baseState(snapshot),
    tab: 'jog'
  })

  assert.match(html, /当前 12\.3 mm/u)
  assert.match(html, /data-rail-position/u)
  assert.match(html, /data-move-rail/u)
  assert.doesNotMatch(html, /只读/u)
})

test('视觉校准页签展示三项校准按钮', () => {
  const html = renderRobotCard({
    ...baseState(snapshotWithRail()),
    tab: 'vision'
  })

  assert.match(html, /视觉校准/u)
  assert.match(html, /data-calibrate-camera/u)
  assert.match(html, /data-calibrate-tcp/u)
  assert.match(html, /data-record-marker/u)
})

test('勾选视觉参数后显示 marker 下拉框', () => {
  const snapshot = snapshotWithRail()
  const html = renderRobotCard({
    ...baseState(snapshot),
    tab: 'jog',
    includeVisionOnRecord: true,
    selectedMarkerRef: 'szlab_s08_cap_station/default'
  })

  assert.match(html, /加入视觉校准的参数/u)
  assert.match(html, /data-marker-ref/u)
  assert.match(html, /szlab_s08_cap_station\/default/u)
})

test('未勾选视觉参数时不显示 marker 下拉框', () => {
  const html = renderRobotCard({
    ...baseState(snapshotWithRail()),
    tab: 'jog',
    includeVisionOnRecord: false
  })

  assert.doesNotMatch(html, /data-marker-ref/u)
})
