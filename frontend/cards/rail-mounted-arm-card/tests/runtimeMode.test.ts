import assert from 'node:assert/strict'
import test from 'node:test'

import {
  canCallAction,
  canReadPointCatalog,
  inferRuntimeMode,
  readAllowedActions,
  readAllowedState,
  resolveCatalogReadAction
} from '../src/runtimeMode.ts'

test('inferRuntimeMode：无 read_debug_snapshot 时为 preview', () => {
  assert.equal(
    inferRuntimeMode(['moveJ', 'home', 'stop'], 'live'),
    'preview'
  )
})

test('inferRuntimeMode：含 read_debug_snapshot 时为 commissioning', () => {
  assert.equal(
    inferRuntimeMode(['read_debug_snapshot', 'home'], 'live'),
    'commissioning'
  )
})

test('inferRuntimeMode：空列表 mock 回落 commissioning', () => {
  assert.equal(inferRuntimeMode([], 'mock'), 'commissioning')
})

test('resolveCatalogReadAction 优先级', () => {
  assert.equal(
    resolveCatalogReadAction(['read_preview_snapshot', 'read_point_catalog']),
    'read_point_catalog'
  )
  assert.equal(
    resolveCatalogReadAction(['read_preview_snapshot', 'read_debug_snapshot']),
    'read_debug_snapshot'
  )
  assert.equal(canReadPointCatalog(['moveJ']), false)
})

test('canCallAction 与 readAllowed*', () => {
  assert.equal(canCallAction(['home'], 'home'), true)
  assert.equal(canCallAction(['home'], 'moveJ'), false)
  assert.equal(canCallAction(undefined, 'home'), true)
  assert.deepEqual(
    readAllowedActions({ allowedActions: ['moveJ', 1, 'home'] }),
    ['moveJ', 'home']
  )
  assert.deepEqual(readAllowedState({ allowedState: ['online', false, 'jointState'] }), ['online', 'jointState'])
})
