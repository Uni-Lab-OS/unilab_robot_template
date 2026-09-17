export type CardRuntimeMode = 'preview' | 'commissioning'

const CATALOG_READ_ACTIONS = [
  'read_point_catalog',
  'read_debug_snapshot',
  'read_preview_snapshot'
] as const

/** 按 Host 注入的 allowedActions 推断卡片 UI 模式（过渡；优先用 tab 级门控）。 */
export function inferRuntimeMode(
  allowedActions: readonly string[],
  mode: 'mock' | 'live' = 'live'
): CardRuntimeMode {
  if (allowedActions.length === 0) {
    return mode === 'mock' ? 'commissioning' : 'preview'
  }
  if (allowedActions.includes('read_debug_snapshot') || allowedActions.includes('jog_joint_once')) {
    return 'commissioning'
  }
  return 'preview'
}

/** 点位目录读取动作优先级：canonical → MoveIt alias → Preview alias。 */
export function resolveCatalogReadAction(
  allowedActions: readonly string[] | undefined
): string | null {
  if (allowedActions === undefined) return 'read_debug_snapshot'
  for (const action of CATALOG_READ_ACTIONS) {
    if (allowedActions.includes(action)) return action
  }
  return null
}

export function canReadPointCatalog(
  allowedActions: readonly string[] | undefined
): boolean {
  return resolveCatalogReadAction(allowedActions) !== null
}

export function canShowPointsTab(
  allowedActions: readonly string[] | undefined
): boolean {
  return canReadPointCatalog(allowedActions)
}

export function canShowJogTab(
  allowedActions: readonly string[] | undefined
): boolean {
  if (allowedActions === undefined) return true
  return allowedActions.includes('set_joint')
    || allowedActions.includes('moveJ')
    || allowedActions.includes('jog_joint_once')
    || allowedActions.includes('jog_tcp_once')
    || allowedActions.includes('record_current_point')
    || allowedActions.includes('teach_point_from_current')
    || allowedActions.includes('move_rail_to_position')
    || allowedActions.includes('home')
}

export function canShowVisionTab(
  allowedActions: readonly string[] | undefined
): boolean {
  if (allowedActions === undefined) return true
  return allowedActions.includes('calibrate_camera_extrinsic')
    || allowedActions.includes('calibrate_tcp')
    || allowedActions.includes('record_marker')
}

export function usesMoveItExclusiveJog(
  allowedActions: readonly string[] | undefined
): boolean {
  if (allowedActions === undefined) return true
  return allowedActions.includes('jog_joint_once')
    || allowedActions.includes('move_to_anchor')
    || allowedActions.includes('move_rail_to_position')
}

/** callAction 与按钮渲染前的统一授权校验；undefined 表示 mock 未注入列表。 */
export function canCallAction(
  allowedActions: readonly string[] | undefined,
  action: string
): boolean {
  if (allowedActions === undefined) return true
  return allowedActions.includes(action)
}

export function readAllowedActions(config: Record<string, unknown>): string[] {
  const raw = config.allowedActions
  return Array.isArray(raw) ? raw.filter((item): item is string => typeof item === 'string') : []
}

export function readAllowedState(config: Record<string, unknown>): string[] {
  const raw = config.allowedState
  return Array.isArray(raw) ? raw.filter((item): item is string => typeof item === 'string') : []
}
