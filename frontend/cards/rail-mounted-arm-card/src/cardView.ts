/** 能力驱动单壳卡片渲染入口。 */
export { renderRobotCard as renderUnifiedCard, type RobotCardViewState } from './view'
export {
  canReadPointCatalog,
  canShowJogTab,
  canShowPointsTab,
  canShowVisionTab,
  resolveCatalogReadAction,
  usesMoveItExclusiveJog
} from './runtimeMode'
export { parsePointCatalogSnapshot } from './previewModel'
