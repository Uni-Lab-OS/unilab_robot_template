import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import test from 'node:test'

const sourceUrl = new URL('../src/index.ts', import.meta.url)

test('卡片挂载时自动读取领域 PointSet 目录', async () => {
  const source = (await readFile(sourceUrl, 'utf8')).replace(/\r\n/g, '\n')
  const connectedCallback = source.match(
    /async connectedCallback\(\): Promise<void> \{(?<body>[\s\S]*?)\n  \}\n\n  \/\*\* 断开时/u
  )?.groups?.body

  assert.ok(connectedCallback, '必须能定位 connectedCallback 实现')
  assert.match(connectedCallback, /refreshSnapshot\s*\(/u)
  assert.match(connectedCallback, /subscribeState\s*\(/u)
  assert.doesNotMatch(connectedCallback, /mockRobotDebugSnapshot/u)
})
