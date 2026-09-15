import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import test from 'node:test'

const sourceUrl = new URL('../src/index.ts', import.meta.url)

test('live 卡片挂载不隐式提交任何设备 Action', async () => {
  const source = (await readFile(sourceUrl, 'utf8')).replace(/\r\n/g, '\n')
  const connectedCallback = source.match(
    /async connectedCallback\(\): Promise<void> \{(?<body>[\s\S]*?)\n  \}\n\n  \/\*\* 断开时/u
  )?.groups?.body

  assert.ok(connectedCallback, '必须能定位 connectedCallback 实现')
  assert.doesNotMatch(connectedCallback, /callAction\s*\(/u)
  assert.doesNotMatch(connectedCallback, /refreshSnapshot\s*\(/u)
  assert.match(connectedCallback, /subscribeState\s*\(/u)
})
