# 资产布局（通用）

```text
devices/<device>/
  moveit_model.py
  models/
    model.yaml
    <vendor>.urdf      # 厂商只读副本
    meshes/<mesh_slug>/
```

## vendor URDF 铁律

| 规则 | 说明 |
|------|------|
| **只读拷贝** | 从 `model.yaml` → `source.repository` + `source.path` 整文件复制 |
| **禁止编辑** | 不得改 `<origin>` xyz/rpy、`<axis>`、link/joint 名 |
| **禁止手搓** | 不得按 CR7/其它型号模板「生成」新型号 URDF |
| **digest 对上游** | `source.sha256` = 上游 vendor 文件 SHA256，不是改完再算 |
| **mesh 同源** | STL 与 URDF 必须同一 vendor 包/版本 |

验收：

```bash
python -c "import hashlib, pathlib; p=pathlib.Path('devices/<device>/models/<vendor>.urdf'); print(hashlib.sha256(p.read_bytes()).hexdigest())"
# 必须等于 model.yaml source.sha256，且等于上游官方同路径文件
```

禁止：`model.yaml` 在 device 根目录；只有 URDF 无 STL；URDF 与 mesh 来自不同 vendor 版本。

换 URDF 时：**重新只读拷贝** → 更新 digest → 同 PR 重填 `moveit_model` 映射（**仍不改 URDF 几何**）。
