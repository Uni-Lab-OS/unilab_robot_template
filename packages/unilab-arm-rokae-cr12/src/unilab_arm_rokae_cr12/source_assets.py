"""逐次核对冻结型号与网格，拒绝摘要漂移。"""
from pathlib import Path
import hashlib
import json

_MODEL_ROOT = Path(__file__).resolve().parent / "models"
_LOCK_SHA256 = "f7b33e346bd43e0b79d730580e8f9ab25cfe198bd20dfc708e41e938ca2d510a"

def load_source_bytes() -> bytes:
    """校验全套型号资料后返回原字节URDF；不展开原厂家xacro或控制插件。"""
    root = _MODEL_ROOT.resolve()
    lock_path = root / "source-lock.json"
    if hashlib.sha256(lock_path.read_bytes()).hexdigest() != _LOCK_SHA256:
        raise ValueError("CR12 来源清单摘要漂移")
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    for row in lock["files"]:
        path = (root / row["package_path"]).resolve()
        path.relative_to(root)
        if path.stat().st_size != row["bytes"] or hashlib.sha256(path.read_bytes()).hexdigest() != row["sha256"]:
            raise ValueError("CR12 型号资产摘要漂移: " + row["package_path"])
    return (root / "rokae_cr12_robot.urdf").read_bytes()
