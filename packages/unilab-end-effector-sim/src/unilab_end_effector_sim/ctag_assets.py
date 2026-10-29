"""CTAG2F90 的冻结来源与离线型号派生；不装配机械臂或启动仿真。"""
from __future__ import annotations

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from unilab_robot_contracts.urdf_reference import inertia_record

from .mimic import mimic_map
from .urdf_reference import MimicModel

_ASSET_ROOT = Path(__file__).parent / "models" / "ctag2f90"
_SOURCE_LOCK_SHA256 = "6cd70099d16cfacf47f5270fedd1d63622d130efd4c2cc3a85af1ac22bf32c0f"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def _sources() -> tuple[dict, dict[str, Path]]:
    """先核对全部冻结输入，再允许写出派生文件。"""
    root = _ASSET_ROOT.resolve()
    lock_path = root / "source-lock.json"
    if _sha(lock_path) != _SOURCE_LOCK_SHA256:
        raise ValueError("CTAG 来源清单摘要不匹配")
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    paths = {}
    for row in lock["files"]:
        path = (root / row["package_path"]).resolve()
        path.relative_to(root)
        if path.stat().st_size != row["bytes"] or _sha(path) != row["sha256"]:
            raise ValueError("CTAG 来源文件摘要不匹配: " + row["id"])
        paths[row["id"]] = path
    return lock, paths


def _normalize(grip: ET.Element, max_velocity_rad_s: float) -> tuple[list, dict]:
    affine=mimic_map(grip)
    if set(s for s,_,_ in affine.values())!={'Left_1_Joint'} or len(affine)!=8:
        raise ValueError('unexpected CTAG linkage')
    changes=[];inertias={}
    for link in grip.findall('link'):
        rec=inertia_record(link)
        if rec is None or not rec['necessary_conditions_pass']: raise ValueError('invalid gripper source inertia')
        inertias[link.get('name')]=rec
        link.set('name','ctag_'+link.get('name'))
    for j in grip.findall('joint'):
        old=j.get('name');j.set('name','ctag_'+old)
        for tag in ('parent','child'):j.find(tag).set('link','ctag_'+j.find(tag).get('link'))
        if j.get('type')=='fixed':
            if j.find('axis') is not None:j.remove(j.find('axis'))
            continue
        source_name,m,b=affine[old]
        lo,hi=sorted([b,m+b])
        lim=j.find('limit');before=dict(lim.attrib)
        lim.attrib.update(lower=str(lo),upper=str(hi),velocity=str(abs(m)*max_velocity_rad_s),effort='2.0')
        changes.append({'joint':j.get('name'),'before':before,'after':dict(lim.attrib),
                        'reason':'range from original affine mimic over actuator [0,1]; development velocity; passive effort is parser metadata only'})
        mimic=j.find('mimic')
        if mimic is not None:
            mimic.attrib.update(joint='ctag_'+source_name,multiplier=str(m),offset=str(b))
    return changes, inertias



def prepare_ctag_model(output: str | Path, *, max_velocity_rad_s: float) -> Path:
    """将已有型号派生规则应用到冻结来源，返回独立夹爪 URDF。

    速度必须由调用者明确提供，属于开发参数。输出目录必须不存在。
    沿用旧参考模型的 ctag_ 名称；本入口不是 OS Provider，也不提供双实例装配。
    """
    if type(max_velocity_rad_s) not in (int, float) or not math.isfinite(max_velocity_rad_s) or max_velocity_rad_s <= 0:
        raise ValueError("开发速度必须为正有限弧度每秒")
    lock, paths = _sources()
    tree = ET.parse(paths["ctag_portable"]).getroot()
    changes, inertias = _normalize(tree, max_velocity_rad_s)
    tree.set("name", "ctag2f90_reference")
    meshes = {row["source_uri"]: paths[row["id"]] for row in lock["files"] if "source_uri" in row}
    copies = {}
    for mesh in tree.findall(".//mesh"):
        source = meshes[mesh.get("filename")]
        name = "mesh_" + _sha(source)[:16] + source.suffix.lower()
        copies[name] = source
        mesh.set("filename", "meshes/" + name)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "meshes").mkdir()
    for name, source in copies.items():
        shutil.copyfile(source, output / "meshes" / name)
    model_path = output / "model.urdf"
    ET.indent(tree)
    ET.ElementTree(tree).write(model_path, encoding="utf-8", xml_declaration=True)
    model = MimicModel(model_path)
    _write(output / "mechanics.json", {
        "mimic_map": model.affine,
        "joint_limit_derivations": changes,
        "source_inertias": inertias,
        "source_mass_kg": sum(item["mass_kg"] for item in inertias.values()),
        "physics_qualified": False,
        "hardware_qualified": False,
        "contact_qualified": False,
        "scope": "冻结仿射连杆参考，非闭环四连杆、实物开度或抓力模型",
    })
    _write(output / "provenance.json", {
        "package_source_lock_sha256": _SOURCE_LOCK_SHA256,
        "historical_source_lock_sha256": lock["historical_source_lock_sha256"],
        "source_files": {row["id"]: row["sha256"] for row in lock["files"]},
        "max_velocity_rad_s": max_velocity_rad_s,
        "velocity_authority": "调用者显式开发参数，不是厂家控制器参数",
        "transformations": ["沿用ctag命名", "展开仿射mimic", "派生从动限位", "显式开发速度", "网格闭包"],
    })
    artifacts = [{"path": str(p.relative_to(output)), "bytes": p.stat().st_size, "sha256": _sha(p)}
                 for p in sorted(output.rglob("*")) if p.is_file()]
    _write(output / "bundle.json", {"schema": "lab.family_sim_bundle/v0", "family": "ctag2f90",
                                    "artifacts": artifacts, "qualification": False})
    return model_path
