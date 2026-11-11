"""资产子树的显式装配适配；不包含设备名称、安装位置或规划算法。"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


def mesh_path(filename, root):
    if filename.startswith('file:'):
        value=unquote(urlparse(filename).path)
        if len(value)>2 and value[0]=='/' and value[2]==':':value=value[1:]
        return Path(value).resolve(strict=True)
    if '://' in filename:raise ValueError('Unresolved mesh URI: '+filename)
    return (Path(root)/filename).resolve(strict=True)


@dataclass(frozen=True)
class AssetSubtree:
    render_urdf: str
    source_digest: str
    qualified_joint_names: tuple[str,...]
    sources: dict
    mount_link: str = ''


def asset_subtree(path, *, root_link, prefix, descendants=True, collision_from_visual=False, root_visual_indices=None):
    """保留资产的局部几何/关节；视觉派生碰撞必须由调用方显式选择并记录。"""
    path=Path(path);source=ET.parse(path).getroot()
    links={x.get('name'):x for x in source.findall('link')}
    if root_link not in links:raise ValueError('Missing subtree root: '+root_link)
    if root_visual_indices is not None:
        indices=tuple(root_visual_indices)
        count=len(links[root_link].findall('visual'))
        if not collision_from_visual:
            raise ValueError('Geometry subset requires explicit visual-derived collision')
        if not indices or len(set(indices))!=len(indices) or any(type(i) is not int or not 0<=i<count for i in indices):
            raise ValueError('Invalid root visual selection')
    chosen={root_link};joints=[]
    if descendants:
        while True:
            ready=[j for j in source.findall('joint') if j.find('parent').get('link') in chosen and j.find('child').get('link') not in chosen]
            if not ready:break
            joints.extend(ready);chosen.update(j.find('child').get('link') for j in ready)
    tree=ET.Element('robot',name=prefix+'asset')
    sources={str(path):hashlib.sha256(path.read_bytes()).hexdigest()}
    for name in sorted(chosen):
        link=deepcopy(links[name]);link.set('name',prefix+name)
        if name==root_link and root_visual_indices is not None:
            for index, visual in enumerate(link.findall('visual')):
                if index not in indices:link.remove(visual)
            # 分件几何不能继承整个原 link 的质量；此子树只供几何/运动学使用。
            for inertial in link.findall('inertial'):link.remove(inertial)
        if collision_from_visual:
            for old in link.findall('collision'):link.remove(old)
            for visual in link.findall('visual'):
                collision=deepcopy(visual);collision.tag='collision'
                for material in collision.findall('material'):collision.remove(material)
                link.append(collision)
        for mesh in link.findall('.//mesh'):
            resolved=mesh_path(mesh.get('filename'),path.parent)
            sources[str(resolved)]=hashlib.sha256(resolved.read_bytes()).hexdigest()
            mesh.set('filename',resolved.as_uri())
        tree.append(link)
    moving=[]
    for item in joints:
        joint=deepcopy(item);joint.set('name',prefix+joint.get('name'))
        for tag in ('parent','child'):
            element=joint.find(tag);element.set('link',prefix+element.get('link'))
        mimic=joint.find('mimic')
        if mimic is not None:mimic.set('joint',prefix+mimic.get('joint'))
        if joint.get('type')!='fixed':moving.append(joint.get('name'))
        tree.append(joint)
    return AssetSubtree(ET.tostring(tree,encoding='unicode'),sources[str(path)],tuple(moving),
        dict(files=sources,collision_source='exact_visual_geometry' if collision_from_visual else 'authored_collision',
             root_visual_indices=None if root_visual_indices is None else list(indices),
             inertial_source='omitted_for_geometry_subset' if root_visual_indices is not None else 'authored'))


def bind_mesh_identity(model, *, root):
    """对当前可读文件做内容绑定；迁移安装目录不会改变模型身份。"""
    from dataclasses import replace
    filenames={m.get('filename') for m in ET.fromstring(model.urdf_xml).findall('.//mesh')}
    hashes={name:hashlib.sha256(mesh_path(name,root).read_bytes()).hexdigest() for name in filenames}
    if model.mesh_sha256 and dict(model.mesh_sha256)!=hashes:
        raise ValueError('URDF mesh content differs from bound model')
    return replace(model,mesh_sha256=hashes)


def collision_objects(urdf_xml, frames, *, root):
    """读取 URDF 明确声明的碰撞几何，供共享 FCL 查询；不从 visual 推断。"""
    from .urdf_tree import origin_matrix
    from .moveit_planner import _read_stl
    from unilab_robot_contracts.planning import CollisionGeometry, CollisionObject
    objects=[]
    for link in ET.fromstring(urdf_xml).findall("link"):
        geometries=[];poses=[]
        for collider in link.findall("collision"):
            geometry=collider.find("geometry")
            mesh=geometry.find("mesh");box=geometry.find("box");cylinder=geometry.find("cylinder")
            if mesh is not None:
                vertices,triangles=_read_stl(mesh_path(mesh.get("filename"),root))
                shape=CollisionGeometry("mesh",vertices=vertices,triangles=triangles,
                    mesh_scale=tuple(float(x) for x in mesh.get("scale","1 1 1").split()))
            elif box is not None:
                shape=CollisionGeometry("box",size_m=tuple(float(x) for x in box.get("size").split()))
            elif cylinder is not None:
                shape=CollisionGeometry("cylinder",radius_m=float(cylinder.get("radius")),length_m=float(cylinder.get("length")))
            else:raise ValueError("Unsupported authored URDF collision geometry")
            geometries.append(shape);poses.append(tuple(map(tuple,origin_matrix(collider.find("origin")))))
        if geometries:
            name=link.get("name")
            objects.append(CollisionObject(name,tuple(map(tuple,frames[name])),tuple(geometries),geometry_poses=tuple(poses)))
    return tuple(objects)
