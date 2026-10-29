"""CTAG2F90 冻结模型的指垫几何参考；不证明接触、抓力或实物开度。"""
from __future__ import annotations

import itertools
import struct


def pad_bounds(model):
    result={}
    for side in ('Left','Right'):
        name='ctag_'+side+'_Pad_Link'
        link=model.xml.find("link[@name='%s']"%name)
        mesh=link.find('visual/geometry/mesh');raw=(model.path.parent/mesh.get('filename')).read_bytes()
        n=struct.unpack_from('<I',raw,80)[0]
        if len(raw)!=84+n*50:raise ValueError('expected binary STL pads')
        points=[struct.unpack_from('<fff',raw,84+i*50+12+j*12) for i in range(n) for j in range(3)]
        result[name]=list(itertools.product(*[(min(p[k] for p in points),max(p[k] for p in points)) for k in range(3)]))
    return result


def projected_pad_gap(bounds, palm_relative_poses):
    """Conservative separation of pad AABBs projected on palm X; not full collision."""
    extents={}
    for name,points in bounds.items():
        t=palm_relative_poses[name]
        x=[sum(t[0][k]*p[k] for k in range(3))+t[0][3] for p in points]
        extents[name]=(min(x),max(x))
    return extents['ctag_Left_Pad_Link'][0]-extents['ctag_Right_Pad_Link'][1]
