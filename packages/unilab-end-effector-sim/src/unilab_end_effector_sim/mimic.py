"""展开URDF仿射mimic关系；不发布关节命令，不生成实测反馈。"""
import math


def mimic_map(tree):
    moving={j.get('name'):j for j in tree.findall('joint') if j.get('type')!='fixed'}
    result={}
    def visit(name, stack):
        if name in stack: raise ValueError('mimic cycle')
        if name not in moving: raise ValueError('unknown mimic joint')
        if name in result: return result[name]
        tag=moving[name].find('mimic')
        if tag is None: value=(name,1.,0.)
        else:
            m=float(tag.get('multiplier','1'));b=float(tag.get('offset','0'))
            if not all(math.isfinite(x) for x in (m,b)) or m==0: raise ValueError('invalid mimic coefficient')
            source,scale,offset=visit(tag.get('joint'),stack+[name])
            value=(source,m*scale,m*offset+b)
        result[name]=value
        return value
    for name in moving: visit(name,[])
    return result
