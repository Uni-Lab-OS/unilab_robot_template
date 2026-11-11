"""Reusable FCL snapshot checker with explicit moving-vs-scene collision pairs.

Optional python-fcl backend for saved-path verification; no planner, inventory,
hardware, or simulation clock. Objects retain their existing public identities.
Body AABBs prune candidates, geometry AABBs prune shapes, FCL tests exact survivors.
Only exact caller-specified allowed pairs are omitted. Sibling attachments are NOT
implicitly exempt. Access to an instance must be serialized by its owner.
"""
from __future__ import annotations
from dataclasses import dataclass
import hashlib,math,time
import numpy as np


def _rows(value,code):
    if hasattr(value,'data') and getattr(value,'code',None)==code:
        return np.frombuffer(value.data,dtype='<f8' if code=='d' else '<i8').reshape(-1,3)
    return np.asarray(value,dtype=float if code=='d' else np.int64)


def _pose(value):
    matrix=np.asarray(value,dtype=float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1],atol=1e-10,rtol=0):
        raise ValueError('Expected finite homogeneous pose')
    r=matrix[:3,:3]
    if np.max(abs(r.T@r-np.eye(3)))>1e-6 or np.linalg.det(r)<=0:
        raise ValueError('Expected rigid pose')
    # Match native rigid-transform consumers for float32 measured-pose roundoff.
    if np.max(abs(r.T@r-np.eye(3)))>1e-12:
        u,_,vt=np.linalg.svd(r);matrix=matrix.copy();matrix[:3,:3]=u@vt
    return matrix



@dataclass
class _Shape:
    native: object
    local_pose: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    revision: int = -1


class FCLSceneChecker:
    def __init__(self,objects,*,geometry_cache=None):
        import fcl
        objects=tuple(objects)
        self.fcl=fcl;self.ids=tuple(o.object_id for o in objects)
        if len(set(self.ids))!=len(self.ids) or not self.ids:raise ValueError('Nonempty unique object identities required')
        self.index={name:i for i,name in enumerate(self.ids)};self.shapes=[];self.poses={}
        self.lower=np.zeros((len(self.ids),3));self.upper=self.lower.copy()
        self._geometry_cache={} if geometry_cache is None else geometry_cache;self._request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=False)
        self._policy_key=None;self._pairs=None
        self.shape_lower={};self.shape_upper={}
        self.centers=[];self.extents=[];self.local_rotations=[];self.local_translations=[]
        self.revisions=np.zeros(len(self.ids),dtype=np.int64)
        for obj in objects:
            entries=[]
            for i,g in enumerate(obj.geometry):
                native,low,high=self._geometry(g)
                local=_pose(obj.geometry_poses[i]) if obj.geometry_poses else np.eye(4)
                entries.append(_Shape(fcl.CollisionObject(native),local,low,high))
            self.shapes.append(entries)
            self.centers.append(np.array([(s.lower+s.upper)*.5 for s in entries]))
            self.extents.append(np.array([(s.upper-s.lower)*.5 for s in entries]))
            self.local_rotations.append(np.array([s.local_pose[:3,:3] for s in entries]))
            self.local_translations.append(np.array([s.local_pose[:3,3] for s in entries]))
            self._update(obj.object_id,obj.world_pose)

    def _geometry(self,g):
        fcl=self.fcl
        if g.kind=='mesh':
            if g.vertices is None or g.triangles is None:raise ValueError('In-memory mesh required')
            v=_rows(g.vertices,'d')*np.asarray(g.mesh_scale);t=_rows(g.triangles,'q')
            if not len(v) or not len(t) or not np.isfinite(v).all() or t.min()<0 or t.max()>=len(v):raise ValueError('Invalid triangle mesh')
            key=hashlib.sha256(np.asarray([len(v),len(t)],dtype='<i8').tobytes()+v.tobytes()+t.tobytes()).hexdigest()
            if key in self._geometry_cache:return self._geometry_cache[key]
            model=fcl.BVHModel();model.beginModel(len(v),len(t));model.addSubModel(v,t);model.endModel()
            value=(model,v.min(0),v.max(0));self._geometry_cache[key]=value;return value
        if g.kind=='box':
            size=np.asarray(g.size_m);return fcl.Box(*size),-size*.5,size*.5
        if g.kind=='cylinder':
            ext=np.array([g.radius_m,g.radius_m,g.length_m*.5]);return fcl.Cylinder(g.radius_m,g.length_m),-ext,ext
        raise ValueError('Unsupported FCL geometry: '+g.kind)

    def _update(self,identity,value):
        m=_pose(value)
        if identity in self.poses and np.array_equal(m,self.poses[identity]):return
        i=self.index[identity]
        rotations=np.einsum('ij,njk->nik',m[:3,:3],self.local_rotations[i])
        translations=self.local_translations[i]@m[:3,:3].T+m[:3,3]
        centers=np.einsum('nij,nj->ni',rotations,self.centers[i])+translations
        extents=np.einsum('nij,nj->ni',np.abs(rotations),self.extents[i])+1e-10
        self.shape_lower[i]=centers-extents;self.shape_upper[i]=centers+extents
        self.lower[i]=self.shape_lower[i].min(axis=0)
        self.upper[i]=self.shape_upper[i].max(axis=0)
        self.poses[identity]=m.copy();self.revisions[i]+=1

    def _native_at(self,index,shape_index):
        # Meshes need native pose updates only after both broad phases pass.
        # Thousands of distant pieces in a carried tray stay entirely in NumPy.
        shape=self.shapes[index][shape_index]
        if shape.revision!=self.revisions[index]:
            world=self.poses[self.ids[index]]@shape.local_pose
            shape.native.setTransform(self.fcl.Transform(world[:3,:3],world[:3,3]))
            shape.revision=self.revisions[index]
        return shape.native

    def check(self,poses,*,moving_ids,allowed_pairs=(),stop_at_first=True):
        """Every selected moving object needs a fresh pose; static updates are explicit.

        Pair omission is limited to no-moving-member pairs and the exact allowed
        pair set. Unknown ids, self pairs, duplicates and missing poses fail closed.
        Does not interpolate between the supplied states.
        """
        started=time.perf_counter()
        if isinstance(moving_ids,(str,bytes)):raise ValueError('Moving identities must be a sequence of ids')
        moving=tuple(moving_ids)
        if not moving or len(set(moving))!=len(moving) or set(moving)-self.index.keys():raise ValueError('Invalid moving identities')
        if set(poses)-self.index.keys() or set(moving)-set(poses):raise ValueError('Missing/unknown object poses')
        exclusions=set()
        for pair in allowed_pairs:
            if not isinstance(pair,(tuple,list)) or len(pair)!=2 or pair[0]==pair[1] or set(pair)-self.index.keys():raise ValueError('Invalid allowed collision pair')
            canonical=tuple(sorted(pair))
            if canonical in exclusions:raise ValueError('Duplicate allowed collision pair')
            exclusions.add(canonical)
        key=(tuple(sorted(moving)),tuple(sorted(exclusions)))
        if key!=self._policy_key:
            moving_indices=sorted(self.index[x] for x in moving);members=set(moving_indices)
            pairs=[(i,j) for i in moving_indices for j in range(len(self.ids)) if i!=j and (j not in members or i<j)
                   and tuple(sorted((self.ids[i],self.ids[j]))) not in exclusions]
            self._pairs=np.asarray(pairs,dtype=np.int64).reshape(-1,2);self._policy_key=key
        for identity,pose in poses.items():self._update(identity,pose)
        pairs=self._pairs
        if len(pairs):
            a,b=pairs.T;mask=np.all(self.lower[a]<=self.upper[b],axis=1)&np.all(self.lower[b]<=self.upper[a],axis=1);candidates=pairs[mask]
        else:candidates=pairs
        hits=[];narrow=0
        for i,j in candidates:
            found=False
            # Vectorize shape broad phase; avoid quadratic Python loops on CAD
            # compounds (e.g. thousands of triangles/convex pieces in a tray).
            left,right=(i,j) if len(self.shapes[i])<=len(self.shapes[j]) else (j,i)
            for ai in range(len(self.shapes[left])):
                overlap=np.all(self.shape_lower[left][ai]<=self.shape_upper[right],axis=1)&np.all(self.shape_lower[right]<=self.shape_upper[left][ai],axis=1)
                for k in np.flatnonzero(overlap):
                    narrow+=1;result=self.fcl.CollisionResult()
                    self.fcl.collide(self._native_at(left,ai),self._native_at(right,k),self._request,result)
                    if result.is_collision:found=True;break
                if found:break
            if found:
                hits.append((self.ids[i],self.ids[j]))
                if stop_at_first:break
        return dict(collision=bool(hits),pairs=hits,eligible_pairs=len(pairs),aabb_candidates=len(candidates),narrow_phase_calls=narrow,wall_s=time.perf_counter()-started,
                    coverage='moving_scene_and_moving_moving; explicit pair exclusions',continuous_collision_proof=False)


def joint_samples(points,*,max_step_rad=.002,max_samples=1000000):
    """Preserve all saved internal points; subdivide each edge with a fixed bound."""
    values=np.asarray(points,dtype=float)
    if values.ndim!=2 or not len(values) or not values.shape[1] or not np.isfinite(values).all():raise ValueError('Invalid joint path')
    if not math.isfinite(max_step_rad) or max_step_rad<=0 or type(max_samples)is not int or max_samples<1:raise ValueError('Invalid sampling bounds')
    with np.errstate(over='ignore',invalid='ignore'):
        counts=np.maximum(1,np.ceil(np.max(abs(np.diff(values,axis=0)),axis=1)/max_step_rad))
    if not np.isfinite(counts).all() or np.any(counts>max_samples) or 1+sum(counts)>max_samples:
        raise ValueError('Sampling budget exceeded; path not checked')
    counts=counts.astype(int)
    yield values[0].tolist()
    for left,right,count in zip(values[:-1],values[1:],counts):
        for step in range(1,count+1):yield (left+(right-left)*(step/count)).tolist()
