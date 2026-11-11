"""Compiled URDF chain for candidate IK and independent Cartesian path measurements."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

class UrdfKinematics:
    def __init__(self,robot):
        self.robot=robot
        tree=ET.fromstring(robot.urdf_xml)
        joints={j.find("child").get("link"):j for j in tree.findall("joint")}
        chain=[];link=robot.flange_link
        while link!=robot.base_frame:
            if link not in joints:raise ValueError("URDF flange is disconnected from planning frame")
            j=joints[link];origin=np.eye(4);o=j.find("origin")
            if o is not None:
                origin[:3,3]=[float(x) for x in o.get("xyz","0 0 0").split()]
                origin[:3,:3]=Rotation.from_euler("xyz",[float(x) for x in o.get("rpy","0 0 0").split()]).as_matrix()
            a=j.find("axis");axis=np.array([float(x) for x in (a.get("xyz") if a is not None else "1 0 0").split()])
            axis=axis/np.linalg.norm(axis)
            chain.append((j.get("name"),j.get("type"),origin,axis))
            link=j.find("parent").get("link")
        self.chain=tuple(reversed(chain))
        self.bounds=np.asarray(robot.joint_limits,dtype=float).T

    def forward(self,q,external):
        positions=dict(external);positions.update(zip(self.robot.joint_names,q))
        result=np.eye(4)
        for name,kind,origin,axis in self.chain:
            result=result@origin
            if kind=="fixed":continue
            if name not in positions:raise ValueError("Missing external axis state: "+name)
            value=float(positions[name]);move=np.eye(4)
            if kind in ("revolute","continuous"):move[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
            elif kind=="prismatic":move[:3,3]=axis*value
            else:raise ValueError("Unsupported joint type: "+kind)
            result=result@move
        return result

    def inverse(self,target,seed,external,*,position_tolerance_m=1e-5,rotation_tolerance_rad=1e-4):
        target=np.asarray(target);seed=np.clip(np.asarray(seed),self.bounds[0]+1e-9,self.bounds[1]-1e-9)
        def residual(q):
            pose=self.forward(q,external)
            return np.r_[pose[:3,3]-target[:3,3],
                Rotation.from_matrix(target[:3,:3].T@pose[:3,:3]).as_rotvec()]
        fit=least_squares(residual,seed,bounds=self.bounds,max_nfev=160,
                          ftol=1e-10,xtol=1e-10,gtol=1e-10)
        error=residual(fit.x)
        if (np.linalg.norm(seed)<1e-8 and np.all(self.bounds[0]<=0) and np.all(self.bounds[1]>=0) and
                (np.linalg.norm(error[:3])>position_tolerance_m or
                 np.linalg.norm(error[3:])>rotation_tolerance_rad)):
            # A near-zero TRF seed can produce an effectively zero initial
            # trust region. Retry the bounded solve, preserving all residual gates.
            fit=least_squares(residual,np.zeros_like(seed),bounds=self.bounds,method="dogbox",
                              max_nfev=160,ftol=1e-10,xtol=1e-10,gtol=1e-10)
            error=residual(fit.x)
        if np.linalg.norm(error[:3])>position_tolerance_m or np.linalg.norm(error[3:])>rotation_tolerance_rad:
            return None
        # Choose the equivalent winding nearest the seed, inside physical URDF bounds.
        q=fit.x.copy()
        for i in range(len(q)):
            options=[q[i]+2*np.pi*k for k in range(-2,3)
                     if self.bounds[0,i]<=q[i]+2*np.pi*k<=self.bounds[1,i]]
            q[i]=min(options,key=lambda x:abs(x-seed[i]))
        return tuple(float(v) for v in q)
