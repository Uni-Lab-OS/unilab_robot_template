"""Deterministic fixed-external-axis IK candidates and whole-sequence travel ranking.

This is a geometric candidate ranking. Only a collision-checked native trajectory
may be accepted; these endpoint costs are not a collision-free path certificate.
"""
import math
import numpy as np

def fixed_axis_candidates(kinematics,frames,axis_id,axis_values,seeds):
    rows=[]
    for axis in axis_values:
        external={axis_id:float(axis)}
        for seed in seeds:
            contact=kinematics.inverse(frames["grasp"],seed,external)
            if contact is None:continue
            approach=kinematics.inverse(frames["approach"],contact,external)
            if approach is None:continue
            entry=kinematics.inverse(frames["entry"],approach,external)
            if entry is None:continue
            if any(max(abs(a-b) for a,b in zip(previous,next_))>1.5
                   for previous,next_ in ((contact,approach),(approach,entry))):continue
            if any(abs(row["rail_m"]-axis)<1e-8 and np.linalg.norm(np.array(row["grasp"])-contact)<1e-3 for row in rows):continue
            rows.append(dict(rail_m=float(axis),grasp=list(contact),approach=list(approach),entry=list(entry)))
    return rows

def select_candidate_chain(layers,start_q,start_rail,*,rail_weight=6.,bounds=None,transition_allowed=None):
    """在显式候选图上选路；可由领域声明相邻动作的装夹兼容条件。"""
    if not layers or any(not layer for layer in layers):raise ValueError("Every action needs candidates")
    previous=[dict(q=tuple(start_q),rail=float(start_rail),cost=0.,path=[],rows=[],row=None)]
    for layer_index,layer in enumerate(layers):
        next_rows={}
        for index,original in enumerate(layer):
            for old in previous:
                if transition_allowed and not transition_allowed(layer_index,old["row"],original):continue
                row=nearest_legal_winding(original,old["q"],bounds) if bounds is not None else dict(original)
                local=2*sum(abs(a-b) for a,b in zip(row["entry"],row["approach"]))+2*sum(abs(a-b) for a,b in zip(row["approach"],row["grasp"]))
                cost=old["cost"]+sum(abs(a-b) for a,b in zip(old["q"],row["entry"]))+rail_weight*abs(old["rail"]-row["rail_m"])+local
                key=(index,tuple(round(q,8) for q in row["entry"]))
                if key not in next_rows or cost<next_rows[key]["cost"]:
                    next_rows[key]=dict(q=row["entry"],rail=row["rail_m"],cost=cost,
                        path=old["path"]+[index],rows=old["rows"]+[row],row=row)
        if not next_rows:raise ValueError("No custody-compatible candidate chain at layer "+str(layer_index))
        previous=list(next_rows.values())
    best=min(previous,key=lambda row:row["cost"])
    return {"indices":best["path"],"candidates":best["rows"],"cost":best["cost"],
            "metric":"arm L1 radians + rail_weight * metres","rail_weight":rail_weight,
            "qualification":"endpoint ranking only; native path validation required"}


def nearest_legal_winding(row,reference,bounds):
    """Choose equivalent whole-block joint winding before planning, inside URDF limits."""
    result=dict(row)
    values={name:np.asarray(row[name],dtype=float).copy() for name in ("entry","approach","grasp")}
    for axis,(low,high) in enumerate(bounds):
        minimum=min(q[axis] for q in values.values());maximum=max(q[axis] for q in values.values())
        choices=range(math.ceil((low-minimum)/(2*math.pi)),math.floor((high-maximum)/(2*math.pi))+1)
        choices=list(choices)
        if not choices:raise ValueError("No legal common block winding")
        turn=min(choices,key=lambda k:abs(values["entry"][axis]+2*math.pi*k-reference[axis]))
        for q in values.values():q[axis]+=2*math.pi*turn
    result.update({name:q.tolist() for name,q in values.items()})
    return result


def legal_winding_candidates(row,reference,bounds,*,max_candidates=16):
    """Enumerate bounded common-block windings in endpoint travel order.

    A nearest endpoint may force a long obstacle detour. Keep several equivalent
    legal windings for native path comparison. This does not modify an executed
    trajectory and does not certify any connecting path.
    """
    import heapq
    if isinstance(max_candidates,bool) or not isinstance(max_candidates,int) or max_candidates<1:
        raise ValueError("max_candidates must be a positive integer")
    reference=tuple(float(x) for x in reference);bounds=tuple(bounds)
    values={name:np.asarray(row[name],dtype=float) for name in ("entry","approach","grasp")}
    if not reference or len(bounds)!=len(reference) or any(
        q.shape!=(len(reference),) or not np.isfinite(q).all() for q in values.values()):
        raise ValueError("Candidate dimensions or values differ")
    if not all(math.isfinite(x) for x in reference):raise ValueError("Non-finite reference")
    shifts=[]
    for axis,(low,high) in enumerate(bounds):
        if not math.isfinite(low) or not math.isfinite(high) or low>high:raise ValueError("Invalid winding limits")
        minimum=min(q[axis] for q in values.values());maximum=max(q[axis] for q in values.values())
        options=list(range(math.ceil((low-minimum)/(2*math.pi)),math.floor((high-maximum)/(2*math.pi))+1))
        if not options:raise ValueError("No legal common block winding")
        shifts.append(sorted(options,key=lambda k:(abs(values["entry"][axis]+2*math.pi*k-reference[axis]),k)))
    def cost(indices):
        return sum(abs(values["entry"][axis]+2*math.pi*shifts[axis][index]-reference[axis])
                   for axis,index in enumerate(indices))
    origin=(0,)*len(reference);queue=[(cost(origin),origin)];seen={origin};result=[]
    while queue and len(result)<max_candidates:
        score,indices=heapq.heappop(queue)
        turns=[shifts[axis][index] for axis,index in enumerate(indices)]
        candidate=dict(row)
        candidate.update({name:(q+2*math.pi*np.asarray(turns)).tolist() for name,q in values.items()})
        candidate["winding_turns"]=turns;candidate["endpoint_travel_rad"]=score
        result.append(candidate)
        for axis,index in enumerate(indices):
            if index+1>=len(shifts[axis]):continue
            next_=list(indices);next_[axis]+=1;next_=tuple(next_)
            if next_ not in seen:
                seen.add(next_);heapq.heappush(queue,(cost(next_),next_))
    return result
