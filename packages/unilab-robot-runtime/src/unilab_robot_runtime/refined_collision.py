"""Conservative profile screening with lazy exact-source confirmation.

An outer hit alone is not a rejection: preserve openings by checking the pinned
exact source pair. Exact loading/cooking is charged to the first query that needs
it; all later poses reuse that geometry. Never silently fall back on failure.
"""
import time
from dataclasses import replace
from .fcl_collision import FCLSceneChecker

class RefinedCollisionChecker:
    def __init__(self, outer_objects, exact_objects):
        self.outer=FCLSceneChecker(outer_objects)
        if set(self.outer.ids)!=set(exact_objects):raise ValueError('Exact/outer object identities differ')
        self.exact_objects=exact_objects;self.pair_checkers={};self.geometry_cache={}
        self.index=self.outer.index;self.fcl=self.outer.fcl
    def check(self,poses,*,moving_ids,allowed_pairs=(),stop_at_first=True):
        started=time.perf_counter()
        broad=self.outer.check(poses,moving_ids=moving_ids,allowed_pairs=allowed_pairs,stop_at_first=False)
        hits=[];narrow=0;cooked=0
        for a,b in broad['pairs']:
            key=tuple(sorted((a,b)))
            if key not in self.pair_checkers:
                self.pair_checkers[key]=FCLSceneChecker([self.exact_objects[a],self.exact_objects[b]],geometry_cache=self.geometry_cache)
                cooked+=1
            checker=self.pair_checkers[key]
            result=checker.check({a:self.outer.poses[a],b:self.outer.poses[b]},moving_ids=[a,b])
            narrow+=result['narrow_phase_calls']
            if result['collision']:
                hits.append((a,b))
                if stop_at_first:break
        return dict(broad,collision=bool(hits),pairs=hits,outer_hit_pairs=len(broad['pairs']),
                    exact_narrow_phase_calls=narrow,new_exact_pair_caches=cooked,
                    cached_exact_pair_count=len(self.pair_checkers),wall_s=time.perf_counter()-started,
                    coverage='conservative_outer_then_exact_source; moving_scene_and_moving_moving')
