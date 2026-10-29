"""名义从动正解与完整实测关节正解分别计算，不推断抓取结果。"""
import math

from unilab_robot_contracts.urdf_reference import Model
from .mimic import mimic_map


class MimicModel(Model):
    """fk(q) consumes ALL measured joints; nominal_fk consumes independent q."""
    def __init__(self,path):
        super().__init__(path,allow_mimic=True)
        self.affine=mimic_map(self.xml)
        self.independent_names=[n for n in self.names if self.affine[n][0]==n]

    def expand(self, positions):
        if set(positions)!=set(self.independent_names): raise ValueError('independent joint set mismatch')
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in positions.values()): raise ValueError('invalid independent position')
        q=[positions[s]*m+b for s,m,b in (self.affine[n] for n in self.names)]
        self.fk(q) # Reject out-of-range derived states as well.
        return q

    def nominal_fk(self,positions):
        return self.fk(self.expand(positions))
