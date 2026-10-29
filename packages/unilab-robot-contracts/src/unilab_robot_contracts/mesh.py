"""不可变的紧凑三列数据；保留标准 Python 数值迭代语义。"""
from dataclasses import dataclass
import struct
from collections.abc import Sequence

@dataclass(frozen=True,slots=True)
class PackedMeshRows(Sequence):
    data: bytes
    code: str
    def __post_init__(self):
        if self.code not in ("d","q") or not isinstance(self.data,bytes) or len(self.data)%24:
            raise ValueError("Packed mesh requires immutable 3 x float64/int64 rows")
    def __len__(self):return len(self.data)//24
    def __iter__(self):return struct.iter_unpack("<3"+self.code,self.data)
    def __getitem__(self,index):
        if isinstance(index,slice):return tuple(self)[index]
        if index<0:index+=len(self)
        if not 0<=index<len(self):raise IndexError(index)
        return struct.unpack_from("<3"+self.code,self.data,index*24)
