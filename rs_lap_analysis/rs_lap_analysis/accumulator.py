from dataclasses import dataclass
import numpy as np

from rs_msgs.msg import SectorData

@dataclass
class Accumulator:
    sector_idx: int
    start_s: float
    end_s: float
    t0: float                       
    n: int = 0
    sum_abs_d: float = 0.0
    max_abs_d: float = 0.0
    min_boundary_dist: float = np.inf
    sum_vs: float = 0.0
    max_vs: float = -np.inf

    def add(self, d: float, vs: float, boundary_dist: float):
        self.n += 1
        self.sum_abs_d += abs(d)
        self.max_abs_d = max(self.max_abs_d, abs(d))
        self.min_boundary_dist = min(self.min_boundary_dist, boundary_dist)
        self.sum_vs += vs
        self.max_vs = max(self.max_vs, vs)

    def to_msg(self, t_end: float, lap_count: int) -> SectorData:
        msg = SectorData()
        msg.lap_count = lap_count
        msg.sector_idx = self.sector_idx
        msg.start_s, msg.end_s = self.start_s, self.end_s
        msg.time = t_end - self.t0
        msg.mean_abs_d = self.sum_abs_d / self.n
        msg.max_abs_d = self.max_abs_d
        msg.min_boundary_dist = self.min_boundary_dist
        msg.mean_vs = self.sum_vs / self.n
        msg.max_vs = self.max_vs
        return msg
