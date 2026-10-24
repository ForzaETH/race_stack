from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np

from rs_msgs.msg import WpntArray
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

@dataclass
class GlobalPlan:
    map_info: str
    est_lap_time: float
    centerline: WpntArray
    raceline: WpntArray
    shortest_path: WpntArray
    # Track walls [[x, y], ...] in meters, right/left in driving direction; None for plans saved before they were stored
    bound_right: np.ndarray | None = None
    bound_left: np.ndarray | None = None

    def save(self, map_dir: Path) -> None:
        d = {'map_info_str': {'data': self.map_info},
             'est_lap_time': {'data': self.est_lap_time},
             'centerline_waypoints': message_to_ordereddict(self.centerline),
             'global_traj_wpnts_iqp': message_to_ordereddict(self.raceline),
             'global_traj_wpnts_sp': message_to_ordereddict(self.shortest_path)}
        if self.bound_right is not None and self.bound_left is not None:
            d['track_bound_right'] = np.asarray(self.bound_right).tolist()
            d['track_bound_left'] = np.asarray(self.bound_left).tolist()
        
        (map_dir / 'global_waypoints.json').write_text(json.dumps(d))
    
    @classmethod
    def load(cls, map_dir: Path) -> 'GlobalPlan':
        d = json.loads((map_dir / 'global_waypoints.json').read_text())
        
        def wpnts(key):
            msg = WpntArray(); set_message_fields(msg, d[key])
            return msg
        
        return cls(map_info=d['map_info_str']['data'], 
                   est_lap_time=d['est_lap_time']['data'],
                   centerline=wpnts('centerline_waypoints'), 
                   raceline=wpnts('global_traj_wpnts_iqp'),
                   shortest_path=wpnts('global_traj_wpnts_sp'),
                   bound_right=np.array(d['track_bound_right']) if 'track_bound_right' in d else None,
                   bound_left=np.array(d['track_bound_left']) if 'track_bound_left' in d else None)

