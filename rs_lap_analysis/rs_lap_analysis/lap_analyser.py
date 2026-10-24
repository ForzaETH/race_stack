#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.node import Node
from rs_msgs.msg import FrenetState, LapData, WpntArray, SectorArray, SectorData

from .accumulator import Accumulator

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class LapAnalyser(Node):
    def __init__(self):
        super().__init__('lap_analyser')
                
        # Get sectors
        self.sector_starts = self.sector_ends = np.array([])
        self.sectors_sub = self.create_subscription(SectorArray, "/sectors", self.sectors_cb, LATCHED)
        
        # Publish data per sector
        self.sector_pub = self.create_publisher(SectorData, "~/sector_data", 10)
        
        # Publish data per lap
        self.lap_pub = self.create_publisher(LapData, "~/lap_data", LATCHED)
        
        # Subscribe to the global waypoints
        self.wpnt_s = self.wpnt_d_left = self.wpnt_d_right = np.array([])
        self.track_length = 0.0
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, LATCHED)
        
        # Update metrics at each state update
        self.frenet_sub = self.create_subscription(FrenetState, "/car_state/frenet", self.frenet_state_cb, 10)
        
        # Initialize the accumulator
        self.acc = None
        self.last_sector = None
        self.last_s = None

        # Lap state: lap 0 is the partial lap the node starts in and is never published
        self.lap_count = 0
        self.lap_sectors = []        # SectorData of the current lap
        self.lap_sample_counts = []  # samples per sector, to weight the lap means
        
    def waypoints_cb(self, msg: WpntArray):
        self.wpnt_s = np.array([w.s_m for w in msg.wpnts])
        self.wpnt_d_left = np.array([w.d_left for w in msg.wpnts])
        self.wpnt_d_right = np.array([w.d_right for w in msg.wpnts])
        self.track_length = float(self.wpnt_s[-1])

    def sectors_cb(self, msg: SectorArray):
        self.sector_starts = np.array([s.start_s for s in msg.sectors])
        self.sector_ends = np.array([s.end_s for s in msg.sectors])
    
    def get_curr_sector(self, curr_s):
        return int(np.searchsorted(self.sector_starts, curr_s, side='right') - 1)
    
    def dist_to_boundary(self, curr_s, curr_d):
        nearest_raceline_idx = min(int(np.searchsorted(self.wpnt_s, curr_s)), len(self.wpnt_s) - 1)
        return min(self.wpnt_d_left[nearest_raceline_idx] - curr_d,
                   self.wpnt_d_right[nearest_raceline_idx] + curr_d)

    def frenet_state_cb(self, msg: FrenetState):

        # Skip, still waiting for global waypoints or sectors
        if self.wpnt_s.size == 0 or self.sector_starts.size == 0:
            return

        curr_s = msg.s
        curr_d = msg.d
        curr_sector = self.get_curr_sector(curr_s)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Check if the lap is finished or the sector changed
        finished_lap = self.last_s is not None and self.last_s > self.track_length - 1.0 and curr_s < 1.0
        changed_sector = self.last_sector is not None and self.last_sector != curr_sector

        if finished_lap or changed_sector:
            # Publish the sector just left, no accumulator before the first sector change
            if self.acc is not None:
                sector_msg = self.acc.to_msg(t, self.lap_count)
                sector_msg.header.stamp = msg.header.stamp
                self.sector_pub.publish(sector_msg)
                self.lap_sectors.append(sector_msg)
                self.lap_sample_counts.append(self.acc.n)

            # Publish the lap, only if every sector was driven
            if finished_lap:
                if len(self.lap_sectors) == len(self.sector_starts):
                    self.publish_lap(msg.header.stamp)
                self.lap_count += 1
                self.lap_sectors = []
                self.lap_sample_counts = []

            # Initialize the accumulator of the new sector
            self.acc = Accumulator(sector_idx=curr_sector,
                                   start_s=self.sector_starts[curr_sector],
                                   end_s=self.sector_ends[curr_sector],
                                   t0=t)

        # Update accumulator
        if self.acc is not None:
            self.acc.add(curr_d, msg.vs, self.dist_to_boundary(curr_s, curr_d))

        # Track the last values
        self.last_s = curr_s
        self.last_sector = curr_sector

    def publish_lap(self, stamp):
        n = np.array(self.lap_sample_counts)

        lap_msg = LapData()
        lap_msg.header.stamp = stamp
        lap_msg.lap_count = self.lap_count
        lap_msg.lap_time = sum(s.time for s in self.lap_sectors)

        # Means weighted by the number of samples in each sector
        lap_msg.mean_abs_d = float(np.dot([s.mean_abs_d for s in self.lap_sectors], n) / n.sum())
        lap_msg.mean_vs = float(np.dot([s.mean_vs for s in self.lap_sectors], n) / n.sum())

        lap_msg.max_abs_d = max(s.max_abs_d for s in self.lap_sectors)
        lap_msg.max_vs = max(s.max_vs for s in self.lap_sectors)
        lap_msg.min_boundary_dist = min(s.min_boundary_dist for s in self.lap_sectors)
        lap_msg.sectors = self.lap_sectors

        self.lap_pub.publish(lap_msg)

def main():
    rclpy.init()
    lap_analyser = LapAnalyser()
    rclpy.spin(lap_analyser)
    lap_analyser.destroy_node()
    rclpy.shutdown()
