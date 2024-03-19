#!/usr/bin/env python3

import rospy
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, LapData
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from numpy import random
import numpy as np
import math

class ObstaclePublisher:
  def __init__(self):
    rospy.Subscriber('/car_state/odom_frenet', Odometry, self.odom_cb)
    rospy.Subscriber('/global_waypoints', WpntArray, self.global_trajectory_cb)
    self.obstacle_pub = rospy.Publisher('/obstacles', ObstacleArray, queue_size=10) 
    
    self.n_sectors = rospy.get_param('~n_obstacles') + 1
    self.publish_at_lookahead = rospy.get_param('~publish_at_lookahead')
    self.lookahead_distance = rospy.get_param('~lookahead_distance')
    self.obstacle_width = rospy.get_param('~obstacle_width')
    self.obstacle_length = rospy.get_param('~obstacle_length')
    self.obstacle_max_d_from_traj = rospy.get_param('~obstacle_max_d_from_traj')
    seed = rospy.get_param('~rnd_seed')
    
    self.obstacle_array = []
    self.has_traj = False
    self.has_odom = False
    self.gen = random.default_rng(seed)
    self.s = 0.0
    # loop while not shutdown
    node_rate = rospy.Rate(25) # 25hz
    # start_time = rospy.Time.now()
    rospy.sleep(0.1)
    while (not self.has_traj or not self.has_odom) and not rospy.is_shutdown():
      node_rate.sleep()
    
    self.update_obstacles()

    while not rospy.is_shutdown():
      self.publish_obstacles()
      node_rate.sleep()

  def update_obstacles(self):
    if self.has_traj:
      self.obstacle_array.clear()
      self.final_s = self.gb_wpnts[-1].s_m
      s_spacing = self.final_s / self.n_sectors
      margin = max(0.5, self.obstacle_length)
      for sec in range(self.n_sectors):
        s_start = sec * s_spacing
        s_end = s_start + s_spacing - margin
        ob = self.generate_random_obstacle(sec, s_start, s_end)
        self.obstacle_array.append(ob)

  def generate_random_obstacle(self, id, s_start, s_end):
    ob = Obstacle()
    ob.id = id
    # random s within s_start and s_end
    p1 = self.gen.random()
    ob.s_start = s_start + (s_end - s_start) * p1
    ob.s_end = ob.s_start + self.obstacle_length

    # get track bounds at s_start
    wpt_id = self.get_closest_point_on_traj(ob.s_start)
    track_right = -min(self.gb_wpnts[wpt_id].d_right, self.obstacle_max_d_from_traj) # negative
    track_left = min(self.gb_wpnts[wpt_id].d_left - self.obstacle_width, self.obstacle_max_d_from_traj)

    p2 = self.gen.random()
    ob.d_right = track_right + (track_left - track_right) * p2
    ob.d_left = ob.d_right + self.obstacle_width
    ob.is_actually_a_gap = False

    return ob

  def get_closest_point_on_traj(self, s):
    min_d = 1000
    id = 0
    for wpt in self.gb_wpnts:
      d = (wpt.s_m-s)**2
      if d < min_d:
        min_d = d
        id = wpt.id
    return id

  def publish_obstacles(self):
    obstacle_msg = ObstacleArray()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "frenet"
    if self.publish_at_lookahead:
      for ob in self.obstacle_array:
        # too lazy to handle wrapping 
        dist = math.fmod(self.s + self.lookahead_distance, self.final_s) - ob.s_start
        if dist > 0 and dist < (self.lookahead_distance + 1):
          obstacle_msg.obstacles.append(ob)
    else:
      obstacle_msg.obstacles = self.obstacle_array
    self.obstacle_pub.publish(obstacle_msg)

  def global_trajectory_cb(self, msg):
    self.has_traj = True
    self.gb_wpnts = msg.wpnts

  def odom_cb(self, msg):
    self.has_odom = True
    self.s = msg.pose.pose.position.x


if __name__ == '__main__':
  rospy.init_node('random_obstacle_publisher')
  DummyObstacles = ObstaclePublisher()