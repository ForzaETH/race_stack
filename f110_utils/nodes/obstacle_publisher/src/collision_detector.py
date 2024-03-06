#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from f110_msgs.msg import ObstacleArray, WpntArray
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

class OdCollisionDetector:
    def __init__(self):
        rospy.Subscriber("/perception/obstacles", ObstacleArray, self.od_cb)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.odom_cb)
        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        self.col_pub = rospy.Publisher("/opponent_collision", Bool, queue_size=10)
        self.opp_dist_pub = rospy.Publisher("/opponent_dist", Float32, queue_size=10)
        self.coll_marker_pub = rospy.Publisher("/collision_marker", MarkerArray, queue_size=10)

        self.first_visualization = True
        self.x_viz = 0.0
        self.y_viz = 0.0
        self.viz_q = quaternion_from_euler(0, 0, 0)
        self.viz_counter = 0
        self.obs_arr = ObstacleArray()
        self.car_odom = Odometry()
        self.rate = rospy.Rate(50) # 50Hz

    def od_cb(self, data: ObstacleArray):
        # Process the received data here
        self.obs_arr = data

    def odom_cb(self, data: Odometry):
        self.car_odom = data

    def glb_wpnts_cb(self, data:WpntArray):
        self.glb_waypoints = data.wpnts

    def loop(self):
        rospy.loginfo("[Dummy OD] Collision Detector wating...")
        rospy.wait_for_message("/car_state/odom_frenet", Odometry)
        rospy.wait_for_message("/global_waypoints", WpntArray)
        rospy.loginfo("[Dummy OD] Collision Detector ready!")
        while not rospy.is_shutdown():
            # sample the callback data
            obs_arr = self.obs_arr
            car_odom = self.car_odom

            # check for collision
            collision_bool, min_dist_s, min_dist_d = self.collision_check(obs_arr=obs_arr, car_odom=car_odom)

            # visualize collision or clear visualization
            if self.viz_counter > 0:
                self.viz_counter -= 1
                if self.viz_counter == 0:
                    self.viz_collision(clear=True)

            # publish collision
            col_msg = Bool()
            if collision_bool:
                #rospy.loginfo("[Dummy OD] Collision with Opponent detected! Distance: {} [m]".format(min_dist))
                col_msg.data = True
                self.viz_counter = int(1 / self.rate.sleep_dur.to_sec()) * 2 # How long to show the collision marker [s]
                self.viz_collision(dist_s=min_dist_s, dist_d=min_dist_d, clear=False)
            else:
                col_msg.data = False
            self.col_pub.publish(col_msg)

            # publish distance to closest obstacle
            dist_msg = Float32()
            dist_msg.data = np.sqrt(min_dist_s**2 + min_dist_d**2)
            self.opp_dist_pub.publish(dist_msg)

            self.rate.sleep()

    def collision_check(self, obs_arr: ObstacleArray, car_odom: Odometry):
        for obs in obs_arr.obstacles:
            car_s = car_odom.pose.pose.position.x
            car_d = car_odom.pose.pose.position.y

            od_s = obs.s_center
            od_d = obs.d_center
            
            if (od_s - car_s)%self.glb_waypoints[-2].s_m < 0.55 and abs(car_d - od_d) < 0.35:
                rospy.loginfo("[Dummy OD] Front Collision with Opponent detected! Position Frenet s: {} [m]".format(car_s))
                return True, (od_s - car_s)%self.glb_waypoints[-2].s_m, abs(car_d - od_d)
            if (car_s - od_s)%self.glb_waypoints[-2].s_m < 0.25 and abs(car_d - od_d) < 0.30:
                rospy.loginfo("[Dummy OD] Back Collision with Opponent detected! Position Frenet s: {} [m]".format(car_s))
                return True, (car_s - od_s)%self.glb_waypoints[-2].s_m, abs(car_d - od_d)
        # no collision
        return False, 100, 100

    def viz_collision(self, dist_s=0, dist_d=0, clear=False):
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_waypoints[len(self.glb_waypoints)//4].x_m
            y0 = self.glb_waypoints[len(self.glb_waypoints)//4].y_m
            x1 = self.glb_waypoints[len(self.glb_waypoints)//4+1].x_m
            y1 = self.glb_waypoints[len(self.glb_waypoints)//4+1].y_m
            # compute normal vector of 175% length of trackboundary but to the left of the trajectory
            xy_norm = -np.array([y1-y0, x0-x1]) / np.linalg.norm([y1-y0, x0-x1]) * 1.75 * self.glb_waypoints[len(self.glb_waypoints)//4].d_left
            # compute orientation of normal vector and convert to quaternion
            yaw = np.arctan2(xy_norm[1], xy_norm[0])
            self.viz_q = quaternion_from_euler(0, 0, yaw)
            # compute position of marker
            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        coll_mrk = MarkerArray()
        marker_text = Marker()
        marker_text.header.frame_id = "map"
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = marker_text.TEXT_VIEW_FACING
        marker_text.text = "COLLISION: dist_s :{:.1f}, dist_d :{:.1f}m".format(dist_s, dist_d) if not clear else ""
        marker_text.scale.z = 1.0
        marker_text.color.r = 1.0
        marker_text.color.g = 0.0
        marker_text.color.b = 0.0
        marker_text.color.a = 1.0
        marker_text.pose.orientation.x = self.viz_q[0]
        marker_text.pose.orientation.y = self.viz_q[1]
        marker_text.pose.orientation.z = self.viz_q[2]
        marker_text.pose.orientation.w = self.viz_q[3]
        marker_text.pose.position.x = self.x_viz
        marker_text.pose.position.y = self.y_viz
        marker_text.pose.position.z = 0.0
        marker_text.id = 0
        coll_mrk.markers.append(marker_text)

        self.coll_marker_pub.publish(coll_mrk)

if __name__ == '__main__':
    rospy.init_node('OdCollisionDetector_node')
    collision_detector = OdCollisionDetector()
    collision_detector.loop()