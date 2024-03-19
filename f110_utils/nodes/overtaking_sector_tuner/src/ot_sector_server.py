#!/usr/bin/env python3
import rospy
import numpy as np
from f110_msgs.msg import WpntArray
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from overtaking_sector_tuner.cfg import ot_dyn_sect_tunerConfig
from visualization_msgs.msg import Marker, MarkerArray

def callback(config, level):
    return config

class OvertakingSectorPublisher:
    def __init__(self):
        self.glb_waypoints = None
        self.sector_pub = rospy.Publisher('/ot_sector_markers', MarkerArray, queue_size=10)
        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)

    def glb_wpnts_cb(self, data):
        self.glb_waypoints = []
        for waypoint in data.wpnts:
            self.glb_waypoints.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])

    def pub_sector_markers(self):
        rate = rospy.Rate(1)
        while (not rospy.is_shutdown()):
            if self.glb_waypoints is None:
                continue

            # one needs to set the overtaking_map_params rosparams
            sectors_params = rospy.get_param("/ot_map_params")
            n_sectors = sectors_params['n_sectors']
            sec_markers = MarkerArray()

            for i in range(n_sectors):
                s = sectors_params[f"Overtaking_sector{i}"]['start']
                if s == (len(self.glb_waypoints) - 1):
                    theta = np.arctan2((self.glb_waypoints[0][1] - self.glb_waypoints[s][1]),(self.glb_waypoints[0][0] - self.glb_waypoints[s][0]))
                else:
                    theta = np.arctan2((self.glb_waypoints[s+1][1] - self.glb_waypoints[s][1]),(self.glb_waypoints[s+1][0] - self.glb_waypoints[s][0]))
                quaternions = quaternion_from_euler(0, 0, theta)
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.type = marker.ARROW
                marker.scale.x = 0.5
                marker.scale.y = 0.05
                marker.scale.z = 0.15
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.position.x = self.glb_waypoints[s][0]
                marker.pose.position.y = self.glb_waypoints[s][1]
                marker.pose.position.z = 0
                marker.pose.orientation.x = quaternions[0]
                marker.pose.orientation.y = quaternions[1]
                marker.pose.orientation.z = quaternions[2]
                marker.pose.orientation.w = quaternions[3]
                marker.id = i
                sec_markers.markers.append(marker)

                marker_text = Marker()
                marker_text.header.frame_id = "map"
                marker_text.header.stamp = rospy.Time.now()
                marker_text.type = marker_text.TEXT_VIEW_FACING
                marker_text.text = f"Start Overtaking Sector {i}"
                marker_text.scale.z = 0.4
                marker_text.color.r = 0.1
                marker_text.color.g = 0.1
                marker_text.color.b = 1.2
                marker_text.color.a = 1.0
                marker_text.pose.position.x = self.glb_waypoints[s][0]
                marker_text.pose.position.y = self.glb_waypoints[s][1]
                marker_text.pose.position.z = 1.5
                marker_text.pose.orientation.x = 0.0
                marker_text.pose.orientation.y = 0.0
                marker_text.pose.orientation.z = 0.0436194
                marker_text.pose.orientation.w = 0.9990482
                marker_text.id = i + n_sectors
                sec_markers.markers.append(marker_text)
            self.sector_pub.publish(sec_markers)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dynamic_ot_sector_tuner", anonymous=False)
    print('Dynamic Overtaking Sector Server Launched...')
    # ot_dyn_sect_tunerConfig # Does this do anything? (removed)
    srv = Server(ot_dyn_sect_tunerConfig, callback)
    sec_pub = OvertakingSectorPublisher()
    sec_pub.pub_sector_markers()
