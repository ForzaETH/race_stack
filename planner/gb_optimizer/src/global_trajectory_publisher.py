#!/usr/bin/env python3

import rospy
from rospy import loginfo
from std_msgs.msg import String, Float32
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import MarkerArray

from readwrite_global_waypoints import read_global_waypoints

class GlobalRepublisher:
    """
    Node for publishing the global waypoints/markers and track bounds markers frequently after they have been calculated
    """
    def __init__(self):

        rospy.init_node('global_republisher_node', anonymous=True)

        self.glb_markers = None
        self.glb_wpnts = None
        self.track_bounds = None

        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber('/global_waypoints/markers', MarkerArray, self.glb_markers_cb)
        rospy.Subscriber('/trackbounds/markers', MarkerArray, self.bounds_cb)

        self.glb_wpnts_pub = rospy.Publisher('global_waypoints', WpntArray, queue_size=10)
        self.glb_markers_pub = rospy.Publisher('global_waypoints/markers', MarkerArray, queue_size=10)
        self.vis_track_bnds = rospy.Publisher('trackbounds/markers', MarkerArray, queue_size=10)

        # shortest_path
        self.glb_sp_markers = None
        self.glb_sp_wpnts = None
        rospy.Subscriber('/global_waypoints/shortest_path', WpntArray, self.glb_sp_wpnts_cb)
        rospy.Subscriber('/global_waypoints/shortest_path/markers', MarkerArray, self.glb_sp_markers_cb)
        self.glb_sp_wpnts_pub = rospy.Publisher('global_waypoints/shortest_path', WpntArray, queue_size=10)
        self.glb_sp_markers_pub = rospy.Publisher('global_waypoints/shortest_path/markers', MarkerArray, queue_size=10)

        # centerline
        self.centerline_wpnts = None
        self.centerline_markers = None
        rospy.Subscriber('/centerline_waypoints', WpntArray, self.centerline_wpnt_cb)
        rospy.Subscriber('/centerline_waypoints/markers', MarkerArray, self.centerline_markers_cb)
        self.centerline_wpnts_pub = rospy.Publisher('/centerline_waypoints', WpntArray, queue_size=10)
        self.centerline_markers_pub = rospy.Publisher('/centerline_waypoints/markers', MarkerArray, queue_size=10)

        # map infos
        self.map_infos = None
        rospy.Subscriber('/map_infos', String, self.map_info_cb)
        self.map_info_pub = rospy.Publisher('map_infos', String, queue_size=10)
        
        self.est_lap_time = None
        rospy.Subscriber('estimated_lap_time', Float32, self.est_lap_time_cb)
        self.est_lap_time_pub = rospy.Publisher('estimated_lap_time', Float32, queue_size=10)


        # graph lattice 
        self.graph_lattice = None
        rospy.Subscriber('/lattice_viz', MarkerArray, self.lattice_cb)
        self.lattice_pub = rospy.Publisher('/lattice_viz', MarkerArray, queue_size=10)
        # Read info from json file if it is provided, so everything is always published
        if rospy.has_param('/global_republisher/map_name'):
            map_name = rospy.get_param('/global_republisher/map_name')
            loginfo(f"Reading parameters from {map_name}")

            try:
                self.map_infos, self.est_lap_time, self.centerline_markers, self.centerline_wpnts,\
                self.glb_markers, self.glb_wpnts,\
                self.glb_sp_markers, self.glb_sp_wpnts, \
                self.track_bounds = read_global_waypoints(map_name)
            except FileNotFoundError:
                rospy.logwarn(f"{map_name} param not found. Not publishing")
        else:
            loginfo(f"global_trajectory_publisher did not find any map_name param")

    def glb_wpnts_cb(self, data):
        self.glb_wpnts = data
        track_length = data.wpnts[-1].s_m
        rospy.set_param('global_republisher/track_length', track_length)

    def glb_markers_cb(self, data):
        self.glb_markers = data

    def glb_sp_wpnts_cb(self, data):
        self.glb_sp_wpnts = data

    def glb_sp_markers_cb(self, data):
        self.glb_sp_markers = data

    def centerline_wpnt_cb(self, data: WpntArray):
        self.centerline_wpnts = data

    def centerline_markers_cb(self, data: MarkerArray):
        self.centerline_markers = data

    def bounds_cb(self, data):
        self.track_bounds = data

    def map_info_cb(self, data):
        self.map_infos = data    
    
    def est_lap_time_cb(self, data):
        self.est_lap_time = data

    def lattice_cb(self, data):
        self.graph_lattice = data

    def global_republisher(self):
        rate = rospy.Rate(0.5)  # in Hertz
        while not rospy.is_shutdown():

            if self.glb_wpnts is not None and self.glb_markers is not None:
                self.glb_wpnts_pub.publish(self.glb_wpnts)
                self.glb_markers_pub.publish(self.glb_markers)
            if self.glb_sp_wpnts is not None and self.glb_sp_markers is not None:
                self.glb_sp_wpnts_pub.publish(self.glb_sp_wpnts)
                self.glb_sp_markers_pub.publish(self.glb_sp_markers)
            if self.centerline_wpnts is not None and self.centerline_markers is not None:
                self.centerline_wpnts_pub.publish(self.centerline_wpnts)
                self.centerline_markers_pub.publish(self.centerline_markers)
            if self.track_bounds is not None:
                self.vis_track_bnds.publish(self.track_bounds)
            if self.map_infos is not None:
                self.map_info_pub.publish(self.map_infos)
            if self.est_lap_time is not None:
                self.est_lap_time_pub.publish(self.est_lap_time)
            if self.graph_lattice is not None:
                self.lattice_pub.publish(self.graph_lattice)

            rate.sleep()


if __name__ == "__main__":
    republisher = GlobalRepublisher()
    republisher.global_republisher()
