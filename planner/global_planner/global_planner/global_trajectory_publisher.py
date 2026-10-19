#!/usr/bin/env python3

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rs_msgs.msg import WpntArray
from visualization_msgs.msg import MarkerArray

# the `.` tells Python to look in the current directory at runtime
from .readwrite_global_waypoints import read_global_waypoints

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class GlobalRepublisher(Node):
    """
    Node for publishing the global waypoints/markers and track bounds markers of a map, once and latched
    """
    def __init__(self):
        super().__init__('global_republisher_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.glb_markers = None
        self.glb_wpnts = None
        self.track_bounds = None
        self.publish_markers = self.get_parameter('publish_markers').get_parameter_value().bool_value
        

        #Declare local ros params
        self.declare_parameter(name='track_length', value=6.9)

        self.glb_wpnts_pub = self.create_publisher(WpntArray, '/global_waypoints', LATCHED)
        if self.publish_markers:
            self.glb_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/markers', LATCHED)
            self.vis_track_bnds = self.create_publisher(MarkerArray, '/trackbounds/markers', LATCHED)

        # shortest_path
        self.publish_shortest_path = self.get_parameter('publish_shortest_path').get_parameter_value().bool_value
        if self.publish_shortest_path:
            self.get_logger().debug("Global Republisher: Publishing shortest path waypoints and markers")
            self.glb_sp_markers = None
            self.glb_sp_wpnts = None
            self.glb_sp_wpnts_pub = self.create_publisher(WpntArray, '/global_waypoints/shortest_path', LATCHED)
            if self.publish_markers:
                self.glb_sp_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/shortest_path/markers', LATCHED)

        # centerline
        self.publish_centerline = self.get_parameter('publish_centerline').get_parameter_value().bool_value
        if self.publish_centerline:
            self.get_logger().info("Global Republisher: Publishing centerline waypoints and markers")
            self.centerline_wpnts = None
            self.centerline_markers = None
            self.centerline_wpnts_pub = self.create_publisher(WpntArray, '/centerline_waypoints', LATCHED)
            if self.publish_markers:
                self.centerline_markers_pub = self.create_publisher(MarkerArray, '/centerline_waypoints/markers', LATCHED)

        # map infos
        self.map_infos = None
        self.map_info_pub = self.create_publisher(String, 'map_infos', LATCHED)

        self.est_lap_time = None
        self.est_lap_time_pub = self.create_publisher(Float32, 'estimated_lap_time', LATCHED)

        # graph lattice
        self.publish_lattice = self.get_parameter('publish_lattice').get_parameter_value().bool_value
        if self.publish_lattice:
            self.get_logger().info("Global Republisher: Publishing graph lattice")
            self.graph_lattice = None
            self.lattice_pub = self.create_publisher(MarkerArray, '/lattice_viz', LATCHED)

        # Read info from json file if it is provided, so everything is always published
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        if map_path:
            self.get_logger().debug(f"Reading parameters from {map_path}")
            try:
                (
                    self.map_infos, self.est_lap_time, self.centerline_markers,
                    self.centerline_wpnts, self.glb_markers, self.glb_wpnts,
                    self.glb_sp_markers, self.glb_sp_wpnts, self.track_bounds
                ) = read_global_waypoints(map_dir=map_path)
            except FileNotFoundError:
                self.get_logger().warn(f"{map_path} param not found. Not publishing")
        else:
            self.get_logger().info(f"global_trajectory_publisher did not find any map_path param {map_path}")
        
        # latched, so publishing once is enough
        self.global_republisher()

    def global_republisher(self):

        # Publish global waypoints
        if self.glb_wpnts is not None:
            self.glb_wpnts_pub.publish(self.glb_wpnts)
        if self.publish_markers and self.glb_markers is not None and hasattr(self, 'glb_markers_pub'):
            self.glb_markers_pub.publish(self.glb_markers)
        
        # Publish shortest path
        if self.publish_shortest_path:
            if self.glb_sp_wpnts is not None and hasattr(self, 'glb_sp_wpnts_pub'):
                self.glb_sp_wpnts_pub.publish(self.glb_sp_wpnts)
            if self.publish_markers and self.glb_sp_markers is not None and hasattr(self, 'glb_sp_markers_pub'):
                self.glb_sp_markers_pub.publish(self.glb_sp_markers)
        
        # Publish centerline
        if self.publish_centerline:
            if self.centerline_wpnts is not None and hasattr(self, 'centerline_wpnts_pub'):
                self.centerline_wpnts_pub.publish(self.centerline_wpnts)
            if self.publish_markers and self.centerline_markers is not None and hasattr(self, 'centerline_markers_pub'):
                self.centerline_markers_pub.publish(self.centerline_markers)
        
        # Publish track bounds
        if self.publish_markers and self.track_bounds is not None and hasattr(self, 'vis_track_bnds'):
            self.vis_track_bnds.publish(self.track_bounds)
        
        # Publish map info and lap time (always)
        if self.map_infos is not None:
            self.map_info_pub.publish(self.map_infos)
        if self.est_lap_time is not None:
            self.est_lap_time_pub.publish(self.est_lap_time)
        
        # Publish lattice
        if self.publish_lattice and self.graph_lattice is not None and hasattr(self, 'lattice_pub'):
            self.lattice_pub.publish(self.graph_lattice)

         

def main(args=None):
    rclpy.init(args=args)
    republisher = GlobalRepublisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
