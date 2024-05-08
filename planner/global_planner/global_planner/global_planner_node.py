#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from f110_msgs.msg import WpntArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from .global_planner_utils import get_data_path
from .global_planner_logic import GlobalPlannerLogic
from .readwrite_global_waypoints import read_global_waypoints


class GlobalPlanner(Node):
    """
    Global planner node
    """

    def __init__(self):
        super().__init__('global_planner_node', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.rate = self.get_parameter('rate').value
        self.map_name = self.get_parameter('map_name').value
        self.create_map = self.get_parameter('create_map').value
        self.map_editor = self.get_parameter('map_editor').value
        self.reverse_mapping = self.get_parameter('reverse_mapping').value
        self.safety_width = self.get_parameter('safety_width').value
        self.safety_width_sp = self.get_parameter('safety_width_sp').value
        self.occupancy_grid_threshold = self.get_parameter('occupancy_grid_threshold').value
        self.filter_kernel_size = self.get_parameter('filter_kernel_size').value
        self.show_plots = self.get_parameter('show_plots').value
        self.required_laps = self.get_parameter('required_laps').value

        # get map source path
        self.map_dir = get_data_path('maps/' + self.map_name)

        # use watershed algorithm to find the centerline
        self.watershed = True

        self.logic = GlobalPlannerLogic(
            self.safety_width,
            self.safety_width_sp,
            self.occupancy_grid_threshold,
            self.map_editor,
            self.create_map,
            self.map_name,
            self.map_dir,
            get_data_path('scripts/finish_map.sh'),
            os.path.join(get_package_share_directory('stack_master'), 'config', 'global_planner'),
            self.show_plots,
            self.filter_kernel_size,
            self.required_laps,
            self.reverse_mapping,
            self.get_logger().info,
            self.get_logger().warn,
            self.get_logger().error
        )

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/car_state/pose', self.pose_cb, 10)
        self.get_logger().info("Waiting for map and car pose...")

        # Publishers
        # For the local planner
        self.global_waypoints_pub = self.create_publisher(WpntArray, '/global_waypoints', 10)
        self.centerline_waypoints_pub = self.create_publisher(WpntArray, '/centerline_waypoints', 10)
        # For visualization
        self.global_waypoints_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/markers', 10)
        self.centerline_waypoints_markers_pub = self.create_publisher(MarkerArray, '/centerline_waypoints/markers', 10)
        self.track_bounds_pub = self.create_publisher(MarkerArray, '/trackbounds/markers', 10)
        # For the shortest path
        self.shortest_path_waypoints_pub = self.create_publisher(WpntArray, '/global_waypoints/shortest_path', 10)
        self.shortest_path_waypoints_markers_pub = self.create_publisher(
            MarkerArray, '/global_waypoints/shortest_path/markers', 10)
        # Map infos
        self.map_infos_pub = self.create_publisher(String, '/map_infos', 10)
        # Estimated lap time (for l1_param_optimizer)
        self.est_lap_time_pub = self.create_publisher(Float32, '/estimated_lap_time', 10)

        # Main loop
        self.create_timer(1 / self.rate, self.global_plan_callback)

    def map_cb(self, data: OccupancyGrid):
        """
        Callback function of /map subscriber.

        Parameters
        ----------
        data
            Data received from /map topic
        """
        self.logic.update_map(data)

    def pose_cb(self, data: PoseStamped):
        """
        Callback function of /tracked_pose subscriber.

        Parameters
        ----------
        data
            Data received from /tracked_pose topic
        """

        self.logic.update_pose(data)

    def global_plan_callback(self):
        # Map name may be renamed from parameter if the map name already exists.
        # So look for the one returned by the function
        try:
            success, map_name = self.logic.global_plan_logic()
        except OSError as e:
            self.get_logger().error(f"{e}")
            self.get_logger().warn("Killing global planner...")
            self.destroy_node()
            return
        if success:
            self.get_logger().warn(f"Global planner succeeded: {map_name=}")
            self.read_and_publish(self.map_editor, self.create_map)
            self.get_logger().info("Killing global planner...")
            self.destroy_node()

    def read_and_publish(self, map_editor_mode: bool, create_map: bool) -> None:
        """Reads infos serialized by the global planner and publishes them, thus decoupling ROS publishing from main logic.

        Args:
            map_name (str): eventual map name returned by main logic
            map_editor_mode (bool): if map editor mode is on
        """

        # in map editor mapping mode, just exit (anyway the JSON is not written)
        if map_editor_mode and create_map:
            self.get_logger().warn("In map_editor_mapping mode. Waypoints are not calculated, and thus not published.")
            return

        # Read global waypoints and publish
        try:
            map_infos, est_lap_time, centerline_markers, centerline_wpnts, \
                glb_markers, glb_wpnts, \
                glb_sp_markers, glb_sp_wpnts, \
                track_bounds = read_global_waypoints(map_dir=self.map_dir)
        except FileNotFoundError as e:
            self.get_logger().error(f"{e}. Not republishing waypoints.")
            return

        self.global_waypoints_pub.publish(glb_wpnts)
        self.centerline_waypoints_pub.publish(centerline_wpnts)
        self.centerline_waypoints_markers_pub.publish(centerline_markers)
        self.global_waypoints_markers_pub.publish(glb_markers)
        self.track_bounds_pub.publish(track_bounds)
        self.shortest_path_waypoints_pub.publish(glb_sp_wpnts)
        self.shortest_path_waypoints_markers_pub.publish(glb_sp_markers)
        self.map_infos_pub.publish(map_infos)
        self.est_lap_time_pub.publish(est_lap_time)


def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
