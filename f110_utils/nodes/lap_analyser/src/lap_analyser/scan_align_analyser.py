#!/usr/bin/env python3

import rospy, tf
import time
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from f110_msgs.msg import WpntArray
from std_msgs.msg import Float32
import numpy as np

class LaserMapOverlapChecker:
    """
    A node that checks if laser scan points overlap with the Track boundaries.
    """

    def __init__(self):
        """
        Constructor for the LaserMapOverlapChecker class.
        Initializes the node, subscribers, and class variables.
        """
        # Initialize the node
        rospy.init_node('laser_map_overlap_checker', anonymous=True)

        # Rosparams
        self.debug_flag = rospy.get_param('~debug', default=False)
        self.downsample_factor = rospy.get_param('~downsample_factor', default=54)
        self.threshold = rospy.get_param('~threshold', default=0.1)
        self.frequency = rospy.get_param('~frequency', default=5)

        # Subscriber for the map and laser scan
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.trackbounds_sub = rospy.Subscriber('/trackbounds/markers', MarkerArray, self.trackbounds_callback)
        self.gb_sub = rospy.Subscriber("/global_waypoints", WpntArray, self.glb_wpnts_cb)  # from og wpnts

        # Publishers
        self.debug_lidar_pub = rospy.Publisher('/scanalign/debug_lidar', MarkerArray, queue_size=1)
        self.alignment_score_pub = rospy.Publisher('/scanalign/alignment_score', Float32, queue_size=1)
        self.alignment_textmrk_pub = rospy.Publisher('/scanalign/alignment_textmrk', Marker, queue_size=1)

        # Map data
        self.map = None
        self.trackbounds = None
        self.glb_wpnts = None

        # Laser scan data
        self.scan = None

        #misc
        self.rate = rospy.Rate(5)  # 5 Hz
        self.tf_listener = tf.TransformListener()
        self.first_visualization = True
        self.x_viz = None
        self.y_viz = None

        #Keep on going
        self.loop()

    def map_callback(self, data):
        """
        Callback function for the map topic.
        Stores the map data and metadata.

        Args:
        - data (OccupancyGrid): The received map.
        """
        self.map = data

    def laser_callback(self, data):
        """
        Callback function for the laser scan topic.
        
        Args:
        - data (LaserScan): The received laser scan.
        """
        self.scan = data

    def trackbounds_callback(self, data):
        """
        Callback function for the track bounds topic.
        
        Args:
        - data (MarkerArray): The received track bounds.
        """
        self.trackbounds = data

    def glb_wpnts_cb(self, data):
        """
        Callback function for the global waypoints topic.
        ONLY USED FOR VISUALIZATION OF TEXT MARKER.
        
        Args:
        - data (WpntArray): The received global waypoints.
        """
        self.glb_wpnts = data.wpnts[:-1]

    ############END OF CALLBACKS############
    
    def compute_scan_alignment(self, scan_in_map, trackbounds: MarkerArray):
        """
        Compute the alignment between the laser scan and the map.
        Checks if the laser scan points overlap with the map.

        Args:
        - scan_in_map (np Array): scan points in map frame.
        - trackbounds (Markerarray): MarkerArray of the track boundaries.
        """
        
        # Extract boundary positions from the MarkerArray
        boundary_positions = np.array([[marker.pose.position.x, marker.pose.position.y, marker.pose.position.z] for marker in trackbounds.markers])

        overlaps = np.zeros(scan_in_map.shape[1], dtype=bool)

        # For each scan point, check if it's close to any boundary marker
        for i in range(scan_in_map.shape[1]):
            distances = np.linalg.norm(boundary_positions - scan_in_map[:3, i][:, np.newaxis].T, axis=1)
            # Check if any distance is less than the 5X marker's radius (assuming all markers have the same scale)
            if np.any(distances < self.threshold):
                overlaps[i] = True

        # Compute the alignment score in percentage
        align_cnt = np.sum(overlaps)
        align_score = (align_cnt / (1080/self.downsample_factor)) * 100 

        return align_score
    
    def laser2map_conversion(self, laser: LaserScan):
        # Get Transform between laser and radar
        t_laser_map = self.tf_listener.getLatestCommonTime("map", "laser")
        trans, quat = self.tf_listener.lookupTransform('/map', '/laser', t_laser_map)
        T = np.array(trans)
        R = tf.transformations.quaternion_matrix(quat)

        # Filter out ranges that are under 0.1 meters
        valid_indices = np.array(laser.ranges) > 0.1
        valid_ranges = np.array(laser.ranges)[valid_indices]

        # Transform laser points to map frame
        angles = np.linspace(laser.angle_min, laser.angle_max, len(laser.ranges))[valid_indices]
        x_lf = (valid_ranges * np.cos(angles)).flatten()
        y_lf = (valid_ranges * np.sin(angles)).flatten()
        z_lf = (-T[-1] * np.ones(len(valid_ranges))).flatten()
        xyz_lf = np.vstack((x_lf, y_lf, z_lf, np.ones(len(valid_ranges))))

        # Build homogenous transform from laser to map frame
        H_l2m = R
        H_l2m[0:3, -1] = T

        # Apply transform to the xyz points s.t. the points are transformed into map frame
        xyz_mf = H_l2m @ xyz_lf

        #Downsample the points for compute but in a normal distribution around the middle
        # Define parameters
        center_index = len(xyz_mf[0]) // 2  
        num_samples = int(1080/self.downsample_factor)
        std_dev = int(0.68*num_samples)  # Standard deviation for the normal distribution

        # Generate random indices based on a normal distribution
        indices = np.random.normal(center_index, std_dev, num_samples).astype(int)

        # Clip indices to ensure they are within valid bounds
        indices = np.clip(indices, 0, len(xyz_mf[0]) - 1)

        # Sample the array using the generated indices
        sampled_xyz_mf = xyz_mf[:, indices]

        return sampled_xyz_mf


    def loop(self):
        """
        Main loop of the node.
        """
        rospy.loginfo("[Scan Alignment] Waiting for scan, and track bounds...")
        rospy.wait_for_message('/scan', LaserScan)
        rospy.wait_for_message('/trackbounds/markers', MarkerArray)
        rospy.loginfo("[Scan Alignment] Ready!")
        while not rospy.is_shutdown():
            start_time = time.time()

            #sample data
            scan = self.scan
            tb = self.trackbounds

            #Convert laser scan to map frame
            xyz_mf = self.laser2map_conversion(laser=scan)

            #Debug publish laser points in map frame
            if self.debug_flag:
                self._pub_map_lidarpoints(points=xyz_mf, timestamp=scan.header.stamp)

            #Compute alignment score
            alignment = self.compute_scan_alignment(scan_in_map=xyz_mf, trackbounds=tb)
            self.alignment_score_pub.publish(alignment)
            self._pub_alignscore_txt(score=alignment)

            #print('[Scan Align] Scan alignment score: {} [%]'.format(alignment))
            #print('[Scan Align] Loop time: {} [s], Possible Frequency: {} [Hz]'.format(time.time() - start_time, 1/(time.time() - start_time)))

            self.rate.sleep()


    #########HELPERS#########
    def _pub_map_lidarpoints(self, points: np.ndarray, timestamp: rospy.Time):
        # Publish points in radar frame
        mrks = MarkerArray()

        # Only use every 1th point
        laser = points[:, ::1]

        for i in range(laser.shape[1]):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.header.stamp = timestamp
            mrk.id = i
            mrk.type = mrk.SPHERE
            mrk.action = mrk.ADD
            mrk.pose.position.x = laser[0, i]
            mrk.pose.position.y = laser[1, i]
            mrk.pose.position.z = laser[2, i]
            mrk.pose.orientation.w = 1
            mrk.scale.x = 0.1
            mrk.scale.y = 0.1
            mrk.scale.z = 0.1
            mrk.color.a = 1
            mrk.color.r = 0.0
            mrk.color.g = 1
            mrk.color.b = 0.6
            mrks.markers.append(mrk)
        self.debug_lidar_pub.publish(mrks)

    def _pub_alignscore_txt(self, score: float):
        # Publish alignment score as text marker
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 150% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.5 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = rospy.Time.now()
        mrk.id = 0
        mrk.type = mrk.TEXT_VIEW_FACING
        mrk.action = mrk.ADD
        mrk.pose.position.x = self.x_viz
        mrk.pose.position.y = self.y_viz
        mrk.pose.position.z = 1.0
        mrk.pose.orientation.w = 1
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 1
        mrk.color.r = 1
        mrk.color.g = 0
        mrk.color.b = 0
        mrk.text = "Scan Alignment Score: {:.2f} [%]".format(score)
        self.alignment_textmrk_pub.publish(mrk)



if __name__ == '__main__':
    """
    Main execution point. Instantiates the LaserMapOverlapChecker class.
    """
    checker = LaserMapOverlapChecker()
