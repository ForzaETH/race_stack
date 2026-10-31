#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node


class FTG_Controller(Node):
    #Lidar processing params
    PREPROCESS_CONV_SIZE = 3
    
    #Steering params
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees
    MILD_CURVE_ANGLE = np.pi / 6  # 30 degrees
    ULTRASTRAIGHTS_ANGLE = np.pi / 60  # 3 deg

    def __init__(self,
                 mapping,
                 debug,
                 safety_radius,
                 max_lidar_dist,
                 max_speed,
                 range_offset,
                 track_width) -> None:
        super().__init__('controller_manager')
        """
        Initialize the FTG controller.

        Parameters:
            mapping (bool): Flag indicating whether FTG is used for mapping or not.
        """
        self.mapping = mapping
        self.DEBUG = debug
        self.SAFETY_RADIUS = safety_radius
        self.MAX_LIDAR_DIST = max_lidar_dist
        self.MAX_SPEED = max_speed
        self.radians_per_elem = None # used when calculating the angles of the LiDAR data
        self.range_offset = range_offset
        self.track_width = track_width
        
        
        # Speed params
        scale = 0.6  # .575 is  max
        self.CORNERS_SPEED = 0.3 * self.MAX_SPEED * scale
        self.MILD_CORNERS_SPEED = 0.45 * self.MAX_SPEED * scale
        self.STRAIGHTS_SPEED = 0.8 * self.MAX_SPEED * scale
        self.ULTRASTRAIGHTS_SPEED = self.MAX_SPEED * scale
        
        
        self.velocity = 0
        self.scan = None

        self.best_pnt = self.create_publisher(Marker, '/best_points/marker', 10)
        self.scan_pub = self.create_publisher(MarkerArray, '/scan_proc/markers', 10)
        self.best_gap = self.create_publisher(MarkerArray, '/best_gap/markers', 10)

    def _preprocess_lidar(self, ranges) -> np.ndarray:
        """ 
        Preprocess the LiDAR scan array.

        This method performs preprocessing on the LiDAR scan array. The preprocessing steps include:
        1. Setting each value to the mean over a specified window.
        2. Rejecting high values (e.g., values greater than 3m).

        Parameters:
            ranges (numpy.ndarray): The LiDAR scan array.

        Returns:
            numpy.ndarray: The preprocessed LiDAR scan array.
        """
        self.radians_per_elem = (1.5 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        # full angle is -135 135
        # every point in the array is
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        # sets each value to the mean over a given window to smoothen the signal
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE)/self.PREPROCESS_CONV_SIZE, 'valid') 
        # clip the ranges between 0 and your maximum lidar distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        # reverse lidar because it is right to left
        return proc_ranges[::-1]

    def _get_steer_angle(self, point_x, point_y) -> float:
        """ 
        Get the angle of a particular element in the LiDAR data and
        transform it into an appropriate steering angle.
        
        Parameters:
            point_x (float): The x-coordinate of the LiDAR data point
            point_y (float): The y-coordinate of the LiDAR data point
        
        Returns:
            float: The transformed steering angle
        
        """
        steering_angle = math.atan2(point_y, point_x)
        return np.clip(steering_angle, -0.4, 0.4)

    def _get_best_range_point(self, proc_ranges) -> tuple:
        """ 
        Find the best point i.e. the middle of the largest gap within the bubble radius.
        
        Parameters:
            proc_ranges (list): List of processed ranges.
        
        Returns:
            tuple: The x and y coordinates of the best point.
        """
        #Get the bubble radius 
        radius = self._get_radius()
        
        #Find the largest gap
        gap_left, gap_right = self._find_largest_gap(ranges=proc_ranges, radius=radius)
        gap_left += self.range_offset - 180
        gap_right += self.range_offset - 180
        gap_middle = int((gap_right + gap_left) / 2)
        #Calculate cartesian point of the best point position from the lidar measurements in laser frame
        best_y = np.cos(gap_middle * self.radians_per_elem) * radius
        best_x = np.sin(gap_middle * self.radians_per_elem) * radius
        
        if self.DEBUG:
            #Delete old gaps from RVIZ
            self._delete_gap_markers()

            #Visualise the gap
            gap_markers = MarkerArray()
            for i in range(gap_left, gap_right):
                mrk = Marker()
                mrk.header.frame_id = 'car_state/laser'
                mrk.header.stamp = self.get_clock().now().to_msg()
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.05
                mrk.scale.y = 0.05
                mrk.scale.z = 0.05
                mrk.color.a = 1.0
                mrk.color.r = 1.0
                mrk.color.g = 1.0
                mrk.id = int(i - gap_left)
                #Calculate cartesian point of the gap  marker position from the lidar measurements in laser frame
                mrk.pose.position.y = math.cos(i * self.radians_per_elem) * radius
                mrk.pose.position.x = math.sin(i * self.radians_per_elem) * radius
                mrk.pose.orientation.w = 1.0
                gap_markers.markers.append(mrk)
            self.best_gap.publish(gap_markers)

            # visualize best point aka middle of the gap
            best_mrk = Marker()
            best_mrk.header.frame_id = 'car_state/laser'
            best_mrk.header.stamp = self.get_clock().now().to_msg()
            best_mrk.type = best_mrk.SPHERE
            best_mrk.scale.x = 0.2
            best_mrk.scale.y = 0.2
            best_mrk.scale.z = 0.2
            best_mrk.color.a = 1.0
            best_mrk.color.b = 1.0
            best_mrk.color.g = 1.0
            best_mrk.id = 0
            best_mrk.pose.position.y = best_y
            best_mrk.pose.position.x = best_x
            best_mrk.pose.orientation.w = 1.0
            self.best_pnt.publish(best_mrk)
        
        return best_x, best_y

    def process_lidar(self, ranges) -> tuple:
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm &
        calculate the speed and steering angle.

        Parameters:
            ranges (list): List of LiDAR scan ranges

        Returns:
            tuple: A tuple containing the speed and steering angle
        """
        #Preprocess the LiDAR to smoothen it
        proc_ranges = self._preprocess_lidar(ranges)
        
        proc_ranges = self._safety_border(proc_ranges)
        
        if self.DEBUG:
            scan_markers = MarkerArray()
            for i, scan in enumerate(proc_ranges):
                mrk = Marker()
                mrk.header.frame_id = 'car_state/laser'
                mrk.header.stamp = self.get_clock().now().to_msg()
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.05
                mrk.scale.y = 0.05
                mrk.scale.z = 0.05
                mrk.color.a = 1.0
                mrk.color.r = 1.0
                mrk.color.b = 1.0

                mrk.id = i
                mrk.pose.position.x = math.sin(i* self.radians_per_elem) * scan
                mrk.pose.position.y = math.cos(i* self.radians_per_elem) * scan
                mrk.pose.orientation.w = 1.0
                scan_markers.markers.append(mrk)
            self.scan_pub.publish(scan_markers)

        #Get best point to target aka middle of the largest gap
        best_x, best_y = self._get_best_range_point(proc_ranges)

        #Get steer angle from best points
        steering_angle = self._get_steer_angle(point_x=best_x, point_y=best_y)

        if self.mapping:
            speed = 1.5
        else:
            if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                speed = self.CORNERS_SPEED
            elif abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
                speed = self.MILD_CORNERS_SPEED
            elif abs(steering_angle) > self.ULTRASTRAIGHTS_ANGLE:
                speed = self.STRAIGHTS_SPEED
            else:
                speed = self.ULTRASTRAIGHTS_SPEED

        return speed, steering_angle

    def _find_largest_gap(self, ranges, radius) -> tuple:
        """ 
        Find the index of the starting and ending of the largest gap and its width

        Parameters:
            ranges (numpy.ndarray): Array of range values
            radius (float): Threshold radius value

        Returns:
            tuple: A tuple containing the index of the starting of the largest gap, 
                    the index of the ending of the largest gap, and the width of the largest gap.

        """
        #Binarise the ranges in zeros for values under the radius threshold and ones for above and equal
        bin_ranges = np.where(ranges >= radius, 1, 0)
        
        #Get largest gap from binary ranges
        bin_diffs = np.abs(np.diff(bin_ranges))
        bin_diffs[0] = 1
        bin_diffs[-1] = 1

        diff_idxs = bin_diffs.nonzero()[0]
        #Check that binarised ranges are positive
        high_gaps = []
        for i in range(len(diff_idxs)-1):
            low = diff_idxs[i]
            high = diff_idxs[i+1]
            high_gaps.append(np.mean(bin_ranges[low:high]) > 0.5)

        gap_left = diff_idxs[np.argmax(high_gaps * np.diff(diff_idxs))]
        gap_width = np.max(high_gaps * np.diff(diff_idxs))
        gap_right = gap_left + gap_width        

        return gap_left, gap_right

    def _get_radius(self) -> float:
        """
        Calculate the radius based on the track width and velocity.

        Returns:
            float: The calculated radius.
        """
        # Empirically determined that this radius choosing makes sense
        return min(5., self.track_width / 2 + 2 * (self.velocity / self.MAX_SPEED))

    def set_vel(self, velocity) -> None:
        """
        Set the velocity of the car.
        
        Parameters:
            velocity (float): The desired velocity value.
        """
        self.velocity = velocity
    
    def _safety_border(self, ranges) -> np.ndarray:
        """
        Add a safety bubble if there is a big increase in the range between two points.

        Parameters:
            ranges (list): List of range values.

        Returns:
            np.ndarray: Array of filtered range values.
        """
        filtered = list(ranges)
        ranges_len = len(ranges)
        i = 0
        while i < ranges_len - 1:
            if ranges[i + 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i + j < ranges_len:
                        filtered[i + j] = ranges[i]
                i += self.SAFETY_RADIUS - 2
            i += 1
        # in other direction
        i = ranges_len - 1
        while i > 0:
            if ranges[i - 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i - j >= 0:
                        filtered[i - j] = ranges[i]
                i = i - self.SAFETY_RADIUS + 2
            i -= 1
        return np.array(filtered)

    def _delete_gap_markers(self) -> None:
        """
        Delete marker for rviz when not needed
        """
        del_mrk_array = MarkerArray()
        for i in range(1):
            del_mrk = Marker()
            del_mrk.header.frame_id = 'car_state/laser'
            del_mrk.header.stamp = self.get_clock().now().to_msg()
            del_mrk.action = del_mrk.DELETEALL
            del_mrk.id = i
            del_mrk_array.markers.append(del_mrk)
        self.best_gap.publish(del_mrk_array)
