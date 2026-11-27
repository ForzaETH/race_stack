import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException  # type: ignore

from stack_master.qos import LATCHED_QOS, LATEST_SENSOR_QOS
from perception import obstacle_detection as od
from perception import visualization as viz
from perception.obstacle_detection_parameters_codegen import obstacle_detection as parameters
from f110_msgs.msg import WpntArray, Detection, DetectionArray
from frenet_conversion.frenet_converter import FrenetConverter


def _se2(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    return np.array([t.x, t.y, od.yaw_from_quat(q.x, q.y, q.z, q.w)])


class ObstacleDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_detection")

        # Parameters: config/obstacle_detection_parameters.yaml
        self.params = parameters.ParamListener(self).get_params()

        # Static map built from the track bounds
        self.grid = self.track = self.wdist = None

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Publishers
        self.points_pub = self.create_publisher(PointCloud2, "/perception/candidate_points", 10)
        self.rejected_pub = (self.create_publisher(PointCloud2, "/perception/rejected_points", 10)
                             if self.params.publish_rejected else None)
        self.obstacles_pub = self.create_publisher(DetectionArray, "/perception/detections", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/perception/detections/markers", 10)

        # Obtain trackbounds from the planner's track bounds (TODO: this is hacky)
        self.create_subscription(MarkerArray, "/trackbounds/markers", self.bounds_cb, LATCHED_QOS)
        # Newest scan only: if a scan takes longer than the scan period, skip scans instead of processing stale ones
        self.create_subscription(LaserScan, self.params.scan_topic, self.scan_cb, LATEST_SENSOR_QOS)
        self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, LATCHED_QOS)

        self.converter: FrenetConverter | None = None

    def bounds_cb(self, msg: MarkerArray) -> None:
        """Construct the static grid, track mask and wall distance from the planner's trackbounds."""
        xy = np.array([[m.pose.position.x, m.pose.position.y] for m in msg.markers])

        # TODO: hacky. In the future just get the trackbounds directly from a topic, not inferring them through marker
        # colours
        is_left = np.array([m.color.g > 0.9 for m in msg.markers])
        right, left = xy[~is_left], xy[is_left]

        # Construct the static grid
        self.grid = od.grid_from_bounds(right, left, self.params.grid_res, self.params.grid_pad)
        self.track = od.track_mask(self.grid, left, right)
        self.wdist = od.wall_distance(self.grid, right, left)
        self.get_logger().info(f"grid {self.grid.width}x{self.grid.height} @ {self.params.grid_res} m")

    def pose(self, target: str, source: str, t: Time) -> np.ndarray:
        return _se2(self.tf_buffer.lookup_transform(target, source, t))

    def pose_at(self, target: str, source: str, t: Time) -> np.ndarray:
        """Pose at time t: exact TF if available, else linearly extrapolated from the two newest poses."""
        if self.tf_buffer.can_transform(target, source, t):
            return self.pose(target, source, t)

        tb = Time.from_msg(self.tf_buffer.lookup_transform(target, source, Time()).header.stamp)

        # Too far past the newest pose, skip this scan
        gap = (t - tb).nanoseconds * 1e-9
        if gap > self.params.max_extrapolation:
            raise TransformException(f"newest {target}->{source} pose is {gap * 1e3:.0f} ms older than the scan")

        ta = tb - Duration(nanoseconds=20_000_000)
        alpha = (t - ta).nanoseconds / (tb - ta).nanoseconds
        return od._se2_interp(self.pose(target, source, ta), self.pose(target, source, tb), [alpha])[0]

    def laser_poses(self, scan: LaserScan, scan_duration: float):
        """
        Returns:
            - map_odom (where odom sits in the map (in sim that's just the identity)
            - odom_laser_start (laser pose in odom of the first beam)
            - odom_laser_end (laser pose in odom of the last beam)
        """
        t0 = Time.from_msg(scan.header.stamp)
        t1 = t0 + Duration(nanoseconds=int(scan_duration * 1e9))
        frame = scan.header.frame_id

        try:
            # There is no odom frame in sim
            if not self.params.odom_frame:                
                # Obtain the laser pose directly in map frame
                start = self.pose_at(self.params.map_frame, frame, t0)
                end = self.pose_at(self.params.map_frame, frame, t1)
                return np.zeros(3), start, end

            # Obtain the laser pose in odom frame
            start = self.pose_at(self.params.odom_frame, frame, t0)
            end = self.pose_at(self.params.odom_frame, frame, t1)

            try:
                map_odom = self.pose(self.params.map_frame, self.params.odom_frame, t0)
            except TransformException:
                tf = self.tf_buffer.lookup_transform(self.params.map_frame, self.params.odom_frame, Time())
                lag = (t0 - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                if lag > self.params.max_map_odom_lag:
                    raise TransformException(f"latest map->odom is {lag:.3f} s old")
                map_odom = _se2(tf)
            return map_odom, start, end
        except TransformException as e:
            self.get_logger().warn(f"skipping scan: {e}", throttle_duration_sec=1.0)
            return None

    def scan_cb(self, scan: LaserScan) -> None:
        """
        Detect obstacles in a LiDAR scan:
        
        1. Project valid beams into the map frame, each at its own beam time (motion-compensated)
        2. Break up the scan at range jumps, keeping segments only if all its points are inside the track and clear of
           the walls.
        3. Merge nearby kept segments into car candidates, dropping those not car-sized.
        4. Fit a box heading near the raceline heading, and extract a seen outline (rear / front / left / right)
        5. Publish the kept points and observations as Frenet frame obstacles.
        """
        if self.grid is None:
            return

        # Laser poses in odom frame of the first and last beam
        scan_duration = (len(scan.ranges) - 1) * scan.time_increment
        poses = self.laser_poses(scan, scan_duration)
        if poses is None:
            return

        # Valid beams: within range, not NaN or inf, and their timestamp relative to header.stamp
        idx, xy, r, dt = od.beam_geometry(scan.ranges, scan.angle_min, scan.angle_increment,
                                          scan.time_increment, scan.range_min, scan.range_max)

        # Segment the scan at range jumps
        labels = od.segment_scan(
            idx, xy, r, scan.angle_increment, np.radians(
                self.params.lambda_deg), self.params.sigma)

        # Transform each point to map frame
        xy_map = od.beams_to_map(xy, dt, scan_duration, *poses)

        # Keep points on the track and clear of the walls
        ok = od.candidate_mask(xy_map, r, self.grid, self.track, self.wdist, self.params.m0, self.params.k_range)

        # Keep a segment only if ALL its points are ok (strict to minimize false positives) and within max_range
        keep = od.accept_segments(labels, ok) & (r <= self.params.max_range)

        # Group the kept segments into obstacle candidates and observe what of each is seen
        observations, headings, boxes = [], [], []
        if self.converter is not None:
            ranges = np.asarray(scan.ranges, dtype=float)
            max_extent=self.params.max_object_extent
            for members in od.group_segments(xy_map, labels, keep):
                if not od.plausible_group(xy_map, r, members, scan.angle_increment,
                                          face_width=self.params.min_face_width, max_extent=max_extent,
                                          frac=self.params.min_points_frac):
                    continue

                # Fit on the largest contiguous segment only
                main = od.main_segment(labels, members)
                pts = xy_map[main]
                sensor = self.lidar_position(poses, dt[main], scan_duration)
                
                # Get the best fitting heading angle, given the scan and starting from a prior
                heading = od.closeness_heading(pts, self.box_heading(pts.mean(axis=0)),
                                               search=np.radians(self.params.heading_search_deg))
                
                # Get an outline of the object that is actually seen
                rear, front, left, right = od.observe_face(pts, idx[main], r[main], ranges, sensor, heading)
                sd_all = self.to_frenet(xy_map[members])
                boxes.append(od.frenet_box(sd_all[:, 0], sd_all[:, 1], self.track_length))
                
                observations.append((pts.mean(axis=0), rear, front, left, right))
                headings.append(float(np.arctan2(np.sin(heading), np.cos(heading))))

        # Publish the accepted points and optionally the rejected ones
        self.publish_points(self.points_pub, scan.header.stamp, xy_map[keep], labels[keep], dt[keep])
        if self.rejected_pub is not None:
            self.publish_points(self.rejected_pub, scan.header.stamp, xy_map[~keep], labels[~keep], dt[~keep])
        self.publish_detections(scan.header.stamp, observations, headings, boxes)

    def to_frenet(self, xy) -> np.ndarray:
        """Map (x, y) to Frenet (s, d), wrapped to track length"""
        sd = np.full(xy.shape, np.nan)
        ok = np.isfinite(xy).all(axis=1)
        if ok.any():
            s, d = self.converter.get_frenet(xy[ok, 0], xy[ok, 1])
            sd[ok, 0] = np.asarray(s) % self.track_length
            sd[ok, 1] = d
        return sd

    def publish_points(self, pub, stamp, xy, seg, dt) -> None:
        header = Header(stamp=stamp, frame_id=self.params.map_frame)
        pub.publish(viz.points_cloud(header, xy, seg, dt))

    def publish_detections(self, stamp, observations, headings, boxes) -> None:
        """One Detection per observed object (what was seen, NaN otherwise), plus RViz markers."""
        msg = DetectionArray()
        msg.header = Header(stamp=stamp, frame_id=self.params.map_frame)
        detections = []

        if observations:
            # One conversion for all face/edge points; rows per observation: centroid, rear, front, left, right
            sd = self.to_frenet(np.array([p for obs in observations for p in obs]))
            for i in range(len(observations)):
                _, rear, front, left, right = sd[5 * i: 5 * i + 5]
                s_mid, d_mid, length, width = boxes[i]
                detections.append(Detection(
                    s_rear=float(rear[0]), s_front=float(front[0]),
                    d_left=float(left[1]), d_right=float(right[1]),
                    heading=headings[i],
                    s_center=s_mid, d_center=d_mid, length=length, width=width))

        msg.detections = detections
        self.obstacles_pub.publish(msg)
        self.markers_pub.publish(viz.observation_markers(msg.header, observations))

    def waypoints_cb(self, msg: WpntArray) -> None:
        x = np.array([w.x_m for w in msg.wpnts])
        y = np.array([w.y_m for w in msg.wpnts])
        psi = np.array([w.psi_rad for w in msg.wpnts])
        self.converter = FrenetConverter(x, y, psi)
        self.wp_s = np.array([w.s_m for w in msg.wpnts])
        self.wp_psi = np.unwrap(psi)
        self.track_length = float(self.wp_s[-1])

    def box_heading(self, xy: np.ndarray) -> float:
        """Heading prior for the box fit at xy (the raceline direction usually)."""
        assert self.converter is not None
        sd = np.asarray(self.converter.get_frenet(np.array([xy[0]]), np.array([xy[1]])))
        s = sd[0, 0] % self.track_length
        return float(np.interp(s, self.wp_s, self.wp_psi))

    def lidar_position(self, poses, dt, scan_duration) -> np.ndarray:
        """Laser position in the map frame at the mean beam time of a group."""
        map_odom, start, end = poses
        alpha = np.mean(dt) / scan_duration if scan_duration > 0 else 0.0
        return od._se2_compose(map_odom, od._se2_interp(start, end, [alpha])[0])[:2]


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
