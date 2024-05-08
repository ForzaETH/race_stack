import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from f110_msgs.msg import ObstacleArray, WpntArray  # Assuming f110_msgs are available in your ROS2 environment
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf_transformations import quaternion_from_euler

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collisiond_detector')
        
        self.glb_waypoints = None
        
        self.sub_obstacles = self.create_subscription(ObstacleArray, "/perception/obstacles", self.od_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/car_state/odom_frenet", self.odom_cb, 10)
        self.sub_global_waypoints = self.create_subscription(WpntArray, '/global_waypoints', self.glb_wpnts_cb, 10)
        self.pub_collision = self.create_publisher(Bool, "/opponent_collision", 10)
        self.pub_distance = self.create_publisher(Float32, "/opponent_dist", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/collision_marker", 10)

        self.first_visualization = True
        self.x_viz = 0.0
        self.y_viz = 0.0
        self.viz_q = quaternion_from_euler(0, 0, 0)
        self.viz_counter = 0
        self.obs_arr = ObstacleArray()
        self.car_odom = Pose()
       
        
        while self.glb_waypoints is None:
            self.get_logger().info("Waiting for global waypoints...")
            rclpy.spin_once(self)
            
        self.timer = self.create_timer(0.02, self.loop)  # 50Hz
        
        
    def od_cb(self, data):
        self.obs_arr = data

    def odom_cb(self, data):
        self.car_odom = data.pose.pose

    def glb_wpnts_cb(self, data):
        self.glb_waypoints = data.wpnts
        
    def loop(self):
        obs_arr = self.obs_arr
        car_odom = self.car_odom

        collision_bool, min_dist_s, min_dist_d = self.collision_check(obs_arr=obs_arr, car_odom=car_odom)

        if self.viz_counter > 0:
            self.viz_counter -= 1
            if self.viz_counter == 0:
                self.viz_collision(clear=True)

        col_msg = Bool()
        col_msg.data = collision_bool
        self.pub_collision.publish(col_msg)

        if collision_bool:
            self.viz_counter = 100  # 2 seconds at 50Hz
            self.viz_collision(dist_s=min_dist_s, dist_d=min_dist_d, clear=False)

        dist_msg = Float32()
        dist_msg.data = np.sqrt(min_dist_s**2 + min_dist_d**2)
        self.pub_distance.publish(dist_msg)

    def collision_check(self, obs_arr, car_odom):
        for obs in obs_arr.obstacles:
            car_s = car_odom.position.x
            car_d = car_odom.position.y

            od_s = obs.s_center
            od_d = obs.d_center

            if (od_s - car_s) % self.glb_waypoints[-2].s_m < 0.55 and abs(car_d - od_d) < 0.35:
                return True, (od_s - car_s) % self.glb_waypoints[-2].s_m, abs(car_d - od_d)
            if (car_s - od_s) % self.glb_waypoints[-2].s_m < 0.25 and abs(car_d - od_d) < 0.30:
                return True, (car_s - od_s) % self.glb_waypoints[-2].s_m, abs(car_d - od_d)
        return False, 100, 100

    def viz_collision(self, dist_s=0, dist_d=0, clear=False):
        if self.first_visualization and not clear:
            self.first_visualization = False
            idx = len(self.glb_waypoints)//4
            x0, y0 = self.glb_waypoints[idx].x_m, self.glb_waypoints[idx].y_m
            x1, y1 = self.glb_waypoints[idx + 1].x_m, self.glb_waypoints[idx + 1].y_m
            xy_norm = -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.75 * self.glb_waypoints[idx].d_left
            yaw = np.arctan2(xy_norm[1], xy_norm[0])
            self.viz_q = quaternion_from_euler(0, 0, yaw)
            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        coll_mrk = MarkerArray()
        marker_text = Marker()
        marker_text.header.frame_id = "map"
        marker_text.type = marker_text.TEXT_VIEW_FACING
        marker_text.text = f"COLLISION: dist_s :{dist_s:.1f}, dist_d :{dist_d:.1f}m" if not clear else ""
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

        self.pub_markers.publish(coll_mrk)
        
def main(args=None):
    rclpy.init(args=args)
    detector = CollisionDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
