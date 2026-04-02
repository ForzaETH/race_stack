import rclpy
from rclpy.node import Node
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np
from frenet_conversion.frenet_converter import FrenetConverter


class ObstaclePublisher(Node):
    """Publish a dynamic obstacle in the F110 simulator
    """
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.ego_wpnts = None
        self.max_s = None
        self.waypoints_topic = None
        self.wpnts = None
        
        

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('speed_scaler', 0.5),
                ('constant_speed', False),
                ('trajectory', "min_curv"),
                ('start_s', 0.0),
        ])
        
        self.speed_scaler = self.get_parameter('speed_scaler').value
        self.constant = self.get_parameter('constant_speed').value
        self.waypoints_type = self.get_parameter('trajectory').value
        self.starting_s = float(self.get_parameter('start_s').value)

        # choose trajectory
        if self.waypoints_type == 'min_curv':
            self.waypoints_topic = '/global_waypoints'
        elif self.waypoints_type == 'shortest_path':
            self.waypoints_topic = '/global_waypoints/shortest_path'
        elif self.waypoints_type == 'centerline':
            self.waypoints_topic = '/centerline_waypoints'
        else:
            self.get_logger().error('Invalid trajectory type')
        

        # Subscriptions and Publishers
        self.odom_subscriber = self.create_subscription(Odometry, '/car_state/odom_frenet', self.odom_cb, 10)
        self.waypoints_subscriber = self.create_subscription(WpntArray, self.waypoints_topic, self.waypoints_cb, 10)
        self.global_waypoints_subscriber = self.create_subscription(WpntArray, '/global_waypoints', self.global_waypoints_cb, 10)

        self.virtual_obstacle_pub = self.create_publisher(ObstacleArray, '/perception/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dummy_obstacle_markers', 10)
        self.opponent_traj_pub = self.create_publisher(OpponentTrajectory, '/opponent_waypoints', 10)

        # Frenet Conversion Service Clients
        self.converter = None
        
        self.dynamic_obstacle = self.init_dynamic_obstacle()
        self.obj_len = 0.5
        
        self.loop_setup()

    def init_dynamic_obstacle(self):
        """ Initializes the dynamic obstacle configuration."""
        dynamic_obstacle = Obstacle()
        dynamic_obstacle.id = 1
        dynamic_obstacle.d_right = -0.1
        dynamic_obstacle.d_left = 0.1
        dynamic_obstacle.is_actually_a_gap = False
        
        return dynamic_obstacle
    
    ### Callbacks ###
    def global_waypoints_cb(self, data: WpntArray):
        self.ego_wpnts = data.wpnts[:-1]
        self.max_s = self.ego_wpnts[-1].s_m
        
        if self.converter == None:
            waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad]
                                  for wpnt in data.wpnts])
            self.converter = FrenetConverter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])
    
    def waypoints_cb(self, data: WpntArray):
        self.wpnts = data.wpnts[:-1]

    def odom_cb(self, data: Odometry):
        """ Odometry callback to track the car's state."""
        self.car_odom = data
        
    ### Helpers ###
    
    def publish_obstacle_cartesian(self, obstacles):
        """Publishes obstacles as markers in the Cartesian frame."""
        marker_array = MarkerArray()
        for obs in obstacles:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.SPHERE
            marker.id = obs.id
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker_xy = self.converter.get_cartesian(obs.s_center, obs.d_center)
            marker.pose.position.x = marker_xy[0]
            marker.pose.position.y = marker_xy[1]
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def loop_setup(self):
        """ Main loop for updating and publishing the dynamic obstacle."""
        
        # Wait for global waypoint message to arrive
        while self.ego_wpnts is None: # equivalent of wait for message
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)
            
        s_array = np.array([wpnt.s_m for wpnt in self.ego_wpnts])
        self.get_logger().info("Got global waypoints message")
        
        # Read in opponent waypoints
        if self.constant:
            for i in range(len(self.ego_wpnts)):
                self.ego_wpnts[i].vx_mps = self.speed_scaler
        else:
            for i in range(len(self.ego_wpnts)):
                self.ego_wpnts[i].vx_mps = self.ego_wpnts[i].vx_mps * self.speed_scaler
                
        # Wait for opponent waypoints message to arrive
        while self.wpnts is None:
            self.get_logger().info("Waiting for opponent waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)
        
        
        opponent_frenet = self.converter.get_frenet([wpnt.x_m for wpnt in self.wpnts], [wpnt.y_m for wpnt in self.wpnts])
        opponent_s = opponent_frenet[0]
        opponent_d = opponent_frenet[1]
        # self.get_logger().info("opponent d: {}".format(opponent_d))
        sorted_indices = sorted(range(len(opponent_s)), key=lambda k: opponent_s[k])
        opponent_s_sorted = [opponent_s[i] for i in sorted_indices]
        opponent_d_sorted = [opponent_d[i] for i in sorted_indices]
        
        resampled_opponent_d = np.interp(s_array, opponent_s_sorted, opponent_d_sorted)
        resampled_opponent_vs = [wpnt.vx_mps for wpnt in self.ego_wpnts]
        resampled_opponent_xy = self.converter.get_cartesian(s_array, resampled_opponent_d)
        
        self.opponent_wpnts = OpponentTrajectory()
        for i in range(len(s_array)):
            wpnt = OppWpnt()
            wpnt.x_m = resampled_opponent_xy[0][i]
            wpnt.y_m = resampled_opponent_xy[1][i]
            wpnt.proj_vs_mps = resampled_opponent_vs[i]
            wpnt.s_m = s_array[i]
            wpnt.d_m = resampled_opponent_d[i]
            self.opponent_wpnts.oppwpnts.append(wpnt)
            
        self.dynamic_obstacle.s_center = self.starting_s
        
        self.opponent_s_array = np.array([wpnt.s_m for wpnt in self.opponent_wpnts.oppwpnts])
        self.get_logger().info("Obstacle Publisher Ready")
        
        self.counter = 0
        
        self.looptime = 1/50
        self.timer = self.create_timer(self.looptime, self.timer_cb)
        
    
    ### MAIN LOOP ###   
        
    def timer_cb(self):
        
        # Publish Obstacle message
        obstacle_msg = ObstacleArray()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'frenet'
        
        s = self.dynamic_obstacle.s_center
        approx_idx = np.abs(self.opponent_s_array - s).argmin()
        
        self.dynamic_obstacle_speed = self.opponent_wpnts.oppwpnts[approx_idx].proj_vs_mps
        self.dynamic_obstacle.s_center = (self.dynamic_obstacle.s_center + self.dynamic_obstacle_speed * self.looptime) % self.max_s
        self.dynamic_obstacle.s_start = (self.dynamic_obstacle.s_center - self.obj_len/2) % self.max_s
        self.dynamic_obstacle.s_end = (self.dynamic_obstacle.s_center + self.obj_len/2) % self.max_s
        self.dynamic_obstacle.d_center = self.opponent_wpnts.oppwpnts[approx_idx].d_m
        self.dynamic_obstacle.d_right = self.dynamic_obstacle.d_center - 0.1
        self.dynamic_obstacle.d_left = self.dynamic_obstacle.d_center + 0.1
        self.dynamic_obstacle.vs = self.dynamic_obstacle_speed
        
        obstacle_msg.obstacles.append(self.dynamic_obstacle)
        
        self.publish_obstacle_cartesian(obstacle_msg.obstacles)
        self.virtual_obstacle_pub.publish(obstacle_msg)
        
        self.counter += 1
        
        if self.counter > 25:
            # Lap count has to be bigger than 1 to show that the trajectory is updated after one lap
            opponent_traj_msg = OpponentTrajectory()
            opponent_traj_msg.header.stamp = self.get_clock().now().to_msg()
            opponent_traj_msg.header.frame_id = 'frenet'
            opponent_traj_msg.lap_count = 2
            opponent_traj_msg.oppwpnts = self.opponent_wpnts.oppwpnts
            self.opponent_traj_pub.publish(opponent_traj_msg)
            self.counter = 0

def main(args=None):
    
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher()
    rclpy.spin(obstacle_publisher)
    obstacle_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
