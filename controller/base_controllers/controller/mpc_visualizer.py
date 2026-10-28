import numpy as np
import rclpy
from rclpy.node import Node
from stack_master.qos import LATCHED_QOS

from f110_msgs.msg import WpntArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.frenet_converter import FrenetConverter

# number of states per horizon node, the first ones are always s, n, alpha/theta, v
STATE_DIMS = {"KMPC": 5, "STMPC": 8}


class MPCVisualizer(Node):
    """Shows the MPC predicted horizon in rviz, from the flattened states of /mpc_controller/states."""

    def __init__(self):
        super().__init__('mpc_visualizer')
        self.nx = STATE_DIMS[self.declare_parameter('mode', 'STMPC').value]
        self.converter = None
        self.v_max = 1.0

        self.marker_pub = self.create_publisher(MarkerArray, '/mpc_controller/horizon_markers', 10)
        self.create_subscription(WpntArray, '/global_waypoints', self.global_waypoints_cb, LATCHED_QOS)
        self.create_subscription(Float32MultiArray, '/mpc_controller/states', self.states_cb, 10)

    def global_waypoints_cb(self, data: WpntArray):
        wpnts = np.array([[w.x_m, w.y_m, w.psi_rad, w.vx_mps] for w in data.wpnts])
        self.converter = FrenetConverter(wpnts[:, 0], wpnts[:, 1], wpnts[:, 2])
        self.v_max = max(float(wpnts[:, 3].max()), 1.0)

    def states_cb(self, data: Float32MultiArray):
        if self.converter is None:
            return
        states = np.asarray(data.data).reshape(-1, self.nx)
        # the horizon runs past the finish line
        s = states[:, 0] % self.converter.raceline_length
        xy = self.converter.get_cartesian(s, states[:, 1])
        points = [Point(x=float(x), y=float(y), z=0.05) for x, y in zip(xy[0], xy[1])]
        # red slow, green fast w.r.t. the global raceline top speed
        ratios = np.clip(states[:, 3] / self.v_max, 0.0, 1.0)
        colors = [ColorRGBA(r=float(1 - r), g=float(r), b=0.0, a=1.0) for r in ratios]

        stamp = self.get_clock().now().to_msg()
        line = Marker(type=Marker.LINE_STRIP, id=0, points=points, colors=colors)
        line.scale.x = 0.03
        nodes = Marker(type=Marker.SPHERE_LIST, id=1, points=points, colors=colors)
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.08
        for marker in (line, nodes):
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'mpc_horizon'
            marker.pose.orientation.w = 1.0
        self.marker_pub.publish(MarkerArray(markers=[line, nodes]))


def main():
    rclpy.init()
    node = MPCVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()
