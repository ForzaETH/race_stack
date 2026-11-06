"""Republish topics from another ROS_DOMAIN_ID into this one.

The node uses the local zenoh router as usual, and reaches the opponent's domain as a client of the opponent
router (opp_ip). Message types are discovered from the remote publishers, so any type works.
"""
import os
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class DomainBridge(Node):
    def __init__(self):
        super().__init__('domain_bridge')
        opp_domain = self.declare_parameter('opp_domain', -1).value
        if opp_domain < 0:
            raise SystemExit('set the opp_domain parameter to the opponent ROS_DOMAIN_ID')
        opp_ip = self.declare_parameter('opp_ip', '').value
        if not opp_ip:
            raise SystemExit('set the opp_ip parameter to the opponent IP or hostname')
        self.prefix = self.declare_parameter('prefix', '/opp').value
        self.pending = set(self.declare_parameter('topics', ['/car_state/frenet/odom']).value)

        local_override = os.environ.get('ZENOH_CONFIG_OVERRIDE')
        os.environ['ZENOH_CONFIG_OVERRIDE'] = f'mode="client";connect/endpoints=["tcp/{opp_ip}:7447"]'
        self.remote_ctx = rclpy.Context()
        rclpy.init(args=[], context=self.remote_ctx, domain_id=opp_domain)  # args=[]: ignore CLI remaps/params
        if local_override is None:
            del os.environ['ZENOH_CONFIG_OVERRIDE']
        else:
            os.environ['ZENOH_CONFIG_OVERRIDE'] = local_override
        self.remote = rclpy.create_node('domain_bridge_remote', context=self.remote_ctx)
        self.remote_exec = SingleThreadedExecutor(context=self.remote_ctx)
        self.remote_exec.add_node(self.remote)
        threading.Thread(target=self.remote_exec.spin, daemon=True).start()

        self.create_timer(1.0, self.discover)

    def discover(self):
        for topic in list(self.pending):
            infos = self.remote.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            msg_type = get_message(infos[0].topic_type)
            pub = self.create_publisher(msg_type, self.prefix + topic, 10)
            self.remote.create_subscription(msg_type, topic, pub.publish, 10)
            self.pending.discard(topic)
            self.get_logger().info(f'bridging {topic} [{infos[0].topic_type}] -> {self.prefix + topic}')

    def destroy_node(self):
        self.remote_exec.shutdown()
        rclpy.try_shutdown(context=self.remote_ctx)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DomainBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
