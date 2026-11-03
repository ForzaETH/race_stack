#!/usr/bin/env python3
"""Republish a LaserScan with its header stamp shifted by `offset` seconds.

Replay-only helper: /scan in a bag carries the stamps urg_node produced on the
car, so the effect of a different `default_user_latency` can only be tested by
shifting them here. offset = 0 is a pure passthrough.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class ScanStampOffset(Node):

    def __init__(self):
        super().__init__('scan_stamp_offset')
        self.declare_parameter('input_topic', '/scan_bag')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('offset', 0.0)
        self.offset_ns = int(self.get_parameter('offset').value * 1e9)
        self.pub = self.create_publisher(LaserScan, self.get_parameter('output_topic').value, 10)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value, self._callback, 10)

    def _callback(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds + self.offset_ns
        msg.header.stamp = Time(nanoseconds=t).to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanStampOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
