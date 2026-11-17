#!/usr/bin/env python3
"""Republish selected static transforms recorded in a bag.

Used by the bag replay: the RealSense driver publishes its internal TF tree
(camera_link -> camera_gyro_frame -> camera_imu_frame -> camera_imu_optical_frame)
from device extrinsics, and the Basalt->EKF bridge needs it. The driver is not
running in replay, so the transforms are taken from the bag's /tf_static instead.
Only transforms whose parent frame matches `frame_regex` are published, the car
mounts keep coming from static_transforms.yaml.
"""

import re

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from tf2_ros import StaticTransformBroadcaster


class BagStaticTf(Node):

    def __init__(self):
        super().__init__('bag_static_tf')
        self.declare_parameter('bag', '')
        self.declare_parameter('frame_regex', '^camera_')
        bag = self.get_parameter('bag').value
        pattern = re.compile(self.get_parameter('frame_regex').value)

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        found = {}
        with Reader(bag) as reader:
            conns = [c for c in reader.connections if c.topic == '/tf_static']
            for conn, _, raw in reader.messages(connections=conns):
                for tr in typestore.deserialize_cdr(raw, conn.msgtype).transforms:
                    if pattern.search(tr.header.frame_id):
                        found[(tr.header.frame_id, tr.child_frame_id)] = tr

        msgs = []
        for (parent, child), tr in found.items():
            m = TransformStamped()
            m.header.frame_id = parent
            m.child_frame_id = child
            t, q = tr.transform.translation, tr.transform.rotation
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t.x, t.y, t.z
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q.x, q.y, q.z, q.w
            msgs.append(m)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msgs)
        self.get_logger().info(f'published {len(msgs)} static transforms from {bag}: '
                               + ', '.join(f'{p}->{c}' for p, c in found))


def main(args=None):
    rclpy.init(args=args)
    node = BagStaticTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
