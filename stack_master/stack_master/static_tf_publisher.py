"""Publishes the static transforms listed in a per-car yaml file.

File format (translation in metres, rotation as rpy in radians or quat [x, y, z, w]):

    transforms:
      - {parent: base_link, child: laser, xyz: [0.27, 0.0, 0.11], rpy: [0.0, 0.0, 0.0]}
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        config_file = self.declare_parameter('config_file', '').value
        with open(config_file) as f:
            transforms = yaml.safe_load(f)['transforms']

        stamp = self.get_clock().now().to_msg()
        msgs = []
        for t in transforms:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = t['parent']
            msg.child_frame_id = t['child']
            x, y, z = t.get('xyz', (0.0, 0.0, 0.0))
            qx, qy, qz, qw = t['quat'] if 'quat' in t else quat_from_rpy(*t.get('rpy', (0.0, 0.0, 0.0)))
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = float(x), float(y), float(z)
            msg.transform.rotation.x, msg.transform.rotation.y = float(qx), float(qy)
            msg.transform.rotation.z, msg.transform.rotation.w = float(qz), float(qw)
            msgs.append(msg)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msgs)
        self.get_logger().info(f'Published {len(msgs)} static transforms from {config_file}')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
