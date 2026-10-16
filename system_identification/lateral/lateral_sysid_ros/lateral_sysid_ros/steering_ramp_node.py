"""Steady-state tire experiment: drive at constant speed while slowly ramping the steering angle.

Record a bag while it runs, then identify with `steady_state_from_bag`.
"""
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class SteeringRampNode(Node):
    def __init__(self):
        super().__init__('steering_ramp')
        p = self.declare_parameters('', [
            ('drive_topic', '/drive'),
            ('const_speed', 3.0),
            ('start_angle', -0.15),
            ('end_angle', -0.4),
            ('ramp_time', 20.0),
        ])
        self.params = {param.name: param.value for param in p}
        self.pub = self.create_publisher(AckermannDriveStamped, self.params['drive_topic'], 10)
        self.start = self.get_clock().now()
        self.warned = False
        self.create_timer(1.0 / 30, self.step)
        self.get_logger().warn(f"Driving at {self.params['const_speed']} m/s, steering from "
                               f"{self.params['start_angle']} to {self.params['end_angle']} rad "
                               f"in {self.params['ramp_time']} s")

    def step(self):
        frac = (self.get_clock().now() - self.start).nanoseconds * 1e-9 / self.params['ramp_time']
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if frac <= 1.0:
            msg.drive.speed = self.params['const_speed']
            msg.drive.steering_angle = self.params['end_angle'] * frac + self.params['start_angle'] * (1 - frac)
            if frac >= 0.9 and not self.warned:
                self.get_logger().warn("Ending soon, stop the bag")
                self.warned = True
        else:
            # stop the car once the ramp is over
            self.get_logger().info("Experiment over", once=True)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SteeringRampNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
