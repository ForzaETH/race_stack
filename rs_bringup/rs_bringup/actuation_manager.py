"""ROS node exposing the physical command actually allowed for the car."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from .actuation_limiter import ActuationLimiter
from .actuation_limiter import ActuationLimits
from .actuation_limiter import select_command_source
from .actuation_limiter import vesc_range_to_physical


class ActuationManager(Node):
    """Limit selected Ackermann commands and enforce the joystick deadman."""

    def __init__(self):
        super().__init__('actuation_manager')

        self.declare_parameter('navigation_topic', 'drive')
        self.declare_parameter('teleop_topic', 'teleop')
        self.declare_parameter('output_topic', 'ackermann_cmd')
        self.declare_parameter('output_rate', 75.0)
        self.declare_parameter('max_acceleration', 2.5)
        self.declare_parameter('max_deceleration', 2.5)
        self.declare_parameter('max_steering_rate', 3.2)
        self.declare_parameter('command_timeout', 0.2)
        self.declare_parameter('joy_timeout', 0.2)
        self.declare_parameter('human_deadman_button', 4)
        self.declare_parameter('autonomous_deadman_button', 5)
        self.declare_parameter('frame_id', 'base_link')

        self.output_rate = self._positive_parameter('output_rate')
        self.command_timeout = self._positive_parameter('command_timeout')
        self.joy_timeout = self._positive_parameter('joy_timeout')
        self.human_deadman_button = int(
            self.get_parameter('human_deadman_button').value)
        self.autonomous_deadman_button = int(
            self.get_parameter('autonomous_deadman_button').value)
        if min(self.human_deadman_button,
               self.autonomous_deadman_button) < 0:
            raise ValueError('deadman button indices must be non-negative')
        if self.human_deadman_button == self.autonomous_deadman_button:
            raise ValueError('human and autonomous deadman buttons must differ')

        # Speed and steering bounds are what the VESC can reach, derived from
        # the shared vesc parameters so they cannot drift from the calibration.
        # speed_min/speed_max are the VESC's ERPM limits here.
        speed_min, speed_max = vesc_range_to_physical(
            self._vesc_parameter('speed_min'),
            self._vesc_parameter('speed_max'),
            self._vesc_parameter('speed_to_erpm_gain'),
            self._vesc_parameter('speed_to_erpm_offset'))
        steering_min, steering_max = vesc_range_to_physical(
            self._vesc_parameter('servo_min'),
            self._vesc_parameter('servo_max'),
            self._vesc_parameter('steering_angle_to_servo_gain'),
            self._vesc_parameter('steering_angle_to_servo_offset'))
        self.get_logger().info(
            f'Speed limits [{speed_min:.3f}, {speed_max:.3f}] m/s, '
            f'steering limits [{steering_min:.4f}, {steering_max:.4f}] rad')

        limits = ActuationLimits(
            speed_min=speed_min,
            speed_max=speed_max,
            max_acceleration=self._positive_parameter('max_acceleration'),
            max_deceleration=self._positive_parameter('max_deceleration'),
            steering_min=steering_min,
            steering_max=steering_max,
            max_steering_rate=self._positive_parameter(
                'max_steering_rate'),
        )
        self.limiter = ActuationLimiter(limits)

        navigation_topic = self.get_parameter('navigation_topic').value
        teleop_topic = self.get_parameter('teleop_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publisher = self.create_publisher(
            AckermannDriveStamped, output_topic, 1)
        self.create_subscription(AckermannDriveStamped, navigation_topic,
                                 self._navigation_callback, 1)
        self.create_subscription(AckermannDriveStamped, teleop_topic,
                                 self._teleop_callback, 1)
        self.create_subscription(Joy, '/joy', self._joy_callback, 1)

        self.navigation_command = (0.0, 0.0)
        self.teleop_command = (0.0, 0.0)
        self.last_navigation_time = None
        self.last_teleop_time = None
        self.last_joy_time = None
        self.human_deadman = False
        self.autonomous_deadman = False
        self.last_tick_time = time.monotonic()
        self.last_source = object()
        self.create_timer(1.0 / self.output_rate, self._publish_applied_command)

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _vesc_parameter(self, name):
        self.declare_parameter(name, Parameter.Type.DOUBLE)
        return float(self.get_parameter(name).value)

    def _read_command(self, message):
        speed = float(message.drive.speed)
        steering = float(message.drive.steering_angle)
        if not math.isfinite(speed) or not math.isfinite(steering):
            self.get_logger().error('Ignoring non-finite Ackermann command')
            return None
        if message.header.frame_id:
            self.frame_id = message.header.frame_id
        return speed, steering

    def _navigation_callback(self, message):
        command = self._read_command(message)
        if command is not None:
            self.navigation_command = command
            self.last_navigation_time = time.monotonic()

    def _teleop_callback(self, message):
        command = self._read_command(message)
        if command is not None:
            self.teleop_command = command
            self.last_teleop_time = time.monotonic()

    def _joy_callback(self, message):
        self.last_joy_time = time.monotonic()
        self.human_deadman = self._button_is_held(
            message, self.human_deadman_button)
        self.autonomous_deadman = self._button_is_held(
            message, self.autonomous_deadman_button)

    @staticmethod
    def _button_is_held(message, button):
        return button < len(message.buttons) and message.buttons[button] == 1

    def _publish_applied_command(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_tick_time)
        self.last_tick_time = now

        source = select_command_source(
            now,
            self.last_joy_time,
            self.human_deadman,
            self.autonomous_deadman,
            self.last_teleop_time,
            self.last_navigation_time,
            self.command_timeout,
            self.joy_timeout,
        )
        self._report_source_change(source)
        command = (0.0, 0.0)
        if source == 'teleop':
            command = self.teleop_command
        elif source == 'navigation':
            command = self.navigation_command

        state = self.limiter.step(
            command[0],
            command[1],
            dt,
            enabled=source is not None,
        )
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.drive.speed = state.speed
        message.drive.steering_angle = state.steering_angle
        self.publisher.publish(message)

    def _report_source_change(self, source):
        if source == self.last_source:
            return
        self.last_source = source
        if source is None:
            self.get_logger().warning('Actuation disabled')
        else:
            self.get_logger().info(f'Actuation source: {source}')


def main(args=None):
    """Run the actuation manager node."""
    rclpy.init(args=args)
    node = ActuationManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
