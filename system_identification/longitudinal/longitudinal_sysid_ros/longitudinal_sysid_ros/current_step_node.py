"""Longitudinal experiment: constant motor current for a short time, then brake.

Commands the VESC driver directly, with the servo centred. It must run without `base_system`,
whose speed commands would alternate with the current commands: use `current_step_launch.xml`.

Nothing else guards the car, so the node checks the deadman itself: the experiment starts when
the button is pressed, and releasing it brakes at once. If the node dies, the VESC driver stops
the motor `command_timeout` after the last command.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class CurrentStep:
    """The experiment sequence: wait for the deadman, drive, brake, done."""

    def __init__(self, accel_time, brake_time):
        self.accel_time = accel_time
        self.brake_time = brake_time
        self.accel_start = None
        self.brake_start = None

    def step(self, now, deadman):
        """Return what to command at time `now`: None (waiting), 'current', 'brake' or 'done'."""
        if self.brake_start is None:
            if self.accel_start is None:
                if not deadman:
                    return None
                self.accel_start = now
            if deadman and now - self.accel_start < self.accel_time:
                return 'current'
            self.brake_start = now
        return 'brake' if now - self.brake_start < self.brake_time else 'done'


class CurrentStepNode(Node):
    def __init__(self):
        super().__init__('current_step')
        p = self.declare_parameters('', [
            ('current', 20.0),
            ('accel_time', 1.0),
            ('brake_current', 20.0),
            ('brake_time', 2.0),
            ('deadman_button', 5),
            ('joy_timeout', 0.2),
            ('steering_angle_to_servo_offset', Parameter.Type.DOUBLE),  # from vesc.yaml
        ])
        self.params = {param.name: param.value for param in p}
        if self.params['steering_angle_to_servo_offset'] is None:
            raise ValueError('steering_angle_to_servo_offset is not set: pass vesc.yaml')
        self.experiment = CurrentStep(self.params['accel_time'], self.params['brake_time'])
        self.pubs = {
            'current': self.create_publisher(Float64, '/commands/motor/current', 1),
            'brake': self.create_publisher(Float64, '/commands/motor/brake', 1),
        }
        self.values = {'current': self.params['current'], 'brake': self.params['brake_current']}
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 1)
        self.last_deadman = float('-inf')  # last time the deadman was seen held
        self.last_command = None
        self.create_subscription(Joy, '/joy', self.joy_cb, 1)
        # wall time on purpose: this drives hardware, and so does the deadman timeout
        self.create_timer(1.0 / 100, self.step)
        self.get_logger().warn(
            f"Hold button {self.params['deadman_button']} to start: {self.params['current']} A "
            f"for {self.params['accel_time']} s, then {self.params['brake_current']} A brake "
            f"for {self.params['brake_time']} s. Release to brake.")

    def joy_cb(self, msg):
        button = self.params['deadman_button']
        held = button < len(msg.buttons) and msg.buttons[button] == 1
        self.last_deadman = time.monotonic() if held else float('-inf')

    def step(self):
        if self.count_publishers('/commands/motor/speed'):
            self.get_logger().fatal("Something publishes speed commands: stop base_system first")
            raise SystemExit
        self.servo_pub.publish(Float64(data=self.params['steering_angle_to_servo_offset']))
        now = time.monotonic()
        command = self.experiment.step(now, now - self.last_deadman <= self.params['joy_timeout'])
        if command != self.last_command:
            self.get_logger().info(f"Phase: {command}")
            self.last_command = command
        if command == 'done':
            # the VESC driver sees the commands end, and holds the car at 0 erpm
            raise SystemExit
        if command is not None:
            self.pubs[command].publish(Float64(data=self.values[command]))


def main():
    rclpy.init()
    node = CurrentStepNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()
