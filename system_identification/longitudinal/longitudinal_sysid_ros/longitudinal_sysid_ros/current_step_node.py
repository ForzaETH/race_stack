"""Longitudinal experiment: constant motor current for a short time, then brake.

Commands the VESC driver directly, with the servo centred. It must run without `base_system`,
whose speed commands would alternate with the current commands: use `current_step_launch.xml`.

Nothing else guards the car, so the node checks the deadman itself: the experiment starts when
the button is pressed, and releasing it brakes at once. After the brake, release and press the
button again to repeat the experiment. If the node dies, the VESC driver stops the motor
`command_timeout` after the last command.

Between experiments the car can be driven by hand with the human deadman: the launch file runs the
usual teleop chain, with `ackermann_to_vesc` publishing on this node's `~/teleop/speed` and
`~/teleop/servo`, and the node forwards those to the VESC only while no experiment runs.
"""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class CurrentStep:
    """The experiment sequence: wait for the deadman, hold, drive, brake, done, and again.

    A run starts only on a press made while waiting, after a release seen while waiting: holding
    the button through the end of a run does not start the next one, and neither does a joystick
    timeout, which stops the deadman but is no release.

    The hold commands nothing to the motor, so that the VESC driver's speed watchdog, started when
    the manual commands stop, fires before the current starts.
    """

    def __init__(self, hold_time, accel_time, brake_time):
        self.hold_time = hold_time
        self.accel_time = accel_time
        self.brake_time = brake_time
        self.start = None
        self.brake_start = None
        self.armed = True

    def step(self, now, deadman, released):
        """Return what to command at time `now`: None (waiting), 'hold', 'current', 'brake' or
        'done'. 'done' is returned once, at the end of each run; then it waits again.

        `deadman`: the button is held and the joystick is fresh. `released`: the joystick is fresh
        and reports the button released.
        """
        if self.start is None:
            if released:
                self.armed = True
            if not (deadman and self.armed):
                return None
            self.start = now
            self.armed = False
        if self.brake_start is None:
            elapsed = now - self.start
            if elapsed < self.hold_time:
                if deadman:
                    return 'hold'
                # stopped before driving: nothing to brake
                self.start = None
                self.armed = released
                return None
            if deadman and elapsed < self.hold_time + self.accel_time:
                return 'current'
            self.brake_start = now
        if now - self.brake_start < self.brake_time:
            return 'brake'
        self.start = self.brake_start = None
        return 'done'


class CurrentStepNode(Node):
    def __init__(self):
        super().__init__('current_step')
        p = self.declare_parameters('', [
            ('hold_time', 0.1),
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
        self.experiment = CurrentStep(
            self.params['hold_time'], self.params['accel_time'], self.params['brake_time'])
        self.pubs = {
            'current': self.create_publisher(Float64, '/commands/motor/current', 1),
            'brake': self.create_publisher(Float64, '/commands/motor/brake', 1),
        }
        self.values = {'current': self.params['current'], 'brake': self.params['brake_current']}
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 1)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 1)
        self.running = False  # an experiment runs: manual commands are dropped
        self.last_joy = float('-inf')  # last time a joystick message arrived
        self.held = False  # the deadman, in the last joystick message
        self.last_command = None
        self.create_subscription(Joy, '/joy', self.joy_cb, 1)
        self.create_subscription(Float64, '~/teleop/speed', self.teleop_cb(self.speed_pub), 1)
        self.create_subscription(Float64, '~/teleop/servo', self.teleop_cb(self.servo_pub), 1)
        # steady time on purpose, never sim time: this drives hardware, and so does the deadman
        # timeout
        self.create_timer(1.0 / 100, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warn(
            f"Hold button {self.params['deadman_button']} to start: {self.params['current']} A "
            f"for {self.params['accel_time']} s, then {self.params['brake_current']} A brake "
            f"for {self.params['brake_time']} s. Release to brake. Release and hold again to "
            "repeat. Drive by hand in between with the human deadman.")

    def joy_cb(self, msg):
        button = self.params['deadman_button']
        self.held = button < len(msg.buttons) and msg.buttons[button] == 1
        self.last_joy = time.monotonic()

    def teleop_cb(self, pub):
        def forward(msg):
            if not self.running:
                pub.publish(msg)
        return forward

    def step(self):
        # this node has one publisher there: any other, even from a node with the same name, is not
        # this node's
        if self.count_publishers('/commands/motor/speed') > 1:
            self.get_logger().fatal(
                "Something else publishes speed commands: stop base_system first")
            raise SystemExit
        now = time.monotonic()
        fresh = now - self.last_joy <= self.params['joy_timeout']
        command = self.experiment.step(now, fresh and self.held, fresh and not self.held)
        if command != self.last_command:
            self.get_logger().info(f"Phase: {command}")
            if command == 'done':
                self.get_logger().warn("Done: release and hold the button again to repeat")
            self.last_command = command
        # after a run the VESC driver sees the current commands end, and holds the car at 0 erpm
        # until manual speed commands arrive
        self.running = command in ('hold', 'current', 'brake')
        if self.running:
            self.servo_pub.publish(Float64(data=self.params['steering_angle_to_servo_offset']))
        if command in self.pubs:
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
