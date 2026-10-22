"""Collects driving data, identifies the tires on track and writes tires + steering LUT to stack_master/config."""
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from on_track_sysid import OnTrackConfig, identify_on_track
from on_track_sysid.pacejka import steady_state_forces
from forza_config_ros import get_remote_parameter
from rclpy.node import Node
from rclpy.task import Future

from lateral_sysid_ros.config_io import (data_dir, load_tires, load_vehicle, plot_tires, save_raw_data,
                                         save_results)

RATE = 50  # [Hz], the data sample time the identification assumes


class OnTrackSysIdNode(Node):
    def __init__(self):
        super().__init__('on_track_sysid')
        self.racecar_version = get_remote_parameter(self, 'global_parameters', 'racecar_version')
        self.floor = get_remote_parameter(self, 'global_parameters', 'floor')

        p = self.declare_parameters('', [
            ('odom_topic', '/state_estimation/odom'),
            ('drive_topic', '/drive'),
            ('data_duration', 50.0),
            ('min_speed', 1.0),
            ('num_of_iterations', 6),
            ('num_of_epochs', 100),
            ('lr', 5e-4),
            ('weight_decay', 0.0),
            ('equal_tires', False),
            ('vy_at_rear_axle', self.racecar_version != 'SIM'),
            ('plot_model', True),
            ('save_raw_data', False),
            ('update_latest', True),
        ])
        self.params = {param.name: param.value for param in p}
        if self.params['equal_tires']:
            self.get_logger().warn("Using 'equal_tires' is discouraged, it does not work well.")

        self.vehicle = load_vehicle(self.racecar_version)
        self.init_tires = load_tires(self.racecar_version, self.floor, self.get_logger().warn)

        self.data = np.zeros((int(self.params['data_duration'] * RATE), 4))
        self.counter = 0
        self.state = np.zeros(4)
        self.done = Future()

        self.create_subscription(Odometry, self.params['odom_topic'], self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, self.params['drive_topic'], self.drive_cb, 10)
        self.timer = self.create_timer(1.0 / RATE, self.collect)
        self.get_logger().info(f"Identifying {self.racecar_version} on {self.floor}: "
                               f"collecting {self.params['data_duration']} s above {self.params['min_speed']} m/s")

    def odom_cb(self, msg: Odometry):
        self.state[0] = msg.twist.twist.linear.x
        self.state[1] = msg.twist.twist.linear.y
        self.state[2] = msg.twist.twist.angular.z

    def drive_cb(self, msg: AckermannDriveStamped):
        self.state[3] = msg.drive.steering_angle

    def collect(self):
        if self.state[0] <= self.params['min_speed']:
            return
        self.data[self.counter] = self.state
        self.counter += 1
        if self.counter % RATE == 0:
            self.get_logger().info(f"Collected {self.counter // RATE} / {len(self.data) // RATE} s")
        if self.counter == len(self.data):
            self.timer.cancel()
            self.identify()
            self.done.set_result(True)

    def identify(self):
        log = self.get_logger()
        if self.params['save_raw_data']:
            log.info(f"Saved raw data to {save_raw_data(self.data, self.racecar_version, self.floor, 'on_track')}")

        cfg = OnTrackConfig(num_of_iterations=self.params['num_of_iterations'],
                            num_of_epochs=self.params['num_of_epochs'],
                            lr=self.params['lr'],
                            weight_decay=self.params['weight_decay'],
                            dt=1.0 / RATE,
                            vy_at_rear_axle=self.params['vy_at_rear_axle'],
                            equal_tires=self.params['equal_tires'])
        plot_stem = data_dir(self.racecar_version, self.floor) if self.params['plot_model'] else None

        def on_iteration(i, tires, sweep):
            log.info(f"Iteration {i}/{cfg.num_of_iterations}: front {np.round(tires.front, 4)}, "
                     f"rear {np.round(tires.rear, 4)}")
            if plot_stem is not None:
                alpha_f, alpha_r, _, _, F_yf, F_yr = steady_state_forces(self.vehicle, *sweep)
                plot_tires(plot_stem / f'on_track_iteration_{i}.png', self.vehicle, tires, self.init_tires,
                           (alpha_f, F_yf, alpha_r, F_yr))

        log.info("Training...")
        tires = identify_on_track(self.data, self.vehicle, self.init_tires, cfg, on_iteration)
        if plot_stem is not None:
            log.info(f"Saved fit plots to {plot_stem}")
        save_results(tires, self.vehicle, self.racecar_version, self.floor, self.params['update_latest'], log.info)


def main():
    rclpy.init()
    node = OnTrackSysIdNode()
    rclpy.spin_until_future_complete(node, node.done)
    node.destroy_node()
    rclpy.shutdown()
