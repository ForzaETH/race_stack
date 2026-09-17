#!/usr/bin/env python3
import os
import threading
import time

import numpy as np
import rclpy
import tf_transformations as tft
from ackermann_msgs.msg import AckermannDriveStamped
from filterpy.kalman import ExtendedKalmanFilter
from nav_msgs.msg import Odometry
from pbl_config import load_car_config_ros, load_pacejka_tire_config_ros, get_remote_parameter
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from . import models
from .sensors import ImuSensor, OdomSensor


class EkfNode(Node):
    """
    EKF Node implementing an Extended Kalman Filter. Applies sequential updates to the filter with newest sensor data.

    Measurement data used:
        - IMU: [psi, vpsi, ax, ay]
        - VESC: [x, y, psi, vx, vy, vpsi]
        - VIO: [x, y, psi, vx, vy, vpsi]
        - ACKERMANN: [accel, steer_angle]

    Sensors are configured in a single list in __init__ (see self.sensors); adding
    a new sensor is one line there.
    """

    def __init__(self):
        super().__init__('ekf_node')

        self.declare_parameter('frequency', 100)
        self.declare_parameter('model_type', 'point_mass_model')
        self.declare_parameter('odom_topic', '/state_estimation/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('vesc_topic', '/odom')
        self.declare_parameter('vio_topic', '/basalt/odom')
        self.declare_parameter('send_transform', False)
        self.declare_parameter('floor', rclpy.Parameter.Type.STRING)
        self.declare_parameter('R_imu', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('R_vesc', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('R_vio', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('Q', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.Hz = self.get_parameter('frequency').value
        self.racecar_version = os.environ.get('RACECAR_VERSION', 'NUC6')
        self.floor = get_remote_parameter(self, 'global_parameters', 'floor', default='dubi')

        self.model_type = self.get_parameter('model_type').value
        self.ODOM_TOPIC = self.get_parameter('odom_topic').value
        self.IMU_TOPIC = self.get_parameter('imu_topic').value
        self.VESC_TOPIC = self.get_parameter('vesc_topic').value
        self.VIO_TOPIC = self.get_parameter('vio_topic').value
        
        self.send_transform = self.get_parameter('send_transform').value

        self.get_logger().info(f"[EKF Node] Racecar is: {self.racecar_version}")

        self.tf_broadcaster = TransformBroadcaster(self)


        # Get covariance matrices
        self.R_imu = np.diag(self.get_parameter('R_imu').value)
        self.R_vesc = np.diag(self.get_parameter('R_vesc').value)
        self.R_vio = np.diag(self.get_parameter('R_vio').value)
        self.Q = np.diag(self.get_parameter('Q').value)

        # Get Car Parameters
        tire_config = load_pacejka_tire_config_ros(self.racecar_version, self.floor)
        car_config = load_car_config_ros(self.racecar_version)
        self.get_logger().info(f"[EKF Node] Loaded Config with: {self.racecar_version} and {self.floor}")
        self.p = {
            # Tire Parameters
            'Bf': tire_config.Bf,
            'Cf': tire_config.Cf,
            'Df': tire_config.Df,
            'Ef': tire_config.Ef,
            'Br': tire_config.Br,
            'Cr': tire_config.Cr,
            'Dr': tire_config.Dr,
            'Er': tire_config.Er,
            'mu': tire_config.friction_coeff,

            # Car Physical Configs
            'm': car_config.m,
            'I_z': car_config.Iz,
            'l_f': car_config.lf,
            'l_r': car_config.lr,
            'h_cg': car_config.h_cg,
            'wheelbase': car_config.wheelbase
        }

        # Build the right Model Object
        if self.model_type == "point_mass_model":
            self.model = models.point_mass_model(self.Q)
        elif self.model_type == "kinematic_bicycle_model":
            self.model = models.kinematic_bicycle_model(self.p, self.Q)
        elif self.model_type == "single_track_model":
            self.model = models.single_track_model(self.p, self.Q)
        else:
            self.get_logger().warn("[EKF Node] Unknown Model provided. Using point-mass motion model instead")
            self.model = models.point_mass_model(self.Q)

        # Initialize EKF
        # dim_z is just for initializaiton (can change depending on update)
        self.ekf = ExtendedKalmanFilter(dim_x=self.model.dim_x, dim_z=4)
        self.ekf.x = np.array(self.model.x_0, dtype=float)
        self.ekf.P = np.array(self.model.P_0, dtype=float)

        # Control input array
        self.ackermann_data = np.zeros(2)  # [accel, steer_angle]

        # General stuff
        self.lock = threading.Lock()
        self.last_filter_time = None
        self.is_initialized = False

        # Commanded velocity deriviation
        self.last_cmd_vel = 0.0
        self.last_cmd_time = None

        # Sensors. Each sensor owns its own subscription, buffering and EKF update.
        # Adding a new sensor is a single line here. VIO shares the full-odometry
        # measurement model with the VESC.
        # ``relative=True`` zero-references a sensor to its own first measurement,
        # so it contributes no absolute origin. The IMU needs it: its heading comes
        # from the VESC AHRS and shares no reference with the odometry sources.
        self.sensors = [
            ImuSensor(self, self.IMU_TOPIC, self.R_imu,
                      self.model.Hx_imu, self.model.HJacobian_imu,
                      on_init=self.init_state_from_imu, relative=True),
            OdomSensor(self, 'vesc', self.VESC_TOPIC, self.R_vesc,
                       self.model.Hx_vesc, self.model.HJacobian_vesc,
                       relative=False),
            OdomSensor(self, 'vio', self.VIO_TOPIC, self.R_vio,
                       self.model.Hx_vesc, self.model.HJacobian_vesc,
                       relative=False),
        ]

        # Control input subscriptions
        self.create_subscription(AckermannDriveStamped, '/drive', self.ackermann_callback, 1)
        self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self.ackermann_callback, 1)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.ODOM_TOPIC, 1)

        # Evaulating computational expences
        self.predict_count = 0
        self.predict_mean_ms = 0.0
        self.predict_max_ms = 0.0

        self.tot_update_count = 0
        self.tot_update_mean_ms = 0.0
        self.tot_update_max_ms = 0.0

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.timer = self.create_timer(1.0 / self.Hz, self.timer_callback)

        self.get_logger().info(f"[EKF Node] EKF node initialized sesfully using {self.model_type}")

    def parameter_callback(self, params):
        self.get_logger().info(f"[EKF Node] Parameters updated: {params}")
        result = SetParametersResult()
        result.successful = True  # Assume success by default
        return result

    def init_state_from_imu(self, imu_data):
        """Seed the filter state from the first IMU orientation reading."""
        if self.model_type == "single_track_model" or self.model_type == "point_mass_model":
            self.ekf.x[2] = imu_data[0]
            self.ekf.x[5] = imu_data[1]
            self.ekf.P[2, 2] = self.R_imu[0][0]
            self.ekf.P[5, 5] = self.R_imu[1][1]
        elif self.model_type == "kinematic_bicycle_model":
            self.ekf.x[4] = imu_data[0]
            self.ekf.x[5] = imu_data[1]
            self.ekf.P[4, 4] = self.R_imu[0][0]
            self.ekf.P[5, 5] = self.R_imu[1][1]

        self.get_logger().info(f"[EKF Node] Initial yaw set to {imu_data[0]}")
        self.get_logger().info(f"[EKF Node] Initial omega set to {imu_data[1]}")
        self.is_initialized = True

    # Callbacks
    def ackermann_callback(self, msg):
        with self.lock:
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            current_vel = msg.drive.speed

            if self.last_cmd_time is None:
                calculated_accel = 0.0
            else:
                dt = now - self.last_cmd_time

                if dt > 1e-4:
                    raw_accel = (current_vel - self.last_cmd_vel) / dt
                    calculated_accel = np.clip(raw_accel, -15.0, 15.0)
                else:
                    calculated_accel = self.ackermann_data[0]

            self.last_cmd_vel = current_vel
            self.last_cmd_time = now

            self.ackermann_data[0] = calculated_accel
            self.ackermann_data[1] = -msg.drive.steering_angle

    # Helper functions
    def shutdown(self):
        print(f"--- EKF Performance Report ---")
        print(f"**** Predict ****")
        print(f"Predict Steps Run: {self.predict_count}")
        print(f"Mean Execution Time: {self.predict_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.predict_max_ms:.3f} ms")
        print(f"**** Update Total****")
        print(f"Update Steps Run: {self.tot_update_count}")
        print(f"Mean Execution Time: {self.tot_update_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.tot_update_max_ms:.3f} ms")
        for sensor in self.sensors:
            print(f"**** Update {sensor.name.upper()} ****")
            print(f"Update Steps Run: {sensor.update_count}")
            print(f"Mean Execution Time: {sensor.update_mean_ms:.3f} ms")
            print(f"Peak Execution Time: {sensor.update_max_ms:.3f} ms")
        self.get_logger().info("Shutting down...")

    def publish_odometry(self):
        state = self.model.get_odom_state(self.ekf.x)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]

        q = tft.quaternion_from_euler(0, 0, state[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = state[3]
        odom.twist.twist.linear.y = state[4]
        odom.twist.twist.angular.z = state[5]

        # Covariances
        pose_cov = np.zeros((6, 6))
        pose_cov[0:2, 0:2] = self.ekf.P[0:2, 0:2]
        pose_cov[0:2, 5] = self.ekf.P[0:2, 2]
        pose_cov[5, 0:2] = self.ekf.P[2, 0:2]
        pose_cov[5, 5] = self.ekf.P[2, 2]
        odom.pose.covariance = pose_cov.flatten().tolist()

        twist_cov = np.zeros((6, 6))
        twist_cov[0:2, 0:2] = self.ekf.P[3:5, 3:5]
        twist_cov[0:2, 5] = self.ekf.P[3:5, 5]
        twist_cov[5, 0:2] = self.ekf.P[5, 3:5]
        twist_cov[5, 5] = self.ekf.P[5, 5]
        odom.twist.covariance = twist_cov.flatten().tolist()
        
        
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'  # 'odom'
        t.child_frame_id = 'base_link'    # 'base_link'

        t.transform.translation.x = float(state[0])
        t.transform.translation.y = float(state[1])
        t.transform.translation.z = float(state[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        if self.send_transform:
            self.tf_broadcaster.sendTransform(t)


        self.odom_pub.publish(odom)

    def timer_callback(self):
        """
        Main Loop of the EKF Node
        """
        with self.lock:
            # Get snapshot of newest measurement data, cmd inputs, and flags to use in prediction and update steps
            initialized = self.is_initialized
            u = np.copy(self.ackermann_data)
            snapshots = [(sensor, *sensor.snapshot()) for sensor in self.sensors]

        if not initialized:
            self.get_logger().log(f"[EKF Node] Initial state not set yet", rclpy.logging.LoggingSeverity.WARN, once=True)
            return

        self.get_logger().log(f"[EKF Node] Initial state is set", rclpy.logging.LoggingSeverity.INFO, once=True)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_filter_time is None:  # first loop
            self.last_filter_time = now
            return

        dt = now - self.last_filter_time
        if dt == 0:
            return
        self.last_filter_time = now

        t0 = time.perf_counter_ns()

        # predict
        self.model.predict(ekf=self.ekf, dt=dt, u=u)

        t1 = time.perf_counter_ns()
        duration_ms = (t1 - t0) / 1_000_000.0
        if duration_ms > self.predict_max_ms:
            self.predict_max_ms = duration_ms

        self.predict_count += 1
        self.predict_mean_ms += (duration_ms - self.predict_mean_ms) / self.predict_count

        t2 = time.perf_counter_ns()
        # update: apply each sensor's update sequentially when it has fresh data
        for sensor, fresh, z in snapshots:
            if fresh:
                sensor.update(self.ekf, z, u)

        t3 = time.perf_counter_ns()
        duration_ms = (t3 - t2) / 1_000_000.0
        if duration_ms > self.tot_update_max_ms:
            self.tot_update_max_ms = duration_ms

        self.tot_update_count += 1
        self.tot_update_mean_ms += (duration_ms - self.tot_update_mean_ms) / self.tot_update_count

        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
