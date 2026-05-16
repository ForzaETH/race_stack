#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from tf_transformations import euler_from_quaternion


class InitialPoseBridge(Node):
    """
    Bridge node that subscribes to /initialpose, published by RViz's
    "2D Pose Estimate" tool, and restarts Cartographer's localization
    trajectory at the clicked pose.

    Expected Cartographer setup:
      - trajectory 0: frozen map trajectory
      - trajectory 1 or higher: active localization trajectory

    This node:
      1. Receives a pose from RViz.
      2. Finishes the current Cartographer trajectory.
      3. Starts a new trajectory using the clicked pose as initial pose.
    """

    def __init__(self):
        super().__init__('initialpose_bridge')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('slam_config_path', '')
        self.declare_parameter('slam_config_basename', 'f110_2d_loc.lua')
        self.declare_parameter('current_trajectory_id', 1)
        self.declare_parameter('relative_to_trajectory_id', 0)
        self.declare_parameter('expected_frame_id', 'map')

        self.config_dir = self.get_parameter('slam_config_path').value
        self.config_basename = self.get_parameter('slam_config_basename').value
        self.current_trajectory_id = (
            self.get_parameter('current_trajectory_id').value
        )
        self.relative_to_trajectory_id = (
            self.get_parameter('relative_to_trajectory_id').value
        )
        self.expected_frame_id = self.get_parameter('expected_frame_id').value

        # Prevent multiple RViz clicks from triggering overlapping resets.
        self._busy = False

        # Allows async service calls inside callbacks.
        self.cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # Service clients
        # ------------------------------------------------------------------
        self.finish_client = self.create_client(
            FinishTrajectory,
            '/finish_trajectory',
            callback_group=self.cb_group,
        )

        self.start_client = self.create_client(
            StartTrajectory,
            '/start_trajectory',
            callback_group=self.cb_group,
        )

        self.get_logger().info('Waiting for Cartographer services...')

        if not self.finish_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/finish_trajectory service not available.')

        if not self.start_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/start_trajectory service not available.')

        self.get_logger().info('Cartographer services are available.')

        # ------------------------------------------------------------------
        # Subscriber to RViz initial pose
        # ------------------------------------------------------------------
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_cb,
            10,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            'InitialPoseBridge ready. Use RViz "2D Pose Estimate" '
            'to reset Cartographer localization.'
        )

        self.get_logger().info(
            f'Using config: directory="{self.config_dir}", '
            f'basename="{self.config_basename}"'
        )

        self.get_logger().info(
            f'Current trajectory ID: {self.current_trajectory_id}, '
            f'relative-to trajectory ID: {self.relative_to_trajectory_id}'
        )

    async def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """
        Called when RViz publishes a pose on /initialpose.
        """

        if self._busy:
            self.get_logger().warn(
                'Already resetting Cartographer trajectory. '
                'Ignoring this initial pose.'
            )
            return

        self._busy = True

        try:
            # --------------------------------------------------------------
            # Check frame
            # --------------------------------------------------------------
            if msg.header.frame_id != self.expected_frame_id:
                self.get_logger().warn(
                    f'Received initial pose in frame "{msg.header.frame_id}", '
                    f'but expected "{self.expected_frame_id}". '
                    'The pose will still be used, but make sure this is correct.'
                )

            pose = msg.pose.pose

            # --------------------------------------------------------------
            # Check quaternion validity
            # --------------------------------------------------------------
            if not self._is_quaternion_valid(pose.orientation):
                self.get_logger().error(
                    'Invalid initial pose quaternion. '
                    'Ignoring RViz initial pose.'
                )
                return

            q = pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            self.get_logger().info(
                f'Received initial pose: '
                f'x={pose.position.x:.3f}, '
                f'y={pose.position.y:.3f}, '
                f'z={pose.position.z:.3f}, '
                f'yaw={yaw:.3f} rad'
            )

            # --------------------------------------------------------------
            # Finish current trajectory
            # --------------------------------------------------------------
            finish_success = await self._finish_current_trajectory()

            if not finish_success:
                self.get_logger().error(
                    'Could not finish current trajectory. '
                    'New trajectory will not be started.'
                )
                return

            # --------------------------------------------------------------
            # Start new trajectory
            # --------------------------------------------------------------
            start_success = await self._start_new_trajectory(pose)

            if not start_success:
                self.get_logger().error(
                    'Could not start new Cartographer trajectory.'
                )
                return

            self.get_logger().info(
                'Cartographer localization trajectory reset successfully.'
            )

        finally:
            self._busy = False

    def _is_quaternion_valid(self, q):
        """
        Checks whether the quaternion is finite and approximately normalized.
        Cartographer requires a normalized quaternion.
        """

        values = [q.x, q.y, q.z, q.w]

        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                f'Quaternion contains non-finite values: {values}'
            )
            return False

        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

        if abs(norm - 1.0) > 1e-3:
            self.get_logger().error(
                f'Quaternion is not normalized. Norm = {norm:.6f}'
            )
            return False

        return True

    async def _finish_current_trajectory(self):
        """
        Calls Cartographer's /finish_trajectory service.
        Returns True only if the service succeeds and Cartographer returns OK.
        """

        req = FinishTrajectory.Request()
        req.trajectory_id = self.current_trajectory_id

        self.get_logger().info(
            f'Finishing trajectory {self.current_trajectory_id}...'
        )

        future = self.finish_client.call_async(req)
        await future

        result = future.result()

        if result is None:
            self.get_logger().error(
                f'/finish_trajectory returned no result for trajectory '
                f'{self.current_trajectory_id}.'
            )
            return False

        self.get_logger().info(
            f'FinishTrajectory response: {result.status.message}'
        )

        if result.status.code != 0:
            self.get_logger().error(
                f'Failed to finish trajectory {self.current_trajectory_id}. '
                f'Status code: {result.status.code}, '
                f'message: {result.status.message}'
            )
            return False

        return True

    async def _start_new_trajectory(self, pose):
        """
        Calls Cartographer's /start_trajectory service using the RViz pose
        as the initial pose.
        """

        req = StartTrajectory.Request()

        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_basename

        req.use_initial_pose = True
        req.initial_pose = pose

        # The clicked pose is interpreted relative to this trajectory.
        # In localization mode, this is usually the frozen map trajectory.
        req.relative_to_trajectory_id = self.relative_to_trajectory_id

        self.get_logger().info(
            f'Starting new trajectory relative to trajectory '
            f'{self.relative_to_trajectory_id}...'
        )

        future = self.start_client.call_async(req)
        await future

        result = future.result()

        if result is None:
            self.get_logger().error('/start_trajectory returned no result.')
            return False

        self.get_logger().info(
            f'StartTrajectory response: {result.status.message}'
        )

        if result.status.code != 0:
            self.get_logger().error(
                f'Failed to start new trajectory. '
                f'Status code: {result.status.code}, '
                f'message: {result.status.message}'
            )
            return False

        new_id = result.trajectory_id

        self.get_logger().info(
            f'Started new Cartographer trajectory with ID {new_id}.'
        )

        self.current_trajectory_id = new_id

        return True


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = InitialPoseBridge()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'InitialPoseBridge crashed: {exc}')
        else:
            print(f'InitialPoseBridge failed to start: {exc}')

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()