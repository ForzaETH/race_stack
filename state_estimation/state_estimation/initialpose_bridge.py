import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from tf_transformations import euler_from_quaternion


class InitialPoseBridge(Node):
    """
    Bridge node that subscribes to /initialpose (published by RViz's "2D Pose Estimate" tool)
    and restarts Cartographer's localization trajectory at the clicked pose.

    Cartographer does not natively listen to /initialpose — it uses a service-based trajectory
    management model. This node translates between the two.
    """

    def __init__(self):
        super().__init__('initialpose_bridge')

        # Parameters — same config used by cartographer_node in the launch file
        self.declare_parameter('slam_config_path', '')
        self.declare_parameter('slam_config_basename', 'f110_2d_loc.lua')

        self.config_dir = self.get_parameter('slam_config_path').value
        self.config_basename = self.get_parameter('slam_config_basename').value

        # Trajectory ID tracking
        # When Cartographer launches in localization mode, trajectory 0 is the frozen map
        # and trajectory 1 is the initial localization trajectory.
        self.current_trajectory_id = 1

        # ReentrantCallbackGroup allows async service calls from within a callback
        # without deadlocking the executor.
        cb_group = ReentrantCallbackGroup()

        self.finish_client = self.create_client(FinishTrajectory, '/finish_trajectory', callback_group=cb_group)
        self.start_client = self.create_client(StartTrajectory, '/start_trajectory', callback_group=cb_group)

        self.get_logger().info('Waiting for Cartographer services...')
        self.finish_client.wait_for_service(timeout_sec=30.0)
        self.start_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Cartographer services available.')

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_cb,
            10,
            callback_group=cb_group,
        )

        self.get_logger().info(
            'InitialPose Bridge ready. Use the "2D Pose Estimate" tool in RViz '
            'to set the robot\'s initial position on the map.'
        )

    async def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """Called when the user clicks '2D Pose Estimate' in RViz."""
        pose = msg.pose.pose

        q = pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.get_logger().info(
            f'Received initial pose: x={pose.position.x:.2f}, '
            f'y={pose.position.y:.2f}, yaw={yaw:.2f} rad'
        )

        await self._finish_current_trajectory()
        await self._start_new_trajectory(pose)

    async def _finish_current_trajectory(self):
        req = FinishTrajectory.Request()
        req.trajectory_id = self.current_trajectory_id

        self.get_logger().info(f'Finishing trajectory {self.current_trajectory_id}...')
        future = self.finish_client.call_async(req)
        await future

        if future.result() is not None:
            self.get_logger().info(
                f'Finished trajectory {self.current_trajectory_id}: '
                f'{future.result().status.message}'
            )
        else:
            self.get_logger().warn(
                f'Failed to finish trajectory {self.current_trajectory_id}.'
            )

    async def _start_new_trajectory(self, pose):
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_basename
        req.use_initial_pose = True
        req.initial_pose = pose
        req.relative_to_trajectory_id = 0  # Relative to the frozen map

        self.get_logger().info('Starting new trajectory at clicked pose...')
        future = self.start_client.call_async(req)
        await future

        if future.result() is not None:
            new_id = future.result().trajectory_id
            self.get_logger().info(
                f'Started new trajectory {new_id}: '
                f'{future.result().status.message}'
            )
            self.current_trajectory_id = new_id
        else:
            self.get_logger().error('Failed to start new trajectory!')


def main():
    rclpy.init()
    node = InitialPoseBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
