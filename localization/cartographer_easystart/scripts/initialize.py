#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import GetTrajectoryStates, FinishTrajectory, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class GetTrajectoryStatesNode(Node):
    def __init__(self):
        super().__init__('get_trajectory_states_node')

        # Create clients for the /get_trajectory_states, /finish_trajectory, and /start_trajectory services
        self.get_trajectory_states_client = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.finish_trajectory_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.start_trajectory_client = self.create_client(StartTrajectory, '/start_trajectory')

        # Subscribe to the /initialpose topic from RViz
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

    def call_get_trajectory_states(self):
        # Wait for the service to be available
        self.get_logger().info('Waiting for /get_trajectory_states service...')
        while not self.get_trajectory_states_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        # Create the service request
        request = GetTrajectoryStates.Request()

        # Send the service request
        self.get_logger().info('Sending request to /get_trajectory_states...')
        future = self.get_trajectory_states_client.call_async(request)

        # Add a callback to handle the response
        future.add_done_callback(self.handle_get_trajectory_response)

    def handle_get_trajectory_response(self, future):
        try:
            response = future.result()
            if response.status.code == 0:
                self.get_logger().info(f"Received trajectory states: {response.trajectory_states}")
                self.get_logger().info(f"Trajectory IDs: {response.trajectory_states.trajectory_id}")
                self.get_logger().info(f"Trajectory States: {response.trajectory_states.trajectory_state}")

                # Process the response to finish only the last trajectory
                self.finish_last_trajectory(
                    response.trajectory_states.trajectory_id,
                    response.trajectory_states.trajectory_state
                )
            else:
                self.get_logger().error(f"Error from service: {response.status.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to call service /get_trajectory_states: {str(e)}")

    def finish_last_trajectory(self, trajectory_ids, trajectory_states):
        """Finish the last trajectory based on the maximum trajectory ID."""

        if not trajectory_ids:
            self.get_logger().info("No trajectories found.")
            return

        # Find the last trajectory (max ID)
        last_trajectory_id = max(trajectory_ids)
        last_trajectory_state = trajectory_states[len(trajectory_ids)-1]

        # Finish the last trajectory only if it's inactive (state == 0)
        if last_trajectory_state == 0:  # State 0 indicates 'INACTIVE'
            self.get_logger().info(f"Finishing last trajectory ID: {last_trajectory_id}")

            # Create a request to finish the trajectory
            request = FinishTrajectory.Request()
            request.trajectory_id = last_trajectory_id

            # Call the finish_trajectory service
            future = self.finish_trajectory_client.call_async(request)
            future.add_done_callback(lambda fut: self.handle_finish_trajectory_response(fut, last_trajectory_id))

    def handle_finish_trajectory_response(self, future, trajectory_id):
        try:
            future.result()
            self.get_logger().info(f"Successfully finished trajectory ID: {trajectory_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to finish trajectory ID {trajectory_id}: {str(e)}")

    def start_new_trajectory(self, msg):
        """Start a new trajectory using the initial pose."""
        self.get_logger().info("Starting a new trajectory...")

        # Create the request for the StartTrajectory service
        request = StartTrajectory.Request()
        request.configuration_directory = '/home/orin_nx/localization/src/cartographer_easystart/config'  # Update this path
        request.configuration_basename = 'localization_2d.lua'  # Update this file name
        request.use_initial_pose = True
        request.initial_pose.position.x = msg.pose.pose.position.x
        request.initial_pose.position.y = msg.pose.pose.position.y
        request.initial_pose.position.z = msg.pose.pose.position.z
        request.initial_pose.orientation = msg.pose.pose.orientation
        request.relative_to_trajectory_id = 0  # Use relative trajectory ID as needed

        # Call the start_trajectory service
        future = self.start_trajectory_client.call_async(request)
        # Pass 'msg' to the callback using a lambda function
        future.add_done_callback(lambda fut: self.handle_start_trajectory_response(fut, msg))

    def handle_start_trajectory_response(self, future, msg):
        try:
            response = future.result()
            if response.status.code == 0:
                self.get_logger().info(f"Successfully started trajectory with ID: {response.trajectory_id}")
            else:
                self.get_logger().error(f"Failed to start trajectory: {response.status.message}")
                            
                # Retry if Topics are already used
                if "Topics are already used" in response.status.message:
                    self.get_logger().info("Retrying to start trajectory...")
                    time.sleep(0.1)  # Wait before retrying
                    self.start_new_trajectory(msg)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to start trajectory: {str(e)}")

    def initial_pose_callback(self, msg):
        """Callback function for the /initialpose topic."""
        self.get_logger().info(f"Received initial pose: Position: [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}], "
                               f"Orientation: [{msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, "
                               f"{msg.pose.pose.orientation.z}, {msg.pose.pose.orientation.w}]")

        # Call the get_trajectory_states service
        self.call_get_trajectory_states()

        # Start a new trajectory using the received initial pose
        self.start_new_trajectory(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GetTrajectoryStatesNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
