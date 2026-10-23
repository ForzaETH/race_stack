#!/usr/bin/env python3
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from .plan import GlobalPlan
from .plan_publisher import GlobalPlanPublisher


class GlobalPlanPublisherNode(Node):
    """Publishes the stored global plan of a map (global_waypoints.json), latched, for the rest of the stack."""

    def __init__(self):
        super().__init__('global_plan_publisher')

        # required: declared by type only, so it fails at startup if missing
        map_dir = Path(self.declare_parameter(
            'map_path', rclpy.Parameter.Type.STRING,
            ParameterDescriptor(description='Map directory containing global_waypoints.json', read_only=True)).value)

        if not (map_dir / 'global_waypoints.json').exists():
            self.get_logger().fatal(f"No global plan in {map_dir}, run the global planner first (planner_launch.yaml)")
            raise SystemExit(1)
        plan = GlobalPlan.load(map_dir)
        self.plan_publisher = GlobalPlanPublisher(self)
        self.plan_publisher.publish(plan)
        self.get_logger().info(f"Published global plan of {map_dir.name}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GlobalPlanPublisherNode()
        # Keep spinning: latched topics are only available while their publisher is alive
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # rclpy's SIGINT handler already shuts down the context


if __name__ == '__main__':
    main()
