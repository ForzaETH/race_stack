#!/usr/bin/env python3
from pathlib import Path

import rclpy
import yaml
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
        self.plan_publisher.publish_sectors(plan.raceline, self.load_speed_sector_starts(map_dir, len(plan.raceline.wpnts)))
        self.get_logger().info(f"Published global plan of {map_dir.name}")

    def load_speed_sector_starts(self, map_dir: Path, n_wpnts: int) -> list[int]:
        """Start waypoint indices of the speed sectors in speed_scaling.yaml; a single whole-lap sector without them."""
        path = map_dir / 'speed_scaling.yaml'
        if not path.exists():
            self.get_logger().warn(f"No {path.name} in {map_dir}, publishing the whole lap as one sector")
            return [0]

        params = yaml.safe_load(path.read_text())['sector_tuner']['ros__parameters']
        starts = [int(params[f'Sector{i}']['start']) for i in range(params['n_sectors'])]

        # Sectors sliced on a different raceline (e.g. re-planned since) no longer fit it
        if not starts or starts[0] != 0 or starts != sorted(starts) or starts[-1] >= n_wpnts:
            self.get_logger().error(f"Sectors in {path.name} don't fit the raceline ({n_wpnts} waypoints), "
                                    "publishing the whole lap as one sector; slice the sectors again")
            return [0]
        return starts


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
