#!/usr/bin/env python3
import array
import shutil
from pathlib import Path

import yaml
import cv2
import numpy as np

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from cartographer_ros_msgs.msg import StatusCode
from tf_transformations import euler_from_quaternion

from rs_mapping.parameters_codegen import mapping as mapping_parameters

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class MappingNode(Node):
    
    def __init__(self):
        super().__init__('mapping_node')

        self.params = mapping_parameters.ParamListener(self).get_params()

        self.map_dir = Path(self.params.map_path)
        self.map_name = self.map_dir.name

        self.filtered_map: OccupancyGrid | None = None
        self.map: OccupancyGrid | None = None

        self.initial_position = None

        # The pose directly comes from Cartographer SLAM (map → base_link)
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.finish_trajectory_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.write_state_client = self.create_client(WriteState, '/write_state')

        
        # Subscribe to Cartographer
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        
        # Create a service to save the map
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/save_map', self.save_map_cb, callback_group=self.cb_group)
        
        # Publish the filtered map
        self.filtered_map_pub = self.create_publisher(OccupancyGrid, "~/filtered_map", LATCHED)
        
        self.create_timer(1 / self.params.rate_hz, self.loop)
        
    def latch_initial_pose(self) -> None:
        """
        Latch the start pose from the map -> base_link transform, once it exists.

        Called every loop until it succeeds; the first transform available is the
        one saved as the map's ``initial_pose``.
        """
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException as e: # type: ignore
            self.get_logger().warning(
                f"Waiting for the {self.map_frame}->{self.base_frame} transform: {e}",
                once=True
            )
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.initial_position = (t.x, t.y, theta)
        self.get_logger().info(
            f"Initial pose latched from TF: x={t.x:.3f} y={t.y:.3f} theta={theta:.3f}")

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map = msg
        self.filter_map_occupancy_grid()

    def filter_map_occupancy_grid(self) -> None:
        """
        Filters the occupancy grid map by performing morphological opening and binarization.
        """

        # Reshape and cast OccupancyGrid to int8
        original_map = np.asarray(self.map.data, dtype=np.int8).reshape(self.map.info.height, self.map.info.width)
        
        # Binarize the map (unknown = -1 < 0 <= free cells < self.params.occupancy_grid_threshold <= occupied)
        free = (original_map >= 0) & (original_map < self.params.occupancy_grid_threshold)
        bw = np.where(free, 255, 0).astype(np.uint8)
        
        # Filter with morphological opening
        k = 2 * round(self.params.min_free_width_m / (2 * self.map.info.resolution)) + 1  # odd, in cells
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        filtered_map = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)

        # Convert back to OccupancyGrid
        occupancy = np.where(filtered_map == 255, 0, 100).astype(np.int8)
        self.filtered_map = OccupancyGrid(header=self.map.header, info=self.map.info)
        self.filtered_map.data = array.array('b', occupancy.tobytes())
        self.filtered_map_pub.publish(self.filtered_map)
    
    async def _call_cartographer(self, client, request) -> str | None:
        if not client.service_is_ready():
            return f'{client.srv_name} not available, is Cartographer running?'
        result = await client.call_async(request)
        if result.status.code != StatusCode.OK:
            return f'{client.srv_name} failed: {result.status.message}'
        return None 


    async def save_map_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Saves the filtered map as PNG and YAML and the Cartographer state as pbstream into map_dir."""
        response.success, response.message = False, ''
        if self.filtered_map is None:
            response.message = "No map received yet, cannot save."
        elif self.initial_position is None:
            response.message = f"No {self.map_frame}->{self.base_frame} transform was seen, cannot save."
        else:
            # Resolve map name conflicts with latest
            _check_default_map(self.map_name, self.map_dir, self.map_dir.parent / 'backup', self.get_logger().warn)
            if self.map_dir.exists():
                response.message = f"{self.map_dir} already exists, choose another map name."
        if response.message:
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f"Saving map '{self.map_name}'...")
        self.map_dir.mkdir(parents=True)
        _save_map_to_directory(self.map_dir, self.map_name, self.filtered_map, self.initial_position)

        # finish_trajectory has to complete before write_state, so all submaps are finished
        pbstream_path = str(self.map_dir / f'{self.map_name}.pbstream')
        error = (await self._call_cartographer(self.finish_trajectory_client, FinishTrajectory.Request(trajectory_id=0))
                 or await self._call_cartographer(self.write_state_client, WriteState.Request(filename=pbstream_path)))
        if error:
            response.message = f"PNG and YAML saved, but no pbstream: {error}"
            self.get_logger().error(response.message)
            return response

        response.success, response.message = True, f"Map saved to {self.map_dir}"
        self.get_logger().info(response.message)
        return response

    def loop(self):
        """Main Loop"""
        self.get_logger().info("[MAPPING_NODE]: Mapping ready", once=True)
        
        if self.initial_position is None:
            self.latch_initial_pose()


def _check_default_map(map_name: str, map_dir: Path, backup_dir: Path, logwarn) -> None:
    if map_name == 'latest':
        if map_dir.exists():
            if backup_dir.exists():
                shutil.rmtree(backup_dir)
            shutil.move(str(map_dir), str(backup_dir))
            logwarn(f"Map directory already exists. Backed up the old map to {backup_dir}")
            

def _save_map_to_directory(
    map_dir: Path,
    map_name: str,
    grid: OccupancyGrid,
    initial_position: tuple
    ) -> None:
    # Occupancy back to an image (free 0 -> white 255, occupied 100 -> black 0);
    # flipped because image rows go top-down while grid row 0 is at the origin
    occupancy = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    image = np.where(occupancy == 0, 255, 0).astype(np.uint8)
    cv2.imwrite(str(map_dir / f'{map_name}.png'), cv2.flip(image, 0))

    origin = grid.info.origin.position
    dict_map = {'image': map_name + '.png',
                'resolution': grid.info.resolution,
                'origin': [origin.x, origin.y, 0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196, 
                'initial_pose': [initial_position[0], initial_position[1], initial_position[2]]}

    with open(map_dir / f'{map_name}.yaml', 'w') as file:
        _ = yaml.dump(dict_map, file, default_flow_style=False)

def main(args=None):
    """Main function to run the mapping node."""
    
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
