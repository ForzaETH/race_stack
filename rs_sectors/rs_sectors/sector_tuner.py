import copy

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, FloatingPointRange
from rclpy.node import Node
from rs_msgs.msg import WpntArray
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import quaternion_from_euler

from stack_master.parameter_event_handler import ParameterEventHandler

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class SectorTuner(Node):
    """
    Sector scaler for the velocity of the global waypoints
    """
    def __init__(self):
        super().__init__('sector_tuner',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.glb_wpnts_sp_og = None
        self.glb_wpnts_sp_scaled = None

        # get initial scaling
        self.sectors_params=self.parameters_to_dict()
        self.n_sectors = self.sectors_params['n_sectors']
        
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])
        self.set_descriptor('global_limit',descriptor=desc)
        for i in range(self.n_sectors):
            self.set_descriptor('Sector'+str(i)+'.scaling',descriptor=desc)

        # dyn params sub
        self.glb_wpnts_name = "/global_plan/raceline"
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_event_callback(
            callback=self.dyn_param_cb,
        )
        self.global_waypoint_sub = self.create_subscription(
            WpntArray,
            self.glb_wpnts_name,
            self.global_waypoints_cb,
            LATCHED)
        self.global_waypoint_sp_sub = self.create_subscription(
            WpntArray,
            "/global_plan/shortest_path",
            self.global_waypoints_sp_cb,
            LATCHED)
        
        # new glb_waypoints pub
        self.scaled_points_pub = self.create_publisher(WpntArray, "/global_waypoints_scaled", LATCHED)
        self.scaled_points_sp_pub = self.create_publisher(WpntArray, "/global_waypoints_scaled/shortest_path", LATCHED)
        
        # Visualizations
        self.sector_visualization_pub = self.create_publisher(MarkerArray, '/sector_markers', LATCHED)
        
        self.get_logger().debug("Waiting for global waypoints...")
        
    def parameters_to_dict(self):
        params = {}
        for key in self._parameters:
            keylist = key.split('.')
            paramit = params
            for subkey in keylist[:-1]:
                paramit = paramit.setdefault(subkey, {})
            paramit[keylist[-1]] = self.get_parameter(key).value
        return params

    def global_waypoints_cb(self, data:WpntArray):
        """
        Saves the global waypoints of the main trajectory (e.g. min curvature) and publishes them scaled
        """
        self.glb_wpnts_og = data
        self.publish_scaled()
        self.publish_sector_markers()
     
    def global_waypoints_sp_cb(self, data:WpntArray):
        """
        Saves the global waypoints of the shortest path
        """
        self.glb_wpnts_sp_og = data

    def dyn_param_cb(self, parameter_event):
        """
        Notices the change in the parameters and scales the global waypoints
        """
        if(parameter_event.node != '/sector_tuner'):
            return
        self.sectors_params = self.parameters_to_dict()
        # update params 
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                self.sectors_params[f"Sector{i}"]['scaling'], 0, self.sectors_params['global_limit']
            )

        self.get_logger().info(str(self.sectors_params))
        # outputs are latched and only published on change
        self.publish_scaled()
        self.publish_sector_markers()

    def get_vel_scaling(self, s):
        """
        Gets the dynamically reconfigured velocity scaling for the points.
        Linearly interpolates for points between two sectors
        
        Parameters
        ----------
        s
            s parameter whose sector we want to find
        """
        hl_change = 10

        if self.n_sectors == 0:
            return self.sectors_params['global_limit']
        elif self.n_sectors > 1:
            for i in range(self.n_sectors):
                if i == 0 :
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{self.n_sectors-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i+1}']['start']-hl_change, self.sectors_params[f'Sector{i+1}']['start']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                    )
                elif i != self.n_sectors-1:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i+1}']['start'] - hl_change) and (s < self.sectors_params[f'Sector{i+1}']['start']):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i+1}']['start']-hl_change, self.sectors_params[f'Sector{i+1}']['start']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{i+1}']['scaling']]
                    )
                else:
                    if (s >= self.sectors_params[f'Sector{i}']['start']) and (s < self.sectors_params[f'Sector{i}']['start'] + hl_change):
                        scaler = np.interp(
                            x=s,
                            xp=[self.sectors_params[f'Sector{i}']['start']-hl_change, self.sectors_params[f'Sector{i}']['start']+hl_change],
                            fp=[self.sectors_params[f'Sector{i-1}']['scaling'], self.sectors_params[f'Sector{i}']['scaling']]
                        )
                    elif (s >= self.sectors_params[f'Sector{i}']['start'] + hl_change) and (s < self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = self.sectors_params[f"Sector{i}"]['scaling']
                    elif (s >= self.sectors_params[f'Sector{i}']['end'] - hl_change):
                        scaler = np.interp(
                        x=s,
                        xp=[self.sectors_params[f'Sector{i}']['end']-hl_change, self.sectors_params[f'Sector{i}']['end']+hl_change],
                        fp=[self.sectors_params[f'Sector{i}']['scaling'], self.sectors_params[f'Sector{0}']['scaling']]
                    )
        elif self.n_sectors == 1:
            scaler = self.sectors_params["Sector0"]['scaling']

        return scaler

    def scale_points(self):
        """
        Scales the global waypoints' velocities
        """
        # scale a fresh copy: scaling the og message in place would compound on every publish
        self.glb_wpnts_scaled = copy.deepcopy(self.glb_wpnts_og)
        for i, wpnt in enumerate(self.glb_wpnts_og.wpnts):
            self.glb_wpnts_scaled.wpnts[i].vx_mps = wpnt.vx_mps * self.get_vel_scaling(i)

    def publish_scaled(self):
        if self.glb_wpnts_og is None:
            return
        self.scale_points()
        self.scaled_points_pub.publish(self.glb_wpnts_scaled)

    def publish_sector_markers(self):
        if self.glb_wpnts_og is None:
            return
        
        global_waypoints_vis = []
        for waypoint in self.glb_wpnts_og.wpnts:
            global_waypoints_vis.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])
        
        n_sectors = self.sectors_params['n_sectors']
        sec_markers = MarkerArray()

        for i in range(n_sectors):
            s = self.sectors_params[f"Sector{i}"]['start']
            if s == (len(global_waypoints_vis) - 1):
                theta = np.arctan2((global_waypoints_vis[0][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[0][0] - global_waypoints_vis[s][0]))
            else:
                theta = np.arctan2((global_waypoints_vis[s+1][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[s+1][0] - global_waypoints_vis[s][0]))
            quaternions = quaternion_from_euler(0, 0, theta)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.ARROW
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = global_waypoints_vis[s][0]
            marker.pose.position.y = global_waypoints_vis[s][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = quaternions[0]
            marker.pose.orientation.y = quaternions[1]
            marker.pose.orientation.z = quaternions[2]
            marker.pose.orientation.w = quaternions[3]
            marker.id = i
            sec_markers.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "map"
            marker_text.header.stamp = self.get_clock().now().to_msg()
            marker_text.type = marker_text.TEXT_VIEW_FACING
            marker_text.text = f"Start Sector {i}"
            marker_text.scale.z = 0.4
            marker_text.color.r = 0.2
            marker_text.color.g = 0.1
            marker_text.color.b = 0.1
            marker_text.color.a = 1.0
            marker_text.pose.position.x = global_waypoints_vis[s][0]
            marker_text.pose.position.y = global_waypoints_vis[s][1]
            marker_text.pose.position.z = 1.5
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0436194
            marker_text.pose.orientation.w = 0.9990482
            marker_text.id = i + n_sectors
            sec_markers.markers.append(marker_text)
        self.sector_visualization_pub.publish(sec_markers)


def main():
    rclpy.init()
    node = SectorTuner()
    rclpy.spin(node)
    rclpy.shutdown()
