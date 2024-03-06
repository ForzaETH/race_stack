#! /usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from f110_msgs.msg import WpntArray
from dynamic_reconfigure.msg import Config


class VelocityScaler:
    """
    Sector scaler for the velocity of the global waypoints
    """

    def __init__(self, debug_plot: bool = False) -> None:
        self.debug_plot = rospy.get_param("/velocity_scaler/debug_plot")

        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.glb_wpnts_sp_og = None
        self.glb_wpnts_sp_scaled = None
        
        # get initial scaling
        self.sectors_params = rospy.get_param("/map_params")
        self.n_sectors = self.sectors_params['n_sectors']

        # dyn params sub
        self.glb_wpnts_name = "/global_waypoints"
        rospy.Subscriber("/dyn_sector_server/parameter_updates", Config, self.dyn_param_cb)
        rospy.Subscriber(self.glb_wpnts_name, WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber(self.glb_wpnts_name+"/shortest_path", WpntArray, self.glb_wpnts_sp_cb)
        
        # new glb_waypoints pub
        self.scaled_points_pub = rospy.Publisher("/global_waypoints_scaled", WpntArray, queue_size=10)
        self.scaled_points_sp_pub = rospy.Publisher("/global_waypoints_scaled/shortest_path", WpntArray, queue_size=10)

    def glb_wpnts_cb(self, data:WpntArray):
        """
        Saves the global waypoints
        """
        self.glb_wpnts_og = data

    def glb_wpnts_sp_cb(self, data:WpntArray):
        """
        Saves the global waypoints
        """
        self.glb_wpnts_sp_og = data

    def dyn_param_cb(self, params:Config):
        """
        Notices the change in the parameters and scales the global waypoints
        """
        # get global limit
        self.sectors_params['global_limit'] = params.doubles[0].value

        # update params 
        for i in range(self.n_sectors):
            self.sectors_params[f"Sector{i}"]['scaling'] = np.clip(
                params.doubles[i+1].value, 0, self.sectors_params['global_limit']
            )

        rospy.loginfo(self.sectors_params)

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

        if self.n_sectors > 1:
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
        scaling = []

        if self.glb_wpnts_scaled is None:
            self.glb_wpnts_scaled = self.glb_wpnts_og
            self.glb_wpnts_sp_scaled = self.glb_wpnts_sp_og

        for i, wpnt  in enumerate(self.glb_wpnts_og.wpnts):
            vel_scaling = self.get_vel_scaling(i)
            new_vel = wpnt.vx_mps*vel_scaling
            self.glb_wpnts_scaled.wpnts[i].vx_mps = new_vel
            scaling.append(self.get_vel_scaling(i))

        if self.debug_plot:
            plt.clf()
            plt.plot(scaling)
            plt.legend(['og', 'scaled'])
            plt.ylim(0,1)
            plt.show()


    def loop(self):
        rospy.loginfo("Waiting for global waypoints...")
        rospy.wait_for_message(self.glb_wpnts_name, WpntArray)
        rospy.loginfo("Global waypoints received!")

        # initialise scaled points
        self.scale_points()

        run_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.scale_points()
            self.scaled_points_pub.publish(self.glb_wpnts_scaled)
            run_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("vel_scaler")
    vel_scaler = VelocityScaler()
    vel_scaler.loop()
