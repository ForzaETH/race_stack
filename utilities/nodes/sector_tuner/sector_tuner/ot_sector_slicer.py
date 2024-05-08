import yaml, os, subprocess, time
import rclpy
from rclpy.node import Node
from f110_msgs.msg import WpntArray
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Arrow

def get_data_path(subpath=''):
    """
    Helper function to get an absolute path to the specified (relative) path within the data folder.
    """
    return Path(get_package_share_directory('stack_master')).parents[3]/'src/race_stack/stack_master'/subpath

class OvertakingSectorSlicer(Node):
    """
    Node for listening to gb waypoints and running a GUI to tune the sectors s.t. a yaml can be exported for dynamic reconfigure
    """
    def __init__(self, future):
        super().__init__('ot_sector_slicer',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=False)
        self.declare_parameter('map_name','hangar_1905_v0')
        self.future = future
        
        self.glb_wpnts = None
        self.glb_sp_wpnts = None
        self.track_bounds = None

        self.glob_slider_s = 0
        self.sector_pnts = [0] #Sector always has tostart at 0

        self.wpnt_sub = self.create_subscription(WpntArray, '/global_waypoints', self.glb_wpnts_cb, 10)
        self.wpnt_sub_sp = self.create_subscription(WpntArray, '/global_waypoints/shortest_path', self.glb_sp_wpnts_cb, 10)
        self.bounds_sub = self.create_subscription(MarkerArray, '/trackbounds/markers', self.bounds_cb, 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #rosparam to define yaml dir but filename will always be speed_scaling.yaml
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.yaml_dir = get_data_path('maps/'+map_name)
        self.get_logger().info('Waiting for global waypoints...')
    
    def glb_wpnts_cb(self, data):
        self.glb_wpnts = data
    
    def glb_sp_wpnts_cb(self, data):
        self.glb_sp_wpnts = data

    def bounds_cb(self, data):
        self.track_bounds = data
        
    def timer_callback(self):
        if(self.track_bounds is None):
            return
        if(self.glb_wpnts is None):
            return
        if(self.glb_sp_wpnts is None):
            return
        
        #Select Sectors via the GUI
        self.sector_gui()
        self.get_logger().info('Selected Overtaking Sector IDXs: '+str(self.sector_pnts))

        #Write sectors to yaml
        self.sectors_to_yaml()
        #Indicate that the task is done and execution of the node can be stopped
        self.future.set_result(None)

    def sector_gui(self):
        #get wpnt message in list format for plotting
        s = []
        v = []
        x = []
        y = []
        for wpnt in self.glb_wpnts.wpnts:
            s.append(wpnt.s_m)
            v.append(wpnt.vx_mps)
            x.append(wpnt.x_m)
            y.append(wpnt.y_m)
        s = np.array(s)
        x = np.array(x)
        y = np.array(y)

        # plot raceline without sector points
        fig, (ax1, axslider, axselect, axfinish) = plt.subplots(4, 1, gridspec_kw={'height_ratios': [5, 1, 1, 1]})
        ax1.plot(x, y, "b-", linewidth=0.7)
        ax1.plot([mrk.pose.position.x for mrk in self.track_bounds.markers], [mrk.pose.position.y for mrk in self.track_bounds.markers], 'g-', linewidth=0.4)
        ax1.plot([wpnt.x_m for wpnt in self.glb_sp_wpnts.wpnts], [wpnt.y_m for wpnt in self.glb_sp_wpnts.wpnts], 'r-', linewidth=0.7)
        ax1.grid()
        ax1.set_aspect("equal", "datalim")
        ax1.set_xlabel("east in m")
        ax1.set_ylabel("north in m")
        # Plot arrow at start
        arr_par = {'x': x[0], 'dx': 10 * (x[1] - x[0]),
                   'y': y[0], 'dy': 10 * (y[1] - y[0]),
                   'color': 'gray',
                   'width': 0.5}
        ax1.add_artist(Arrow(**arr_par))
        
        #Slider stuff
        def update_s(val):
            idx = int(slider.val) - 1
            self.glob_slider_s = idx #update the global slider s
            update_map(x=x, y=y, cur_s=idx)
            fig.canvas.draw_idle()

        #Btn stuff
        def select_s(event):
            #When pressing button append the new position
            self.sector_pnts.append(self.glob_slider_s)
            update_map(x=x, y=y, cur_s=self.glob_slider_s)
        
        def finish(event):
            plt.close()

            #Sectors always end at end
            self.sector_pnts.append(len(s))

            #Eliminate duplicates if necessary
            self.sector_pnts = sorted(list(set(self.sector_pnts)))
            return 

        def update_map(x, y, cur_s):
            ax1.cla()
            ax1.plot(x, y, "b-", linewidth=0.7)
            ax1.plot([mrk.pose.position.x for mrk in self.track_bounds.markers], [mrk.pose.position.y for mrk in self.track_bounds.markers], 'g-', linewidth=0.4)
            ax1.plot([wpnt.x_m for wpnt in self.glb_sp_wpnts.wpnts], [wpnt.y_m for wpnt in self.glb_sp_wpnts.wpnts], 'r-', linewidth=0.7)
            ax1.grid()
            ax1.set_aspect("equal", "datalim")
            ax1.set_xlabel("east in m")
            ax1.set_ylabel("north in m")
            ax1.scatter(x[cur_s], y[cur_s])
            if len(self.sector_pnts) > 0:
                ax1.scatter(x[self.sector_pnts], y[self.sector_pnts], c='blue')

        #Matplotlib widgets for GUI
        slider = Slider(axslider, 'S [m]', 0, len(s), valinit=0, valfmt='%d')
        slider.on_changed(update_s)

        btn_select = Button(axselect, 'Select Overtaking S')
        btn_select.on_clicked(select_s)

        btn_finish = Button(axfinish, 'Done')
        btn_finish.on_clicked(finish)
        
        plt.show()

    def sectors_to_yaml(self):
        #Create yaml with default sectors values
        n_sectors = len(self.sector_pnts) - 1
        dict_file = {
                'n_sectors': n_sectors,
                'yeet_factor': 1.25,
                'spline_len': 30,
                'ot_sector_begin': 0.5
            }
        for i in range(0, n_sectors):
            #Add sectors with scaling field
            dict_file['Overtaking_sector' + str(i)] = {'start':self.sector_pnts[i] if i == 0 else self.sector_pnts[i] + 1,
                                            'end':self.sector_pnts[i+1]
                                            }
            dict_file['Overtaking_sector' + str(i)].update({'ot_flag': False})
        ros_yaml_preamble = {'ot_interpolator': {'ros__parameters': dict_file}}
        
        #Save yaml to the respective maps folder 
        yaml_path = os.path.join(self.yaml_dir, 'ot_sectors.yaml')
        with open(yaml_path, 'w') as file:
            self.get_logger().info('Dumping to {}: {}'.format(yaml_path, ros_yaml_preamble))
            yaml.dump(ros_yaml_preamble, file, sort_keys=False)

        # Directly build the stack_master package
        self.get_logger().info('Writing yaml file and invoking colcon build on stack_master package to install the yaml config for the next use.')
        subprocess.Popen("ros2 run sector_tuner finish_sector.sh", shell=True) 


def main():
    rclpy.init()
    future = rclpy.Future()
    node = OvertakingSectorSlicer(future)
    rclpy.spin_until_future_complete(node, future)
    rclpy.shutdown()
