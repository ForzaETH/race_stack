#!/usr/bin/env python3
import rospy, rospkg
import yaml, os, subprocess, time
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import MarkerArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Arrow

class SectorSlicer:
    """
    Node for listening to gb waypoints and running a GUI to tune the sectors s.t. a yaml can be exported for dynamic reconfigure
    """
    def __init__(self):
        rospy.init_node('sector_node', anonymous=True)

        self.glb_wpnts = None
        self.track_bounds = None

        self.speed_scaling = 0.5
        self.glob_slider_s = 0
        self.sector_pnts = [0] #Sector always has tostart at 0

        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber('/trackbounds/markers', MarkerArray, self.bounds_cb)

        #rosparam to define yaml dir but filename will always be speed_scaling.yaml
        self.yaml_dir = rospy.get_param('~save_dir')

    def glb_wpnts_cb(self, data):
        self.glb_wpnts = data

    def bounds_cb(self, data):
        self.track_bounds = data

    def slice_loop(self):
        #Wait until we get the waypoints
        print('Waiting for global waypoints...')
        rospy.wait_for_message('/global_waypoints', WpntArray)

        #Select Sectors via the GUI
        self.sector_gui()
        print('Selected Sector IDXs:', self.sector_pnts)

        #Write sectors to yaml
        self.sectors_to_yaml()

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
        ax1.plot(x, y, "r-", linewidth=0.7)
        ax1.plot([mrk.pose.position.x for mrk in self.track_bounds.markers], [mrk.pose.position.y for mrk in self.track_bounds.markers], 'g-', linewidth=0.4)
        ax1.grid()
        ax1.set_aspect("equal", "datalim")
        ax1.set_xlabel("east in m")
        ax1.set_ylabel("north in m")
        #Plot arrow at start
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
            ax1.plot(x, y, "r-", linewidth=0.7)
            ax1.plot([mrk.pose.position.x for mrk in self.track_bounds.markers], [mrk.pose.position.y for mrk in self.track_bounds.markers], 'g-', linewidth=0.4)
            ax1.grid()
            ax1.set_aspect("equal", "datalim")
            ax1.set_xlabel("east in m")
            ax1.set_ylabel("north in m")
            ax1.scatter(x[cur_s], y[cur_s])
            if len(self.sector_pnts) > 0:
                ax1.scatter(x[self.sector_pnts], y[self.sector_pnts], c='red')

        #Matplotlib widgets for GUI
        slider = Slider(axslider, 'S [m]', 0, len(s), valinit=0, valfmt='%d')
        slider.on_changed(update_s)

        btn_select = Button(axselect, 'Select S')
        btn_select.on_clicked(select_s)

        btn_finish = Button(axfinish, 'Done')
        btn_finish.on_clicked(finish)
        
        plt.show()

    def sectors_to_yaml(self):
        #Create yaml with default speed scaling values
        n_sectors = len(self.sector_pnts) - 1
        dict_file = {'global_limit': self.speed_scaling, 'n_sectors': n_sectors}
        for i in range(0, n_sectors):
            #Add sectors with scaling field
            dict_file['Sector' + str(i)] = {'start':self.sector_pnts[i] if i == 0 else self.sector_pnts[i] + 1,
                                            'end':self.sector_pnts[i+1],
                                            'scaling':self.speed_scaling}
            #Add only_FTG field to sector
            dict_file['Sector' + str(i)].update({'only_FTG': False})
            #Add no_FTG field to sector
            dict_file['Sector' + str(i)].update({'no_FTG': False})
        
        #Save yaml to the respective maps folder 
        yaml_path = os.path.join(self.yaml_dir, 'speed_scaling.yaml')
        with open(yaml_path, 'w') as file:
            print('Dumping to {}: {}'.format(yaml_path, dict_file))
            yaml.dump(dict_file, file, sort_keys=False)

        #Copy content to sector_tuner package for catkin build
        ros_path = rospkg.RosPack().get_path('sector_tuner')
        yaml_path = os.path.join(ros_path, 'cfg/speed_scaling.yaml')
        with open(yaml_path, 'w') as file:
            print('Dumping to {}: {}'.format(yaml_path, dict_file))
            yaml.dump(dict_file, file, sort_keys=False)

        #Directly build the sector_tuner package
        time.sleep(1)
        print('Copying and writing the yaml to sector_tuner package and building catkin for next use...')
        shell_dir = os.path.join(ros_path, 'scripts/finish_sector.sh')
        subprocess.Popen(shell_dir, shell=True) 

if __name__ == "__main__":
    sector_slicer = SectorSlicer()
    sector_slicer.slice_loop()