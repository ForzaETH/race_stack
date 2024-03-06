#!/usr/bin/env python3
import time
import csv
import rospy
import os
 
class Plotter:
    def __init__(self):
        home_dir = os.environ["HOME"]
        home_dir += '/plot_datas/plot_data_{}.csv'.format(int(time.time()))
        try:
            self.file = open(home_dir, 'w+')
            self.writer = csv.writer(self.file, delimiter=';')
            headers = ['goal_speed', 'car_speed', 'calculated_speed', 'curvature', 'trackbounds_avg', 'lateral_error', 'position_on_path', 'lookahead_distance', 'steering_angle']
            self.writer.writerow(headers)
        except:
            rospy.logwarn('>>>>>>>>>> DID YOU CREATE A FOLDER CALLED plot_datas IN THE HOME DIRECTORY? <<<<<<<<<<<<<<<<<<')

    def __del__(self):
        self.file.close()

    def add_values(self, goal_speed, car_speed, calculated_speed, curvature, trackbounds_avg, lateral_error, position_on_path, lookahead_distance, steering_angle):
        data = [goal_speed, car_speed, calculated_speed, curvature, trackbounds_avg, lateral_error, position_on_path, lookahead_distance, steering_angle]
        self.writer.writerow(data)
    
    def flush(self):
        self.file.flush()
