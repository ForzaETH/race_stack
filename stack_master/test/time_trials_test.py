#!/usr/bin/env python3

import datetime

import rospy
import rospkg
import rostest
import unittest
from f110_msgs.msg import LapData


class TimeTrialsTester(unittest.TestCase):
    def test_main(self):
        node = TimeTrialsTesterNode()
        assert node.loop()

class TimeTrialsTesterNode:
    def __init__(self):
        rospy.init_node("time_trials_test", anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        # TODO move config params in a yaml
        self.lap_data = LapData()
        self.lap_data.lap_time = -1
        self.lap_time_sub = rospy.Subscriber('lap_data', LapData, self._lap_data_cb)
        self.n_laps = 3

        # file preparation
        self.output_text = self.init_output_report()

    def _lap_data_cb(self, data: LapData):
        self.lap_data = data
        self.output_text += f"Lap #{data.lap_count}: lap time {data.lap_time:5.5f} - average lateral deviation {data.average_lateral_error_to_global_waypoints:5.5f} - maximum lateral deviation {data.max_lateral_error_to_global_waypoints:5.5f}\n"

    def loop(self):
        while not rospy.is_shutdown():
            if self.lap_data.lap_count >= self.n_laps:
                rospy.loginfo(f"The car completed {self.n_laps:d} laps!")
                self.save_report()
                break
        self.save_report()
        return True

    def init_output_report(self):
        stack_master_path = rospkg.RosPack().get_path('stack_master')
        with open(stack_master_path+"/test/templates/time_trials.md", "r") as file:
            txt = file.readlines()

        datetime_now = datetime.datetime.now()
        datetime_string = datetime_now.strftime("%a %d %B %Y, %H:%M:%S")
        txt += f"This test was run on: {datetime_string}\n\n"

        txt += f"Configuration parameters: TODO\n\n"

        txt += "### Laps\n\n"
        return txt
    
    def save_report(self):
        stack_master_path = rospkg.RosPack().get_path('stack_master')
        full_path = stack_master_path+"/test/artifacts/time_trials_test_report.md"
        print("THE SAVING PATH: " + full_path)
        with open(full_path, "w") as file:
            file.writelines(self.output_text)
    
    def on_shutdown(self):
        self.save_report()

if __name__ == '__main__':
    rostest.rosrun("stack_master", 'time_trials_test', TimeTrialsTester)
