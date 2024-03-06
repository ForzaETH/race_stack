#!/usr/bin/env python3

import rospy
import rostest
import unittest
from std_msgs.msg import Bool
from f110_msgs.msg import LapData


class Head2HeadTester(unittest.TestCase):
    def test_main(self):
        node = Head2HeadTesterNode()
        assert node.loop()

class Head2HeadTesterNode:
    def __init__(self):
        rospy.init_node("head_to_head_test", anonymous=True)

        self.lap_data = LapData()
        self.lap_data.lap_time = 99
        self.lap_time_sub = rospy.Subscriber('lap_data', LapData, self._lap_data_cb)
        self.counter = 0
        self.n_laps = 1
        self.laptime_threshold = 30

        self.pub_obs_flag = rospy.Publisher('/test/obst_start', Bool, queue_size=10)

    def _lap_data_cb(self, data: LapData):
        self.lap_data = data

    def loop(self):
        while not rospy.is_shutdown():
            if self.lap_data.lap_time < self.laptime_threshold:
                break
            else:
                if self.lap_data.lap_count > self.n_laps:
                    rospy.loginfo(f"The car completed {self.n_laps:d} laps without going under {self.laptime_threshold:d} s. IT SUCKS!")
                    break

        self.pub_obs_flag.publish(True)
        
        while not rospy.is_shutdown():
            if self.lap_data.lap_time < self.laptime_threshold:
                break
            else:
                if self.lap_data.lap_count > self.n_laps:
                    rospy.loginfo(f"The car completed {self.n_laps:d} laps without going under {self.laptime_threshold:d} s. IT SUCKS!")
                    break

        return True

if __name__ == '__main__':
    rostest.rosrun("stack_master", 'head_to_head_test', Head2HeadTester)
