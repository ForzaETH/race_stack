#!/usr/bin/env python3
from typing import List

import numpy as np
import rospy
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray, OpponentTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from ps_planner.msg import OTSection, OTSections
from f110_msgs.msg import LapData
from std_msgs.msg import String

class Overtaking_Tester():
    """ This class is used to test the overtaking algorithms. It launches the the obstacle publisher and collision detector nodes,
    and continously listens to the collision message. For every setting of speed scaler and spawn position it generates an entry in a csv file. Additionally, the csv file
    contins the information if the ego car crashed into the opponent car or not, the track, boundaries and if the opponent car was overtaken or not.
    Every setting is run three times. The csv file is saved in the same directory as this file. The name of the file is the ot planer name, map name, solver name, and the date and time.
    The ego car has 7 laps to catch up to the opponent car and switch into trailing. After the first switch into trailing the ego car has 3 laps to overtake the opponent car.
    """
    def __init__(self) -> None:
        rospy.init_node('ot_testing', anonymous=True)
        looprate = 0.1
        self.rate = rospy.Rate(looprate)

        # Parameters
        self.current_lap = 0
        self.last_free_lab = 0
        self.first_lap = 0
        self.no_trailing = True



        # Subscribers
        rospy.Subscriber("/lap_data", LapData, self.lap_data_cb)
        rospy.Subscriber("/state_machine", String, self.state_cb)

    ### Callbacks ###
    def lap_data_cb(self, data: LapData):
        self.current_lap = data.lap_count
        if self.no_trailing == True:
            self.last_free_lab = data.lap_count
        self.no_trailing = True

    def state_cb(self, data: String):
       if data.data == "TRAILING" or data.data == "OVERTAKING":
            self.no_trailing = False5
