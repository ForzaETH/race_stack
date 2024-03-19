#!/usr/bin/env python3
import tf
import rospy
import math, sys
from f110_msgs.msg import Wpnt, WpntArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from copy import deepcopy


class GBPubNode():
    #circle center (x_o, y_o) and radius r in meters and number of discretised circle
    def __init__(self):
        rospy.init_node('gbpub_node', anonymous=True)
        self.pub_gb = rospy.Publisher('global_waypoints_scaled', WpntArray, queue_size=10)
        self.pub_markers = rospy.Publisher('gb_traj_markers', MarkerArray, queue_size=10)

        #Get params
        exp = rospy.get_param("/gb_traj_pub/experiment")
        self.speed = rospy.get_param("/gb_traj_pub/speed")
        if exp == 'circle' or exp == 'circle_reverse':
            print('Circle center (x_o, y_o) and radius r in meters and number of discretised circle')
            x_o = rospy.get_param("/gb_traj_pub/x_o")
            y_o = rospy.get_param("/gb_traj_pub/y_o")
            r = rospy.get_param("/gb_traj_pub/r")
            nb = rospy.get_param("/gb_traj_pub/nb")
            if exp == 'circle':
                self.circle_loop(center=(x_o, y_o), r=r, nb_points=int(nb))
            elif exp == 'circle_reverse':
                self.circle_loop_reverse(center=(x_o, y_o), r=r, nb_points=int(nb))
            

        elif exp == 'ellipse':
            print('Ellipse center (x_o, y_o) and x/y radius a/b in meters and number of discretised circle')
            x_o = rospy.get_param("/gb_traj_pub/x_o")
            y_o = rospy.get_param("/gb_traj_pub/y_o")
            a = rospy.get_param("/gb_traj_pub/a")
            b = rospy.get_param("/gb_traj_pub/b")
            nb = rospy.get_param("/gb_traj_pub/nb")

            self.ellipse_loop(center=(x_o, y_o), a=a, b=b, nb_points=int(nb))

        elif exp == 'line':
            print('Line center (x_o, y_o) and length a in meters and number of discretised line')
            x_o = rospy.get_param("/gb_traj_pub/x_o")
            y_o = rospy.get_param("/gb_traj_pub/y_o")
            a = rospy.get_param("/gb_traj_pub/a")
            nb = rospy.get_param("/gb_traj_pub/nb")

            self.line_loop(center=(x_o, y_o), l=a, nb_points=int(nb))

    def line_loop(self, center=(0,0), l=100, nb_points=10):
        rate = rospy.Rate(5)
        pos =  [i*l/nb_points for i in range(nb_points+1)]
        gb_wpnts = WpntArray()
        gb_markers = MarkerArray()
        
        for i, pos in enumerate(pos):
            # Pass Wpnts
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = pos #+ 0.3*(-1)**i
            wpnt.y_m = pos #0.2*(-1)**i
            wpnt.s_m = np.sqrt(2)*pos
            wpnt.psi_rad = math.pi/4
            wpnt.vx_mps = self.speed
            wpnt.d_left = 0.5
            wpnt.d_right = 1
            gb_wpnts.wpnts.append(wpnt)

            #Pass Markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.orientation.w = 1

            marker.id = i*3
            marker.color.r = 1.0
            marker.pose.position.x = wpnt.x_m + wpnt.d_right * math.cos(angle)
            marker.pose.position.y = wpnt.y_m + wpnt.d_right * math.sin(angle)
            right_bound = deepcopy(marker)
            gb_markers.markers.append(right_bound)

            marker.id = i*3 + 1
            marker.color.g = 1.0
            marker.pose.position.x = wpnt.x_m - wpnt.d_left * math.cos(angle)
            marker.pose.position.y = wpnt.y_m - wpnt.d_left * math.sin(angle)
            left_bound = deepcopy(marker)
            gb_markers.markers.append(left_bound)

            marker.id = i*3 + 2
            marker.color.b = 1.0
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            middle = deepcopy(marker)
            gb_markers.markers.append(middle)

        # Publish stuff
        while not rospy.is_shutdown():
            gb_wpnts.header.stamp = rospy.Time.now()
            self.pub_gb.publish(gb_wpnts)
            self.pub_markers.publish(gb_markers)
            rate.sleep()
    
    def circle_loop(self, center=(0,0), r=3, nb_points=10):
        rate = rospy.Rate(5)  # rate in hertz

        angles = [i*2*math.pi/nb_points for i in range(nb_points+1)]
        gb_wpnts = WpntArray()
        gb_markers = MarkerArray()


        for i, angle in enumerate(angles):
            # Pass Wpnts
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = r * math.cos(angle) + center[0]
            wpnt.y_m = r * math.sin(angle) + center[1]
            wpnt.s_m = r * angle
            wpnt.psi_rad = (angle + math.pi/2) % (2*math.pi)
            wpnt.vx_mps = self.speed
            wpnt.d_left = abs(2 * math.cos(angle)) - 1
            wpnt.d_right = 3
            gb_wpnts.wpnts.append(wpnt)

            #Pass Markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.pose.orientation.w = 1
            
            marker.id = i*3
            marker.color.r = 1.0
            marker.pose.position.x = wpnt.x_m + wpnt.d_right * math.cos(angle)
            marker.pose.position.y = wpnt.y_m + wpnt.d_right * math.sin(angle)
            right_bound = deepcopy(marker)
            gb_markers.markers.append(right_bound)

            marker.id = i*3 + 1
            marker.color.g = 1.0
            marker.pose.position.x = wpnt.x_m - wpnt.d_left * math.cos(angle)
            marker.pose.position.y = wpnt.y_m - wpnt.d_left * math.sin(angle)
            left_bound = deepcopy(marker)
            gb_markers.markers.append(left_bound)

            marker.id = i*3 + 2
            marker.color.b = 1.0
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            middle = deepcopy(marker)
            gb_markers.markers.append(middle)

        # Publish stuff
        while not rospy.is_shutdown():
            gb_wpnts.header.stamp = rospy.Time.now()
            self.pub_gb.publish(gb_wpnts)
            self.pub_markers.publish(gb_markers)
            rate.sleep()

    def circle_loop_reverse(self, center=(0,0), r=3, nb_points=10):
        rate = rospy.Rate(5)  # rate in hertz

        angles = [i*2*math.pi/nb_points for i in range(nb_points+1)]
        gb_wpnts = WpntArray()
        gb_markers = MarkerArray()


        for i, angle in enumerate(angles):
            # Pass Wpnts
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = r * math.cos(-angle) + center[0]
            wpnt.y_m = r * math.sin(-angle) + center[1]
            wpnt.s_m = r * angle
            wpnt.psi_rad = (-angle + 3*math.pi/2) % (2*math.pi)
            wpnt.vx_mps = self.speed
            wpnt.d_left = 3
            wpnt.d_right = abs(2 * math.cos(angle)) - 1
            gb_wpnts.wpnts.append(wpnt)

            #Pass Markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.pose.orientation.w = 1
            
            marker.id = i*3
            marker.color.r = 1.0
            marker.pose.position.x = wpnt.x_m - wpnt.d_right * math.cos(-angle)
            marker.pose.position.y = wpnt.y_m - wpnt.d_right * math.sin(-angle)
            right_bound = deepcopy(marker)
            gb_markers.markers.append(right_bound)

            marker.id = i*3 + 1
            marker.color.g = 1.0
            marker.pose.position.x = wpnt.x_m + wpnt.d_left * math.cos(-angle)
            marker.pose.position.y = wpnt.y_m + wpnt.d_left * math.sin(-angle)
            left_bound = deepcopy(marker)
            gb_markers.markers.append(left_bound)

            marker.id = i*3 + 2
            marker.color.b = 1.0
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            middle = deepcopy(marker)
            gb_markers.markers.append(middle)

        # Publish stuff
        while not rospy.is_shutdown():
            gb_wpnts.header.stamp = rospy.Time.now()
            self.pub_gb.publish(gb_wpnts)
            self.pub_markers.publish(gb_markers)
            rate.sleep()

    def ellipse_loop(self, center=(0,0), a=3, b=5, nb_points=10):
        rate = rospy.Rate(5)  # rate in hertz

        angles = [i*2*math.pi/nb_points for i in range(nb_points+1)]
        gb_wpnts = WpntArray()
        gb_markers = MarkerArray()
        s = 0
        prev_angle = 0


        for i, angle in enumerate(angles):
            # Pass Wpnts
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.x_m = a * math.cos(angle) + center[0]
            wpnt.y_m = b * math.sin(angle) + center[1]
            # TODO: s_m is approximated
            s += a*(angle-prev_angle)*np.sqrt(1 - (a**2 - b**2)/(a**2) * math.sin(angle)**2)
            wpnt.s_m = s
            prev_angle = angle
            wpnt.psi_rad = (angle + math.pi/2) % (2*math.pi)
            wpnt.vx_mps = self.speed
            wpnt.d_left = 2
            wpnt.d_right = 2
            gb_wpnts.wpnts.append(wpnt)

            #Pass Markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = marker.SPHERE
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.orientation.w = 1

            marker.id = i*3
            marker.color.r = 1.0
            marker.pose.position.x = wpnt.x_m + wpnt.d_right * math.cos(angle)
            marker.pose.position.y = wpnt.y_m + wpnt.d_right * math.sin(angle)
            right_bound = deepcopy(marker)
            gb_markers.markers.append(right_bound)

            marker.id = i*3 + 1
            marker.color.g = 1.0
            marker.pose.position.x = wpnt.x_m - wpnt.d_left * math.cos(angle)
            marker.pose.position.y = wpnt.y_m - wpnt.d_left * math.sin(angle)
            left_bound = deepcopy(marker)
            gb_markers.markers.append(left_bound)

            marker.id = i*3 + 2
            marker.color.b = 1.0
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            middle = deepcopy(marker)
            gb_markers.markers.append(middle)
            

        # Publish stuff
        while not rospy.is_shutdown():
            gb_wpnts.header.stamp = rospy.Time.now()
            self.pub_gb.publish(gb_wpnts)
            self.pub_markers.publish(gb_markers)
            rate.sleep()

if __name__ == "__main__":
    gbnode = GBPubNode()
