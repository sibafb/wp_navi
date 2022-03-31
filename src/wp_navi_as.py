#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from math import pi

from Wpnav.msg import WpnavAction, WpnavResult, WpnavFeedback

from waypoint import Waypoint

from waypoints import Waypoints
from naviclient import NavigationClient

class WpNavi():
    def __init__(self, robot_name = ""): 
        rospy.init_node('wp_navi')
        rospy.on_shutdown(self.shutdown) 

        self.__nav_client = NavigationClient()
        self.__waypoints = Waypoints()

        ## resister waypoints
        self.__waypoints.append([ 6.1, 0.8,  0.0 * pi])
        self.__waypoints.append([ 6.1, 4.2,  0.5 * pi])
        self.__waypoints.append([ 2.7, 4.2, -1.0 * pi])
        self.__waypoints.append([ 2.7, 0.8, -0.5 * pi])

        self._action_server = actionlib.SimpleActionServer( 'wp_navigation_action', WpnavAction, execute_cb = self.wp_navigation, auto_start = False )

        self._action_server.start()

    def wp_navigation(self):

        rate = rospy.Rate( 1.0 ) 

        for waypoint in self.__waypoints:

            self.__nav_client.send_goal(waypoint)

            while self.__nav_client.is_reached_goal() == False:
                feedback = WpnavFeedback( next_wp_index = self.__waypoints.index(), next_wp_label = self.__waypoints.next_label() )
                rate.sleep()

        result = WpnavResult()
        self._action_server.set_succeeded( result )

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.__nav_client.cancel_goal()

if __name__ == '__main__':
    try:
        WpNavi()
        while not rospy.is_shutdown():
            rospy.spin()
            delay(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")