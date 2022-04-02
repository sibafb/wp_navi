#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *

from math import pi

from wp_navi.msg import WpnavAction, WpnavResult, WpnavFeedback

from waypoint import Waypoint
from waypoints import Waypoints
from naviclient import NavigationClient

class WpNavi():
    def __init__(self, robot_name = ""): 
        rospy.init_node('wp_navi')
        rospy.on_shutdown(self.shutdown) 

        self.__nav_client = NavigationClient(robot_name = robot_name)
        self.__waypoints = Waypoints()

        self._action_server = actionlib.SimpleActionServer( 'wp_navigation_action', WpnavAction, execute_cb = self.wp_navigation, auto_start = False )

        self._action_server.start()

    def add_waypoint(self, waypoint):
        rospy.loginfo('add waypoint')
        self.__waypoints.append(waypoint)

        rospy.loginfo('waypoint num is now :' + len(self.__waypoints))

    def wp_navigation(self):

        rate = rospy.Rate( 1.0 ) 

        for waypoint in self.__waypoints:

            self.__nav_client.send_goal(waypoint)

            while self.__nav_client.is_reached_goal() == False:

                feedback = WpnavFeedback( next_wp_index = self.__waypoints.index(), next_wp_label = self.__waypoints.next_label() )
                self._action_server.publish_feedback(feedback)
                rate.sleep()

        result = WpnavResult()
        self._action_server.set_succeeded( result )

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.__nav_client.cancel_goal()

if __name__ == '__main__':
    try:
        wp = WpNavi("create1")

        wp.add_waypoint(Waypoint("first", 6.1, 0.8,  0.0 * pi))
        wp.add_waypoint(Waypoint("second", 6.1, 4.2,  0.5 * pi))
        wp.add_waypoint(Waypoint("third", 2.7, 4.2, -1.0 * pi))
        wp.add_waypoint(Waypoint("fourth", 2.7, 0.8, -0.5 * pi))

        while not rospy.is_shutdown():
            rospy.spin()
            rospy.sleep(0.2)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")