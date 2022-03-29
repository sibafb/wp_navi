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

from Wpnav.msg import WpnavAction
from Wpnav.msg import WpnavResult
from Wpnav.msg import WpnavFeedback

from waypoint import Waypoint

from waypoints import Waypoints
from naviclient import NavigationClient

class WpNavi():
    def __init__(self, robot_name = ""): 
        rospy.init_node('wp_navi')
        rospy.on_shutdown(self.shutdown) 

        self._nav_client = NavigationClient()
        self._waypoints = Waypoints()

        ## resister waypoints
        self._waypoints.append([ 6.1, 0.8,  0.0 * pi])
        self._waypoints.append([ 6.1, 4.2,  0.5 * pi])
        self._waypoints.append([ 2.7, 4.2, -1.0 * pi])
        self._waypoints.append([ 2.7, 0.8, -0.5 * pi])

    # シャットダウン時の処理
    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal()


if __name__ == '__main__':
    try:
        WpNavi()
        while not rospy.is_shutdown():
            rospy.spin()
            delay(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")