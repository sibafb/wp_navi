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

class WpNavi():
    def __init__(self):
        rospy.init_node('wp_navi')
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber('button_pushed', Bool, self.button_pushed)

        self.ac = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)

        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The servers comes up")

        self.goal = MoveBaseGoal() 
        self.goal.target_pose.header.frame_id = 'map' 
        self.goal.target_pose.header.stamp = rospy.Time.now() 

        self.way_point_robot1 = [[ 6.1, 0.8, 0.0 * pi], [6.1, 4.2, 0.5 * pi],
                     [ 2.7, 4.2, -1.0 * pi], [2.7, 0.8, -0.5 * pi],
                     [999, 999, 999]]

        self.i = 0

    def button_pushed(self, button_pushed):
        if button_pushed.data == True:
            self.goal.target_pose.pose.position.x = self.way_point_robot1[self.i][0]
            self.goal.target_pose.pose.position.y = self.way_point_robot1[self.i][1]

            if self.way_point_robot1[self.i][0] == 999:
                return

            q = tf.transformations.quaternion_from_euler(0, 0, self.way_point_robot1[self.i][2])
            self.goal.target_pose.pose.orientation = Quaternion(
                q[0], q[1], q[2], q[3])

            rospy.loginfo("Sending goal: No" + str(self.i + 1))

            self.ac.send_goal(self.goal)

            succeeded = self.ac.wait_for_result(rospy.Duration(30))

            state = self.ac.get_state()
            if succeeded:
                rospy.loginfo(
                    "Succeeded: No." + str(self.i + 1) + "(" + str(state) + ")")
            else:
                rospy.loginfo(
                    "Failed: No." + str(self.i + 1) + "(" + str(state) + ")")

            self.i = self.i + 1

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal()  # ゴールをキャンセル


if __name__ == '__main__':
    try:
        WpNavi()
        while not rospy.is_shutdown():
            rospy.spin()
            delay(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")