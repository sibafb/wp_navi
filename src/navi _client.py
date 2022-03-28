#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from waypoint import Waypoint

class NavigationClient():
    def __init__(self, goal_frame = 'map',robot_name = ''): 

        self.goal_frame = goal_frame

        ### Action client 
        self.navClient = actionlib.SimpleActionClient( robot_name + '/move_base', MoveBaseAction)

        while not self.ac.wait_for_server(rospy.Duration(30)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        ### Subscliber
        self.navStatus = rospy.Subscliber( robot_name + '/move_base/status', GoalStatusArray,self.cb_status)

        rospy.loginfo("The servers comes up")

    def send_goal(self, waypoint):

        wp_pose = waypoint.ToPose()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position = wp_pose.position
        goal.target_pose.pose.orientation = wp_pose.orientation

        self.navClient.send_goal(self.goal)
    
    def cancel_goal(self):

        self.navClient.cancel_goal()
        
    def cb_status(self, status):
        pass

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal() 
