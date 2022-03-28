#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from waypoint import Waypoint

class NavigationClient():
    def __init__(self, goal_frame = 'map',robot_name = ''): 

        self.__goal_frame = goal_frame
        self.__goal_status = GoalStatus.PENDING

        ### Action client 
        self.__navClient = actionlib.SimpleActionClient( robot_name + '/move_base', MoveBaseAction)

        while not self.__navClient.wait_for_server(rospy.Duration(30)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        ### Subscliber
        self.__navStatus = rospy.Subscliber( robot_name + '/move_base/status', GoalStatusArray, self.cb_status)

        rospy.loginfo("The servers comes up")

    def send_goal(self, waypoint):

        wp_pose = waypoint.ToPose()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position = wp_pose.position
        goal.target_pose.pose.orientation = wp_pose.orientation

        self.navClient.send_goal(goal)
    
    def cancel_goal(self):

        self.navClient.cancel_goal()
        
    def cb_status(self, status):
        '''
        http://wiki.ros.org/actionlib/DetailedDescription
        '''

        if self.__goal_status != status.status_list[-1].status :
            self.__goal_status = status.status_list[-1].status

            if self.__goal_status == GoalStatus.PENDING:
                pass
            elif self.__goal_status == GoalStatus.ACTIVE:
                pass
            elif self.__goal_status == GoalStatus.PREEMPTED:
                pass
            elif self.__goal_status == GoalStatus.SUCCEEDED:
                pass
            elif self.__goal_status == GoalStatus.ABORTED:
                pass
            elif self.__goal_status == GoalStatus.REJECTED:
                pass
            elif self.__goal_status == GoalStatus.PREEMPTING:
                pass
            elif self.__goal_status == GoalStatus.RECALLING:
                pass
            elif self.__goal_status == GoalStatus.RECALLED:
                pass
            elif self.__goal_status == GoalStatus.LOST:
                pass  


        

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal() 
