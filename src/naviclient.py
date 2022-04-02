#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionFeedback,MoveBaseActionResult

from waypoint import Waypoint

class NavigationClient():
    """
    based on http://wiki.ros.org/move_base
    """
    def __init__(self, goal_frame = 'map',robot_name = ''): 

        self.__goal_frame = goal_frame
        self.__goal_status = GoalStatus.PENDING

        ### Action client 
        self.__navClient = actionlib.SimpleActionClient( robot_name + '/move_base', MoveBaseAction )

        while not self.__navClient.wait_for_server(rospy.Duration(10)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The "+str(robot_name)+" move_base servers comes up")

        ### Subscliber
        self.__navStatus = rospy.Subscriber( robot_name + '/move_base/status', GoalStatusArray, self.cb_status )
        self.__navFeedback = rospy.Subscriber( robot_name + '/move_base/feedback', MoveBaseActionFeedback, self.cb_feedback )
        self.__navResult = rospy.Subscriber( robot_name + '/move_base/result', MoveBaseActionResult, self.cb_result )


    def send_goal(self, waypoint):

        wp_pose = waypoint.toPose()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.__goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position    = wp_pose.position
        goal.target_pose.pose.orientation = wp_pose.orientation

        self.__navClient.send_goal(goal)
    
    def wait_for_result(self):
        self.__navClient.wait_for_result()

    def is_reached_goal(self):

        if self.__goal_status == GoalStatus.SUCCEEDED:
            return True

        return False

    def cancel_goal(self):
        self.__navClient.cancel_goal()

    def cb_result(self, result):
        pass

    def cb_feedback(self, feedback):
        pass

    def cb_status(self, status):
        '''
        http://wiki.ros.org/actionlib/DetailedDescription
        '''
        if len(status.status_list) == 0:
            return
        
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
