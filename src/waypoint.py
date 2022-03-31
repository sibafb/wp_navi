#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Pose,Quaternion
import tf

class Waypoint():
    """
    Waypont represents a waypoint, whitch is a goal position and orientation of robot. 
    When the robot is carring on the Navigation Task.  
    """
    def __init__(self, label, pos_x , pos_y, pos_theta):
        self.__x = pos_x
        self.__y = pos_y
        self.__theta = pos_theta
        self.__label = label 

    def toList(self):
        return [self.__x, self.__y, self.__theta]

    def toPose(self):

        pose = Pose()

        pose.position.x = self.__x
        pose.position.y = self.__y
        pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, self.__theta)
        pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        return pose

    def labelStr(self):
        return self.__label

    def __str__(self):
        return "["+str(self.__label)+", "+str(self.__x)+", "+str(self.__y)+", "+str(self.__theta)+"]"
