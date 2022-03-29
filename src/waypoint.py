#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import ENUM

from geometry_msgs.msg import Pose,Quaternion
import tf

class WaypointType(ENUM):
    Coordinate_Point_As_List = auto()


class Waypoint():
    """
    Waypont represents a waypoint, whitch is a goal position and orientation of robot. 
    When the robot is carring on the Navigation Task.  
    """
    def __init__(self, label, pos_x , pos_y, pos_theta):
        self.x = pos_x
        self.y = pos_y
        self.theta = pos_theta
        self.Type = WaypointType.Coordinate_Point_As_List

    def toList(self):
        return [self.x, self.y, self.theta]

    def toPose(self):

        pose = Pose()

        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        return pose
