#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import deque

from waypoint import Waypoint

class Waypoints():
    """
    Wayponts represent a list of waypoints whith are the passing point of waypoint navigation task. 
    """
    def __init__(self):
        self.waypoints = deque()
        self._idx = 0

    def append(self,waypoint):

        if type(waypoint) == type(Waypoint) :
            raise TypeError(str(waypoint) + 'is not waypoint type')

        self.waypoints.append(waypoint)

    def pop(self):

        return self.waypoints.pop()

    def dequeue(self):

        return self.waypoints.popleft()

    def dequeue_and_append(self):
        
        ret = self.waypoints.popleft()
        self.waypoints.append(ret)

        return ret

    def __iter__(self):
        return self

    def next(self):
        rospy.loginfo("next")
        if self._idx == len(self.waypoints):
            raise StopIteration()
        ret = self.waypoints[self._idx]
        self._idx += 1
        return ret

    def __str__(self):
        return str(self.waypoints)

from math import pi
import rospy

if __name__ == '__main__':

    rospy.init_node('waypiubts', anonymous=True)

    rospy.loginfo("hello")

    wp = Waypoints()
    wp.append(Waypoint("first", 6.1, 0.8,  0.0 * pi))
    wp.append(Waypoint("second", 6.1, 4.2,  0.5 * pi))
    wp.append(Waypoint("third", 2.7, 4.2, -1.0 * pi))
    wp.append(Waypoint("fourth", 2.7, 0.8, -0.5 * pi))

    for p in wp:
        rospy.loginfo("loop")
        rospy.loginfo(str(p))
