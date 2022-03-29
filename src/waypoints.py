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

    def append(self,waypoint):

        if type(waypoint) is not Waypoint :
            return False

        self.waypoints.append(waypoint)

    def pop(self):

        return self.waypoints.pop()

    def dequeue(self):

        return self.waypoints.popleft()

    def dequeue_and_append(self):
        
        ret = self.waypoints.popleft()
        self.waypoints.append(ret)

        return ret


    
        
        