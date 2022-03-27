#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import this 
import sys
sys.path.append(‘../’)

from waypoint import Waypoint

class WaypointsList():
    """
    WaypontsList represent a list of waypoints whith are the passing point of waypoint navigation task. 
    """
    def __init__(self):
        self.waypointsList = []

    def append(self,waypoint):

        if type(waypoint) is not Waypoint :
            return False

        self.waypointsList.append(waypoint)

    def pop(self):

        return self.waypointsList.pop()

    
        
        