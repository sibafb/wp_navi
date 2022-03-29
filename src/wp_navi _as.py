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

# クラス
class WpNavi():
    def __init__(self, robot_name = ""): 
        rospy.init_node('wp_navi')
        rospy.on_shutdown(self.shutdown) 

        self.ac = actionlib.SimpleActionClient( robot_name + '/move_base', MoveBaseAction)

        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The servers comes up")

        self.goal = MoveBaseGoal()  # ゴールの生成
        self.goal.target_pose.header.frame_id = 'map'  # 地図座標系
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

        # [x,y,theta]
        self.way_point_robot1 = [[ 6.1, 0.8, 0.0 * pi], [6.1, 4.2, 0.5 * pi],
                     [ 2.7, 4.2, -1.0 * pi], [2.7, 0.8, -0.5 * pi],
                     [999, 999, 999]]
   
    # シャットダウン時の処理
    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal()  # ゴールをキャンセル


if __name__ == '__main__':
    # 例外処理。rospy.ROSInterruptExceptionを捕まえる。
    # この例外はCrl+cキーが押されるときに発生するので、
    # この例外処理によりこのノードが終了する。
    try:
        WpNavi()
        while not rospy.is_shutdown():
            rospy.spin()
            delay(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")