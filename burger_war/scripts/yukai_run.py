#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

from IPython.core.debugger import set_trace

'''
Run to target position and orientation in order
using navigation packages.
'''

class YukaiBot():
    def __init__(self):
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('goal', MoveBaseGoal, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def set_goal(self, x, y, yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(goal)
        wait = self.client.wait_for_result() #ここから進まない
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def strategy(self):
        rospy.Rate(10)
        pi = 3.1415
        # set_trace()

        while not rospy.is_shutdown():
            self.set_goal(-0.80,0.45,0)
            self.set_goal(-0.80,-0.45,0)
            self.set_goal(-0.5,0,0)

            self.set_goal(0,0.5,0)
            self.set_goal(0,0.5,-pi/2)
            self.set_goal(0,0.5,pi)
            
            self.set_goal(0.5,0,pi)
            self.set_goal(0.80,-0.45,pi)
            self.set_goal(0.80,0.45,pi)

            self.set_goal(0,-0.5,pi)
            self.set_goal(0,-0.5,pi/2)
            self.set_goal(0,-0.5,0)

if __name__ == '__main__':
    rospy.init_node('yukai_run') #declare node name
    bot = YukaiBot()
    bot.strategy()