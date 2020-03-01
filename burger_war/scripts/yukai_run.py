#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from std_msgs.msg import String

from IPython.core.debugger import set_trace
import numpy as np

PI = 3.1416

'''
Run to target position and orientation in order
using navigation packages.
'''

class YukaiBot():
    def __init__(self):
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('goal', MoveBaseGoal, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.detect_red_sub = rospy.Subscriber('enemy', String, self.detectRedCallback)
        self.yukai_position = np.array([-1.3, 0.0])
        self.yukai_direction = 0.0

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

    def detectRedCallback(self, data):
        print(data)
        if data == "0.0":
            print('detect enemy go left from screen')
            self.turn_to(PI/6)
        if data == "1.0":
            print('detect enemy go right from screen')
            self.turn_to(-PI/6)

    def poseCallback(self, data):
        self.yukai_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.yukai_direction = rpy[2]

    def turn_to(self, yaw):
        self.set_goal(self.yukai_position[0], self.yukai_position[1], self.yukai_direction + yaw)

    def strategy(self):
        rospy.Rate(10)
        # set_trace()

        while not rospy.is_shutdown():
            self.set_goal(-0.80,0.45,0)
            self.set_goal(-0.80,-0.45,0)
            self.set_goal(-0.5,0,0)

            self.set_goal(0,0.5,0)
            self.set_goal(0,0.5,-PI/2)
            self.set_goal(0,0.5,PI)
            
            self.set_goal(0.5,0,PI)
            self.set_goal(0.80,-0.45,PI)
            self.set_goal(0.80,0.45,PI)

            self.set_goal(0,-0.5,PI)
            self.set_goal(0,-0.5,PI/2)
            self.set_goal(0,-0.5,0)

if __name__ == '__main__':
    rospy.init_node('yukai_run') #declare node name
    bot = YukaiBot()
    bot.strategy()