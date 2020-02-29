#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

'''
Run to target position and rotation
in order with navigation.

'''

class YukaiBot():
    def __init__(self, "NoName"):
        self.name = bot_name
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def