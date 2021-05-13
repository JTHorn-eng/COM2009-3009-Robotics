#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveTB3(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def move(self, time, linear, angular):
        StartTime = rospy.get_rostime()
        self.set_move_cmd(linear, angular)
        self.publish()
        while rospy.get_rostime().secs - StartTime.secs < time:
            continue
        self.stop()


    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()