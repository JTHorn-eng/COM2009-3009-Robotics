#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-20, 21)
        self.min_distance = 0
        self.left_arc_min = 0
        self.right_arc_min = 0
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        self.left_arc_min = np.array(left_arc).min()
        self.right_arc_min=np.array(right_arc).min()
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
    
    def action_server_launcher(self, goal):
        r = rospy.Rate(60)

        success = True
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

       

        # Get the current robot odometry:
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        
        while success :
        
        
         while self.min_distance > goal.approach_distance:
             self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
             self.robot_controller.publish()
             # check if there has been a request to cancel the action mid-way through:
             if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the camera sweep.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                break
                # exit the loop:
         while self.left_arc_min <= goal.approach_distance: # if the left side is too close 
             self.robot_controller.set_move_cmd(0.0, -0.5)
             self.robot_controller.publish()
             
             
             
         while self.right_arc_min<=goal.approach_distance: # if the right side is too close 
             self.robot_controller.set_move_cmd(0.0,0.5)
             self.robot_controller.publish()
             
                  
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
