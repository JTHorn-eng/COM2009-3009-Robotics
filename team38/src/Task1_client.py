#! /usr/bin/env python

import rospy
import actionlib

from com2009_actions.msg import SearchAction, SearchGoal

class action_client(object):
   
    def feedback_callback(self, feedback_data):
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            

    def __init__(self):
 
        self.action_complete = False
        
        rospy.init_node("search_action_client")

        self.rate = rospy.Rate(60)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

        self.distance = 0.0

        self.i = 0

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity = 0.24, approach = 0.55)
        prempt = False
        StartTime = rospy.get_rostime()
        print("the robot will now move for 60 seconds...")
        while self.client.get_state() < 2:
            if rospy.get_rostime().secs - StartTime.secs > 60 :
                rospy.logwarn("Cancelling goal now...")
                self.client.cancel_goal()
                rospy.logwarn("Goal Cancelled")
                prempt = True
                rospy.loginfo('60 seconds have elapsed, stopping the robot...')
                break

            self.rate.sleep()
        
        self.action_complete = True
       

if __name__ == '__main__':
    ac_object = action_client()
    try:
        ac_object.main()
    except rospy.ROSInterruptException:
        pass