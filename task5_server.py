#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi):
from math import sqrt, pow, pi
# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import colour_search
from collections import Counter
import colour_search_obj
import time


# Import some other useful Python Modules
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.rate = rospy.Rate(5)

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-20, 21)
        self.yaw = self.robot_odom.yaw
        self.posyaw0= self.yaw # initialize xy yaw at the start
        self.lower = (115,50,100)
        self.upper = (130,255,255)
        self.min_distance = 100000

    def detect_pillar(self):
        detect_pillar = False
        while detect_pillar == False:
            if colour_search_obj.colour_search_obj().main(self.lower ,self.upper):
                detect_pillar = True
                break
            else:

                StartTime = rospy.get_rostime()          
                while rospy.get_rostime().secs - StartTime.secs > 5:
                    self.robot_controller.set_move_cmd(0.3, 0.0)
                    self.robot_controller.publish()
            
                    while self.left_arc_min <= 0.5: # if the left side is too close 
                        self.robot_controller.set_move_cmd(0.0, -0.5)
                        self.robot_controller.publish()
                    
                    while self.right_arc_min<=0.5: # if the right side is too close 
                        self.robot_controller.set_move_cmd(0.0,0.5)
                        self.robot_controller.publish()
       
        print("TARGET BEACON IDENTIFIED: Beaconing initiated")
       

    def move_to_pillar(self):
        while self.min_distance > 450:
            self.robot_controller.set_move_cmd(0.3, 0.0)
            self.robot_controller.publish()




    def action_server_launcher(self, goal): 
        self.get_target()

        self.maze_nav()
        
        #add the colour obj here
        #add the code which move infront to target pillar
        #you need to scan the target colour first, before move to it
        if self.detect_pillar():
            self.move_to_pillar()
        self.robot_controller.stop()
        print("FINAL CHALLENEGE COMPLETE: The robot has now stopped")

        
    def get_target(self):
        global hsv
        
        r = rospy.Rate(10)
        complete_1 = False
        complete_2 =False
        complete_3 =False
        complete_4 = False
        moving = True
        self.x = self.robot_odom.posx
        self.y = self.robot_odom.posy
        self.yaw = self.robot_odom.yaw
        self.posx0 = self.x
        self.posy0 = self.y
        self.posyaw0= self.yaw # initialize xy yaw at the start
        # Get the current robot odometry:
        
        print("The robot will start to move now...")
        # set the robot velocity:
        while moving :
            while complete_1 == False:
                self.x = self.robot_odom.posx
                self.y = self.robot_odom.posy
                self.yaw = self.robot_odom.yaw # update the value
             
            
                if abs(self.posyaw0 -self.yaw) <= 90 :
                    self.robot_controller.set_move_cmd(0.0, 0.4)
                    self.robot_controller.publish() # keep turning until it turned 90 degree
                    
                    
            
                if abs(self.posyaw0 -self.yaw) >= 90:
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    result = colour_search.colour_search().main()
                    hsv = result[0]
                    complete_1 = True
                    break
            
            self.posx0 = self.x
            self.posy0 = self.y
            self.posyaw0= self.yaw 
            while complete_2 ==False:
                global colour_detected
                self.x = self.robot_odom.posx
                self.y = self.robot_odom.posy
                self.yaw = self.robot_odom.yaw # update the value
            
                if abs(self.posyaw0 -self.yaw) <= 90 :
                    self.robot_controller.set_move_cmd(0.0, 0.4)
                    self.robot_controller.publish() # keep turning until it turned further90 degree
                    
            
                if abs(self.posyaw0 -self.yaw) >= 90:
                  
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    result = colour_search.colour_search().main()
                    hsv = result[0]
                    colour_range = hsv[:,0]
                    h_range = colour_range[:,0]
                    s_range = colour_range[:,1]
                    v_range = colour_range[:,2]
                    hsv =  ((Counter(h_range).most_common(1)[0][0]),
                            (Counter(s_range).most_common(1)[0][0]),
                            (Counter(v_range).most_common(1)[0][0]))
                      
                    if 0<=hsv[0]<=9  and 50<= hsv[1]<=255 :
                        self.lower=(0,50,100)
                        self.upper=(9,255,255)
                        colour_detected = "red"
                        #print(colour_detected)
                    if  159<=hsv[0]<=180 and 50<=hsv[1]<255 :
                        colour_detected = "red"
                        self.lower=(159,50,100)
                        self.upper=(180,255,255)
                       #print(colour_detected)
                    if 85<=hsv[0]<=93  and 50<= hsv[1]<=255 :
                        colour_detected ="turquoise"
                        self.lower=(85,50,100)
                        self.upper=(93,255,255)
                    if 129<=hsv[0]<=155  and 50<= hsv[1]<=255 : 
                        colour_detected ="purple"
                        self.lower=(129,50,100)
                        self.upper=(155,255,255)
                    if 115<=hsv[0]<=130  and 50<= hsv[1]<=255 : 
                        colour_detected ="blue"
                        self.lower=(115,50,100)
                        self.upper=(130,255,255)
                    if 25<=hsv[0]<=35  and 50<= hsv[1]<=255 : 
                        colour_detected ="yellow"
                        self.lower=(25,50,100)
                        self.upper=(35,255,255)
                    if 55<=hsv[0]<=65  and 50<= hsv[1]<=255 : 
                        colour_detected ="green"
                        self.lower=(55,50,100)
                        self.upper=(65,255,255)
                    for i in range(0,5) :
                        i=0
                        print( 'SEARCH INITIATED: The target beacon colour is '+colour_detected+" .")
                        i+=1
                    
                    complete_2 = True
                    
                    break
            self.posx0 = self.x
            self.posy0 = self.y
            self.posyaw0= self.yaw 

            time_left = 8
            while complete_3 ==False:
                  
                while time_left > 0:
                    self.robot_controller.set_move_cmd(0.0, 0.4)
                    self.robot_controller.publish() # keep turning until it turned 180 degree
                    time.sleep(1)
                    time_left = time_left - 1
                complete_3 = True
                break

            while complete_4 == False:
                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish() # keep turning until it turned 180 degree
                #colour_search_obj.colour_search_obj().main(lower,upper)
                complete_4 = True
                moving = False
                #rospy.loginfo("Goal completed.")
                #self.actionserver.set_succeeded()
                # stop the robot:
                #self.robot_controller.stop()
    
    def scan_callback(self, scan_data):
        left_arc0 = scan_data.ranges[0:20]
        right_arc0 = scan_data.ranges[-20:]
        front_arc0 = np.array(left_arc0 + right_arc0)

        left_arc1 = scan_data.ranges[40:50]
        right_arc1 = scan_data.ranges[50:70]
        front_arc1 = np.array(left_arc1 + right_arc1)

        left_arc2 = scan_data.ranges[290:300]
        right_arc2 = scan_data.ranges[300:320]
        front_arc2 = np.array(left_arc2 + right_arc2)
        front_arc = np.array(left_arc0[::-1] + right_arc0[::-1])

        self.min_distance = front_arc.min()

        self.front = front_arc0.min()
        self.front_right = front_arc2.min()


    def maze_nav(self):
        global m00
        global m00_min
        global cy

        search = False
        while search == False:
            if self.front_right > 0.38:
                self.robot_controller.set_move_cmd(0.0, -0.3)
            elif self.front < 0.38 or self.front_right < 0.18:
                self.robot_controller.set_move_cmd(0.0, 0.3)
            else:
                self.robot_controller.set_move_cmd(0.25, 0.0)
            self.robot_controller.publish()
            self.rate.sleep()
            result = colour_search.colour_search().main()
            m00 = result[1]
            cy = result[2]
            m00_min = result[3]
            
            if m00 > m00_min:
                # if the robot is far enough away from start arena
                if self.distance() > 2:
                    # blob detected
                    if cy >= 0 and cy <= 1120:
                        search = True
        #preventing buffering errors when stopping
        i = 0
        for i in range(5):
            self.robot_controller.set_move_cmd(0.0,0.0)
            self.rate.sleep()
            i += 1
    
    def distance(self):
        xdistance = self.x - self.posx0
        ydistance = self.y - self.posy0
        distance = sqrt(pow(xdistance, 2) + pow(ydistance, 2))
        return distance

                  
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
