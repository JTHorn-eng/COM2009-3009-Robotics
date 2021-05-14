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


# Import some other useful Python Modules
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
        self.yaw = self.robot_odom.yaw
        self.posyaw0= self.yaw # initialize xy yaw at the start
     
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        self.left_arc_min = np.array(left_arc).min()
        self.right_arc_min=np.array(right_arc).min()
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
    
    def action_server_launcher(self, goal):
        
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
                    hsv = colour_search.colour_search().main()
                   
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
                    hsv = colour_search.colour_search().main()
                    colour_range = hsv[:,0]
                    h_range = colour_range[:,0]
                    s_range = colour_range[:,1]
                    v_range = colour_range[:,2]
                    hsv =  ((Counter(h_range).most_common(1)[0][0]),
                            (Counter(s_range).most_common(1)[0][0]),
                            (Counter(v_range).most_common(1)[0][0]))
                      
                    if 0<=hsv[0]<=9  and 50<= hsv[1]<=255 :
                        lower=(0,50,100)
                        upper=(9,255,255)
                        colour_detected = "red"
                        #print(colour_detected)
                    if  159<=hsv[0]<=180 and 50<=hsv[1]<255 :
                        colour_detected = "red"
                        lower=(159,50,100)
                        upper=(180,255,255)
                       #print(colour_detected)
                    if 85<=hsv[0]<=93  and 50<= hsv[1]<=255 :
                        colour_detected ="turquoise"
                        lower=(85,50,100)
                        upper=(93,255,255)
                    if 129<=hsv[0]<=155  and 50<= hsv[1]<=255 : 
                        colour_detected ="purple"
                        lower=(129,50,100)
                        upper=(155,255,255)
                    if 115<=hsv[0]<=130  and 50<= hsv[1]<=255 : 
                        colour_detected ="blue"
                        lower=(115,50,100)
                        upper=(130,255,255)
                    if 25<=hsv[0]<=35  and 50<= hsv[1]<=255 : 
                        colour_detected ="yellow"
                        lower=(25,50,100)
                        upper=(35,255,255)
                    if 55<=hsv[0]<=65  and 50<= hsv[1]<=255 : 
                        colour_detected ="green"
                        lower=(55,50,100)
                        upper=(65,255,255)
                    for i in range(0,5) :
                        i=0
                        print( 'SEARCH INITIATED: The target colour is '+colour_detected+" .")
                        i+=1
                    
                    complete_2 = True
                    
                    break
            self.posx0 = self.x
            self.posy0 = self.y
            self.posyaw0= self.yaw 
            while complete_3 ==False:
                self.x = self.robot_odom.posx
                self.y = self.robot_odom.posy
                self.yaw = self.robot_odom.yaw # update the value
                
                if abs(self.posyaw0 -self.yaw) <= 180 :
                    self.robot_controller.set_move_cmd(0.0, 0.3)
                    self.robot_controller.publish() # keep turning until it turned 180 degree
                    
            
                if abs(self.posyaw0 -self.yaw) >= 180:
                    self.robot_controller.set_move_cmd(0.2, 0.0)
                    self.robot_controller.publish()
                    if abs(self.posy0 - self.y) >= 1:
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                        self.robot_controller.publish()    
                        complete_3 = True
                      
                        break
            while complete_4 == False:
                colour_search_obj.colour_search_obj().main(lower,upper)
                complete_4 = True
                rospy.loginfo("Goal completed.")
                self.actionserver.set_succeeded()
                # stop the robot:
                self.robot_controller.stop()
                

                  
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
