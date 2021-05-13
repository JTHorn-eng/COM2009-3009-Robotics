#! /usr/bin/env python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

class colour_search(object):

    def __init__(self):
        
        #we need to get these colours from the arena's scatter plot
        self.colour_thresholds = {

            "red" : [(175, 205, 100), (255, 255, 255)],
            "turquoise" : [(85, 140, 100), (93, 255, 255)],
            "purple" : [(145, 198, 100), (155, 255, 255)],
            "blue" : [(115, 220, 100), (125, 255, 255)],
            "yellow" : [(25, 160, 100), (35, 255, 255)],
            "green" : [(55, 140, 100), (65, 255, 255)],



        }

        self.cd = [[],[],[],[],[],[], []] #leave empty
        
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0 

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00_min = 10000

        self.target_colour = ""

        self.found = False
        self.colour_index = 0

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        itera = 0
        for colour in self.colour_thresholds:
            
            
            mask = cv2.inRange(hsv_img, self.colour_thresholds[colour][0], self.colour_thresholds[colour][1])
            #res = cv2.bitwise_and(crop_img, crop_img, mask = mask)
        
            m = cv2.moments(mask)
            self.cd[itera][0] = colour                         #colour
            self.cd[itera][1] = m['m00']                       #m00
            self.cd[itera][2] = m['m10'] / (m['m00'] + 1e-5)   #cy
            itera+=1
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:

            #PART 1
            while self.target_colour != "":
                itera = 0
                print(itera)
                if self.cd[itera][1] > self.m00_min:
                    # blob detected
                    if self.cd[itera][2] >= 560-100 and self.cd[itera][2] <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            self.stop_counter = 30
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'
                    
                if self.move_rate == 'fast':
                    print("Scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print("Found colour: " + self.cd[itera][0])
                    self.target_colour = self.cd[itera][0]
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop' and self.stop_counter > 0:
                    print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cd[itera][2], self.stop_counter))
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                else:
                    print("Found colour: " + self.cd[itera][0])
                    self.target_colour = self.cd[itera][0]
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                
                self.robot_controller.publish()
                self.rate.sleep()
                if (itera > len(self.colour_thresholds)):
                    itera = 0
                else:
                    itera += 1

                self.colour_index = itera


            print("SEARCH INITIATED: The target colour is " + self.target_colour)

            #MOVE TO CENTER
            self.robot_controller.move(10, 0.1, 0)

            #ROTATE TO FACE FIRST PILLAR
            self.robot_controller.move(1, 0, -0.1)

            while not self.found:
                print("Finding pillar")
                if self.cd[self.colour_index][1] > self.m00_min:
                    # blob detected
                    if self.cd[self.colour_index][2] >= 560-100 and self.cd[self.colour_index][2] <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            self.stop_counter = 30
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'
                    
                if self.move_rate == 'fast':
                    print("Scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print("Found colour: " + self.cd[itera][0])
                    self.target_colour = self.cd[itera][0]
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop' and self.stop_counter > 0:
                    print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                else:
                    print("Found colour: " + self.cd[itera][0])
                    self.target_colour = self.cd[itera][0]
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                
                self.robot_controller.publish()
                self.rate.sleep()


            



            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass