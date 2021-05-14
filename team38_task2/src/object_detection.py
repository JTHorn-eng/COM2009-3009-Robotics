#!/usr/bin/env python

import rospy
import os
from collections import Counter
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "task2_detection"
rospy.init_node(node_name)
print("Launched the '{}' node. Currently waiting for an image...".format(node_name))
rate = rospy.Rate(5)

cvbridge_interface = CvBridge()

waiting_for_image = True


def show_and_save_image(img, img_name):
    base_image_path = "/home/student/myrosdata/task2_image"
    full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

    cv2.imshow(img_name, img)
    cv2.waitKey(0)

    cv2.imwrite(full_image_path, img)
    print("Saved an image to '{}'\nimage dims = {}x{}px\nfile size = {} bytes".format(full_image_path, 
            img.shape[0], img.shape[1], os.path.getsize(full_image_path)))
def camera_cb(img_data):
    global waiting_for_image 
    global hsv_img 
    global hsv
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True:
        height, width, channels = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        print("Obtained an image of height {}px and width {}px.".format(height, width))
        colour_range = hsv_img[:,0]
        h_range = colour_range[:,0]
        s_range = colour_range[:,1]
        v_range = colour_range[:,2]
        print(Counter(h_range).most_common(1)[0][0])
        print(Counter(s_range).most_common(1)[0][0])
        print(Counter(v_range).most_common(1)[0][0])
        show_and_save_image(cv_img, img_name = "step1_original")

        waiting_for_image = False


def getHsv():
    colour_range = hsv_img[:,0]
    h_range = colour_range[:,0]
    s_range = colour_range[:,1]
    v_range = colour_range[:,2]
    hsv =  (Counter(h_range).most_common(1)[0][0],
            Counter(s_range).most_common(1)[0][0],
            Counter(v_range).most_common(1)[0][0])
    return hsv

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()