#!/usr/bin/env python

import rospy
import os

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
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True:
        height, width, channels = cv_img.shape

        print("Obtained an image of height {}px and width {}px.".format(height, width))

        show_and_save_image(cv_img, img_name = "step1_original")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()