#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
import cv2
import utils

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError

CONTROL_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
CAMERA_TOPIC = '/camera/color/image_raw'
IMAGE_TOPIC = '/camera/color/image_processed'

class Vision_Controller:
    
    # Initializer
    def __init__(self, ):
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_process_cb, queue_size = 1)
        self.controller_pub = rospy.Publisher(CONTROL_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 1)
        
    # Callback to process the incoming image from the camera    
    def image_process_cb(self, msg):
        x, y = color_track(msg)
        print x, y

        # CODE HERE
        # Put in code to interpret the x,y position and make appropriate control

def color_track(self, msg):
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    size_limit = 100000 

    # Convert to CV2 image
    try:
        image = CvBridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
    print(e)

    image      = cv2.Smooth(image, image, cv2.CV_BLUR, 3)
    hsv_image  = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)
    mask_image = cv2.inRange(cv_image_hsv,lower_blue,upper_blue)

    moments = cv2.Moments(mask_image, 0)        
    area    = cv2,GetCentralMoment(moments, 0, 0)

    if(area > size_limit):
        x = cv2.GetSpatialMoment(moments, 1, 0)/area
        y = cv2.GetSpatialMoment(moments, 0, 1)/area

        overlay = cv.CreateImage(cv.GetSize(img), 8, 3)
        cv.Circle(overlay, (x, y), 2, (255, 255, 255), 20)
        cv.Add(image, overlay, image)
        cv.Merge(mask_image, None, None, None, image)

    # Publish the image so we can view it for debug
    self.image_pub.publish(CvBridge.cv2_to_imgmsg(image, "passthrough"))

    return x, y
        
if __name__ == '__main__':
  rospy.spin()
