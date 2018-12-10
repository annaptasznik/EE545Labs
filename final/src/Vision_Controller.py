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
    def __init__(self, lowerLimit, upperLimit, min_area, speed, pixel_to_angle):
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_process_cb, queue_size = 1)
        self.controller_pub = rospy.Publisher(CONTROL_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 1)

        self.bridge = CvBridge()
        self.upperLimit = upperLimit
        self.lowerLimit = lowerLimit
        
        self.min_area = min_area
        self.speed = speed
        self.pixel_to_angle = pixel_to_angle

        self.counter = 0
        
    # Callback to process the incoming image from the camera    
    def image_process_cb(self, msg):
        # First look to see if we have found a blue object
        x, y = self.color_track(msg)
 
        # Interpret the x,y position and make appropriate control
        if((x != 0 and y !=0) and (y < 360) and (x < 280 or x < 360)):
            print 'blue object detected', self.counter, x, y
            self.counter += 1
            ads = self.compute_steering(x, y)
            self.controller_pub.publish(ads)

    def color_track(self, msg): 
        # Convert to CV2 image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to HSV color and apply thresholds to detect blue
        hsv_image  = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
        mask_image = cv2.inRange(hsv_image, self.lowerLimit, self.upperLimit)
            
        moments = cv2.moments(mask_image, 0)
        area    = moments['m00']

        if(area > self.min_area):
            x = int(moments['m10'] / area)
            y = int(moments['m01'] / area)
        
        else:
            x = 0 
            y = 0

        # Publish the image so we can view it for debug
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask_image, "passthrough"))

        return x, y

    def compute_steering(self, x, y):
        error = x - 320
        delta = float(-error * self.pixel_to_angle)
        
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed

        return ads
        
def main():
    rospy.init_node('Vision_Controller', anonymous=True)
    R_value = rospy.get_param("~R_value", 70)
    G_value = rospy.get_param("~G_value", 120)
    B_value = rospy.get_param("~B_value", 175)
    min_area = rospy.get_param("~min_area", 50000)
    speed = rospy.get_param("~speed", 1.0)
    pixel_to_angle = rospy.get_param("~pixel_to_angle", 0.0014)
    
    # Upper and lower limit for blue values
    blue = np.uint8([[[R_value, G_value, B_value]]])
    hsvBlue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)
    print(hsvBlue)
    
    lowerLimit = (hsvBlue[0][0][0]-10,100,100)
    upperLimit = (hsvBlue[0][0][0]+10,255,255)
    print(upperLimit)
    print(lowerLimit)
                                     
    vc = Vision_Controller(lowerLimit, upperLimit, min_area, speed, pixel_to_angle)
    rospy.spin()

if __name__ == '__main__':
    main()
  
