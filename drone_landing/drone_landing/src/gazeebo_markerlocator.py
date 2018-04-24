#!/usr/bin/env python
import sys, time
import math

# numpy and scipy
import numpy as np
from MarkerLocator.MarkerTracker import MarkerTracker

# OpenCV
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray


def gazeebo_marker_locator_node():

    rospy.init_node('gazeebo_marker_locator', anonymous = False)
    rate = rospy.Rate(10) #This is in Hz
    image_sub = rospy.Subscriber("/iris/camera/image_raw",Image, callback,queue_size = 1)

def publish_location(data):
    marker_publisher = rospy.Publisher("/marker/location",Float32MultiArray,queue_size =1)
    marker_publisher.publish(data)

def callback(image):
    kernel = 51
    order = 6
    scale = 1
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image,"bgr8")
    grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ml = MarkerTracker(order,kernel,scale)
    pos = ml.locate_marker(grey)
    message_array = Float32MultiArray(data= [pos.x, pos.y, pos.theta, pos.quality])
    publish_location(message_array)


if __name__ == '__main__':
    print "Entering main"
    gazeebo_marker_locator_node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

