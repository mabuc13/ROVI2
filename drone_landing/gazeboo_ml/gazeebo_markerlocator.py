import sys, time
import math

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from markerlocator.MarkerTracker import MarkerTracker

# OpenCV
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def gazeebo_marker_locator_node():

        rospy.init_node('gazeebo_marker_locator', anonymous = False)
        rate = rospy.rate(10) #This is in Hz

        image_sub = rospy.Subscriber("/iris/camera/image_raw",Image, callback,queue_size = 1)

def callback(image):
    kernel = 51
    order = 7
    scale = 1
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data,"bgr8")
    grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ml = MarkerTracker(order,kernel,scale)
    pos = ml.locate_marker(grey)
    print pos.x, pos.y

if __name__ == '__main__':
    gazeebo_marker_locator_node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
