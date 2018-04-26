#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sys
import signal
from geometry_msgs.msg import Vector3
import math
import numpy as np

import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

bridge = CvBridge()
centroids_str = ""
centroids_float = Float32MultiArray(data=[0, 0, 0, 0])
stats_float = 0

# my img conversion from ros to opencv
def callback(ros_data):
    # entering callback and loading img
    # print("enter callback")
    img = bridge.imgmsg_to_cv2(ros_data, "bgr8")

    # cvt to grayscale and binary img
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, bw = cv2.threshold(gray_img, 200, 255, cv2.THRESH_BINARY_INV)

    # find conected components
    connectivity = 4
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity, cv2.CV_32S)
    sizes = stats[1:, -1]
    nb_components = nb_components - 1
    min_size = 250  # threshhold value for objects in scene
    img2 = np.zeros((img.shape), np.uint8)
    for i in range(0, nb_components + 1):
        #print nb_components
        if nb_components > 5:
            break
        # use if sizes[i] >= min_size: to identify your objects
        color = np.random.randint(255, size=3)
        # draw the bounding rectangele around each object
        cv2.rectangle(img2, (stats[i][0], stats[i][1]), (stats[i][0] + stats[i][2], stats[i][1] + stats[i][3]),
                      (0, 255, 0), 2)
        img2[output == i + 1] = color
        #we are only interested in components not the background which starts at 0, 0 and is thus discounted
        if (stats[i][0] > 1):
            print("Component: %f, centroids:x,y => %f %f" % (i, centroids[i][0], centroids[i][1]))
            global centroids_str
            global stats_float
            global centroids_float
            stats_float = stats[i]
            centroids_float = Float32MultiArray(data=[centroids[i][0], centroids[i][1], -1, 1])
            centroids_str = str(centroids[i][0]) + " " + str(centroids[i][1])


        cv2.rectangle(img2, (stats[i][0], stats[i][1]), (stats[i][0] + stats[i][2], stats[i][1] + stats[i][3]),
                      (0, 255, 0), 2)
        img2[output == i + 1] = color

    # makeMatrix(obj)
    # save(obj)

    cv2.namedWindow('The BW image', cv2.WINDOW_NORMAL)
    cv2.imshow('The BW image', bw)
    cv2.namedWindow('The segmented image', cv2.WINDOW_NORMAL)
    cv2.imshow('The segmented image', img2)
    # cv2.imshow('The gray image', gray_img)
    cv2.waitKey(2)
    #return centroids


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    global centroids_str
    centroids_str = ""
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# def publish_location(data):
#     pub_cent = rospy.Publisher('marker/location', Float32MultiArray,  queue_size=10) #/object/centroid #string and not Float
#     pub_cent.publish(data)

def main():
    rospy.init_node('default_objrecog', anonymous=True)
    rate = rospy.Rate(10)
    mavros.set_namespace('/mavros')

    subscriber = rospy.Subscriber("/iris/camera/image_raw", Image, callback, queue_size=1)
    pub_cent = rospy.Publisher('/centroid/location', String,  queue_size=10) #/object/centroid #string and not Float
    pub_stat = rospy.Publisher('marker/location', Float32MultiArray,  queue_size=10)


    #print("test")

    while not rospy.is_shutdown():

        #rospy.loginfo(centroids_str)

        #pub_cent.publish(centroids_str)
        pub_stat.publish(centroids_float)
        rate.sleep()

        # k = cv2.waitKey(1) & 0xff
        # if k == 27: break

    return 0

if __name__ == '__main__':
    main()
    print('exit')