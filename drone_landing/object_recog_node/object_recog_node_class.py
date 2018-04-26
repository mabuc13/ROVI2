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
from std_msgs.msg import UInt32MultiArray, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

bridge = CvBridge()
centroids_str = ""
centroids_point = []

pub = None


class ObjectRecogNode(object):
    def __init__(self):
        rospy.init_node('object_recog')
        subscriber = rospy.Subscriber("/iris/camera/image_raw", Image, callback, queue_size=1)
        pub = rospy.Publisher('/object/centroid', Point, queue_size=10)

    def img_centroids(self, img):
        # print("enter callback")
        img = bridge.imgmsg_to_cv2(img, "bgr8")

        # cvt to grayscale and binary img
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, bw = cv2.threshold(gray_img, 128, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # find conected components
        connectivity = 4
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(bw, connectivity, cv2.CV_32S)
        sizes = stats[1:, -1]
        nb_components = nb_components - 1
        min_size = 250  # threshhold value for objects in scene
        img2 = np.zeros((img.shape), np.uint8)
        for i in range(0, nb_components + 1):
            # use if sizes[i] >= min_size: to identify your objects
            color = np.random.randint(255, size=3)
            # draw the bounding rectangele around each object
            cv2.rectangle(img2, (stats[i][0], stats[i][1]), (stats[i][0] + stats[i][2], stats[i][1] + stats[i][3]),
                          (0, 255, 0), 2)
            img2[output == i + 1] = color
            # we are only interested in components not the background which starts at 0, 0 and is thus discounted
            if (stats[i][0] > 1):
                return centroids
            cv2.rectangle(img2, (stats[i][0], stats[i][1]), (stats[i][0] + stats[i][2], stats[i][1] + stats[i][3]),
                          (0, 255, 0), 2)
            img2[output == i + 1] = color

        cv2.namedWindow('The segmented image', cv2.WINDOW_NORMAL)
        cv2.imshow('The segmented image', img2)
        # cv2.imshow('The gray image', gray_img)
        cv2.waitKey(2)
        # return centroids

    # my img conversion from ros to opencv
    def callback(self, img):
        centroids = self.img_centroids(self, img)
        print("Component: %f, centroids:x,y => %f %f" % (i, centroids[i][0], centroids[i][1]))
        global centroids_str
        global centroids_point
        centroids_point = centroids[i]
        centroids_str = str(centroids[i][0]) + " " + str(centroids[i][1])
        pub.publish(centroids)

    def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    def run(self):
        rospy.init_node('default_objrecog', anonymous=True)
        rate = rospy.Rate(10)
        mavros.set_namespace('/mavros')
        print("test")
        while not rospy.is_shutdown():
            # rospy.loginfo(centroids_point)
            # pub.publish(centroids_point)
            rate.sleep()
            # k = cv2.waitKey(1) & 0xff
            # if k == 27: break
        return 0


if __name__ == '__main__':
    node = ObjectRecogNode()
    node.run()
