#!/usr/bin/env python
# Python libs
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import serial
import time
import string
from std_msgs.msg import Int32MultiArray

import rospy
import sys
commands = Int32MultiArray()

def publish():
    tx_pub = rospy.Publisher("/drone_landing/tx",Int32MultiArray, queue_size = 1)
    rate = rospy.Rate(10)
    message = Int32MultiArray(data = [1150, 1500, 1500, 1500])
    counter = 0
    while not rospy.is_shutdown():
        counter = counter +1
        print "Its something"
        message = Int32MultiArray(data = [1720, 1500, 1500, 1500])
        tx_pub.publish(message)
        rate.sleep()


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('pub_test', anonymous=False)
    publish()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image coord pub module"


if __name__ == '__main__':
    main(sys.argv)
