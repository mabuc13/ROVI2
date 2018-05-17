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

#def test_device():
 #   dev = '/dev/ttyUSB0'p
  #  if disk_exists(dev):
  #      return dev
  #  dev = '/dev/ttyUSB1'
  #  if disk_exists(dev):
  #      return dev
  #  dev = '/dev/ttyUSB2'
  #  if disk_exists(dev):
  #      return dev


#def disk_exists(path):
#   try:
#        return stat.S_ISBLK(os.stat(path).st_mode)
#    except:
#        return False
class pythonTX:

    def __init__(self):
        self.ser =  serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
        time.sleep(2)
        #self.ser.open()

    def txCallback(self, commands):
        thr = commands.data[0]
        roll = commands.data[1]
        pitch = commands.data[2]
        yaw = commands.data[3]
        print "Entered callback"
        self.ser.flush()
        string = ""
        string = string + str(thr)
        string = string + str(roll)
        string = string + str(pitch)
        string = string + str(yaw)
        string = string + "\n"
        #string = "1700150015001500"
        print string
        self.ser.write(string)



    def main(self):
        rospy.init_node('serial_to_tx', anonymous=False)
        rospy.Rate(10)
        subscriber = rospy.Subscriber("/drone_landing/tx", Int32MultiArray, self.txCallback, queue_size = 1)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down serial_to_tx node"


#
#if __name__ == '__main__':
#    main()
#
#def main(args):
#    '''Initializes and cleanup ros node'''
#    image_feature()
#    rospy.init_node('feature_detect', anonymous=False)
#    try:
#        rospy.spin()
#    except KeyboardInterrupt:
#        print "Shutting down ROS Image feature detector module"
#    cv2.destroyAllWindows()

if __name__ == '__main__':
    mark = pythonTX()
    mark.main()
