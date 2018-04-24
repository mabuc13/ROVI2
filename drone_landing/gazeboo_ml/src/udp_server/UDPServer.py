#!/usr/bin/env python
#/****************************************************************************
# MarkerLocator ROS based UDP server
# Copyright (c) 2016, Kristian Husum Terkildsen,
# krter12@student.sdu.dk kristian.h.terkildsen@gmail.com
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
'''
Using the following sources:
 - http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
 - https://wiki.python.org/moin/UdpCommunication 

 Author: Kristian Husum Terkildsen, krter12@student.sdu.dk/kristian.h.terkildsen@gmail.com

 2016-10-27 Kjeld Jensen <kjen@mmmi.sdu.dk> moved configuration to the ROS parameter server.
'''

import rospy
from markerlocator.msg import markerpose

import socket
import struct

UDP_IP = ''
UDP_PORT = ''


def callback(data):
    secs = rospy.get_rostime().to_sec()
    
    #rospy.loginfo('%d %d %d %f %f %f' % (data.order, data.x, data.y, data.theta, data.quality, secs))

    dataString = str(data.order) + ", " + str(data.x) + ", " + str(data.y) + ", " + str(data.theta) + ", " + str(data.quality) + ", " + str(secs)

    '''
    print "UDP target IP:", UDP_IP
    print "UDP target port", UDP_PORT
    print "order: %d, x: %d, y: %d, theta: %f, quality: %f, secs: %f" % (data.order, data.x, data.y, data.theta, data.quality, secs)
    '''

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(dataString, (UDP_IP, UDP_PORT))

def listener():
    global UDP_IP
    global UDP_PORT

    rospy.init_node('listener', anonymous=True)

    topic = rospy.get_param("~markerpose_sub", '/markerlocator/markerpose') 
    UDP_IP = rospy.get_param("~udp_address", "127.0.0.1") 
    UDP_PORT = rospy.get_param("~udp_port", 5005) 

    rospy.Subscriber(topic, markerpose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

