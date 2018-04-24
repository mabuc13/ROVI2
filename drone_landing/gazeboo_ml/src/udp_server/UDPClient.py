#!/usr/bin/env python
#/****************************************************************************
# MarkerLocator UDP client example
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
 - https://wiki.python.org/moin/UdpCommunication

 Author: Kristian Husum Terkildsen, krter12@student.sdu.dk/kristian.h.terkildsen@gmail.com
'''

import socket
import struct

UDP_NAME = 'robolab01.projectnet.wlan.sdu.dk'
UDP_NAME = 'localhost'
UDP_NAME = 'fmcomp00.projectnet.wlan.sdu.dk'
UDP_IP = socket.gethostbyname_ex(UDP_NAME)
UDP_IP = UDP_IP[2][0]
UDP_PORT = 50006

print UDP_NAME
print UDP_IP
print UDP_PORT

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) #1024 is buffer size 24?
    print "Received data:", data

