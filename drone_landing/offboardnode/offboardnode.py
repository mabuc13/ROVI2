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
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

bridge = CvBridge()

#global marker_pos
marker_pos = Float32MultiArray()

#global pixel_scale
pixel_scale = 0

#global x_setpoint
x_setpoint = 0

#global y_setpoint
y_setpoint = 0

#global iterator
iterator = 0

#The wanted orientation of the marker
yaw_setpoint = math.pi*1.5
# = setpoint_msg.pose.orientation

#The first state, where the drone hovers high to find the markerlocator
controller_state = 1

#Error vector
error_vector = [10, 10]

#Yaw error
yaw_err = 2

#Global altitude
glob_alt = 23

#Global yaw setpoint
tot_yaw = 0


#Quality measure for hovering over the marker
quality_meas = 0
def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

current_pose = Vector3()
UAV_state = mavros_msgs.msg.State()

def callback(ros_data):
    img = bridge.imgmsg_to_cv2(ros_data,"bgr8")


    cv2.imshow('the raw image',img)
    #cv2.imshow('consist only of 0 and 7',obj)
    cv2.waitKey(2)
    #msg=bridge.cv2_to_imgmsg(obj, encoding="8UC1")
    #msg2=bridge.cv2_to_imgmsg(img,encoding="bgr8")
    #self.image_pub.publish(msg)
    #self.image_pub_display.publish(msg2)

def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def euler_to_quaternion(yaw):
    mavros.set_namespace('/mavros')
    setpoint_msg = mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
    )
    q = setpoint_msg.pose.orientation
    roll = 0
    pitch = 0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q.w = cy * cr * cp + sy * sr * sp
    q.x = cy * sr * cp - sy * cr * sp
    q.y = cy * cr * sp + sy * sr * cp
    q.z = sy * cr * cp - cy * sr * sp
    return q

def quaternion_to_euler_angle(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return Z

def _setpoint_position_callback(topic):
    pass

def _local_position_callback(topic):
    global pixel_scale
    pixel_scale = topic.pose.position.z / 800
    global quat
    quat = topic.pose.orientation
    #print "current yaw", quaternion_to_euler_angle(quat.x, quat.y, quat.z, quat.w)
    #print(topic.pose.position.z)
    pass

def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header = mavros.setpoint.Header(
        frame_id="att_pose",
        stamp=rospy.Time.now())

def marker_callback(recieved):
    dummy = recieved.data
    global marker_pos
    marker_pos.data = dummy

def update_setpoint():
    pass

def rotate_vector(vector, angle):
    rot = [[math.cos(angle),-math.sin(angle), 0],[math.sin(angle),math.cos(angle), 0],[0, 0, 1]]
    result = np.dot(rot, vector)
    return result

def marker_angle_to_radians(angle):
    if angle >= 0:
        #print("input, output", angle, angle)
        return angle
    else:
        result = math.pi + math.pi+angle
        #print("input, output", angle, result)
        return result

def hover(altitude):

    setpoint_msg = mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
    )
    global marker_pos
    global pixel_scale
    global x_setpoint
    global y_setpoint
    global iterator
    global quat
    global error_vector
    center = 400*pixel_scale
    if iterator > 100:
        if marker_pos.data[3] > 0.009:

            x_meter = center - marker_pos.data[1]*pixel_scale #- center
            y_meter =  center - marker_pos.data[0]*pixel_scale #- center
            current_yaw = quaternion_to_euler_angle(quat.x, quat.y, quat.z, quat.w)

            error_vec = [[x_meter],[y_meter],[0]]
            error_rotated = rotate_vector(error_vec, current_yaw)
            x_setpoint = x_setpoint + 0.8*error_rotated[0]
            y_setpoint = y_setpoint + 0.8*error_rotated[1]
            iterator = 0
            error_vector = error_rotated
            print("x and y of marker in meters", x_setpoint, y_setpoint)
            print("Errors / position offset", error_rotated[0], error_rotated[1])

            print("Marker quality", marker_pos.data[3])

    iterator = iterator + 1

    setpoint_msg.pose.position.y = y_setpoint
    setpoint_msg.pose.position.x = x_setpoint
    setpoint_msg.pose.position.z = altitude

    return setpoint_msg, error_vector

#def backup():
    ##lint:disable
    #center = 400*pixel_scale
        #if iterator > 100:
            #if marker_pos.data[3] > 0.02:
                #yaw = marker_angle_to_radians(marker_pos.data[2])
                #x_meter = center - marker_pos.data[1]*pixel_scale #- center
                #y_meter =  center - marker_pos.data[0]*pixel_scale #- center
                #current_yaw = quaternion_to_euler_angle(quat.x, quat.y, quat.z, quat.w)
                #yaw_error = yaw_setpoint - yaw
                #total_yaw = current_yaw+yaw_error*0.49
                ##print "yaw", total_yaw
                #setpoint_msg.pose.orientation = euler_to_quaternion(total_yaw)
                #error_vec = [[x_meter],[y_meter],[0]]
                #error_rotated = rotate_vector(error_vec, current_yaw)
                #x_setpoint = x_setpoint + 0.8*error_rotated[0]
                #y_setpoint = y_setpoint + 0.8*error_rotated[1]
                #iterator = 0
                #print("x and y of marker in meters", x_setpoint, y_setpoint)
                #print("Errors / position offset", error_rotated[0], error_rotated[1])
                ##print("yaw", yaw)
                #print("Marker quality", marker_pos.data[3])
        ##print("Height: %f" % setpoint_msg.pose.position.z) #
        ##print ("Iterator: ", iterator)
        ##print("Marker qual", marker_pos.data[3], marker_pos.data[2])
        #iterator = iterator + 1
        ##x_setpoint = 2
        ##y_setpoint = 2
        #setpoint_msg.pose.position.y = y_setpoint
        #setpoint_msg.pose.position.x = x_setpoint
        #setpoint_msg, hover_error = hover()
        #setpoint_local_pub.publish(setpoint_msg)
        #rate.sleep()
    ##lint:enable
def hover_n_yaw(altitude):
    setpoint_msg = mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
    )
    global marker_pos
    global pixel_scale
    global x_setpoint
    global y_setpoint
    global iterator
    global quat
    global error_vector
    global yaw_err
    global yaw_setpoint
    global tot_yaw
    center = 400*pixel_scale
    if iterator > 100:
        if marker_pos.data[3] > 0.009:

            x_meter = center - marker_pos.data[1]*pixel_scale #- center
            y_meter =  center - marker_pos.data[0]*pixel_scale #- center
            current_yaw = quaternion_to_euler_angle(quat.x, quat.y, quat.z, quat.w)

            error_vec = [[x_meter],[y_meter],[0]]
            error_rotated = rotate_vector(error_vec, current_yaw)
            x_setpoint = x_setpoint + 0.8*error_rotated[0]
            y_setpoint = y_setpoint + 0.8*error_rotated[1]

            yaw = marker_angle_to_radians(marker_pos.data[2])
            yaw_error = yaw_setpoint - yaw
            yaw_err = yaw_error
            total_yaw = current_yaw+yaw_error*0.49
            tot_yaw = total_yaw

            print "SHould have assigned yaw to the drone", total_yaw
            iterator = 0
            error_vector = error_rotated
            print("x and y of marker in meters", x_setpoint, y_setpoint)
            print("Errors / position offset", error_rotated[0], error_rotated[1])

            print("Marker quality", marker_pos.data[3])

    iterator = iterator + 1

    setpoint_msg.pose.position.y = y_setpoint
    setpoint_msg.pose.position.x = x_setpoint
    setpoint_msg.pose.position.z = altitude
    setpoint_msg.pose.orientation = euler_to_quaternion(tot_yaw)
    #setpoint_msg.pose.orientation = euler_to_quaternion(4)
    #setpoint_msg.pose.orientation = euler_to_quaternion(total_yaw)

    return setpoint_msg, error_vector, yaw_err

def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
                                 mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
         SP.PoseStamped, _local_position_callback)
    # /mavros/setpoint_raw/target_local
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
                                          mavros_msgs.msg.PositionTarget, _setpoint_position_callback)


    #Setting up image subscriber
    subscriber = rospy.Subscriber("/iris/camera/image_raw", Image, callback, queue_size = 1)
    #Setting up marker subscriber
    marker_sub = rospy.Subscriber("marker/location", Float32MultiArray, marker_callback, queue_size =1)
    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    setpoint_msg = mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
    )

    # wait for FCU connection
    while (not UAV_state.connected):
        rate.sleep()

    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 0

    mavros.command.arming(True)

    # send 100 setpoints before starting
    for i in range(0, 50):
        setpoint_local_pub.publish(setpoint_msg)
        rate.sleep()

    set_mode(0, 'OFFBOARD')

    last_request = rospy.Time.now()

    # enter the main loop
    while (True):
        # print "Entered whiled loop"
        if (UAV_state.mode != "OFFBOARD" and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            set_mode(0, 'OFFBOARD')
            print("enabling offboard mode")
            last_request = rospy.Time.now()
        else:
            if (not UAV_state.armed and
                    (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if (mavros.command.arming(True)):
                    print("Vehicle armed")
                last_request = rospy.Time.now()
        #Controller logic

        global glob_alt
        global marker_pos
        global pixel_scale
        global x_setpoint
        global y_setpoint
        global iterator
        global quat
        global controller_state
        global error_vector
        global quality_meas
        global yaw_err
        if controller_state == 1:
            setpoint_msg, error_vector = hover(glob_alt)
            if (abs(error_vector[0]) < 0.7 and abs(error_vector[1]) < 0.7):
                quality_meas = quality_meas + 1
            else:
                quality_meas = 0
            if quality_meas > 15:
                glob_alt = glob_alt -0.5
                quality_meas = 5
            if glob_alt < 10:
                if marker_pos.data[2] == -1:
                    controller_state = 4 #should loiter above object
                controller_state = 2

        if controller_state == 2:
            setpoint_msg, error_vector, yaw_err = hover_n_yaw(glob_alt)
            if (abs(error_vector[0])< 0.3 and abs(error_vector[1]) < 0.3 and abs(yaw_err)<0.2):
                quality_meas = quality_meas + 1
            else:
                quality_meas = 0
            if quality_meas > 15:
                glob_alt = glob_alt -0.1
                quality_meas = 5
            if glob_alt < 0.15:
                controller_state = 3
        if controller_state == 3:
            setpoint_msg.pose.position.z = 0
        #center = 400*pixel_scale
        if controller_state == 4:
            setpoint_msg, error_vector = hover(glob_alt)
            glob_alt = 9
        print "controller_state: %f. Aaltitude: %f" % (controller_state, glob_alt)
        #if iterator > 100:
        #    if marker_pos.data[3] > 0.02:
        #        yaw = marker_angle_to_radians(marker_pos.data[2])
        #        x_meter = center - marker_pos.data[1]*pixel_scale #- center
        #        y_meter =  center - marker_pos.data[0]*pixel_scale #- center
        #        current_yaw = quaternion_to_euler_angle(quat.x, quat.y, quat.z, quat.w)
        #        yaw_error = yaw_setpoint - yaw
        #        total_yaw = current_yaw+yaw_error*0.49
        #        #print "yaw", total_yaw
        #        setpoint_msg.pose.orientation = euler_to_quaternion(total_yaw)
        #        error_vec = [[x_meter],[y_meter],[0]]
        #        error_rotated = rotate_vector(error_vec, current_yaw)
        #        x_setpoint = x_setpoint + 0.8*error_rotated[0]
        #        y_setpoint = y_setpoint + 0.8*error_rotated[1]
        #        iterator = 0
        #        print("x and y of marker in meters", x_setpoint, y_setpoint)
        #        print("Errors / position offset", error_rotated[0], error_rotated[1])
                #print("yaw", yaw)
        #        print("Marker quality", marker_pos.data[3])
        #print("Height: %f" % setpoint_msg.pose.position.z) #
        #print ("Iterator: ", iterator)
        #print("Marker qual", marker_pos.data[3], marker_pos.data[2])
        #iterator = iterator + 1
        #x_setpoint = 2
        #y_setpoint = 2
        #setpoint_msg.pose.position.y = y_setpoint
        #setpoint_msg.pose.position.x = x_setpoint
        #setpoint_msg, hover_error = hover()
        setpoint_local_pub.publish(setpoint_msg)
        rate.sleep()
    return 0




if __name__ == '__main__':
    main()

