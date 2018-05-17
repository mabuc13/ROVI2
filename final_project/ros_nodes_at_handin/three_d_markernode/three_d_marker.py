#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import ROS libraries
import rospy
import math
import cv2
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
import yaml
import signal
import sys
from pid_controller.pid import PID

#PRINT THROTTLE OG TUNE DEN

bridge = CvBridge()


class ThreeDMarker:

    def __init__(self):
        self.tvecs = np.ndarray(shape = (1, 3), dtype = float)
        self.rvecs = np.ndarray(shape = (1, 3), dtype = float)
        self.gray = None
        self.thr_cmd = 1700
        self.yaw_cmd = 1500
        self.targetZ = 0.9
        self.neutral_roll = 1500#1431
        self.neutral_pitch = 1500# 1495
        self.neutral_yaw = 1500
        self.neutral_thr = 1759
        self.roll_cmd = self.neutral_roll
        self.pitch_cmd = self.neutral_pitch
        self.roll_counter = 0
        self.pitch_counter = 0
        self.userNumber = 0
        self.pidHoverDown = PID(15, 1.5, 0)
        self.trpy_send = Int32MultiArray(data = [self.neutral_thr, self.neutral_roll, self.neutral_pitch, self.neutral_yaw])
        self.pidHoverUp = PID(12, 0.5, 0.001) #12, 0.5, 0.001 siegler nicholas metoden
        self.pidRoll = PID(6, 0.1, 0) # P=7 or 5
        self.pidPitch = PID(6, 0.1, 0) #P=10 or 5
        self.pidYaw = PID(2, 0.001, 0) # P=7 or 5
        self.pidBlindRoll = PID(5, 0.1, 0.1)
        self.pidBlindPitch = PID(5, 0.1, 0.1)
        self.old_tvecs = np.ndarray(shape = (1, 3), dtype = float)
        self.old_rvecs = np.ndarray(shape = (1, 3), dtype = float)
        self.aruco_rate = 0
        self.aruco_counter = 0
        self.video_init = False
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        with open("/home/mark/catkin_ws/src/three_d_markernode/calibration.yaml") as f:
            loadeddict = yaml.load(f)
        self.mtxloaded = loadeddict.get('camera_matrix')
        self.distloaded = loadeddict.get('dist_coeff')
        self.new_cap = False
        self.cap_rate = 0
        self.debug = True
        self.time = 0.0
        self.blindY = 0.0
        self.blindX = 0.0

    def regulate_z(self, z_error):

        self.thr_cmd -= self.pidHoverUp(z_error)
        if self.thr_cmd < 1150:
            self.thr_cmd = 1150
        if self.thr_cmd > 1900:
            self.thr_cmd = 1900
        # if (self.debug):
        #     print self.time, z_error, self.tvecs.item(2)
        self.neutral_thr = self.thr_cmd

    def regulate_roll(self, roll_error):
        self.roll_cmd -= self.pidRoll(roll_error)

        if self.roll_cmd < 1490:
            self.roll_cmd = 1490
        if self.roll_cmd > 1510:
            self.roll_cmd = 1510
        # if (self.debug):
        #     print self.time, roll_error, self.tvecs.item(0)

    def regulate_pitch(self, pitch_error):
        self.pitch_cmd -= self.pidRoll(pitch_error)

        if self.pitch_cmd < 1490:
            self.pitch_cmd = 1490
        if self.pitch_cmd > 1510:
            self.pitch_cmd = 1510
        # if (self.debug):
          # print self.time, z_error, self.tvecs.item(1)

    def regulate_yaw(self, yaw_error):

        if yaw_error < 0.3 or yaw_error > -0.3:
            yaw_error = 0
        self.yaw_cmd -= self.pidYaw(yaw_error)

    def control_loop(self):
        if (self.new_cap):
            self.new_cap = False

            targetYaw = 0
            curr_Yaw = self.rvecs.item(0)
            yaw_error = targetYaw - curr_Yaw

            # print "tvecs", self.tvecs
            # rotated = rot(self.tvecs)
            z = self.tvecs.item(2)
            difference = z - self.targetZ
            # print "Error yaw: ", errorYaw
            r = self.rvecs
            # print "r: ", r, "\n"
            t = np.array([self.tvecs.item(0), self.tvecs.item(1), self.tvecs.item(2)])
            rotZ = np.array([[math.cos(yaw_error), -math.sin(yaw_error), 0],
                             [math.sin(yaw_error), math.cos(yaw_error), 0],
                             [0, 0, 1]])
            # print "t: ", t, "\n"
            rot = cv2.Rodrigues(r)[0]
            # tt = cv2.transpose(t)
            tRotated = rotZ.dot(t)
            # print "T rotated: ", tRotated
            # print "rot : \n", rot, "\n"
            target_x = 0
            target_y = 0
            curr_x = tRotated[0]
            curr_y = tRotated[1]
            curr_x = self.tvecs.item(0) #this is for debug above is valid
            curr_y = self.tvecs.item(1) #this is for debug above is valid

            x_error = curr_x - target_x
            # x_error = - x_error
            # self.blindX = x_error
            y_error = curr_y - target_y
            y_error = y_error #this might work
            self.blindY = y_error
            print x_error, y_error, yaw_error


            z_error = z - self.targetZ
            self.regulate_z(z_error)

            self.pidBlindRoll(x_error)
            self.regulate_roll(x_error)

            self.pidBlindPitch(y_error)
            self.regulate_pitch(y_error)

            self.regulate_yaw(yaw_error)
        else:
            self.regulate_z(0)
            self.regulate_roll(0)
            self.regulate_pitch(0)
           # self.pitch_cmd = self.pidBlindPitch(self.blindY)
          #  self.roll_cmd = self.pidBlindRoll(self.blindX)
            self.thr_cmd = self.neutral_thr
            #self.roll_cmd = self.neutral_roll
            #self.pitch_cmd = self.neutral_pitch
            self.yaw_cmd = self.neutral_yaw
        self.trpy_send = Int32MultiArray(data=[self.thr_cmd, self.roll_cmd, self.pitch_cmd, self.yaw_cmd])



    def draw_marker(self):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        print(aruco_dict)
        # second parameter is id number
        # last parameter is total image size
        img = aruco.drawMarker(aruco_dict, 2, self.size)
        cv2.imwrite("test_marker.jpg", img)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.imshow('frame', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def videoCapture(self, cap):
        ret, frame = cap.read()

        # Our operations on the frame come here
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray = frame
        # rejected_img_points_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(self.gray, self.dictionary, parameters=self.parameters)
        res = aruco.detectMarkers(self.gray, self.dictionary)
        #   print(res[0],res[1],len(res[2]))

        if len(res[0]) > 0:

            self.new_cap = True
            self.time += 1
            #cv2.aruco.drawDetectedMarkers(self.gray, corners, ids)
            mtx_np = np.asarray(self.mtxloaded)
            dist_np = np.asarray(self.distloaded)
            # rvecs_np = np.asarray(rvecsloaded)
            # tvecs_np = np.asarray(tvecsloaded)

            #  rvec is rotation matrix, tvec is translation matrix
            self.rvecs, self.tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_np, dist_np)  # , rvecs_np, tvecs_np)
            cv2.aruco.drawAxis(self.gray, mtx_np, dist_np, self.rvecs, self.tvecs, 0.1)
            #print self.rvecs #'r+t' + str(self.rvecs) + str(self.tvecs)


    def correct_drone(self):
        #get info from cam
        print heej


    def euler_to_quaternion(self, yaw, roll, pitch):
        mavros.set_namespace('/mavros')
        setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
        )
        q = setpoint_msg.pose.orientation
        # roll = 0
        # pitch = 0
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

    def quaternion_to_euler_angle(self, x, y, z, w):
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


    def calib_with_chessboard(self, cap):
        pc_cam = 0
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6 * 7, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        found = 0
        while (found < 20):  # Here, 10 can be changed to whatever number you like to choose
            ret, img = cap.read()  # Capture frame-by-frame
            self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(self.gray, (7, 6), None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.
                corners2 = cv2.cornerSubPix(self.gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)
                found += 1

            cv2.imshow('img', img)
            cv2.waitKey(10)

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, self.gray.shape[::-1], None, None)

        # It's very important to transform the matrix to list.

        data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist(),
                'rvecs_coeff': np.asarray(rvecs).tolist(), 'tvecs_coeff': np.asarray(tvecs).tolist()}

        with open("/home/mark/catkin_ws/src/three_d_markernode/calibration.yaml", "w") as f:
            yaml.dump(data, f)

    def testDevice(self, source):
        cap = cv2.VideoCapture(source)
        if cap is None or not cap.isOpened():
            print('Warning: unable to open video source: ', source)
            return 0
        return 1

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def mat2euler(self, R):

        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

   # def pos_error(self):
        #message = Int32MultiArray(data = [1150, 1500, 1500, 1500])
    #    Int = 0

       #
        #
       #  #eulerAngles = transform3d.self.mat2euler(rot)
       #  #print "eulerAngles: ", eulerAngles
       #  #tilt = eulerAngles[1]
       #  #rot = eulerAngles[0]
       #  #print "roll: ", eulerAngles[0], "pitch", eulerAngles[1], "Yaw", eulerAngles[2]
       #
       #  targetYaw = 0
       #  curr_Yaw = self.rvecs.item(0)
       #  errorYaw = targetYaw - curr_Yaw
       #
       # # print "tvecs", self.tvecs
       # # rotated = rot(self.tvecs)
       #  z = self.tvecs.item(2)
       #  difference = z-self.targetZ
       #  #print "Error yaw: ", errorYaw
       #  r = self.rvecs
       #  #print "r: ", r, "\n"
       #  t = np.array([self.tvecs.item(0), self.tvecs.item(1), self.tvecs.item(2)])
       #  rotZ = np.array([[math.cos(errorYaw), -math.sin(errorYaw), 0],
       #                   [math.sin(errorYaw), math.cos(errorYaw), 0],
       #                   [0, 0, 1]])
       #  rotY = np.array([[math.cos(self.rvecs.item(1)), 0, math.sin(self.rvecs.item(1))], [0, 1, 0],
       #                   [-math.sin(self.rvecs.item(1)), 0, math.cos(self.rvecs.item(1))]])
       #  #print "t: ", t, "\n"
       #  rot = cv2.Rodrigues(r)[0]
       #  # tt = cv2.transpose(t)
       #  tRotated = rotZ.dot(t)
       #  #print "T rotated: ", tRotated
       #  #print "rot : \n", rot, "\n"
       #  target_x = 0
       #  target_y = 0
       #  curr_x = tRotated[0]
       #  curr_y = tRotated[1]
       #  x_error = curr_x - target_x
       #  y_error = curr_y - target_y
       #  if z != 0.0:
       #      #Height correction
       #      if z > self.targetZ-0.05 and z < self.targetZ+0.05:
       #          self.thr = self.hover_thr
       #      else: #When the drone is lower than the target altitude the difference will be negative thus minus minus is positive and vice versa
       #          self.thr = self.thr - (difference*7)
       #      if self.thr > 1900:
       #          self.thr = 1900
       #      if self.thr < 1150:
       #          self.thr = 1150
       #
       #      #Yaw correction
       #      if errorYaw > targetYaw - 0.3 and errorYaw < targetYaw + 0.3:
       #          self.yawCmd = 1500
       #      else:  # When the drone is lower than the target altitude the difference will be negative thus minus minus is positive and vice versa
       #          self.yawCmd = self.yawCmd + (errorYaw * 0.5)
       #      if self.yawCmd > 1550:
       #          self.yawCmd = 1550
       #      if self.yawCmd < 1450:
       #          self.yawCmd = 1450
       #
       #      #Pos correction
       #      #print "x_err: ", x_error, " y_error: ", y_error
       #      # if self.roll_counter > 5:
       #     # if x_error > target_x -0.05 and x_error < target_x + 0.05:
       #          #self.roll_cmd = self.neutral_roll
       #      #else:
       #      self.roll_cmd -= int(self.pidRoll(x_error))
       #      #print "PID value to plus on roll: ",  self.pidRoll(x_error)
       #          #print "x_error: ", x_error
       #          #if x_error <= 0.0:
       #          #    print "going right with, ", 75*x_error
       #          #    self.roll_cmd = self.neutral_roll + 75*x_error
       #          #else:
       #          #    print "going left with, ", 75*x_error
       #          #    self.roll_cmd = self.neutral_roll + 75*x_error
       #          #self.roll_counter = 0
       #      # if self.roll_counter >= 1:
       #      #     self.roll_cmd = self.neutral_roll
       #      # self.roll_counter += 1
       #
       #      # if self.pitch_counter > 5:
       #      self.pitch_cmd -= int(self.pidPitch(y_error))
       #     # print "PID value to plus on pitch: ", self.pidRoll(y_error)
       #      print self.roll_cmd
       #      if self.roll_cmd > self.neutral_roll:
       #          print "Going right"
       #      else:
       #          print "Going left"
       #
       #      print self.pitch_cmd
       #      if self.pitch_cmd > self.neutral_pitch:
       #          print "Going forward"
       #      else:
       #          print "Going backwards\n"
       #
       #  limit = 40
       #  if self.pitch_cmd > self.neutral_pitch + limit:
       #      self.pitch_cmd = self.neutral_pitch + limit
       #  if self.pitch_cmd < self.neutral_pitch - limit:
       #      self.pitch_cmd = self.neutral_pitch - limit
       #
       #  if self.roll_cmd > self.neutral_roll + limit:
       #      self.roll_cmd = self.neutral_roll + limit
       #  if self.roll_cmd < self.neutral_roll - limit:
       #      self.roll_cmd = self.neutral_roll - limit
       #      #if y_error > target_y - 0.05 and y_error < target_y + 0.05:
       #      #    self.pitch_cmd = self.neutral_pitch
       #      #else:
       #        #  self.p
       #        #  print "y_error: ", y_error
       #        #  if y_error <= 0.0:
       #        #      print "FORWARDS we march with: " ,  75*y_error
       #        #      self.pitch_cmd = self.neutral_pitch + 75*y_error
       #        #  else:
       #        #      print "REATREAT with ",  75*y_error
       #        #      self.pitch_cmd = self.neutral_pitch + 75*y_error
       #        #  self.pitch_counter = 0
       #      # if self.pitch_counter == 1:
       #      #     self.pitch_cmd = self.neutral_pitch
       #      # self.pitch_counter += 1
       #
       #
       #      #print difference*50,
       #      #print "The orientation", self.rvecs
       #      #print "Rotated: ", rotated
       #  self.trpy_send = Int32MultiArray(data = [1760, self.roll_cmd, self.pitch_cmd, self.yawCmd])
       #      #publish
       #  #z = self.tvecs
       #  #print z
       # # print self.thr
       #  #print "The orientation", self.rvecs

    def trim(self):
        # Ask for the number and store it in userNumber
        self.userNumber = raw_input('Give me an integer number: ')

        try:
            # Try to convert the user input to an integer
            #userNumber = int(userNumber)
            self.userNumber = int(self.userNumber)
        # Catch the exception if the input was not a number
        except ValueError:
            self.userNumber = 0
        else:
            # Get the square of the number
            # userNumber**2 is the same as saying pow(userNumber, 2)
            self.userNumber = self.userNumber

        # Print square of given number
        print 'Integer to be added to roll: ' + str(self.userNumber)
        self.neutral_thr += self.userNumber
        self.trpy_send = Int32MultiArray(data = [self.neutral_thr, self.neutral_roll, self.neutral_pitch, self.neutral_yaw])

    def yaw_error(self):
        print self.rvecs

    def main(self):
        #print "init main"
        rospy.init_node('three_d_marker', anonymous=True)
        rate = rospy.Rate(100)

        pub = rospy.Publisher('/drone_landing/tx', Int32MultiArray,  queue_size=1)

        if self.testDevice(1) == 0:
            cap = cv2.VideoCapture(0)
        else:
            cap = cv2.VideoCapture(1)

        i = 0
        while not rospy.is_shutdown():
            self.videoCapture(cap)
            self.control_loop()
            pub.publish(self.trpy_send)
            cv2.imshow('frame', self.gray)

            #self.calib_with_chessboard(cap)

            #self.trim()
            if(i > 50):
               # print "running!!!"
                i = 0
            else:
                i += 1
            rate.sleep()
            k = cv2.waitKeyEx(1) & 0xff
            if k == 27:  # esc
                break
        return 0


def signal_handler(signal, frame):
    #print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    #runObject = ThreeDMarker(700)
    #runObject.calibrate_my_cam(get_calibration_images(calib_img_paths))
    cal = ThreeDMarker()
    cal.main()