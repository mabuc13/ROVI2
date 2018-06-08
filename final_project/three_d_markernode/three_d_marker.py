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


trpy_send = Int32MultiArray()
bridge = CvBridge()


class ThreeDMarker:

    def __init__(self):
        self.tvecs = 0
        self.rvecs = 0
        self.gray = None

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
        #dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        #dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        trpy_send = Int32MultiArray()
#        while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        parameters = aruco.DetectorParameters_create()

        # Our operations on the frame come here
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray = frame
        # rejected_img_points_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(self.gray, dictionary, parameters=parameters)
        res = aruco.detectMarkers(self.gray, dictionary)
        #   print(res[0],res[1],len(res[2]))

        if len(res[0]) > 0:
            with open('/home/mathias/catkin_ws_rmurv2/src/three_d_markernode/calibration.yaml') as f:
                loadeddict = yaml.load(f)

            mtxloaded = loadeddict.get('camera_matrix')
            distloaded = loadeddict.get('dist_coeff')
            rvecsloaded = loadeddict.get('rvecs_coeff')
            tvecsloaded = loadeddict.get('tvecs_coeff')

            cv2.aruco.drawDetectedMarkers(self.gray, corners, ids)
            mtx_np = np.asarray(mtxloaded)
            dist_np = np.asarray(distloaded)
            # rvecs_np = np.asarray(rvecsloaded)
            # tvecs_np = np.asarray(tvecsloaded)

            #  rvec is rotation matrix, tvec is translation matrix
            self.rvecs, self.tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx_np, dist_np)  # , rvecs_np, tvecs_np)
            cv2.aruco.drawAxis(self.gray, mtx_np, dist_np, self.rvecs, self.tvecs, 0.1)
            #print self.rvecs #'r+t' + str(self.rvecs) + str(self.tvecs)

            #global trpy_send
            trpy_send = Int32MultiArray(data=[1000, 2000, 3000, 4000])
        #     # Display the resulting frame
        #     # cv2.imshow('rejected', rejected_img_points_frame)
        # cv2.imshow('frame', self.gray)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        return trpy_send

        # When everything done, release the capture
        #cap.release()
        #cv2.destroyAllWindows()

    def correct_drone(self):
        #get info from cam
        global trpy_send
        trpy_send = Int32MultiArray(data=[1000, 2000, 3000, 4000])
        return trpy_send

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

        with open("/home/mathias/catkin_ws_rmurv2/src/three_d_markernode/calibration.yaml", "w") as f:
            yaml.dump(data, f)

    def testDevice(self, source):
        cap = cv2.VideoCapture(source)
        if cap is None or not cap.isOpened():
            print('Warning: unable to open video source: ', source)
            return 0
        return 1

    def pos_error(self):
        print self.tvecs

    def yaw_error(self):
        print self.rvecs

    def main(self):
        print "init main"
        rospy.init_node('three_d_marker', anonymous=True)
        rate = rospy.Rate(20)

        pub = rospy.Publisher('/drone_landing/tx', Int32MultiArray,  queue_size=1)

        if self.testDevice(1) == 0:
            cap = cv2.VideoCapture(1)
        else:
            cap = cv2.VideoCapture(0)
        # self.calib_with_chessboard()
        # self.draw_marker()

        # while(True):
        #     ret, frame = cap.read()
        #     cv2.imshow("frame", frame)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        # cap.release()
        # cv2.destroyAllWindows()

        i = 0
        while not rospy.is_shutdown():
            # trpy = self.correct_drone()
            trpy_send = self.videoCapture(cap)
            self.yaw_error()
            pub.publish(trpy_send)
            #self.calib_with_chessboard(cap)

            if(i > 50):
                print "running!!!"
                i = 0
            else:
                i += 1
            rate.sleep()
            cv2.imshow('frame', self.gray)
            k = cv2.waitKeyEx(1) & 0xff
            if k == 27:  # esc
                break
        return 0


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    #runObject = ThreeDMarker(700)
    #runObject.calibrate_my_cam(get_calibration_images(calib_img_paths))
    cal = ThreeDMarker()
    cal.main()