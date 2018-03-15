# -*- coding: utf-8 -*-
import cv2
import sys

#(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')￼

if __name__ == '__main__' :

    # Set up tracker.
    # Instead of MIL, you can also use

    # Read video
    video = cv2.VideoCapture('nice_filename.mp4') #2016 06 23 1418 Krydset Søndre Boulevard Kløvermosevej Tietgens Alle.mp4

    # Exit if video not opened.
    if not video.isOpened():
        print ("Could not open video")
        sys.exit()

    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print ('Cannot read video file')
#        sys.exit()



    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box


        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
