import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt



#(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')￼
def track_car_video(video, centroids):
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[2]

    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()

    # Read video
#    video = cv2.VideoCapture("2016 06 23 1418 Krydset Søndre Boulevard Kløvermosevej Tietgens Alle.mp4")

    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()

    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print('Cannot read video file')
        sys.exit()

    # Define an initial bounding box
    one_bbox = centroids[1]
    bbox = (one_bbox[0]-15 ,one_bbox[1]-15, 30, 30 )
    #bbox = (287, 23, 86, 320)

    # Uncomment the line below to select a different bounding box
    #bbox = cv2.selectROI(frame, False)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)

    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break



def track_car_frame(frame):
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[4]

    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()

    # Define an initial bounding box
    bbox = (287, 23, 86, 320)

    # Uncomment the line below to select a different bounding box
    bbox = cv2.selectROI(frame, False)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)

    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display tracker type on frame
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    # Display result
    cv2.imshow("Tracking", frame)
    cv2.waitKey(1)
    return frame


def blob_detection(window_name, detect_image, display_image, connectivity=4):

    ret, thresh = cv2.threshold(detect_image, 0, 255, cv2.THRESH_BINARY)

    output = cv2.connectedComponentsWithStats(detect_image, connectivity, cv2.CV_32S)

    # Get the results
    # The first cell is the number of labels
    num_labels = output[0]
    # The second cell is the label matrix
    labels = output[1]
    # The third cell is the stat matrix
    stats = output[2]
    # The fourth cell is the centroid matrix
    centroids = output[3]

    display_image = detect_image
    # Draw circles where blobs where found
    for i, val in enumerate(centroids):
        if i == 0:
            continue
        cv2.circle(display_image, (int(centroids[i][0]), int(centroids[i][1])), 10, (50, 255, 0), 3, 8, 0)
    return centroids



def background_subtract(video):
    fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = False, history = 40, varThreshold = 35)
    while(1):
        ret, frame = video.read()

        frame = transform_perspective(frame)

        blur_size = 11
        cv2.blur(frame, (blur_size, blur_size), frame)

        fgmask = fgbg.apply(frame)

        elip_val = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

        fgmask = cv2.erode(fgmask, elip_val)
        fgmask = cv2.dilate(fgmask, elip_val, iterations = 3)

        centroids = blob_detection('test', fgmask, fgmask)

        track_car_video(video, centroids)

        cv2.imshow('frame1', fgmask)

        #cv2.imshow('frame2', frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
    video.release()
    cv2.destroyAllWindows()



def transform_perspective(frame):
     #find pixel pos in frame
    #plt.figure(figsize=(8, 6))
    #plt.imshow(frame)
    #plt.show()

    points = np.array([[251, 39],[1110, 62],[1141, 685], [178, 659]])

    lenght_in_n2w = (np.sqrt(1110**2 + 62**2) - np.sqrt(251**2 + 39**2))/84.789

    #print(lenght_in_n2w) To get the constant to transform from meter to pixel

    north_to_west = 84.789*lenght_in_n2w # meter, x top direction
    south_to_east = 84.594*lenght_in_n2w # meter, x bottom direction
    north_to_east = 114.315*lenght_in_n2w # meter, y right
    south_to_west = 90.462*lenght_in_n2w # meter, y left
     #go from north_west clock dir end in south_west
    new_corner_positions = np.array([[0, 0], [north_to_west, 0], [south_to_east, north_to_east], [0, south_to_west]])
    h, status =cv2.findHomography(points, new_corner_positions)

    size = (int(max(north_to_west,south_to_east)), int(max(north_to_east, south_to_west)))
    #print(h)
    im_out = cv2.warpPerspective(frame, h, size)
    #cv2.namedWindow('frame1', cv2.WINDOW_NORMAL)
    #cv2.imshow('frame1', frame)

    #cv2.namedWindow('frame2', cv2.WINDOW_NORMAL)
    #cv2.imshow('frame2', im_out)
    #cv2.waitKey(0)

    #plt.imshow(im_out)
    #plt.show()

        #1292, 380
        #2708, 1012
        #2732, 2188
        #1312, 3024
    #1328, 2884
    #1232, 308
    #2708, 988
    #2724, 2180
    return im_out



if __name__ == '__main__' :
    video = cv2.VideoCapture \
        ('2016 06 23 1418 Krydset Søndre Boulevard Kløvermosevej Tietgens Alle.mp4')

    if not video.isOpened():
        print("Could not open video")
        sys.exit()

## Read first frame.
    ok, frame = video.read()
    if not ok:
        print('Cannot read video file')
        sys.exit()

    #do homography transform
    #frame = transform_perspective(frame)

    #here the background is subtractet
    background_subtract(video)

    #cv2.imshow("test", frame)
    #cv2.waitKey(0)

    print('Program EXIT!')