import cv2 as cv
from cv2 import aruco
import numpy as np


dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)


capture = cv.VideoCapture(0)

while True:
    ret,frame = capture.read()
    (h, w) = frame.shape[:2]
    frame_centre = ((w//2,h//2))
    cv.circle(frame, (w//2, h//2), 5, (0, 0, 255), -1)
    if not ret:
        break
    gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, rejectedCandidates = detector.detectMarkers(frame)
    if marker_corners:
        for corners in marker_corners:
            
            cv.polylines(
                frame,[corners.astype(np.int32)],True,(0,255,0),4,cv.LINE_AA
            )
            corners=corners.reshape(4,2)
            aruco_center = ((corners[0][0] + corners[2][0])//2, (corners[0][1] + corners[2][1])//2)
            
    cv.imshow("frame",frame)
    key = cv.waitKey(1)
    if key==ord("q"):
        break
capture.release()
cv.destroyAllWindows()