import cv2 as cv
from cv2 import aruco
import numpy as np
import math

gps_centre = (12.841299, 80.153895)
vit_centre=  (12.840850, 80.153418)
vit_corner = (12.841299, 80.153895)

def haversine(lat1, lon1, lat2, lon2):
     
    # distance between latitudes
    # and longitudes
    dLat = (lat2 - lat1) * math.pi / 180.0
    dLon = (lon2 - lon1) * math.pi / 180.0
 
    # convert to radians
    lat1 = (lat1) * math.pi / 180.0
    lat2 = (lat2) * math.pi / 180.0
 
    # apply formulae
    a = (pow(math.sin(dLat / 2), 2) +
         pow(math.sin(dLon / 2), 2) *
             math.cos(lat1) * math.cos(lat2))
    rad = 6371
    c = 2 * math.asin(math.sqrt(a))
    return rad * c


marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

param_markers = aruco.DetectorParameters_create()


capture = cv.VideoCapture(0)

while True:
    ret,frame = capture.read()
    (h, w) = frame.shape[:2]
    
    frame_centre = ((w//2,h//2))
    cv.circle(frame, (w//2, h//2), 5, (0, 0, 255), -1)
    if not ret:
        break
    gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    marker_corners,marker_IDs,reject = aruco.detectMarkers(
        gray_frame,marker_dict,parameters=param_markers
    )
    if marker_corners:
        for corners in marker_corners:
            corners=corners.reshape(4,2)
            aruco_center = ((corners[0][0] + corners[2][0])//2, (corners[0][1] + corners[2][1])//2)
            cv.polylines(
                frame,[corners.astype(np.int32)],True,(0,255,0),4,cv.LINE_AA
            )
            cv.circle(frame, (int(aruco_center[0]), int(aruco_center[1])), 5, (0, 255, 0),-1)
            cv.circle(frame, (int(corners[0][0]), int(corners[0][1])), 5, (0, 0, 255),-1)
            cv.circle(frame, (int(corners[1][0]), int(corners[1][1])), 5, (0, 255, 0),-1)
            cv.circle(frame, (int(corners[2][0]), int(corners[2][1])), 5, (255, 0, 0), -1)
            cv.circle(frame, (int(corners[3][0]), int(corners[3][1])), 5, (255, 255, 255), -1)
            old_pixel_dist = math.sqrt(((corners[0][0] - corners[1][0]) ** 2 + (corners[0][1] - corners[1][1]) ** 2))
            centers_dist = math.sqrt(((aruco_center[0]- frame_centre[0]) ** 2) + ((aruco_center[1] - frame_centre[1]) ** 2))
            actual_centers_dist = 0.0762* (centers_dist/old_pixel_dist)  # 3 inches = 0.076 meters
            x_distance = 0.0762*(abs(aruco_center[0]-frame_centre[0])/old_pixel_dist)
            y_distance = 0.0762*(abs(aruco_center[1]-frame_centre[1])/old_pixel_dist)
            dist = math.sqrt(x_distance**2 + y_distance **2)
        print("The actual centers distance is: ",actual_centers_dist)
        print("The distance calculated indirectly is: ",dist)

        if actual_centers_dist==0:
            aruco_gps=gps_center
        cos_angle = ((aruco_center[1]-frame_centre[1])/actual_centers_dist)
        sin_angle = ((aruco_center[0]-frame_centre[0])/actual_centers_dist)
        
        dist = (haversine(vit_centre[0],vit_centre[1],vit_corner[0],vit_corner[1]))
        print(dist)
        lat_per_meter = (abs(vit_centre[0]-vit_corner[0]))/(dist*1000)
        print("latitude change per meter: ",lat_per_meter)
        lon_per_meter = (abs(vit_centre[1]-vit_corner[1]))/(dist*1000)
        print("longitude change per meter: ",lon_per_meter)
        aruco_gps = (gps_centre[0]+(lat_per_meter*actual_centers_dist)*cos_angle , gps_centre[1]+(lon_per_meter*actual_centers_dist)*sin_angle)
        print(aruco_gps[0],",",aruco_gps[1])
    cv.imshow("frame",frame)
    key = cv.waitKey(1)
    if key==ord("q"):
        break
capture.release()
cv.destroyAllWindows()