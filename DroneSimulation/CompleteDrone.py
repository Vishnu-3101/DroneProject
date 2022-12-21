import threading

from dronekit import connect, VehicleMode, LocationGlobalRelative

from pymavlink import mavutil
import time

import cv2 as cv
from cv2 import aruco
import numpy as np
import math

import RPi.GPIO as GPIO 

relay=26 #GPIO pin connected from relay to rpi
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay,GPIO.OUT)

#-- Connect to vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect

print("Conncetion to the vehicle on: " + connection_string)

vehicle = connect(connection_string, wait_ready=True)


marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

param_markers = aruco.DetectorParameters_create()

capture = cv.VideoCapture(1)

#(h, w) = frame.shape[:2]

(hf,wf) = (480,640)

aruco_package=0    #checks if marker is detected or not
aruco_dropzone=0
color=0
x_distance=0
y_distance=0
aruco_centers_dist=0
package1=0

def arm_and_takeoff(aTargetAltitude):
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)    
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print ("Reached target altitude")
            break
        time.sleep(1)


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)



def DisplayFrame():
    global aruco_package,aruco_dropzone,color,x_distance,y_distance,actual_centers_dist,angle,hf,wf
    while True:
        ret,frame = capture.read()
        cv.circle(frame, (wf//2, hf//2), 5, (0, 0, 255), -1)
        if not ret:
            break
        gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        marker_corners,marker_IDs,reject = aruco.detectMarkers(
            gray_frame,marker_dict,parameters=param_markers
        )
        if marker_corners and aruco!=3:
            for corners in marker_corners:
                corners=corners.reshape(4,2)
                aruco_center = ((corners[0][0] + corners[2][0])//2, (corners[0][1] + corners[2][1])//2)
                cv.circle(frame, (int(aruco_center[0]), int(aruco_center[1])), 5, (0, 255, 0),-1)
                cv.circle(frame, (int(corners[0][0]), int(corners[0][1])), 5, (0, 0, 255),-1)
                cv.circle(frame, (int(corners[1][0]), int(corners[1][1])), 5, (0, 255, 0),-1)
                cv.circle(frame, (int(corners[2][0]), int(corners[2][1])), 5, (255, 0, 0), -1)
                cv.circle(frame, (int(corners[3][0]), int(corners[3][1])), 5, (255, 255, 255), -1)
        cv.imshow("frame",frame)
        key = cv.waitKey(1)
        if key==ord("q") or aruco_package==2:
            break
    


def DetectAruco():
    global aruco_package,aruco_dropzone,x_distance,y_distance,actual_centers_dist,angle,hf,wf
    startTime=time.time()
    while True:
        ret,frame = capture.read()
        frame_centre = ((wf//2,hf//2))
        cv.circle(frame, (wf//2, hf//2), 5, (0, 0, 255), -1)
        if not ret:
            break
        gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        marker_corners,marker_IDs,reject = aruco.detectMarkers(
            gray_frame,marker_dict,parameters=param_markers
        )
        if marker_corners:
            for ids,corners in zip(marker_IDs,marker_corners):
                corners=corners.reshape(4,2)
                aruco_center = ((corners[0][0] + corners[2][0])//2, (corners[0][1] + corners[2][1])//2)
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
                if ids==0 and aruco_package==0:
                    aruco_package=1
                elif ids==1:
                    aruco_dropzone=1
            print("Aruco marker detected")
            break
        endTime=time.time()
        if(endTime-startTime > 20):
            print("Time limit Exceeded. No marker detected")
            break
        cv.imshow("frame",frame)
        key = cv.waitKey(1)
        if key==ord("q"):
            break


def detectColor(){
    global color,x_distance,y_distance,actual_centers_dist,angle,hf,wf
    startTime=time.time()
    while True:
        ret,frame=capture.read()
        frame_centre = ((wf//2,hf//2))
        if not ret:
            break
            #converting the frame to hsv format
        hsv_frame =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        #defining the hsv of the color to be detected
        #for purple color
        lower = np.array([129, 50, 70])
        upper = np.array([158, 255, 255])
        
        
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        #Making the background of detected color as black. If no color is detected whole frame is black
        result = cv2.bitwise_and(frame, frame, mask=mask)

        #finding the borders of the detected color
        contours,ret=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        for c in contours:
            area=cv2.contourArea(c)
            peri=cv2.arcLength(c,True)
            approx=cv2.approxPolyDP(c,peri*0.02,True)

            if len(approx) == 4 and area>300:

                cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)

                x,y,w,h=cv2.boundingRect(c)
                img_center=(x+(w/2),y+(h/2))
                cv2.circle(frame, (int(img_center[0]),int(img_center[1])), 5, (0, 0, 255),-1)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                centers_dist = math.sqrt(((img_center[0]- frame_centre[0]) ** 2) + ((img_center[1] - frame_centre[1]) ** 2))
                if w>0:
                    actual_centers_dist = 0.076* (centers_dist/w)  # 3 inches = 0.076 meters
                    x_distance = 0.0762*(abs(img_center[0]-frame_centre[0])/centers_dist)
                    y_distance = 0.0762*(abs(img_center[1]-frame_centre[1])/centers_dist)
                    color=1
                    
        if color==1:
            print("Color detected")
            break
        endTime=time.time()
        if(endTime-startTime > 20):
            print("Time limit Exceeded. No color detected")
            break
        cv.imshow("frame",frame)
        key = cv.waitKey(1)
        if key==ord("q"):
            break
}

def Calculate():
    print("Entered calculate function")
    global aruco_dropzone,aruco_dropzone,color,x_distance,y_distance,actual_centers_dist,angle
    if aruco_package==1 or aruco_dropzone==1:
        if aruco_package==1:
            print("Marker detected")
        if aruco_package==1:
            print("Dropzone detected")
        print(x_distance,",",y_distance)
        dur = 5
        print(dur)
        while x_distance>0.05 or y_distance>0.05:
            print("Moving towards the marker")
            send_global_velocity(x_distance,y_distance,0,dur)
            time.sleep(5)
            print("Moved in the direction of Aruco Marker till 5 seconds")
            print("Checking for the aruco marker again")
            DetectAruco()
            print(x_distance,",",y_distance)
            time.sleep(1)
        print("Reached the aruco marker")
        if aruco_package==1:
            print("Reducing the altitude to pick the package")
            send_global_velocity(0,0,1,2)
            time.sleep(10)
            onElectromagnet()
            print("Picked the package")
            aruco_package=2
        if aruco_dropzone==1:
            print("Reducing the altitude to drop the package")
            send_global_velocity(0,0,1,2)
            time.sleep(10)
            offElectromagnet()
            print("droped the package")
            aruco_dropzone=0

    if color==1:
        print("color detected")
        print(x_distance,",",y_distance)
        dur = 5
        print(dur)
        while x_distance>0.05 or y_distance>0.05:
            print("Moving towards the marker")
            send_global_velocity(x_distance,y_distance,0,dur)
            time.sleep(5)
            print("Moved in the direction of detected till 5 seconds")
            print("Checking for the color again")
            detectColor()
            print(x_distance,",",y_distance)
            time.sleep(1)
        print("Reached the color")
        print("Reducing the altitude to pick the package")
        send_global_velocity(0,0,1,2)
        time.sleep(10)
        onElectromagnet()
        print("Picked the package")
        color=2

def onElectromagnet():
    global relay
    GPIO.output(relay,GPIO.HIGH)

def offElectromagnet():
    global relay
    GPIO.output(relay,GPIO.LOW)


def Check():
    print("Entered check function")
    global aruco_package,aruco_dropzone,x_distance,y_distance,actual_centers_dist,angle,package1
    d=0
    y=1
    de=5
    send_global_velocity(0,y,0,de)
    print("Moved in y axis")
    if aruco_package==0 and package1==0:
        print("Checking for aruco marker package")
        DetectAruco()
    elif color==0 and package1==1:
        print("Checking for color package")
        detectColor()
    elif aruco_dropzone==0:
        print("Checking for dropzone package")
        DetectAruco() 
    y=-2
    while d<3:
        if aruco_package==1 or aruco_dropzone==1 or color==1:
            Calculate()
            break
        else:
            send_global_velocity(0,y, 0, de)
            print("Moved in y axis")
            if aruco_package==0 and package1==0:
                print("Checking for aruco marker package")
                DetectAruco()
            elif color==0 and package1==1:
                print("Checking for color package")
                detectColor()
            elif aruco_dropzone==0:
                print("Checking for dropzone package")
                DetectAruco()
            if aruco_package==1 or aruco_dropzone==1 or color==1:
                Calculate()
                break
            d=d+1
            send_global_velocity(1,0,0,de)
            print("Moved in x axis")
            if aruco_package==0 and package1==0:
                print("Checking for aruco marker package")
                DetectAruco()
            elif color==0 and package1==1:
                print("Checking for color package")
                detectColor()
            elif aruco_dropzone==0:
                print("Checking for dropzone package")
                DetectAruco()
            if(d==1):
                y=2
            else:
                y=-2
         
def move():
    Check()
    Check()
    Check()
    Check()



arm_and_takeoff(5)



t1=threading.Thread(target = DisplayFrame)
t2=threading.Thread(target = move)

t2.start()
t1.start()


t2.join()
t1.join()



print("Now lets go to home")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object
vehicle.close()

capture.release()
cv.destroyAllWindows()