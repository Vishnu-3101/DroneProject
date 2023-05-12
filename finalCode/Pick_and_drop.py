import threading

import cv2 as cv
from cv2 import aruco
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect

print("Conncetion to the vehicle on: " + connection_string)

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc


marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

param_markers = aruco.DetectorParameters_create()

print("Detecting cam")
capture = cv.VideoCapture(1)
print("Detected cam")

# width= int(capture.get(cv.CAP_PROP_FRAME_WIDTH))
# height= int(capture.get(cv.CAP_PROP_FRAME_HEIGHT))

# writer= cv.VideoWriter('rpivideo.mp4', cv.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

#####################################################
#    to check if aruco marker is detected or not    #

Aruco = False
isarucoDetected = False
dropZoneAruco = False
startAruco = False

angle_relative_to_start = 0
#####################################################



##########################################################################
#            continuous display of frame                                 #
##########################################################################

def DisplayFrame():
    global distance,isarucoDetected,dir,startAruco,dropZoneAruco,Aruco,Angle
    # starttime=time.time()
    box_id = -1
    print("Checking for aruco marker")
    while True:
        ret,frame = capture.read()
        (hf,wf) = (480,640)
        frame_centre = ((wf//2,hf//2))
        cv.circle(frame, (wf//2, hf//2), 5, (0, 0, 255), -1)
        if not ret:
            break
        gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        marker_corners,marker_IDs,reject = aruco.detectMarkers(
            gray_frame,marker_dict,parameters=param_markers
        )
        if marker_corners:
            for corners,ids in zip(marker_corners,marker_IDs):   
                if ids!=0 and ids!=1:
                    if Aruco:
                        box_id = ids
                        Aruco = False
                        isarucoDetected = True

                elif ids==0:
                    box_id = ids
                    startAruco = True
                    # isarucoDetected = True

                elif ids==1:
                    box_id = ids
                    dropZoneAruco = True
                    # isarucoDetected = True

                if box_id==ids:     

                    cv.polylines(frame, [corners.astype(np.int32)], True, (0, 155, 0), 3, cv.LINE_AA)

                    corners=corners.reshape(4,2)
                    aruco_center = ((corners[0][0] + corners[2][0])//2, (corners[0][1] + corners[2][1])//2)
                    old_pixel_dist = math.sqrt(((corners[0][0] - corners[1][0]) ** 2 + (corners[0][1] - corners[1][1]) ** 2))
                    centers_dist = math.sqrt(((aruco_center[0]- frame_centre[0]) ** 2) + ((aruco_center[1] - frame_centre[1]) ** 2))
                    actual_centers_dist = 7.62* (centers_dist/old_pixel_dist)  # 3 inches = 0.076 meters = 7.6 cm

                    cv.line(frame,(int(frame_centre[0]),int(frame_centre[1])),(int(aruco_center[0]),int(aruco_center[1])),(255,0,255),5)
                    cv.line(frame,(int(frame_centre[0]),int(frame_centre[1])),(int(aruco_center[0]),int(frame_centre[1])),(255,0,255),5)
                    cv.line(frame,(int(aruco_center[0]),int(aruco_center[1])),(int(aruco_center[0]),int(frame_centre[1])),(255,0,255),5)

                    distX = aruco_center[0] - frame_centre[0]
                    distY = aruco_center[1] - frame_centre[1]

                    if distY==0:
                        distY = 1

                    angle = int((np.arctan(int(distX)/int(distY))/3.14)*360)

                    
                    ###################################################
                    #    changing angle based on quadrants            #
                    #     dir = 1 is clockwise rotation               #
                    #     dir =-1 is anti-clockwise rotation          #
                    ###################################################

                    ###################################################
                    #             1st quadrant                        #
                    ###################################################

                    if distX>0 and distY<0:
                        angle = -angle
                        dir = 1

                    ###################################################
                    #             2nd quadrant                        #
                    ###################################################

                    if distX<0 and distY<0:
                        angle = angle
                        dir = -1

                    ###################################################
                    #             3rd quadrant                        #
                    ###################################################

                    if distX<0 and distY>0:
                        angle = 180 + angle
                        dir = -1

                    ###################################################
                    #             4th quadrant                        #
                    ###################################################
                
                    if distX>0 and distY>0:
                        angle = 180 - angle
                        dir = 1
                    
                    

                    cv.circle(frame, (int(aruco_center[0]),int(aruco_center[1])), 5, (0, 0, 255), -1)

                    #################################################################
                    #indexes 0 = starting point
                    #        1 = dropzone
                    #        2-7 =  aruco packages
                    ###############################################################
 
                    Angle = angle
                    distance = actual_centers_dist
                    break

        # writer.write(frame)

        endtime=time.time()

        # if(endtime-starttime>20):
        #     break
                
        cv.imshow("frame",frame)
        key = cv.waitKey(1)
        if key==ord("q"):
            break



# Function to arm and then takeoff to a user specified altitude
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


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    

def condition_yaw(heading, dir, relative=True):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees)."""
    
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        dir,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def find_and_move():
    global angle_relative_to_start,distance,dir,Angle,isarucoDetected,Aruco
    while not isarucoDetected:
        send_ned_velocity(1,0,0)
        # print("Moving forward,aruco not detected")
        # time.sleep(1)
    if isarucoDetected:
        print("Turning towards aruco marker by angle: ",Angle)
        print("dir is:",dir)

        condition_yaw(Angle,dir)
        if dir == 1:
            angle_relative_to_start = angle_relative_to_start + Angle
        if dir == -1:
            angle_relative_to_start = angle_relative_to_start - Angle

        time.sleep(10)
        
        
        print("Moving forwards towards aruco")
        while True:
            if distance<10:
                print("Reached aruco marker")
                break
            
            velocity = 5
            k=0.03
            velocity = 5 - k*distance
            send_ned_velocity(velocity,0,0)

            ############################################################
            #         updating angle everytime it moves forward        #
            # condition_yaw(Angle,dir)
            # time.sleep(3)
            # if dir == 1:
            #     angle_relative_to_start = angle_relative_to_start + Angle
            # if dir == -1:
            #     angle_relative_to_start = angle_relative_to_start - Angle
            ############################################################          
            # time.sleep(1)


def move_to_dropzone():
    global Angle,angle_relative_to_start,Aruco,startAruco,dropZoneAruco
    ##################################################################
    # moving to the south of drone angle before the start of mission
    print("Moving to south by angle: ",180 - angle_relative_to_start)

    condition_yaw(180 - angle_relative_to_start,1)
    time.sleep(10)
    angle_relative_to_start = 180
    ##################################################################

    while not dropZoneAruco and not startAruco:
        send_ned_velocity(1,0,0)

    ##############################################################################
    # if arucomarker of start point detected move tangentially to detected angle    
    if startAruco:
        Angle = 90 - Angle
        condition_yaw(Angle,1)
        print("Startzone detected, moving tangentially to it by angle: ",Angle)
        angle_relative_to_start = angle_relative_to_start + Angle
        time.sleep(10)
    ##############################################################################

    ###################################
    #find dropzone and move towards it 
    find_and_move()
    ###################################

    condition_yaw(180-angle_relative_to_start,1)
    angle_relative_to_start=0



def main():
    
    global Angle,distance,dir,angle_relative_to_start,Aruco,isarucoDetected
    print("Turning 45 degrees")
    condition_yaw(45,-1)
    angle_relative_to_start = angle_relative_to_start - 45
    time.sleep(10)
      
    ###############################
    #find aruco of package 1 and move towards it
    Aruco = True
    find_and_move()
    isarucoDetected = False
    ###############################
    move_to_dropzone()
    present_aruco = present_aruco + 1


arm_and_takeoff(5)

t1 = threading.Thread(target = DisplayFrame)
t2 = threading.Thread(target = main)


t1.start()
t2.start()

t1.join()
t2.join()



print("Now lets go to home")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object
vehicle.close()