'''
*****************************************************************************************
*
*        =================================================
*             Pharma Bot Theme (eYRC 2022-23)
*        =================================================
*                                                         
*  This script is intended for implementation of Task 2B   
*  of Pharma Bot (PB) Theme (eYRC 2022-23).
*
*  Filename:			task_2b.py
*  Created:				
*  Last Modified:		8/10/2022
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2b.py
# Functions:		control_logic, read_qr_code
# 					[ Comma separated list of functions in this file ]
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
import  sys
import traceback
import time
import os
import math
from zmqRemoteApi import RemoteAPIClient
import zmq
import numpy as np
import cv2
import random
from pyzbar.pyzbar import decode
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

################# ADD UTILITY FUNCTIONS HERE #################
v0=1
def show_qr(sim,checkpoint):
    arena_dummy_handle = sim.getObject("/Arena_dummy")
    childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")
    sim.callScriptFunction("activate_qr_code", childscript_handle, "checkpoint "+str(checkpoint))
def hide_qr(sim,checkpoint):
    arena_dummy_handle = sim.getObject("/Arena_dummy")
    childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")
    sim.callScriptFunction("deactivate_qr_code", childscript_handle, "checkpoint "+str(checkpoint))




##############################################################

def control_logic(sim):
    """
    Purpose:
    ---
    This function should implement the control logic for the given problem statement
    You are required to make the robot follow the line to cover all the checkpoints
    and deliver packages at the correct locations.

    Input Arguments:
    ---
    `sim`    :   [ object ]
        ZeroMQ RemoteAPI object

    Returns:
    ---
    None

    Example call:
    ---
    control_logic(sim)
    """
    ##############  ADD YOUR CODE HERE  ##############
    global v0
    v0 = 1
    kp = 10
    e= 0
    vis_sensor = sim.getObjectHandle("vision_sensor")
    rw = sim.getObjectHandle("right_joint")
    lw = sim.getObjectHandle("left_joint")
    # print(sim.getJointTargetPosition(rw))
    kernel = np.ones((5,5))
     
    def get_image():
        global img
        img, resX, resY = sim.getVisionSensorCharImage(vis_sensor)
        img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        #In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        #decoded.resize([res[0], res[1], 3])
        #image = np.array(Image.open(io.BytesIO(image_buffer))) 	
        return img
    
    def set_left_motor(sim,vl):
        sim.setJointTargetVelocity(lw, vl)

    def set_right_motor(sim,vr):
        sim.setJointTargetVelocity(rw, vr)
    
    def goLeft(sim):
        for i in range(0,90):
            sim.setJointTargetVelocity(rw, 2)
        sim.setJointTargetVelocity(rw, 0)
    def goLefta(sim,angle):
        for i in range(0,angle):
            sim.setJointTargetVelocity(rw, 2)
        sim.setJointTargetVelocity(rw, 0)
        
    def goRight(sim):
        for i in range(0,90):
            sim.setJointTargetVelocity(lw, 2)
        sim.setJointTargetVelocity(lw, 0)
    def goRighta(sim,angle):
        for i in range(0,angle):
            sim.setJointTargetVelocity(lw, 2)
        sim.setJointTargetVelocity(lw, 0)
    def goRight89(sim):
        for i in range(0,89):
            sim.setJointTargetVelocity(lw, 2)
        sim.setJointTargetVelocity(lw, 0)
    

    def goStraight():
    
        global v0
        img_init = get_image()

        while(True):
            # print(sim.getJointTargetPosition(rw))
            set_left_motor(sim,v0)
            set_right_motor(sim,v0)
            img = get_image()
            img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret,thres = cv2.threshold(img_grey,240,255,cv2.THRESH_BINARY) #white black no gray
            contours,_ = cv2.findContours(thres,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            #diff = cv2.matchShapes(cnt,cnt_p,1,0.0)
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

            ###
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])
            mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
            mask_yellow = cv2.dilate(mask_yellow,kernel)
            res_y = cv2.bitwise_and(img,img,mask = mask_yellow)
            ####


            cont_yellow,_ = cv2.findContours(mask_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # print(cont_yellow)
            cv2.drawContours(img, cont_yellow,-1, (0,255,0), 3)
            #cnt_p = cnt
            for pic,contour in enumerate(cont_yellow):
            #for pic,contour in enumerate(cont_yellow):
                # print(cv2.contourArea(contour))
                if  cv2.contourArea(contour)>50000: #yellow_detected
                    # cv2.drawContours(img, cont_yellow[0],-1, (0,0,255), 3)
                    #   time.sleep(0.2)
                    set_left_motor(sim,0)
                    set_right_motor(sim,0)
                    
                    v0=0
                    x,y,w,h = cv2.boundingRect(contour)
                    cv2.line(img,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2) +100),(255,0,0),5)
                    break
            if v0 ==0 :
                break
               

                """"
                
                    while(cv2.contourArea(contours[0]) < 4000):
                        set_right_motor(sim,v0/4)
                        set_left_motor(sim,v0/4)
                        img = get_image()
                        img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                        ret,thres = cv2.threshold(img_grey,240,255,cv2.THRESH_BINARY) #white black no gray
                        contours,_ = cv2.findContours(thres,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                        print(cv2.contourArea(contours[0]))
                    break
                    """
                
            cv2.imshow("PRACHIT",thres)
            #print(img)
            k = cv2.waitKey(1)
            if k == ord('z'):
                cv2.destroyAllWindows()
                break
    def goStraight1():
        global v0
        img_init = get_image()

        while(True):
            # print(sim.getJointTargetPosition(rw))
            set_left_motor(sim,v0)
            set_right_motor(sim,v0)
            img = get_image()
            img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret,thres = cv2.threshold(img_grey,240,255,cv2.THRESH_BINARY) #white black no gray
            contours,_ = cv2.findContours(thres,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            #diff = cv2.matchShapes(cnt,cnt_p,1,0.0)
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

            ###
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])
            mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
            mask_yellow = cv2.dilate(mask_yellow,kernel)
            res_y = cv2.bitwise_and(img,img,mask = mask_yellow)
            ####


            cont_yellow,_ = cv2.findContours(mask_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # print(cont_yellow)
            cv2.drawContours(img, cont_yellow,-1, (0,255,0), 3)
            #cnt_p = cnt
            for pic,contour in enumerate(cont_yellow):
            #for pic,contour in enumerate(cont_yellow):
                # print(cv2.contourArea(contour))
                if  cv2.contourArea(contour)>50000: #yellow_detected
                    # cv2.drawContours(img, cont_yellow[0],-1, (0,0,255), 3)
                    time.sleep(0.1)
                    set_left_motor(sim,0)
                    set_right_motor(sim,0)
                    
                    v0=0
                    x,y,w,h = cv2.boundingRect(contour)
                    cv2.line(img,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2) +100),(255,0,0),5)
                    break
            if v0 ==0 :
                break
               

                """"
                
                    while(cv2.contourArea(contours[0]) < 4000):
                        set_right_motor(sim,v0/4)
                        set_left_motor(sim,v0/4)
                        img = get_image()
                        img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                        ret,thres = cv2.threshold(img_grey,240,255,cv2.THRESH_BINARY) #white black no gray
                        contours,_ = cv2.findContours(thres,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                        print(cv2.contourArea(contours[0]))
                    break
                    """
                
            cv2.imshow("PRACHIT",thres)
            #print(img)
            k = cv2.waitKey(1)
            if k == ord('z'):
                cv2.destroyAllWindows()
                break
    global pkg
    pkg=1
    goStraight()
    goLeft(sim)
    v0=1
    goStraight()
    goRight(sim)

    v0=1
    goStraight()
    goLeft(sim)

    v0=1
    goStraight()
    goRight(sim)

    v0=1
    goStraight1()
    show_qr(sim,'E')    
    print(read_qr_code(sim,"E"))
    hide_qr(sim,'E')
    v0=2
    pkg=2
    for i in range(0,40):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    goStraight()
    goRighta(sim,91)
    goStraight()
    v0=2
    goStraight()
    goLeft(sim)
    v0=2
    goStraight()
    goRight(sim)
    v0=2
    goStraight()
    v0=2
    for i in range(0,3):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    set_left_motor(sim,0)
    set_right_motor(sim,0)
    show_qr(sim,"I")    
    # time.sleep(5)
    print(read_qr_code(sim,"I"))
    hide_qr(sim,"I")
    pkg=3
    v0=2
    for i in range(0,20):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    goStraight()
    goRighta(sim,91)
    v0=2    
    goStraight()
    goLefta(sim,92)
    v0=2
    goStraight()
    goRighta(sim,92)
    v0=2
    goStraight()
    v0=2
    for i in range(0,3):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    set_left_motor(sim,0)
    set_right_motor(sim,0)
    show_qr(sim,"M")   
    print(read_qr_code(sim,"M"))
    hide_qr(sim,"M")
    v0=2
    for i in range(0,20):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    goStraight()
    goRighta(sim,90)
    v0=2
    goStraight()
    goLefta(sim,92)
    v0=2
    goStraight()
    goRight(sim)
    v0=2
    goStraight()
    v0=2
    for i in range(0,3):
        set_left_motor(sim,v0)
        set_right_motor(sim,v0)
    v0=0    
    # for i in range(0,10):
    # 	set_left_motor(sim,v0)
    # 	set_right_motor(sim,v0)
    # # goStraight()
    # goLeft(sim)
    # v0=2
    # goStraight()
    # for i in range(0,10):
    # 	set_left_motor(sim,v0)
    # 	set_right_motor(sim,v0)
    # goStraight()
    # v0=2
    # goStraight()
    # v0=2
    # goRighta(sim,90)
    # goStraight()
    # v0=2
    # for i in range(0,10):
    # 	set_left_motor(sim,v0)
    # 	set_right_motor(sim,v0)
    # goStraight()
    # # goRight89(sim)



    ##################################################

def read_qr_code(sim,checkpoint):
    arena_dummy_handle = sim.getObject("/Arena_dummy")
    childscript_handle = sim.getScript(sim.scripttype_childscript, arena_dummy_handle, "")
    """
    Purpose:
    ---
    This function detects the QR code present in the camera's field of view and
    returns the message encoded into it.

    Input Arguments:
    ---
    `sim`    :   [ object ]
        ZeroMQ RemoteAPI object

    Returns:
    ---
    `qr_message`   :    [ string ]
        QR message retrieved from reading QR code

    Example call:
    ---
    control_logic(sim)
    """
    qr_message = None
    ##############  ADD YOUR CODE HERE  ##############
    vis_sensor = sim.getObjectHandle("vision_sensor")
    rw = sim.getObjectHandle("right_joint")
    lw = sim.getObjectHandle("left_joint")
    img, resX, resY = sim.getVisionSensorCharImage(vis_sensor)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    # cv2.imshow(" ",img)
    # k = cv2.waitKey(1)
    # if k == ord('q'):
    #       cv2.destroyAllWindows()
    Qr_codes_details = " "
    

## Retrieve the handle of the child script attached to the Arena_dummy scene object.
    

## Call the activate_qr_code() function defined in the child script to make the QR code visible at checkpoint E
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (T, threshInv) = cv2.threshold(gray, 90, 255,
    cv2.THRESH_BINARY)
    # cv2.imshow("Threshold Binary", threshInv)
    # cv2.waitKey(0)
    barcodes = decode(threshInv)
    #print ((str(barcodes[0][0])[1:])) 
    for i in range (len(barcodes)):
        Qr_codes_details=(str(barcodes[i][0])[2:-1])
    
    sim.callScriptFunction("deliver_package", childscript_handle, "package_"+str(pkg), "checkpoint "+str(checkpoint))
#waits for user to press any key 
#(this is necessary to avoid Python kernel form crashing)
    ##################################################
    
    return Qr_codes_details    	
    
    ##################################################
    


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')	

    try:

        ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the robot navigation logic written by participants
        try:
            control_logic(sim)
            time.sleep(5)

        except Exception:
            print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

        
        ## Stop the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except KeyboardInterrupt:
        ## Stop the simulation using ZeroMQ RemoteAPI
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation interrupted by user in CoppeliaSim.')
        else:
            print('\nSimulation could not be interrupted. Stop the simulation manually .')
            sys.exit()