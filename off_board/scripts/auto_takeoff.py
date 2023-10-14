#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest,CommandTOL

lat,lon =0,0


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException:
        print ("service set_mode call failed: . GUIDED Mode could not be set. Check that GPS is enabled")

def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException:
        print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")

def setArm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
       armService(True)
   except rospy.ServiceException:
       print("Service arm call failed: ")

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armService(False)
    except rospy.ServiceException:
        print ("Service arm call failed:")

def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException:
        print ("Service takeoff call failed:")

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException:
        print ("service land call failed: %s. The vehicle cannot")

def moveCircle():

   circle_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
   circle = TwistStamped()


   circle.twist.linear.x=0.5
   circle.twist.linear.z=0.5
   circle_pub.publish(circle)

def CallBack(data):
    global lat, lon
    msg = data
    lat = msg.latitude
    lon = msg.longitude 

if __name__ == '__main__':
   rospy.init_node('gapter_pilot_node', anonymous=True)
   rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, CallBack)
   velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
   x='1'
   while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7','9'])):
    x = input("Enter your input: ")
    if (x=='1'):
        setGuidedMode()
    elif(x=='2'):
        setStabilizeMode()
    elif(x=='3'):
        setArm()
    elif(x=='4'):
        setDisarm()
    elif(x=='5'):
        setTakeoffMode()
    elif(x=='6'):
        setLandMode()
    elif(x=='7'):
        print(f"{lat} , {lon}")
    elif(x == '9'):
        moveCircle()

