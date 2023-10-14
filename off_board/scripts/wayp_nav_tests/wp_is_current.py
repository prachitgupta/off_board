#!/usr/bin/env python2
from cmath import sqrt
import numpy as np
import rospy
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from geometry_msgs.msg import *
import math
from time import sleep
from tf.transformations import euler_from_quaternion
ARM_RAD=1
DEADBAND_WIDTH = 0.2
#Variables
d_s = 10 #start distance
d_p = 20 #pilon distance
d = 2 #drone width
v_d = 20 #max velocity


class stateMoniter:
	def __init__(self):
		self.state = State()
		# Instantiate a setpoints message
		
	def stateCb(self, msg):
		# Callback function for topic /mavros/state
		self.state = msg

class wpMissionCnt:

	def __init__(self):
		self.wp =Waypoint()
		
	def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
		self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.og/api/mavros_msgs/html/msg/Waypoint.html
		self.wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
		self.wp.is_current= is_current
		self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
		self.wp.param1=param1 # To know more about these params, visit https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
		self.wp.param2=param2
		self.wp.param3=param3
		self.wp.param4=param4
		self.wp.x_lat= x_lat 
		self.wp.y_long=y_long
		self.wp.z_alt= z_alt #relative altitude.

		return self.wp

class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()
		self.orient = Quaternion()
		self.angles = Point()
		self.gps = Point()
		stateMt = stateMoniter()
		self.transformation_matrix = np.array([[0, -1, 0], [1,0,0], [0,0,1]])
		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		# self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel,)
		# self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)
		self.get_gps_location = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.get_gps)
		self.state_subsciber = rospy.Subscriber('/mavros/state',State, stateMt.stateCb)
		rospy.Subscriber("/mavros/local_position/local",PoseStamped, self.get_yaw)
		rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=0)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
		self.waypoint_curr = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)

		rospy.loginfo('INIT')

		self.pt.x = 2
		self.pt.y = 2
		self.pt.z = 2

	#MODE SETUP

	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		# self.gps_subscriber

		# t_lat = self.gps_lat
		# t_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
	
	
	def land(self, l_alt):

		# self.gps_subscriber

		# l_lat = self.gps_lat
		# l_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/land')
		try:
			self.land_service(0.0, 0.0, 0, 0, l_alt)
			rospy.loginfo("LANDING")

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	def set_mode(self,md):

		rospy.wait_for_service('/mavros/set_mode')
		try:
			self.flight_mode_service(0, md)
			rospy.loginfo("Mode changed")
				
		except rospy.ServiceException as e:
			rospy.loginfo("Mode could not be set: " %e)

	def wpPush(self,wps):
		# Call /mavros/mission/push to push the waypoints
		# and print fail message on failure
		rospy.wait_for_service('/mavros/mission/push')
		try:
			self.waypoint_push(0, wps)
			rospy.loginfo("Waypoint pushed")

		except:
			print ("Service waypoint push call failed")



	def set_Guided_mode(self):
		
		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def set_Altitude_Hold_mode(self):

		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("ALT_HOLD")	

	#CALLBACKS

	def get_gps(self, data):
		self.gps.x = data.latitude
		self.gps.y = data.longitude
		self.gps.z = data.altitude


	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

		# orientation in space  
		self.orient.x = location_data.pose.orientation.x
		self.orient.y = location_data.pose.orientation.y
		self.orient.z = location_data.pose.orientation.z
		self.orient.w = location_data.pose.orientation.w

	def within_rad(self):
		if (((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2) < (ARM_RAD)**2):
			return True
		print((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2)
		return False

	def reached(self, data):
		self.rch_index=data.wp_seq


# this function rotates the coordinates considering the spawn point and direction as the origin
	def rotate(self, point):
		angle = self.angles.z - math.pi/2
		px, py = point[0], point[1]
		nx =  math.cos(angle) * px  - math.sin(angle) * py 
		ny =  math.sin(angle) * px  + math.cos(angle) * py 
		return [nx, ny, point[2]]
		
	def get_eulers(self,q):
		eulers = euler_from_quaternion(q)
		return eulers[2]

	def get_yaw(self, data):
		q = []
		q.append(data.pose.pose.orientation.x)
		q.append(data.pose.pose.orientation.y)
		q.append(data.pose.pose.orientation.z)
		q.append(data.pose.pose.orientation.w)
		angle  = self.get_eulers(q)
		self.angles.x = 0.0
		self.angles.y = 0.0
		self.angles.z = angle
		print(angle)

		
# this function corrects the frame offset converting the ardupilot frame (x-right, y-front) to 
# gazebo local frame (x-front, y-left)
	def corrected_pose(self, current_pos):
		current_pos = np.array(current_pos)
		new_pos = np.matmul(self.transformation_matrix,current_pos)
		return list(new_pos)


	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		given_position = np.array([x,y,z])
		x = given_position[0]
		y = given_position[1]
		z = given_position[2]
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z

		ix = self.orient.x
		iy = self.orient.y
		iz = self.orient.z
		iw = self.orient.w

		sp.pose.orientation.x = ix
		sp.pose.orientation.y = iy
		sp.pose.orientation.z = iz
		sp.pose.orientation.w = iw


		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > DEADBAND_WIDTH):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		print('Reached')



# helper functions for xy to latlon conversions
def mdeglon(lat0):
	lat0rad = math.radians(lat0)
	return (111415.13 * math.cos(lat0rad)- 94.55 * math.cos(3.0*lat0rad)- 0.12 * math.cos(5.0*lat0rad) )

def mdeglat(lat0):
	lat0rad = math.radians(lat0)
	return (111132.09 - 566.05 * math.cos(2.0*lat0rad)+ 1.20 * math.cos(4.0*lat0rad)- 0.002 * math.cos(6.0*lat0rad) )

# xy to latlon conversions

def xy2latlon(local, origin):
	lon = local[0]/mdeglon(origin[0]) + origin[1]
	lat = local[1]/mdeglat(origin[1]) + origin[0]
	alt = local[2] + origin[2]
	gl = np.array([lat, lon, alt])
	return gl

def latlon2xy(coordinates, origin):
	x = (coordinates[1]- origin[1]) + mdeglon(origin[0])
	y = (coordinates[0] - origin[0]) + mdeglat(origin[0])
	return x,y
	

def compute_waypoints(mav):
		wayp_11 = wpMissionCnt()
		wayp_12 = wpMissionCnt()
		wayp_13 = wpMissionCnt()
		wayp_21 = wpMissionCnt()
		wayp_22 = wpMissionCnt()
		wayp_23 = wpMissionCnt()
		wayp_last=wpMissionCnt()



		# defining the origin as the gps coordinates of the spawn point
		origin = [mav.gps.x, mav.gps.y, 0]
		#####################################################################################
		
		#####################################################################################

		# coordinates given in the gazebo frame (x-front, y- left)
		wayp11 = [0, 0 ,5]
		wayp12 = [0, 34.64 ,5]
		wayp13 = [-20, 34.64,5]
		wayp21 = [80, 0,5]
		wayp22 = [60,  -34.64 ,5]
		wayp23 = [20,-34.64,5]

		# the coordinates are received in ardupilot frame (x- right, y- front)

		# converting the ardupilot frame coordinates to gazebo frame coordinates
		# wayp11 = mav.corrected_pose(wayp11)
		# wayp12 = mav.corrected_pose(wayp12)
		# wayp13 = mav.corrected_pose(wayp13)
		# wayp21 = mav.corrected_pose(wayp21)
		# wayp22 = mav.corrected_pose(wayp22)
		# wayp23 = mav.corrected_pose(wayp23)
		
		
		# rotate changes the axes to the current heading direction
		# wayp11 = mav.rotate(wayp11)
		# wayp12 = mav.rotate(wayp12)
		# wayp13 = mav.rotate(wayp13)
		# wayp21 = mav.rotate(wayp21)
		# wayp22 = mav.rotate(wayp22)
		# wayp23 = mav.rotate(wayp23)

		# compensated the spawn orientation

		# extracting latitude and longitude from given x,y,z coordinates 
		wp_g_11 = xy2latlon(wayp11, origin)
		wp_g_12 = xy2latlon(wayp12, origin)
		wp_g_13 = xy2latlon(wayp13, origin)
		wp_g_21 = xy2latlon(wayp21, origin)
		wp_g_22 = xy2latlon(wayp22, origin)
		wp_g_23 = xy2latlon(wayp23, origin)

		# print("waypoints in global frame:")
		# print (wp_g_11)
		# print (wp_g_12)
		# print (wp_g_13)
		# print (wp_g_21)
		# print (wp_g_22)
		# print (wp_g_23)

		wps = []
		#LAP1
		w = wayp_11.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),wp_g_11[0], wp_g_11[1], wp_g_11[2])
		# appending twice to avoid the skipping of first waypoint
		wps.append(w)
		wps.append(w)
		w = wayp_12.setWaypoints(3,16,False,False,0.0,0.0,0.0,float('nan'),wp_g_12[0], wp_g_12[1], wp_g_12[2])
		wps.append(w)
		while(mav.rch_index!=3):
			continue 
		w= wayp_11.setWaypoints(3,16,True,False,0.0,0.0,0.0,float('nan'),wp_g_13[0], wp_g_13[1], wp_g_13[2])
		wps.append(w)
		mav.wpPush(w)


if __name__ == '__main__':

	mav = FLIGHT_CONTROLLER()
	stateMt = stateMoniter()

	rate= rospy.Rate(20.0)
	time.sleep(3)
	print(mav.within_rad())
	if (True):		
		# defining waypoints
		# 16 -> NAVIGATE
		# 21 -> LAND
		# 22 -> TAKEOFF
		mav.set_mode('STABILIZE')
		mav.toggle_arm(1)
		time.sleep(3)
		mav.set_Guided_mode()
		mav.takeoff(5)
		time.sleep(2)
		mav.set_Guided_mode()
		time.sleep(2)
		compute_waypoints(mav)
		mav.set_mode("AUTO")
		# mav.land(5)	
		mav.toggle_arm(0)

