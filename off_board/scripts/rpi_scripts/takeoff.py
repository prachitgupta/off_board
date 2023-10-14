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

class stateMoniter:
	def __init__(self):
		self.state = State()
		# Instantiate a setpoints message
		
	def stateCb(self, msg):
		# Callback function for topic /mavros/state
		self.state = msg

class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()
		self.orient = Quaternion()
		self.angles = Point()
		self.gps = Point()
		stateMt = stateMoniter()
		self.reached_index=0 
		self.transformation_matrix = np.array([[0, -1, 0], [1,0,0], [0,0,1]])
		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_gps_location = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.get_gps)
		self.state_subsciber = rospy.Subscriber('/mavros/state',State, stateMt.stateCb)
		rospy.Subscriber("/mavros/local_position/local",PoseStamped, self.get_yaw)
		self.bat_status = rospy.Subscriber('/mavros/battery', BatteryState, self.get_battery_status)
		

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=0)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		
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
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
	
	
	def land(self, l_alt):
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


	def get_battery_status(self, data):
		self.bat_percentage = data.percentage


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


if __name__ == '__main__':

	mav = FLIGHT_CONTROLLER()
	stateMt = stateMoniter()

	#Set time checkpoint for 800 seconds
	warn_time = time.time()+500

	rate= rospy.Rate(20.0)
	time.sleep(3)
	mav.set_mode('STABILIZE')
	mav.toggle_arm(1)
	time.sleep(3)
	mav.set_Guided_mode()
	mav.takeoff(2)
	time.sleep(10)
	mav.land(0)


		
