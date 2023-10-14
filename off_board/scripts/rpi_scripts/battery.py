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

class FLIGHT_CONTROLLER:

    def get_battery_status(self, data):
            self.bat_percentage = data.percentage
            self.current = data.current
            self.voltage = data.voltage
            self.capacity = data.design_capacity

    self.bat_status = rospy.Subscriber('/mavros/battery', BatteryState, self.get_battery_status)




if __name__ == '__main__':
     mav = FLIGHT_CONTROLLER()
     bat_data = []
     while True:
            bat_data.append([mav.current,mav.voltage,mav.capacity])
            print(mav.current,mav.voltage,mav.capacity)
            time.sleep(0.002)
            if mav.capacity>=0.8*22000:
                break
        #print(bat_data[10:20])
        #bat_data = np.array(bat_data)
       

