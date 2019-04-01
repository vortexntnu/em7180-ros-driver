#!/usr/bin/env python
'''
   publisherEM7180.py uses parts of
		mastertest.py: Example Python script for running EM7180 SENtral sensor hub in master mode.

		Copyright (C) 2018 Simon D. Levy

		This file is part of EM7180.

		EM7180 is free software: you can redistribute it and/or modify
		it under the terms of the GNU General Public License as published by
		the Free Software Foundation, either version 3 of the License, or
		(at your option) any later version.

		EM7180 is distributed in the hope that it will be useful,
		but WITHOUT ANY WARRANTY without even the implied warranty of
		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
		GNU General Public License for more details.
		You should have received a copy of the GNU General Public License
		along with EM7180.  If not, see <http://www.gnu.org/licenses/>.

	publisherEM7180.py is a part of ros_EM7180. A package for ROS to be used with the EM7180 IMU
	This project is maintained by vortexntnu
'''

from em7180 import EM7180_Master


import math
import time

import rospy

# Ros messages
##from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from ros_em7180.msg import Ximu
#from geometry_msgs.msg import Vector3

#This gives the following output
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#geometry_msgs/Quaternion orientation
#  float64 x
#  float64 y
#  float64 z
#  float64 w
#float64[9] orientation_covariance
#geometry_msgs/Vector3 angular_velocity
#  float64 x
#  float64 y
#  float64 z
#float64[9] angular_velocity_covariance
#geometry_msgs/Vector3 linear_acceleration
#  float64 x
#  float64 y
#  float64 z
#float64[9] linear_acceleration_covariance



#from std_msgs.msg import String
#from std_msgs.msg import Float64


# -------------------------------------

#
	
			

		
		
		
	
	

MAG_RATE       = 100  # Hz
ACCEL_RATE     = 200  # Hz
GYRO_RATE      = 200  # Hz
BARO_RATE      = 50   # Hz
Q_RATE_DIVISOR = 3    # 1/3 gyro rate

em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR)

# Start the EM7180 in master mode
if not em7180.begin():
	print(em7180.getErrorString())
	exit(1)

while True:

	em7180.checkEventStatus()

	if em7180.gotError():
		print('ERROR: ' + em7180.getErrorString())
		exit(1)

    # Define output variables from updated quaternion---these are Tait-Bryan
    # angles, commonly used in aircraft orientation.  In this coordinate
    # system, the positive z-axis is down toward Earth.  Yaw is the angle
    # between Sensor x-axis and Earth magnetic North (or true North if
    # corrected for local declination, looking down on the sensor positive
    # yaw is counterclockwise.  Pitch is angle between sensor x-axis and
    # Earth ground plane, toward the Earth is positive, up toward the sky is
    # negative.  Roll is angle between sensor y-axis and Earth ground plane,
    # y-axis up is positive roll.  These arise from the definition of the
    # homogeneous rotation matrix constructed from q.  Tait-Bryan
    # angles as well as Euler angles are non-commutative that is, the get
    # the correct orientation the rotations must be applied in the correct
    # order which for this configuration is yaw, pitch, and then roll.  For
    # more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles 
    # which has additional links.
    
	def publishIMUSensorData(magX, magY, magZ):
	
		
	
		# Publisher
		magneticFieldPublisher=rospy.Publisher('imu/mag',MagneticField,queue_size=10)

		# Initialize node
		rospy.init_node('em7180', anonymous=False)
	
		rate=rospy.Rate(10)

		while not rospy.is_shutdown():
			# Initilize objects
			
			magneticVector = MagneticField()

			magneticVector.header.stamp=rospy.Time.now()
			magneticVector.header.frame_id="magnetometer_link"

			magneticVector.magnetic_field.x=magX
			magneticVector.magnetic_field.y=magY
			magneticVector.magnetic_field.z=magZ

			magneticVector.magnetic_field_covariance=[700,0,0,0,700,0,0,0,700] 
				
			
			magneticFieldPublisher.publish(magneticVector)
				
			rate.sleep()
    

	if em7180.gotMagnetometer():

		mx,my,mz = em7180.readMagnetometer()
		
        
	if __name__ == '__main__':
		try:
			publishIMUSensorData(mx,my,mz)
		except rospy.ROSInterruptException:
			pass
    
    #time.sleep(.1)
    #rospy.Rate(5).sleep() # 10=10Hz; 1=1sec
