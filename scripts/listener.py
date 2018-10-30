#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from ros_em7180.msg import Ximu
from Tkinter import Tk, Label, Button, W, E, StringVar
root = Tk()

gtemp = StringVar(root) # callback1
gpitch = StringVar(root) # callback2
groll = StringVar(root) # callback3
gyaw = StringVar(root) # callback4
gax = StringVar(root) # callback5
gay = StringVar(root) # callback6
gaz = StringVar(root) # callback7
ggx = StringVar(root) # callback8
ggy = StringVar(root) # callback9
ggz = StringVar(root) # callback10
gpressure = StringVar(root) # callback11
galtitude = StringVar(root) # callback12


# Parse the data from the subscriber
def callback(data):
	gtemp.set('%2.2f C' % data.temperature.temperature)
	gpressure.set('%2.2f mbar' % data.pressure.fluid_pressure)
	galtitude.set('%2.2f m' % data.altitude)
	
	groll.set('%2.2f ' % data.imu.orientation.x)
	gpitch.set('%2.2f ' % data.imu.orientation.y)
	gyaw.set('%2.2f ' % data.imu.orientation.z)
	
	gax.set('%2.2f x' % data.imu.linear_acceleration.x)
	gay.set('%2.2f y' % data.imu.linear_acceleration.y)
	gaz.set('%2.2f z' % data.imu.linear_acceleration.z)
	
	ggx.set('%2.2f x' % data.imu.angular_velocity.x)
	ggy.set('%2.2f y' % data.imu.angular_velocity.y)
	ggz.set('%2.2f z' % data.imu.angular_velocity.z)





def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	#rospy.Subscriber('chatter', String, callback)
	rospy.Subscriber('SensorData', Ximu, callback)

	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()

# GUI stuff
root.title("ROS EM7180")

label = Label(root, text="Displaying data collected from EM7180 IMU")
label.grid(columnspan=5, sticky=W)
label.config(font=("Courier", 30))

label_ph2 = Label(root,  text="             ")
label_ph2.grid(row=1, column=2)
label_ph2.config(font=("Courier", 22))
label_ph3 = Label(root,  text="             ")
label_ph3.grid(row=1, column=3)
label_ph3.config(font=("Courier", 22))


label_pitchT = Label(root,  text="Pitch:")
label_pitchT.grid(row=3, sticky=E)
label_pitchT.config(font=("Courier", 22))

label_pitch = Label(root,  textvariable=gpitch)
label_pitch.grid(row=3, column=1)
label_pitch.config(font=("Courier", 22))

label_rollT = Label(root,  text="Roll:")
label_rollT.grid(row=4, sticky=E)
label_rollT.config(font=("Courier", 22))

label_roll = Label(root,  textvariable=groll)
label_roll.grid(row=4, column=1)
label_roll.config(font=("Courier", 22))

label_yawT = Label(root,  text="Yaw:")
label_yawT.grid(row=5, sticky=E)
label_yawT.config(font=("Courier", 22))

label_yaw = Label(root,  textvariable=gyaw)
label_yaw.grid(row=5, column=1)
label_yaw.config(font=("Courier", 22))

label_accT = Label(root,  text="Acceleration:")
label_accT.grid(row=6, sticky=E)
label_accT.config(font=("Courier", 22))

label_accX = Label(root,  textvariable=gax)
label_accX.grid(row=6, column=1)
label_accX.config(font=("Courier", 22))
label_accY = Label(root,  textvariable=gay)
label_accY.grid(row=6, column=2)
label_accY.config(font=("Courier", 22))
label_accZ = Label(root,  textvariable=gaz)
label_accZ.grid(row=6, column=3)
label_accZ.config(font=("Courier", 22))

label_gyroT = Label(root,  text="Gyro:")
label_gyroT.grid(row=7, sticky=E)
label_gyroT.config(font=("Courier", 22))

label_gyroX = Label(root,  textvariable=ggx)
label_gyroX.grid(row=7, column=1)
label_gyroX.config(font=("Courier", 22))
label_gyroY = Label(root,  textvariable=ggy)
label_gyroY.grid(row=7, column=2)
label_gyroY.config(font=("Courier", 22))
label_gyroZ = Label(root,  textvariable=ggz)
label_gyroZ.grid(row=7, column=3)
label_gyroZ.config(font=("Courier", 22))

label_tempT = Label(root,  text="Temperature:")
label_tempT.grid(row=8, sticky=E)
label_tempT.config(font=("Courier", 22))

label_temp = Label(root,  textvariable=gtemp)
label_temp.grid(row=8, column=1)
label_temp.config(font=("Courier", 22))

label_pressT = Label(root,  text="Pressure:")
label_pressT.grid(row=9, sticky=E)
label_pressT.config(font=("Courier", 22))

label_press = Label(root,  textvariable=gpressure)
label_press.grid(row=9, column=1)
label_press.config(font=("Courier", 22))

label_altT = Label(root,  text="Altitude:")
label_altT.grid(row=10, sticky=E)
label_altT.config(font=("Courier", 22))

label_alt = Label(root,  textvariable=galtitude)
label_alt.grid(row=10, column=1)
label_alt.config(font=("Courier", 22))


	
if __name__ == '__main__':
		listener()
		

root.mainloop()


