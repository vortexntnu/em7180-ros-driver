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


def callback1(data):
	global gtemp
	#rospy.loginfo(rospy.get_caller_id() + 'Temperature is: %s \n', data.data)
	gtemp.set('%2.2f C' % data.data)
def callback2(data):
	global gpitch
	#rospy.loginfo(rospy.get_caller_id() + 'Pitch is: %s', data.data)
	gpitch.set('%2.2f rad' % data.data)
def callback3(data):
	global groll
	#rospy.loginfo(rospy.get_caller_id() + 'Roll is: %s', data.data)
	groll.set('%2.2f' % data.data)
def callback4(data):
	global gyaw
	#rospy.loginfo(rospy.get_caller_id() + 'Yaw is: %s', data.data)
	gyaw.set('%2.2f' % data.data)
def callback5(data):
	global gax
	#rospy.loginfo(rospy.get_caller_id() + 'ax is: %s', data.data)
	gax.set('%2.2f x' % data.data)
def callback6(data):
	global gay
	#rospy.loginfo(rospy.get_caller_id() + 'ay is: %s', data.data)
	gay.set('%2.2f y' % data.data)
def callback7(data):
	global gaz
	#rospy.loginfo(rospy.get_caller_id() + 'az is: %s', data.data)
	gaz.set('%2.2f z' % data.data)
def callback8(data):
	global ggx
	#rospy.loginfo(rospy.get_caller_id() + 'gx is: %s', data.data)
	ggx.set('%2.2f x' % data.data)
def callback9(data):
	global ggy
	#rospy.loginfo(rospy.get_caller_id() + 'gy is: %s', data.data)
	ggy.set('%2.2f y' % data.data)
def callback10(data):
	global ggz
	#rospy.loginfo(rospy.get_caller_id() + 'gz is: %s', data.data)
	ggz.set('%2.2f z' % data.data)
def callback11(data):
	global gpressure
	#rospy.loginfo(rospy.get_caller_id() + 'Pressure is: %s', data.data)
	gpressure.set('%2.2f mbar' % data.data)
def callback12(data):
	global galtitude
	#rospy.loginfo(rospy.get_caller_id() + 'Altitude is: %s', data.data)
	galtitude.set('%2.2f m' % data.data)





def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	#rospy.Subscriber('chatter', String, callback)
	rospy.Subscriber('temperature', Float64, callback1)
	rospy.Subscriber('pitch', Float64, callback2)
	rospy.Subscriber('roll', Float64, callback3)
	rospy.Subscriber('yaw', Float64, callback4)
	rospy.Subscriber('ax', Float64, callback5)
	rospy.Subscriber('ay', Float64, callback6)
	rospy.Subscriber('az', Float64, callback7)
	rospy.Subscriber('gx', Float64, callback8)
	rospy.Subscriber('gy', Float64, callback9)
	rospy.Subscriber('gz', Float64, callback10)
	rospy.Subscriber('pressure', Float64, callback11)
	rospy.Subscriber('altitude', Float64, callback12)

	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()

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


