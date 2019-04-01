#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) Vortex NTNU.
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


import rospy
import math
import time
from datetime import datetime
from sensor_msgs.msg import MagneticField
from visualization_msgs import Marker





def callback(data):

	global x_vector, y_vector, z_vector

	x_vector=data.magnetic_field.x
	y_vector=data.magnetic_field.y
	z_vector=data.magnetic_field.z


def listenForMagneticVector():
	rospy.init_node('VisulazationNode', anonymous=False)

	rospy.Subscriber('imu/mag',MagneticField, callback)

	rospy.spin()


def publishVisulazationOfMagneticVector():

	visulazationPublisher=rospy.Publisher('Em7180/vis',Marker,queue_size=10)

	# Initialize node
	rospy.init_node('VisulationOfMagneticVector', anonymous=False)


	rate=rospy.Rate(10)

	while not rospy.is_shutdown():


		visualization=Marker()

		visualization.header.frame_id="magnetometer_link"
		visualization.header.stamp=rospy.Time()
		visualization.ns="my_namespace"
		visualization.id=0
		visualization.type=0
		visualization.action=0
		visualization.pose.position.x=0
		visualization.pose.position.y=0
		visualization.pose.position.z=0
		visualization.points[0].x = 0
		visualization.points[0].y = 0
		visualization.points[0].z = 0
		visualization.points[1].x = x_vector
		visualization.points[1].x = y_vector
		visualization.points[1].x = z_vector
		visualization.pose.orientation.x =0
		visualization.pose.orientation.y =0
		visualization.pose.orientation.z =0
		visualization.pose.orientation.w =1
		visualization.scale.x=0.2
		visualization.scale.y=0.3
		visualization.scale.z=0.2

		visulazationPublisher.publish(visualization)

		rate.sleep()


if __name__ == '__main__':
	listenForMagneticVector()
	try:
		publishVisulazationOfMagneticVector()
	except rospy.ROSinterruptException:
		pass

	