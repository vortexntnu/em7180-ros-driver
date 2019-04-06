#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs import Marker


def callback(data):
    global x_vector, y_vector, z_vector

    x_vector = data.magnetic_field.x
    y_vector = data.magnetic_field.y
    z_vector = data.magnetic_field.z


def listen_for_magnetic_vector():
    rospy.init_node('visualization_node')
    rospy.Subscriber('imu/mag', MagneticField, callback)
    rospy.spin()


def publish_visualization_of_magnetic_vector():
    visualization_publisher = rospy.Publisher('em7180/vis', Marker, queue_size=10)

    # Initialize node
    rospy.init_node('magnetic_vector_visualization')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        visualization = Marker()

        visualization.header.frame_id = "magnetometer_link"
        visualization.header.stamp = rospy.Time()
        visualization.ns = "my_namespace"
        visualization.id = 0
        visualization.type = 0
        visualization.action = 0
        visualization.pose.position.x = 0
        visualization.pose.position.y = 0
        visualization.pose.position.z = 0
        visualization.points[0].x = 0
        visualization.points[0].y = 0
        visualization.points[0].z = 0
        visualization.points[1].x = x_vector
        visualization.points[1].y = y_vector
        visualization.points[1].z = z_vector
        visualization.pose.orientation.x = 0
        visualization.pose.orientation.y = 0
        visualization.pose.orientation.z = 0
        visualization.pose.orientation.w = 1
        visualization.scale.x = 0.2
        visualization.scale.y = 0.3
        visualization.scale.z = 0.2

        visualization_publisher.publish(visualization)
        
    rate.sleep()


if __name__ == '__main__':
    listen_for_magnetic_vector()
    try:
        publish_visualization_of_magnetic_vector()
    except rospy.ROSinterruptException:
        pass
