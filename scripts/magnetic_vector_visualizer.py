#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def callback(data):
    visualization = Marker()

    visualization.header.frame_id = "magnetometer_link"
    visualization.header.stamp = rospy.Time.now()
    visualization.ns = "my_namespace"
    visualization.id = 0
    visualization.type = visualization.ARROW
    visualization.action = visualization.ADD
    visualization.pose.position.x = 0
    visualization.pose.position.y = 0
    visualization.pose.position.z = 0

    p1 = Point()
    p2 = Point()

    p1.x = 0
    p1.y = 0
    p1.z = 0
    p2.x = data.magnetic_field.x
    p2.y = data.magnetic_field.y
    p2.z = data.magnetic_field.z

    visualization.points.append(p1)
    visualization.points.append(p2)

    visualization.pose.orientation.x = 0
    visualization.pose.orientation.y = 0
    visualization.pose.orientation.z = 0
    visualization.pose.orientation.w = 1
    visualization.scale.x = 0.2
    visualization.scale.y = 0.3
    visualization.scale.z = 0.2

    visualization.color.a = 1

    visualization_publisher.publish(visualization)


def publish_visualization_of_magnetic_vector():
    global visualization_publisher

    rospy.init_node('magnetic_vector_visualization')
    visualization_publisher = rospy.Publisher('em7180/vis', Marker, queue_size=10)
    rospy.Subscriber('imu/mag', MagneticField, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        publish_visualization_of_magnetic_vector()
    except rospy.ROSinterruptException:
        pass
