#!/usr/bin/env python
from em7180 import EM7180_Master
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped
import tf
import rospy

MAG_RATE = 100  # Hz
ACCEL_RATE = 200  # Hz
GYRO_RATE = 200  # Hz
BARO_RATE = 50  # Hz
Q_RATE_DIVISOR = 3  # 1/3 gyro rate


def main():
    rospy.init_node('em7180')
    publisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    # transformed_publisher = rospy.Publisher('imu/magTransformed', Vector3Stamped, queue_size=10)

    rate = rospy.Rate(10)

    em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR)

    listener = tf.TransformListener()
    listener.waitForTransform("em7180_link", "imu_0", rospy.Time(0), rospy.Duration(1.0))

    # Start the EM7180 in master mode
    if not em7180.begin():
        print(em7180.getErrorString())
        exit(1)

    while not rospy.is_shutdown():

        em7180.checkEventStatus()
        if em7180.gotError():
            print('ERROR: ' + em7180.getErrorString())
            exit(1)

        if em7180.gotMagnetometer():
            mx, my, mz = em7180.readMagnetometer()

            magnetic_vector = Vector3Stamped()
            magnetic_vector.header.stamp = rospy.Time.now()
            magnetic_vector.header.frame_id = "em7180_link"

            magnetic_vector.vector.x = mx
            magnetic_vector.vector.y = my
            magnetic_vector.vector.z = mz

            magnetic_vector_transformed = tf.transformVector3("imu_0", magnetic_vector)

            magnetic_field_msg = MagneticField()
            magnetic_field_msg.header = magnetic_vector_transformed.header
            magnetic_field_msg.magnetic_field = magnetic_vector_transformed.vector

            magnetic_field_msg.magnetic_field_covariance = [700, 0, 0, 0, 700, 0, 0, 0, 700]

            publisher.publish(magnetic_field_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
