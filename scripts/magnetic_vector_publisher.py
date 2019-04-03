#!/usr/bin/env python
from em7180 import EM7180_Master
import rospy
from sensor_msgs.msg import MagneticField

MAG_RATE = 100  # Hz
ACCEL_RATE = 200  # Hz
GYRO_RATE = 200  # Hz
BARO_RATE = 50  # Hz
Q_RATE_DIVISOR = 3  # 1/3 gyro rate

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

    def publish_imu_sensor_data(mag_x, mag_y, mag_z):
        rospy.init_node('em7180')

        magnetic_field_publisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Initialize objects

            magnetic_vector = MagneticField()

            magnetic_vector.header.stamp = rospy.Time.now()
            magnetic_vector.header.frame_id = "magnetometer_link"

            magnetic_vector.magnetic_field.x = mag_x
            magnetic_vector.magnetic_field.y = mag_y
            magnetic_vector.magnetic_field.z = mag_z

            magnetic_vector.magnetic_field_covariance = [700, 0, 0, 0, 700, 0, 0, 0, 700]

            magnetic_field_publisher.publish(magnetic_vector)

            rate.sleep()


    if em7180.gotMagnetometer():
        mx, my, mz = em7180.readMagnetometer()

    try:
        publish_imu_sensor_data(mx, my, mz)
    except rospy.ROSInterruptException:
        pass
