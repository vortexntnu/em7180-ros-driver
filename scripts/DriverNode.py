#!/usr/bin/env python


from sensor_msgs.msg import MagneticField
from em7180.msg import Ximu
from em7180 import EM7180_Master
import math
import rospy




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

    def publishIMUSensorData(pitch, roll, yaw, angVelx, angVely, angVelz, linAccx, linAccy, linAccz, temp, press, alt,
                             magX, magY, magZ):

        # Initialize node
        rospy.init_node('em7180', anonymous=False)

        # Publisher
        imuSensorPublisher = rospy.Publisher('sensors/imus/em7180', Ximu, queue_size=10)
        magneticFieldPublisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

        rate = rospy.Rate(10)

        # Initilize objects

        magneticVector = MagneticField()

        theXimu = Ximu()

        # Make a struct that contains the IMU data
        imuMsg = theXimu.imu

        # Make a struct that contains the Temperature data
        tempMsg = theXimu.temperature

        # Make a struct that contains the Pressure data
        pressMsg = theXimu.pressure

        # Make a struct that contains the Altitude data
        altMsg = theXimu.altitude

        # Set Temperature variables
        tempMsg.header.stamp = rospy.Time.now()

        tempMsg.temperature = temp
        tempMsg.variance = 0

        # Set Pressure variables
        pressMsg.header.stamp = rospy.Time.now()

        pressMsg.fluid_pressure = press
        pressMsg.variance = 0

        # Set Altitude variables
        altMsg = alt

        # Covariance matricies

        # imuMsg.orientation_covariance=[0,0,0,0,0,0,0,0,0] # Place in the covariance matrix here

        # imuMsg.angular_velocity_covariance=[0,0,0,0,0,0,0,0,0] # Place in the covariance matrix here

        # imuMsg.linear_acceleration_covariance=[0,0,0,0,0,0,0,0,0] # Same here

        imuMsg.orientation_covariance[0] = -1
        imuMsg.angular_velocity_covariance[0] = -1
        imuMsg.linear_acceleration_covariance[0] = -1

        # Place sensor data from IMU to message

        imuMsg.header.stamp = rospy.Time.now()

        imuMsg.linear_acceleration.x = linAccx
        imuMsg.linear_acceleration.y = linAccy
        imuMsg.linear_acceleration.z = linAccz

        imuMsg.angular_velocity.x = angVelx
        imuMsg.angular_velocity.y = angVely
        imuMsg.angular_velocity.z = angVelz

        imuMsg.orientation.x = roll
        imuMsg.orientation.y = pitch
        imuMsg.orientation.z = yaw

        # Compile custom message
        theXimu.imu = imuMsg
        theXimu.temperature = tempMsg
        theXimu.pressure = pressMsg
        theXimu.altitude = altMsg

        # Magnetic field vector

        magneticVector.header.stamp = rospy.Time.now()
        magneticVector.header.frame_id = "magnetometer_link"

        magneticVector.magnetic_field.x = magX
        magneticVector.magnetic_field.y = magY
        magneticVector.magnetic_field.z = magZ

        magneticVector.magnetic_field_covariance = [700, 0, 0, 0, 700, 0, 0, 0, 700]

        imuSensorPublisher.publish(theXimu)
        magneticFieldPublisher.publish(magneticVector)

        # Info to ros_console and screen
        rospy.loginfo("Publishing sensor data from IMU")

        # Sleep in order to maintain the rate
        rate.sleep()


    if (em7180.gotQuaternion()):

        qw, qx, qy, qz = em7180.readQuaternion()

        roll = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        pitch = -math.asin(2.0 * (qx * qz - qw * qy))
        yaw = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)

        pitch *= 180.0 / math.pi
        yaw *= 180.0 / math.pi
        yaw += 13.8
        if yaw < 0: yaw += 360.0
        roll *= 180.0 / math.pi


    if em7180.gotAccelerometer():
        ax, ay, az = em7180.readAccelerometer()


    if em7180.gotGyrometer():
        gx, gy, gz = em7180.readGyrometer()


    if em7180.gotBarometer():
        pressure, temperature = em7180.readBarometer()

        altitude = (1.0 - math.pow(pressure / 1013.25, 0.190295)) * 44330

    if em7180.gotMagnetometer():
        mx, my, mz = em7180.readMagnetometer()

    if __name__ == '__main__':
        try:
            publishIMUSensorData(pitch, roll, yaw, gx, gy, gz, ax, ay, az, temperature, pressure, altitude, mx, my, mz)
        except rospy.ROSInterruptException:
            pass

