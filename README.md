# em7180

A simple python package to use em7180 IMU in ROS. Tested on Raspberry Pi 3B and Odroid XU4 with ROS Kinec.

# Visualization

Data from the em7180 can be viewed with the provided subscriberEM7180.py subscriber. The provided subscriber displays the data read from the sensor and visualizes its positioning in 3D space.

# Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/vortexntnu/em7180.git
    cd ~/catkin_ws
    catkin_make

# Usage

Python SMbus requires root access. It may therefore be required to run it as root:

    sudo su
You could however consider adding the user to the I2C usergroup to avoid running the package as root.

    #To run the publisher:
    rosrun em7180 publisherEM7180.py
    
    #To run the listener:

    rosrun em7180 subscriberEM7180.py

    
    
# Documentation

publisherEM7180: 
The __init__.py and parts of publisherEM7180.py are based off of the github repository https://github.com/simondlevy/EM7180 to get the desired sensor data from the EM7180. This mastertest.py has now been updated to publish data to ROS where a custom message including sensor_msgs/IMU type is used for roll, pitch, yaw, accelerationX, accelerationY, accelerationZ (body coordinates), angularVelX, angularVely, angularVelz. Temperature, pressure and altidude has been implemented with sensor_msgs/Temperature, sensor_msgs/FluidPressure and a Float64 for altitude inside this custom message. 


subscriberEM7180:
The listener.py displays the data received by subscribing to the topic /SensorData published by EM7180Publisher. The code is meant to serve as an example on how to fetch data from the topic. It's also useful for debugging purposes as it displays the data and visualizes the position in 3D space.

## Angles
This project uses Tait-Bryan angles, commonly used in aircraft orientation.  In this coordinate system, the positive z-axis is down toward Earth.  Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.  Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.  Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.  These arise from the definition of the homogeneous rotation matrix constructed from q.  Tait-Bryan angles as well as Euler angles are non-commutative that is, the get the correct orientation the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.  For more information see this [Wikipedia article](http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles) which has additional links.


# Credits
The package uses some python scripts provided by simondlevy's repository: https://github.com/simondlevy/EM7180

