# ros_em7180

A simple python package to use em7180 IMU in ROS. Tested on Raspberry Pi 3B and Odroid XU4 with ROS Kinec.

# Visualization

Data from the em7180 can viewed the provided listener.py subscriber. Future updates will provide graphs and 3d visualization. For now the listener GUI simply displays the raw data.

# Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/vortexntnu/ros_em7180.git
    cd ~/catkin_ws
    catkin_make

# Usage

Python SMbus requires root access. It may therefore be required to run it as root:

    sudo su
You could however consider adding the user to the I2C usergroup to avoid running the package as root.

    #To run the publisher:
    rosrun ros_em7180 mastertest.py
    
    #To run the listener:
    rosrun ros_em7180 listener.py
    
    
# Documentation

publisherEM7180: 
The __init__.py and parts of mastertest.py are based off of the github repository https://github.com/simondlevy/EM7180 to get the desired sensor data from the EM7180. This mastertest.py has now been updated to publish data to ROS where a custom message including sensor_msgs/IMU type is used for roll, pitch, yaw, accelerationX, accelerationY, accelerationZ (body coordinates), angularVelX, angularVely, angularVelz. Temperature, pressure and altidude has been implemented with sensor_msgs/Temperature, sensor_msgs/FluidPressure and a Float64 for altitude inside this custom message. 

subscriberEM7180:
The listener.py displays the data received by subscribing to the topic /SensorData published by EM7180Publisher. The code is meant to serve as an example on how to fetch data from the topic. It's also useful for debugging purposes as it displays the data and visualizes the position in 3D space.






# Credits
The package uses the python scripts provided by simondlevy's repository: https://github.com/simondlevy/EM7180
