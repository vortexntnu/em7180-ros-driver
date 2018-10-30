# ros_em7180

A simple python package to use em7180 IMU in ROS. Tested on Raspberry Pi 3B and Odroid XU4 with ROS Kinect.


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

# Credits
The package uses the python scripts provided by simondlevy's repository: https://github.com/simondlevy/EM7180
