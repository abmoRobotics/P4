## Requirements
** Install ubuntu 18.04

**Ros melodic - install via: http://wiki.ros.org/melodic/Installation/Ubuntu.

** Install moveit: sudo apt install ros-melodic-moveit
## Downloading the project
**git clone https://github.com/abmoRobotics/p4.git**
## Updating submodules
To run the code, the required external packages must be downloaded, this is done by following the steps listed below:
1. Navigate to the project folder(p4) folder (cd p4)
2. Run: git submodule update --init --recursive
3. Run: source /opt/ros/melodic/setup.bash
4. Run : catkin_make
  - If catkin_make fails run the following command: source devel/setup.bash
  - Run: catkin_make
5. sudo apt-get install ros-melodic-trac-ik-kinematics-plugin
6. Because the gazebo needs controllers run the follwing command: sudo apt-get install ros*controller*


# Usb permission for iTongue
sudo chmod 777 /dev/ttyUSB0
