## Requirements
** Install ubuntu 18.04

** Ros melodic - install via: http://wiki.ros.org/melodic/Installation/Ubuntu.

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


# Vision node setup
## CUDA setup

1. Install the newest NVIDIA driver via runtime - this can be troublesome - ask Jacob if there is problems

2. Download CUDA toolkit 10.2
3. Install CUDA toolkit without driver
4. Download CUDNN for CUDA 10.2
5. Put CUDNN into /usr/local/cuda

6. Læg følgende ind i bashrc:
```
# NVIDIA CUDA Toolkit
export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64
```

If the last line interrupts your python, try deleting it.

## YoloV5
1. cd /P4/src/vision/src/YoloV5
2. sudo pip install -r requirements.txt
3. pip3 install rospkg

## Pytorch
1. Install the version of pytorch that is available for your version of CUDA

For CUDA 10.2
```
pip install torch==1.8.0 torchvision==0.9.0 torchaudio==0.8.0
```

## Start vision node 
```
rosrun vision pick.py --source 0 --weights '../../weights/Glass350E.pt' --save-txt --name John --conf 0.4
```
