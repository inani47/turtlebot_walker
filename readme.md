# TURTLEBOT WALKER


## Overview

This package modifies the default Turtlebot simulation and implements a simple walker algorithm. The robot moves forward until it reaches an obstacle, then rotates in place (without colliding) until the way ahead is clear, then move forward again and repeat. It records all messages being published on all topics except the topics related to the RGB-D sensor in a bagfile. 

## License

This software is released under the BSD-2 License, see [LICENSE.txt](LICENSE.txt).

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu (Wily/Xenial) or Debian (Jessie)
* ROS Kinetic
* Gazebo 7.x (part of ros-kinetic-desktop-full package)
* Turtlebot simulation stack

To instal ROS, use this [link](http://wiki.ros.org/kinetic/Installation)

To install turtlebot simulation stack. In a terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
source /opt/ros/kinetic/setup.bash
```



## Build Instructions
If you do not have a catkin workspace, follow this:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/walker.git
cd ..
catkin_make
```
If you wish to run this code in an existing catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/walker.git
cd ..
catkin_make
```


## Run Instructions Using Launch File

After following the build instructions:

To run the simulation, In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch walker walker.launch 
```
To enable bag file recording of all topics except camera. In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch walker walker.launch rec:=1
```

The bag file will be stored in the results folder as recording.bag

## Inspecting and Replaying Bag Files

To inspect the bag file go the folder where bag file was saved. In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/walker/results
rosbag info recording.bag
```

To replay the bag file, first run rosmaster from the terminal:
```
roscore
```
Now, from the results folder run the following command in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/walker/results
rosbag play recording.bag
```







