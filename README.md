# CSDS 373 Lab 3: STDR Smart Simulator
## By Kris Zhao (kxz167), Sanhita Kumari (sxk1409), Michael Koltisko (mek169)

### Development:
Developement was done in Ros Melodic for Ubuntu 18.04 natively on hardware / virtualbox with Ros Melodic for Ubuntu.

### Information:
Package name:
```
team2_smart_teleoperation
```
#### Package directory:
- csds373_f21_team2_smart_teleoperation (from github)
    - launch
        - launch_st_publisher.launch
        - launch_stp_stdr.launch
    - src
        - teleoperation_publisher.cpp
    - CMakeLists.txt
    - package.xml
    - README.md

### Running:
Confirm ROS is available in the shell:
```
source /opt/ros/melodic/setup.bash
```
#### Install STDR Simulator requirements
```
git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git
```
```
sudo apt-get install libqt4-dev ros-melodic-map-server
```
```
sudo -- /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install
```

Download the project to:
```
catkin_ws/src/
```
Using:
```
git clone git@github.com:cwru-courses/csds373_f21_team2_smart_teleoperation.git
```
From the `catkin_ws` directory, build the package (I just build all packages):
```
catkin_make
```
Source the devel:
```
source devel/setup.bash
```
#### Running our smart interceptor:
Running the smart teleopoeration node without the simulator:
```
roslaunch team2_smart_teleoperation launch_st_publisher.launch [options]
```
Options:
- `ns:=<namespace_for_the_robot[default=robot0]>` - Let's you define the namespace for which the smart teleoperation node will be operating on (which robot).

Running the smart teleoperation node in addition to the simulator:
```
roslaunch team2_smart_teleoperation launch_stp_stdr.launch [options]
```
Options:
- `ns:=<namespace_for_the_robot[default=robot0]>` - Let's you define the namespace for which the smart teleoperation node will be operating on (which robot).

By defining the name space for the smart teleoperation nodes, you are able to control multiple robots on one simulation.
#### Setup of robots:
Similar to the assignment:
- In the case of robots being unresponsive, verify topic naming is aligned with the assignment.