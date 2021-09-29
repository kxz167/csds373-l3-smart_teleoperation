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
Running the node:
```
roslaunch team2_smart_teleoperation  [options]
```
NOTE: depending on other package names from other students, pacakge direction may not work properly. Make sure to clear out other packages named navvis_description. In the future I will make sure and add my student ID to the package name.\
\
Options:
- `use_xacro:= [true/false]` - Use the xacro file (true) or the urdf file (false : default)
- `jsp_gui:= [true/false]` - Utilize the Joint State Processor Gui to command wheels (true : default) or "render" vehicles through single joint msg (false)

### Notes:
- For the full functionality and to see the final robot, the `use_xacro:=true` flag should be used. Utilizing the old URDF file will not show the completed robot.
- You will need to rename the downloaded folder to navvis_descriptions in order to match the directory naming scheme due to github package naming conventions. In terms of running, this does not seem to impact running the code.