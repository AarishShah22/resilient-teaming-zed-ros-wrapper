# ROS-Driver
# Quick Start Guide
```
 Note*  
 -This ROS driver only supports firmware version 2.0 or 2.0+. 
 -You can check your firmware version from Roborun+ console tab by querying - "?fid". 
 -If firmware is not the latest one then please update it with the latest one available on Roboteq website 
  or contact "techsupport.roboteq@mail.nidec.com".
```
This file outlines the instructions for running the current development of the mobile robot platform. This repository contains the ROS driver for Roboteq controllers and Robot controller. The package requires ROS Melodic to be installed properly to your system and proper serial connection of Roboteq controller. 

First, clone this repository and the serial repository to catkin_ws/src:
```
git clone https://gitlab.com/barton-research-group/resilient-teaming/ros-driver.git
git clone https://github.com/wjwwood/serial.git 
```

To unlock USB terminal connected to the motor driver, run command:

```
sudo chmod 666 /dev/ttyACM0
```

Program currently allows user to enter speed duty cycle commands into the terminal in the form 'rpm%_Motor1 rpm%_Motor2'
Example for 50% of max rpm input for both wheels: '500 500'
Max rpm of this robot is 74rpm

The `Roboteq motor controller driver` is the package for the Roboteq ROS-driver. Make sure not to change package name as it will change the definition in the Cmake and Package files. Open new terminal and copy these steps -

```
cd catkin_ws/
catkin_make
source devel/setup.bash
```
Within terminal 1:
```
roslaunch roboteq_motor_controller_driver driver.launch
```
Within terminal 2:
```
rosrun roboteq_motor_controller_driver keyBoardReader_node
```
(Optional) Currently odometry is not integrated with the main program. To view program performance, run the following in terminal 3:
```
rosrun roboteq_motor_controller_driver diff_odom.launch
```


The roboteq driver is designed to be dynamic and users can publish the controller queries as per their requirements. The publishing queries is not limited to any value. By default total 9 queries are published by launching this driver. Users can change or add queries in configuration file. For that go to config/query.yaml

```
frequencyH : 50   #higher frequency (value is in ms)
frequencyL : 100  #lower frequency
frequencyG : 100  #frequency for general queries

queryH:
 motor_amps : ?A 
 motor_command : ?M
 encoder_count : ?C
 encoder_speed : ?S  #these queries will publish with higher frequency and users can add other queries below encoder_speed.

queryL: 
 error : ?E 
 feedback : ?F 
 battery_amps : ?BA
 power : ?P          #these queries will publish with lower frequency and users can add other queries below power.
 

queryG: 
 fault_flag : ?FF  
# status_flag : ?FS
# firmware_id : ?FID  Users can add queries which do not require channel number under queryG tab. 
```
