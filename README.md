# autonomous_drivin

## Introduction
This repository is about the robot we made for the WRO Future Engineers competition in the season 2023. Here, we are going to explain in detail how the robot operates and we will also go through the general structure of our code.

## Mechanical parts
The vehicle itself is mostly consisted out of LEGOs, but some parts had to be 3D printed, in order for us to connect the sensors and the motors onto the robot. 
### Sensors
We used:
- 2 distance sensors
- 1 color sensor
- 1 gyroscope
- 2 encoders
- 1 camera
### Motors
We used one motor which moves both back weels of our vehicle with the help of a differential. We also used a servo motor which is responsible for the steering process of our robot.

## Code
The general structure of our code is not that complicated. What we did is that instead of writing one big script, we wrote multiple ones for each operation our robot had to complete. We managed to do that with the help of ROS. ROS stands for Robot Operating System and it is a set of software libraries and tools that help you build robot applications. The key feature of ROS is the way the software is run and the way it communicates. So we wrote python scripts called "nodes" which run at the same time and give valuable information to the main node. The main node is different for each challenge of the competition. For the first one the node is run1.py and for the second one is run2.py. During each run, we have different nodes runing. The nodes required for each challenge run automatically with a launch file, when the system boots.
### Run1
In this run, the launch file runs the following nodes:
- dc.py
- distances.py
- gyro.py
- run1.py




