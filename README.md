# autonomous_driving

## Introduction
This repository is about the robot we made for the WRO Future Engineers competition in the 2023 season. Here, we are going to explain in detail how the robot operates and we will also go through the general structure of our code.

## Mechanical parts
The vehicle itself is mostly consisted out of LEGOs, but some parts had to be 3D printed, in order for us to connect the sensors and the motors onto the robot. 
### Sensors
We used:
- 2 distance sensors
- 1 color sensor
- 1 gyroscope
- 3 encoders
- 1 camera
### Motors
We used one motor which moves both rear weels of our vehicle with the help of a differential. We also used a servo motor which is responsible for the steering of our robot.

## Code
The general structure of our code is not that complicated. What we did is that instead of writing one big script, we wrote multiple ones for each operation our robot had to complete. We managed to do that with the help of ROS. ROS stands for Robot Operating System and it is a set of software libraries and tools that help you build robot applications. The key feature of ROS is the way the software is run and the way it communicates. So we wrote python scripts called "nodes" which run at the same time and give valuable information to the main node. The main node is different for each challenge of the competition. For the first 
one the node is run1.py and for the second one is run2.py. During each run, we have different nodes runing. The nodes required for each challenge run automatically with a launch file, when the system boots up.

### First mission
For this mission, the launch file runs the following nodes:
- dc.py, which controls the motor and reads the encoders
- distances.py which handles interfacing with the distance sensors
- gyro.py which communicates with the gyroscope

#### Basic algorythm for the first mission
When the first mission  starts, the robot vehicle starts going straight. In order to do that we have the gyro node which uses the gyroscope to calculate the deviation from the target and then the run1 node corrects the servo motor according to that information so that he robot is always going straight. If the distance sensors detect a big gap on the right or on the left of the vehicle, the target changes accordingly so that the robot turns 90 degrees to continue the round. This happens 12 more times which is 3 rounds. When the vehicle detects a gap for the 13th time, it starts going backwards in order to park where it started.

Video of the robot completing the first mission can be found here: https://youtu.be/2mD64tW20yI

### Second mission 
This run has the same nodes runing, with some additional ones:
- pillar.py which detects traffic signs to be avoided using the camera
- lines.py which check the lines to change direction

  
### Basic algorythm for the second mission
The algorythm for the second mission is the same as the one for the first on, but here we also have to avoid red and green traffic signs. Using image processing library OpenCV, we are able to detect traffic signs in the camera feed. Additional processing of the image provides us with further information about the pillar, such as its color and its distance from our vehicle. Using different parameters we are able to identify which traffic signs we need to focus on avoiding. For example, traffic signs which are far away, or already on the correct side of the vehicle can be safely ignored. If a traffic sign is on the wrong side of the vehicle, the robot starts steering to avoid it. The steering angle is determined by how much the vehicle has to move to pass it from the correct side. The robot keeps record of all the traffic signs it has seen, so at the end of the second round it checks whether the last traffic sing it encountered was red, so that it can turn back to go through the last round the opposite direction. Otherwise, it keeps its direction the same and finishes normally.

Video of the robot completing the second mission can be found here: https://youtu.be/rZwB47i2jOo

## How to start the robot
It is not that dificult to start the vehicle and we can easily describe the proccess to do so. First of all, we have to turn on the robot and connetct to it with the help of the Visual Studio extension that allows us to change remotelly the contents of our scripts. That way, we can change the launch file that runs on boot to be either for the first or the second mission. After that, we can shut down the system and turn off the power on our vehicle. Then we can only turn it on after we have placed it at the starting position. At this stage, it is crucial that we do not move the robot because the gyroscope will not be able to callibrate properly. If everything is done correctly, about 40 to 45 seconds after the boot a green light will indicate that we are ready to start. So all we have to do now, is to push the button in order for the vehicle to start moving forward.







