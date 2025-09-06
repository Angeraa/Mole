## The Mole

The Mole is my introduction to applying ROS in the real world at home. It will serve as my base platform to do more exciting things without having to re-implement anything. The Mole will be using ROS2 for SLAM and navigation with the goal of being able to use waypoints to direct the robot around a map which it will traverse autonomously with Nav2.

The robot will be powered by a raspberry pi for wireless communication and hosting several nodes and connects to any sensors. It will then communicate to an esp32 via my hardware interface over serial which controls the motors and any servos I add in the future. The raspberry pi will communicate over wifi to my laptop which will host the computationally expensive nodes like the SLAM and Nav2 nodes.

Currently working on: Hardware interface and esp32 code

### Gazebo Simulation:
Gazebo will be used to simulate the robot as I cannot purchase a 2d lidar right now.
<img width="1049" height="757" alt="image" src="https://github.com/user-attachments/assets/e2f339c3-f9de-41cb-85ca-1e7bc0274fd8" />

### Rvis Visualization:
<img width="1049" height="757" alt="image" src="https://github.com/user-attachments/assets/a272afa5-7912-4def-9225-84e06d250f75" />

