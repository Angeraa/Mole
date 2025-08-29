## The Mole

The Mole is my introduction to applying ROS in the real world at home. It will serve as my base platform to do more exciting things without having to re-implement anything. The Mole will be using ROS2 for SLAM and navigation with the goal of being able to use waypoints to direct the robot around a map which it will traverse autonomously with Nav2.

Gazebo will be used to simulate the robot as I cannot purchase a 2d lidar right now. When I can, I plan to begin building the robot and working on the hardware interface.

The robot will be powered by a raspberry pi for wireless communication and hosting several nodes and connects to any sensors. It will then communicate to an esp32 via the hardware interface over serial which controls the motors and any servos I add in the future. The raspberry pi will communicate over wifi to my laptop which will host the computationally expensive nodes like the SLAM and Nav2 nodes.

### Rvis Visualization:
<img width="1102" height="853" alt="image" src="https://github.com/user-attachments/assets/15d3c864-c426-46ee-9574-eef20e15d825" />

### Gazebo Simulation (Different instance than the Rvis screenshot):
<img width="1435" height="1083" alt="lidar_sim" src="https://github.com/user-attachments/assets/675c8bc4-e4a2-4f3e-b8a6-ab764e2097aa" />
