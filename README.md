Project Description:
This project implements autonomous and manual obstacle avoidance navigation for a TurtleBot3 robot in a Webots simulation environment. The robot uses LiDAR, GPS, a compass, and a camera to navigate through predefined target points while avoiding obstacles. It also includes logging functionality for 3D mapping using LiDAR point clouds and path tracking.

Project Objectives:
Enable autonomous obstacle avoidance and target navigation using sensor data.

Implement manual override control via keyboard inputs.

Integrate LiDAR-based dynamic obstacle detection and camera-based environmental perception.

Log robot paths and 3D point cloud data for mapping purposes.

Algorithm/Model Development:
Autopilot Mode: Calculates the robot’s orientation and distance to the target, adjusts wheel speeds accordingly, and uses a Braitenberg-like reactive system for obstacle avoidance.

Obstacle Avoidance: Uses Gaussian-weighted influence from LiDAR data to steer away from nearby obstacles.

Manual Mode: Allows direct keyboard control for exploration.

Path Logging: Records LiDAR point cloud and robot GPS coordinates into .csv files.

Camera Analysis: Periodic RGB sampling at the center of the image to observe surroundings.

GPS and compass modules

Raspberry Pi or OpenCR board

5. Software Tools:
Webots: For 3D simulation and robot control

C Programming Language: For writing the controller logic

CSV Logging: For point cloud and path data output

GNU Compiler: To compile the robot controller


Output Screenshots:

![pic1](https://github.com/user-attachments/assets/33ca75a8-7d26-46af-a447-aac1d75ca31e)

![pic1](https://github.com/user-attachments/assets/a1edcf1c-733f-406e-8743-59d627fc49b6)




