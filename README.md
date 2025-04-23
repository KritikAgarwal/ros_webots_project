📌 Project Description:
 This project implements both autonomous and manual navigation for a TurtleBot3 robot in the Webots simulation environment. The robot navigates through predefined target points while avoiding obstacles using various sensors including LiDAR, GPS, compass, and a camera. It also supports logging functionality for 3D mapping using LiDAR point clouds and path tracking.

 
🎯 Project Objectives :
•	Enable autonomous obstacle avoidance and target navigation using real-time sensor data.
•	Implement manual override control via keyboard inputs.
•	Integrate LiDAR-based dynamic obstacle detection and camera-based environmental perception.
•	Log robot paths and 3D point cloud data for mapping and analysis.


🧠 Algorithm & Model Development Autopilot Mode:
•	Calculates the robot’s orientation and distance to the target. Adjusts wheel speeds using a Braitenberg-like reactive system for real-time obstacle avoidance.
•	Obstacle Avoidance: Employs Gaussian-weighted influence derived from LiDAR data to steer the robot away from nearby obstacles.
•	Manual Mode: Allows keyboard-controlled navigation for free exploration.
•	Path Logging: Logs GPS coordinates and LiDAR point cloud data into .csv files for analysis.
•	Camera Analysis: Periodic RGB sampling from the center of the image to gather environmental context.


🧩 Hardware Requirements GPS and Compass Modules
•	LiDAR Sensor
•	Raspberry Pi or OpenCR Board

🛠️ Software Tools Webots: 3D simulation and robot control
•	C Programming Language: Controller logic implementation
•	GNU Compiler (GCC): For compiling the control code
•	CSV Logging: For storing point cloud and path data

📸Output Screenshots:
![WhatsApp Image 2025-04-23 at 20 13 10_5e775fb2](https://github.com/user-attachments/assets/f76bcf8c-db4f-4a75-a70e-3d4d3bb6c980)

![WhatsApp Image 2025-04-23 at 20 14 42_8966ab07](https://github.com/user-attachments/assets/852f63b9-69e6-426a-9570-b5401e61cccb)
