# Vision-Guided Industrial Defect Sorting System

A ROS 2 based industrial defect sorting simulation using **Gazebo Sim, OpenCV, UR5 and Robotiq 2F-85**.

The system simulates an automated industrial production line in which products are transported along a conveyor belt, inspected using an RGB camera, classified by colour, and automatically handled by a robotic arm.

Red and green products are treated as normal products and pass through the production line. Blue products are classified as defective, stopped at a calibrated pick position, picked by the UR5 robot, and placed into a reject bin.

