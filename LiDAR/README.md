# ROS Obstacle Avoidance with LaserScan

This Python script is designed for obstacle avoidance using LaserScan data in a ROS (Robot Operating System) environment. It reads LaserScan data and, if it detects an obstacle within a specified range, it stops the robot to avoid a collision.

## Prerequisites

Before using this script, ensure you have the following:

- Python
- ROS (Robot Operating System)

## Usage

1. Run the ROS master: `roscore`.
2. Run the script using the `rosrun` command, assuming you have the necessary permissions:
```bash
rosrun <package_name> Obstacle_Avoidance.py
```

1. The script initializes the ROS node and sets up a publisher for controlling the robot's velocity (/cmd_vel) and a subscriber for receiving LaserScan data (/scan).
2. It monitors the LaserScan data, specifically checking for obstacle distances within 30 cm (adjustable as needed). If it detects an obstacle within this range, it stops the robot by setting the linear and angular velocities to zero.
3. The script continuously listens for LaserScan data and performs obstacle avoidance by stopping the robot whenever necessary.

## Node Information
Node Name:  
`obstacle_avoidance`  

Publishers:  
`/cmd_vel (for robot velocity control)`  

Subscribers:     
`/scan (for LaserScan data)`

## Acknowledgments
This script is a basic example of obstacle avoidance in a ROS environment using LaserScan data. Credit to the creators of ROS for providing the tools and libraries for robotics development. We would like to express our gratitude to Anya Robotics Pvt Ltd for their contributions or support.
