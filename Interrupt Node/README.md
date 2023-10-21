# Robot Control with PID Positioning

This project is a Python script that controls a robot's movement using a PID (Proportional-Integral-Derivative) controller. The script subscribes to messages from sensors, adjusts the robot's behavior based on those inputs, and publishes velocity commands to control the robot's movement.

## Prerequisites

Before using this script, ensure that you have the following installed:

- Python
- ROS (Robot Operating System)
- `rospy` - ROS Python library
- `std_msgs` - ROS standard messages library
- `geometry_msgs` - ROS geometry messages library

## Configuration

The script can be configured using the following parameters:

- `Kp`, `Ki`, and `Kd`: PID control constants
- `SetPoint`: The target value for the PID controller
- `windup_guard`: A limit to prevent integral windup
- `initial_vel`: The initial velocity of the robot
- Other parameters for specific sensor inputs and control logic

## Usage

1. Run the ROS master: `roscore`.
2. Run the `pump_cont` script using the `rosrun` command, assuming you have the necessary permissions:
```bash
rosrun <package_name> interrupt.py
```
The script will subscribe to sensor data (pot position from the open cv node and depth from the obstacle avoidance node) and control the robot's movement based on the PID controller and sensor readings.

The robot's behavior may include stopping, turning, or moving forward based on the detected "pot position."

## Node Information
Node Name:  
pot_runner  

Subscribers:  
`pot_pos (topic for pot position)`  
`depth (topic for depth sensor)`  

Publishers:  
`cmd_vel (topic for velocity commands)`

## Acknowledgments
This script uses a PID controller to control the robot's movement. Credit to the creators of the PID control algorithm.
