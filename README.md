# Agricultural Plant Watering Robot

Welcome to the Agricultural Plant Watering Robot repository! This project aims to create a mobile robot capable of autonomously watering potted plants in an agricultural setting. The robot utilizes a comprehensive and integrated ROS-based framework, addressing key aspects of perception, control, actuation, and safety, while also considering user interaction and feedback mechanisms.

# Overview
## The robot's operation can be divided into several key steps:

## Initial Dry Run:
The robot acquaints itself with its surroundings, mapping obstacles and the locations of potted plants.

## User Control:
Users can maneuver the robot using an accompanying app and direct it to specific areas of operation.

## Mapping:
The robot gracefully navigates within its designated map, tending to the watering needs of the potted plants.

## Plant Detection:
Using image processing techniques, the robot detects potted plants and approaches them.

## Soil Moisture Measurement:
The robot's arm probes the soil moisture of each plant, ensuring precise measurement.

## Automated Watering:
Based on the moisture reading, the plant receives tailored watering.

## Navigation and Obstacle Avoidance:
Sophisticated SLAM techniques are employed to elegantly circumvent obstacles.

## Water and Battery Management:
The robot returns home to replenish its water supply and recharge its battery as needed.

## User Interaction:
Users can mark new potted plants, define restricted regions, and set default exceptions.

# ROS Nodes
## The robot's control system is composed of various ROS nodes:

## Image Processing Node:
Captures real-time data from the Raspberry Pi camera and processes it to locate plant pots. Data is communicated via the dir_values topic.

## PID Calculation Node: 
Computes PID control values for precise robot navigation based on plant pot position.

## PWM Publisher Node: 
Bridges high-level navigation commands with low-level motor control actions for the robot's wheels.

## Active Collision Node: 
Analyzes LiDAR scan data to detect obstacles and communicates this information via the obs_detected topic.

## Interrupt Manager Node: 
Handles concurrent processes and prioritizes them based on real-time data from various sources.

## Arm Controller Node: 
Controls the robot's arm, including stepper motors, limit switches, and moisture sensors for safety and accuracy.

## Pump Node: 
Regulates the plant watering process based on real-time moisture data and water levels.

## Moisture Sensor Node: 
Monitors soil moisture levels and publishes data to the moisture_level topic.

## Load Cell Node: 
Manages the measurement of water levels in the robot's reservoir, publishing data to the water_level topic.

## LED Controller Node: 
Provides system condition feedback through LED indicators.

# Getting Started
To use this repository and deploy the robot, follow the instructions in the respective folders of each ROS node. Ensure you have ROS noetic installed and configured on your system. Make sure that you have
- Ubuntu 20 installed and configured on the Raspberry Pi
- ROS (Robot Operating System) installed on the Raspberry Pi
- [RPi.GPIO library](https://pypi.org/project/RPi.GPIO/) (Install it if not installed by default)

# Acknowledgments
Credit to the creators of ROS and the RPi.GPIO library.   
We would like to express our gratitude to Anya Robotics Pvt Ltd for their contributions or support.
