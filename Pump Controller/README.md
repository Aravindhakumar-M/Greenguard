# Pump Controller

This Python script is designed to control water pumps (inlet and outlet) based on sensor readings. It utilizes the Raspberry Pi GPIO pins for controlling MOSFETs that, in turn, control the pumps.

## Prerequisites

Before running the script, ensure that you have the following:

- Raspberry Pi set up with RPi.GPIO library.
- ROS (Robot Operating System) installed if you are using ROS topics for sensor data.

## Hardware Setup

- Connect the inlet pump to the MOSFET controlled by `MOSFET_IN` (pin 18).
- Connect the outlet pump to the MOSFET controlled by `MOSFET_OUT` (pin 19).
- Connect relevant sensors to measure water level and moisture levels.

## Usage

1. Run the ROS master: `roscore`.
2. Run the `pump_cont` script using the `rosrun` command, assuming you have the necessary permissions:
   ```bash
   rosrun <package_name> pump_controller.py
   ```
