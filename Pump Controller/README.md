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
2. Run the script using the `rosrun` command, assuming you have the necessary permissions:
   ```bash
   rosrun <package_name> pump_controller.py
   ```

1. The script subscribes to the `/moisture_level` and `/water_level` topics, which should provide sensor data.
2. The inlet pump will turn on when the water level is below 300, and it will turn off when the level is above or equal to 300.
3. The outlet pump operates based on the moisture level as follows:
   - If moisture is greater than or equal to 60, the outlet pump turns off.
   - If moisture is between 40 and 59, the outlet pump turns on for 4 seconds.
   - If moisture is between 20 and 39, the outlet pump turns on for 6 seconds.
   - If moisture is below 20, the outlet pump turns on for 8 seconds.
4. The script includes error handling and cleanup procedures.

## Acknowledgments
Credit to the creators of ROS for providing the tools and libraries for robotics development. We would like to express our gratitude to Anya Robotics Pvt Ltd for their contributions or support.
