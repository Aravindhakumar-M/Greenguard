# Water Level Monitoring with Raspberry Pi and ROS

This Python script is designed to read weight data from a sensor connected to Raspberry Pi's GPIO pins and publish the water level to a ROS (Robot Operating System) topic. It uses GPIO pins to interface with the sensor and provides a ROS interface to monitor water levels.

## Prerequisites

Before using this script, ensure you have the following:

- Python
- ROS (Robot Operating System)
- RPi.GPIO (for Raspberry Pi GPIO control)

## Hardware Setup

Make sure you have the appropriate hardware setup:

- Raspberry Pi with GPIO pins.
- Weight sensor or load cell connected to GPIO pins.
- Appropriate calibration for the sensor's weight-to-load conversion.

## Configuration

The script requires configuring the GPIO pins and may need adjustments based on your hardware setup:

- `DAT` and `CLK`: GPIO pin numbers for data and clock connections.
- Weight calibration values to convert raw data to actual load/weight.

## Usage

1. Run the ROS master: `roscore`.
2. Run the `pump_cont` script using the `rosrun` command, assuming you have the necessary permissions:
```bash
rosrun <package_name> water_level.py
```

1. The script initializes the ROS node and sets up a publisher for water level data (/water_level).
2. It reads weight data from the sensor using GPIO pins and converts it to a water level.
3. The script publishes the water level data to the /water_level ROS topic at a regular interval.
4. Monitor the water level by subscribing to the /water_level topic using other ROS nodes.

## Node Information  
Node Name:  
`water_level_node`  

Publisher:  
`/water_level (for water level data)`

## Acknowledgments
This script demonstrates water level monitoring using a weight sensor and Raspberry Pi GPIO pins in a ROS environment. Credit to the creators of ROS and the RPi.GPIO library.
