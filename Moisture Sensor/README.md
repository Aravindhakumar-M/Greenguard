# Moisture Sensing with Raspberry Pi and ROS

This Python script is designed to read moisture level data from a sensor connected to a serial port on a Raspberry Pi and publish the data to a ROS (Robot Operating System) topic. It uses serial communication to interface with the moisture sensor and provides a ROS interface for monitoring moisture levels.

## Prerequisites

Before using this script, ensure you have the following:

- Python 3
- ROS (Robot Operating System) installed and configured
- A moisture sensor connected to a serial port (e.g., `/dev/ttyUSB0`)
- Appropriate sensor drivers and configuration

## Configuration

The script may require configuration based on your hardware setup:

- Adjust the serial port (`/dev/ttyUSB0`) to match your serial connection.
- Modify the baud rate (`9600`) if your serial uses a different rate.
- Set the desired timeout duration to handle serial communication.

## Usage

1. Run the ROS master: `roscore`.
2. Run the script using the `rosrun` command, assuming you have the necessary permissions:
```bash
rosrun <package_name> moisture_level.py
```

1. The script initializes the ROS node as 'moisture_node' and sets up a publisher for moisture level data (/moisture_level).
2. It establishes a serial connection to the moisture sensor and reads data from it.
3. The script publishes the moisture level data as an Int16 message to the /moisture_level ROS topic.
4. Monitor the moisture levels by subscribing to the /moisture_level topic using other ROS nodes.

## Node Information
Node Name:  
`moisture_node`  

Publisher:  
`/moisture_level (for moisture level data)`

## Acknowledgments
This script demonstrates moisture level sensing using a serial-connected sensor and Raspberry Pi in a ROS environment. Credit to the creators of ROS and the Raspberry Pi ecosystem. Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
