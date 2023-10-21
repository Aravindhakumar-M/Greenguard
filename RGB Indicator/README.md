# LED Control with ROS and Raspberry Pi

This project is a Python script that controls three LEDs based on water level data received via a ROS (Robot Operating System) topic. The script runs on a Raspberry Pi and uses the RPi.GPIO library to interface with the GPIO pins, allowing it to control the LEDs.

## Prerequisites

Before using this script, ensure that you have the following installed and set up:

- Python 3
- ROS (Robot Operating System)
- RPi.GPIO (for Raspberry Pi GPIO control)
- `rospy` - ROS Python library
- `std_msgs` - ROS standard messages library

## Hardware Setup

- Connect three LEDs to the GPIO pins of your Raspberry Pi. In this script, we've used the GPIO pins 17, 18, and 19 for the LEDs.

## Configuration

- Modify the `LED1_PIN`, `LED2_PIN`, and `LED3_PIN` constants in the script to match the GPIO pins to which your LEDs are connected.

## Usage

1. Start a ROS environment on your Raspberry Pi.

2. Run the Python script by executing the following command:

```bash
rosrun <package_name> <script_name>.py
```

The script subscribes to a ROS topic named 'water_level' and monitors water level data.

## Based on the received water level data, the script controls the LEDs as follows:

If the water level is between 70 and 100, it turns on all LEDs.
If the water level is between 50 and 70, it blinks the first LED and turns on the other two.
If the water level is between 30 and 50, it blinks the second LED and turns off the first LED.
If the water level is between 10 and 30, it blinks the third LED and turns off the first two LEDs.
For any other water level, it turns off all LEDs.
You can observe the LED status based on the water level data.

## Node Information

Node Name: led_control_node
Subscribers:
water_level (topic for water level data)

## Acknowledgments
This script uses ROS and Raspberry Pi GPIO to control LEDs based on water level data. Credit to the creators of ROS and the RPi.GPIO library.
