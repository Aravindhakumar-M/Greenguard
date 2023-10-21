# Ultrasonic Sensor Node for Depth Measurement

This Python script is designed for a Raspberry Pi to interface with an ultrasonic sensor and measure the depth of a liquid or any other substance. It utilizes the ROS (Robot Operating System) framework and the GPIO library for Raspberry Pi.

## Prerequisites

- A Raspberry Pi with GPIO pins
- ROS (Robot Operating System) installed on the Raspberry Pi
- Appropriate wiring for connecting the ultrasonic sensor to the Raspberry Pi's GPIO pins

## Setup

1. Connect the ultrasonic sensor to the Raspberry Pi's GPIO pins as per your hardware configuration.

2. Make sure you have ROS installed and set up on your Raspberry Pi.

3. Run the script using the `rosrun` command, assuming you have the necessary permissions:
```bash
rosrun <package_name> depth.py
```


## Operation

The script operates as follows:

- It sets up the GPIO pins for the ultrasonic sensor, specifying the trigger and echo pins.
- A callback function, `ultrasonic_sensor_callback`, is defined to perform the following actions:
- Trigger the ultrasonic sensor to send a pulse.
- Measure the pulse duration using the echo pin.
- Calculate the distance based on the pulse duration and the speed of sound.
- Publish the calculated distance to a ROS topic.
- The `ultrasonic_sensor_node` function initializes the ROS node and sets up a publisher for the depth measurements.
- The script publishes depth measurements at a rate of 10 Hz.

## Acknowledgments
This script demonstrates Active Collision Avoidance using a Ultrasonic sensor and Raspberry Pi GPIO pins in a ROS environment. Credit to the creators of ROS and the RPi.GPIO library.
