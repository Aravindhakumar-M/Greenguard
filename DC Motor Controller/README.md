# Velocity to PWM Conversion for Motor Control

This Python script is designed to translate velocity commands received via ROS topics into PWM signals for controlling motors connected to a Raspberry Pi's GPIO pins. It assumes that you have motor drivers connected to the GPIO pins and that the ROS velocity commands are published on specific topics.

## Prerequisites

Before using this script, ensure you have the following:

- Python
- ROS (Robot Operating System)
- RPi.GPIO (for Raspberry Pi GPIO control)

## Hardware Setup

Make sure you have the appropriate hardware setup:

- Motor drivers connected to the GPIO pins on your Raspberry Pi.
- Appropriate motor drivers for your motors.

## Configuration

Adjust the following configuration parameters in the script as per your hardware and motor driver specifications:

- `MOTOR1_PIN_1`, `MOTOR1_PIN_2`, `MOTOR2_PIN_1`, `MOTOR2_PIN_2`, `MOTOR3_PIN_1`, `MOTOR3_PIN_2`, `MOTOR4_PIN_1`, `MOTOR4_PIN_2`: GPIO pin numbers for motor driver connections.
- `PWM_FREQUENCY`: PWM frequency suitable for your motor driver.
- `PWM_MIN_DUTY_CYCLE`, `PWM_MAX_DUTY_CYCLE`: Minimum and maximum PWM duty cycle values suitable for your motor driver.

## Usage

1. Run the ROS master: `roscore`.
2. Run the script using the `rosrun` command, assuming you have the necessary permissions:

```bash
rosrun <package_name> Motor_Controller.py
```

1. The script initializes the GPIO pins and sets them as outputs.
2. It creates subscribers for motor commands on specific topics (e.g., /Revolute1_velocity_controller/command, /Revolute2_velocity_controller/command, etc.).
3. When velocity commands are received, the script translates them into PWM signals and applies them to the specified GPIO pins for motor control.
4. Motor commands should be published on the corresponding ROS topics, and the script will convert them into PWM signals.

## Node Information
Node Name:  
velocity_to_pwm  

Subscribers:  
`/Revolute1_velocity_controller/command (for Motor 1)`  
`/Revolute2_velocity_controller/command (for Motor 2)`  
`/Revolute3_velocity_controller/command (for Motor 3)`  
`/Revolute4_velocity_controller/command (for Motor 4)`

## Acknowledgments
This script uses ROS and RPi.GPIO for controlling motors with PWM signals. Credit to the creators of ROS and RPi.GPIO library. Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
