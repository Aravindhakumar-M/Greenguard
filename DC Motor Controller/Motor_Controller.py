#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO

# GPIO pin numbers for motor driver connections
MOTOR1_PIN_1 = 16
MOTOR1_PIN_2 = 18
MOTOR2_PIN_1 = 19
MOTOR2_PIN_2 = 21
MOTOR3_PIN_1 = 22
MOTOR3_PIN_2 = 23
MOTOR4_PIN_1 = 24
MOTOR4_PIN_2 = 26

# PWM frequency and duty cycle limits
PWM_FREQUENCY = 100  # Adjust as per your motor driver
PWM_MIN_DUTY_CYCLE = 0  # Adjust as per your motor driver
PWM_MAX_DUTY_CYCLE = 100  # Adjust as per your motor driver

def motor1_cmd_callback(msg):
    duty_cycle = convert_velocity_to_duty_cycle(msg.data)
    set_motor_pwm(MOTOR1_PIN_1, MOTOR1_PIN_2, duty_cycle)

def motor2_cmd_callback(msg):
    duty_cycle = convert_velocity_to_duty_cycle(msg.data)
    set_motor_pwm(MOTOR2_PIN_1, MOTOR2_PIN_2, duty_cycle)

def motor3_cmd_callback(msg):
    duty_cycle = convert_velocity_to_duty_cycle(msg.data)
    set_motor_pwm(MOTOR3_PIN_1, MOTOR3_PIN_2, duty_cycle)

def motor4_cmd_callback(msg):
    duty_cycle = convert_velocity_to_duty_cycle(msg.data)
    set_motor_pwm(MOTOR4_PIN_1, MOTOR4_PIN_2, duty_cycle)

def convert_velocity_to_duty_cycle(velocity):
    # Convert velocity (ranging from -1.0 to 1.0) to duty cycle in the range of 0 to 1
    duty_cycle = (velocity + 1.0) / 2.0

    # Clip duty cycle to the range of 0 to 1
    duty_cycle = max(min(duty_cycle, 1.0), 0.0)

    return duty_cycle

def set_motor_pwm(pin1, pin2, duty_cycle):
    # Convert duty cycle to PWM value in the range of 0 to 100
    pwm_value = int(duty_cycle * (PWM_MAX_DUTY_CYCLE - PWM_MIN_DUTY_CYCLE) + PWM_MIN_DUTY_CYCLE)

    # Set the PWM duty cycle on the motor driver GPIO pins
    GPIO.output(pin1, pwm_value)
    GPIO.output(pin2, 0)  # Set the other pin to 0

if __name__ == '__main__':
    rospy.init_node('velocity_to_pwm')

    # Initialize GPIO pins as output
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR1_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR1_PIN_2, GPIO.OUT)
    GPIO.setup(MOTOR2_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR2_PIN_2, GPIO.OUT)
    GPIO.setup(MOTOR3_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR3_PIN_2, GPIO.OUT)
    GPIO.setup(MOTOR4_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR4_PIN_2, GPIO.OUT)

    # Create subscribers for the motor commands
    motor1_cmd_sub = rospy.Subscriber('/Revolute1_velocity_controller/command', Float64, motor1_cmd_callback)
    motor2_cmd_sub = rospy.Subscriber('/Revolute2_velocity_controller/command', Float64, motor2_cmd_callback)
    motor3_cmd_sub = rospy.Subscriber('/Revolute3_velocity_controller/command', Float64, motor3_cmd_callback)
    motor4_cmd_sub = rospy.Subscriber('/Revolute4_velocity_controller/command', Float64, motor4_cmd_callback)

    rospy.spin()

    # Cleanup GPIO pins
    GPIO.cleanup()
