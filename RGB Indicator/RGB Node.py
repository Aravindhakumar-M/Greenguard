#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

# Define the GPIO pins for the LEDs
LED1_PIN = 17
LED2_PIN = 18
LED3_PIN = 19

# Initialize the GPIO configuration
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)
GPIO.setup(LED3_PIN, GPIO.OUT)

# Callback function to handle received water level data
def run():
    rospy.Subscriber('water_level', Int16, value_callback)
    rospy.spin()

# Callback function to process water level data and control LEDs
def value_callback(msg):
    value = msg.data
    if 70 <= value <= 100:
        turn_on_all_leds()
    elif 50 <= value < 70:
        blink_led(LED1_PIN)
        turn_on_led(LED2_PIN)
        turn_on_led(LED3_PIN)
    elif 30 <= value < 50:
        blink_led(LED2_PIN)
        turn_off_led(LED1_PIN)
        turn_on_led(LED3_PIN)
    elif 10 <= value < 30:
        blink_led(LED3_PIN)
        turn_off_led(LED1_PIN)
        turn_off_led(LED2_PIN)
    else:
        turn_off_all_leds()

# Functions to control LEDs
def turn_on_all_leds():
    GPIO.output(LED1_PIN, GPIO.HIGH)
    GPIO.output(LED2_PIN, GPIO.HIGH)
    GPIO.output(LED3_PIN, GPIO.HIGH)

def turn_on_led(pin):
    GPIO.output(pin, GPIO.HIGH)

def turn_off_led(pin):
    GPIO.output(pin, GPIO.LOW)

def turn_off_all_leds():
    GPIO.output(LED1_PIN, GPIO.LOW)
    GPIO.output(LED2_PIN, GPIO.LOW)
    GPIO.output(LED3_PIN, GPIO.LOW)

# Function to blink an LED
def blink_led(pin):
    GPIO.output(pin, GPIO.HIGH)
    rospy.sleep(0.5)
    GPIO.output(pin, GPIO.LOW)
    rospy.sleep(0.5)

# Cleanup GPIO when the script ends
def shutdown():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('led_control_node', anonymous=True)
        print("Started RGB node")
        run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Cleanup GPIO
        shutdown()
