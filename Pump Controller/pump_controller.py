#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import RPi.GPIO as gpio
import time

# Define GPIO pin numbers for MOSFET control
MOSFET_IN = 18
MOSFET_OUT = 19

# Set the GPIO mode to use board pin numbering
gpio.setmode(gpio.BOARD)

# Set up GPIO pins as output
gpio.setup(MOSFET_IN, gpio.OUT)
gpio.setup(MOSFET_OUT, gpio.OUT)

# Function to control the inlet pump based on the water level
# 300 Value depends on individual sensor and the calibration
def inlet(load):
    if load.data < 300:
        gpio.output(MOSFET_IN, gpio.HIGH)
        print("Inlet Pump Turned ON")
    elif load.data >= 300:
        gpio.output(MOSFET_IN, gpio.LOW)
        print("Inlet Pump Turned OFF")

# Function to control the outlet pump based on moisture level
def outlet(moist):
    if moist.data >= 60:
        gpio.output(MOSFET_OUT, gpio.LOW)
        print("Outlet Pump Turned OFF")
    elif 60 > moist.data >= 40:
        gpio.output(MOSFET_OUT, gpio.HIGH)
        print("Outlet Pump Turned ON for 4 seconds")
        time.sleep(4)
    elif 40 > moist.data >= 20:
        gpio.output(MOSFET_OUT, gpio.HIGH)
        print("Outlet Pump Turned ON for 6 seconds")
        time.sleep(6)
    elif moist.data < 20:
        gpio.output(MOSFET_OUT, gpio.HIGH)
        print("Outlet Pump Turned ON for 8 seconds")
        time.sleep(8)

# Function to perform GPIO cleanup
def shutdown():
    gpio.cleanup()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('pump_controller', anonymous=True)
        print("Started Pump Controller")

        # Subscribe to topics for moisture level and water level
        rospy.Subscriber('/moisture_level', Int16, outlet)
        rospy.Subscriber('/water_level', Int16, inlet)

        # Start the ROS node and listen for messages
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Perform GPIO cleanup on program exit
        shutdown()
