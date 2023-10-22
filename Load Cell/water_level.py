#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import RPi.GPIO as gpio
import time

# Define GPIO pins for data (DAT) and clock (CLK)
DAT = 13
CLK = 8
num = 0

# Set GPIO pin numbering mode to BOARD
gpio.setmode(gpio.BOARD)

# Set up the clock (CLK) pin as an output
gpio.setup(CLK, gpio.OUT)

# Function to read weight data from a sensor
def weight(event):
    i = 0
    num = 0

    # Set data (DAT) pin as an output, and set it to 1
    gpio.setup(DAT, gpio.OUT)
    gpio.output(DAT, 1)

    # Set clock (CLK) pin to 0
    gpio.output(CLK, 0)

    # Set data (DAT) pin as an input
    gpio.setup(DAT, gpio.IN)

    # Wait for the falling edge of the data signal
    while gpio.input(DAT) == 1:
        i = 0

    # Read 24 bits of data
    for i in range(24):
        gpio.output(CLK, 1)
        num = num << 1
        gpio.output(CLK, 0)

        # Check the data signal and update the value
        if gpio.input(DAT) == 0:
            num = num + 1

    # Finalize the data reading
    gpio.output(CLK, 1)
    num = num ^ 0x800000
    gpio.output(CLK, 0)

    # Calculate weight and load according to your requirements
    wei = 0
    wei = (num / 1406)
    load = (wei - 6020) - 95

    # Publish the load value to a ROS topic
    pub.publish(Int16(int(load))

# Function to handle cleanup when the script is terminated
def shutdown():
    gpio.cleanup()

if __name__ == '__main__':
    try:
        rospy.init_node('water_level_node', anonymous=True)
        print("Started Water Level node")

        # Create a ROS publisher to publish water level data
        pub = rospy.Publisher('/water_level', Int16, queue_size=10)

        # Create a timer to trigger the weight function at a regular interval
        rospy.Timer(rospy.Duration(0.1), weight)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Cleanup GPIO pins when the script terminates
        shutdown()
