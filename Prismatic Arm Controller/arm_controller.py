#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO
import time

flag = False
moisture = 0
GPIO.setmode(GPIO.BCM)

# Control pin for one arm movement direction
GPIO.setup(17, GPIO.OUT)
# Control pin for the other arm movement direction
GPIO.setup(22, GPIO.OUT)
# Arm control signal
GPIO.setup(23, GPIO.OUT)
# Input pin for the limit switch
GPIO.setup(21, GPIO.IN)

# Enable the arm control signal
GPIO.output(23, True)

# Set one arm direction to low
GPIO.output(17, False)

# Set the other arm direction to high
GPIO.output(22, True)

# Callback function to handle moisture level data
def func_moisture(moist):
    global moisture
    moisture = moist.data

# Callback function to control the arm based on depth sensor data (limit switch)
def func_depth(data):
    global flag

    # Check if the depth reading is less than 5 (considering it as infront of a potted plant)
    if data.data < 5:

        # If the depth sensor detects obstacle and the limit switch is active (arm control signal is high)
        if GPIO.input(21) == 1:
            GPIO.output(17, False)  # Turn off one arm direction
            GPIO.output(22, False)  # Turn off the other arm direction
            flag = True  # Set the flag to True

        # If the depth sensor detects obstacle and the flag is False (limit switch not active)
        elif flag == False:
            GPIO.output(17, False)  # Turn off one arm direction
            GPIO.output(22, True)   # Set the other arm direction to high

        # If the depth sensor detects obstacle and the flag is True (limit switch active)
        elif flag == True:
            GPIO.output(17, True)    # Set one arm direction to high
            GPIO.output(22, False)   # Turn off the other arm direction

            # Check the moisture level
            if moisture > 20:
                GPIO.output(17, False)  # Turn off one arm direction
                GPIO.output(22, False)  # Turn off the other arm direction
                time.sleep(2)           # Wait for 2 seconds

                # Set the arm direction to retract
                GPIO.output(17, False)
                GPIO.output(22, True)

def shutdown():
    GPIO.cleanup()  # Cleanup GPIO pins

# Main execution
if __name__ == '__main__':
    try:
        rospy.init_node('arm_cont', anonymous=True)
        print("Started Arm Controller node")

        # Subscribe to depth and moisture level topics
        rospy.Subscriber('/depth', Int16, func_depth)
        rospy.Subscriber('/moisture_level', Int16, func_moisture)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        shutdown()  # Cleanup GPIO pins on node shutdown
