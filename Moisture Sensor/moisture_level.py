#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import serial
import time

# Initialize a ROS node
rospy.init_node('moisture_node', anonymous=True)

print("Started Moisture Sensing node")

# Create a ROS publisher to publish moisture level data
pub = rospy.Publisher('/moisture_level', Int16, queue_size=10)

# Initialize a serial connection to the moisture sensor on '/dev/ttyUSB0' with a baud rate of 9600 and a timeout of 1 second
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1.00)

# Wait for 1 second to ensure the serial connection is established
time.sleep(1)

# Reset the input buffer of the serial connection
ser.reset_input_buffer()

try:
    while True:
        # Pause the loop for 0.01 seconds to control the sensor data reading rate
        time.sleep(0.01)

        # Check if there is data waiting to be read from the serial connection
        if ser.in_waiting > 0:
            # Read a line of data from the serial connection and decode it as UTF-8
            line = ser.readline().decode('utf-8')

            # Publish the moisture level data as an Int16 message to the '/moisture_level' topic
            pub.publish(Int16(int(line)))

except KeyboardInterrupt:
    # Close the serial connection gracefully when the script is terminated
    ser.close()
