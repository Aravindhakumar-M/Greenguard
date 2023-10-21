#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

# Define GPIO pins for the ultrasonic sensor
TRIG_PIN = 23  # Trigger pin
ECHO_PIN = 24  # Echo pin

def ultrasonic_sensor_callback(publisher):
    # Set up the GPIO pins for the ultrasonic sensor
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # Trigger the ultrasonic sensor
    GPIO.output(TRIG_PIN, GPIO.LOW)
    rospy.sleep(0.2)  # Wait for the sensor to settle
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    rospy.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure the pulse duration of the echo pin
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = rospy.Time.now()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = rospy.Time.now()

    pulse_duration = pulse_end - pulse_start

    # Calculate the distance based on the pulse duration and the speed of sound
    distance = pulse_duration.to_sec() * 34300 / 2  # Speed of sound is 343 m/s

    # Publish the distance value
    publisher.publish(distance)

def ultrasonic_sensor_node():
    rospy.init_node('depth_node', anonymous=True)
    publisher = rospy.Publisher('/depth', Int16, queue_size=10)
    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

    while not rospy.is_shutdown():
        ultrasonic_sensor_callback(publisher)
        rate.sleep()

    # Clean up GPIO pins when the node is terminated
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        ultrasonic_sensor_node()
    except rospy.ROSInterruptException:
        pass
