#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import time

class Pid_positioning:
    def __init__(self, current_time=None):
        # PID control constants, tune it according to your requirements
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0
        # Initial feedback value
        self.feedback_value = 0.0
        # Setpoint for the PID controller
        self.SetPoint = 100
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.error = 1000.0
        self.windup_guard = 1000000000.0
        self.vel_msg = Twist()
        self.initial_vel = Twist()
        self.depth = 0
        self.flag = 1
        self.depth = 0

        rospy.Subscriber('pot_pos', Int16, self.callback)
        rospy.Subscriber('depth', Int16, self.stop)

    def calculate(self, Kp, Ki, Kd, feedback_value, current_time=None):
        # PID control calculation
        self.error = (self.SetPoint - feedback_value)
        current_time = current_time if current_time is not None else time.time()
        self.delta_time = (current_time - self.last_time)
        self.delta_error = self.error - self.last_error
        self.PTerm = self.error
        self.ITerm += self.error * self.delta_time
        if self.ITerm < -self.windup_guard:
            self.ITerm = -self.windup_guard
        elif self.ITerm > self.windup_guard:
            self.ITerm = self.windup_guard
        self.DTerm = 0.0
        if self.delta_time > 0:
            self.DTerm = self.delta_error / self.delta_time
        self.last_time = self.current_time
        self.last_error = self.error
        self.output = (Kp * self.PTerm) + (Ki * self.ITerm) + (Kd * self.DTerm)
        return self.output

    def stop(self, data):
        # Stop the robot if depth sensor detects a certain condition
        self.depth = Twist()
        if data.data == 1:
            self.flag = 2
            self.depth.linear.x = self.depth.angular.z = 0

    def callback(self, data):
        # Handle pot position data and control the robot
        self.value = data.data
        self.vel_msg.linear.x = self.vel_msg.linear.y = self.vel_msg.linear.z = self.vel_msg.angular.x = self.vel_msg.angular.y = 0
        if -3 <= self.value <= 3:
            self.flag = 0
            if 0.2 < self.value <= 3:
                self.SetPoint = 0
                self.feedback_value = self.value
                self.Kp = 0.5
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -self.calculate(self.Kp, self.Ki, self.Kd, self.feedback_value)
            elif -0.2 > self.value >= -3:
                self.SetPoint = 0
                self.feedback_value = self.value
                self.Kp = 0.5
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = -self.calculate(self.Kp, self.Ki, self.Kd, self.feedback_value)
            elif -0.2 <= self.value <= 0.2:
                self.vel_msg.angular.z = 0
                self.vel_msg.linear.x = 0.5
                print("Pot found, Moving Forward !!")
        else:
            print("Desired pot not detected")
            self.flag = 1

    def publisher(self):
        # Publish velocity commands
        self.initial_vel.angular.z = 0.5
        while not rospy.is_shutdown():
            if self.flag == 1:
                pub.publish(self.initial_vel)
            elif self.flag == 0:
                pub.publish(self.vel_msg)
            else:
                pub.publish(self.depth)

if __name__ == '__main__':
    try:
        rospy.init_node('pot_runner', anonymous=True)
        print("Started Pot Runner Node")
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        positioning = Pid_positioning()
        positioning.publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
