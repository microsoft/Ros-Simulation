#!/usr/bin/env python

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

"""
Robot controller for controlling an ARGoS simulated robot via argos_bridge.
"""
import rospy
import math, random
from geometry_msgs.msg import Twist

class RobotController:
    cmdVelPub = None

    # Constants
    MAX_FORWARD_SPEED = 1
    MAX_ROTATION_SPEED = 2.5

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def dance(self, speed, distance):
        # Do the line dance!
        while not rospy.is_shutdown():
            self.move(speed, distance, True)  # Forward
            self.move(speed, distance, False) # Backward

    def move(self, speed, distance, isForward):
        vel_msg = Twist()

        #Checking if the movement is forward or backwards
        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)

       #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #Set the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0
        send_freq_hz = 20
        duration = rospy.Duration.from_sec(1.0 / send_freq_hz)

        #Loop to move the robot in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.cmdVelPub.publish(vel_msg)
            #Sleep to ensure reasonable send freq
            rospy.sleep(duration)
            #Takes actual time to velocity calculus
            t1=float(rospy.Time.now().to_sec())
            #Calculates distancePoseStamped
            current_distance = speed*(t1-t0)

        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        self.cmdVelPub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node("robot_controller")
    controller = RobotController()
    controller.dance(1,3)
    rospy.spin()
