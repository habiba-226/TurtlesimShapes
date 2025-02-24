#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import math

def draw_rectangle():
    rospy.init_node('turtle_rectangle', anonymous=True)
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    time.sleep(2)  # give time for the node to initialize

    vel_msg = Twist()

    for _ in range(2):  # repeat for 2 times 2 sides 

        # moves straight ahead for 2 seconds at speed 5.0 
        # long side of rectangle (5 units)
        vel_msg.linear.x = 5.0
        vel_msg.angular.z = 0.0 # ensures no prev rotation from other commands
        publisher.publish(vel_msg)
        time.sleep(2)  # move forward for 2 seconds

        # stop for 0.5 before turning
        # as if robot is still moving forward while turning
        # might take a curved path instead of a sharp 90-degree turn
        vel_msg.linear.x = 0.0
        publisher.publish(vel_msg)
        time.sleep(0.5)

        # turn 90 degrees
        vel_msg.angular.z = math.radians(90)
        publisher.publish(vel_msg)
        time.sleep(1.5)

        # moves straight ahead for 2 seconds at speed 2.5 
        # short side of rectangle (2.5 units)
        vel_msg.linear.x = 2.5
        vel_msg.angular.z = 0.0
        publisher.publish(vel_msg)
        time.sleep(2)

        # stop for 0.5 before turning
        vel_msg.linear.x = 0.0
        publisher.publish(vel_msg)
        time.sleep(0.5)

        # turn 90 degrees
        vel_msg.angular.z = math.radians(90)
        publisher.publish(vel_msg)
        time.sleep(1.5)

        # stop turning
        vel_msg.angular.z = 0.0
        publisher.publish(vel_msg)
        time.sleep(1)

    # stop the turtle at the end
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        draw_rectangle()
    except rospy.ROSInterruptException:
        pass
