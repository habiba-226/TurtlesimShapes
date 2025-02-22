#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import math

def draw_triangle():
    rospy.init_node('turtle_triangle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    time.sleep(2)  # Give time for the node to initialize

    vel_msg = Twist()

    # Triangle has 3 sides, so we loop 3 times
    for _ in range(3):
        # Move forward
        vel_msg.linear.x = 2.0  # Move at speed 2.0
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        time.sleep(2)  # Move forward for 2 seconds

        # Stop before turning
        vel_msg.linear.x = 0.0
        pub.publish(vel_msg)
        time.sleep(0.5)  # Pause briefly

        # Turn 120 degrees counterclockwise
        vel_msg.angular.z = math.radians(120)  # Convert degrees to radians
        pub.publish(vel_msg)
        time.sleep(1.5)  # Adjust timing for turning

        # Stop turning before moving forward again
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        time.sleep(0.5)  # Pause briefly

    # Stop the turtle at the end
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        draw_triangle()
    except rospy.ROSInterruptException:
        pass
