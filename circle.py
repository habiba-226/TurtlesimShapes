#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def draw_circle():
    rospy.init_node('turtle_circle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    vel_msg = Twist()
    vel_msg.linear.x = 1.0  # Move forward
    vel_msg.angular.z = 1.0  # Rotate to make a circle

    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass
