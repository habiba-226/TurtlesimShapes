#!/usr/bin/env python3

import rospy # python lib
from geometry_msgs.msg import Twist # ros message type used to control robot movement (velocity commands)

def draw_circle():

    # init node, anon ensures unique node name
    rospy.init_node('turtle_circle', anonymous=True) 

    # creating publisher that sends messages to topic
    # /turtle1/cmd_vel is default, can be changed 
    # queue size = buffer size
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 

    #loop runs at 10 times per second so code doesnt send commands too quickly
    rate = rospy.Rate(10)

    vel_msg = Twist() # create twist message object
    vel_msg.linear.x = 2.5  # move forward 1 unit, large no, large circle size 
    vel_msg.angular.z = 1.0  # rotate to make a circle, large no, small circle size

    while not rospy.is_shutdown():

        #publishes the vel command to the topic
        pub.publish(vel_msg)

        # pauses execution so loop runs at the desired frequency
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass

