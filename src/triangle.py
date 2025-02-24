#!/usr/bin/env python3

import rospy # python lib
from geometry_msgs.msg import Twist # ros message type used to control robot movement (velocity commands)
import time # adds delays
import math #math calcs 

def draw_triangle():

    # init node, anon ensures unique node name
    rospy.init_node('turtle_triangle', anonymous=True)

    # creating publisher that sends messages to topic
    # /turtle1/cmd_vel is default, can be changed 
    # queue size = buffer size
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # give time for the node to initialize
    # without this, 1 side is missing 
    time.sleep(2)  

    vel_msg = Twist() # create twist message object

    # triangle has 3 sides, loop 3 times
    # dont care about the loop var, use _
    for _ in range(3):

        # moves straight ahead for 2 seconds at speed 2.0
        vel_msg.linear.x = 2.0  # move at speed 2.0
        vel_msg.angular.z = 0.0 # ensures no prev rotation from other commands
        pub.publish(vel_msg) # publish message to topic 

        # without sleep next command will be sent immediately 
        # turtle won’t have time to move forward properly
        time.sleep(2)  #  keep last command active for 2 seconds before sending a new one

        # stops turtle for 0.5 seconds before turning
        vel_msg.linear.x = 0.0
        pub.publish(vel_msg)
        time.sleep(0.5) 

        # turn 120 degrees anticlockwise
        vel_msg.angular.z = math.radians(120)  # convert degrees to radians
        pub.publish(vel_msg)
        time.sleep(1.5) # turn takes 1.5 seconds.

        # stops rotating for 0.5 seconds 
        # before starting the next side
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        time.sleep(0.5)  

    # stop the turtle after loop finsh
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        draw_triangle()
    except rospy.ROSInterruptException:
        pass
