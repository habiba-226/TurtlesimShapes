#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def move_straight(pub, speed, distance, duration):
    vel_msg = Twist()
    vel_msg.linear.x = speed  # Move forward
    vel_msg.angular.z = 0  # No rotation
    
    rate = rospy.Rate(10)  # 10 Hz (updates per second)
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(vel_msg)
        rate.sleep()

    # Stop the turtle after moving
    vel_msg.linear.x = 0
    pub.publish(vel_msg)

def rotate(pub, angle_speed, angle, duration):
    vel_msg = Twist()
    vel_msg.linear.x = 0  # No forward movement
    vel_msg.angular.z = angle_speed  # Rotate

    rate = rospy.Rate(10)
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(vel_msg)
        rate.sleep()

    # Stop rotation
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def draw_rectangle():
    rospy.init_node('turtle_rectangle', anonymous=True)
    
    # Publisher to send velocity commands
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Wait for the clear service
    rospy.wait_for_service('/clear')
    clear_screen = rospy.ServiceProxy('/clear', Empty)
    clear_screen()  # Clears background
    
    rospy.sleep(1)  # Small delay before starting
    
    speed = 2.0  # Turtle's movement speed
    turn_speed = 1.57  # Turtle's turning speed (90 degrees = 1.57 rad/s)
    
    # Rectangle dimensions
    width = 4.0
    height = 2.0
    
    # Move in a rectangle (repeat for all four sides)
    move_straight(pub, speed, width, width/speed)  # Move right
    rotate(pub, turn_speed, 1.57, 1.0)  # Turn 90 degrees left
    
    move_straight(pub, speed, height, height/speed)  # Move up
    rotate(pub, turn_speed, 1.57, 1.0)  # Turn 90 degrees left
    
    move_straight(pub, speed, width, width/speed)  # Move left
    rotate(pub, turn_speed, 1.57, 1.0)  # Turn 90 degrees left
    
    move_straight(pub, speed, height, height/speed)  # Move down
    
    print("Rectangle drawing completed!")

if __name__ == '__main__':
    try:
        draw_rectangle()
    except rospy.ROSInterruptException:
        pass
