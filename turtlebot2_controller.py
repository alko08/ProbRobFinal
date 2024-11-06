#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_turtlebot():
    # Initialize the ROS node
    rospy.init_node('turtlebot2_controller', anonymous=True)

    # Create a publisher to the cmd_vel topic
    velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    # Alternatively, if your TurtleBot uses the cmd_vel topic directly, uncomment the following line:
    # velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Twist message instance
    vel_msg = Twist()

    # Move forward for 5 seconds
    vel_msg.linear.x = 0.2  # Move forward at 0.2 m/s
    vel_msg.angular.z = 0.0  # No rotation

    rospy.loginfo("Moving forward")
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < 5:
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    # Stop the robot
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(1)

    # Rotate for 5 seconds
    vel_msg.angular.z = 0.5  # Rotate at 0.5 rad/s
    rospy.loginfo("Rotating")
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < 5:
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    # Stop the robot
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Motion complete")

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass
