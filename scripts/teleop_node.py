import rospy
from geometry_msgs.msg import Twist

def move_turtlebot():
	rospy.init_node('teleop_node', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		twist = Twist()
		twist.linear.x = 0.2
		twist.angular.z = 0.5
		pub.publish(twist)
		rate.sleep()

if __name__ == '__main__':
	try:
		move_turtlebot()
	except rospy.ROSInterrutpionException:
		pass

