import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

def monitor_robot_status():
    rospy.init_node('robot_monitor')

    def odom_callback(msg):
        rospy.loginfo(f"Robot at position: {msg.pose.pose.position}")

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    monitor_robot_status()