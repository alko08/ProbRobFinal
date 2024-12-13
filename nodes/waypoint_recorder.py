import rospy
from geometry_msgs.msg import PoseStamped

waypoints = []

def callback(msg):
    waypoints.append(msg.pose)
    rospy.loginfo(f"Waypoint recorded: {msg.pose}")

def waypoint_recorder():
    rospy.init_node('waypoint_recorder')
    rospy.Subscriber('/amcl_pose', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    waypoint_recorder()
