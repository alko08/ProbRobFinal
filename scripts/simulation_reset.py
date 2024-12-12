import rospy
from std_srvs.srv import Empty

def reset_simulation():
    rospy.init_node('simulation_reset')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    rospy.wait_for_service('/gazebo/reset_world')

    try:
        reset_world()
        rospy.loginfo("Simulation reset!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset simulation: {e}")

if __name__ == '__main__':
    reset_simulation()
