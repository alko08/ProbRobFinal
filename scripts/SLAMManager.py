#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import String

class SlamManager:
    def __init__(self):
        rospy.init_node('slam_manager', anonymous=True)

        # Get the default SLAM method from parameter or use 'gmapping'
        self.slam_method = rospy.get_param('~default_slam', 'gmapping')

        # Subscribe to the /slam_method topic
        rospy.Subscriber('/slam_method', String, self.slam_method_callback)

        # Create a ROSLaunch instance
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.process = None

        # Start the default SLAM method
        self.start_slam(self.slam_method)

    def slam_method_callback(self, msg):
        """Callback to handle incoming SLAM method requests."""
        new_method = msg.data
        if new_method != self.slam_method:
            rospy.loginfo(f"Changing SLAM method from {self.slam_method} to {new_method}")
            self.slam_method = new_method
            self.start_slam(self.slam_method)
        else:
            rospy.loginfo(f"Received {new_method} but SLAM is already using it.")

    def start_slam(self, slam_method):
        """Start the SLAM method using ROS launch."""
        # Stop any currently running process
        if self.process:
            rospy.loginfo(f"Stopping current SLAM method: {self.slam_method}")
            self.process.stop()
            self.process = None

        # Define the package and executable launch file
        package = "vanessa_alex"
        executable = "slam.launch"

        # Set the SLAM method argument and start the process
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments([package, executable])[0]
        args = {'slam_methods': slam_method}

        # Launch the node with arguments
        rospy.loginfo(f"Starting SLAM method: {slam_method}")
        self.process = self.launch.launch(roslaunch.core.Node(package, executable, output="screen", args=f"slam_methods:={slam_method}"))

    def shutdown(self):
        """Stop the running process and shutdown."""
        if self.process:
            rospy.loginfo("Stopping running SLAM process...")
            self.process.stop()
        rospy.loginfo("Shutting down SLAM manager...")

if __name__ == "__main__":
    try:
        manager = SlamManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        manager.shutdown()
