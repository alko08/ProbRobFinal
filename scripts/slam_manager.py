#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import String

class SlamManager:
	def __init__(self):
		rospy.init_node('slam_manager', anonymous=True)
		self.slam_method = rospy.get_param('~default_slam', 'gmapping')
		rospy.Subscriber('/slam_method', String, self.slam_method_callback)
		
		self.launch = roslaunch.scriptapi.ROSLaunch()
		self.process = None
		self.start_slam(self.slam_method)

	def slam_method_callback(self,msg):
		"""Callback to handle incoming SLAM method requests"""
	
		new_method = msg.data
		if new_method != self.slam_method:
			rospy.loginfo(f"Changing SLAM method from {self.slam_method} to {new_method}")
			self.slam_method = new_method
			self.start_slam(selfslam_method)
		else:
			rospy.loginfo(f"Received {new_method} but was already using it for SLAM.")

	def start_slam(self, slam_method):
		"""Start the SLAM method using ROS launch"""
		if self.process: # used to stop any currently running launch file
			rospy.loginfo(f"Stopping current SLAM method: {self.slam_method}")
			self.process.stop()
			self.process = None
		package = "vanessa_alex"
		executable = "slam.launch"
		# Set the SLAM method argument
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		node_args = [(roslaunch.rlutil.resolve_launch_arguments([package, executable])[0], {'slam_methods': slam_method})]
		self.process = self.launch.launch(roslaunch.Node(package, executable, args=node_args))
		rospy.loginfo(f"Starting SLAM method: {slam_method}")
		self.launch.start()
	def shutdown(self):
		if self.launch:
			rospy.loginfo("Shutting down SLAM manager")
			self.launch.shutdown()
if __name__ == "__main__":
	try:
		manager = SlamManager()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Some error happened")
		pass
	finally:
		manager.shutdown()
