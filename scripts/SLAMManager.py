#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import String

class SLAMManager:
    def __init__(self):
        rospy.init_node('slam_manager', anonymous=True)

        # Parameters
        self.enable_teleop = rospy.get_param('~teleop', True)
        self.enable_gazebo = rospy.get_param('~Gazebo', True)
        self.enable_slam_toolbox = rospy.get_param('~slam_toolbox', True)

        # Paths
        self.mapper_params_file = rospy.get_param('~mapper_params_file', '/path/to/mapper_params_online_sync.yaml')

        # Process handlers
        self.processes = []

        # Publisher for status
        self.status_pub = rospy.Publisher('/slam_status', String, queue_size=10)

    def start_process(self, command):
        """Start a subprocess and track it."""
        rospy.loginfo(f"Starting process: {' '.join(command)}")
        process = subprocess.Popen(command)
        self.processes.append(process)

    def start_teleop(self):
        if self.enable_teleop:
            self.start_process([
                "rosrun", "turtlebot3_teleop", "turtlebot3_teleop_key"
            ])

    def start_gazebo(self):
        if self.enable_gazebo:
            self.start_process([
                "roslaunch", "turtlebot3_gazebo", "turtlebot3_house.launch"
            ])

    def start_slam_toolbox(self):
        if self.enable_slam_toolbox:
            self.start_process([
                "rosrun", "slam_toolbox", "sync_slam_toolbox_node",
                "_params_file:=%s" % self.mapper_params_file
            ])

    def run(self):
        rospy.loginfo("Starting SLAM Manager...")

        # Start requested components
        self.start_teleop()
        rospy.sleep(1)  # Allow slight delay for initialization
        self.start_gazebo()
        rospy.sleep(5)  # Allow Gazebo some time to start
        self.start_slam_toolbox()

        # Periodically publish status
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.status_pub.publish(String(data="SLAM system running"))
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down SLAM Manager...")
        for process in self.processes:
            process.terminate()
            process.wait()

if __name__ == '__main__':
    manager = SLAMManager()
    try:
        manager.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        manager.shutdown()
