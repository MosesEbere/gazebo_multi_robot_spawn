#!/usr/bin/python3

"""
Code for resetting the launch file for spawning multiple robots in Gazebo.
"""

import rospy
import rospkg
import os

def reset_launch_file(file_name):
    # delete the file if it exists
    if os.path.exists(file_name):
        os.remove(file_name)
    with open(file_name, "w") as f:
        f.write('<!-- This file is generated at runtime and reset after all robots have been spawned -->\n')
        f.write('<launch>\n')
        f.write('</launch>\n')

def ros_node():
    rospy.init_node('launch_file_resetter', anonymous=True)
    launch_file_name = 'spawn_multi_robot.launch'
    rospack = rospkg.RosPack()
    # use os.path.join to get the path to the launch file dont add / anywhere
    file_name = os.path.join(os.path.join(rospack.get_path('gazebo_multi_robot_spawn'), 'launch'), launch_file_name)

    reset_launch_file(file_name)

    rospy.loginfo("Launch file has been reset: %s", file_name)

    # Shutdown the node
    rospy.signal_shutdown("Reset Completed")

if __name__ == "__main__":
    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass