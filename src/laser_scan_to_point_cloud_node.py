#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg


rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("~converted_pc", PointCloud2, queue_size=1)
ref_robot_id = rospy.get_param("/ref_robot_id")

def scan_cb(msg):

    pc2_msg = lp.projectLaser(msg)
    pc2_msg.header.frame_id = f"robot_{ref_robot_id}/base_scan"
    pc_pub.publish(pc2_msg)


rospy.Subscriber(f"/robot_{ref_robot_id}/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()