#!/usr/bin/python3

import rospy
from geometry_msgs.msg import *
import time
import math 

rospy.init_node('init_pose_script', anonymous=True)
pub_  = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,latch = True ,queue_size=1)

p_x = rospy.get_param("~start_point_x",   default=0.0)
p_y = rospy.get_param("~start_point_y",   default=0.0)
yaw = rospy.get_param("~start_point_yaw", default=0.0)
print("p_x: ", p_x)
print("p_y: ", p_y)
print("yaw: ", yaw)

init_pose = PoseWithCovarianceStamped()
init_pose.header.stamp = rospy.Time.now()
init_pose.header.frame_id = "map"
init_pose.pose.pose.position.x = p_x
init_pose.pose.pose.position.y = p_y
init_pose.pose.pose.orientation.w = math.cos(yaw/2) 
init_pose.pose.pose.orientation.z = math.sin(yaw/2) 
init_pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
init_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
init_pose.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

pub_.publish(init_pose)
time.sleep(1.0)
print("setup pose finished.")
