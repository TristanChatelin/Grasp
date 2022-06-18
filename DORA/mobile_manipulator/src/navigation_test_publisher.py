#!/usr/bin/env python3

from math import sqrt
from copy import copy
import rospy
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


if __name__=="__main__":
    rospy.init_node("navigation_tester_publisher", anonymous=True)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pose_to_be_published = PoseStamped()
    pose_to_be_published.header.frame_id = "map"
    pose_to_be_published.header.stamp = rospy.Time.now()
    pose_to_be_published.pose.position.x = -3.515613
    pose_to_be_published.pose.position.y = 6.208450
    pose_to_be_published.pose.orientation.z = 0.6959281
    pose_to_be_published.pose.orientation.w = 0.7181115
    rospy.sleep(3)
    print(pose_to_be_published)
    goal_pub.publish(pose_to_be_published)