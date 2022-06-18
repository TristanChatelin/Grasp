#!/usr/bin/env python3

from copy import deepcopy
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import ros_numpy
import tf_conversions as tfc
import pcl_ros as pcl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf2_geometry_msgs import PointStamped
import tf2_ros
import open3d as o3d
import ctypes
import struct
import statistics
from visualization_msgs.msg import MarkerArray
from move_group_python import MoveGroupPython 
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import tf

from std_msgs.msg import String
import os
import json
import cv2
import random
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

## PARAMETERS

bin_pose_x = 0.97
bin_pose_y = -0.56
bin_pose_z = 0.297

home_pose_x = 0.28
home_pose_y = 0.15
home_pose_z = 0.06
home_pose_qx = 0.9988359688661409
home_pose_qy = -0.010401328978104684
home_pose_qz = -0.04115566981316654
home_pose_qw = 0.02290699668128736

## CODE


def move_arm(delta_x, delta_y, delta_z, delta_qx, delta_qy, delta_qz, delta_qw):

    global ur10_planner
    listener = tf.TransformListener()
    trans = False

    while (not rospy.is_shutdown()) and (trans == False):
        try:
            (trans,rot) = listener.lookupTransform("base_link", 'flange', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    print("\n\n", trans, "\n\n")

    pose_goal = PoseStamped()
    pose_goal.pose.position.x = trans[0] + delta_x
    pose_goal.pose.position.y = trans[1] + delta_y
    pose_goal.pose.position.z = trans[2] + delta_z

    pose_goal.pose.orientation.x = rot[0] + delta_qx
    pose_goal.pose.orientation.y = rot[1] + delta_qy
    pose_goal.pose.orientation.z = rot[2] + delta_qz
    pose_goal.pose.orientation.w = rot[3] + delta_qw
  
    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    tfListener = tf2_ros.TransformListener(tfBuffer)

    car_path = ur10_planner.plan_cartesian_path(pose_goal)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def move_arm_to_pose(delta_x, delta_y, delta_z, delta_qx, delta_qy, delta_qz, delta_qw):

    global ur10_planner
    listener = tf.TransformListener()
    trans = False

    while (not rospy.is_shutdown()) and (trans == False):
        try:
            (trans,rot) = listener.lookupTransform("base_link", 'flange', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    print("\n\n", trans, "\n\n")

    pose_goal = PoseStamped()
    pose_goal.pose.position.x = delta_x
    pose_goal.pose.position.y = delta_y
    pose_goal.pose.position.z = delta_z

    pose_goal.pose.orientation.x = rot[0]
    pose_goal.pose.orientation.y = rot[1]
    pose_goal.pose.orientation.z = rot[2]
    pose_goal.pose.orientation.w = rot[3]
  
    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    #tfListener = tf2_ros.TransformListener(tfBuffer)

    car_path = ur10_planner.plan_cartesian_path(pose_goal)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def go_to_home_pose():

    global ur10_planner

    """
    listener = tf.TransformListener()
    trans = False

    while (not rospy.is_shutdown()) and (trans == False):
        try:
            (trans,rot) = listener.lookupTransform("base_link", 'flange', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    print("\n\n", trans, "\n\n")
    """

    pose_goal = PoseStamped()
    pose_goal.pose.position.x = home_pose_x
    pose_goal.pose.position.y = home_pose_y
    pose_goal.pose.position.z = home_pose_z

    pose_goal.pose.orientation.x = home_pose_qx
    pose_goal.pose.orientation.y = home_pose_qy
    pose_goal.pose.orientation.z = home_pose_qz
    pose_goal.pose.orientation.w = home_pose_qw
  
    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    #tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    # tfListener = tf2_ros.TransformListener(tfBuffer)

    car_path = ur10_planner.plan_cartesian_path(pose_goal)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def get_magn(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    return (x**2 + y**2 + z**2)**0.5


def get_diff(pose1, pose2):
    x = pose1.position.x - pose2.position.x
    y = pose1.position.y - pose2.position.y
    z = pose1.position.z - pose2.position.z
    pose3 = Pose()
    pose3.position.x = x
    pose3.position.y = y
    pose3.position.z = z
    return pose3


def move_to_grasp_pose(pose_marker):

    pose_goal_from_gpd = PoseStamped()
    pose_goal_from_gpd.header = pose_marker.markers[2].header
    pose_goal_from_gpd.pose = pose_marker.markers[2].pose
    pose_a = PoseStamped()
    pose_a.header = pose_marker.markers[2].header
    pose_a.pose = pose_marker.markers[2].pose
    pose_b = PoseStamped()
    pose_b.header = pose_marker.markers[3].header
    pose_b.pose = pose_marker.markers[3].pose

    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    #tfListener = tf2_ros.TransformListener(tfBuffer)
    #pose_a = tfListener#tfBuffer.transform(pose_a, "base_link", rospy.Duration(1))
    #pose_b = tfListener # tfBuffer.transform(pose_b, "base_link", rospy.Duration(1))
    pose_c = deepcopy(pose_a)
    pose_c.pose = get_diff(pose_b.pose, pose_a.pose)
    magn_pose_c = get_magn(pose_c.pose)
    offset = 0.15 # scaling grasp offset between gripper and object 
    pose_c.pose.position.x = pose_c.pose.position.x * offset/magn_pose_c + pose_b.pose.position.x
    pose_c.pose.position.y = pose_c.pose.position.y * offset/magn_pose_c + pose_b.pose.position.y
    pose_c.pose.position.z = pose_a.pose.position.z * offset/magn_pose_c + pose_b.pose.position.z
    pose_c.pose.orientation = deepcopy(pose_b.pose.orientation)
    offseted_pose = deepcopy(pose_c)

    car_path = ur10_planner.plan_cartesian_path(offseted_pose)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def main():

    rospy.init_node("grasp_pose_subscriber", anonymous=True, log_level=rospy.ERROR)
    #ur10_planner = MoveGroupPython("manipulator", create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    #tfListener = tf2_ros.TransformListener(tfBuffer)
    print("Waiting for the topic '/detect_grasps/plot_grasps'")
    pose_marker = rospy.wait_for_message("/detect_grasps/plot_grasps", MarkerArray)

    move_to_grasp_pose(pose_marker)
    print("\n\nPress ENTER to move the arm to the bin")
    input()

    move_arm(0, 0, 0.30, 0, 0, 0, 0)
    move_arm_to_pose(bin_pose_x, bin_pose_y, bin_pose_z+30, 0, 0, 0, 0)
    move_arm(0, 0, -0.30, 0, 0, 0, 0)
    print("\n\nPress ENTER to move the arm to the home pose")
    input()

    go_to_home_pose()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass