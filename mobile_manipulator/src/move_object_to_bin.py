#!/usr/bin/env python3

from copy import deepcopy
import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
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
import rospy
from move_group_python import MoveGroupPython 
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np # Scientific computing library for Python
import tf

from std_msgs.msg import String
import sys
import os
import json
import cv2
import random
import rospy
import math as mt
import copy
import detectron2
from detectron2.utils.logger import setup_logger
from detectron2 import model_zoo ##
from detectron2.engine import DefaultPredictor ##
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer ##
from detectron2.data import MetadataCatalog, DatasetCatalog ##
import matplotlib.pyplot as plt
setup_logger() ##
import open3d as o3d
from argparse import ArgumentParser
from datetime import datetime ##
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf

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
    tfListener = tf2_ros.TransformListener(tfBuffer)

    car_path = ur10_planner.plan_cartesian_path(pose_goal)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def go_to_home_pose():

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
    pose_goal.pose.position.x = home_pose_x
    pose_goal.pose.position.y = home_pose_y
    pose_goal.pose.position.z = home_pose_z

    pose_goal.pose.orientation.x = home_pose_qx
    pose_goal.pose.orientation.y = home_pose_qy
    pose_goal.pose.orientation.z = home_pose_qz
    pose_goal.pose.orientation.w = home_pose_qw
  
    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    tfListener = tf2_ros.TransformListener(tfBuffer)

    car_path = ur10_planner.plan_cartesian_path(pose_goal)
    ur10_planner.display_trajectory(car_path[0])
    ur10_planner.execute_plan(car_path[0])


def main(): 
    rospy.init_node('move_arm', anonymous=True)
    move_arm(0, 0, 0.3, 0, 0, 0, 0)
    move_arm_to_pose(bin_pose_x, bin_pose_y, bin_pose_z+0.3, 0, 0, 0, 0)
    print("\n\nPress ENTER to move the arm to the home pose")
    input()
    move_arm(0, 0, -0.3, 0, 0, 0, 0)
    go_to_home_pose()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass