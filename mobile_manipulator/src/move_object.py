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
import numpy as np
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


def move_arm(delta_x, delta_y, delta_z, delta_qx, delta_qy, delta_qz, delta_qw):
    global ur10_planner
    listener = tf.TransformListener()
    trans = False
    print("A")

    while (not rospy.is_shutdown()) and (trans == False):
        try:
            (trans,rot) = listener.lookupTransform("base_link", 'flange', rospy.Time(0))
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    #print("A")
    
    pose_goal = PoseStamped()
    #print(trans[0])
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




def main(): 
    rospy.init_node('move_arm', anonymous=True)
    move_arm(0, 0, 0, 0, 0, 0, 0)
    move_arm(0.2, 0.5, 0, 0, 0, 0, 0)
    move_arm(0, 0, -0.30, 0, 0, 0, 0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
 

"""
def publish_pose(marker):
    publisher = rospy.Publisher("/gpd_goal_pose", PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        publisher.publish(marker)
        rospy.sleep(0.1)


def get_diff(pose1, pose2):
  x = pose1.position.x - pose2.position.x
  y = pose1.position.y - pose2.position.y
  pose3 = Pose()
  pose3.position.x = x
  pose3.position.y = y
  return pose3


def callback(grasp_poses, tfBuffer):
    global ur10_planner
    #print(grasp_poses)
    pose_goal_from_gpd = PoseStamped()
    # the visualized grasp is a MarkerArray of four blocks. grasp_poses.markers[2] is the "wrist" block
    # just need to offset the depth a bit to avoid slamming the object, but can't be done with frame_id='map' 
    pose_goal_from_gpd.header = grasp_poses.markers[2].header
    pose_goal_from_gpd.pose = grasp_poses.markers[2].pose
    #ur10_planner.go_to_pose_goal(pose_goal_from_gpd)
    # transformed_pose_goal = tfBuffer.transform(pose_goal_from_gpd, "base_link", rospy.Duration(1))
    # roll, pitch, yaw = euler_from_quaternion(transformed_pose_goal.pose.orientation.x, transformed_pose_goal.pose.orientation.y, transformed_pose_goal.pose.orientation.z, transformed_pose_goal.pose.orientation.w)
    # pitch -= np.pi/2
    # rotated_pose = copy(transformed_pose_goal)
    # new_quat = get_quaternion_from_euler(roll, pitch, yaw)
    # rotated_pose.pose.orientation.x = new_quat[0]
    # rotated_pose.pose.orientation.y = new_quat[1]
    # rotated_pose.pose.orientation.z = new_quat[2]
    # rotated_pose.pose.orientation.w = new_quat[3]
    pose_a = PoseStamped()
    pose_a.header = grasp_poses.markers[2].header
    pose_a.pose = grasp_poses.markers[2].pose
    pose_b = PoseStamped()
    pose_b.header = grasp_poses.markers[3].header
    pose_b.pose = grasp_poses.markers[3].pose
    pose_a = tfBuffer.transform(pose_a, "base_link", rospy.Duration(1))
    pose_b = tfBuffer.transform(pose_b, "base_link", rospy.Duration(1))
    pose_c = deepcopy(pose_a)
    pose_c.pose = get_diff(pose_b.pose, pose_a.pose)
    magn_pose_c = get_magn(pose_c.pose)
    offset = 0.15 # scaling grasp offset between gripper and object 
    pose_c.pose.position.x = pose_c.pose.position.x * offset/magn_pose_c + pose_b.pose.position.x
    pose_c.pose.position.y = pose_c.pose.position.y * offset/magn_pose_c + pose_b.pose.position.y
    pose_c.pose.position.z = pose_a.pose.position.z
    pose_c.pose.orientation = deepcopy(pose_b.pose.orientation)
    offseted_pose = deepcopy(pose_c)
    # offseted_pose = give_offset(transformed_pose_goal,ur10_planner.move_group.get_current_pose().pose)
    #print(f"pose_a: {pose_a}\npose_c: {pose_c}")
    car_path = ur10_planner.plan_cartesian_path(offseted_pose)
    # reg_path = ur10_planner.go_to_pose_goal(offseted_pose)
    ur10_planner.display_trajectory(car_path[0])
    # ur10_planner.display_trajectory(reg_path)
    #rospy.sleep(5)
    ur10_planner.execute_plan(car_path[0])
    
    

if __name__ == '__main__':
    rospy.init_node("grasp_pose_subscriber", anonymous=True, log_level=rospy.ERROR)
    ur10_planner = MoveGroupPython("manipulator", create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(2)
    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback, queue_size=1, buff_size=52428800, callback_args=tfBuffer) 
    rospy.spin()
"""