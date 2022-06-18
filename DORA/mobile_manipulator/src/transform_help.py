#!/usr/bin/env python3

import numpy as np
from sklearn.exceptions import PositiveSpectrumWarning
import rospy
import math
from sensor_msgs.msg import PointCloud, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import ros_numpy
import pcl_ros as pcl
import tf2_ros
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped, PoseStamped
import open3d as o3d
import ctypes
import struct
import statistics
import copy
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2


if __name__ == '__main__':
    rospy.init_node('talker',log_level=rospy.DEBUG, anonymous=True)
    publisher = rospy.Publisher("/pointcloud_transform_help", PointCloud, queue_size=1) 
    publisher2 = rospy.Publisher("/pil", PoseStamped, queue_size=1) 
    pointcloud = PointCloud()
    
    point = Point()
    point.x = 0.74
    point.y = 0.7
    point.z = -0.05

    point2 = Point()
    point2.x = 0.74
    point2.y = -0.7
    point2.z = -0.05

    point3 = Point()
    point3.x = 1.44
    point3.y = 0.7
    point3.z = -0.05

    point4 = Point()
    point4.x = 1.44
    point4.y = -0.7
    point4.z = -0.05 

    pil = PoseStamped()
    pil.header.frame_id = "base_link"
    pil.pose.position.x = 0.7994203319333663
    pil.pose.position.y = 0.03284495633616635
    pil.pose.position.z = 0.009234213851340045

    pil.pose.orientation.x = 0.9990638510582398
    pil.pose.orientation.y = 0.023953760412496496
    pil.pose.orientation.z = -0.02066913222153613
    pil.pose.orientation.w = -0.02950298025604187


    pointcloud.header.frame_id = "base_link"
    pointcloud.points.append(point)   
    pointcloud.points.append(point2) 
    pointcloud.points.append(point3)  
    pointcloud.points.append(point4)   
    
    while not rospy.is_shutdown():
        publisher.publish(pointcloud)
        publisher2.publish(pil)
        rospy.sleep(0.5)





