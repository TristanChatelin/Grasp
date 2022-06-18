#!/usr/bin/env python3

import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import ros_numpy
import pcl_ros as pcl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf2_geometry_msgs import PointStamped
import open3d as o3d
import ctypes
import struct
import statistics
from gpd_ros.msg import *
from pcl2_o3d import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
#import rclpy 
#from rclpy.node import Node
#from open3d_ros_helper import open3d_ros_helper as orh



def callback(ros_point_cloud):
    #pc = ros_numpy.numpify(data)
    #callback_args.append(pc)
    #save_pointcloud(pc)
        xyz = np.array([[0,0,0]])
        #rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            #test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            #s = struct.pack('>f' ,test)
            #i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            #pack = ctypes.c_uint32(i).value
            #r = (pack & 0x00FF0000)>> 16
            #g = (pack & 0x0000FF00)>> 8
            #b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            #rgb = np.append(rgb,[[r,g,b]], axis = 0)

        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd, x = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        out_pcd = convertCloudFromOpen3dToRos(pcd,frame_id="base_link")
        cloud_pub = rospy.Publisher("/clouddata",data_class=PointCloud2, queue_size=queue_size)
        cloud_pub.publish(out_pcd)
        rate = rospy.Rate(10)
        rate.sleep()


#continously listens to the point-cloud topic and runs callback on the data
def listener():
    #rospy.init_node('listener',log_level=rospy.DEBUG, anonymous=True)
    rospy.Subscriber("/extract_objects_indices/output", PointCloud2, callback, queue_size=1, buff_size=52428800) 
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node("cloud_data")
    queue_size = 1000
    buffer_size = 52428800
    listener()
