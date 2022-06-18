#!/usr/bin/env python3

import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
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

center_point = PointStamped()


def callback(ros_point_cloud):
    #pc = ros_numpy.numpify(data)
    #callback_args.append(pc)
    #save_pointcloud(pc)
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd, x = out_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        out_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        #out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        o3d.ge
        o3d.io.write_point_cloud("/home/arvid/Desktop/GDP_DATA/cloud.pcd",out_pcd,write_ascii=True)
        

def create_centerpoint(data):
    pc = ros_numpy.numpify(data)
    
    x = []
    y = []
    z = []
    color = []
    for row in pc:
        x.append(row[0])
        y.append(row[1])
        z.append(row[2])
        color.append(row[3])

    global center_point
    center_point.header.frame_id = "camera_link"
    center_point.header.stamp = rospy.Time.now()
   
    center_point.point.x = statistics.median(x)
    center_point.point.y = statistics.median(y)
    center_point.point.z = statistics.median(z)

    


#continously listens to the point-cloud topic and runs callback on the data
def listener():
    rospy.init_node('listener',log_level=rospy.DEBUG, anonymous=True)
    rospy.Subscriber("/extract_objects_indices/output", PointCloud2, callback, queue_size=1, buff_size=52428800) 
    rospy.spin()

#grabs message from the point-cloud topic and runns callback on its data
def get_data():
    rospy.init_node('listener', anonymous=True)
    data = rospy.wait_for_message("/convex_hull/output",PointCloud2) #"/camera/depth/points"
    callback(data) 


if __name__ == '__main__':
    rospy.init_node("camera_center_point")
    queue_size = 1000
    buffer_size = 52428800
    point_publisher = rospy.Publisher("can_centerpoint", data_class=PointStamped,queue_size=queue_size)
    rospy.Subscriber("/extract_objects_indices/output", data_class=PointCloud2, 
        callback=create_centerpoint,queue_size=queue_size, buff_size=buffer_size)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown(): 
        point_publisher.publish(center_point)
        rate.sleep()
    rospy.spin()
    #listener()






