import numpy as np
import math as mt
import sys
import os
import klampt
from klampt.io import numpy_convert
from klampt import vis
import open3d as o3d
path_to_gpd = os.path.dirname(os.path.abspath(__file__))+'/../'
sys.path.append(path_to_gpd)
import gpd.build.gpd_python as gpd


def generate_grasp_poses(point_cloud, name_object):

    # Generate a pcd file which will be the input of GPD 
    point_cloud_pcd = np.array([])
    point_cloud_pcd = np.hstack((point_cloud_pcd,'# .PCD v0.7 - Point Cloud Data file format'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'VERSION 0.7'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'FIELDS x y z'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'SIZE 4 4 4'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'TYPE F F F'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'COUNT 1 1 1'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'WIDTH '+str(point_cloud.shape[0])))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'HEIGHT 1'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'VIEWPOINT 0 0 0 0 1 0 0'))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'POINTS '+str(point_cloud.shape[0])))
    point_cloud_pcd = np.vstack((point_cloud_pcd,'DATA ascii'))
    for i in range(point_cloud.shape[0]):
        line = str(point_cloud[i, 0]) + ' ' + str(point_cloud[i,1]) + ' ' + str(point_cloud[i,2])
        point_cloud_pcd = np.vstack((point_cloud_pcd, line))

    np.savetxt('objects/' + 'pcd_' + name_object + '.pcd', point_cloud_pcd, fmt='%s')
    path_to_eigen_params = os.path.dirname(os.path.abspath(__file__))+'/../gpd/cfg/eigen_params.cfg'
    detector = gpd.GraspDetector(path_to_eigen_params)
    pcd = o3d.io.read_point_cloud('objects/' + 'pcd_' + name_object + '.pcd')
    os.remove('objects/' + 'pcd_' + name_object + '.pcd')

    pts = np.asarray(pcd.points)
    pts = pts[np.logical_not(np.isnan(pts[:, 0])), :]
    tf = np.eye(4)
    pts = (tf @ (np.hstack(( pts, np.ones((len(pts), 1)) ))).T).T[:, :3]
    offset = np.mean(pts, axis=0, keepdims=True)
    grasps = gpd.detectGrasps(detector, pts - offset, np.ones((1, len(pts))), np.zeros((3, 1)))

    k_pcd = klampt.PointCloud()
    for i in range(len(pts)):
        k_pcd.addPoint(pts[i, :])
    world = klampt.WorldModel()
    for i in range(1, len(grasps)):
        g: np.ndarray = grasps[i, :]
        pos = g[:3] #+ offset   # Undo mean subtraction
        rot = g[3:12].reshape(3, 3)
        k_tf = (rot.T.flatten().tolist(), pos.reshape(-1).tolist())

    np.savetxt('objects/' + 'grasps_' + name_object + '.txt', grasps, fmt='%s')


def generate_cuboid(width, length, height, space_between_points, name_object):

    point_cloud = np.array([])
    point_cloud = np.hstack((point_cloud,(0, 0, 0)))

    for x_i in range(int(width/space_between_points)):
        for y_i in range(int(length/space_between_points)):
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, y_i*space_between_points-length/2, -height/2)))
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, y_i*space_between_points-length/2, height/2)))

    for y_i in range(int(length/space_between_points)):
        for z_i in range(int(height/space_between_points)):
            point_cloud = np.vstack((point_cloud,(-width/2, y_i*space_between_points-length/2, z_i*space_between_points-height/2)))
            point_cloud = np.vstack((point_cloud,(width/2, y_i*space_between_points-length/2, z_i*space_between_points-height/2)))

    for x_i in range(int(width/space_between_points)):
        for z_i in range(int(height/space_between_points)):
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, -length/2, z_i*space_between_points-height/2)))
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, length/2, z_i*space_between_points-height/2)))
            
    print("Number of points in the point cloud =",point_cloud.shape[0])
    point_cloud_file_name = 'point_cloud_' + name_object + '.txt'
    np.savetxt('objects/' + point_cloud_file_name, point_cloud, fmt='%s') # This line can be used to store the point cloud in a file
    generate_grasp_poses(point_cloud, name_object)
    # viz_mayavi(point_cloud) #viz_mayavi prevent the generate_grasp_pose to work properly, so it always needs to be run after each call of generate_grasp_poses


def generate_cylinder(radius, height, space_between_points, name_object):

    point_cloud = np.array([])
    point_cloud = np.hstack((point_cloud,(0,0,0)))

    for radius_i in range(int(radius/space_between_points)):
        for teta_i in range(int(2*mt.pi*radius_i)):
            teta = teta_i/(int(2*mt.pi*radius_i))*2*mt.pi
            point_cloud = np.vstack((point_cloud,(mt.cos(teta)*radius_i*space_between_points, mt.sin(teta)*radius_i*space_between_points, -height/2)))
            point_cloud = np.vstack((point_cloud,(mt.cos(teta)*radius_i*space_between_points , mt.sin(teta)*radius_i*space_between_points, height/2)))
    
    for z_i in range(int(height/space_between_points)):
        for teta_i in range(int(2*mt.pi*radius/space_between_points)):
            teta = teta_i/(int(2*mt.pi*radius/space_between_points))*2*mt.pi
            point_cloud = np.vstack((point_cloud, (mt.cos(teta)*radius, mt.sin(teta)*radius, z_i*space_between_points-height/2)))
    
    print("Number of points in the point cloud =",point_cloud.shape[0])
    point_cloud_file_name = 'point_cloud_' + name_object + '.txt'
    np.savetxt('objects/' + point_cloud_file_name, point_cloud, fmt='%s') # This line can be used to store the point cloud in a file
    generate_grasp_poses(point_cloud, name_object)
    # viz_mayavi(point_cloud) #viz_mayavi prevent the generate_grasp_pose to work properly, so it always needs to be run after each call of generate_grasp_poses


def generate_milk_carton(width, length, height, height_triangle, height_plate_top, diameter_cap, height_cap, space_between_points, name_object):
    point_cloud = np.array([])
    point_cloud = np.hstack((point_cloud,(0,0,0)))

    for x_i in range(int(width/space_between_points)):
        for y_i in range(int(length/space_between_points)):
            z_i = (1-abs((x_i*space_between_points-width/2)*2/width))*height_triangle
            z_i_inside = (1-abs((y_i*space_between_points-length/2)*2/length))*height_triangle
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, y_i*space_between_points-length/2, -height/2)))
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, y_i*space_between_points-length/2, height/2+z_i)))
            if(z_i_inside < z_i):
                point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, y_i*space_between_points-length/2, height/2+z_i_inside)))

    for y_i in range(int(length/space_between_points)):
        for z_i in range(int(height_plate_top/space_between_points)):
            point_cloud = np.vstack((point_cloud,(0, y_i*space_between_points-length/2, z_i*space_between_points+height/2+height_triangle)))

    for y_i in range(int(length/space_between_points)):
        for z_i in range(int(height/space_between_points)):
            point_cloud = np.vstack((point_cloud,(-width/2, y_i*space_between_points-length/2, z_i*space_between_points-height/2)))
            point_cloud = np.vstack((point_cloud,(width/2, y_i*space_between_points-length/2, z_i*space_between_points-height/2)))

    for x_i in range(int(width/space_between_points)):
        for z_i in range(int(height/space_between_points)):
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, -length/2, z_i*space_between_points-height/2)))
            point_cloud = np.vstack((point_cloud,(x_i*space_between_points-width/2, length/2, z_i*space_between_points-height/2)))
    
    for radius_i in range(int(diameter_cap/2/space_between_points)):
        for teta_i in range(int(2*mt.pi*radius_i)):
            teta = teta_i/(int(2*mt.pi*radius_i))*2*mt.pi
            alpha = mt.atan(height_triangle*2/width)
            z_i = (mt.cos(teta)*radius_i*space_between_points-width/4-mt.sin(alpha)*height_cap)*2*height_triangle/width
            point_cloud = np.vstack((point_cloud,(mt.cos(teta)*radius_i*space_between_points-width/4-mt.sin(alpha)*height_cap, mt.sin(teta)*radius_i*space_between_points, height/2+height_triangle+z_i+mt.cos(alpha)*height_cap)))

    for z_i in range(int(height_cap/space_between_points)):
        for teta_i in range(int(2*mt.pi*diameter_cap/2/space_between_points)):
            teta = teta_i/(int(2*mt.pi*diameter_cap/2/space_between_points))*2*mt.pi
            z_i_second = (mt.cos(teta)*diameter_cap/2-width/4-mt.sin(alpha)*height_cap/2)*2*height_triangle/width
            point_cloud = np.vstack((point_cloud, (mt.cos(teta)*diameter_cap/2-width/4-mt.sin(alpha)*height_cap/2, mt.sin(teta)*diameter_cap/2, z_i_second+z_i*space_between_points+height/2+height_triangle)))

    print("Number of points in the point cloud =",point_cloud.shape[0])
    point_cloud_file_name = 'point_cloud_' + name_object + '.txt'
    np.savetxt('objects/' + point_cloud_file_name, point_cloud, fmt='%s') # This line can be used to store the point cloud in a file
    generate_grasp_poses(point_cloud, name_object)

    # viz_mayavi(point_cloud)
    

def viz_mayavi(points, vals="c"): # This function enable to visualize the point cloud of the object

    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor

    # Plot using mayavi -Much faster and smoother than matplotlib
    import mayavi.mlab

    if vals == "height":
        col = z
    else:
        col = d

    fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mayavi.mlab.points3d(x, y, z,
                         col,          # Values used for Color
                         mode="point",
                         colormap='spectral', # 'bone', 'copper', 'gnuplot'
                         # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                         figure=fig,
                         )
    mayavi.mlab.show()
    


# generate_cuboid(0.060, 0.060, 0.160, 0.01, 'Box') # 0.002
# generate_cylinder(0.0375, 0.22, 0.002, 'Bottle') # 0.002
generate_milk_carton(0.072, 0.097, 0.215, 0.02, 0.017, 0.03, 0.01, 0.002, 'Milk_carton_Coop_1.5L')
