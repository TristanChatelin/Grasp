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



# def callback(grasp_poses, tfBuffer):
#     # print(f"--------------------->: {grasp_poses.markers[0]}")
#     global ur10_planner
#     # ur10_planner.planning_frame = grasp_poses.markers[0].header.frame_id
#     pose_goal_from_gpd = PoseStamped()
#     pose_goal_from_gpd.header = grasp_poses.markers[0].header
#     pose_goal_from_gpd.pose = grasp_poses.markers[0].pose
#     transformed_pose_goal = tfBuffer.transform(pose_goal_from_gpd, "base_link", rospy.Duration(1))
#     car_path = ur10_planner.plan_cartesian_path(transformed_pose_goal)
#     ur10_planner.display_trajectory(car_path[0])
#     ur10_planner.execute_plan(car_path[0])
#     # ur10_planner.go_to_default_pose()


# def listener(buffer):
#     rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback, queue_size=1, buff_size=52428800, callback_args=buffer) 
#     rospy.spin()

# if __name__ == '__main__':
#     # rospy.init_node('listener',log_level=rospy.DEBUG, anonymous=True)
#     ur10_planner = MoveGroupPython("manipulator")
#     tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
#     tfListener = tf2_ros.TransformListener(tfBuffer)
#     listener(tfBuffer)

import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def give_offset(object_pose, end_effector_pose, offset=0.15):
    """
        This function will return a pose that is the closest point some offset away
        function defined using https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point

        Basically making a circle with the radius the same as the offset, centered at the can's position.
        Then finding the point on this circle that is closest to the robot's own pose

        Parameters

        ------------------

        pose : The pose of the object we want to calculate the offset from i.e. the can's x,y,z as a PointStamped

        robot_pose : The transform to get the pose of the robot from the map frame

        offset : desired offset from pose, in m
    """
    goal_pose = object_pose
    # Defining the center point of the circle
    cx = object_pose.pose.position.x
    cy = object_pose.pose.position.y
    cz = object_pose.pose.position.z
    # Defining the vector from the robot to the centerpoint
    vx = end_effector_pose.position.x - cx
    vy = end_effector_pose.position.y - cy
    vz = end_effector_pose.position.z - cz
    # turning to unit vector
    vmag = (vx**2 + vy**2 + vz**2)**0.5
    print(vx/vmag, vy/vmag)
    # going in the direction of the vector v by the offset amount
    # hence we get a point that is the offset away from the can
    goal_pose.pose.position.x = cx + vx / vmag * offset
    goal_pose.pose.position.y = cy + vy / vmag * offset
    goal_pose.pose.position.z = cz + vz / vmag * offset
    return goal_pose

def publish_pose(marker):
    publisher = rospy.Publisher("/gpd_goal_pose", PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        publisher.publish(marker)
        rospy.sleep(0.1)

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


def callback(grasp_poses, args):
    global ur10_planner
    tfBuffer = args[0]
    grasp_goal_pub = args[1]
    pose_goal_from_gpd = PoseStamped()
    # the visualized grasp is a MarkerArray of four blocks. grasp_poses.markers[2] is the "wrist" block
    # just need to offset the depth a bit to avoid slamming the object, but can't be done with frame_id='map' 
    pose_goal_from_gpd.header = grasp_poses.markers[2].header
    pose_goal_from_gpd.pose = grasp_poses.markers[2].pose
    #ur10_planner.go_to_pose_goal(pose_goal_from_gpd)
    transformed_pose_goal = tfBuffer.transform(pose_goal_from_gpd, "MiR_footprint", rospy.Duration(1))
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
    pose_c = deepcopy(pose_a)
    pose_c.pose = get_diff(pose_a.pose, pose_b.pose)
    magn_pose_c = get_magn(pose_c.pose)
    offset = 0.25 # scaling grasp offset between gripper and object 
    pose_c.pose.position.x = pose_c.pose.position.x * offset/magn_pose_c + pose_b.pose.position.x
    pose_c.pose.position.y = pose_c.pose.position.y * offset/magn_pose_c + pose_b.pose.position.y
    pose_c.pose.position.z = pose_c.pose.position.z * offset/magn_pose_c + pose_b.pose.position.z
    pose_c.pose.orientation = deepcopy(pose_a.pose.orientation)
    offseted_pose = tfBuffer.transform(pose_c,"MiR_footprint", rospy.Duration(1))
    grasp_goal_pub.publish(offseted_pose)
    # offseted_pose = give_offset(transformed_pose_goal,ur10_planner.move_group.get_current_pose().pose)
    print(f"pose_a: {pose_a}\npose_c: {pose_c}")
    car_path = ur10_planner.plan_cartesian_path(offseted_pose)
    ur10_planner.display_trajectory(car_path[0])
    #rospy.sleep(5)
    ur10_planner.execute_plan(car_path[0])
    

if __name__ == '__main__':
    rospy.init_node("grasp_pose_subscriber", anonymous=True, log_level=rospy.ERROR)
    ur10_planner = MoveGroupPython(create_node=False)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(2)
    grasp_goal_pub = rospy.Publisher("Grasp_pose_pub", PoseStamped, queue_size=3)
    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback, queue_size=1, buff_size=52428800, callback_args=(tfBuffer, grasp_goal_pub)) 
    rospy.spin()
