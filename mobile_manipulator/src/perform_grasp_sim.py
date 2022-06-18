#!/usr/bin/env python3

import rospy
from move_group_python import MoveGroupPython 
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
import copy

def publish_pose(pose_a, pose_b, pose_c):
    publisher_a = rospy.Publisher("/marker_index_2", PoseStamped, queue_size=1)
    publisher_b = rospy.Publisher("/marker_index_3", PoseStamped, queue_size=1)
    publisher_c = rospy.Publisher("/pose_diff", PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        publisher_a.publish(pose_a)
        publisher_b.publish(pose_b)
        publisher_c.publish(pose_c)
        rospy.sleep(0.1)

def callback(grasp_poses):
    global ur10_planner
    pose_a = PoseStamped()
    # the visualized grasp is a MarkerArray of four blocks. grasp_poses.markers[2] is the "wrist" block
    # just need to offset the depth a bit to avoid slamming the object, but can't be done with frame_id='map' 
    pose_a.header = grasp_poses.markers[2].header
    pose_a.pose = grasp_poses.markers[2].pose
    pose_b = PoseStamped()
    pose_b.header = grasp_poses.markers[3].header
    pose_b.pose = grasp_poses.markers[3].pose
    pose_c = copy.deepcopy(pose_a)
    scaling = 4 # scaling grasp offset between gripper and object 
    pose_c.pose.position.x = (pose_a.pose.position.x - pose_b.pose.position.x) * scaling + pose_b.pose.position.x
    pose_c.pose.position.y = (pose_a.pose.position.y - pose_b.pose.position.y) * scaling + pose_b.pose.position.y
    pose_c.pose.position.z = (pose_a.pose.position.z - pose_b.pose.position.z) * scaling + pose_b.pose.position.z

    # ur10_planner.go_to_pose_goal(pose_goal_from_gpd)
    publish_pose(pose_a, pose_b, pose_c) # debugging 

if __name__ == '__main__':
    rospy.sleep(5)
    ur10_planner = MoveGroupPython()
    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback, queue_size=1, buff_size=52428800) 
    rospy.spin()
