#!/usr/bin/env python3

import rospy
from move_group_python import MoveGroupPython
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
import geometry_msgs.msg as msg
from navigation_goal import give_offset
from math import tau

if __name__ == '__main__':
    # Defining planner & transforms
    ur10_planner = MoveGroupPython()
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    listener = tf2_ros.TransformListener(tfBuffer)
    # Getting the can's calculated centerpoint
    # Publisher can be found in navigation_goal.py
    can_point = rospy.wait_for_message("/transformed_can_centerpoint", PointStamped, rospy.Duration(100))
    # Delay to allow for the robot to drive up to the table
    # To avoid using a hardcoded sleep time, can define an
    # Algorithm that only moves the arm when it is close enough 
    # so something like a precheck on the points given the robot's
    # Current position
    rospy.sleep(7)
    # Getting the can point relative to the MiR footprint
    point_goal = tfBuffer.transform(can_point, "MiR_footprint", rospy.Duration(1), PointStamped)
    # The transform above is not exactly corrent  so the transform defined below is used to correct it
    robot_pose = tfBuffer.lookup_transform("MiR_footprint", "map", rospy.Time.now(), rospy.Duration(1))
    # Pose_goal is the final pose that the end effector should go to
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = point_goal.header.frame_id
    pose_goal.header.stamp = rospy.Time.now()

    offsetted_point = give_offset(point_goal, robot_pose, 0.2)
    corrected_offsetted_point = PointStamped()
    corrected_offsetted_point.point.x = offsetted_point.point.x + robot_pose.transform.translation.x
    corrected_offsetted_point.point.y = offsetted_point.point.y + robot_pose.transform.translation.y
    corrected_offsetted_point.point.z = offsetted_point.point.z
    pose_goal.pose.position.x = corrected_offsetted_point.point.x
    pose_goal.pose.position.y = corrected_offsetted_point.point.y
    pose_goal.pose.position.z = corrected_offsetted_point.point.z
    # Currently no rotation
    pose_goal.pose.orientation.w = 1
    # used for Debugging
    print(f"#########################\n\n{robot_pose}\n\n#########################")
    print(f"#########################\n\n{offsetted_point}\n\n#########################")
    print(f"#########################\n\n{corrected_offsetted_point}\n\n#########################")
    print(f"#########################\n\n{can_point}\n\n#########################")
    print(f"#########################\n\n{point_goal}\n\n#########################")
    print(f"#########################\n\n{pose_goal}\n\n#########################")
    ur10_planner.go_to_pose_goal(pose_goal)
    rospy.sleep(10)
    bh282_planner = MoveGroupPython("bh282_gripper")
    ang = tau/5
    angles = [ang, ang, ang/2, ang, ang, ang/2, ang, ang/2]
    bh282_planner.go_to_joint_state(angles)