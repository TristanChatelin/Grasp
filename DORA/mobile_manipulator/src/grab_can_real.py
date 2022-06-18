#!/usr/bin/env python3

import rospy
from move_group_python import MoveGroupPython
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
from visualization_msgs.msg import MarkerArray
import geometry_msgs.msg as msg
from math import tau
import copy

def give_offset(can_pose, robot_pose_transform, offset=1.3):
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
    goal_pose = copy.copy(can_pose)
    # Defining the center point of the circle
    cx = can_pose.pose.position.x
    cy = can_pose.pose.position.y
    # Defining the vector from the robot to the centerpoint
    vx = robot_pose_transform.transform.translation.x - cx
    vy = robot_pose_transform.transform.translation.y - cy
    # turning to unit vector
    vmag = (vx**2 + vy**2)**0.5
    print(vx/vmag, vy/vmag)
    # going in the direction of the vector v by the offset amount
    # hence we get a point that is the offset away from the can
    goal_pose.pose.position.x = cx + vx / vmag * offset
    goal_pose.pose.position.y = cy + vy / vmag * offset

    return goal_pose

if __name__ == '__main__':
    # Defining planner & transforms
    ur10_planner = MoveGroupPython("manipulator")
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
    listener = tf2_ros.TransformListener(tfBuffer)
    # Getting the can's calculated centerpoint
    # Publisher can be found in navigation_goal.py
    marker_array = rospy.wait_for_message("/plot_grasps", MarkerArray, rospy.Duration(100))
    can_pose = copy(marker_array.pose)
    # Delay to allow for the robot to drive up to the table
    # To avoid using a hardcoded sleep time, can define an
    # Algorithm that only moves the arm when it is close enough 
    # so something like a precheck on the points given the robot's
    # Current position
    rospy.sleep(7)



    # ----MUST BE CHANGED---- Getting the can point relative to the MiR footprint
    pose_goal_initial = tfBuffer.transform(can_pose, "MiR_footprint", rospy.Duration(1), PoseStamped)
    # ----MUST BE CHANGED---- The transform above is not exactly corrent  so the transform defined below is used to correct it
    robot_pose = tfBuffer.lookup_transform("MiR_footprint", "map", rospy.Time.now(), rospy.Duration(1))


    # Pose_goal is the final pose that the end effector should go to
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = pose_goal_initial.header.frame_id
    pose_goal.header.stamp = rospy.Time.now()
    
    # offsetted_pose is the pose_goal with a given offset, function defined at start of file.
    offsetted_pose = give_offset(pose_goal_initial, robot_pose, 0.2)

    # corrected_offsetted_pose is the pose relative to the robot's current position (uses the footprint frame/link)
    corrected_offsetted_pose = PointStamped()
    corrected_offsetted_pose.position.x = offsetted_pose.pose.position.x + robot_pose.transform.translation.x
    corrected_offsetted_pose.position.y = offsetted_pose.pose.position.y + robot_pose.transform.translation.y
    corrected_offsetted_pose.position.z = offsetted_pose.pose.position.z
    #update the pose goal with the values for corrected_offsetted_pose
    pose_goal.pose.position.x = corrected_offsetted_pose.position.x
    pose_goal.pose.position.y = corrected_offsetted_pose.position.y
    pose_goal.pose.position.z = corrected_offsetted_pose.position.z
    # Currently no rotation
    pose_goal.pose.orientation.w = 1
    # used for Debugging
    print(f"#########################\n\n{robot_pose}\n\n#########################")
    print(f"#########################\n\n{offsetted_pose}\n\n#########################")
    print(f"#########################\n\n{corrected_offsetted_pose}\n\n#########################")
    print(f"#########################\n\n{can_pose}\n\n#########################")
    print(f"#########################\n\n{pose_goal_initial}\n\n#########################")
    print(f"#########################\n\n{pose_goal}\n\n#########################")
    ur10_planner.go_to_pose_goal(pose_goal)
    rospy.sleep(10)

