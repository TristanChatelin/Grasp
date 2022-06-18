#!/usr/bin/env python3

from copy import deepcopy
import rospy
import tf2_ros
from visualization_msgs.msg import MarkerArray
import rospy
from move_group_python import MoveGroupPython 
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np # Scientific computing library for Python
    

if __name__ == '__main__':
    rospy.init_node("gripper_controller", anonymous=True, log_level=rospy.ERROR)
    rg6_planner = MoveGroupPython("rg6_gripper", create_node=False)
    print(rg6_planner.move_group.get_current_joint_values())
    while not rospy.is_shutdown():
        input("press a key to open the gripper")
        
        angles = [0, -0.454, 0, 0, 0.454, 0]
        rg6_planner.go_to_joint_state(angles)

        input("press a key to close the gripper")
        
        angles = [0, 0.140, 0, 0, -0.140, 0]
        rg6_planner.go_to_joint_state(angles)
    rospy.spin()
    # rg6_planner.test_each_joint()
