#!/usr/bin/env python3

from math import sqrt
from copy import copy
import rospy
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

robot_est_pose = PoseStamped()
robot_est_old_pose = PoseStamped()
goal_pose = PoseStamped()


def main_loop(goalPose):
    global goal_pose
    goal_pose = copy(goalPose)

    

def calc_distance(pose1, pose2):
    x1 = pose1.pose.position.x
    x2 = pose2.pose.position.x
    y1 = pose1.pose.position.y
    y2 = pose2.pose.position.y
    dist = sqrt((x1-x2)**2+(y1-y2)**2)
    return dist

def get_time(time_init):
    current_time = time.time()
    time_taken = current_time - time_init 
    return time_taken

def update_current_pose(current_pose):
    global robot_est_pose, robot_est_old_pose
    robot_est_pose.header = current_pose.header
    robot_est_pose.pose.position = current_pose.pose.pose.position
    robot_est_pose.pose.orientation = current_pose.pose.pose.orientation



if __name__ == "__main__":
    rospy.init_node("Navigation_stack_tester")
    rospy.loginfo("Beginning test")
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, update_current_pose, queue_size=4)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, main_loop, queue_size=4)
    tol_time = 5
    tol_dist = 0.3
    reported_time = 0
    while goal_pose.header.seq == 0: pass
    tstart = time.time()
    old_dist = 0
    diff_dist = 0
    test_num = 1
    old_diff = 0
    diff_found = False
    while(not rospy.is_shutdown()):
        current_seq = robot_est_pose.header.seq
        current_time = robot_est_pose.header.stamp
        current_time = rospy.Time.to_sec(current_time) + tstart
        dist_diff_temp = calc_distance(goal_pose, robot_est_pose)
        if dist_diff_temp < tol_dist:
            reported_time = get_time(tstart)
            if reported_time > tol_time:
                #print(robot_est_old_pose, robot_est_pose)
                diff_dist = calc_distance(goal_pose, robot_est_pose)
                rospy.loginfo("----------------------------------")
                rospy.loginfo("here are results of test number %s:", test_num)
                rospy.loginfo("time taken: %s", reported_time)
                rospy.loginfo("----------------------------------")
                rospy.loginfo("distance offset: %s", diff_dist)
                rospy.loginfo("----------------------------------")
                test_num += 1
                while(dist_diff_temp == calc_distance(goal_pose, robot_est_pose)):
                    pass
                tstart = time.time()
                    
                
                
        # check if robot stopped moving
        # current_dist = abs(calc_distance(robot_est_pose, robot_est_old_pose)) 
        #print(current_dist)
        #print("-------------")
        #print("current: ",robot_est_pose)
        # print("-------------")
        # print("old", robot_est_old_pose)
        # print("-------------")
        # if current_dist < tol:

        
    rospy.spin()