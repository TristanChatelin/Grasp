#!/usr/bin/env python3
from cmath import sqrt
from turtle import position
from tf2_geometry_msgs import PoseStamped, PointStamped
import rospy
import math
import tf2_ros
from tf.transformations import quaternion_from_euler

def give_offset(can_point, robot_pose_transform, offset=0.8):
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
    goal_pose = can_point
    # Defining the center point of the circle
    cx = can_point.point.x
    cy = can_point.point.y
    # Defining the vector from the robot to the centerpoint
    vx = robot_pose_transform.transform.translation.x - cx
    vy = robot_pose_transform.transform.translation.y - cy
    # turning to unit vector
    vmag = (vx**2 + vy**2)**0.5
    print(vx/vmag, vy/vmag)
    # going in the direction of the vector v by the offset amount
    # hence we get a point that is the offset away from the can
    goal_pose.point.x = cx + vx / vmag * offset
    goal_pose.point.y = cy + vy / vmag * offset

    return goal_pose

def go_to_can(can_point, args):
    """
        This function uses the can centerpoint and after some prep
        sets a navigation goal. This prep is getting an offset from the can 
        that is closest to the robot's current position so that the robot
        does not drive into the can/table.

        Parameters:

        ----------------

        can_point : Can centerpoint defined in point_cloud_subscriber.py, type: PointStamped

        args : triplet defined in the subscriber that includes the navigation goal publisher,
        The transform buffer to find the robot position, and the transformed can centerpoint
        publisher.
    """
    # Getting the needed arguments
    goal_pub = args[0]
    tfBuffer = args[1]
    can_tf_pub = args[2]
    # Initializing goal pose
    goal_pose = PoseStamped()
    # Using transform defined in the static transform found in the launch file mobile_demo.launch 
    # to transform can's centerpoint calculated in point_cloud_subscriber.py
    can_point_tf = tfBuffer.transform(can_point, "map",rospy.Duration(1), PointStamped)
    # Publishing this centerpoint to be used in other files
    can_tf_pub.publish(can_point_tf)
    # This transform is used to get the current position of the robot as 
    # odom_topic is not giving correct positions.


    pose_of_robot = tfBuffer.lookup_transform("map", "MiR_footprint", rospy.Time.now(), rospy.Duration(1))
    #goal_position = give_offset(can_point_tf, pose_of_robot, 0.7)
    # Since the centerpoint is a point and we need a pose,
    # we have to define a new stamped pose variable to publish
    goal_pose.header.frame_id = can_point_tf.header.frame_id
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = can_point_tf.point.x
    goal_pose.pose.position.y = can_point_tf.point.y
    goal_pose.pose.position.z = can_point_tf.point.z
    # no rotation hence only w is 1
    goal_pose.pose.orientation.w = 1
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.z = 0
    # After publishing, a delay of 15s is used to allow time 
    # for planning and execution of the plan
    goal_pub.publish(goal_pose)
    # rospy.sleep(15)
    while True:
        print(f"within loop:\npose_of_robot = {pose_of_robot}\ngoal_pose = {goal_pose}\nsqrt((x1-x2)^2+(y1-y2)^2) = {math.sqrt(math.pow(pose_of_robot.transform.translation.x - goal_pose.pose.position.x, 2) + math.pow(pose_of_robot.transform.translation.y - goal_pose.pose.position.y, 2))}\n")
        pose_of_robot = tfBuffer.lookup_transform("map", "MiR_footprint", rospy.Time.now(), rospy.Duration(1))
        if math.sqrt(math.pow(pose_of_robot.transform.translation.x - goal_pose.pose.position.x, 2) + math.pow(pose_of_robot.transform.translation.y - goal_pose.pose.position.y, 2)) < 2:
            goal_position = give_offset(can_point_tf, pose_of_robot, 0.8)
            # Since the centerpoint is a point and we need a pose,
            # we have to define a new stamped pose variable to publish
            goal_pose.header.frame_id = can_point_tf.header.frame_id
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = goal_position.point.x
            goal_pose.pose.position.y = goal_position.point.y
            goal_pose.pose.position.z = goal_position.point.z
            euler_2d_angle_from_x = math.atan2(goal_pose.pose.position.y - pose_of_robot.transform.translation.y, goal_pose.pose.position.x - pose_of_robot.transform.translation.x) 
            quat = quaternion_from_euler(0, 0, euler_2d_angle_from_x)
            # no rotation hence only w is 1
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            # After publishing, a delay of 15s is used to allow time 
            # for planning and execution of the plan
            goal_pub.publish(goal_pose)
            break
        rospy.sleep(0.2)
    rospy.signal_shutdown("navigation goal set. shutting down goal subscriber ...")

if __name__ == "__main__":
    rospy.init_node("navigation_publisher")
    # Defining the needed variables for the transform procedure to be done later
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # Delay of 2s to allow time for everything to initialize
    rospy.sleep(2)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=4)
    can_tf_pub = rospy.Publisher("/transformed_can_centerpoint", PointStamped, queue_size=4)
    rate = rospy.Rate(5)
    # waits untill the publisher has some connections
    while goal_pub.get_num_connections() == 0:
        rate.sleep()
    rospy.Subscriber("can_centerpoint",PointStamped, callback=go_to_can, callback_args=(goal_pub, tfBuffer, can_tf_pub))
    rospy.spin()
