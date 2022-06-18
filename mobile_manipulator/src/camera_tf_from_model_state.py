#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from gazebo_msgs.srv import GetModelState
from grasp_pose_subscriber import euler_from_quaternion, get_quaternion_from_euler
 
if __name__ == '__main__':
    # Starting the node and the publisher
    rospy.init_node('camera_tf_from_model_state')
    # This node will publish messages of type tf2_msgs.msg.TFMessage into the topic /tf 

    # main loop where data is published
    # if shutdown process not initiated, while code loops
    while not rospy.is_shutdown():
        # run code once every 1 sec, i.e. with rate of 1Hz
        rospy.sleep(1)
        # creating transform message with time stamp
        t = geometry_msgs.msg.TransformStamped()

        # rosservice call /gazebo/get_model_state [model_name] [relative_entity_name]
        getState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # lookup the kinect model relative to map
        mapToCamera = getState("kinect", "map")

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"
        transformer = tf2_ros.TransformBroadcaster()
        
        # defining transform. Variable names are self-explanatory
        # just rerouting the pose data from gazebo model state to tf
        t.transform.translation.x = mapToCamera.pose.position.x
        t.transform.translation.y = mapToCamera.pose.position.y
        t.transform.translation.z = mapToCamera.pose.position.z
        
        euler = euler_from_quaternion(mapToCamera.pose.orientation.x, mapToCamera.pose.orientation.y, mapToCamera.pose.orientation.z, mapToCamera.pose.orientation.w)
        ang = [euler[0], euler[1], euler[2]]
        ang[1] -= math.pi/2
        ang[0] += math.pi/2
        ang[2] += math.pi/2

        quat = get_quaternion_from_euler(ang[0], ang[1], ang[2])

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # t.transform.rotation.x = mapToCamera.pose.orientation.x
        # t.transform.rotation.y = mapToCamera.pose.orientation.y
        # t.transform.rotation.z = mapToCamera.pose.orientation.z
        # t.transform.rotation.w = mapToCamera.pose.orientation.w
        transformer.sendTransform(t)

    # "spin() code simply sleeps until the is_shutdown() flag is True. 
    # It is mainly used to prevent your Python Main thread from exiting."
    rospy.spin()