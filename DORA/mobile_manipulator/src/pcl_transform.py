#!/usr/bin/env python  

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('pcl_transform')
    pcl_frame = "camera_depth_frame" # transform this ...
    to_frame = rospy.get_param("~to_frame", "UR10_fore_arm_link") # ... relative to this (second arg default, first passed from .launch)

    listener = tf.TransformListener()
    broadcaster = tf2_ros.TransformBroadcaster()
    ts = geometry_msgs.msg.TransformStamped()

    while not rospy.is_shutdown():
        listener.waitForTransform(pcl_frame, to_frame, rospy.Time(), rospy.Duration(3.0))
        t = listener.getLatestCommonTime(to_frame, pcl_frame)
        trans, rot = listener.lookupTransform(to_frame, pcl_frame, t)

        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = to_frame
        ts.child_frame_id = pcl_frame 
        ts.transform.translation.x = trans[0]
        ts.transform.translation.y = trans[1]
        ts.transform.translation.z = trans[2]
        ts.transform.rotation.x = rot[0]
        ts.transform.rotation.y = rot[1]
        ts.transform.rotation.z = rot[2]
        ts.transform.rotation.w = rot[3]
        broadcaster.sendTransform(ts)

        rospy.sleep(0.01) # 100 Hz
    rospy.spin()
