#!/usr/bin/env python3  

import rospy
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('octomap_clearer')
    rospy.wait_for_service('/clear_octomap')
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    while not rospy.is_shutdown():
        clear_octomap()
        rospy.sleep(0.5) # 10 Hz
    rospy.spin()
