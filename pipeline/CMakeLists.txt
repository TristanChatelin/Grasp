cmake_minimum_required(VERSION 3.0.2)
project(pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

catkin_install_python(PROGRAMS scripts/get_grasp_poses_from_camera_data_publisher.py scripts/get_pose_robot_publisher.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
