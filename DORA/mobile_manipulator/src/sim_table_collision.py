#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
import moveit_commander

rospy.init_node('sim_table_collision')
rospy.sleep(2) # TODO: needs to wait for something to load, change hardcode

# measurements from DORA/mobile_manipulator/models/table/model.sdf
tableHeight = 1 + 0.03 / 2 
tableSize = (1.5, 0.8, tableHeight)
# rosservice call /gazebo/get_model_state [model_name] [relative_entity_name]
getState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# lookup the table model relative to map
mapToTable = getState("table", "map")
tablePose = PoseStamped()
tablePose.pose.position.x = mapToTable.pose.position.x
tablePose.pose.position.y = mapToTable.pose.position.y
tablePose.pose.position.z = tableHeight / 2
tablePose.pose.orientation.x = mapToTable.pose.orientation.x
tablePose.pose.orientation.y = mapToTable.pose.orientation.y
tablePose.pose.orientation.z = mapToTable.pose.orientation.z
tablePose.pose.orientation.w = mapToTable.pose.orientation.w
tablePose.header.frame_id = "map"
tablePose.header.stamp = rospy.Time.now()

x = moveit_commander.PlanningSceneInterface(synchronous=True)
x.add_box("table_box", tablePose, tableSize) 
