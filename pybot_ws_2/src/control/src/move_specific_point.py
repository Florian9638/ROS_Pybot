#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

moveit_commander.roscpp_initialize(sys.argv) # to initialize the moveit_commander
rospy.init_node('move_pybot_3', anonymous=True) # creation of the ros node

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("scara_arm")
# group_name = "scara_arm"
# move_group = moveit_commander.MoveGroupCommander(group_name) # I specify the group pf my robot that I want to move
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

# MOVE TO A SPECIFIC POINT
# Create a pose message containing the coordainates of the end-effector I want to achieve
# Coordinates are based on the world_fame (virtual_joint)
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = -3.1787e-06
pose_goal.orientation.w = 0.99999999 # current: W = 1.0
pose_goal.position.x = 0.281 # 0.174 # current: x = 0.185
pose_goal.position.y = -0.064 # -0.045 # current: y = 0
pose_goal.position.z = 0.049 # 0.004 # current: z = 0

group.set_pose_target(pose_goal)
#group.set_planner_id("RRTConnectkConfigDefault")
#group.set_planning_time(20)

# Plan and Execute this position (pose_goal)
#plan1 = group.go(wait=True)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory

#group.stop()
#group.clear_pose_targets()

moveit_commander.roscpp_shutdown()
