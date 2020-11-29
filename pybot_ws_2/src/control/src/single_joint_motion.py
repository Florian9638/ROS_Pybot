#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

moveit_commander.roscpp_initialize(sys.argv) # to initialize the moveit_commander
rospy.init_node('move_pybot_2', anonymous=True) # creation of the ros node

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("scara_arm") # I specify the group of my robot that I want to move (the group was specified in the Moveit Setup Assistant)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

# PLANNING A SEQUENCE OF JOINT GOALS

# Prismatic joint
joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.02 # prismatic_joint [m]
joint_goal[1] = 0 # shoulder [rad]
joint_goal[2] = 0 # elbow [rad]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory

joint_goal[0] = 0.02 # prismatic_joint [m]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory
rospy.sleep(1)

# Shoulder joint
joint_goal[0] = 0 # prismatic_joint [m]
joint_goal[1] = 0.7 # shoulder [rad]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory

joint_goal[1] = -0.7 # shoulder [rad]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory
rospy.sleep(1)

# Elbow joint
joint_goal[1] = 0 # shoulder [rad]
joint_goal[2] = 0.7 # elbow [rad]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory

joint_goal[2] = -0.7 # elbow [rad]
group.set_joint_value_target(joint_goal)

plan = group.plan() # plan the trajectory
group.go(wait=True) # execute the trajectory

moveit_commander.roscpp_shutdown()
