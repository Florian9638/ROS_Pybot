#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv) # to initialize the moveit_commander
rospy.init_node('move_pybot', anonymous=True) # creation of the ros node
robot = moveit_commander.RobotCommander()

group_name = "scara_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Put the arm in the 45deg_pose
move_group.set_named_target("45deg_pose")
# Plan and Execute this position
plan1 = move_group.go() # plan1 = move_group.plan() --> this command just planns without executing

# PLANNING A JOINT GOAL

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = -0.01 # prismatic_joint [m]
joint_goal[1] = 0 # shoulder [rad]
joint_goal[2] = -pi/4 # elbow [rad]

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()


'''
# CARTESIAN PATHS

# Specify a list of waypoints for the end-effector to go through
waypoints = []
scale = 1.0

wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.01  # First move up (z)
wpose.position.y += scale * 0.1  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.05  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
#return plan, fraction

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)


display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);


move_group.execute(plan, wait=True)


# PLANNING A POSE GOAL

# Create a pose message containing the coordainates of the end-effector I want to achieve
# Coordinates are based on the world_fame (virtual_joint)
#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 1.0
#pose_goal.position.x = 0.130 # 0.065 + 0.130
#pose_goal.position.y = 0.08
#pose_goal.position.z = 0.01

#move_group.set_pose_target(pose_goal)
# Plan and Execute this position (pose_goal)
#plan1 = move_group.go(wait=True)
'''


# Close the move group
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
