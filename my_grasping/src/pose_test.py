#!/usr/bin/env python

import rospy
import roslib
import tf
import sys
import copy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon
import actionlib

# Initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
# Initialize node
rospy.init_node('pose_test', anonymous=True)
# Get the robot used
robot = moveit_commander.RobotCommander()
# Planning Scene
scene = moveit_commander.PlanningSceneInterface()

# ARM Move GROUP #############################
move_arm = moveit_commander.MoveGroupCommander("panda_arm")
joints = move_arm.get_joints()
arm_publisher = rospy.Publisher('/move_arm/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
scale = 1.0 
print "===== Ready Pose"
move_arm.set_named_target("ready")
plan1 = move_arm.go(wait=True)

waypoints = []

wpose = move_arm.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:

print "===== Display trajectory"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
arm_publisher.publish(display_trajectory);

valid_trajectory = raw_input('TYPE ok to execute trajectory, else press enter ')
if valid_trajectory == 'ok':
  move_arm.execute(plan)
  rospy.sleep(1)
else:
  exit()


