#!/usr/bin/env python

import rospy
import roslib
import tf
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
# Initialize node
rospy.init_node('move_demo_test', anonymous=True)
# Get the robot used
robot = moveit_commander.RobotCommander()
# Planning Scene
scene = moveit_commander.PlanningSceneInterface()

# ARM Move GROUP #############################
move_arm = moveit_commander.MoveGroupCommander("panda_arm")
joints = move_arm.get_joints()
arm_publisher = rospy.Publisher('/move_arm/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory)

# GRIPPER Move GROUP ###########################
move_hand = moveit_commander.MoveGroupCommander("hand")
hand_publisher = rospy.Publisher('/move_hand/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory)

# GRIPPER Move GROUP ###########################
move_arm_hand = moveit_commander.MoveGroupCommander("panda_arm_hand")
arm_hand_publisher = rospy.Publisher('/move_arm_hand/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory)

# Print Info ###################################
print "===== Reference frame: %s", move_arm.get_planning_frame()
print "===== End effector link: %s", move_arm.get_end_effector_link()
print "===== Available Planning Groups:", robot.get_group_names()


rate = rospy.Rate(1)
listener = tf.TransformListener()
stage = 0
restart = 1
while not rospy.is_shutdown():
  while restart == 1:
    if stage == 0:
      print "===== Stage 0: Getting into initial pose"
      # Arm go into ready pose
      move_arm.set_named_target("ready")
      plan1 = move_arm.go()
      # Open the gripper
      move_hand.set_named_target("open")
      plan2 = move_hand.go()
      stage = 1
      rospy.sleep(1) # Wait

    if stage == 1:
      print "===== Stage 1: Getting arm into pre-grasp pose"
      try:
        listener.waitForTransform('/panda_link0', '/arm_goal', rospy.Time(), rospy.Duration(20.0))      # Wait for TF
        (translation, rotation) = listener.lookupTransform('/panda_link0', '/arm_goal', rospy.Time(0))
        print "===== Object found"
        #rospy.sleep(1) # Wait
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      else:
        print "===== Go to Pre-Grasp Pose"
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = rotation[0]
        pose_goal.orientation.y = rotation[1]
        pose_goal.orientation.z = rotation[2]
        pose_goal.orientation.w = rotation[3]
        pose_goal.position.x = translation[0] 
        pose_goal.position.y = translation[1] 
        pose_goal.position.z = translation[2]
        move_arm.set_pose_target(pose_goal)
        plan_3 = move_arm.plan()

        print "===== Display trajectory"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan_3)
        # Publish
        arm_publisher.publish(display_trajectory);

        valid_trajectory = raw_input('TYPE ok to execute trajectory, else press enter ')
        if valid_trajectory == 'ok':
          move_arm.go()
          stage = 2
	  restart = 0
  	  rospy.sleep(1)
        else:
	  print "===== Restart"
          stage = 0

    if stage == 2: 
      print "===== Stage 2: Grasping"
      move_hand.set_named_target("close")
      plan4 = move_hand.go()
      stage = 3
      rospy.sleep(1)

    if stage == 3:
      print "===== Stage 3: Going back to init pose"
      move_arm.set_named_target("ready")
      plan5 = move_arm.go()
      rospy.sleep(2)

  exit()

    
    
  


