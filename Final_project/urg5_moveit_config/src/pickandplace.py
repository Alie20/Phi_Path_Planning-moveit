#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# intialize the node 
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("pickandplace",anonymous=True)

# Activate moveGroupCommander
robot = moveit_commander.RobotCommander()
print("Starting....\n")

#Create two groups one for Arm and other for gripper
arm_group = moveit_commander.MoveGroupCommander("Arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

#Starting by opening the gripper
hand_group.set_named_target("open")
print("Open Gripper....\n")
plan2 = hand_group.go()

#Create a position variable in form geometry_msg.pose
pose_target = geometry_msgs.msg.Pose()

#Position and orientation for the point of the object 
pose_target.orientation.w = 1.0
pose_target.position.x = - 0.1
pose_target.position.y = 0.586
pose_target.position.z = 0.12

#Going to the certain position 
print("Pose Position.....\n")
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

#close the gripper
hand_group.set_named_target("semi_close")
print("Closing gripper ...\n")
plan2 = hand_group.go()

#Go to home position
arm_group.set_named_target("home")
print("Ready Position....\n")
plan1 = arm_group.go()

#open the gripper
hand_group.set_named_target("open")
print("Open Gripper....\n")
plan2 = hand_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
