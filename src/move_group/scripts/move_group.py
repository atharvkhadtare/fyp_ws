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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group_arm = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group_arm.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group_arm.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing group_arm pose"
print group_arm.get_current_pose()
print ""

# print "============ Printing group_arm pose"
# print group_arm.get_current_pose()
# print ""
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0
pose_goal.position.y = 0 #-0.2668
pose_goal.position.z = 0.441 #0.3305
group_arm.set_pose_target(pose_goal)
# pose_goal = group_arm.get_random_pose()
# plan = group_arm.go(wait=True)
# # # Calling `stop()` ensures that there is no residual movement
# group_arm.stop()
print "Goal Reached"
print "============ Printing group_arm pose"
print group_arm.get_current_pose()
print ""

print "============ Printing group_arm pose"
print group_arm.get_current_joint_values()
print ""
# print "============ Printing group_arm pose"
# print group_arm.get_current_pose()
# print ""
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""

# # group_hand = moveit_commander.MoveGroupCommander("hand")
# # print "============ Printing hand pose"
# # print group_hand.get_current_pose()
# # print ""
# # # It is always good to clear your targets after planning with poses.
# # # Note: there is no equivalent function for clear_joint_value_targets()
group_arm.clear_pose_targets()