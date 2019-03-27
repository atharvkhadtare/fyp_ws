#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import tf

from moveit_commander.conversions import pose_to_list
def change_joint_angles(group, joint_goal):
    # joint_goal = group.get_current_joint_values()
    group.go(joint_goal, wait=True)
    group.stop()
    print "\nJoint angles moved to ", joint_goal

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('forward_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()
print "============ \nReference frame: %s" % planning_frame
eef_link = group_arm.get_end_effector_link()
print "End effector: %s" % eef_link
group_names = robot.get_group_names()
print "Robot Groups:", robot.get_group_names(), "\n============"


print "Group_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()

change_joint_angles(group_arm, [0, 0, 0, 0, 1.57, 0])


print "\nGroup_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()

print "\nGroup_arm pose\n", pose_to_list(group_arm.get_current_pose().pose)

pose_list = pose_to_list(group_arm.get_current_pose().pose)
quaternion = (pose_list[3], pose_list[4], pose_list[5], pose_list[6])

print "\norientation in quaternion: ", quaternion

# while 1:
#     pose_list = pose_to_list(group_arm.get_current_pose().pose)
#     quaternion = (pose_list[3], pose_list[4], pose_list[5], pose_list[6])
#     print "\nOrientation in Euler: ", tf.transformations.euler_from_quaternion(quaternion)
