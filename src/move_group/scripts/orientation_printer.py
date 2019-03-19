#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('inverse_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")
# group_arm.set_planner_id("SPARStwo")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()
group_arm.set_goal_tolerance(0.005)
print "============ \nReference frame: %s" % planning_frame
eef_link = group_arm.get_end_effector_link()
print "End effector: %s" % eef_link
group_names = robot.get_group_names()
print "Goal_tolerance\n", group_arm.get_goal_tolerance()
print "Robot Groups:", robot.get_group_names(), "\n============"


while(1) :
    print "\nGroup_arm pose after moving\n", group_arm.get_current_pose().pose.position

    current_orientation_in_quats = (group_arm.get_current_pose().pose.orientation.x, group_arm.get_current_pose().pose.orientation.y, group_arm.get_current_pose().pose.orientation.z, group_arm.get_current_pose().pose.orientation.w)
    current_orientation_in_euler = tf.transformations.euler_from_quaternion(current_orientation_in_quats) 
    print "\nGroup_arm orientation in euler\n", current_orientation_in_euler
    print "--------------------------------------------------------------\n"
# print "Joint Angles\n", group_arm.get_current_joint_values()


