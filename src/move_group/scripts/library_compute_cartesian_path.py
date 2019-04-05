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
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('library_compute_cartesian_path', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()

group_arm.set_goal_orientation_tolerance(0.0001)

waypoints = []
scale = 0.01
wpose = group_arm.get_current_pose().pose
waypoints.append(copy.deepcopy(wpose))


if (len(sys.argv) == 5):
    if (sys.argv[1] == "rel" or sys.argv[1] == "r"):
        wpose.position.x += scale * int(sys.argv[2])  # and sideways (y)
        wpose.position.y += scale * int(sys.argv[3])
        wpose.position.z += scale * int(sys.argv[4])
        waypoints.append(copy.deepcopy(wpose))
    elif(sys.argv[1] == "abs" or sys.argv[1] == "a"):
        wpose.position.x = scale * int(sys.argv[2])  # and sideways (y)
        wpose.position.y = scale * int(sys.argv[3])
        wpose.position.z = scale * int(sys.argv[4])
        waypoints.append(copy.deepcopy(wpose))
else:
    print "\033[0;31;40m Enter 3 Arguments x y z \033[0;37;40m"

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

(plan, fraction) = group_arm.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.001,       # eef_step
                                10)           # jump_threshold (10 is optimum)

if (fraction < 0.95):
    print "\033[0;31;40m #############################  Fraction  ############################# : ", fraction, "\033[0;37;40m"
else:
    print "\033[0;32;40m #############################  Fraction  ############################# : ", fraction, "\033[0;37;40m"

group_arm.execute(plan, wait=True)
