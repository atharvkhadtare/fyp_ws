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


waypoints = []
scale = 0.01
wpose = group_arm.get_current_pose().pose
waypoints.append(copy.deepcopy(wpose))

# wpose.orientation.x = 0.0
# wpose.orientation.y = 0.0
# wpose.or
# ientation.z = 0.0
# wpose.orientation.w = 0.0

# wpose.position.z -= scale * 10  # and sideways (y)
wpose.position.x +  = scale * 10
# wpose.position.y -= scale * 6
waypoints.append(copy.deepcopy(wpose))
# wpose.position.y -= scale * 25
# waypoints.append(copy.deepcopy(wpo se))
# wpose.position.x -= scale * 25
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.y += scale * 10
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.x -= scale * 5
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.x += scale * 8  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.z -= scale * 8  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

print waypoints
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
(plan, fraction) = group_arm.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.001,        # eef_step
                                5) # 10 is optimum         # jump_threshold

print "fraction :", fraction

# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# # Publish
# display_trajectory_publisher.publish(display_trajectory)


# print plan
group_arm.execute(plan, wait=True)
# print "quat:", quaternion
# print "array:", end_effector_coordinate
# change_end_pose(group_arm, end_effector_coordinate)
# print "\nGroup_arm pose\n", group_arm.get_current_pose()
# print "Joint Angles\n", group_arm.get_current_joint_values()