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
rospy.init_node('inverse_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()
group_arm.set_goal_tolerance(0.0005)
# print "============ \nReference frame: %s" % planning_frame
# eef_link = group_arm.get_end_effector_link()
# print "End effector: %s" % eef_link
# group_names = robot.get_group_names()
# print "Goal_tolerance\n", group_arm.get_goal_tolerance()
# print "Robot Groups:", robot.get_group_names(), "\n============"


# print "Group_arm pose\n", group_arm.get_current_pose()
# print "Joint Angles\n", group_arm.get_current_joint_values()

def create_linear_path(start, goal, intermediate_points):
	waypoints = []
	# wp =  pose_to_list(group_arm.get_current_pose().pose)
	# start = wp
	arrays = []
	quat_arrays = []
	for i in range(len(start)):
		arrays.extend(range(int(start[i]*100), int(goal[i]*100)+1, (int(goal[i]*100) - int(start[i]*100))/(intermediate_points)))
	arrays = np.reshape(np.array(arrays), (6, intermediate_points+1))
	arrays  = arrays.T
	for point in arrays:
		quat = tf.transformations.quaternion_from_euler(point[3], point[4], point[5])
		quat_point =  np.concatenate((point[:3], quat), axis = 0)
		quat_arrays.append(quat_point)
	print quat_arrays
create_linear_path([0, 0, 0, 0, 0, 0], [1, 2, 3, 4, 5, 6], 10)

# wpose = group_arm.get_current_pose().pose
# wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = group_arm.compute_cartesian_path(
#                                     waypoints,   # waypoints to follow
#                                     0.01,        # eef_step
#                                     0.0)         # jump_threshold

# print "quat:", quaternion
# print "array:", end_effector_coordinate
# change_end_pose(group_arm, end_effector_coordinate)
# print "\nGroup_arm pose\n", group_arm.get_current_pose()
# print "Joint Angles\n", group_arm.get_current_joint_values()