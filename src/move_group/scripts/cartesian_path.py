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

def change_end_pose(group, input_pose):
    # end_goal = pose_to_list(group.get_current_pose().pose)
    # for i, ele in enumerate(input_pose):
    #     if ele != None:
    #         end_goal[i] = ele
    # print  "\nOUT: ", input_pose, end_goal
    group.set_pose_target(input_pose)
    if group.go(wait=True):
    # if group.plan():
        print "\nSolution found for ", input_pose
        print "\nGroup moved to ", end_goal
    else:
        print "\nNo solution found for ", input_pose
    # group.stop()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian_path', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()
group_arm.set_goal_tolerance(0.01)

euler = tf.transformations.euler_from_quaternion([0.999627230251, -0.00492478584418, -0.0212156416703, 0.0164634010597])
def create_linear_path(goal, intermediate_points):
	waypoints = []
	wp =  pose_to_list(group_arm.get_current_pose().pose)
	start = wp[:3]
	start.extend(euler)
	print "wp = ", wp
	print "start = ", start
	arrays = []
	quat_arrays = []
	for i in range(len(start)):
		# arrays.extend(range(int(start[i]*100), int(goal[i]*100)+1, (int(goal[i]*100) - int(start[i]*100))*1.0/(intermediate_points)))
		arrays.extend(np.linspace(start[i], goal[i], (goal[i] - start[i])/(intermediate_points)))
	print "\n\narrays = ", np.shape(arrays)
	print "\narrays = ", arrays
	arrays = np.reshape(np.array(arrays), (6, intermediate_points+1))
	arrays  = arrays.T
	for point in arrays:
		quat = tf.transformations.quaternion_from_euler(point[3], point[4], point[5])
		quat_point =  np.concatenate((point[:3], quat), axis = 0)
		quat_arrays.append(quat_point)
	print quat_arrays
	waypoints = []
	for point in quat_arrays:
		waypoints.append(list_to_pose(point))
	print waypoints
	return waypoints
# create_linear_path([0, 0.15, , 0, 0, 0], [0.1, 2, 3, 4, 5, 6], 5)
end_goal = [0.195676452935, 0.15030685154, 0.148892068197]
end_goal.extend(euler)
change_end_pose(group_arm, create_linear_path(end_goal, 5))

# print "quat:", quaternion
# print "array:", end_effector_coordinate
# change_end_pose(group_arm, end_effector_coordinate)
# print "\nGroup_arm pose\n", group_arm.get_current_pose()
# print "Joint Angles\n", group_arm.get_current_joint_values()