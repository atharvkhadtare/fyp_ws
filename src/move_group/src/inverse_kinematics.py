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
def change_end_pose(group, input_pose):
    end_goal = pose_to_list(group.get_current_pose().pose)
    for i, ele in enumerate(input_pose):
        if ele != None:
            end_goal[i] = ele
    # print  "\nOUT: ", input_pose, end_goal
    group.set_pose_target(list_to_pose(end_goal))
    if group.go(wait=True):
        print "\nSolution found for ", input_pose
        print "\nGroup moved to ", end_goal
    else:
        print "\nNo solution found for ", input_pose
    group.stop()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('inverse_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
planning_frame = group_arm.get_planning_frame()
group_arm.set_goal_tolerance(0.1)
print "============ \nReference frame: %s" % planning_frame
eef_link = group_arm.get_end_effector_link()
print "End effector: %s" % eef_link
group_names = robot.get_group_names()
print "Goal_tolerance\n", group_arm.get_goal_tolerance()
print "Robot Groups:", robot.get_group_names(), "\n============"


print "Group_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()

# change_end_pose(group_arm, [0.0, None, 0.5196, 0.0, 0.0, None, 1.0])
change_end_pose(group_arm, [0, 0.015, 0.5196, 0, 0, 0, 1.0])
change_end_pose(group_arm, [0, -0.4559887551713903, 0.07897818182406445, 0.706845410317391, 0, 0, 0.707368055252284]
)
change_end_pose(group_arm, [0.127, 0.0, None, None, -0.037, 0.679, 0.729])
change_end_pose(group_arm, [0.07, 0.9, 0.383, -0.076, -0.037, 0.679, 0.729])

print "\nGroup_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()
# print "get_goal_joint_tolerance\n", group_arm.get_goal_joint_tolerance()
# print "get_goal_position_tolerance\n", group_arm.get_goal_position_tolerance()
# print "get_goal_orientation_tolerancees\n", group_arm.get_goal_orientation_tolerance()