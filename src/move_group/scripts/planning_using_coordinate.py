#!/usr/bin/env python

from custom_msg.msg import array_float
from custom_msg.msg import array
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose
import tf
from dynamixel_msgs.msg import JointState

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planning_using_coordinate', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_arm.set_planner_id("TRRT")

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

print "Group_arm pose\n", group_arm.get_current_pose()
print "Joint Angles\n", group_arm.get_current_joint_values()

object_coordinate = [0, 0, 0]

gripper_length = 0.08
waypoints = []
scale = 0.01
wpose = group_arm.get_current_pose().pose
waypoints.append(copy.deepcopy(wpose))

flag = 2

def change_end_pose(group, input_pose):
	end_goal = pose_to_list(group.get_current_pose().pose)
	for i, ele in enumerate(input_pose):
		if ele != None:
			end_goal[i] = ele
	
	group.set_pose_target(list_to_pose(end_goal))
	if (group.go(wait=True)):
			print "\nSolution found for ", input_pose
			print "\nGroup moved to ", end_goal
	else:
			print "\nNo solution found for ", input_pose
	group.stop()

def calc_gripper_offset_x(kinect_data, kinect_offset):
	object_coordinate[0] = -kinect_data.array[0] - kinect_offset[0]
	object_coordinate[1] = kinect_data.array[2] - kinect_offset[1] 
	object_coordinate[2] = kinect_data.array[1] - kinect_offset[2] 
	theta = math.atan2(abs(object_coordinate[1]), abs(object_coordinate[0]))
	print(gripper_length*math.cos(theta))
	return (gripper_length*math.cos(theta))


def calc_gripper_offset_y(kinect_data, kinect_offset):
	object_coordinate[0] = -kinect_data.array[0] - kinect_offset[0]
	object_coordinate[1] = kinect_data.array[2] - kinect_offset[1]
	object_coordinate[2] = kinect_data.array[1] - kinect_offset[2] 
	theta = math.atan2(abs(object_coordinate[1]), abs(object_coordinate[0]))
	print(gripper_length*math.sin(theta))
	return (gripper_length*math.sin(theta))


def on_coordinate_array(kinect_data):
	global flag
	kinect_offset = [0.03, 0.09, -0.18] #[0.03, 0.06, -0.18]
	global object_coordinate

	if (not(math.isnan(kinect_data.array[0])) and not(math.isnan(kinect_data.array[1])) and not(math.isnan(kinect_data.array[2]))):
		flag = flag - 1
	
		if (flag == 1):
			object_coordinate[0] = -kinect_data.array[0] - (kinect_offset[0] - calc_gripper_offset_x(kinect_data, kinect_offset))
			object_coordinate[1] = kinect_data.array[2] - (kinect_offset[1] + calc_gripper_offset_y(kinect_data, kinect_offset)) 
			object_coordinate[2] = kinect_data.array[1] - kinect_offset[2]
			
			# print(kinect_data.array[0])
			# print(kinect_data.array[2])
			# print(-1*kinect_data.array[1])

			# will be in our frame of reference accepted by moveit
			print "KINECT COORDINATE (our ref): ",  kinect_data.array[0] , "\t", kinect_data.array[2] , "\t", kinect_data.array[1] 
			print "OBJECT COORDINATE (our ref): ",object_coordinate[0], "\t", object_coordinate[1], "\t", object_coordinate[2]

			waypoints = []
			wpose = group_arm.get_current_pose().pose

			# end_effector_coordinate = [object_coordinate[0], object_coordinate[1], object_coordinate[2]]
			# # end_effector_orientatation = [wpose.orientation]
			# # quaternion = tf.transformations.quaternion_from_euler(end_effector_orientatation[0],end_effector_orientatation[1],end_effector_orientatation[2])
			# end_effector_coordinate.extend(wpose.orientation)

			# change_end_pose(group_arm, end_effector_coordinate)

			waypoints.append(copy.deepcopy(wpose))

			wpose.position.x = object_coordinate[0]
			wpose.position.y = object_coordinate[1]
			wpose.position.z = object_coordinate[2]
			waypoints.append(copy.deepcopy(wpose))

			print waypoints
			display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
														moveit_msgs.msg.DisplayTrajectory,
														queue_size=20)

			(plan, fraction) = group_arm.compute_cartesian_path(
											waypoints,		# waypoints to follow
											0.001,			# eef_step
											5)				# jump_threshold (10 is Optimum)

			group_arm.execute(plan, wait=True)
			if (fraction < 0.95):
   					print "\033[0;31;40m #############################  Fraction  ############################# : ", fraction, "\033[0;37;40m"
			else:
    				print "\033[0;32;40m #############################  Fraction  ############################# : ", fraction, "\033[0;37;40m"

			sys.exit(1)
	else:
		flag = 2
		print("\033[0;31;40m *****************************NAN***************************** \033[0;37;40m")

def listener():
        rospy.Subscriber("/position", array_float, on_coordinate_array)

def clean_exit():
        exit()
        
if __name__ == "__main__":
    listener()
    rospy.spin()
    rospy.on_shutdown(clean_exit)

