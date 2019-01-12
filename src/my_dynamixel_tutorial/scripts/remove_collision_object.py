#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
import rospy
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('add_collision')
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

# p = PoseStamped()
# p.header.frame_id = robot.get_planning_frame()
# p.pose.position.x = 0
# p.pose.position.y = 0
# p.pose.position.z = 0.2
scene.remove_world_object("table")