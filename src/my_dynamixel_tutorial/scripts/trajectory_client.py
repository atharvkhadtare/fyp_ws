#!/usr/bin/env python
import roslib
roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

joints = ["shoulder_yaw", "shoulder_pitch", "elbow", "wrist_pitch", "wrist_roll"]

class Joint:
        def __init__(self, motor_name):

            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

        def move_joint(self, points):
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = joints
            for angles in points:
                point = JointTrajectoryPoint()
                point.positions = angles
                point.time_from_start = rospy.Duration(3)
                goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

def main():
            arm = Joint('arm')
            # arm.move_joint([-3.14])
            arm.move_joint([[2.9022916506796332, 2.040194447887903, -0.06902913545485385, 0.1227184630308513, 0.0],
            [2.78724309158821, 1.4097283440669042, -0.9909515889741242, 0.10226538585904275, 0.0], 
            [0, 0, 0, 0, 0],
            [0, 0, 0, -0.57, 0.57]])

if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()