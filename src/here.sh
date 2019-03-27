rostopic pub -1 /gripper_controller/command std_msgs/Float64 -- 1 &&
rosrun move_group inverse_kinematics3.py &&
rosrun move_group inverse_kinematics.py &&
rostopic pub -1 /gripper_controller/command std_msgs/Float64 -- 0.3 &&
rosrun move_group inverse_kinematics3.py &&
rosrun move_group inverse_kinematics2.py &&
rostopic pub -1 /gripper_controller/command std_msgs/Float64 -- 1 &&
rosrun move_group forward_kinematics.py &&
rostopic pub -1 /gripper_controller/command std_msgs/Float64 -- 0
