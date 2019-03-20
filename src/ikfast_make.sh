#!/bin/bash
cd ~/fyp_ws/src/mobile_manipulator/model
rosrun xacro xacro --inorder -o mobile_manipulator.urdf mobile_manipulator.xacro
echo sra | sudo -S pkill ros
export MYROBOT_NAME="mobile_manipulator"
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".xacro
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae
export PLANNING_GROUP="arm"
openrave-robot.py "$MYROBOT_NAME".dae --info links
read -p 'base_link: ' BASE_LINK
read -p 'end_link: ' EEF_LINK
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp
rm -f ./ikfast61_"$PLANNING_GROUP".cpp
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin

cd ~/fyp_ws/src
catkin clean -y
rm -rf ./mobile_manipulator_ikfast_arm_plugin
catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"
catkin build
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"
source ../devel/setup.bash

