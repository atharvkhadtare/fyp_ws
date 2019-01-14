#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "forward_kinematics_cpp");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // static const std::string PLANNING_GROUP = "arm";
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("shoulder_yaw_motor");
    // visual_tools.deleteAllMarkers();

    // visual_tools.loadRemoteControl();

    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // visual_tools.trigger();

    // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //       std::ostream_iterator<std::string>(std::cout, ", "));
    
    ros::shutdown();
    return 0;
}