#ifndef UTILS_ROS_UTILS_H
#define UTILS_ROS_UTILS_H

#include "utils/ros_utils.h"

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>


class RosUtils{
public:

// ------------------------------------------------------------
static Eigen::Isometry3d rosPose2EigenPose(
    const geometry_msgs::Pose ros_pose
){ 
    Eigen::Vector3d pos;
    pos << ros_pose.position.x, ros_pose.position.y, ros_pose.position.z;

    Eigen::Vector4d quat;
    quat << ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w;

    return Utils::EigenPosQuatVec2Isometry3d(pos, quat);
}


// ------------------------------------------------------------
static geometry_msgs::Pose eigenPose2RosPose(
    const Eigen::Isometry3d eigen_pose
){ 
    geometry_msgs::Pose ros_pose;

    Eigen::Vector3d pos = eigen_pose.translation();
    ros_pose.position.x = pos(0);
    ros_pose.position.y = pos(1);
    ros_pose.position.z = pos(2);

    Eigen::Quaterniond quat(eigen_pose.rotation());
    ros_pose.orientation.x = quat.coeffs()(0);
    ros_pose.orientation.y = quat.coeffs()(1);
    ros_pose.orientation.z = quat.coeffs()(2);
    ros_pose.orientation.w = quat.coeffs()(3);

    return ros_pose;
}


// ------------------------------------------------------------
static bool checkGeometryPoseInitialized(
    const geometry_msgs::Pose ros_pose
){
    if(ros_pose.position.x == 0 && ros_pose.position.y == 0 && ros_pose.position.z == 0 &&
       ros_pose.orientation.x == 0 && ros_pose.orientation.y == 0 && 
       ros_pose.orientation.z == 0 && ros_pose.orientation.w == 0)
    {
        return false;
    }
    return true;
}





}; // end class

#endif