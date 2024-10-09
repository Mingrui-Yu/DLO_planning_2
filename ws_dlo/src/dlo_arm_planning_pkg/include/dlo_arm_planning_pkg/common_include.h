#ifndef DLO_ARM_PLANNING_COMMON_INCLUDE_H
#define DLO_ARM_PLANNING_COMMON_INCLUDE_H


#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <memory>
#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <complex>

// cnpy
#include "cnpy.h"
// glog
#include "glog/logging.h"
// yaml-cpp
#include "yaml-cpp/yaml.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense> 
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>


// ROS
#include <ros/ros.h>
#include <ros/console.h>

// ros msg
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlanningScene.h>

// MoveIt
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

// custom utils
#include "utils/utils.h"
#include "utils/ros_utils.h"





#endif