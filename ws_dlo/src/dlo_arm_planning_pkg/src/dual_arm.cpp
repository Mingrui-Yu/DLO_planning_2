#include "dlo_arm_planning_pkg/dual_arm.h"


namespace dlo_arm_planning_pkg{

// -------------------------------------------------------
DualArm::DualArm(
    const ros::NodeHandle& nh,
    const std::string &robot_description_name,
    const std::string &arm_0_group_name,
    const std::string &arm_1_group_name,
    const std::string &dual_arm_group_name
): nh_(nh)
{
    dual_arm_group_name_ = arm_group_name_ = dual_arm_group_name;
    robot_description_name_ = robot_description_name;

    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
        new robot_model_loader::RobotModelLoader(robot_description_name)); // The RobotModelLoader should be kept around. Reference: https://github.com/ros-planning/moveit/issues/2979#issuecomment-984440339

    robot_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_loader_->getModel()));
    robot_state_->setToDefaultValues();

    arm_0_ = std::make_shared<Arm>(nh_, robot_state_, arm_0_group_name);
    arm_1_ = std::make_shared<Arm>(nh_, robot_state_, arm_1_group_name);

    joint_num_ = arm_0_->joint_num_ + arm_1_->joint_num_;
    active_joint_names_ = Utils::concatenateTwoVector(arm_0_->active_joint_names_, arm_1_->active_joint_names_);

    loadParams();
}


// -------------------------------------------------------
void DualArm::loadParams()
{
    std::string param_name;

    std::string group_name = dual_arm_group_name_;
    param_name = "robot_configs/" + group_name + "/base_link_name";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, base_link_name_);
}



// -------------------------------------------------------
bool DualArm::checkDualArmJointNum(
    double dual_arm_joint_value_size,
    const std::string text
){
    bool valid = (dual_arm_joint_value_size == arm_0_->joint_num_ + arm_1_->joint_num_);
    ROS_ERROR_STREAM_COND(valid == false, text << ": the number of dual arm joints is wrong.");
    return valid;
}


// -------------------------------------------------------
void DualArm::splitTwoArmJointPos(
    const Eigen::VectorXd &dual_arm_joint_pos,
    Eigen::VectorXd &arm_0_joint_pos,
    Eigen::VectorXd &arm_1_joint_pos
){
    checkDualArmJointNum(dual_arm_joint_pos.size(), "DualArm::splitTwoArmJointPos()");

    arm_0_joint_pos = dual_arm_joint_pos.block(0, 0, arm_0_->joint_num_, 1);
    arm_1_joint_pos = dual_arm_joint_pos.block(arm_0_->joint_num_, 0, arm_1_->joint_num_, 1);
}


} // end namespace