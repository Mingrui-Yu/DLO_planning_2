#ifndef DLO_ARM_PLANNING_DUAL_ARM_H
#define DLO_ARM_PLANNING_DUAL_ARM_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/arm.h"


namespace dlo_arm_planning_pkg{

class DualArm
{
public:
    typedef std::shared_ptr<DualArm> Ptr;

    DualArm(
        const ros::NodeHandle& nh,
        const std::string &robot_description_name,
        const std::string &arm_0_group_name,
        const std::string &arm_1_group_name,
        const std::string &dual_arm_group_name
    );

    void loadParams();

    bool checkDualArmJointNum(
        double dual_arm_joint_value_size,
        const std::string text = std::string("")
    );

    void splitTwoArmJointPos(
        const Eigen::VectorXd &dual_arm_joint_pos,
        Eigen::VectorXd &arm_0_joint_pos,
        Eigen::VectorXd &arm_1_joint_pos
    );


public:
    ros::NodeHandle nh_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotStatePtr robot_state_;
    
    Arm::Ptr arm_0_, arm_1_;

    std::string robot_description_name_;
    std::string dual_arm_group_name_, arm_group_name_;
    std::string base_link_name_;
    int joint_num_;
    std::vector<std::string> active_joint_names_;

}; // end class



} // end namespace

#endif