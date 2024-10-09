#ifndef DLO_ARM_PLANNING_Arm_H
#define DLO_ARM_PLANNING_Arm_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/arm_reach_space.h"

namespace dlo_arm_planning_pkg
{

    /**
     * @brief Class for one single arm: arm information and kinematics
     */

    // -------------------------------------------------------
    class Arm
    {
    public:
        typedef std::shared_ptr<Arm> Ptr;

        Arm(
            const ros::NodeHandle &nh,
            const std::string &robot_description_name,
            const std::string &group_name);

        Arm(
            const ros::NodeHandle &nh,
            moveit::core::RobotStatePtr &robot_state,
            const std::string &group_name);

        void loadParams();

        void printArmInfo();

        void setJointPositions(const Eigen::VectorXd &joint_pos);

        Eigen::VectorXd randomJointPos();

        bool armTcpRandomIK(
            const Eigen::Isometry3d &target_pose,
            Eigen::VectorXd &result_joint_pos);

        int armTcpAnalyticalIK(
            const Eigen::Isometry3d &ee_pose_in_world,
            std::vector<std::vector<double>> &valid_ik_solutions);

        int armEndEffectorAnalyticalIK(
            const Eigen::Isometry3d &ee_pose_in_world,
            std::vector<std::vector<double>> &valid_ik_solutions);

        bool armTcpClosestIK(
            const Eigen::VectorXd &ref_joint_pos,
            const Eigen::Isometry3d &target_pose,
            Eigen::VectorXd &result_joint_pos);

        bool armEndEffectorClosestIK(
            const Eigen::VectorXd &ref_joint_pos,
            const Eigen::Isometry3d &target_pose,
            Eigen::VectorXd &result_joint_pos);

        bool armTcpIterativeIK(
            const Eigen::VectorXd &ref_joint_pos,
            const Eigen::Isometry3d &target_pose,
            Eigen::VectorXd &result_joint_pos);

        bool armEndEffectorIterativeIK(
            const Eigen::VectorXd &ref_joint_pos,
            const Eigen::Isometry3d &target_pose,
            Eigen::VectorXd &result_joint_pos);

        bool validateIKSolution(
            robot_state::RobotState *robot_state,
            const robot_state::JointModelGroup *joint_group,
            const double *joint_group_variable_value);

        bool checkArmJointNum(
            const Eigen::VectorXd &joint_value,
            const std::string text = std::string(""));

        bool checkArmJointNum(
            const double &num,
            const std::string text = std::string(""));

        Eigen::Isometry3d eeInBaseToTcpInBase(
            const Eigen::Isometry3d &pose_ee);

        Eigen::Isometry3d tcpInBaseToEEInBase(
            const Eigen::Isometry3d &pose_gripper);

        void getJointNames(
            std::vector<std::string> &joint_names);

        Eigen::Isometry3d getLinkPose(
            const std::string &link_name,
            const Eigen::VectorXd &joint_pos);

        Eigen::Isometry3d getLinkPose(
            const std::string &link_name);

        Eigen::Isometry3d getTcpPose(
            const Eigen::VectorXd &joint_pos);

        VecEigenVec3 getLinksPos(
            const Eigen::VectorXd &joint_pos,
            const std::vector<std::string> &link_names);

        VecEigenIsometry3d getLinksPose(
            const Eigen::VectorXd &joint_pos,
            const std::vector<std::string> &link_names);

        Eigen::MatrixXd getJacobianMatrix(
            const Eigen::VectorXd &joint_pos,
            const std::string &link_name,
            Eigen::Vector3d reference_point_position = Eigen::Vector3d::Zero());

        Eigen::MatrixXd getJacobianMatrix(
            const std::string &link_name,
            Eigen::Vector3d reference_point_position = Eigen::Vector3d::Zero());

        Eigen::MatrixXd getTcpJacobianMatrix(
            const Eigen::VectorXd &joint_pos);

        bool checkJointPositionSatisfyBound(
            const Eigen::VectorXd &joint_pos);

        void boundJointPos(
            Eigen::VectorXd &joint_pos);

        double weightedJointPosDistance(
            const Eigen::VectorXd &joint_pos_0,
            const Eigen::VectorXd &joint_pos_1);

        double linksPosDistance(
            const VecEigenVec3 &links_pos_0,
            const VecEigenVec3 &links_pos_1);

        void loadReachSpace(
            const std::string &files_dir);

        bool checkReachabilityByReachSpace(
            const Eigen::Isometry3d &tcp_pose_in_world);

    public:
        ros::NodeHandle nh_;
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotStatePtr robot_state_;

        ArmReachSpace::Ptr reach_space_;

        int joint_num_;
        std::vector<std::string> active_joint_names_;

        std::string arm_group_name_;
        std::string arm_ee_link_name_;
        std::string base_link_name_;
        Eigen::Isometry3d tcp_in_ee_pose_;
        std::vector<double> joint_sample_lb_, joint_sample_ub_;
        std::vector<double> joint_pos_weight_;
        std::vector<std::string> critical_link_names_;

    }; // end class

} // end namespace

#endif