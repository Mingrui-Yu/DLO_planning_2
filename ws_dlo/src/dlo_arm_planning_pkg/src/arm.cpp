#include "dlo_arm_planning_pkg/arm.h"
#include "dlo_arm_planning_pkg/ur5.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    Arm::Arm(
        const ros::NodeHandle &nh,
        const std::string &robot_description_name,
        const std::string &group_name) : nh_(nh)
    {
        arm_group_name_ = group_name;

        robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
            new robot_model_loader::RobotModelLoader(robot_description_name)); // The RobotModelLoader should be kept around. Reference: https://github.com/ros-planning/moveit/issues/2979#issuecomment-984440339

        robot_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_loader_->getModel()));
        robot_state_->setToDefaultValues();

        const moveit::core::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);
        joint_num_ = joint_model_group->getActiveVariableCount();

        loadParams();
    }

    // -------------------------------------------------------
    Arm::Arm(
        const ros::NodeHandle &nh,
        moveit::core::RobotStatePtr &robot_state,
        const std::string &group_name) : nh_(nh)
    {
        arm_group_name_ = group_name;
        robot_state_ = robot_state;

        const moveit::core::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);
        joint_num_ = joint_model_group->getActiveVariableCount();
        active_joint_names_ = joint_model_group->getActiveJointModelNames();

        loadParams();
        // printArmInfo();
    }

    // -------------------------------------------------------
    void Arm::loadParams()
    {
        std::string param_name;
        std::string group_name = arm_group_name_;

        param_name = "robot_configs/" + group_name + "/ee_link_name";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, arm_ee_link_name_);

        param_name = "robot_configs/" + group_name + "/base_link_name";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, base_link_name_);

        std::vector<double> position, quaternion;
        param_name = "robot_configs/" + group_name + "/tcp_in_ee_pose/position";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, position);

        param_name = "robot_configs/" + group_name + "/tcp_in_ee_pose/orientation";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, quaternion);

        tcp_in_ee_pose_ = Utils::stdPosQuatVec2Isometry3d(position, quaternion);

        param_name = "robot_configs/" + group_name + "/joint_sample_lb";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
        nh_.getParam(param_name, joint_sample_lb_);
        ROS_ERROR_STREAM_COND(joint_sample_lb_.size() != joint_num_, "The dimension of joint_sample_lb doesn't match the group name");

        param_name = "robot_configs/" + group_name + "/joint_sample_ub";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
        nh_.getParam(param_name, joint_sample_ub_);
        ROS_ERROR_STREAM_COND(joint_sample_ub_.size() != joint_num_, "The dimension of " << param_name << " doesn't match the group name");

        param_name = "robot_configs/" + group_name + "/joint_pos_weight";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
        nh_.getParam(param_name, joint_pos_weight_);
        ROS_ERROR_STREAM_COND(joint_pos_weight_.size() != joint_num_, "The dimension of " << param_name << " doesn't match the group name");

        param_name = "robot_configs/" + group_name + "/critical_link_names";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
        nh_.getParam(param_name, critical_link_names_);
    }

    // -------------------------------------------------------
    void Arm::printArmInfo()
    {
        std::cout << "--------------- Arm Info ----------------- " << std::endl;

        std::cout << "group name: " << arm_group_name_ << std::endl;

        const moveit::core::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);

        std::cout << "getVariableCount(): " << joint_model_group->getVariableCount() << std::endl;

        std::cout << "getActiveVariableCount(): " << joint_model_group->getActiveVariableCount() << std::endl;

        std::vector<std::string> joint_names = joint_model_group->getVariableNames();
        std::cout << "getVariableNames(): ";
        Utils::coutStdVector(joint_names);

        std::vector<std::string> active_joint_names = joint_model_group->getActiveJointModelNames();
        std::cout << "getActiveJointModelNames(): ";
        Utils::coutStdVector(active_joint_names);

        std::cout << "------------------------------------------- " << std::endl;
    }

    // ------------------------------------------------------------
    bool Arm::checkArmJointNum(
        const Eigen::VectorXd &joint_value,
        const std::string text)
    {
        bool valid = (joint_value.size() == joint_num_);
        ROS_ERROR_STREAM_COND(valid == false, text << ": the number of arm joints doesn't match the group name " << arm_group_name_ << ".");
        return valid;
    }

    // ------------------------------------------------------------
    bool Arm::checkArmJointNum(
        const double &num,
        const std::string text)
    {
        bool valid = (num == joint_num_);
        ROS_ERROR_STREAM_COND(valid == false, text << ": the number of arm joints doesn't match the group name " << arm_group_name_ << ".");
        return valid;
    }

    // ------------------------------------------------------------
    void Arm::setJointPositions(
        const Eigen::VectorXd &joint_pos)
    {
        checkArmJointNum(joint_pos);
        const moveit::core::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);
        robot_state_->setJointGroupPositions(joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));
    }

    // ------------------------------------------------------------
    Eigen::VectorXd Arm::randomJointPos()
    {
        Eigen::VectorXd random_joint_pos(joint_num_);
        for (size_t j = 0; j < random_joint_pos.size(); j++)
        {
            random_joint_pos[j] = Utils::getRandomDouble(joint_sample_lb_[j], joint_sample_ub_[j]);
        }
        return random_joint_pos;
    }

    // ------------------------------------------------------------
    bool Arm::armTcpRandomIK(
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos)
    {
        return armEndEffectorClosestIK(randomJointPos(), tcpInBaseToEEInBase(target_pose), result_joint_pos);
    }

    // ------------------------------------------------------------
    bool Arm::armTcpIterativeIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos)
    {
        return armEndEffectorIterativeIK(ref_joint_pos, tcpInBaseToEEInBase(target_pose), result_joint_pos);
    }

    // ------------------------------------------------------------
    bool Arm::armEndEffectorIterativeIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos)
    {
        checkArmJointNum(ref_joint_pos);

        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);

        // initial value for IK solving
        robot_state_->setJointGroupPositions(
            joint_model_group, Utils::eigenVectorXd2StdVector(ref_joint_pos));

        // solve the IK problem
        double timeout = 0.01;
        moveit::core::GroupStateValidityCallbackFn ik_check_callback =
            boost::bind(&Arm::validateIKSolution, this, _1, _2, _3);

        bool found_ik = robot_state_->setFromIK(joint_model_group, target_pose,
                                                arm_ee_link_name_, timeout, ik_check_callback);

        // get the solution
        if (found_ik)
        {
            std::vector<double> joint_values;
            robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
            result_joint_pos = Utils::stdVector2EigenVectorXd(joint_values);
            return true;
        }
        else
        {
            return false;
        }
    }

    // ------------------------------------------------------------
    bool Arm::armTcpClosestIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos)
    {
        return armEndEffectorClosestIK(ref_joint_pos, tcpInBaseToEEInBase(target_pose), result_joint_pos);
    }

    // ------------------------------------------------------------
    bool Arm::armEndEffectorClosestIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &ee_pose_in_world,
        Eigen::VectorXd &result_joint_pos)
    {
        std::vector<std::vector<double>> valid_ik_solutions;
        armEndEffectorAnalyticalIK(ee_pose_in_world, valid_ik_solutions);

        if (valid_ik_solutions.size() == 0)
            return false;

        // 找 closest solution
        double min_dist = 1e10;
        for (int i = 0; i < valid_ik_solutions.size(); i++)
        {
            Eigen::VectorXd solution = Utils::stdVector2EigenVectorXd(valid_ik_solutions[i]);

            double dist = weightedJointPosDistance(solution, ref_joint_pos);

            if (dist < min_dist)
            {
                min_dist = dist;
                result_joint_pos = solution;
            }
        }

        return true;
    }

    // ------------------------------------------------------------
    int Arm::armTcpAnalyticalIK(
        const Eigen::Isometry3d &target_pose,
        std::vector<std::vector<double>> &valid_ik_solutions)
    {
        return armEndEffectorAnalyticalIK(tcpInBaseToEEInBase(target_pose), valid_ik_solutions);
    }

    // ------------------------------------------------------------
    int Arm::armEndEffectorAnalyticalIK(
        const Eigen::Isometry3d &ee_pose_in_world,
        std::vector<std::vector<double>> &valid_ik_solutions)
    {
        // frame transformation
        std::string ur5_base_link_name;
        if (arm_group_name_ == "arm_0")
            ur5_base_link_name = "arm_0_base_link";
        else
            ur5_base_link_name = "arm_1_base_link";

        Eigen::Isometry3d base_pose_in_world = robot_state_->getGlobalLinkTransform(ur5_base_link_name);
        Eigen::Isometry3d ee_pose_in_base = base_pose_in_world.inverse() * ee_pose_in_world;

        // analytical IK
        dual_ur::UR5 ur5;
        std::vector<std::vector<double>> ik_solutions;
        int n_sols = ur5.inverse(ee_pose_in_base, ik_solutions); // 返回的解的关节角在[0, 2pi]

        if (n_sols == 0)
            return false;

        // constrain the joint range to be no larger than 2pi
        for (int i = 0; i < joint_num_; i++)
        {
            if ((joint_sample_ub_[i] - joint_sample_lb_[i]) > 1.1 * 2 * M_PI)
            {
                ROS_ERROR_STREAM("Arm::armEndEffectorIK(): the range of the valid joint angles cannot be larger than 2pi.");
            }
        }

        // compute the ik solution within the allowed joint range
        for (int i = 0; i < n_sols; i++)
        {
            std::vector<double> solution = ik_solutions[i];
            bool b_in_bound = true;
            for (int j = 0; j < joint_num_; j++)
            {
                if (solution[j] > joint_sample_ub_[j])
                { // if the solution is larger than the upper bound, then minus 2pi
                    solution[j] -= 2 * M_PI;
                }
                if ((joint_sample_lb_[j] > solution[j]) || (solution[j] > joint_sample_ub_[j]))
                { // if the solution is out of the allowed range, then discard it
                    b_in_bound = false;
                    break;
                }
            }
            if (b_in_bound)
                valid_ik_solutions.push_back(solution);
        }

        return valid_ik_solutions.size();
    }

    // -------------------------------------------------------
    bool Arm::validateIKSolution(
        robot_state::RobotState *robot_state,
        const robot_state::JointModelGroup *joint_group,
        const double *joint_group_variable_value)
    {
        robot_state->setJointGroupPositions(joint_group, joint_group_variable_value);

        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_group, joint_values);

        return checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(joint_values));
    }

    // ------------------------------------------------------------
    Eigen::Isometry3d Arm::eeInBaseToTcpInBase(
        const Eigen::Isometry3d &pose_ee)
    {
        return pose_ee * tcp_in_ee_pose_;
    }

    // ------------------------------------------------------------
    Eigen::Isometry3d Arm::tcpInBaseToEEInBase(
        const Eigen::Isometry3d &pose_tcp)
    {
        return pose_tcp * tcp_in_ee_pose_.inverse();
    }

    // ------------------------------------------------------------
    void Arm::getJointNames(
        std::vector<std::string> &joint_names)
    {
        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);
        joint_names = joint_model_group->getVariableNames();
    }

    // ------------------------------------------------------------
    Eigen::Isometry3d Arm::getLinkPose(
        const std::string &link_name)
    {
        Eigen::Isometry3d link_pose = robot_state_->getGlobalLinkTransform(link_name);
        return link_pose;
    }

    // ------------------------------------------------------------
    Eigen::Isometry3d Arm::getLinkPose(
        const std::string &link_name,
        const Eigen::VectorXd &joint_pos)
    {
        setJointPositions(joint_pos);
        return getLinkPose(link_name);
    }

    // ------------------------------------------------------------
    Eigen::Isometry3d Arm::getTcpPose(
        const Eigen::VectorXd &joint_pos)
    {
        return eeInBaseToTcpInBase(getLinkPose(arm_ee_link_name_, joint_pos));
    }

    // ------------------------------------------------------------
    VecEigenVec3 Arm::getLinksPos(
        const Eigen::VectorXd &joint_pos,
        const std::vector<std::string> &link_names)
    {
        checkArmJointNum(joint_pos);

        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);

        robot_state_->setJointGroupPositions(
            joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

        VecEigenVec3 links_pos(link_names.size());
        for (size_t i = 0; i < link_names.size(); i++)
        {
            links_pos[i] = robot_state_->getGlobalLinkTransform(link_names[i]).translation();
        }

        return links_pos;
    }

    // ------------------------------------------------------------
    VecEigenIsometry3d Arm::getLinksPose(
        const Eigen::VectorXd &joint_pos,
        const std::vector<std::string> &link_names)
    {
        checkArmJointNum(joint_pos);

        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);

        robot_state_->setJointGroupPositions(
            joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

        VecEigenIsometry3d links_pose(link_names.size());
        for (size_t i = 0; i < link_names.size(); i++)
        {
            links_pose[i] = robot_state_->getGlobalLinkTransform(link_names[i]);
        }

        return links_pose;
    }

    // ------------------------------------------------------------
    Eigen::MatrixXd Arm::getJacobianMatrix(
        const Eigen::VectorXd &joint_pos,
        const std::string &link_name,
        Eigen::Vector3d reference_point_position)
    {
        checkArmJointNum(joint_pos);

        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);

        robot_state_->setJointGroupPositions(
            joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

        return getJacobianMatrix(link_name, reference_point_position);
    }

    // ------------------------------------------------------------
    Eigen::MatrixXd Arm::getJacobianMatrix(
        const std::string &link_name,
        Eigen::Vector3d reference_point_position)
    {
        const moveit::core::JointModelGroup *joint_model_group =
            robot_state_->getJointModelGroup(arm_group_name_);

        // return the jacobian matrix of the link in the base frame
        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(joint_model_group, robot_state_->getLinkModel(link_name),
                                  reference_point_position, jacobian);

        return jacobian;
    }

    // ------------------------------------------------------------
    Eigen::MatrixXd Arm::getTcpJacobianMatrix(
        const Eigen::VectorXd &joint_pos)
    {
        return getJacobianMatrix(joint_pos, arm_ee_link_name_, tcp_in_ee_pose_.translation());
    }

    // ------------------------------------------------------------
    bool Arm::checkJointPositionSatisfyBound(
        const Eigen::VectorXd &joint_pos)
    {
        checkArmJointNum(joint_pos);

        for (size_t i = 0; i < joint_pos.size(); i++)
        {
            if (joint_pos[i] > joint_sample_ub_[i] || joint_pos[i] < joint_sample_lb_[i])
            {
                return false;
            }
        }

        return true;
    }

    // ------------------------------------------------------------
    void Arm::boundJointPos(
        Eigen::VectorXd &joint_pos)
    {
        checkArmJointNum(joint_pos);

        for (size_t i = 0; i < joint_pos.size(); i++)
        {
            joint_pos[i] = std::max(joint_sample_lb_[i], std::min(joint_pos[i], joint_sample_ub_[i]));
        }
    }

    // ------------------------------------------------------------
    double Arm::weightedJointPosDistance(
        const Eigen::VectorXd &joint_pos_0,
        const Eigen::VectorXd &joint_pos_1)
    {
        Eigen::VectorXd diff = joint_pos_1 - joint_pos_0;

        // weight the differences of the joint angles
        for (size_t j = 0; j < diff.size(); j++)
        {
            diff[j] *= joint_pos_weight_[j];
        }

        return diff.norm();
    }

    // ------------------------------------------------------------
    double Arm::linksPosDistance(
        const VecEigenVec3 &links_pos_0,
        const VecEigenVec3 &links_pos_1)
    {
        return (Utils::stdVecEigenVec3ToEigenVectorXd(links_pos_0) -
                Utils::stdVecEigenVec3ToEigenVectorXd(links_pos_1))
            .norm();
    }

    // ------------------------------------------------------------
    void Arm::loadReachSpace(
        const std::string &files_dir)
    {
        reach_space_ = std::make_shared<ArmReachSpace>(files_dir);
    }

    // ------------------------------------------------------------
    bool Arm::checkReachabilityByReachSpace(
        const Eigen::Isometry3d &tcp_pose_in_world)
    {
        if (reach_space_ == nullptr)
        {
            ROS_ERROR("Arm::checkReachabilityByReachSpace(): reach_space_ is nullptr.");
        }

        std::string reach_space_base_link_name;
        if (arm_group_name_ == "arm_0")
            reach_space_base_link_name = "arm_0_base_link";
        else
            reach_space_base_link_name = "arm_1_base_link";

        Eigen::Isometry3d ee_pose_in_world = tcpInBaseToEEInBase(tcp_pose_in_world);
        Eigen::Isometry3d base_pose_in_world = robot_state_->getGlobalLinkTransform(reach_space_base_link_name);
        Eigen::Isometry3d ee_pose_in_base = base_pose_in_world.inverse() * ee_pose_in_world;

        return reach_space_->checkReachability(ee_pose_in_base);
    }

} // end namespace
