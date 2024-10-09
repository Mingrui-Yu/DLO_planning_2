#include "dlo_arm_planning_pkg/planner/planner_base.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    PlannerBase::PlannerBase(
        const ros::NodeHandle &nh,
        const Scene::Ptr &scene) : nh_(nh)
    {
        scene_ = scene;
        dlo_ = scene_->dlo_;
        dual_arm_ = scene_->dual_arm_;

        loadCommonParams();
    }

    // -------------------------------------------------------
    void PlannerBase::setVisualizer(Visualize::Ptr &visualizer)
    {
        visualizer_ = visualizer;
    }

    // -------------------------------------------------------
    void PlannerBase::loadCommonParams()
    {
        std::string param_name;

        param_name = "planner_configs/common/pose_distance/pos_weight";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, pose_dist_pos_weight_);

        param_name = "planner_configs/common/pose_distance/rot_weight";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, pose_dist_rot_weight_);

        param_name = "planner_configs/common/path_interpolation_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_interpolation_step_size_);

        std::vector<double> dlo_sample_lb, dlo_sample_ub;
        param_name = "planner_configs/common/dlo_sample_range/lb";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_sample_lb);
        param_name = "planner_configs/common/dlo_sample_range/ub";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_sample_ub);
        dlo_sample_lb_ = Utils::stdVector2EigenVectorXd(dlo_sample_lb);
        dlo_sample_ub_ = Utils::stdVector2EigenVectorXd(dlo_sample_ub);

        param_name = "planner_configs/common/dlo_projection/function_tolerance";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_projection_function_tolerance_);

        param_name = "planner_configs/common/dlo_projection/max_num_iterations";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_projection_max_num_iterations_);

        param_name = "planner_configs/common/num_random_sample_goal";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, num_random_sample_goal_);

        param_name = "planner_configs/common/new_goal_ik_probability";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, new_goal_ik_probability_);

        param_name = "planner_configs/common/rrt_max_iter";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, rrt_max_iter_);

        param_name = "planner_configs/common/extend_max_steps";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, extend_max_steps_);

        param_name = "planner_configs/common/path_smooth_max_iter";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_smooth_max_iter_);

        param_name = "planner_configs/common/path_collision_check_cartesian_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_collision_check_cartesian_step_size_);

        param_name = "planner_configs/common/path_collision_check_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_collision_check_step_size_);

        param_name = "planner_configs/common/collision_check_min_dist_thres";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, collision_check_min_dist_thres_);

        param_name = "planner_configs/common/steer_joint_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, steer_joint_step_size_);

        param_name = "planner_configs/common/steer_dlo_cartesian_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, steer_dlo_cartesian_step_size_);

        param_name = "planner_configs/common/steer_dlo_angle_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, steer_dlo_angle_step_size_);

        param_name = "planner_configs/common/connect_cartesian_max_dist";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, connect_cartesian_max_dist_);

        param_name = "planner_configs/common/connect_joint_max_dist";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, connect_joint_max_dist_);

        param_name = "planner_configs/common/constraint_error_thres";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, constraint_error_thres_);

        param_name = "planner_configs/common/constraint_projection_max_iter";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, constraint_projection_max_iter_);

        param_name = "planner_configs/common/constraint_step_gain";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, constraint_step_gain_);

        param_name = "planner_configs/common/use_underactuated_steer";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, use_underactuated_steer_);
    }

    // -------------------------------------------------------
    void PlannerBase::updateNode(
        const Node::Ptr &node)
    {
        updateNodeDualArmJointPos(node);
        if (b_node_links_pos_)
            updateNodeLinksPos(node);
        if (b_node_tcp_pose_)
            updateNodeTcpPose(node);
    }

    // -------------------------------------------------------
    void PlannerBase::updateNodeDualArmJointPos(
        const Node::Ptr &node)
    {
        Eigen::VectorXd dual_arm_joint_pos(node->arm_0_joint_pos_.size() + node->arm_1_joint_pos_.size());

        dual_arm_joint_pos.block(0, 0, node->arm_0_joint_pos_.size(), 1) = node->arm_0_joint_pos_;
        dual_arm_joint_pos.block(node->arm_0_joint_pos_.size(), 0, node->arm_1_joint_pos_.size(), 1) = node->arm_1_joint_pos_;

        node->dual_arm_joint_pos_ = dual_arm_joint_pos;
    }

    // -------------------------------------------------------
    void PlannerBase::updateNodeTcpPose(
        const Node::Ptr &node)
    {
        node->arm_0_tcp_pose_ = dual_arm_->arm_0_->getTcpPose(node->arm_0_joint_pos_);
        node->arm_1_tcp_pose_ = dual_arm_->arm_1_->getTcpPose(node->arm_1_joint_pos_);
    }

    // -------------------------------------------------------
    void PlannerBase::updateNodeLinksPos(
        const Node::Ptr &node)
    {
        node->arm_0_links_pos_ = dual_arm_->arm_0_->getLinksPos(node->arm_0_joint_pos_, dual_arm_->arm_0_->critical_link_names_);
        node->arm_1_links_pos_ = dual_arm_->arm_1_->getLinksPos(node->arm_1_joint_pos_, dual_arm_->arm_1_->critical_link_names_);
    }

    // -------------------------------------------------------
    double PlannerBase::twoNodeDistance(
        const Node::Ptr &node_0,
        const Node::Ptr &node_1,
        std::string dist_metric)
    {
        if (dist_metric == "joint_pos")
        {
            return std::sqrt((node_0->arm_0_joint_pos_ - node_1->arm_0_joint_pos_).squaredNorm() +
                             (node_0->arm_1_joint_pos_ - node_1->arm_1_joint_pos_).squaredNorm());
        }
        else if (dist_metric == "dlo_fps_pos")
        {
            return (node_0->dlo_state_.fps_pos_ - node_1->dlo_state_.fps_pos_).norm();
        }
        else if (dist_metric == "arm_and_dlo_pos_max_dist")
        {
            double max_dist = 0.0;
            for (size_t i = 0; i < node_0->arm_0_links_pos_.size(); i++)
            {
                double dist = (node_0->arm_0_links_pos_[i] - node_1->arm_0_links_pos_[i]).norm();
                if (dist > max_dist)
                    max_dist = dist;
            }
            for (size_t i = 0; i < node_0->arm_1_links_pos_.size(); i++)
            {
                double dist = (node_0->arm_1_links_pos_[i] - node_1->arm_1_links_pos_[i]).norm();
                if (dist > max_dist)
                    max_dist = dist;
            }
            for (size_t k = 0; k < node_0->dlo_state_.num_fps_; k++)
            {
                double dist = (node_0->dlo_state_.getFpPos(k) - node_1->dlo_state_.getFpPos(k)).norm();
                if (dist > max_dist)
                    max_dist = dist;
            }
            return max_dist;
        }
        else if (dist_metric == "dlo_fps_pos_max_dist")
        {
            double max_dist = 0.0;
            for (size_t k = 0; k < node_0->dlo_state_.num_fps_; k++)
            {
                double dist = (node_0->dlo_state_.getFpPos(k) - node_1->dlo_state_.getFpPos(k)).norm();
                if (dist > max_dist)
                    max_dist = dist;
            }
            return max_dist;
        }
        else
        {
            ROS_ERROR_STREAM("twoNodeDistance(): invalid distance metric: " << dist_metric);
            throw "Shut down the program.";
        }
    }

    // -------------------------------------------------------
    double PlannerBase::pathLength(
        const std::vector<Node::Ptr> &path,
        int begin_idx,
        int end_idx)
    {
        if (begin_idx < 0)
            begin_idx = 0;
        if (end_idx < 0)
            end_idx = path.size() - 1;

        // distance is defined as the average moving displacement of each DLO feature point in the Cartesian space
        double length = 0.0;
        for (size_t k = 0; k < dlo_->num_fps_; k++)
        {
            for (size_t i = begin_idx; i < end_idx; i++)
            {
                double distance = (path[i]->dlo_state_.getFpPos(k) - path[i + 1]->dlo_state_.getFpPos(k)).norm();
                length += distance;
            }
        }
        length /= dlo_->num_fps_;

        return length;
    }

    // -------------------------------------------------------
    Node::Ptr PlannerBase::getNearestNode(
        const std::vector<Node::Ptr> &node_list,
        const Node::Ptr &target_node,
        const std::string &dist_metric)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // linear search
        double min_dist = 1e10;
        int min_idx = -1;
        for (size_t i = 0; i < node_list.size(); i++)
        {
            double dist = twoNodeDistance(target_node, node_list[i], dist_metric);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_idx = i;
            }
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        time_cost_nearest_node_search_ += time_used.count();
        count_nearest_node_search_++;

        return node_list[min_idx];
    }

    // -------------------------------------------------------
    bool PlannerBase::checkNodeCollision(const Node::Ptr &node)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        if (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(node->arm_0_joint_pos_))
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): arm_0 out of bound.");
            return true;
        }
        if (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(node->arm_0_joint_pos_))
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): arm_1 out of bound.");
            return true;
        }

        if (scene_->checkRobotCollision(node->arm_0_joint_pos_, node->arm_1_joint_pos_, collision_check_min_dist_thres_))
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): dual_arm is in collision.");
            return true;
        }
        if (scene_->checkDLOAndWorldCollision(node->dlo_state_, collision_check_min_dist_thres_))
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): DLO is in collision with the world.");
            return true;
        }

        // FCL-based collision detection
        scene_->cd_fcl_->setDloObjectsTransform(node->dlo_state_);
        scene_->cd_fcl_->setRobotObjectsTransform(node->arm_0_joint_pos_, node->arm_1_joint_pos_);
        if (scene_->cd_fcl_->checkDLOSelfCollision())
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): DLO is in self-collision.");
            return true;
        }
        if (scene_->cd_fcl_->checkDLOAndRobotCollision())
        {
            ROS_DEBUG_STREAM("checkNodeCollision(): DLO is in collision with the robot.");
            return true;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        time_cost_collision_detection_ += time_used.count();
        count_collision_detection_++;

        return false;
        // true: in collision; false: no collision
    }

    // -------------------------------------------------------
    bool PlannerBase::checkTwoNodePathCollision(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node)
    {
        dual_arm_->checkDualArmJointNum(from_node->dual_arm_joint_pos_.size(), "PlannerBase::checkTwoNodePathCollision(), from_node");
        dual_arm_->checkDualArmJointNum(to_node->dual_arm_joint_pos_.size(), "PlannerBase::checkTwoNodePathCollision(), to_node");

        // compute the interpolation ratio t in [0, 1]
        double max_dist = 0.0;
        for (size_t k = 0; k < dlo_->num_fps_; k++)
        {
            double dist = (from_node->dlo_state_.getFpPos(k) - to_node->dlo_state_.getFpPos(k)).norm();
            max_dist = std::max(max_dist, dist);
        }
        double t_step = std::min(1.0, path_collision_check_cartesian_step_size_ / max_dist);

        // linear interpolation between from_node and to_node (not including from_node and to_node）
        for (double t = t_step; t < 1.0; t += t_step)
        {
            Eigen::VectorXd fps_pos = from_node->dlo_state_.fps_pos_ * (1 - t) + to_node->dlo_state_.fps_pos_ * t;
            Eigen::VectorXd dual_arm_joint_pos = from_node->dual_arm_joint_pos_ * (1 - t) + to_node->dual_arm_joint_pos_ * t;

            Eigen::VectorXd arm_0_joint_pos, arm_1_joint_pos;
            dual_arm_->splitTwoArmJointPos(dual_arm_joint_pos, arm_0_joint_pos, arm_1_joint_pos);

            Node::Ptr temp_node = std::make_shared<Node>(DLOState(fps_pos), arm_0_joint_pos, arm_1_joint_pos);

            if (checkNodeCollision(temp_node))
            {
                return true;
            }
        }

        return false;
        // true: in collision; false: no collision
    }

    // ------------------------------------------------------------
    std::vector<Node::Ptr> PlannerBase::pathExtract(
        const Node::Ptr &node_forward_end,
        const Node::Ptr &node_backward_end)
    {
        std::vector<Node::Ptr> path_list;

        // tree from the start
        Node::Ptr node = node_forward_end;
        while (node != nullptr)
        {
            path_list.push_back(node);
            node = node->parent_;
        }
        std::reverse(path_list.begin(), path_list.end());

        // tree from the goal
        node = node_backward_end;
        while (node != nullptr)
        {
            path_list.push_back(node);
            node = node->parent_;
        }

        return path_list;
    }

    // ------------------------------------------------------------
    void PlannerBase::pathSmooth(
        std::vector<Node::Ptr> &path_list)
    {
        int iter = 0;
        while (ros::ok() && iter < path_smooth_max_iter_)
        {
            std::vector<Node::Ptr> smooth_path_list;
            std::vector<Node::Ptr> path_shortcut;

            if (path_list.size() < 3)
            {
                return;
            }

            int i = Utils::getRandomInt(0, path_list.size() - 3);
            int j = Utils::getRandomInt(i + 2, path_list.size() - 1);

            Node::Ptr node_reached = extend(path_shortcut, path_list[i], path_list[j], /*greedy*/ true);

            if (checkTwoNodeCanConnect(node_reached, path_list[j], 1e-2, 1e-2))
            {
                path_shortcut.push_back(path_list[j]);

                if (pathLength(path_shortcut) < pathLength(path_list, i, j))
                {
                    for (int idx = 0; idx <= i; idx++)
                    {
                        smooth_path_list.push_back(path_list[idx]);
                    }
                    for (int idx = 0; idx < path_shortcut.size(); idx++)
                    {
                        smooth_path_list.push_back(path_shortcut[idx]);
                    }
                    for (int idx = j + 1; idx < path_list.size(); idx++)
                    {
                        smooth_path_list.push_back(path_list[idx]);
                    }

                    path_list = smooth_path_list;
                    ROS_DEBUG_STREAM("pathSmooth(): iter = " << iter << ", path_length = " << path_list.size());
                }
            }
            iter++;
        }
    }

    // ------------------------------------------------------------
    // only for path visualization
    std::vector<Node::Ptr> PlannerBase::pathInterpolation(
        const std::vector<Node::Ptr> &path_list)
    {
        std::vector<Node::Ptr> interpolated_path;

        for (size_t i = 0; ros::ok() && i < path_list.size() - 1; i++)
        {

            Node::Ptr from_node = path_list[i];
            Node::Ptr to_node = path_list[i + 1];

            double t_step = dlo_->calcInterpolationRatio(from_node->dlo_state_, to_node->dlo_state_,
                                                         steer_dlo_cartesian_step_size_ / 10, steer_dlo_angle_step_size_ / 10);

            for (double t = 0; t < 1; t += t_step)
            {
                DLOState new_dlo_state = dlo_->interpolate(from_node->dlo_state_, to_node->dlo_state_, t);

                Eigen::VectorXd new_arm_0_joint_pos = from_node->arm_0_joint_pos_ * (1 - t) + to_node->arm_0_joint_pos_ * t;
                Eigen::VectorXd new_arm_1_joint_pos = from_node->arm_1_joint_pos_ * (1 - t) + to_node->arm_1_joint_pos_ * t;

                Node::Ptr new_node = std::make_shared<Node>(new_dlo_state, new_arm_0_joint_pos, new_arm_1_joint_pos, nullptr, "interpolation");

                interpolated_path.push_back(new_node);
            }
        }
        interpolated_path.push_back(path_list[path_list.size() - 1]); // mannually add the last path point

        return interpolated_path;
    }

    // -------------------------------------------------------
    void PlannerBase::nodeListToPath(
        const std::vector<Node::Ptr> &node_list,
        Path &path)
    {
        path.clear();

        for (auto node : node_list)
        {
            path.dlo_path_.push_back(node->dlo_state_);
            path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(node->arm_0_joint_pos_));
            path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(node->arm_1_joint_pos_));
        }
    }

    // ------------------------------------------------------------
    void PlannerBase::swapTrees(
        std::vector<Node::Ptr> &node_list_a,
        std::vector<Node::Ptr> &node_list_b)
    {
        auto tmp_list = node_list_a;
        node_list_a = node_list_b;
        node_list_b = tmp_list;
    }

    // ------------------------------------------------------------
    void PlannerBase::swapNodes(
        Node::Ptr &node_a,
        Node::Ptr &node_b)
    {
        auto tmp_node = node_a;
        node_a = node_b;
        node_b = tmp_node;
    }

    // -------------------------------------------------------
    Eigen::VectorXd PlannerBase::constraintError(
        const Eigen::VectorXd &pose, // pos + rpy angle
        const Eigen::VectorXd &pose_lb,
        const Eigen::VectorXd &pose_ub)
    {
        Eigen::VectorXd pose_vec = pose;
        Eigen::VectorXd constraint_error = Eigen::VectorXd::Zero(6);

        for (size_t i = 0; i < pose.size(); i++)
        {
            if (pose(i) < pose_lb(i))
            {
                constraint_error(i) = pose(i) - pose_lb(i);
            }
            else if (pose(i) > pose_ub(i))
            {
                constraint_error(i) = pose(i) - pose_ub(i);
            }
        }

        return constraint_error;
    }

    // -------------------------------------------------------
    bool PlannerBase::constrainConfig(
        Node::Ptr &node,
        const Node::Ptr &node_old)
    {
        Eigen::VectorXd init_arm_0_joint_pos = node->arm_0_joint_pos_;
        Eigen::VectorXd init_arm_1_joint_pos = node->arm_1_joint_pos_;

        // arm_0
        bool arm_0_success = false;
        int iter = 0;
        while (ros::ok() && iter < constraint_projection_max_iter_)
        {
            updateNodeTcpPose(node);

            Eigen::Isometry3d constraint_frame = node->dlo_state_.getLeftEndPose();
            Eigen::Isometry3d pose_in_constraint_frame = constraint_frame.inverse() * node->arm_0_tcp_pose_;
            Eigen::VectorXd pose_vec = Utils::isometryToPosAndRPYAngle(pose_in_constraint_frame);

            Eigen::VectorXd constraint_error = constraintError(pose_vec, /*pose_lb*/ Eigen::VectorXd::Zero(6), /*pose_ub*/ Eigen::VectorXd::Zero(6));

            if (constraint_error.norm() < constraint_error_thres_)
            {
                ROS_DEBUG_STREAM("constrainConfig(): arm_0 satify the constraint_error_thres.");
                arm_0_success = true;
                break;
            }

            Eigen::MatrixXd avel2eulervel_transform = Eigen::MatrixXd::Zero(6, 6);
            avel2eulervel_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            avel2eulervel_transform.block<3, 3>(3, 3) = Utils::matrixRelateAngularVelToRPYVel(pose_vec.block<3, 1>(3, 0));

            Eigen::MatrixXd base2constraintframe_transform = Eigen::MatrixXd::Zero(6, 6);
            base2constraintframe_transform.block<3, 3>(0, 0) = constraint_frame.rotation().transpose();
            base2constraintframe_transform.block<3, 3>(3, 3) = constraint_frame.rotation().transpose();

            Eigen::MatrixXd transformed_jacobian = avel2eulervel_transform * base2constraintframe_transform * dual_arm_->arm_0_->getTcpJacobianMatrix(node->arm_0_joint_pos_);

            node->arm_0_joint_pos_ += Utils::pseudoInverse(transformed_jacobian) * (-constraint_error) * constraint_step_gain_;

            if (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(node->arm_0_joint_pos_))
            {
                ROS_DEBUG_STREAM("constrainConfig(): arm_0 failed: out of bound.");
                bool arm_0_success = false;
                break;
            }
            iter++;
        }

        if (!arm_0_success)
        {
            ROS_DEBUG_STREAM("constrainConfig(): arm_0 failed.");
            return false;
        }

        // arm_1
        bool arm_1_success = false;
        iter = 0;
        while (ros::ok() && iter < constraint_projection_max_iter_)
        {

            updateNodeTcpPose(node);

            Eigen::Isometry3d constraint_frame = node->dlo_state_.getRightEndPose();
            Eigen::Isometry3d pose_in_constraint_frame = constraint_frame.inverse() * node->arm_1_tcp_pose_;
            Eigen::VectorXd pose_vec = Utils::isometryToPosAndRPYAngle(pose_in_constraint_frame);

            Eigen::VectorXd constraint_error = constraintError(pose_vec, /*pose_lb*/ Eigen::VectorXd::Zero(6), /*pose_ub*/ Eigen::VectorXd::Zero(6));

            if (constraint_error.norm() < constraint_error_thres_)
            {
                ROS_DEBUG_STREAM("constrainConfig(): arm_1 satify the constraint_error_thres.");
                arm_1_success = true;
                break;
            }

            Eigen::MatrixXd avel2eulervel_transform = Eigen::MatrixXd::Zero(6, 6);
            avel2eulervel_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            avel2eulervel_transform.block<3, 3>(3, 3) = Utils::matrixRelateAngularVelToRPYVel(pose_vec.block<3, 1>(3, 0));

            Eigen::MatrixXd base2constraintframe_transform = Eigen::MatrixXd::Zero(6, 6);
            base2constraintframe_transform.block<3, 3>(0, 0) = constraint_frame.rotation().transpose();
            base2constraintframe_transform.block<3, 3>(3, 3) = constraint_frame.rotation().transpose();

            Eigen::MatrixXd transformed_jacobian = avel2eulervel_transform * base2constraintframe_transform * dual_arm_->arm_1_->getTcpJacobianMatrix(node->arm_1_joint_pos_);

            node->arm_1_joint_pos_ += Utils::pseudoInverse(transformed_jacobian) * (-constraint_error) * constraint_step_gain_;

            if (!dual_arm_->arm_1_->checkJointPositionSatisfyBound(node->arm_1_joint_pos_))
            {
                ROS_DEBUG_STREAM("constrainConfig(): arm_1 failed: out of bound.");
                bool arm_1_success = false;
                break;
            }
            iter++;
        }

        if (!arm_1_success)
        {
            ROS_DEBUG_STREAM("constrainConfig(): arm_1 failed.");
            return false;
        }

        updateNodeTcpPose(node);
        double arm_0_tcp_pose_error = Utils::distanceBetweenTwoPose(node->arm_0_tcp_pose_, node->dlo_state_.getLeftEndPose(), 1.0, 1.0);
        double arm_1_tcp_pose_error = Utils::distanceBetweenTwoPose(node->arm_1_tcp_pose_, node->dlo_state_.getRightEndPose(), 1.0, 1.0);
        if (arm_0_tcp_pose_error > 0.1 || arm_1_tcp_pose_error > 0.1)
        {
            ROS_ERROR("constrainConfig(): too large error.");
        }

        return true;
    }

    // -------------------------------------------------------
    bool PlannerBase::checkPlanningRequest(
        const PlanningRequest &req)
    {
        ROS_INFO("Check the planning request ...");

        ROS_ERROR_COND(req.goal_type_ == "", "checkPlanningRequest(): Didn't specify the goal type (joint_space or task_space)");

        // dual arm
        ROS_ERROR_COND(req.arm_0_start_joint_pos_.size() != dual_arm_->arm_0_->joint_num_, "checkPlanningRequest(): The number of the joints of start configuration is wrong.");
        ROS_ERROR_COND(req.arm_1_start_joint_pos_.size() != dual_arm_->arm_1_->joint_num_, "checkPlanningRequest(): The number of the joints of start configuration is wrong.");

        ROS_DEBUG_STREAM("start arm_0 configuration: " << Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_).transpose());
        ROS_DEBUG_STREAM("start arm_1 configuration: " << Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_).transpose());
        if (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_)))
        {
            ROS_ERROR_STREAM("checkPlanningRequest(): The start arm_0 configuration is out of bound.");
            return false;
        }
        if (!dual_arm_->arm_1_->checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_)))
        {
            ROS_ERROR_STREAM("checkPlanningRequest(): The start arm_1 configuration is out of bound.");
            return false;
        }

        if (req.goal_type_ == "joint_space")
        {
            ROS_ERROR_COND(req.arm_0_goal_joint_pos_.size() != dual_arm_->arm_0_->joint_num_, "checkPlanningRequest(): The number of the joints of goal configuration is wrong.");
            ROS_ERROR_COND(req.arm_1_goal_joint_pos_.size() != dual_arm_->arm_1_->joint_num_, "checkPlanningRequest(): The number of the joints of goal configuration is wrong.");

            ROS_DEBUG_STREAM("goal arm_0 configuration: " << Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_).transpose());
            ROS_DEBUG_STREAM("goal arm_1 configuration: " << Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_).transpose());
            if (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_)))
            {
                ROS_ERROR_STREAM("checkPlanningRequest(): The goal arm_0 configuration is out of bound.");
                return false;
            }
            if (!dual_arm_->arm_1_->checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_)))
            {
                ROS_ERROR_STREAM("checkPlanningRequest(): The goal arm_1 configuration is out of bound.");
                return false;
            }
        }

        if (req.b_check_start_goal_collision_ && scene_->checkRobotCollision(Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_),
                                                                             Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_), collision_check_min_dist_thres_))
        {
            ROS_ERROR_STREAM("checkPlanningRequest(): The start robot configuration is in collision.");
            Eigen::VectorXd arm_0_joint_pos = Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_);
            Eigen::VectorXd arm_1_joint_pos = Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_);
            visualizer_->publishPlanningScene(arm_0_joint_pos, arm_1_joint_pos);
            VecEigenIsometry3d poses;
            std::vector<std::string> geometry_types;
            std::vector<std::vector<double>> geometry_params;
            scene_->cd_fcl_->setRobotObjectsTransform(arm_0_joint_pos, arm_1_joint_pos);
            scene_->cd_fcl_->getInfoForVisualizeRobot(poses, geometry_types, geometry_params);
            visualizer_->publishRobotCollisionShape(poses, geometry_types, geometry_params);
            return false;
        }
        if (req.goal_type_ == "joint_space")
        {
            if (req.b_check_start_goal_collision_ && scene_->checkRobotCollision(Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_),
                                                                                 Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_), collision_check_min_dist_thres_))
            {
                ROS_ERROR_STREAM("checkPlanningRequest(): The goal robot configuration is in collision.");
                Eigen::VectorXd arm_0_joint_pos = Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_);
                Eigen::VectorXd arm_1_joint_pos = Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_);
                visualizer_->publishPlanningScene(arm_0_joint_pos, arm_1_joint_pos);
                VecEigenIsometry3d poses;
                std::vector<std::string> geometry_types;
                std::vector<std::vector<double>> geometry_params;
                scene_->cd_fcl_->setRobotObjectsTransform(arm_0_joint_pos, arm_1_joint_pos);
                scene_->cd_fcl_->getInfoForVisualizeRobot(poses, geometry_types, geometry_params);
                visualizer_->publishRobotCollisionShape(poses, geometry_types, geometry_params);
                return false;
            }
        }

        // DLO
        if (dlo_->dlo_length_ <= 0.0)
        {
            ROS_ERROR_COND(dlo_->dlo_length_ <= 0.0, "checkPlanningRequest(): The dlo length is invalid.");
            return false;
        }
        if (req.b_check_start_goal_collision_ && scene_->checkDLOAndWorldCollision(req.start_dlo_state_, collision_check_min_dist_thres_))
        {
            ROS_ERROR_STREAM("checkPlanningRequest(): The start DLO state is in collision.");
            return false;
        }
        if (req.b_check_start_goal_collision_ && scene_->checkDLOAndWorldCollision(req.goal_dlo_state_, collision_check_min_dist_thres_))
        {
            ROS_ERROR_STREAM("checkPlanningRequest(): The goal DLO state is in collision.");
            return false;
        }

        return true;
    }

    // -------------------------------------------------------
    Node::Ptr PlannerBase::sampleGoalIKNode(
        const DLOState &dlo_state)
    {
        Node::Ptr goal_ik_node;

        while (ros::ok())
        {
            Eigen::VectorXd arm_0_joint_pos, arm_1_joint_pos;
            bool arm_0_success = dual_arm_->arm_0_->armTcpRandomIK(dlo_state.getLeftEndPose(), arm_0_joint_pos);
            if (!arm_0_success)
                continue;
            bool arm_1_success = dual_arm_->arm_1_->armTcpRandomIK(dlo_state.getRightEndPose(), arm_1_joint_pos);
            if (!arm_1_success)
                continue;

            goal_ik_node = std::make_shared<Node>(dlo_state, arm_0_joint_pos, arm_1_joint_pos, nullptr, "goal_ik", /*cost_from_init*/ 0.0);
            if (!checkNodeCollision(goal_ik_node))
            {
                break;
            }
        }

        updateNode(goal_ik_node);

        return goal_ik_node;
    }

    // -------------------------------------------------------
    void PlannerBase::addNewGoalIKNode(
        const DLOState &dlo_state,
        std::vector<Node::Ptr> &node_list_goal)
    {
        int attemp_number = node_list_goal.size() == 0 ? num_random_sample_goal_ : 1;

        for (int i = 0; i < attemp_number; i++)
        {
            Node::Ptr goal_ik_node = sampleGoalIKNode(dlo_state);

            if (node_list_goal.size() == 0)
            {
                node_list_goal.push_back(goal_ik_node);
                ROS_DEBUG_STREAM("addNewGoalIKNode(): add new goal ik node.");
            }
            else
            {
                // add the new node to the goal tree only if the new node cannot be connected to the existing nodes
                Node::Ptr nearest_node = getNearestNode(node_list_goal, goal_ik_node, "joint_pos");
                if (!checkTwoNodeCanConnect(goal_ik_node, nearest_node, connect_cartesian_max_dist_, connect_joint_max_dist_))
                {
                    node_list_goal.push_back(goal_ik_node);
                    ROS_DEBUG_STREAM("addNewGoalIKNode(): add new goal ik node.");
                }
                else
                {
                    ROS_DEBUG_STREAM("addNewGoalIKNode(): sampling existing goal ik node.");
                }
            }
        }
    }

    // -------------------------------------------------------
    void PlannerBase::printPlanningDetails()
    {
        ROS_INFO_STREAM("Details:");

        ROS_INFO_STREAM("Find feasible path: time cost: " << time_cost_find_feasible_path_ << ", rrt iter: " << rrt_iter_find_feasible_path_ << ", path length: " << feasible_path_length_);
        ROS_INFO_STREAM("Path smooth: " << time_cost_path_smooth_ << ", path length: " << smooth_path_length_);

        ROS_INFO_STREAM("dlo projection: "
                        << " total time cost: " << time_cost_dlo_projection_ << ", count: " << count_dlo_projection_ << ", average time cost: " << time_cost_dlo_projection_ / count_dlo_projection_);
        ROS_INFO_STREAM("arm projection: "
                        << " total time cost: " << time_cost_arm_projection_ << ", count: " << count_arm_projection_ << ", average time cost: " << time_cost_arm_projection_ / count_arm_projection_);
        ROS_INFO_STREAM("arm ik: "
                        << " total time cost: " << time_cost_arm_ik_ << ", count: " << count_arm_ik_ << ", average time cost: " << time_cost_arm_ik_ / count_arm_ik_);
        ROS_INFO_STREAM("collision detection: "
                        << " total time cost: " << time_cost_collision_detection_ << ", count: " << count_collision_detection_ << ", average time cost: " << time_cost_collision_detection_ / count_collision_detection_);
        ROS_INFO_STREAM("nearest node search: "
                        << " total time cost: " << time_cost_nearest_node_search_ << ", count: " << count_nearest_node_search_ << ", average time cost: " << time_cost_nearest_node_search_ / count_nearest_node_search_);

        ROS_INFO_STREAM("the rest of the time cost: " << time_cost_find_feasible_path_ + time_cost_path_smooth_ - time_cost_dlo_projection_ - time_cost_arm_projection_ - time_cost_arm_ik_ - time_cost_collision_detection_ - time_cost_nearest_node_search_);

        ROS_INFO_STREAM("random sampling: "
                        << " total time cost: " << time_cost_random_sample_ << ", count: " << count_random_sample_ << ", average time cost: " << time_cost_random_sample_ / count_random_sample_ << ", average iter: " << double(count_random_sample_iter_) / count_random_sample_);
    }

    // -------------------------------------------------------
    void PlannerBase::writePlanningDetailsToResponse(
        PlanningResponse &res)
    {
        res.success_ = true;
        res.total_time_cost_ = time_cost_find_feasible_path_;
        res.total_iter_ = rrt_iter_find_feasible_path_;
        res.path_cost_ = smooth_path_length_;

        // details
        res.details_["rrt_iter_find_feasible_path"] = rrt_iter_find_feasible_path_;
        res.details_["time_cost_find_feasible_path"] = time_cost_find_feasible_path_;
        res.details_["path_length_feasible"] = feasible_path_length_;

        res.details_["time_cost_path_smooth"] = time_cost_path_smooth_;
        res.details_["path_length_smooth"] = smooth_path_length_;

        res.details_["count_dlo_projection"] = count_dlo_projection_;
        res.details_["time_cost_dlo_projection"] = time_cost_dlo_projection_;
        res.details_["ave_time_cost_dlo_projection"] = time_cost_dlo_projection_ / count_dlo_projection_;

        res.details_["count_arm_projection"] = count_arm_projection_;
        res.details_["time_cost_arm_projection"] = time_cost_arm_projection_;
        res.details_["ave_time_cost_arm_projection"] = time_cost_arm_projection_ / count_arm_projection_;

        res.details_["count_arm_ik"] = count_arm_ik_;
        res.details_["time_cost_arm_ik"] = time_cost_arm_ik_;
        res.details_["ave_time_cost_arm_ik"] = time_cost_arm_ik_ / count_arm_ik_;

        res.details_["count_collision_detection"] = count_collision_detection_;
        res.details_["time_cost_collision_detection"] = time_cost_collision_detection_;
        res.details_["ave_time_cost_collision_detection"] = time_cost_collision_detection_ / count_collision_detection_;

        res.details_["count_nearest_node_search"] = count_nearest_node_search_;
        res.details_["time_cost_nearest_node_search"] = time_cost_nearest_node_search_;
        res.details_["ave_time_cost_nearest_node_search"] = time_cost_nearest_node_search_ / count_nearest_node_search_;

        res.details_["count_random_sample"] = count_random_sample_;
        res.details_["count_random_sample_iter"] = count_random_sample_iter_;
        res.details_["time_cost_random_sample"] = time_cost_random_sample_;
        res.details_["ave_time_cost_random_sample"] = time_cost_random_sample_ / count_random_sample_;
    }

} // end namespace