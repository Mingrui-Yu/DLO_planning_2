#include "dlo_arm_planning_pkg/planner/joint_birrt.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    JointBiRRT::JointBiRRT(
        const ros::NodeHandle &nh,
        const Scene::Ptr &scene) : PlannerBase(nh, scene)
    {
        loadParams();

        b_node_tcp_pose_ = true;
        b_node_links_pos_ = true;
    }

    // -------------------------------------------------------
    void JointBiRRT::loadParams()
    {
        std::string param_name;

        param_name = "planner_configs/" + ALGORITHM_NAME + "/sample_only_dlo_probability";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, sample_only_dlo_probability_);
    }

    // -------------------------------------------------------
    Node::Ptr JointBiRRT::randomSampleNodeDloOnly()
    {
        bool b_check_collision = false;
        bool b_dlo_projection = false;

        std::chrono::steady_clock::time_point t1_random_sample = std::chrono::steady_clock::now();

        DLOState rand_dlo_state;

        while (ros::ok())
        {
            count_random_sample_iter_++;

            rand_dlo_state = dlo_->sampleCoarseStateType1(dlo_sample_lb_, dlo_sample_ub_);

            if (b_dlo_projection)
            {
                std::chrono::steady_clock::time_point t1_dlo_projection = std::chrono::steady_clock::now();
                rand_dlo_state = dlo_->optimizeShapeDerm(rand_dlo_state);
                std::chrono::steady_clock::time_point t2_dlo_projection = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used_dlo_projection = std::chrono::duration_cast<std::chrono::duration<double>>(t2_dlo_projection - t1_dlo_projection);
                time_cost_dlo_projection_ += time_used_dlo_projection.count();
                count_dlo_projection_++;
            }

            if (b_check_collision)
            {
                if (scene_->checkDLOAndWorldCollision(rand_dlo_state))
                    continue;
            }
            break;
        }

        Eigen::VectorXd empty_vector = Eigen::VectorXd::Zero(0);
        Node::Ptr rand_node = std::make_shared<Node>(rand_dlo_state,
                                                     /*arm_0_joint_pos*/ empty_vector, /*arm_1_joint_pos*/ empty_vector, /*parent*/ nullptr, "rand");

        ROS_DEBUG_STREAM("randomSampleNode(): generate rand node.");

        std::chrono::steady_clock::time_point t2_random_sample = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_random_sample = std::chrono::duration_cast<std::chrono::duration<double>>(t2_random_sample - t1_random_sample);
        time_cost_random_sample_ += time_used_random_sample.count();
        count_random_sample_++;

        return rand_node;
    }

    // -------------------------------------------------------
    Node::Ptr JointBiRRT::randomSampleNode()
    {
        bool b_check_collision = false;
        bool b_use_ik = true;
        bool b_use_reachability_check = false;
        bool b_dlo_projection = false;

        std::chrono::steady_clock::time_point t1_random_sample = std::chrono::steady_clock::now();

        Node::Ptr rand_node;
        while (ros::ok())
        {
            count_random_sample_iter_++;

            DLOState rand_dlo_state = dlo_->sampleCoarseStateType1(dlo_sample_lb_, dlo_sample_ub_);

            // 采样机械臂关节角
            Eigen::VectorXd arm_0_joint_pos, arm_1_joint_pos;
            if (b_use_ik)
            {
                if (b_use_reachability_check)
                {
                    // 使用 arm reachability space 进行预先于 IK 的判断
                    if (!dual_arm_->arm_0_->checkReachabilityByReachSpace(rand_dlo_state.getLeftEndPose()))
                        continue;
                    if (!dual_arm_->arm_1_->checkReachabilityByReachSpace(rand_dlo_state.getRightEndPose()))
                        continue;
                }

                std::chrono::steady_clock::time_point t1_ik = std::chrono::steady_clock::now();

                // bool arm_0_ik_success = dual_arm_->arm_0_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), rand_dlo_state.getLeftEndPose(), arm_0_joint_pos);
                bool arm_0_ik_success = dual_arm_->arm_0_->armTcpRandomIK(rand_dlo_state.getLeftEndPose(), arm_0_joint_pos);

                std::chrono::steady_clock::time_point t2_ik = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used_ik = std::chrono::duration_cast<std::chrono::duration<double>>(t2_ik - t1_ik);
                time_cost_arm_ik_ += time_used_ik.count();
                count_arm_ik_++;

                if (!arm_0_ik_success)
                    continue;

                t1_ik = std::chrono::steady_clock::now();

                // bool arm_1_ik_success = dual_arm_->arm_1_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), rand_dlo_state.getRightEndPose(), arm_1_joint_pos);
                bool arm_1_ik_success = dual_arm_->arm_1_->armTcpRandomIK(rand_dlo_state.getRightEndPose(), arm_1_joint_pos);

                t2_ik = std::chrono::steady_clock::now();
                time_used_ik = std::chrono::duration_cast<std::chrono::duration<double>>(t2_ik - t1_ik);
                time_cost_arm_ik_ += time_used_ik.count();
                count_arm_ik_++;

                if (!arm_1_ik_success)
                    continue;
            }
            else
            {
                arm_0_joint_pos = dual_arm_->arm_0_->randomJointPos();
                arm_1_joint_pos = dual_arm_->arm_1_->randomJointPos();
            }

            if (b_check_collision)
            {
                bool dual_arm_collision = scene_->checkRobotCollision(arm_0_joint_pos, arm_1_joint_pos);
                if (dual_arm_collision)
                    continue;
            }

            if (b_dlo_projection)
            {
                std::chrono::steady_clock::time_point t1_dlo_projection = std::chrono::steady_clock::now();
                rand_dlo_state = dlo_->optimizeShapeDerm(rand_dlo_state);
                std::chrono::steady_clock::time_point t2_dlo_projection = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used_dlo_projection = std::chrono::duration_cast<std::chrono::duration<double>>(t2_dlo_projection - t1_dlo_projection);
                time_cost_dlo_projection_ += time_used_dlo_projection.count();
                count_dlo_projection_++;
            }

            if (b_check_collision)
            {
                bool dlo_collision = scene_->checkDLOAndWorldCollision(rand_dlo_state);
                if (dlo_collision)
                    continue;
            }

            rand_node = std::make_shared<Node>(rand_dlo_state, arm_0_joint_pos, arm_1_joint_pos, /*parent*/ nullptr, "rand");
            break;
        }

        updateNode(rand_node);

        ROS_DEBUG_STREAM("randomSampleNode(): generate rand node.");

        std::chrono::steady_clock::time_point t2_random_sample = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_random_sample = std::chrono::duration_cast<std::chrono::duration<double>>(t2_random_sample - t1_random_sample);
        time_cost_random_sample_ += time_used_random_sample.count();
        count_random_sample_++;

        return rand_node;
    }

    // -------------------------------------------------------
    bool JointBiRRT::checkTwoNodeCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        const double dlo_max_dist,
        const double arm_max_dist)
    {
        bool dlo_close_enough = (twoNodeDistance(from_node, to_node, "dlo_fps_pos_max_dist")) < dlo_max_dist;
        if (!dlo_close_enough)
        {
            ROS_DEBUG_STREAM("checkTwoNodeCanConnect(): DLO is not close enough.");
            return false;
        }

        if (to_node->note_ != "rand")
        {
            double dual_arm_scale_factor = std::sqrt(from_node->dual_arm_joint_pos_.size());
            bool dual_arm_close_enough = (twoNodeDistance(from_node, to_node, "joint_pos") / dual_arm_scale_factor) < arm_max_dist;
            if (!dual_arm_close_enough)
            {
                ROS_DEBUG_STREAM("checkTwoNodeCanConnect(): dual arm is not close enough.");
                return false;
            }

            bool no_collision = !checkTwoNodePathCollision(from_node, to_node);
            if (!no_collision)
            {
                ROS_DEBUG_STREAM("checkTwoNodeCanConnect(): the path between the two nodes is in collision.");
                return false;
            }
        }

        return true;
    }

    // -------------------------------------------------------
    Node::Ptr JointBiRRT::oneStepSteer(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node)
    {
        // compute the interpolation ratio t
        double t = dlo_->calcInterpolationRatio(from_node->dlo_state_, to_node->dlo_state_,
                                                steer_dlo_cartesian_step_size_, steer_dlo_angle_step_size_);
        if (to_node->arm_0_joint_pos_.size() != 0)
        {
            for (int j = 0; j < dual_arm_->arm_0_->joint_num_; j++)
                t = std::min(t, steer_joint_step_size_ / std::abs(from_node->arm_0_joint_pos_(j) - to_node->arm_0_joint_pos_(j)));
            for (int j = 0; j < dual_arm_->arm_1_->joint_num_; j++)
                t = std::min(t, steer_joint_step_size_ / std::abs(from_node->arm_1_joint_pos_(j) - to_node->arm_1_joint_pos_(j)));
        }
        ROS_DEBUG_STREAM("oneStepSteer(): interpolation t: " << t);

        // DLO interpolation
        DLOState new_dlo_state = dlo_->interpolate(from_node->dlo_state_, to_node->dlo_state_, t);

        // DLO projection (kinodynamic prediction)
        if (use_underactuated_steer_)
        {
            // set the initial value of non-end fps_pos as 'from_node->dlo_state_.fps_pos_'
            for (size_t k = 1; k < dlo_->num_fps_ - 1; k++)
            {
                new_dlo_state.setFpPos(k, from_node->dlo_state_.getFpPos(k));
            }
            new_dlo_state.updateDependentInfo(/*theta_n_init*/ new_dlo_state.getThetaN());
        }

        std::chrono::steady_clock::time_point t1_dlo_projection = std::chrono::steady_clock::now();

        OptimizeOptions optimize_options;
        optimize_options.function_tolerance = dlo_projection_function_tolerance_;
        optimize_options.max_num_iterations = dlo_projection_max_num_iterations_;
        new_dlo_state = dlo_->optimizeShapeDerm(new_dlo_state, optimize_options);

        std::chrono::steady_clock::time_point t2_dlo_projection = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_dlo_projection = std::chrono::duration_cast<std::chrono::duration<double>>(t2_dlo_projection - t1_dlo_projection);
        time_cost_dlo_projection_ += time_used_dlo_projection.count();
        count_dlo_projection_++;

        if (req_.b_visualize_planning_process_)
        {
            visualizer_->publishDLOState(new_dlo_state, Eigen::Vector3d(1.0, 0.0, 0.0));
            visualizer_->publishText("steer: DLO shape after interpolation & optimization");
            ros::Duration(1).sleep();
        }

        // dual arm interpolation
        Eigen::VectorXd new_arm_0_joint_pos, new_arm_1_joint_pos;
        if (to_node->arm_0_joint_pos_.size() == 0)
        {
            new_arm_0_joint_pos = from_node->arm_0_joint_pos_;
            new_arm_1_joint_pos = from_node->arm_1_joint_pos_;
        }
        else
        {
            new_arm_0_joint_pos = from_node->arm_0_joint_pos_ * (1 - t) + to_node->arm_0_joint_pos_ * t;
            new_arm_1_joint_pos = from_node->arm_1_joint_pos_ * (1 - t) + to_node->arm_1_joint_pos_ * t;
        }

        Node::Ptr new_node = std::make_shared<Node>(new_dlo_state, new_arm_0_joint_pos, new_arm_1_joint_pos, nullptr, "steer");

        // closed-chain projection
        std::chrono::steady_clock::time_point t1_arm_projection = std::chrono::steady_clock::now();

        bool success = constrainConfig(new_node, from_node);

        std::chrono::steady_clock::time_point t2_arm_projection = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_arm_projection = std::chrono::duration_cast<std::chrono::duration<double>>(t2_arm_projection - t1_arm_projection);
        time_cost_arm_projection_ += time_used_arm_projection.count();
        count_arm_projection_++;

        if (!success)
            return nullptr;

        updateNode(new_node);

        ROS_DEBUG_STREAM("oneStepSteer(): generate new node.");

        return new_node;
    }

    // -------------------------------------------------------
    Node::Ptr JointBiRRT::extend(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        bool greedy)
    {
        int step = 0;
        Node::Ptr s_node = from_node;
        Node::Ptr s_node_old = from_node;

        while (ros::ok() && (greedy || step < extend_max_steps_))
        {

            if (checkTwoNodeCanConnect(s_node, to_node, /*dlo_max_dist*/ 1e-2, /*arm_max_dist*/ 1e-2))
            {
                ROS_DEBUG_STREAM("extend(): s_node can connect to to_node.");
                return s_node;
            }

            if (s_node != s_node_old)
            {
                if (twoNodeDistance(s_node, to_node, "dlo_fps_pos") > twoNodeDistance(s_node_old, to_node, "dlo_fps_pos") - 1e-3)
                {
                    ROS_DEBUG_STREAM("extend(): s_node is not closer to to_node than s_node_old.");
                    return s_node_old;
                }
            }

            s_node_old = s_node;

            // one step steer
            s_node = oneStepSteer(s_node, to_node);

            if (s_node == nullptr)
            {
                ROS_DEBUG_STREAM("extend(): s_node == nullptr");
                return s_node_old;
            }
            else if (checkNodeCollision(s_node))
            {
                ROS_DEBUG_STREAM("extend(): s_node is in collision.");
                return s_node_old;
            }
            else if (!checkTwoNodeCanConnect(s_node_old, s_node,
                                             /*dlo_max_dist*/ connect_cartesian_max_dist_, /*arm_max_dist*/ connect_joint_max_dist_))
            {
                ROS_DEBUG_STREAM("extend(): s_node_old cannot connect to s_node.");
                return s_node_old;
            }
            else
            { // if new node generated by oneStepSteer() is valid
                s_node->parent_ = s_node_old;
                node_list.push_back(s_node);
            }

            if (req_.b_visualize_planning_process_)
            {
                visualizer_->publishNode(s_node, "s_node while extending");
                ros::Duration(1).sleep();
            }

            step++;
        }

        ROS_DEBUG_STREAM("extend(): have done max steer steps.");
        return s_node;
    }

    // -------------------------------------------------------
    bool JointBiRRT::solve(
        const PlanningRequest &req,
        PlanningResponse &res)
    {
        ROS_INFO_STREAM("params: sample_only_dlo_probability_: " << sample_only_dlo_probability_);
        ROS_INFO_STREAM("params: use_underactuated_steer_: " << use_underactuated_steer_);

        if (!checkPlanningRequest(req))
        {
            res.success_ = false;
            ROS_WARN_STREAM("Failed to find feasible path.");
            return false;
        }

        req_ = req;

        // start and goal node
        Node::Ptr start_node = std::make_shared<Node>(req.start_dlo_state_,
                                                      Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_), Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_), nullptr, "start");
        updateNode(start_node);

        Node::Ptr goal_node;
        if (req.goal_type_ == "joint_space")
        {
            goal_node = std::make_shared<Node>(req.goal_dlo_state_,
                                               Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_), Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_), nullptr, "goal");
            updateNode(goal_node);
        }
        else
        {
            goal_node = std::make_shared<Node>(req.goal_dlo_state_, Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0), nullptr, "goal");
        }

        if (req.b_visualize_start_goal_)
        {
            visualizer_->publishNode(start_node, "start_node");
            ros::Duration(1).sleep();

            visualizer_->publishNode(goal_node, "goal_node");
            ros::Duration(1).sleep();

            visualizer_->publishText("planning ...");
        }

        // setup the two trees
        std::vector<Node::Ptr> node_list_a{start_node};
        std::vector<Node::Ptr> node_list_b;
        if (req.goal_type_ == "joint_space")
            node_list_b.push_back(goal_node);

        std::chrono::steady_clock::time_point t_begin, t_end;
        std::chrono::duration<double> time_used;

        // rrt main loop
        t_begin = std::chrono::steady_clock::now();
        int iter = 0;
        while (ros::ok() && iter < rrt_max_iter_)
        {
            if (req.goal_type_ == "task_space")
            {
                auto &node_list_goal = (node_list_a[0]->note_ == "start") ? node_list_b : node_list_a;
                bool b_new_goal_ik = (node_list_goal.size() == 0) || (Utils::getRandomDouble() < new_goal_ik_probability_);
                if (b_new_goal_ik)
                {
                    addNewGoalIKNode(req.goal_dlo_state_, node_list_goal);
                }
            }

            Node::Ptr rand_node, nearest_node_a;
            if (Utils::getRandomDouble() < sample_only_dlo_probability_)
            { // assistant task-space guided sampling
                rand_node = randomSampleNodeDloOnly();
                ROS_DEBUG_STREAM("solve(): tree A generate rand node.");
                nearest_node_a = getNearestNode(node_list_a, rand_node, "dlo_fps_pos");
                ROS_DEBUG_STREAM("solve(): tree A generate nearest node.");
            }
            else
            {
                rand_node = randomSampleNode();
                ROS_DEBUG_STREAM("solve(): tree A generate rand node.");
                nearest_node_a = getNearestNode(node_list_a, rand_node, "arm_and_dlo_pos_max_dist");
                ROS_DEBUG_STREAM("solve(): tree A generate nearest node.");
            }

            if (req.b_visualize_planning_process_)
            {
                visualizer_->publishNode(rand_node, "rand_node");
                ros::Duration(1).sleep();
                visualizer_->publishNode(nearest_node_a, "nearest_node_a");
                ros::Duration(1).sleep();
            }

            Node::Ptr reached_node_a = extend(node_list_a, nearest_node_a, rand_node, /*greedy*/ false);
            ROS_DEBUG_STREAM("solve(): tree A generate reached node.");

            if (req.b_visualize_planning_process_)
            {
                visualizer_->publishNode(reached_node_a, "reached_node_a");
                ros::Duration(1).sleep();
            }

            // if tree A successfully explores
            if (reached_node_a != nearest_node_a)
            {
                Node::Ptr nearest_node_b = getNearestNode(node_list_b, reached_node_a, "arm_and_dlo_pos_max_dist");
                ROS_DEBUG_STREAM("solve(): tree B generate nearest node.");

                if (req.b_visualize_planning_process_)
                {
                    visualizer_->publishNode(nearest_node_b, "nearest_node_b");
                    ros::Duration(1).sleep();
                }

                Node::Ptr reached_node_b = extend(node_list_b, nearest_node_b, reached_node_a, /*greedy*/ true);
                ROS_DEBUG_STREAM("solve(): tree B generate nearest node.");

                if (req.b_visualize_planning_process_)
                {
                    visualizer_->publishNode(reached_node_b, "reached_node_b");
                    ros::Duration(1).sleep();
                }

                // if finding a feasible path
                if (checkTwoNodeCanConnect(reached_node_a, reached_node_b, /*dlo_max_dist*/ 1e-2, /*arm_max_dist*/ 1e-2))
                {
                    ROS_INFO_STREAM(ALGORITHM_NAME + " find feasible path");

                    if (node_list_b[0]->note_ == "start")
                    {
                        swapNodes(reached_node_a, reached_node_b);
                    }

                    std::vector<Node::Ptr> res_path_list = pathExtract(reached_node_a, reached_node_b);

                    t_end = std::chrono::steady_clock::now();
                    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin);
                    time_cost_find_feasible_path_ = time_used.count();
                    rrt_iter_find_feasible_path_ = iter;
                    feasible_path_length_ = pathLength(res_path_list);

                    t_begin = std::chrono::steady_clock::now();

                    // smooth the feasible path
                    pathSmooth(res_path_list);

                    t_end = std::chrono::steady_clock::now();
                    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin);
                    time_cost_path_smooth_ = time_used.count();
                    smooth_path_length_ = pathLength(res_path_list);

                    printPlanningDetails();

                    if (req.b_visualize_res_path_)
                    {
                        visualizer_->publishText("planned path");
                        visualizer_->publishNodePath(pathInterpolation(res_path_list), /*ros_rate*/ 50);
                    }

                    nodeListToPath(res_path_list, res.smoothed_path_);
                    writePlanningDetailsToResponse(res);

                    return true;
                }
            }

            // balance the two trees
            if (node_list_a.size() > node_list_b.size())
            {
                swapTrees(node_list_a, node_list_b);
            }

            ROS_DEBUG_STREAM("solve(): rrt iter " << iter << " done.");
            iter++;
        }

        res.success_ = false;
        ROS_WARN_STREAM("Failed to find feasible path.");
        return false;
    }

} // end namespace
