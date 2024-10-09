#include "dlo_arm_planning_pkg/controller.h"
#include "dlo_arm_planning_pkg/cnpy_utils.h"

#include <unistd.h>
#include <termios.h>

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    Controller::Controller(
        const ros::NodeHandle &nh) : nh_(nh)
    {
        const std::string robot_description_name = "robot_description";
        const std::string arm_0_group_name = "arm_0";
        const std::string arm_1_group_name = "arm_1";
        const std::string dual_arm_group_name = "dual_arm";

        loadParams();

        if (sim_or_real_ == "sim")
            scene_dir_ = project_dir_ + "data/" + sim_or_real_ + "/scene_" + std::to_string(scene_id_) + "/";
        else if (sim_or_real_ == "real")
            scene_dir_ = project_dir_ + "data/" + sim_or_real_ + "/scene_" + std::to_string(scene_id_) + "/dlo_" + std::to_string(dlo_id_) + "/";
        else
            ROS_ERROR_STREAM("Invalid 'sim_or_real': " << sim_or_real_);

        dlo_ = std::make_shared<DLO>(nh_); // notice: no length is specified
        dual_arm_ = std::make_shared<DualArm>(nh_, robot_description_name, arm_0_group_name,
                                              arm_1_group_name, dual_arm_group_name);

        real_time_interface_ = std::make_shared<RealTimeInterface>(nh_, dlo_, dual_arm_);
        planning_interface_ = std::make_shared<PlanningInterface>(nh_);

        dlo_jaco_model_ = std::make_shared<JacobianModel>(nh_);
        dlo_jaco_model_->loadAllWeights(project_dir_ + dlo_jaco_offline_model_dir_ + env_dimension_ + "/" + dlo_jaco_offline_model_name_ + "/");

        scene_ = std::make_shared<Scene>(nh_, dlo_, dual_arm_);
        scene_->setPlanningScene(real_time_interface_->getLockedPlanningScene()); // only set once, since it is very time-consuming; make sure that the planning scene monitor of move group has already built the scene before.
        real_time_interface_->planning_visualizer_->setScene(scene_);
        real_time_interface_->rt_visualizer_->setScene(scene_);

        mpc_ = std::make_shared<MultiStepMPC>(scene_, dlo_jaco_model_, /*n_step*/ mpc_n_step_);
    }

    // -------------------------------------------------------
    void Controller::loadParams()
    {
        std::string param_name;
        param_name = "project_configs/project_dir";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, project_dir_);

        param_name = "env_configs/sim_or_real";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, sim_or_real_);

        param_name = "env_configs/dimension";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, env_dimension_);

        param_name = "env_configs/scene_id";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, scene_id_);

        if (sim_or_real_ == "real")
        {
            param_name = "dlo_configs/id";
            ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, dlo_id_);
        }

        // ------------------------ dlo jacobian model ------------------------------
        param_name = "dlo_jacobian_model_configs/offline_model_dir";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_jaco_offline_model_dir_);

        param_name = "dlo_jacobian_model_configs/offline_model_name";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_jaco_offline_model_name_);

        // ------------------------ controller ------------------------------

        param_name = "controller_configs/control_rate";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, control_rate_);

        param_name = "controller_configs/horizon_step";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, mpc_n_step_);

        param_name = "controller_configs/min_dist_thres";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, mpc_min_dist_thres_);

        param_name = "controller_configs/control_input_delay_time";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, control_input_delay_time_);

        param_name = "controller_configs/overstretch_thres";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, overstretch_thres_);

        param_name = "controller_configs/dlo_fps_pos_cost_weight";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_fps_pos_cost_weight_);

        param_name = "controller_configs/arm_joint_pos_cost_weight";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, arm_joint_pos_cost_weight_);

        param_name = "controller_configs/arm_joint_vel_cost_weight";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, arm_joint_vel_cost_weight_);

        param_name = "controller_configs/arm_joint_acce_cost_weight";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, arm_joint_acce_cost_weight_);

        param_name = "controller_configs/arm_0_max_joint_vel";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_0_max_joint_vel;
        nh_.getParam(param_name, arm_0_max_joint_vel);
        arm_0_max_joint_vel_ = Utils::stdVector2EigenVectorXd(arm_0_max_joint_vel);

        param_name = "controller_configs/arm_1_max_joint_vel";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_1_max_joint_vel;
        nh_.getParam(param_name, arm_1_max_joint_vel);
        arm_1_max_joint_vel_ = Utils::stdVector2EigenVectorXd(arm_1_max_joint_vel);

        param_name = "controller_configs/arm_0_joint_pos_weights";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_0_joint_pos_weights;
        nh_.getParam(param_name, arm_0_joint_pos_weights);
        arm_0_joint_pos_weights_ = Utils::stdVector2EigenVectorXd(arm_0_joint_pos_weights);

        param_name = "controller_configs/arm_1_joint_pos_weights";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_1_joint_pos_weights;
        nh_.getParam(param_name, arm_1_joint_pos_weights);
        arm_1_joint_pos_weights_ = Utils::stdVector2EigenVectorXd(arm_1_joint_pos_weights);

        param_name = "controller_configs/arm_0_joint_vel_weights";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_0_joint_vel_weights;
        nh_.getParam(param_name, arm_0_joint_vel_weights);
        arm_0_joint_vel_weights_ = Utils::stdVector2EigenVectorXd(arm_0_joint_vel_weights);

        param_name = "controller_configs/arm_1_joint_vel_weights";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        std::vector<double> arm_1_joint_vel_weights;
        nh_.getParam(param_name, arm_1_joint_vel_weights);
        arm_1_joint_vel_weights_ = Utils::stdVector2EigenVectorXd(arm_1_joint_vel_weights);

        param_name = "controller_configs/dlo_fps_max_vel";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_fps_max_vel_);

        param_name = "controller_configs/max_time_for_final_waypoint";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, max_time_for_final_waypoint_);

        param_name = "controller_configs/max_time_for_manipulation";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, max_time_for_manipulation_);

        param_name = "controller_configs/path_vel_scale_factor";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_vel_scale_factor_);

        param_name = "controller_configs/path_dlo_max_vel";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_dlo_max_vel_);

        param_name = "controller_configs/path_dlo_max_avel";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_dlo_max_avel_);

        param_name = "controller_configs/success_error_thres";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, success_error_thres_);

        param_name = "planner_configs/common/path_collision_check_cartesian_step_size";
        ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, path_collision_check_cartesian_step_size_);
    }

    // -------------------------------------------------------
    PlanningResponse Controller::planPathToGoal(
        PlanningRequest &req,
        const DLOState &goal_dlo_state,
        const Eigen::VectorXd arm_0_goal_joint_pos,
        const Eigen::VectorXd arm_1_goal_joint_pos)
    {
        auto robot_obs = real_time_interface_->getRobotObservation();

        PlanningResponse res;

        // dlo
        req.start_dlo_state_ = real_time_interface_->getDLOState();
        req.goal_dlo_state_ = goal_dlo_state;

        // dual arm
        req.arm_0_group_name_ = dual_arm_->arm_0_->arm_group_name_;
        req.arm_1_group_name_ = dual_arm_->arm_1_->arm_group_name_;
        req.dual_arm_group_name_ = dual_arm_->dual_arm_group_name_;

        // use the current joint positions as the start_joint_pos
        req.arm_0_start_joint_pos_ = Utils::eigenVectorXd2StdVector(real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_));
        req.arm_1_start_joint_pos_ = Utils::eigenVectorXd2StdVector(real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_));

        if (arm_0_goal_joint_pos.size() != 0 && arm_1_goal_joint_pos.size() != 0)
        {
            req.goal_type_ = "joint_space";
            req.arm_0_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_goal_joint_pos);
            req.arm_1_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_goal_joint_pos);
        }
        else
        {
            req.goal_type_ = "task_space";
        }

        req.b_visualize_start_goal_ = false;
        req.b_visualize_planning_process_ = false;
        req.b_visualize_res_path_ = false;

        if (sim_or_real_ == "real")
        {
            req.b_visualize_start_goal_ = true;
            req.b_visualize_planning_process_ = false;
            req.b_visualize_res_path_ = true;
        }

        planning_interface_->solve(scene_, "JointBiRRT", req, res, /*visualizer (optional)*/ real_time_interface_->planning_visualizer_);

        return res;
    }

    // -------------------------------------------------------
    void Controller::onlineUpdatingDLOJacobian(
        const bool restart)
    {
        static int n_count = 0;
        static DLOState dlo_last_state;
        static Eigen::VectorXd arm_0_last_joint_pos, arm_1_last_joint_pos;
        static ros::Time last_time;

        if (restart)
            n_count = 0;

        DLOState dlo_state = real_time_interface_->getDLOState();
        Eigen::VectorXd arm_0_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
        Eigen::VectorXd arm_1_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);
        ros::Time current_time = ros::Time::now();

        if (n_count > 0)
        {
            double delta_t = (current_time - last_time).toSec();
            Eigen::VectorXd fps_vel = (dlo_state.fps_pos_ - dlo_last_state.fps_pos_) / delta_t;

            Eigen::VectorXd arm_0_joint_vel = (arm_0_joint_pos - arm_0_last_joint_pos) / delta_t;
            Eigen::VectorXd arm_1_joint_vel = (arm_1_joint_pos - arm_1_last_joint_pos) / delta_t;

            Eigen::VectorXd end_0_vel = dual_arm_->arm_0_->getTcpJacobianMatrix(arm_0_joint_pos) * arm_0_joint_vel;
            Eigen::VectorXd end_1_vel = dual_arm_->arm_1_->getTcpJacobianMatrix(arm_1_joint_pos) * arm_1_joint_vel;

            OnlineData online_data;
            online_data.dlo_state = dlo_state;
            online_data.fps_vel = fps_vel;
            online_data.ends_vel = Utils::concatenateTwoVector(end_0_vel, end_1_vel);

            dlo_jaco_model_->onlineUpdating(online_data, dlo_->dlo_length_);
        }

        n_count++;
        dlo_last_state = dlo_state;
        arm_0_last_joint_pos = arm_0_joint_pos;
        arm_1_last_joint_pos = arm_1_joint_pos;
        last_time = current_time;
    }

    // -------------------------------------------------------
    bool Controller::oneExecution(
        const int task_id,
        const int path_id,
        const int execution_id,
        const ExecutionOptions &options)
    {
        loadDLOParams();

        std::string task_dir = scene_dir_ + "task_" + std::to_string(task_id) + "/";
        std::string path_dir = task_dir + "results/" + options.planner_name + "/path_" + std::to_string(path_id) + "/";
        std::string execution_dir = path_dir + "/" + options.execution_name + "/execution_" + std::to_string(execution_id) + "/";
        Utils::createDirectory(task_dir);
        Utils::createDirectory(path_dir);
        Utils::createDirectory(execution_dir);

        if (sim_or_real_ == "sim")
            real_time_interface_->moveAndGraspDLO();

        std::string dir;
        DLOState goal_dlo_state;
        Eigen::VectorXd arm_0_goal_joint_pos, arm_1_goal_joint_pos;
        PlanningResponse res;
        Path path;

        // ---------------- initial to start -------------------------
        if (options.plan_to_start_config)
        {
            dir = task_dir + "start/";
            CnpyUtils::loadPlanningGoal(dir, goal_dlo_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos, dlo_, dual_arm_);
            PlanningRequest req;
            res = planPathToGoal(req, goal_dlo_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos);
            if (!res.success_)
                return false;
            if (options.save_path_to_start_config)
            {
                CnpyUtils::savePath(task_dir + "path_initial_to_start/", res.smoothed_path_);
            }
            path = res.smoothed_path_;
        }
        else
        {
            if (Utils::ifFileExist(task_dir + "path_initial_to_start/"))
            {
                path = CnpyUtils::loadPath(task_dir + "path_initial_to_start/", dlo_, dual_arm_);
            }
            else
            {
                return false;
            }
        }

        ExecutionResults exe_results;
        moveToGoal(path, options.initial2start_tracking_options, exe_results);

        if (!options.execute)
        {
            return false;
        }
        ros::Duration(2.0).sleep(); // wait for the DLO to stop moving

        // ---------------- start to goal -------------------------
        dir = task_dir + "goal/";
        CnpyUtils::loadPlanningGoal(dir, goal_dlo_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos, dlo_, dual_arm_);
        real_time_interface_->publishGoalDLOState(goal_dlo_state);

        if (options.plan_to_goal_config)
        {
            PlanningRequest req;
            res = planPathToGoal(req, goal_dlo_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos);
            if (options.save_path_to_goal_config)
                savePlanningDetails(path_dir, res);
            if (!res.success_)
                return false;
            if (options.save_path_to_goal_config)
            {
                CnpyUtils::savePath(path_dir + "smoothed_path/", res.smoothed_path_);
            }
            path = res.smoothed_path_;
        }
        else
        {
            if (Utils::ifFileExist(path_dir + "smoothed_path/"))
            {
                path = CnpyUtils::loadPath(path_dir + "smoothed_path/", dlo_, dual_arm_);
            }
            else
            {
                return false;
            }
        }

        if (options.open_loop)
        {
            Path new_path;
            plannedPathToTrackedPath(path, 1.0 / control_rate_, path_dlo_max_vel_ * path_vel_scale_factor_, path_dlo_max_avel_ * path_vel_scale_factor_,
                                     arm_0_max_joint_vel_ * path_vel_scale_factor_, arm_1_max_joint_vel_ * path_vel_scale_factor_, new_path);
            real_time_interface_->dualArmPathControl(new_path.arm_0_path_, new_path.arm_1_path_, /*time_between_points*/ 1.0 / control_rate_, /*b_block*/ true);
        }
        else
        {
            ExecutionResults execution_results;
            moveToGoal(path, options.tracking_options, execution_results);
            if (options.b_save_execution_results)
                saveExecutionResults(execution_dir, execution_results);
        }

        return true;
    }

    // -------------------------------------------------------
    bool Controller::moveToGoal(
        const Path &planned_path,
        const TrackingOptions &tracking_options,
        ExecutionResults &execution_res)
    {
        ROS_DEBUG_STREAM("Controller::moveToGoal(): begin.");

        int path_length = planned_path.dlo_path_.size();
        auto dlo_goal_state = planned_path.dlo_path_[path_length - 1];
        auto arm_0_goal_joint_pos = Utils::stdVector2EigenVectorXd(planned_path.arm_0_path_[path_length - 1]);
        auto arm_1_goal_joint_pos = Utils::stdVector2EigenVectorXd(planned_path.arm_1_path_[path_length - 1]);

        execution_res.clear();
        execution_res.details["replanning_count"] = 0;
        execution_res.details["collision_time"] = 0.0;
        execution_res.details["mpc_count"] = 0.0;
        execution_res.details["mpc_total_time"] = 0.0;

        ros::Time start_time = ros::Time::now();
        double final_error;
        Path planned_path_new = planned_path;

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < max_time_for_manipulation_)
        {
            Path path;
            plannedPathToTrackedPath(planned_path_new, 1.0 / control_rate_, path_dlo_max_vel_ * path_vel_scale_factor_,
                                     path_dlo_max_avel_ * path_vel_scale_factor_, arm_0_max_joint_vel_ * path_vel_scale_factor_,
                                     arm_1_max_joint_vel_ * path_vel_scale_factor_, path);

            if (tracking_options.mode == "open_loop")
            {
                final_error = trackingPathOpenLoop(path, tracking_options, execution_res);
            }
            else
            {
                final_error = trackingPath(path, tracking_options, execution_res);
            }

            // closed-loop reach the goal, or open-loop, then break and finish the execution
            if (final_error < success_error_thres_ || tracking_options.mode == "open_loop")
            {
                ROS_DEBUG("Controller::moveToGoal(): finish one execution.");
                break;
            }

            // tracking cannot reach the final configuration, do re-planning
            while (ros::ok() && (ros::Time::now() - start_time).toSec() < max_time_for_manipulation_)
            {

                PlanningRequest req;
                PlanningResponse res = planPathToGoal(req, dlo_goal_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos);
                execution_res.details["replanning_count"] += 1;
                if (res.success_)
                {
                    planned_path_new = res.smoothed_path_;
                    break; // until re-planning succeed.
                }
                else
                {
                    ros::Duration(1.0).sleep();
                }
            }
        }

        execution_res.details["final_task_error"] = final_error;
        execution_res.details["execution_time"] = (ros::Time::now() - start_time).toSec();
        execution_res.details["ave_time_cost_mpc"] = execution_res.details["mpc_total_time"] / execution_res.details["mpc_count"];

        return (final_error <= success_error_thres_);
    }

    // -------------------------------------------------------
    double Controller::trackingPath(
        const Path &path,
        const TrackingOptions &options,
        ExecutionResults &execution_res)
    {
        ros::Rate rate(control_rate_);
        size_t path_length = path.arm_0_path_.size();

        std::chrono::steady_clock::time_point t1, t2;
        std::chrono::duration<double> time_used;
        double mpc_total_time = 0.0;
        int mpc_count = 0;
        double collision_time = 0.0;
        ros::Time start_time = ros::Time::now();
        ros::Time final_waypoint_time;
        Eigen::VectorXd arm_0_last_joint_vel, arm_1_last_joint_vel;
        DLOState dlo_last_state;

        std::vector<std::string> mode_list = {"closed_loop", "open_loop", "open_loop_replanning"};
        std::vector<std::string> state_list = {"tracking", "dlo_unstable", "away_from_obstacles", "replanning", "dlo_bad_perception"};

        std::string mode = options.mode;
        if (std::find(mode_list.begin(), mode_list.end(), mode) == mode_list.end())
            ROS_ERROR_STREAM("Controller::trackingPath(): invalid mode: " << mode);

        std::string state = "tracking";
        int waypoint_idx = 0;
        double away_from_obstacles_dist = mpc_min_dist_thres_ * 1.1;
        int current_dlo_obs_seq = 0;
        int current_robot_obs_seq = 0;

        while (ros::ok() && waypoint_idx < path_length)
        {
            if (sim_or_real_ == "real")
            {
                if (waypoint_idx == 0)
                {
                    real_time_interface_->waitForLatestObservation();
                }
                else
                {
                    real_time_interface_->waitForNextObservation(current_dlo_obs_seq, current_robot_obs_seq);
                }
                ROS_INFO_STREAM("Controller::trackingPath(): delay: " << (ros::Time::now() - real_time_interface_->getDLOobservation().header.stamp).toSec());
                ROS_INFO_STREAM("Controller::trackingPath(): current_dlo_obs_seq: " << current_dlo_obs_seq);
            }

            ROS_INFO_STREAM("Controller::trackingPath(): current state: " << state);
            if (std::find(state_list.begin(), state_list.end(), state) == state_list.end())
                ROS_ERROR_STREAM("Controller::trackingPath(): invalid state: " << state);

            current_dlo_obs_seq = real_time_interface_->getDLOobservation().header.seq;
            current_robot_obs_seq = real_time_interface_->getRobotObservation().header.seq;
            DLOState dlo_current_state = real_time_interface_->getDLOState();
            Eigen::VectorXd arm_0_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
            Eigen::VectorXd arm_1_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);

            DLOState dlo_desired_state = path.dlo_path_[waypoint_idx];
            Eigen::VectorXd arm_0_desired_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_0_path_[waypoint_idx]);
            Eigen::VectorXd arm_1_desired_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_1_path_[waypoint_idx]);

            // visualize the planned waypoint
            int planned_id = waypoint_idx - 1;
            if (waypoint_idx == 0)
                planned_id = 0;
            else if (waypoint_idx == path_length - 1)
                planned_id = waypoint_idx;
            DLOState dlo_planned_state = path.dlo_path_[planned_id];
            Eigen::VectorXd arm_0_planned_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_0_path_[planned_id]);
            Eigen::VectorXd arm_1_planned_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_1_path_[planned_id]);
            real_time_interface_->planning_visualizer_->publishDLOState(dlo_planned_state, Eigen::Vector3d(0, 0, 1));
            real_time_interface_->planning_visualizer_->publishPlanningScene(arm_0_planned_joint_pos, arm_1_planned_joint_pos);

            // record the data
            execution_res.waypoints_idx.push_back(waypoint_idx);
            // save the current state to the actual path
            execution_res.actual_path.dlo_path_.push_back(dlo_current_state);
            execution_res.actual_path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(arm_0_current_joint_pos));
            execution_res.actual_path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(arm_1_current_joint_pos));
            // reference path (next waypoint)
            execution_res.reference_path.dlo_path_.push_back(dlo_desired_state);
            execution_res.reference_path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(arm_0_desired_joint_pos));
            execution_res.reference_path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(arm_1_desired_joint_pos));

            // check collision between the current state and the environment
            bool dual_arm_collision = scene_->checkRobotCollision(arm_0_current_joint_pos, arm_1_current_joint_pos,
                                                                  /*min_dist_thres*/ 0.0, /*b_self_collision*/ false);
            bool dlo_collision = scene_->checkDLOAndWorldCollision(dlo_current_state, /*min_dist_thres*/ 0.0);
            if (dual_arm_collision || dlo_collision)
            {
                collision_time += 1.0 / control_rate_;
                ROS_WARN_STREAM("collision!!");
            }

            // closed-loop: if the velocity of DLO is too large, stop the robot, wait for DLO to be stable, and then quit and replanning
            if (mode == "closed_loop")
            {
                double dlo_current_fps_max_vel = mpc_count == 0 ? 0.0 : DLO::distanceBetweenTwoDLOStates(dlo_current_state, dlo_last_state, "max_distance") * control_rate_;
                dlo_last_state = dlo_current_state;
                if (sim_or_real_ == "sim" && dlo_current_fps_max_vel > dlo_fps_max_vel_)
                {
                    ROS_WARN_STREAM("dlo_current_fps_max_vel: " << dlo_current_fps_max_vel);
                    state = "dlo_unstable";
                    real_time_interface_->dualArmJointVelControl(Eigen::VectorXd::Zero(dual_arm_->joint_num_)); // stop the robot
                }
                if (state == "dlo_unstable")
                {
                    if (dlo_current_fps_max_vel > 0.01)
                    { // wait for DLO to be stable
                        state == "dlo_unstable";
                        rate.sleep(); // must have
                        continue;
                    }
                    else
                    { // quit, and replanning
                        if (scene_->checkDLOAndWorldCollision(dlo_current_state, /*min_dist_thres*/ mpc_min_dist_thres_))
                        {
                            state = "away_from_obstacles";
                        }
                        else
                        {
                            state = "replanning";
                            break;
                        }
                    }
                }
            }

            // check path collision
            if (mode == "open_loop_replanning")
            {
                if (checkTwoWaypointsPathCollision(dlo_current_state, arm_0_current_joint_pos, arm_1_current_joint_pos,
                                                   dlo_desired_state, arm_0_desired_joint_pos, arm_1_desired_joint_pos))
                {
                    ROS_WARN_STREAM("The path between the current state and planned waypoint is in collision !");
                    if (scene_->checkDLOAndWorldCollision(dlo_current_state, /*min_dist_thres*/ mpc_min_dist_thres_))
                    {
                        state = "away_from_obstacles";
                    }
                    else
                    {
                        state = "replanning";
                        break;
                    }
                }
            }

            // ----------------------- online updating DLO jacobian model -----------------------
            onlineUpdatingDLOJacobian();

            // ----------------------- MPC request -----------------------
            MPCRequest mpc_req;
            MPCResponse mpc_res;
            mpc_req.n_step = mpc_n_step_;
            mpc_req.delta_t = 1.0 / control_rate_;
            mpc_req.step_weight = std::vector<double>(mpc_req.n_step, 1.0);
            mpc_req.control_input_delay_time = control_input_delay_time_;
            mpc_req.arm_0_max_joint_vel = arm_0_max_joint_vel_;
            mpc_req.arm_1_max_joint_vel = arm_1_max_joint_vel_;
            mpc_req.arm_0_joint_pos_weights = arm_0_joint_pos_weights_;
            mpc_req.arm_1_joint_pos_weights = arm_1_joint_pos_weights_;
            mpc_req.arm_0_joint_vel_weights = arm_0_joint_vel_weights_;
            mpc_req.arm_1_joint_vel_weights = arm_1_joint_vel_weights_;
            mpc_req.b_warm_start = (mpc_count > 0);
            mpc_req.arm_0_last_joint_vel = (mpc_count == 0) ? Eigen::VectorXd::Zero(dual_arm_->arm_0_->joint_num_) : arm_0_last_joint_vel;
            mpc_req.arm_1_last_joint_vel = (mpc_count == 0) ? Eigen::VectorXd::Zero(dual_arm_->arm_1_->joint_num_) : arm_1_last_joint_vel;
            mpc_req.b_print_solving_log_level = 0;

            if (mode == "open_loop" || mode == "open_loop_replanning")
            {
                if (state == "away_from_obstacles")
                {
                    mpc_req.dlo_fps_pos_cost_weight = 0.0;
                    mpc_req.arm_joint_pos_cost_weight = 0.0;
                    mpc_req.arm_joint_vel_cost_weight = arm_joint_vel_cost_weight_;
                    mpc_req.arm_joint_acce_cost_weight = arm_joint_acce_cost_weight_;
                    mpc_req.min_dist_thres = away_from_obstacles_dist;
                    mpc_req.dlo_fps_max_vel = dlo_fps_max_vel_;
                    mpc_req.overstretch_thres = overstretch_thres_;
                }
                else
                {
                    mpc_req.dlo_fps_pos_cost_weight = 0.0;
                    mpc_req.arm_joint_pos_cost_weight = arm_joint_pos_cost_weight_;
                    mpc_req.arm_joint_vel_cost_weight = arm_joint_vel_cost_weight_;
                    mpc_req.arm_joint_acce_cost_weight = 0.0;
                    mpc_req.min_dist_thres = -1.0;     // no collision avoidance
                    mpc_req.dlo_fps_max_vel = 0.0;     // no contraint
                    mpc_req.overstretch_thres = -10.0; // no contraint
                }
            }
            else if (mode == "closed_loop")
            {
                if (state == "away_from_obstacles")
                {
                    mpc_req.dlo_fps_pos_cost_weight = 0.0;
                    mpc_req.arm_joint_pos_cost_weight = 0.0;
                    mpc_req.arm_joint_vel_cost_weight = arm_joint_vel_cost_weight_;
                    mpc_req.arm_joint_acce_cost_weight = arm_joint_acce_cost_weight_;
                    mpc_req.min_dist_thres = away_from_obstacles_dist;
                    mpc_req.dlo_fps_max_vel = 0.1 * dlo_fps_max_vel_;
                    mpc_req.arm_0_max_joint_vel = 0.1 * arm_0_max_joint_vel_;
                    mpc_req.arm_1_max_joint_vel = 0.1 * arm_1_max_joint_vel_;

                    mpc_req.overstretch_thres = overstretch_thres_;
                }
                else if (state == "tracking")
                {
                    mpc_req.dlo_fps_pos_cost_weight = dlo_fps_pos_cost_weight_;
                    mpc_req.arm_joint_pos_cost_weight = arm_joint_pos_cost_weight_;
                    mpc_req.arm_joint_vel_cost_weight = arm_joint_vel_cost_weight_;
                    mpc_req.arm_joint_acce_cost_weight = arm_joint_acce_cost_weight_;
                    mpc_req.min_dist_thres = mpc_min_dist_thres_;
                    mpc_req.dlo_fps_max_vel = dlo_fps_max_vel_;
                    mpc_req.overstretch_thres = overstretch_thres_;
                    if (waypoint_idx == path_length - 1)
                    { // the last waypoint does not consider the overstetch constraint
                        mpc_req.overstretch_thres = -10.0;
                    }
                }
            }

            // set desired path
            for (size_t t = 0; t < mpc_n_step_; t++)
            {
                int idx = std::min(waypoint_idx + t, path_length - 1); // pad the reference path with the goal configuration if the MPC horizon is longer than the remaining reference path
                mpc_req.dlo_desired_path.push_back(path.dlo_path_[idx]);
                mpc_req.arm_0_desired_path.push_back(Utils::stdVector2EigenVectorXd(path.arm_0_path_[idx]));
                mpc_req.arm_1_desired_path.push_back(Utils::stdVector2EigenVectorXd(path.arm_1_path_[idx]));
            }

            // start configuration
            mpc_req.dlo_init_state = real_time_interface_->getDLOState();
            mpc_req.arm_0_init_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
            mpc_req.arm_1_init_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);

            ROS_DEBUG_STREAM("Current joint pos: " << mpc_req.arm_0_init_joint_pos.transpose() << ", " << mpc_req.arm_1_init_joint_pos.transpose());
            ROS_DEBUG_STREAM("Next desired joint pos: " << mpc_req.arm_0_desired_path[0].transpose() << ", " << mpc_req.arm_1_desired_path[0].transpose());

            // ----------------------- MPC solving -----------------------

            t1 = std::chrono::steady_clock::now();
            try
            {
                mpc_->solve(scene_, mpc_req, mpc_res); // solving
            }
            catch (...)
            {
                ROS_ERROR_STREAM("MPC solving failed ! (segment error)");
                mpc_res.arm_0_joint_vel = Eigen::VectorXd::Zero(dual_arm_->arm_0_->joint_num_);
                mpc_res.arm_1_joint_vel = Eigen::VectorXd::Zero(dual_arm_->arm_1_->joint_num_);
            }
            t2 = std::chrono::steady_clock::now();
            time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            ROS_DEBUG_STREAM(std::setprecision(6) << "mpc time cost: " << time_used.count());
            mpc_total_time += time_used.count();
            mpc_count += 1;

            // hard-constraint of the commanded robot joint velocity, dealing with potential unconverged MPC solutions
            for (size_t j = 0; j < dual_arm_->arm_0_->joint_num_; j++)
            {
                mpc_res.arm_0_joint_vel(j) = std::max(-mpc_req.arm_0_max_joint_vel(j), std::min(mpc_res.arm_0_joint_vel(j), mpc_req.arm_0_max_joint_vel(j)));
            }
            for (size_t j = 0; j < dual_arm_->arm_1_->joint_num_; j++)
            {
                mpc_res.arm_1_joint_vel(j) = std::max(-mpc_req.arm_1_max_joint_vel(j), std::min(mpc_res.arm_1_joint_vel(j), mpc_req.arm_1_max_joint_vel(j)));
            }

            // ----------------------- publish the robot joint velocity command -----------------------
            Eigen::VectorXd dual_arm_joint_vel_command = Utils::concatenateTwoVector(mpc_res.arm_0_joint_vel, mpc_res.arm_1_joint_vel);
            real_time_interface_->dualArmJointVelControl(dual_arm_joint_vel_command);
            arm_0_last_joint_vel = mpc_res.arm_0_joint_vel;
            arm_1_last_joint_vel = mpc_res.arm_1_joint_vel;

            if (sim_or_real_ == "sim")
            {
                rate.sleep();
            }

            Eigen::VectorXd arm_0_rt_joint_vel, arm_1_rt_joint_vel;
            dual_arm_->splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(real_time_interface_->getRobotObservation().velocity), arm_0_rt_joint_vel, arm_1_rt_joint_vel);
            ROS_DEBUG_STREAM("arm_0_joint_vel: " << mpc_res.arm_0_joint_vel.transpose());
            ROS_DEBUG_STREAM("arm_0_rt_joint_vel: " << arm_0_rt_joint_vel.transpose());
            ROS_DEBUG_STREAM("arm_1_joint_vel: " << mpc_res.arm_1_joint_vel.transpose());
            ROS_DEBUG_STREAM("arm_1_rt_joint_vel: " << arm_1_rt_joint_vel.transpose());
            double arm_0_joint_vel_error = (arm_0_rt_joint_vel - mpc_res.arm_0_joint_vel).norm();
            double arm_1_joint_vel_error = (arm_1_rt_joint_vel - mpc_res.arm_1_joint_vel).norm();
            if (arm_0_joint_vel_error > 0.1)
                ROS_WARN_STREAM("arm_0_joint_vel_error is too big.");
            if (arm_1_joint_vel_error > 0.1)
                ROS_WARN_STREAM("arm_1_joint_vel_error is too big.");

            ROS_DEBUG_STREAM("Controller::trackingPath(): task error: " << DLO::distanceBetweenTwoDLOStates(real_time_interface_->getDLOState(), path.dlo_path_[waypoint_idx], "max_distance"));

            if (waypoint_idx == path_length - 1)
            {
                if (final_waypoint_time.isZero()) // record the timestamp of reaching the last waypoint
                    final_waypoint_time = ros::Time::now();
            }
            else
            {
                waypoint_idx++;
            }

            dlo_last_state = dlo_current_state;

            // ------------------ when to quit the loop ------------------------
            // if the controll input is too small, then quit tracking
            if (sim_or_real_ == "sim" && state == "tracking" && dual_arm_joint_vel_command.norm() < 1e-3)
            {
                if (mode == "closed_loop" || (mode == "open_loop" && waypoint_idx == path_length - 1))
                {
                    ROS_INFO("Controller::trackingPath(): converged.");
                    break;
                }
            }
            if (state == "away_from_obstacles")
            {
                DLOState dlo_current_state = real_time_interface_->getDLOState();
                Eigen::VectorXd arm_0_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
                Eigen::VectorXd arm_1_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);
                if (!scene_->checkDLOAndWorldCollision(dlo_current_state, away_from_obstacles_dist) &&
                    !scene_->checkRobotCollision(arm_0_current_joint_pos, arm_1_current_joint_pos, away_from_obstacles_dist, /*b_self_collision*/ false))
                {
                    ROS_INFO("Controller::trackingPath(): DLO is enough far away from obstacles.");
                    state = "replanning";
                    break;
                }
            }
            // if reach the maximum time allowed for the last waypoint, then quit tracking
            if (state == "tracking" && !final_waypoint_time.isZero() && (ros::Time::now() - final_waypoint_time).toSec() > max_time_for_final_waypoint_)
            {
                ROS_INFO_STREAM("Controller::trackingPath(): reached max_time_for_final_waypoint: " << max_time_for_final_waypoint_ << "s.");
                break;
            }
            if ((ros::Time::now() - start_time).toSec() > max_time_for_manipulation_)
            {
                ROS_INFO_STREAM("Controller::trackingPath(): reached max_time_for_manipulation_: " << max_time_for_manipulation_ << "s.");
                break;
            }
        }

        // stop the robot arms
        real_time_interface_->dualArmJointVelControl(Eigen::VectorXd::Zero(dual_arm_->joint_num_));

        execution_res.details["collision_time"] += collision_time;
        execution_res.details["mpc_count"] += mpc_count;
        execution_res.details["mpc_total_time"] += mpc_total_time;
        ROS_INFO_STREAM("Average MPC time cost: " << mpc_total_time / mpc_count);

        ROS_DEBUG_STREAM("Controller::trackingPath(): finish.");

        double final_error = DLO::distanceBetweenTwoDLOStates(real_time_interface_->getDLOState(), path.dlo_path_[path_length - 1], "L2_norm");
        return final_error;
    }

    // -------------------------------------------------------
    double Controller::trackingPathOpenLoop(
        const Path &path,
        const TrackingOptions &options,
        ExecutionResults &execution_res)
    {
        ros::Rate rate(control_rate_);
        size_t path_length = path.arm_0_path_.size();

        std::chrono::steady_clock::time_point t1, t2;
        std::chrono::duration<double> time_used;
        double collision_time = 0.0;
        ros::Time start_time = ros::Time::now();
        ros::Time final_waypoint_time;

        int waypoint_idx = 0;
        int current_dlo_obs_seq = 0;
        int current_robot_obs_seq = 0;

        // publish the open-loop robot trajectory command
        real_time_interface_->dualArmPathControl(path.arm_0_path_, path.arm_1_path_, /*time_between_points*/ 1.0 / control_rate_, /*b_block*/ false);

        while (ros::ok() && waypoint_idx < path_length)
        {
            if (sim_or_real_ == "real")
            {
                if (waypoint_idx == 0)
                {
                    real_time_interface_->waitForLatestObservation();
                }
                else
                {
                    real_time_interface_->waitForNextObservation(current_dlo_obs_seq, current_robot_obs_seq);
                }

                ROS_INFO_STREAM("Controller::trackingPath(): delay: " << (ros::Time::now() - real_time_interface_->getDLOobservation().header.stamp).toSec());
                ROS_INFO_STREAM("Controller::trackingPath(): current_dlo_obs_seq: " << current_dlo_obs_seq);
            }

            current_dlo_obs_seq = real_time_interface_->getDLOobservation().header.seq;
            current_robot_obs_seq = real_time_interface_->getRobotObservation().header.seq;
            DLOState dlo_current_state = real_time_interface_->getDLOState();
            Eigen::VectorXd arm_0_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
            Eigen::VectorXd arm_1_current_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);

            DLOState dlo_desired_state = path.dlo_path_[waypoint_idx];
            Eigen::VectorXd arm_0_desired_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_0_path_[waypoint_idx]);
            Eigen::VectorXd arm_1_desired_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_1_path_[waypoint_idx]);

            // visualize the planned waypoint
            int planned_id = waypoint_idx;
            DLOState dlo_planned_state = path.dlo_path_[planned_id];
            Eigen::VectorXd arm_0_planned_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_0_path_[planned_id]);
            Eigen::VectorXd arm_1_planned_joint_pos = Utils::stdVector2EigenVectorXd(path.arm_1_path_[planned_id]);
            real_time_interface_->planning_visualizer_->publishDLOState(dlo_planned_state, Eigen::Vector3d(0, 0, 1));
            real_time_interface_->planning_visualizer_->publishPlanningScene(arm_0_planned_joint_pos, arm_1_planned_joint_pos);

            // record data
            execution_res.waypoints_idx.push_back(waypoint_idx);
            // save the current state to the actual path
            execution_res.actual_path.dlo_path_.push_back(dlo_current_state);
            execution_res.actual_path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(arm_0_current_joint_pos));
            execution_res.actual_path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(arm_1_current_joint_pos));
            // reference path (next waypoint)
            execution_res.reference_path.dlo_path_.push_back(dlo_desired_state);
            execution_res.reference_path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(arm_0_desired_joint_pos));
            execution_res.reference_path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(arm_1_desired_joint_pos));

            // check collision between the current state and the environment
            bool dual_arm_collision = scene_->checkRobotCollision(arm_0_current_joint_pos, arm_1_current_joint_pos,
                                                                  /*min_dist_thres*/ 0.0, /*b_self_collision*/ false);
            bool dlo_collision = scene_->checkDLOAndWorldCollision(dlo_current_state, /*min_dist_thres*/ 0.0);
            if (dual_arm_collision || dlo_collision)
            {
                collision_time += 1.0 / control_rate_;
                ROS_WARN_STREAM("collision!!");
            }

            if (sim_or_real_ == "sim")
            {
                rate.sleep();
            }

            if (waypoint_idx == path_length - 1)
            {
                if (final_waypoint_time.isZero()) // record the timestamp of reaching the last waypoint
                    final_waypoint_time = ros::Time::now();
            }
            else
            {
                waypoint_idx++;
            }

            // ------------------ when to quit the loop ------------------------
            // if reach the maximum time allowed for the last waypoint, then quit tracking
            if (!final_waypoint_time.isZero() && (ros::Time::now() - final_waypoint_time).toSec() > max_time_for_final_waypoint_)
            {
                ROS_INFO_STREAM("Controller::trackingPath(): reached max_time_for_final_waypoint: " << max_time_for_final_waypoint_ << "s.");
                break;
            }
            if ((ros::Time::now() - start_time).toSec() > max_time_for_manipulation_)
            {
                ROS_INFO_STREAM("Controller::trackingPath(): reached max_time_for_manipulation_: " << max_time_for_manipulation_ << "s.");
                break;
            }
        }

        // stop robot arm
        real_time_interface_->dualArmJointVelControl(Eigen::VectorXd::Zero(dual_arm_->joint_num_));

        execution_res.details["collision_time"] += collision_time;
        execution_res.details["mpc_count"] += 1;
        execution_res.details["mpc_total_time"] += 0.0;

        ROS_DEBUG_STREAM("Controller::trackingPathOpenLoop(): finish.");

        double final_error = DLO::distanceBetweenTwoDLOStates(real_time_interface_->getDLOState(), path.dlo_path_[path_length - 1], "L2_norm");
        return final_error;
    }

    // -------------------------------------------------------
    void Controller::multiExecution()
    {
        std::vector<int> task_id_list(1);
        for (int i = 0; i < task_id_list.size(); i++)
            task_id_list[i] = i;

        std::vector<int> path_id_list(1);
        for (int i = 0; i < path_id_list.size(); i++)
            path_id_list[i] = i;

        std::vector<int> execution_id_list(1);
        for (int i = 0; i < execution_id_list.size(); i++)
            execution_id_list[i] = i;

        ExecutionOptions options;
        options.plan_to_start_config = false;

        for (int &task_id : task_id_list)
        {
            for (int &path_id : path_id_list)
            {
                for (int &execution_id : execution_id_list)
                {
                    if (execution_id == 0)
                    {
                        options.plan_to_goal_config = true;
                        options.save_path_to_goal_config = true;
                    }
                    else
                    {
                        options.plan_to_goal_config = false;
                        options.save_path_to_goal_config = false;
                    }

                    oneExecution(task_id, path_id, execution_id, options);

                    // reset Unity scene
                    real_time_interface_->resetUnityScene();
                }
            }
        }
    }

    // -------------------------------------------------------
    void Controller::loadDLOParams()
    {
        if (!Utils::ifFileExist(scene_dir_ + "dlo_identification/dlo_derm_params.yaml"))
        {
            ROS_WARN_STREAM("Controller: dlo_derm_params.yaml doesn't exist. Skip the loading.");
            return;
        }

        YAML::Node config = YAML::LoadFile(scene_dir_ + "dlo_identification/dlo_derm_params.yaml");

        DLOEnergyModelParams params;
        params.bend_stiffness = config["bend_stiffness"].as<double>();
        params.twist_stiffness = config["twist_stiffness"].as<double>();
        params.density = config["density"].as<double>();

        dlo_->setDermParams(params);
        ROS_INFO_STREAM("Controller: load DLO parameters.");
    }

    // -------------------------------------------------------
    void Controller::identifyDLOParams()
    {
        if (sim_or_real_ == "sim")
        {
            real_time_interface_->moveAndGraspDLO();
        }

        // --------------------- design two critical dlo states ---------------------------
        // get the first critical dlo_state for identification
        double x = 0.4;
        double z = -0.3;

        double dlo_length = dlo_->dlo_length_;

        Eigen::Vector3d first_fp, mid_fp, last_fp;
        first_fp = Eigen::Vector3d(x, -dlo_length / 3.0, z);
        mid_fp = Eigen::Vector3d(x + std::sqrt(3) / 3.0 * dlo_length, 0.0, z);
        last_fp = Eigen::Vector3d(x, dlo_length / 3.0, z);

        DLOState dlo_state_1 = dlo_->interpolateCoarseShape(first_fp, mid_fp, last_fp);
        dlo_state_1 = dlo_->optimizeShapeDerm(dlo_state_1);

        // get the second critial dlo_state for identification
        DLOState dlo_state_2 = dlo_state_1;
        Eigen::AngleAxisd y_rotation(-M_PI / 3.0, Eigen::Vector3d::UnitY());
        dlo_state_2.end_quat_0_ = (Eigen::Quaterniond(dlo_state_2.end_quat_0_).normalized() * y_rotation).coeffs();
        dlo_state_2.updateDependentInfo();
        dlo_state_2 = dlo_->optimizeShapeDerm(dlo_state_2);

        // --------------------- robot executes the path and records the observed dlo states ---------------------------
        std::vector<DLOState> recorded_dlo_states;
        ros::Rate rate(10);

        // plan and move to dlo_state_1
        PlanningRequest req;
        PlanningResponse res_1 = planPathToGoal(req, dlo_state_1);
        Path path;
        plannedPathToTrackedPath(res_1.smoothed_path_, 1.0 / control_rate_, path_dlo_max_vel_ * path_vel_scale_factor_,
                                 path_dlo_max_avel_ * path_vel_scale_factor_, arm_0_max_joint_vel_ * path_vel_scale_factor_,
                                 arm_1_max_joint_vel_ * path_vel_scale_factor_, path);
        real_time_interface_->dualArmPathControl(path.arm_0_path_, path.arm_1_path_, /*time_between_points*/ 1.0 / control_rate_, /*b_block*/ true);

        // plan and move to dlo_state_2
        real_time_interface_->dualArmMoveToTcpPoses(dlo_state_2.getLeftEndPose(), dlo_state_2.getRightEndPose(), /*b_block*/ false);

        // wait for reaching the dlo_state_2, during which recording the dlo trajectory
        while (ros::ok())
        {
            recorded_dlo_states.push_back(real_time_interface_->getDLOState());

            Eigen::Isometry3d current_arm_0_tcp_pose = dual_arm_->arm_0_->getTcpPose(
                real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_));
            Eigen::Isometry3d current_arm_1_tcp_pose = dual_arm_->arm_1_->getTcpPose(
                real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_));

            double error_thres = sim_or_real_ == "sim" ? 1e-3 : 5e-2;
            if (Utils::distanceBetweenTwoPose(current_arm_0_tcp_pose, dlo_state_2.getLeftEndPose()) < error_thres &&
                Utils::distanceBetweenTwoPose(current_arm_1_tcp_pose, dlo_state_2.getRightEndPose()) < error_thres)
            {
                break;
            }
            rate.sleep();
        }

        // save the observed dlo states
        CnpyUtils::saveDLOStates(scene_dir_ + "dlo_identification/", recorded_dlo_states);

        // --------------------- using PSO for identification ---------------------------
        // train set
        std::vector<DLOState> train_dlo_states;
        for (size_t i = 0; i < recorded_dlo_states.size() && ros::ok(); i++)
        {
            if (i % 10 == 0)
            {
                train_dlo_states.push_back(recorded_dlo_states[i]);
            }
        }
        train_dlo_states.push_back(recorded_dlo_states[recorded_dlo_states.size() - 1]);
        ROS_DEBUG_STREAM("Controller::identifyDLOParams(): trainset size: " << train_dlo_states.size());

        // PSO optimization
        OptimizeOptions optimize_options;
        optimize_options.function_tolerance = 1e-8;
        optimize_options.use_origin_edges_length = true;

        DLOEnergyModelParams params_opt;
        dlo_->optimizeParametersByPSO(train_dlo_states, params_opt);
        ROS_INFO_STREAM("Controller::identifyDLOParams(): opt params: twist_stiffness: " << params_opt.twist_stiffness
                                                                                         << ", density: " << params_opt.density);

        // save the params
        YAML::Node yaml_node;
        yaml_node["bend_stiffness"] = params_opt.bend_stiffness;
        yaml_node["twist_stiffness"] = params_opt.twist_stiffness;
        yaml_node["density"] = params_opt.density;
        Utils::createDirectory(scene_dir_ + "dlo_identification/");
        std::ofstream fout(scene_dir_ + "dlo_identification/dlo_derm_params.yaml");
        fout << yaml_node;

        // set to this DLO
        dlo_->setDermParams(params_opt);
    }

    // -------------------------------------------------------
    void Controller::savePlanningDetails(
        const std::string &dir,
        const PlanningResponse &res)
    {
        YAML::Node yaml_node;

        yaml_node["success"] = res.success_;

        if (res.success_)
        {
            auto it = res.details_.begin();
            while (it != res.details_.end())
            {
                yaml_node[it->first] = it->second;
                ++it;
            }
        }

        Utils::createDirectory(dir);
        std::ofstream fout(dir + "planning_details.yaml");
        fout << yaml_node;

        ROS_INFO_STREAM("savePlanningDetails(): done.");
    }

    // -------------------------------------------------------
    void Controller::saveExecutionResults(
        const std::string &dir,
        const ExecutionResults &res)
    {
        CnpyUtils::savePath(dir + "actual_path/", res.actual_path);
        CnpyUtils::savePath(dir + "reference_path/", res.reference_path);
        cnpy::npy_save(dir + "waypoints_idx.npy", &res.waypoints_idx[0], {static_cast<size_t>(res.waypoints_idx.size())}, "w");

        YAML::Node yaml_node;

        auto it = res.details.begin();
        while (it != res.details.end())
        {
            yaml_node[it->first] = it->second;
            ++it;
        }

        std::ofstream fout(dir + "execution_details.yaml");
        fout << yaml_node;

        ROS_INFO_STREAM("saveExecutionResults(): done.");
    }

    /** -------------------------------------------------------
     * @brief
     * press ctrl-Z to quit
     */
    void Controller::keyboardControl(
        const int task_id)
    {
        char ch;
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // -------------------------------------------
        std::string task_dir = scene_dir_ + "task_" + std::to_string(task_id) + "/";

        ROS_INFO("Controller::keyboardControl(): begin.");
        while (ros::ok())
        {
            if (read(STDIN_FILENO, &ch, 1) > 0)
            {
                std::cout << "input char: " << ch << " (Press 'ctrl + z' to quit)." << std::endl;
            }

            // if(ch == 'c')
            //     break;
            if (ch == 'p')
            { // save the current state
                DLOState current_dlo_state = real_time_interface_->getDLOState();
                Eigen::VectorXd current_arm_0_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
                Eigen::VectorXd current_arm_1_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);

                CnpyUtils::savePlanningGoal(task_dir, current_dlo_state, current_arm_0_joint_pos, current_arm_1_joint_pos);
                ROS_INFO("Controller::keyboardControl(): save current state.");
            }
            else
            { // robot end-effector keyboard control
                double speed = 0.1;
                double duration = 0.1;
                double control_rate = 30;
                Eigen::VectorXd arm_0_tcp_vel, arm_1_tcp_vel;
                keyToVelocity(ch, speed, arm_0_tcp_vel, arm_1_tcp_vel);
                ros::Rate rate(control_rate);
                for (size_t t = 0; t < control_rate * duration; t++)
                {
                    Eigen::VectorXd current_arm_0_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_0_->arm_group_name_);
                    Eigen::VectorXd current_arm_1_joint_pos = real_time_interface_->getRobotJointPos(dual_arm_->arm_1_->arm_group_name_);

                    Eigen::MatrixXd arm_0_jaco = dual_arm_->arm_0_->getTcpJacobianMatrix(current_arm_0_joint_pos);
                    Eigen::MatrixXd arm_1_jaco = dual_arm_->arm_1_->getTcpJacobianMatrix(current_arm_1_joint_pos);

                    Eigen::VectorXd arm_0_joint_vel = Utils::dampedLeastSquareInverse(arm_0_jaco, 0.01, 0.1) * arm_0_tcp_vel;
                    Eigen::VectorXd arm_1_joint_vel = Utils::dampedLeastSquareInverse(arm_1_jaco, 0.01, 0.1) * arm_1_tcp_vel;

                    real_time_interface_->dualArmJointVelControl(Utils::concatenateTwoVector(arm_0_joint_vel, arm_1_joint_vel));
                    rate.sleep();
                }
                real_time_interface_->dualArmJointVelControl(Eigen::VectorXd::Zero(dual_arm_->joint_num_));
            }
        }
    }

    // -------------------------------------------------------
    void Controller::keyToVelocity(
        char &key,
        double speed,
        Eigen::VectorXd &arm_0_tcp_vel,
        Eigen::VectorXd &arm_1_tcp_vel)
    {
        arm_0_tcp_vel = Eigen::VectorXd::Zero(6);
        arm_1_tcp_vel = Eigen::VectorXd::Zero(6);

        double angular_speed = 2 * M_PI * speed;

        if (key == 'w')
            arm_0_tcp_vel[0] = -speed;
        else if (key == 's')
            arm_0_tcp_vel[0] = speed;
        else if (key == 'a')
            arm_0_tcp_vel[1] = -speed;
        else if (key == 'd')
            arm_0_tcp_vel[1] = speed;
        else if (key == 'q')
            arm_0_tcp_vel[2] = -speed;
        else if (key == 'e')
            arm_0_tcp_vel[2] = speed;

        else if (key == 'W')
            arm_0_tcp_vel[3] = -angular_speed;
        else if (key == 'S')
            arm_0_tcp_vel[3] = angular_speed;
        else if (key == 'A')
            arm_0_tcp_vel[4] = -angular_speed;
        else if (key == 'D')
            arm_0_tcp_vel[4] = angular_speed;
        else if (key == 'Q')
            arm_0_tcp_vel[5] = -angular_speed;
        else if (key == 'E')
            arm_0_tcp_vel[5] = angular_speed;

        else if (key == 'i')
            arm_1_tcp_vel[0] = -speed;
        else if (key == 'k')
            arm_1_tcp_vel[0] = speed;
        else if (key == 'j')
            arm_1_tcp_vel[1] = -speed;
        else if (key == 'l')
            arm_1_tcp_vel[1] = speed;
        else if (key == 'u')
            arm_1_tcp_vel[2] = -speed;
        else if (key == 'o')
            arm_1_tcp_vel[2] = speed;

        else if (key == 'I')
            arm_1_tcp_vel[3] = -angular_speed;
        else if (key == 'K')
            arm_1_tcp_vel[3] = angular_speed;
        else if (key == 'J')
            arm_1_tcp_vel[4] = -angular_speed;
        else if (key == 'L')
            arm_1_tcp_vel[4] = angular_speed;
        else if (key == 'U')
            arm_1_tcp_vel[5] = -angular_speed;
        else if (key == 'O')
            arm_1_tcp_vel[5] = angular_speed;
    }

    // -------------------------------------------------------
    void Controller::plannedPathToTrackedPath(
        const Path &planned_path,
        const double delta_t,
        const double dlo_max_vel,
        const double dlo_max_avel,
        const Eigen::VectorXd &arm_0_max_joint_vel,
        const Eigen::VectorXd &arm_1_max_joint_vel,
        Path &new_path)
    {
        new_path.clear();
        int planned_path_length = planned_path.dlo_path_.size();

        double dlo_max_step = dlo_max_vel * delta_t;
        double dlo_max_rot_step = dlo_max_avel * delta_t;
        Eigen::VectorXd arm_0_max_step = arm_0_max_joint_vel * delta_t;
        Eigen::VectorXd arm_1_max_step = arm_1_max_joint_vel * delta_t;

        for (size_t i = 0; ros::ok() && i < planned_path_length - 1; i++)
        {
            auto &dlo_state_from = planned_path.dlo_path_[i];
            auto arm_0_joint_pos_from = Utils::stdVector2EigenVectorXd(planned_path.arm_0_path_[i]);
            auto arm_1_joint_pos_from = Utils::stdVector2EigenVectorXd(planned_path.arm_1_path_[i]);

            auto &dlo_state_to = planned_path.dlo_path_[i + 1];
            auto arm_0_joint_pos_to = Utils::stdVector2EigenVectorXd(planned_path.arm_0_path_[i + 1]);
            auto arm_1_joint_pos_to = Utils::stdVector2EigenVectorXd(planned_path.arm_1_path_[i + 1]);

            if ((arm_0_joint_pos_to - arm_0_joint_pos_from).norm() < 1e-3 // 去除相邻的关节角一样的waypoint
                && (arm_1_joint_pos_to - arm_1_joint_pos_from).norm() < 1e-3)
            {
                continue;
            }

            double t_step_dlo = dlo_->calcInterpolationRatio(dlo_state_from, dlo_state_to,
                                                             dlo_max_step, dlo_max_rot_step);
            double t_step_arm_0 = 1.0;
            for (size_t j = 0; j < dual_arm_->arm_0_->joint_num_; j++)
            {
                double t = arm_0_max_step(j) / std::abs(arm_0_joint_pos_from(j) - arm_0_joint_pos_to(j));
                t_step_arm_0 = std::min(t, t_step_arm_0);
            }
            double t_step_arm_1 = 1.0;
            for (size_t j = 0; j < dual_arm_->arm_1_->joint_num_; j++)
            {
                double t = arm_1_max_step(j) / std::abs(arm_1_joint_pos_from(j) - arm_1_joint_pos_to(j));
                t_step_arm_1 = std::min(t, t_step_arm_1);
            }

            double t_step = std::min(t_step_dlo, std::min(t_step_arm_0, t_step_arm_1));
            t_step = 1.0 / ceil(1.0 / t_step); // 让运动平均分布，防止出现最后一个waypoint的运动特别小

            for (double t = 0.0; t < 1.0 - 1e-5 /*防止数值误差影响*/; t += t_step)
            {
                DLOState new_dlo_state = dlo_->interpolate(dlo_state_from, dlo_state_to, t);
                Eigen::VectorXd new_arm_0_joint_pos = arm_0_joint_pos_from * (1 - t) + arm_0_joint_pos_to * t;
                Eigen::VectorXd new_arm_1_joint_pos = arm_1_joint_pos_from * (1 - t) + arm_1_joint_pos_to * t;

                new_path.dlo_path_.push_back(new_dlo_state);
                new_path.arm_0_path_.push_back(Utils::eigenVectorXd2StdVector(new_arm_0_joint_pos));
                new_path.arm_1_path_.push_back(Utils::eigenVectorXd2StdVector(new_arm_1_joint_pos));
            }
        }

        // mannually add the last path point
        new_path.dlo_path_.push_back(planned_path.dlo_path_[planned_path_length - 1]);
        new_path.arm_0_path_.push_back(planned_path.arm_0_path_[planned_path_length - 1]);
        new_path.arm_1_path_.push_back(planned_path.arm_1_path_[planned_path_length - 1]);
    }

    // -------------------------------------------------------
    bool Controller::checkTwoWaypointsPathCollision(
        const DLOState &from_dlo_state,
        const Eigen::VectorXd &from_arm_0_joint_pos,
        const Eigen::VectorXd &from_arm_1_joint_pos,
        const DLOState &to_dlo_state,
        const Eigen::VectorXd &to_arm_0_joint_pos,
        const Eigen::VectorXd &to_arm_1_joint_pos)
    {
        // compute the interpolation ratio t in [0, 1]
        double max_dist = 0.0;
        for (size_t k = 0; k < dlo_->num_fps_; k++)
        {
            double dist = (from_dlo_state.getFpPos(k) - to_dlo_state.getFpPos(k)).norm();
            max_dist = std::max(max_dist, dist);
        }
        double path_collision_check_cartesian_step_size = 0.01;
        double t_step = std::min(1.0, path_collision_check_cartesian_step_size / max_dist);

        // linear interpolation between from_node and to_node (not including from_node and to_node）
        for (double t = t_step; t < 1.0; t += t_step)
        {
            Eigen::VectorXd fps_pos = from_dlo_state.fps_pos_ * (1 - t) + to_dlo_state.fps_pos_ * t;
            Eigen::VectorXd arm_0_joint_pos = from_arm_0_joint_pos * (1 - t) + to_arm_0_joint_pos * t;
            Eigen::VectorXd arm_1_joint_pos = from_arm_1_joint_pos * (1 - t) + to_arm_1_joint_pos * t;

            if (scene_->checkRobotCollision(arm_0_joint_pos, arm_1_joint_pos, /*min_dist_thres*/ 0.0))
            {
                ROS_DEBUG_STREAM("checkNodeCollision(): dual_arm is in collision.");
                return true;
            }
            if (scene_->checkDLOAndWorldCollision(DLOState(fps_pos), /*min_dist_thres*/ -0.005))
            {
                std::cout << "scene_->minDistBetweenDLOAndWorld(fps_pos): " << scene_->minDistBetweenDLOAndWorld(fps_pos) << std::endl;
                ROS_DEBUG_STREAM("checkNodeCollision(): DLO is in collision with the world.");
                return true;
            }
        }

        return false;
        // true: in collision; false: no collision
    }

} // end namespace