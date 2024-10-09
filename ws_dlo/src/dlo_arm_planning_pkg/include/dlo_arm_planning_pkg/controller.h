#ifndef DLO_ARM_PLANNING_CONTROLLER_H
#define DLO_ARM_PLANNING_CONTROLLER_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/real_time_interface.h"
#include "dlo_arm_planning_pkg/planning_interface.h"
#include "dlo_arm_planning_pkg/visualize.h"
#include "dlo_arm_planning_pkg/jacobian_model.h"

#include "dlo_arm_planning_pkg/mpc/multi_step_mpc.h"

#include <moveit_msgs/RobotTrajectory.h>
#include <my_msgs/DLOStateStamped.h>
#include <my_msgs/VectorStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

namespace dlo_arm_planning_pkg
{

    // --------------------------------------------------------
    struct TrackingOptions
    {
        std::string mode = "closed_loop";
        // bool b_open_loop = false;
    };

    // --------------------------------------------------------
    struct ExecutionOptions
    {
        bool plan_to_start_config = true; // true: plan the path; false: load the planned path.
        bool save_path_to_start_config = false;

        bool plan_to_goal_config = true; // true: plan the path; false: load the planned path.
        bool save_path_to_goal_config = false;

        bool execute = true;
        std::string execute_path_type = "smoothed"; // "smoothed" or "interpolated"
        bool open_loop = false;

        std::string planner_name = "ours";
        std::string execution_name = "test";
        bool b_save_execution_results = true;

        TrackingOptions initial2start_tracking_options;
        TrackingOptions tracking_options;
    };

    // --------------------------------------------------------
    struct ExecutionResults
    {
        Path reference_path;
        Path actual_path;
        std::vector<int> waypoints_idx;

        std::map<std::string, double> details;

        void clear()
        {
            waypoints_idx.clear();
            reference_path.clear();
            actual_path.clear();
            details.clear();
        }
    };

    // --------------------------------------------------------
    class Controller
    {
    public:
        typedef std::shared_ptr<Controller> Ptr;

        Controller(
            const ros::NodeHandle &nh);

        void loadParams();

        void loadPlanningGoal(
            const std::string &dir,
            DLOState &dlo_state,
            Eigen::VectorXd &arm_0_joint_pos,
            Eigen::VectorXd &arm_1_joint_pos);

        void saveDLOStates(
            const std::string &dir,
            const std::vector<DLOState> &dlo_states);

        PlanningResponse planPathToGoal(
            PlanningRequest &req,
            const DLOState &goal_dlo_state,
            const Eigen::VectorXd arm_0_goal_joint_pos = Eigen::VectorXd::Zero(0),
            const Eigen::VectorXd arm_1_goal_joint_pos = Eigen::VectorXd::Zero(0));

        double trackingPath(
            const Path &path,
            const TrackingOptions &options,
            ExecutionResults &execution_res);

        double trackingPathOpenLoop(
            const Path &path,
            const TrackingOptions &options,
            ExecutionResults &execution_res);

        bool moveToGoal(
            const Path &planned_path,
            const TrackingOptions &tracking_options,
            ExecutionResults &execution_res);

        void identifyDLOParams();

        void loadDLOParams();

        void onlineUpdatingDLOJacobian(
            const bool restart = false);

        bool oneExecution(
            const int task_id,
            const int path_id,
            const int execution_id,
            const ExecutionOptions &options);

        void multiExecution();

        void savePlanningDetails(
            const std::string &dir,
            const PlanningResponse &res);

        void saveExecutionResults(
            const std::string &dir,
            const ExecutionResults &res);

        void keyboardControl(
            const int task_id = 0);

        void keyToVelocity(
            char &key,
            double speed,
            Eigen::VectorXd &arm_0_tcp_vel,
            Eigen::VectorXd &arm_1_tcp_vel);

        void plannedPathToTrackedPath(
            const Path &planned_path,
            const double delta_t,
            const double dlo_max_vel,
            const double dlo_max_avel,
            const Eigen::VectorXd &arm_0_max_joint_vel,
            const Eigen::VectorXd &arm_1_max_joint_vel,
            Path &new_path);

        bool checkTwoWaypointsPathCollision(
            const DLOState &from_dlo_state,
            const Eigen::VectorXd &from_arm_0_joint_pos,
            const Eigen::VectorXd &from_arm_1_joint_pos,
            const DLOState &to_dlo_state,
            const Eigen::VectorXd &to_arm_0_joint_pos,
            const Eigen::VectorXd &to_arm_1_joint_pos);

    public:
        ros::NodeHandle nh_;

        // loaded ROS params
        std::string project_dir_;
        std::string sim_or_real_;
        std::string env_dimension_;
        int scene_id_;
        int dlo_id_;

        // load dlo jacobian params
        std::string dlo_jaco_offline_model_dir_;
        std::string dlo_jaco_offline_model_name_;

        // loaded controller params
        double control_rate_;
        Eigen::VectorXd arm_0_max_joint_vel_, arm_1_max_joint_vel_;
        Eigen::VectorXd arm_0_joint_pos_weights_, arm_1_joint_pos_weights_;
        Eigen::VectorXd arm_0_joint_vel_weights_, arm_1_joint_vel_weights_;
        double dlo_fps_max_vel_;
        int mpc_n_step_;
        double mpc_min_dist_thres_;
        double control_input_delay_time_;
        double overstretch_thres_;
        // mpc cost weights
        double dlo_fps_pos_cost_weight_;
        double arm_joint_pos_cost_weight_;
        double arm_joint_vel_cost_weight_;
        double arm_joint_acce_cost_weight_;

        double max_time_for_final_waypoint_;
        double max_time_for_manipulation_;
        double path_vel_scale_factor_;
        double path_dlo_max_vel_, path_dlo_max_avel_;
        double success_error_thres_;
        double path_collision_check_cartesian_step_size_;

        std::string scene_dir_;

        DLO::Ptr dlo_;
        DualArm::Ptr dual_arm_;
        Scene::Ptr scene_;

        RealTimeInterface::Ptr real_time_interface_;
        PlanningInterface::Ptr planning_interface_;
        MPCBase::Ptr mpc_;
        JacobianModel::Ptr dlo_jaco_model_;

    }; // end class

} // end namespace

#endif