#ifndef DLO_ARM_PLANNING_REAL_TIME_INTERFACE_H
#define DLO_ARM_PLANNING_REAL_TIME_INTERFACE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/planning_interface.h"
#include "dlo_arm_planning_pkg/visualize.h"

#include <moveit_msgs/RobotTrajectory.h>
#include <my_msgs/DLOStateStamped.h>
#include <my_msgs/VectorStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

namespace dlo_arm_planning_pkg
{

    class RealTimeInterface
    {
    public:
        typedef std::shared_ptr<RealTimeInterface> Ptr;

        RealTimeInterface(
            const ros::NodeHandle &nh,
            const DLO::Ptr &dlo,
            const DualArm::Ptr &dual_arm);

        ~RealTimeInterface();

        void loadParams();

        bool waitUntilReceiveAllMessages();

        void robotObservationCb(
            const sensor_msgs::JointState &msg);

        void dloObservationCb(
            const my_msgs::DLOStateStamped &msg);

        my_msgs::DLOStateStamped getDLOobservation();

        sensor_msgs::JointState getRobotObservation();

        planning_scene::PlanningSceneConstPtr getLockedPlanningScene();

        void clearMoveGroupPlanningScene();

        DLOState getDLOState();

        Eigen::VectorXd getRobotJointPos(
            const std::string &group_name);

        bool planArmTrajectoryToTargetJointPosByMoveGroup(
            const std::string &group_name,
            const Eigen::VectorXd &start_joint_pos,
            const Eigen::VectorXd &target_joint_pos,
            moveit_msgs::RobotTrajectory &res_trajectory);

        moveit_msgs::RobotTrajectory planArmTrajectoryToTargetPoseByMoveGroup(
            const std::string &group_name,
            const Eigen::Isometry3d &target_pose,
            const Eigen::VectorXd &start_joint_pos);

        bool waitUntilArmReachTargetJointPos(
            const std::string &group_name,
            const Eigen::VectorXd &target_joint_pos,
            const double error_thres = 1e-3);

        bool graspDLOEnds();

        bool resetUnityScene();

        bool moveAndGraspDLO();

        void dualArmJointVelControl(
            const Eigen::VectorXd &dual_arm_joint_vel);

        DLOState DLOMsgToDLOState(
            const my_msgs::DLOStateStamped &msg);

        moveit_msgs::RobotTrajectory armJointPathToTrajectory(
            const std::string &group_name,
            const std::vector<std::vector<double>> &path,
            const double time_between_points);

        void armTrajectoryControl(
            const std::string &group_name,
            const moveit_msgs::RobotTrajectory &trajectory,
            const bool b_block = true);

        void armPathControl(
            const std::string &group_name,
            const std::vector<std::vector<double>> path,
            const double time_between_points = 0.5,
            const bool b_block = true);

        void dualArmPathControl(
            const std::vector<std::vector<double>> arm_0_path,
            const std::vector<std::vector<double>> arm_1_path,
            const double time_between_points,
            const bool b_block);

        bool dualArmMoveToTcpPoses(
            const Eigen::Isometry3d &end_0_pose,
            const Eigen::Isometry3d &end_1_pose,
            bool b_block = true);

        void sendVideoRecordCommand(
            const std::string &command);

        void waitForLatestObservation();

        void waitForNextObservation(
            const int dlo_last_seq,
            const int robot_last_seq);

        void publishGoalDLOState(
            const DLOState &dlo_state);

    public:
        // parameters
        std::string sim_or_real_;

        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        DLO::Ptr dlo_;
        DualArm::Ptr dual_arm_;
        Visualize::Ptr planning_visualizer_, rt_visualizer_;

        ros::NodeHandle nh_;
        planning_scene_monitor::PlanningSceneMonitorPtr psm_;

        ros::Subscriber robot_obs_sub_;
        ros::Subscriber dlo_obs_sub_;
        ros::ServiceClient grasp_dlo_ends_client_;
        ros::ServiceClient unity_reset_scene_client_;
        ros::ServiceClient clear_octomap_client_;

        ros::Publisher arm_0_joint_traj_pub_, arm_1_joint_traj_pub_, dual_arm_joint_traj_pub_;
        ros::Publisher dual_arm_joint_vel_pub_;
        ros::Publisher video_record_command_pub_;
        ros::Publisher dlo_goal_pub_;

        sensor_msgs::JointState robot_rt_obs_;
        bool b_received_robot_rt_obs_ = false;
        my_msgs::DLOStateStamped dlo_rt_obs_;
        bool b_received_dlo_rt_obs_ = false;
        ros::Time get_latest_observation_time_;

    }; // end class

} // end namespace

#endif