#ifndef DLO_ARM_PLANNING_SCENE_H
#define DLO_ARM_PLANNING_SCENE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/collision_detection_fcl.h"

namespace dlo_arm_planning_pkg
{

    /**
     * @brief Class for the planning scene
     * including the robot and the environment
     * mainly for collision checking
     */

    class Scene
    {
    public:
        typedef std::shared_ptr<Scene> Ptr;

        Scene(
            const ros::NodeHandle &nh,
            const DLO::Ptr &dlo,
            const DualArm::Ptr &dual_arm);

        Scene(
            const ros::NodeHandle &nh,
            const DLO::Ptr &dlo,
            const DualArm::Ptr &dual_arm,
            const planning_scene::PlanningSceneConstPtr &planning_scene);

        void setPlanningScene(
            const planning_scene::PlanningSceneConstPtr &planning_scene);

        void closeGrippers();

        void getPlanningSceneMsg(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos,
            moveit_msgs::PlanningScene &planning_scene_msg);

        bool checkRobotCollision(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos,
            double min_dist_thres = 0.0,
            bool b_self_collision = true);

        bool checkRobotCollisionByMoveit(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos);

        bool checkPointsAndWorldCollision(
            const VecEigenVec3 points,
            double min_dist_thres = 0.0);

        bool checkDLOAndWorldCollision(
            const DLOState &dlo_state,
            double min_dist_thres = 0.0);

        double getPointDist(
            const Eigen::Vector3d &point);

        Eigen::Vector3d getPointDistGradient(
            const Eigen::Vector3d &point);

        double getPrecisePointDist(
            const Eigen::Vector3d &point);

        void minDistBetweenPointsAndWorld(
            const VecEigenVec3 points,
            double &min_dist,
            Eigen::Vector3d &min_point_pos,
            int &min_point_index);

        double minDistBetweenDLOAndWorld(
            const DLOState &dlo_state);

        double minDistBetweenDLOAndWorld(
            const Eigen::VectorXd &fps_pos);

        double minDistBetweenRobotAndWorld(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos);

    public:
        ros::NodeHandle nh_;

        DLO::Ptr dlo_;
        DualArm::Ptr dual_arm_;
        CollisionDetectionFCL::Ptr cd_fcl_;

        planning_scene::PlanningScenePtr planning_scene_;
        const collision_detection::CollisionEnvHybrid *collision_env_hybrid_;
        distance_field::DistanceFieldConstPtr distance_field_;

        double dlo_edge_resolution_ = 0.002;

    }; // end class

} // end namespace

#endif