#ifndef DLO_ARM_PLANNING_VISUALIZE_H
#define DLO_ARM_PLANNING_VISUALIZE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/node.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/dlo_state.h"

namespace dlo_arm_planning_pkg
{

    /**
     * @brief Class for visualization: publishing messages to rviz for visualization
     */

    class Visualize
    {
    public:
        typedef std::shared_ptr<Visualize> Ptr;

        Visualize(
            const ros::NodeHandle &nh,
            const Scene::Ptr &scene,
            const std::string topic_prefix = "/planning");

        Visualize(
            const ros::NodeHandle &nh,
            const DualArm::Ptr &dual_arm,
            const std::string topic_prefix);

        void setScene(const Scene::Ptr &scene);

        void loadParams();

        void initiate();

        void publishRobotState(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos);

        void publishRobotState(
            const std::vector<double> &arm_0_joint_pos,
            const std::vector<double> &arm_1_joint_pos);

        void publishPlanningScene(
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos);

        void publishText(
            const std::string &text);

        void publishDLOState(
            const DLOState &dlo_state,
            const Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0) // green
        );

        bool publishAxis(
            const Eigen::Isometry3d &pose,
            double length,
            double radius,
            const Eigen::Vector3d color);

        bool publishAxis2(const Eigen::Isometry3d &pose,
                          const Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0));

        void publishNode(
            const Node::Ptr &node,
            const std::string text = std::string(""));

        void publishNodePath(
            const std::vector<Node::Ptr> &path_list,
            double ros_rate = 2.0);

        void publishDLOCollisionShape(
            const VecEigenIsometry3d &poses,
            const double &capsule_length,
            const double &capsule_radius);

        void publishRobotCollisionShape(
            const VecEigenIsometry3d &poses,
            const std::vector<double> &capsule_lengths,
            const std::vector<double> &capsule_radiuses);

        void publishRobotCollisionShape(
            const VecEigenIsometry3d &poses,
            const std::vector<std::string> &geometry_types,
            const std::vector<std::vector<double>> &geometry_params);

        std::vector<visualization_msgs::Marker> getMarkerForShapeVis(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &color);

        std::vector<visualization_msgs::Marker> getCapsuleMarkersForDLO(
            const VecEigenIsometry3d &poses,
            const double &capsule_length,
            const double &capsule_radius);

        std::vector<visualization_msgs::Marker> getCapsuleMarkers(
            const VecEigenIsometry3d &poses,
            const std::vector<double> &capsule_lengths,
            const std::vector<double> &capsule_radiuses,
            const Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 1.0));

        std::vector<visualization_msgs::Marker> getSphereMarkers(
            const VecEigenIsometry3d &poses,
            const std::vector<double> &radiuses,
            const Eigen::Vector3d color);

        visualization_msgs::Marker getSphereMarker(
            const Eigen::Isometry3d &pose,
            const double &radius,
            const int id = 0,
            const Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 1.0));

        std::vector<visualization_msgs::Marker> getCapsuleMarker(
            const Eigen::Isometry3d &pose,
            const double &radius,
            const double &length,
            const int id = 0,
            const Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 1.0));

        void publishPoints(
            const Eigen::VectorXd &points,
            const Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0) // green
        );

    public: // protected
        ros::NodeHandle nh_;
        std::string topic_prefix_;
        Scene::Ptr scene_;
        DualArm::Ptr dual_arm_;

        std::string arm_base_link_name_;

        ros::Publisher planning_scene_pub_;
        ros::Publisher dlo_state_pub_;

        ros::Publisher dlo_collision_shape_pub_;
        ros::Publisher robot_collision_shape_pub_;

        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    }; // end class

} // end namespace

#endif