#ifndef DLO_ARM_PLANNING_COLLISION_DETECTION_FCL_H
#define DLO_ARM_PLANNING_COLLISION_DETECTION_FCL_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dual_arm.h"

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/config.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/broadphase/detail/sparse_hash_table.h"
#include "fcl/broadphase/detail/spatial_hash.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"


namespace dlo_arm_planning_pkg{


struct DLOEdgeGeometryData
{
    int edge_index;
    bool b_left_end{false};
    bool b_right_end{false};
};

struct RobotLinkGeometryData
{
    std::string group_name;
    std::string link_name;
    int link_index;

    bool b_left_end{false};
    bool b_right_end{false};

    Eigen::Isometry3d relative_pose;
};

struct CollisionObjectUserData
{
    std::string geometry_type; // capsule
    std::vector<double> geometry_params;

    std::string object_type;
    DLOEdgeGeometryData dlo_data;
    RobotLinkGeometryData robot_data;
};


// -------------------------------------------------------
class CollisionDetectionFCL
{
public:
    typedef std::shared_ptr<CollisionDetectionFCL> Ptr;

    CollisionDetectionFCL(
        const ros::NodeHandle& nh,
        const DLO::Ptr &dlo,
        const DualArm::Ptr &dual_arm
    );

    void initializeDLOManager();

    void loadRobotCoarseCollisionShape();

    void initializeRobotManager();

    void setDloObjectsTransform(
        const DLOState &dlo_state
    );

    void setRobotObjectsTransform(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos
    );

    Eigen::VectorXd getCriticalPointsPosVec(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos
    );

    void getInfoForVisualizeDLO(
        VecEigenIsometry3d &poses,
        double &capsule_length,
        double &capsule_radius
    );

    void getInfoForVisualizeRobot(
        VecEigenIsometry3d &poses,
        std::vector<double> &capsule_lengths,
        std::vector<double> &capsule_radiuses
    );

    void getInfoForVisualizeRobot(
        VecEigenIsometry3d &poses,
        std::vector<std::string> &geometry_types,
        std::vector<std::vector<double> > &geometry_params
    );

    bool checkDLOSelfCollision();

    double calcDLOSelfDistance();

    bool checkRobotSelfCollision();

    bool checkDLOAndRobotCollision();



public:
    ros::NodeHandle nh_;

    DLO::Ptr dlo_;
    DualArm::Ptr dual_arm_;

    // for DLO
    fcl::BroadPhaseCollisionManagerd* manager_dlo_;
    std::vector<CollisionObjectUserData> objects_dlo_userdata_;
    std::vector<fcl::CollisionObjectd> objects_dlo_;
    double dlo_capsule_radius_ = 0.01;

    // for dual arm
    fcl::BroadPhaseCollisionManagerd* manager_dual_arm_;
    std::vector<CollisionObjectUserData> objects_dual_arm_userdata_;
    std::vector<fcl::CollisionObjectd> objects_dual_arm_;
};

} // namespace

#endif