#ifndef DLO_ARM_PLANNING_NODE_H
#define DLO_ARM_PLANNING_NODE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"


namespace dlo_arm_planning_pkg{


class Node{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Node> Ptr;

    Node(){}

    Node(
        const DLOState &dlo_state,
        const Eigen::VectorXd &arm_0_joint_pos = Eigen::VectorXd::Zero(0),
        const Eigen::VectorXd &arm_1_joint_pos = Eigen::VectorXd::Zero(0),
        const Node::Ptr parent = nullptr,
        const std::string note = std::string(""),
        const double cost_from_init = -1.0
    ){
        dlo_state_ = dlo_state;
        arm_0_joint_pos_ = arm_0_joint_pos;
        arm_1_joint_pos_ = arm_1_joint_pos;
        parent_ = nullptr;
        note_ = note;
        cost_from_init_ = cost_from_init;
    }


public:
    DLOState dlo_state_;
    Eigen::VectorXd arm_0_joint_pos_, arm_1_joint_pos_;
    Ptr parent_ = nullptr;
    std::string note_ = ""; 

    // dependent dual arm state
    Eigen::VectorXd dual_arm_joint_pos_;
    Eigen::Isometry3d arm_0_tcp_pose_, arm_1_tcp_pose_;
    VecEigenVec3 arm_0_links_pos_, arm_1_links_pos_;

    // for optimal planning
    double cost_from_init_;
};


} // namespace

#endif