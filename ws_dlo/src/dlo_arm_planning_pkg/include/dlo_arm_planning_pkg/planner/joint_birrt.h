#ifndef DLO_ARM_PLANNING_JOINTBIRRT_H
#define DUAL_ARM_PLANNING_JOINTBIRRT_H

#include "dlo_arm_planning_pkg/planner/planner_base.h"

namespace dlo_arm_planning_pkg
{

    class JointBiRRT : public PlannerBase
    {
    public:
        typedef std::shared_ptr<JointBiRRT> Ptr;

        JointBiRRT(
            const ros::NodeHandle &nh,
            const Scene::Ptr &scene);

        void loadParams();

        Node::Ptr randomSampleNodeDloOnly();

        Node::Ptr randomSampleNode();

        bool checkTwoNodeCanConnect(
            const Node::Ptr &from_node,
            const Node::Ptr &to_node,
            const double dlo_max_dist,
            const double arm_max_dist);

        Node::Ptr oneStepSteer(
            const Node::Ptr &from_node,
            const Node::Ptr &to_node);

        Node::Ptr extend(
            std::vector<Node::Ptr> &node_list,
            const Node::Ptr &from_node,
            const Node::Ptr &to_node,
            bool greedy = false);

        bool solve(
            const PlanningRequest &req,
            PlanningResponse &res);

    public:
        std::string ALGORITHM_NAME = "JointBiRRT";

        // parameters
        double sample_only_dlo_probability_;

    }; // end class

} // end namespace

#endif