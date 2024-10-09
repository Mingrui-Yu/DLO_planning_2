#ifndef DLO_ARM_PLANNING_PLANNER_INTERFACE_H
#define DLO_ARM_PLANNING_PLANNER_INTERFACE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/visualize.h"
#include "dlo_arm_planning_pkg/planner/planner_base.h"




namespace dlo_arm_planning_pkg{


class PlanningInterface
{
public:
    typedef std::shared_ptr<PlanningInterface> Ptr;

    PlanningInterface(
        const ros::NodeHandle& nh
    );

    PlanningInterface(
        const ros::NodeHandle& nh,
        const std::string &robot_description_name
    );

    bool solve(
        std::string algorithm,
        const PlanningRequest &req,
        PlanningResponse &res
    );

    bool solve(
        const Scene::Ptr &scene,
        const std::string algorithm,
        const PlanningRequest &req,
        PlanningResponse &res,
        Visualize::Ptr visualizer = nullptr
    );




public:

    ros::NodeHandle nh_;
    std::string robot_description_name_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

    DLO::Ptr dlo_;
    DualArm::Ptr dual_arm_;
    Scene::Ptr scene_;
    Visualize::Ptr visualizer_;

    // parameters
    int dim_;
    int num_fps_;

}; // end class





} // end namespace

#endif