#include "dlo_arm_planning_pkg/planning_interface.h"
#include "dlo_arm_planning_pkg/planner/joint_birrt.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    PlanningInterface::PlanningInterface(
        const ros::NodeHandle &nh,
        const std::string &robot_description_name) : nh_(nh)
    {
        robot_description_name_ = robot_description_name;

        robot_model_loader::RobotModelLoaderPtr robot_model_loader(
            new robot_model_loader::RobotModelLoader(robot_description_name_));

        // planning scene monitor
        psm_ = planning_scene_monitor::PlanningSceneMonitorPtr(
            new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

        psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    }

    // -------------------------------------------------------
    PlanningInterface::PlanningInterface(
        const ros::NodeHandle &nh) : nh_(nh)
    {
    }

    // -------------------------------------------------------
    bool PlanningInterface::solve(
        std::string algorithm,
        const PlanningRequest &req,
        PlanningResponse &res)
    {
        dlo_ = std::make_shared<DLO>(nh_, req.dlo_length_);

        dual_arm_ = std::make_shared<DualArm>(nh_, robot_description_name_, req.arm_0_group_name_,
                                              req.arm_1_group_name_, req.dual_arm_group_name_);

        // 加载 ur5 reachibility space
        const std::string reach_space_file_dir = "../data/ur5_reach_space/";
        dual_arm_->arm_0_->loadReachSpace(reach_space_file_dir);
        dual_arm_->arm_1_->loadReachSpace(reach_space_file_dir);

        psm_->requestPlanningSceneState("/get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        scene_ = std::make_shared<Scene>(nh_, dlo_, dual_arm_, lscene);

        return solve(scene_, algorithm, req, res);
    }

    // -------------------------------------------------------
    bool PlanningInterface::solve(
        const Scene::Ptr &scene,
        const std::string algorithm,
        const PlanningRequest &req,
        PlanningResponse &res,
        Visualize::Ptr visualizer)
    {
        if (!visualizer)
            visualizer = std::make_shared<Visualize>(nh_, scene, /*topic_prefix*/ "/planning");

        bool success = false;
        if (algorithm == "JointBiRRT")
        {
            JointBiRRT::Ptr planner = std::make_shared<JointBiRRT>(nh_, scene);
            planner->setVisualizer(visualizer);
            success = planner->solve(req, res);
        }
        // else if(algorithm == "TaskBiRRT"){
        //     TaskBiRRT::Ptr planner = std::make_shared<TaskBiRRT>(nh_, scene_);
        //     planner->setVisualizer(visualizer);
        //     success = planner->solve(req, res);
        // }
        // else if(algorithm == "JointBiRRTStar"){
        //     JointBiRRTStar::Ptr planner = std::make_shared<JointBiRRTStar>(nh_, scene_);
        //     planner->setVisualizer(visualizer);
        //     success = planner->solve(req, res);
        // }
        else
        {
            ROS_ERROR_STREAM("Invalid planning algorithm: " << algorithm);
        }

        return success;
    }

} // end namespace