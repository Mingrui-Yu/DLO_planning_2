#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/visualize.h"
#include "dlo_arm_planning_pkg/planning_interface.h"

/**
 * an example to quick test the planner without using Unity
 * the obstacles are manually defined in this script
 */

using namespace dlo_arm_planning_pkg;
std::random_device rd;
std::default_random_engine Utils::rng(1);

// -------------------------------------------------------
void addObstacle()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "obstacle_1";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.4;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.4;
        box_pose.position.y = -0.15;
        box_pose.position.z = -0.4;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "obstacle_2";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.4;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.4;
        box_pose.position.y = 0.15;
        box_pose.position.z = -0.4;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();

    // // clear previous collision objects
    // std::vector<std::string> existing_object_ids = planning_scene_interface.getKnownObjectNames();
    // planning_scene_interface.removeCollisionObjects(existing_object_ids);

    // add new collision objects
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep(); // required: leave some time for publishing the message to Move Group
}

// -----------------------------------------------------------------
int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]); // ceres 使用了 glog

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;

    // change the logger level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // planning scene monitor
    std::string ROBOT_DESCRIPTION = "robot_description";
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    // planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    DLO::Ptr dlo = DLO::Ptr(new DLO(nh, /*dlo_length*/ 0.5));
    DualArm::Ptr dual_arm = std::make_shared<DualArm>(nh, ROBOT_DESCRIPTION, "arm_0", "arm_1", "dual_arm");
    Scene::Ptr scene = std::make_shared<Scene>(nh, dlo, dual_arm, planning_scene_monitor::LockedPlanningSceneRO(psm));
    Visualize::Ptr visualizer = std::make_shared<Visualize>(nh, scene);

    // ----------------------- add obstacles ------------------------------

    addObstacle();

    // ----------------------- start and goal ------------------------------

    /**
     * assign the start and goal DLO configurations
     */
    // start
    Eigen::Vector3d first_fp, mid_fp, last_fp;
    first_fp << 0.6, -0.2, -0.4;
    mid_fp << 0.4, 0.0, -0.4;
    last_fp << 0.2, -0.2, -0.4;
    DLOState start_dlo_state = dlo->interpolateCoarseShape(first_fp, mid_fp, last_fp);
    start_dlo_state = dlo->optimizeShapeDermCeres(start_dlo_state);
    // goal
    first_fp << 0.2, 0.2, -0.4;
    mid_fp << 0.4, 0.0, -0.4;
    last_fp << 0.6, 0.2, -0.4;
    DLOState goal_dlo_state = dlo->interpolateCoarseShape(first_fp, mid_fp, last_fp);
    goal_dlo_state = dlo->optimizeShapeDermCeres(goal_dlo_state);

    /**
     * assign the start and goal robot configurations via IK
     */
    std::vector<double> arm_0_default_joint_pos{M_PI, -3.0 / 4.0 * M_PI, -1.0 / 2.0 * M_PI, -3.0 / 4.0 * M_PI, 0.0, 0.0};
    std::vector<double> arm_1_default_joint_pos{M_PI, -1.0 / 4.0 * M_PI, 1.0 / 2.0 * M_PI, -1.0 / 4.0 * M_PI, 0.0, 0.0};
    // start
    Eigen::VectorXd arm_0_start_joint_pos, arm_1_start_joint_pos;
    dual_arm->arm_0_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), start_dlo_state.getLeftEndPose(), arm_0_start_joint_pos);
    dual_arm->arm_1_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), start_dlo_state.getRightEndPose(), arm_1_start_joint_pos);
    // goal
    Eigen::VectorXd arm_0_goal_joint_pos, arm_1_goal_joint_pos;
    dual_arm->arm_0_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), goal_dlo_state.getLeftEndPose(), arm_0_goal_joint_pos);
    dual_arm->arm_1_->armTcpClosestIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), goal_dlo_state.getRightEndPose(), arm_1_goal_joint_pos);
    std::cout << "start joint pos: arm_0: " << arm_0_start_joint_pos.transpose() << ", arm_1: " << arm_1_start_joint_pos.transpose() << std::endl;
    std::cout << "goal joint pos: arm_0: " << arm_0_goal_joint_pos.transpose() << ", arm_1: " << arm_1_goal_joint_pos.transpose() << std::endl;

    // ----------------------- planning request ------------------------------
    PlanningRequest req;
    PlanningResponse res;
    req.goal_type_ = "task_space";
    // dlo
    req.dlo_length_ = 0.5;
    req.start_dlo_state_ = start_dlo_state;
    req.goal_dlo_state_ = goal_dlo_state;
    // dual arm
    req.arm_0_group_name_ = "arm_0";
    req.arm_1_group_name_ = "arm_1";
    req.dual_arm_group_name_ = "dual_arm";
    req.arm_0_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_start_joint_pos);
    req.arm_1_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_start_joint_pos);
    req.arm_0_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_goal_joint_pos);
    req.arm_1_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_goal_joint_pos);

    req.b_visualize_start_goal_ = false;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    // ----------------------- do planning ------------------------------
    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    std::vector<std::string> test_algorthms{"JointBiRRT"};
    const int num_test = 20;
    std::vector<double> time_cost_all;
    std::vector<double> iter_all;
    std::vector<double> path_cost_all;

    for (auto &algorithm_name : test_algorthms)
    {
        int num_success = 0;
        for (size_t i = 0; ros::ok() && i < num_test; i++)
        {
            std::cout << "test " << i << ": " << std::endl;
            pi->solve(algorithm_name, req, res);
            if (res.success_)
            {
                num_success++;
                time_cost_all.push_back(res.total_time_cost_);
                iter_all.push_back(double(res.total_iter_));
                path_cost_all.push_back(res.path_cost_);
            }
        }
        double time_cost_mean, time_cost_std, iter_mean, iter_std, path_cost_mean, path_cost_std;
        Utils::calcMeanAndStdOfVector(time_cost_all, time_cost_mean, time_cost_std);
        Utils::calcMeanAndStdOfVector(iter_all, iter_mean, iter_std);
        Utils::calcMeanAndStdOfVector(path_cost_all, path_cost_mean, path_cost_std);

        std::cout << "Results: success rate: " << double(num_success) / double(num_test) << std::endl;
        std::cout << "Results: time cost: " << time_cost_mean << " +- " << time_cost_std << std::endl;
        std::cout << "Results: iteration: " << iter_mean << " +- " << iter_std << std::endl;
        std::cout << "Results: path cost: " << path_cost_mean << " +- " << path_cost_std << std::endl;
    }

    return 0;
}