#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/controller.h"
#include "utils/utils.h"
#include "dlo_arm_planning_pkg/cnpy_utils.h"

#include <std_srvs/Empty.h>
#include "my_msgs/LoadNewUnityScene.h"

using namespace dlo_arm_planning_pkg;

// generate a random seed
std::random_device rd;
int random_seed = rd();
std::default_random_engine Utils::rng(random_seed); // set random seed; must be initialized before main() function.

// -------------------------------------------------------
auto formatDoubleValue(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

// -------------------------------------------------------
void planningTest(
    const Controller::Ptr controller,
    const int task_id,
    const std::string text,
    std::vector<int> path_id_list,
    PlanningRequest &req,
    bool use_fixed_seeds)
{
    auto &dlo_ = controller->dlo_;
    auto &dual_arm_ = controller->dual_arm_;

    // load the DLO derm parameters
    controller->loadDLOParams();

    std::string task_dir = controller->scene_dir_ + "task_" + std::to_string(task_id) + "/";

    std::string dir;
    DLOState start_dlo_state, goal_dlo_state;
    Eigen::VectorXd arm_0_start_joint_pos, arm_1_start_joint_pos, arm_0_goal_joint_pos, arm_1_goal_joint_pos;

    dir = task_dir + "start/";
    CnpyUtils::loadPlanningGoal(dir, start_dlo_state, arm_0_start_joint_pos, arm_1_start_joint_pos, dlo_, dual_arm_);
    dir = task_dir + "goal/";
    CnpyUtils::loadPlanningGoal(dir, goal_dlo_state, arm_0_goal_joint_pos, arm_1_goal_joint_pos, dlo_, dual_arm_);

    // dlo
    req.start_dlo_state_ = start_dlo_state;
    req.goal_dlo_state_ = goal_dlo_state;

    // dual arm
    req.arm_0_group_name_ = dual_arm_->arm_0_->arm_group_name_;
    req.arm_1_group_name_ = dual_arm_->arm_1_->arm_group_name_;
    req.dual_arm_group_name_ = dual_arm_->dual_arm_group_name_;
    req.arm_0_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_start_joint_pos);
    req.arm_1_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_start_joint_pos);
    req.arm_0_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_goal_joint_pos);
    req.arm_1_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_goal_joint_pos);

    for (auto path_id : path_id_list)
    {
        if (!ros::ok())
            break;
        if (use_fixed_seeds)
            Utils::rng.seed(path_id + 1);

        std::cout << text << ", path_id: " << path_id << std::endl;
        std::string path_dir = task_dir + "results/" + text + "/path_" + std::to_string(path_id) + "/";

        PlanningResponse res;
        controller->planning_interface_->solve(controller->scene_, "JointBiRRT", req, res,
                                               /*visualizer (optional)*/ controller->real_time_interface_->planning_visualizer_);

        if (res.success_)
        {
            CnpyUtils::savePath(path_dir + "smoothed_path/", res.smoothed_path_);
        }
        controller->savePlanningDetails(path_dir, res);
    }
}

// -----------------------------------------------------------------
int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // change the logger level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_STREAM("Random seed for this run: " << random_seed);

    // -------------------------------------------------------------------------------------------

    std::vector<int> scene_ids = {3}; // available elements: 0, 1, 2, 3; corresponding to the simualted task 1, 2, 3, 4 in the paper;
    std::vector<int> dlo_erp_types = {1};
    std::vector<bool> b_underactuateds = {true};
    std::vector<std::string> goal_types = {"joint_space"};
    std::vector<double> sample_only_dlo_probabilities = {0.5};
    bool use_fixed_seeds = true;
    std::vector<int> path_id_list;
    for (int i = 0; i < 100; i++)
        path_id_list.push_back(i);

    PlanningRequest req;
    req.b_visualize_start_goal_ = true;        // whether visualizing the start and goal configuration
    req.b_visualize_planning_process_ = false; // whether visualizing the planning process (such as samling and steering)
    req.b_visualize_res_path_ = true;          // whether visualizing the planned path
    // -------------------------------------------------------------------------------------------

    ros::ServiceClient clear_octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
    ros::ServiceClient load_new_scene_client = nh.serviceClient<my_msgs::LoadNewUnityScene>("/unity/load_new_scene");

    for (auto &scene_id : scene_ids)
    {
        // clear the octomap in move group
        std_srvs::Empty empty;
        clear_octomap_client.call(empty);
        ros::Duration(2.0).sleep();

        // load the new unity scene
        my_msgs::LoadNewUnityScene srv;
        srv.request.scene_name = "scene_" + std::to_string(scene_id);
        load_new_scene_client.call(srv);

        // wait for move group converting the point cloud to octomap
        ros::Duration(10.0).sleep();

        // set the scene_id in ros params
        nh.setParam("env_configs/scene_id", scene_id);

        for (auto &dlo_erp_type : dlo_erp_types)
        {
            nh.setParam("dlo_configs/interpolation_type", dlo_erp_type);

            Controller::Ptr controller = std::make_shared<Controller>(nh);

            for (auto b_underactuated : b_underactuateds)
            {
                nh.setParam("planner_configs/common/use_underactuated_steer", b_underactuated);

                for (auto &sample_only_dlo_probability : sample_only_dlo_probabilities)
                {
                    nh.setParam("planner_configs/JointBiRRT/sample_only_dlo_probability", sample_only_dlo_probability);

                    for (auto &goal_type : goal_types)
                    {
                        if (!ros::ok())
                            break;

                        req.goal_type_ = goal_type;

                        std::string planner_name = b_underactuated ? "underactuated" : "fullactuated";
                        planner_name = planner_name + "_erp" + std::to_string(dlo_erp_type) + "_" + goal_type +
                                       "_P_ts=" + formatDoubleValue(sample_only_dlo_probability, 2);

                        planningTest(controller, /*task_id*/ 0, /*text*/ planner_name, path_id_list, req, use_fixed_seeds);

                        if (!ros::ok())
                            break;
                    }
                }
            }
        }
    }

    ros::shutdown();
    return 0;
}