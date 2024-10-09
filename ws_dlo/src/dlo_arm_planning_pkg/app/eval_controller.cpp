#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/controller.h"
#include "utils/utils.h"

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
    bool b_reload_scene = true;            // whether reloading the Unity Scene
    bool b_new_initial2start_path = false; // whether planning new paths from initial to start configuration
    bool b_new_path = false;               // whether planning new paths from start to goal configuration (overwriting the paths planned by `eval_planner.cpp`)

    std::vector<int> scene_ids = {3}; // available elements: 0, 1, 2, 3; corresponding to the simualted task 1, 2, 3, 4 in the paper;
    std::vector<int> dlo_erp_types = {1};
    std::vector<bool> b_underactuateds = {true};
    std::vector<std::string> goal_types = {"joint_space"};
    std::vector<double> sample_only_dlo_probabilities = {0.5};
    std::vector<std::string> execution_names = {"closed_loop"}; // available elements: "closed_loop", "open_loop", "open_loop_replanning"

    std::vector<int> path_id_list;
    for (int i = 0; i < 100; i++)
        path_id_list.push_back(i);

    ExecutionOptions options;
    options.b_save_execution_results = true;
    options.execute = true;

    // -------------------------------------------------------------------------------------------
    std::vector<int> task_id_list = {0};
    std::vector<int> execution_id_list = {0};

    ros::ServiceClient clear_octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
    ros::ServiceClient load_new_scene_client = nh.serviceClient<my_msgs::LoadNewUnityScene>("/unity/load_new_scene");

    options.execute_path_type = "smoothed";

    for (auto &scene_id : scene_ids)
    {
        if (b_reload_scene)
        {
            // clear the octomap in move group
            std_srvs::Empty empty;
            clear_octomap_client.call(empty);
            ROS_INFO("Clear the octomap.");
            ros::Duration(2.0).sleep();

            // load the new unity scene
            my_msgs::LoadNewUnityScene srv;
            srv.request.scene_name = "scene_" + std::to_string(scene_id);
            load_new_scene_client.call(srv);
            ROS_INFO("Load the new scene.");
            // wait for move group converting the point cloud to octomap
            ros::Duration(10.0).sleep();
        }

        options.plan_to_start_config = b_new_initial2start_path;
        options.save_path_to_start_config = b_new_initial2start_path;

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

                        std::string planner_name = b_underactuated ? "underactuated" : "fullactuated";
                        planner_name = planner_name + "_erp" + std::to_string(dlo_erp_type) + "_" + goal_type +
                                       "_P_ts=" + formatDoubleValue(sample_only_dlo_probability, 2);

                        options.planner_name = planner_name;

                        for (int &task_id : task_id_list)
                        {
                            for (int &path_id : path_id_list)
                            {
                                options.plan_to_goal_config = b_new_path;
                                options.save_path_to_goal_config = b_new_path;

                                for (auto &execution_name : execution_names)
                                {
                                    options.execution_name = execution_name;
                                    options.tracking_options.mode = execution_name;

                                    for (int &execution_id : execution_id_list)
                                    {
                                        std::cout << "scene_id: " << scene_id << ", planner_name: " << planner_name << ", path_id: " << path_id
                                                  << ", execution_name: " << execution_name << ", execution_id: " << execution_id << std::endl;

                                        controller->oneExecution(task_id, path_id, execution_id, options);

                                        // reset Unity scene
                                        controller->real_time_interface_->resetUnityScene();

                                        options.plan_to_start_config = false;
                                        options.save_path_to_start_config = false;
                                        options.plan_to_goal_config = false;
                                        options.save_path_to_goal_config = false;

                                        if (!ros::ok())
                                            break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    ros::shutdown(); // 必须要有
    return 0;
}