import numpy as np
import rospy
import os
import yaml

project_dir = rospy.get_param("project_configs/project_dir")
sim_or_real = rospy.get_param("env_configs/sim_or_real")

scene_id = 3  # available choices: 0, 1, 2, 3; corresponding to the simualted task 1, 2, 3, 4 in the paper;
method_names = ["underactuated_erp1_joint_space_P_ts=0.50"]

scene_dir = project_dir + "data/" + sim_or_real + "/scene_" + str(scene_id) + "/"
task_id = 0
task_dir = scene_dir + "task_" + str(task_id) + "/"
print("scene id: ", scene_id, ", task_id: ", task_id)

ratio = 0.8 # only including the shortest 80% of trails (according to planning time cost) to exclude extreme cases

for method_name in method_names:
    print("---------------- method_name: ", method_name)

    result_dir = task_dir + "results/" + method_name + "/"
    all_results = {}
    average_results = {}

    path_id = 0
    n_success = 0
    while not rospy.is_shutdown():
        result_file = result_dir + "path_" + str(path_id) + "/planning_details.yaml"

        if not os.path.exists(result_file):
            break

        with open(result_file, 'r') as file:
            details = yaml.safe_load(file)
            if details["success"] is True:
                n_success += 1
                for key in details.keys():
                    if key != "success":
                        if key not in all_results:
                            all_results[key] = []
                        all_results[key].append(details[key])

        path_id += 1        

    if path_id > 0:
        # averaging the planning results
        if(n_success > 0):
            for key in all_results.keys():
                average_results[key] = np.mean(np.array(all_results[key]))
                average_results[key + "_std"] = np.std(np.array(all_results[key]))

        average_results["success_rate"] = str(n_success) + "/" + str(path_id)

        # save the average planning details as a yaml file
        with open(result_dir + "average_planning_details.yaml", 'w') as file:
            yaml.dump(average_results, file)

        print("success_rate: ", average_results["success_rate"])
        if(n_success > 0):
            sorted_idx = np.argsort(np.array(all_results["time_cost_find_feasible_path"]))
            sorted_idx = sorted_idx[:int(n_success * ratio)]

            sorted_time_cost = np.array(all_results["time_cost_find_feasible_path"])[sorted_idx]
            sorted_rrt_iter = np.array(all_results["rrt_iter_find_feasible_path"])[sorted_idx]

            print("top {:n}% time_feasible: {:.2f} +- {:.2f}".format(ratio*100, np.mean(sorted_time_cost), np.std(sorted_time_cost)))
            print("path_length_feasible: {:.2f} +- {:.2f}".format(average_results["path_length_feasible"], average_results["path_length_feasible_std"]))
            print("time_smooth: {:.2f} +- {:.2f}".format(average_results["time_cost_path_smooth"], average_results["time_cost_path_smooth_std"]))
            print("path_length_smooth: {:.2f} +- {:.2f}".format(average_results["path_length_smooth"], average_results["path_length_smooth_std"]))
            

            
    
        

        
