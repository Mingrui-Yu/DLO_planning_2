import numpy as np
import rospy
import os
import yaml

project_dir = rospy.get_param("project_configs/project_dir")
sim_or_real = rospy.get_param("env_configs/sim_or_real")
num_fps = rospy.get_param("dlo_configs/num_fps")


scene_id = 3 # available choices: 0, 1, 2, 3; corresponding to the simualted task 1, 2, 3, 4 in the paper;
method_names = ["closed_loop"] # available choices: "closed_loop", "open_loop", "open_loop_replanning"
underactuated = True
overstretch_thres = 1.2

if sim_or_real == "sim":
    dlo_lengths_gt = [0.508538, 0.388826, 0.57, 0.475587]

scene_dir = project_dir + "data/" + sim_or_real + "/scene_" + str(scene_id) + "/"
task_id = 0
task_dir = scene_dir + "task_" + str(task_id) + "/"
print("scene id: ", scene_id, ", task_id: ", task_id)
success_error_thres = rospy.get_param("controller_configs/success_error_thres")


for method_name in method_names:
    print("---------------- method_name: ", method_name)
    
    if underactuated:
        result_dir = task_dir + "results/underactuated_erp1_joint_space_P_ts=0.50/"
    else:
        result_dir = task_dir + "results/fullactuated_erp1_joint_space_P_ts=0.50/"

    all_results = {}
    average_results = {}

    path_id = 0
    n_plan_success = 0
    n_success = 0
    while not rospy.is_shutdown():
        path_dir = result_dir + "path_" + str(path_id) + "/"
        planning_details_file = path_dir + "planning_details.yaml"
        
        # if reach the max number of paths
        if not os.path.exists(planning_details_file):
            break

        # if planning failed
        with open(planning_details_file, 'r') as file:
            details = yaml.safe_load(file)
            if details["success"] is False:
                print("path_" + str(path_id) + " planning failed.")
                path_id += 1
                continue

        exe_result_file = path_dir + method_name + "/execution_0/execution_details.yaml"
        if not os.path.exists(exe_result_file):
            break

        with open(exe_result_file, 'r') as file:
            details = yaml.safe_load(file)
            dlo_fps_pos_path = np.load(path_dir + method_name + "/execution_0/actual_path/dlo_fps_pos_path.npy")
            dlo_fps_pos_path = dlo_fps_pos_path.reshape(-1, num_fps, 3)
            dlo_lengths = np.sum(np.linalg.norm(dlo_fps_pos_path[:, 1:, :] - dlo_fps_pos_path[:, :-1, :], axis=2), axis=1)

            if details["final_task_error"] < success_error_thres \
                    and (sim_or_real == "real" or np.all(dlo_lengths < dlo_lengths_gt[scene_id] * overstretch_thres)):
                n_success += 1
                for key in details.keys():
                    if key not in all_results:
                        all_results[key] = []
                    all_results[key].append(details[key])

        path_id += 1    
        n_plan_success += 1    

    if path_id > 0:
        # averaging the planning results
        if(n_success > 0):
            for key in all_results.keys():
                average_results[key] = np.mean(np.array(all_results[key]))
                average_results[key + "_std"] = np.std(np.array(all_results[key]))

        average_results["success_rate"] = str(n_success) + "/" + str(n_plan_success)

        print("success_rate: ", average_results["success_rate"])
        if(n_success > 0):
            print("final_task_error: {:.4f} +- {:.4f}".format(average_results["final_task_error"]*1000, average_results["final_task_error_std"]*1000))
            print("collision_time: {:.2f} +- {:.2f}".format(average_results["collision_time"], average_results["collision_time_std"]))
            print("execution_time: {:.2f} +- {:.2f}".format(average_results["execution_time"], average_results["execution_time_std"]))
            print("replanning_count: {:n}".format(np.sum(np.array(all_results["replanning_count"]))))
            print("ave_time_cost_mpc: {:.3f} +- {:.3f}".format(average_results["ave_time_cost_mpc"], average_results["ave_time_cost_mpc_std"]))

    
        

        
