# Generalizable whole-body global manipulation of deformable linear objects by dual-arm robot in 3-D constrained environments

[[Project website](https://mingrui-yu.github.io/DLO_planning_2/)]

Repository for the paper Generalizable whole-body global manipulation of deformable linear objects by dual-arm robot in 3-D constrained environments, **IJRR**, 2024.

Here we provide:

- The code of the proposed manipulation algorithm, which is mainly written in C++.

* A simulator built in Unity for simulating the manipulation and demonstrating the performance.

## Installation

1. Install Unity for Linux 2020.03 [doc](https://docs.unity3d.com/2020.2/Documentation/Manual/GettingStartedInstallingHub.html) (First install Unity Hub, then install Unity Editor 2020.03).

1. Install Eigen 3.4.0 [doc](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download). (Note: required verion >= 3.4.0)

1. Install ceres 2.1.0 [doc](http://ceres-solver.org/installation.html).

1. Install Ipopt [doc](https://coin-or.github.io/Ipopt/INSTALL.html).

1. Install ifopt [doc](https://github.com/THU-DA-Robotics/ifopt).

1. Install cnpy [doc](https://github.com/rogersce/cnpy).

1. Install yaml-cpp 0.6.2 [doc](https://github.com/jbeder/yaml-cpp) if it is not installed on your system. (The version 0.7.0 contains a bug which makes `find_package(yaml-cpp)` not work correctly.)

1. Make sure the **default python** of the system is **python3** (and not in conda virtual env). You can set python3 as the default python using

   ```bash
   $ sudo apt install python-is-python3
   ```

1. Install the following dependences in the **system's** python3:

   ```
   pip install empy
   pip install numpy
   ```

1. Install the following dependences in your **conda** python3 env (<= python 3.8.10):

   ```
   pip install rospkg
   pip install scipy
   pip install matplotlib
   pip install scikit-learn
   ```

1. Clone the repo:

   ```bash
   $ git clone https://github.com/Mingrui-Yu/DLO_planning_2.git
   ```

1. Clone third-party ROS packages:

   ```bash
   $ cd <YOUR_PATH>/DLO_planning_2_private/ws_dlo/src

   $ git clone https://github.com/THU-DA-Robotics/universal_robot.git -b calibration_devel
   $ git clone https://github.com/THU-DA-Robotics/robotiq.git -b noetic-devel
   $ git clone https://github.com/THU-DA-Robotics/dual_ur.git -b noetic_devel

   $ git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
   ```

1. Install custom MoveIt! in `ws_dlo/src/moveit_all`. [doc](https://github.com/Mingrui-Yu/moveit)

1. Install other dependences:

   ```
   cd <YOUR_PATH>/DLO_planning_private/ws_dlo
   rosdep install --from-paths src --ignore-src -y
   ```

1. Build the catkin workspaces:

   ```bash
   cd DLO_planning_private/ws_dlo
   catkin_make -j8
   ```

1. Modify the `project_dir` in `ws_dlo/src/dlo_arm_planning_pkg/config/sim/configs.yaml` to <YOUR_PATH_TO_DLO_planning_2>.

1. Install [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) into this Unity project [doc](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md).

## Usage in Simulation

In each following terminal:

```bash
cd <YOUR_PATH>/ws_dlo/
source devel/setup.bash
```

In a new terminal:

```bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

Then, in a new terminal:

```bash
roslaunch dlo_arm_planning_pkg sim_prepare.launch
```

Then, in a new terminal:

```bash
rosrun dlo_arm_planning_pkg <EXECUTABLE_FILE>
```

## Available executable files

**First, follow the above usage instructions.**

### An example to visualize the stable DLO configuration projection

```bash
# save the results
rosrun dlo_arm_planning_pkg example_vis_derm_projection

# visualize the results
# in your conda python3 env
python src/dlo_arm_planning_pkg/scripts/results/plot_derm_projection.py
```

### An example to test the planner without using Unity

Test the planner in a custom example environment for tens of times and print the statistic results.

```bash
rosrun dlo_arm_planning_pkg example_planner_without_unity
```

### Evaluating the planner in the simulated Tasks

1. Launch the Unity Editor.

1. Click the `Run` buttion in the Unity Editor.

1. Run the cpp file:

   ```bash
   rosrun dlo_arm_planning_pkg eval_planner
   ```

   You can change the scene id (task id) and whether visualizing the start goal configurations / planned paths in `ws_dlo/src/dlo_arm_planning_pkg/app/eval_planner.cpp`. Remeber to re-catkin_make the project after any cpp modification.

1. Compute and print the statistic results of all planned paths:

   ```bash
   # in your conda python env
   python src/dlo_arm_planning_pkg/scripts/results/eval_planner.py
   ```

   You can change the scene id (task id) in the python script.

### Evaluating the controller in the simulated Tasks

1. Launch the Unity Editor.

1. Click the `Run` buttion in the Unity Editor.

1. Run the cpp file:

   ```bash
   rosrun dlo_arm_planning_pkg eval_controller
   ```

   You can change the scene id (task id) and whether planning new paths in `ws_dlo/src/dlo_arm_planning_pkg/app/eval_controller.cpp`. Remeber to re-catkin_make the project after any cpp modification.

1. Compute and print the statistic results of all planned paths:

   ```bash
   # in your conda python env
   python src/dlo_arm_planning_pkg/scripts/results/eval_controller.py
   ```

   You can change the scene id (task id) in the python script.

### Structions of the evalution data of the simualted Task 1, 2, 3, 4

- `data/sim/scene_<id>/`:

  - `dlo_identification`:
    - `dlo_derm_params.yaml`: the identificated DLO DER model parameters.
    - `all_dlo_end_0_quat.npy & all_dlo_end_1_quat.npy & all_dlo_fps_pos.npy`: the recorded DLO configurations during the identification trajectory.
  - `task_0`:
    - `start/ & goal/`: the pre-defined start and goal configurations.
    - `path_initial_to_start/`: (irrelevant to the proposed method); the path from the initial configuration in Unity (a straight line) to the start configuration, only used in actual executions.
    - `results/`:
      - `<name>`: the name of the specific planning setup.
        - `path_<id>`:
          - `smoothed_path/`: the planned path.
          - `planning_details.yaml`: the planning details.
          - `<execution_name>`: the name of the execution setup.
            - `execution_<id>`:
              - `execution_details.yaml`: the execution details.
              - `reference_path`: the interpolated planned path, which is used as the reference path of the MPC.
              - `actual_path`: the path of actual execution.
