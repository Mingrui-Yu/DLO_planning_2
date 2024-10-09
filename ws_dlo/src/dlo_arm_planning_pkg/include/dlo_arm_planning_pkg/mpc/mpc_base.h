#ifndef DLO_ARM_PLANNING_MPC_MPC_BASE_MPC_H
#define DLO_ARM_PLANNING_MPC_MPC_BASE_MPC_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/jacobian_model.h"


namespace dlo_arm_planning_pkg{

// ----------------------------------------------------------
struct MPCRequest
{
    double delta_t;
    bool b_warm_start = false; // whether using warm-start
    
    double dlo_fps_pos_cost_weight = 10.0;
    double arm_joint_pos_cost_weight = 1.0;
    double arm_joint_vel_cost_weight = 0.1;
    double arm_joint_acce_cost_weight = 0.1;
    double min_dist_thres = 0.02;
    double control_input_delay_time = 0.0;
    double overstretch_thres = 0.0;

    // start configuration
    DLOState dlo_init_state;
    Eigen::VectorXd arm_0_init_joint_pos, arm_1_init_joint_pos;
    Eigen::VectorXd arm_0_last_joint_vel, arm_1_last_joint_vel;

    // goal for one-step MPC
    DLOState dlo_desired_state;
    Eigen::VectorXd arm_0_desired_joint_pos, arm_1_desired_joint_pos;

    // goal for multi-step MPC
    int n_step; // horizon
    std::vector<DLOState> dlo_desired_path; // t = 1,..T
    VecEigenVectorXd arm_0_desired_path, arm_1_desired_path; // t = 1,..T
    std::vector<double> step_weight; // t = 1,..T

    // constraints
    Eigen::VectorXd arm_0_joint_pos_weights, arm_1_joint_pos_weights;
    Eigen::VectorXd arm_0_joint_vel_weights, arm_1_joint_vel_weights;
    Eigen::VectorXd arm_0_max_joint_vel, arm_1_max_joint_vel;
    double dlo_fps_max_vel;

    bool b_print_problem = false;
    int b_print_solving_log_level = 3;
};


// ----------------------------------------------------------
struct MPCResponse
{
    Eigen::VectorXd arm_0_joint_vel, arm_1_joint_vel;
};


// ----------------------------------------------------------
class MPCBase
{
public:
    typedef std::shared_ptr<MPCBase> Ptr;

    MPCBase(){}

    MPCBase(
        const JacobianModel::Ptr &jaco_model
    );

    virtual bool solve(
        const Scene::Ptr &scene,
        const MPCRequest &req,
        MPCResponse &res
    ) = 0;

public:
    std::string ALGORITHM_NAME = "";

    JacobianModel::Ptr jaco_model_;
    
};










} // end namespace dlo_arm_planning_pkg
#endif