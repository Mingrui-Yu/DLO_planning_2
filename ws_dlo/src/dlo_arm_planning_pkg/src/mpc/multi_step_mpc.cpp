#include "dlo_arm_planning_pkg/mpc/multi_step_mpc.h"

namespace dlo_arm_planning_pkg
{

    using namespace multi_step_mpc_ifopt;

    // ------------------------------------------------------------
    MultiStepMPC::MultiStepMPC(
        const JacobianModel::Ptr &jaco_model) : MPCBase(jaco_model)
    {
        jaco_model_ = jaco_model;

        initializeIpopt();
    }

    // ------------------------------------------------------------
    MultiStepMPC::MultiStepMPC(
        const Scene::Ptr &scene,
        const JacobianModel::Ptr &jaco_model,
        const int n_step) : MPCBase(jaco_model)
    {
        scene_ = scene;
        jaco_model_ = jaco_model;

        initializeComponents(n_step);
        initializeNlpProblem();
        initializeIpopt();
    }

    // ------------------------------------------------------------
    void MultiStepMPC::initializeComponents(
        const int &n_step)
    {
        if (!scene_)
        {
            ROS_ERROR("MultiStepMPC::initializeNlpProblem(): haven't specified scene_.");
            return;
        }
        auto &scene = scene_;
        auto &dual_arm = scene_->dual_arm_;
        auto &arm_0 = scene_->dual_arm_->arm_0_;
        auto &arm_1 = scene_->dual_arm_->arm_1_;
        auto &dlo = scene_->dlo_;
        auto &jaco_model = jaco_model_;

        // ------------------- initialize variables & constraints & costs -----------------------
        // variables
        arm_0_joint_vel_var_ = std::make_shared<ArmJointVelVariables>(n_step, arm_0);
        arm_1_joint_vel_var_ = std::make_shared<ArmJointVelVariables>(n_step, arm_1);

        arm_0_joint_pos_var_ = std::make_shared<ArmJointPosVariables>(n_step, arm_0);
        arm_1_joint_pos_var_ = std::make_shared<ArmJointPosVariables>(n_step, arm_1);

        dual_arm_critial_points_var_ = std::make_shared<DualArmCriticalPointsVariables>(n_step, scene);

        dlo_fps_pos_var_ = std::make_shared<DLOFpsPosVariables>(n_step, dlo->num_fps_);

        dlo_edge_points_var_ = std::make_shared<DLOEdgePointsPosVariables>(n_step, dlo);

        dlo_fps_vel_var_ = std::make_shared<DLOFpsVelVariables>(n_step, dlo->num_fps_);

        // constraints
        dlo_transition_constraint_ = std::make_shared<DLOTransitionConstraint>(n_step, dlo);

        dlo_jacobian_constraint_ = std::make_shared<DLOJacobianConstraint>(n_step, dlo, jaco_model, dual_arm);

        dlo_edge_points_constraint_ = std::make_shared<DLOEdgePointsConstraint>(n_step, dlo);

        arm_0_transition_constraint_ = std::make_shared<ArmTransitionConstraint>(n_step, arm_0);
        arm_1_transition_constraint_ = std::make_shared<ArmTransitionConstraint>(n_step, arm_1);

        dual_arm_critical_points_fk_constraint_ = std::make_shared<DualArmCriticalPointsFKConstraint>(n_step, scene);

        arm_0_start_constraint_ = std::make_shared<ArmJointPosStartConstraint>(arm_0);
        arm_1_start_constraint_ = std::make_shared<ArmJointPosStartConstraint>(arm_1);

        dlo_start_constraint_ = std::make_shared<DLOStartConstraint>(dlo);

        dual_arm_world_dist_constraint_ = std::make_shared<DualArmWorldDistanceConstraint>(n_step, scene);

        dlo_world_dist_constraint_ = std::make_shared<DLOWorldDistanceConstraint>(n_step, scene);

        dlo_edge_world_dist_constraint_ = std::make_shared<DLOEdgeWorldDistanceConstraint>(n_step, scene);

        dlo_overstretch_constraint_ = std::make_shared<DLOOverstretchConstraint>(n_step, dlo);

        // costs
        dlo_fps_pos_cost_ = std::make_shared<DLOFpsPosCost>(n_step, dlo);

        arm_0_joint_pos_cost_ = std::make_shared<ArmJointPosCost>(n_step, arm_0);
        arm_1_joint_pos_cost_ = std::make_shared<ArmJointPosCost>(n_step, arm_1);

        arm_0_joint_vel_cost_ = std::make_shared<ArmJointVelCost>(n_step, arm_0);
        arm_1_joint_vel_cost_ = std::make_shared<ArmJointVelCost>(n_step, arm_1);

        arm_0_joint_acce_cost_ = std::make_shared<ArmJointAcceCost>(n_step, arm_0);
        arm_1_joint_acce_cost_ = std::make_shared<ArmJointAcceCost>(n_step, arm_1);

        ROS_DEBUG_STREAM("MultiStepMPC::initializeNlpProblem(): initialized variables & constraints & costs.");
    }

    // ------------------------------------------------------------
    void MultiStepMPC::initializeNlpProblem(const bool b_print)
    {
        nlp_ = std::make_shared<ifopt::Problem>();

        // variables
        nlp_->AddVariableSet(arm_0_joint_vel_var_);
        nlp_->AddVariableSet(arm_1_joint_vel_var_);

        nlp_->AddVariableSet(arm_0_joint_pos_var_);
        nlp_->AddVariableSet(arm_1_joint_pos_var_);

        nlp_->AddVariableSet(dual_arm_critial_points_var_);

        nlp_->AddVariableSet(dlo_fps_pos_var_);

        nlp_->AddVariableSet(dlo_edge_points_var_);

        nlp_->AddVariableSet(dlo_fps_vel_var_);

        // constraints
        nlp_->AddConstraintSet(dlo_transition_constraint_);

        nlp_->AddConstraintSet(dlo_jacobian_constraint_);

        nlp_->AddConstraintSet(dlo_edge_points_constraint_);

        nlp_->AddConstraintSet(arm_0_transition_constraint_);
        nlp_->AddConstraintSet(arm_1_transition_constraint_);

        nlp_->AddConstraintSet(dual_arm_critical_points_fk_constraint_);

        nlp_->AddConstraintSet(dual_arm_world_dist_constraint_); // avoid collision between dual_arm and world

        nlp_->AddConstraintSet(dlo_world_dist_constraint_); // avoid collision between dlo and world

        nlp_->AddConstraintSet(dlo_edge_world_dist_constraint_); // avoid collision between dlo edges and world

        nlp_->AddConstraintSet(arm_0_start_constraint_);
        nlp_->AddConstraintSet(arm_1_start_constraint_);
        nlp_->AddConstraintSet(dlo_start_constraint_);

        nlp_->AddConstraintSet(dlo_overstretch_constraint_);

        // costs
        nlp_->AddCostSet(dlo_fps_pos_cost_);

        nlp_->AddCostSet(arm_0_joint_pos_cost_);
        nlp_->AddCostSet(arm_1_joint_pos_cost_);

        nlp_->AddCostSet(arm_0_joint_vel_cost_);
        nlp_->AddCostSet(arm_1_joint_vel_cost_);

        nlp_->AddCostSet(arm_0_joint_acce_cost_);
        nlp_->AddCostSet(arm_1_joint_acce_cost_);

        if (b_print)
            nlp_->PrintCurrent();

        ROS_DEBUG_STREAM("MultiStepMPC::initializeNlpProblem(): initialized NLP problem.");
    }

    // ------------------------------------------------------------
    void MultiStepMPC::initializeIpopt()
    {
        // choose solver and options
        ipopt_ = std::make_shared<IpoptSolver>();
        ipopt_->SetOption("linear_solver", "ma27");
        // ipopt_->SetOption("jacobian_approximation", "finite-difference-values");
        ipopt_->SetOption("jacobian_approximation", "exact");
        ipopt_->SetOption("hessian_approximation", "limited-memory");
        ipopt_->SetOption("print_level", 5);
        ipopt_->SetOption("tol", 1e-3);
        ipopt_->SetOption("max_iter", 3000);
        // ipopt_->SetOption("print_user_options", "yes");
        // ipopt_->SetOption("max_wall_time", 10.0); // limit the max computation time
        ipopt_->SetOption("print_frequency_iter", 10);

        ROS_DEBUG_STREAM("MultiStepMPC::initializeIpopt(): finished.");
    }

    /** ------------------------------------------------------------------------------
     * @vars: arm_joint_pos, arm_end_pose
     * @constraints: end_pose_fk_constraint
     * @costs: arm_end_pose
     */
    bool MultiStepMPC::solve(
        const Scene::Ptr &scene,
        const MPCRequest &req,
        MPCResponse &res)
    {
        ROS_DEBUG_STREAM("MultiStepMPC::solveTest(): start.");

        std::chrono::steady_clock::time_point t1, t2;
        std::chrono::duration<double> time_used;
        t1 = std::chrono::steady_clock::now();

        auto &dual_arm = scene->dual_arm_;
        auto &arm_0 = scene->dual_arm_->arm_0_;
        auto &arm_1 = scene->dual_arm_->arm_1_;
        auto &dlo = scene->dlo_;
        auto &jaco_model = jaco_model_;
        int n_step = req.n_step;
        bool b_warm_start = req.b_warm_start;

        initializeNlpProblem(req.b_print_problem);

        ROS_ERROR_STREAM_COND(req.dlo_desired_path.size() != req.n_step, "MultiStepMPC::solve(): the length of req.dlo_desired_path is wrong.");
        ROS_ERROR_STREAM_COND(req.arm_0_desired_path.size() != req.n_step, "MultiStepMPC::solve(): the length of req.arm_0_desired_path is wrong.");
        ROS_ERROR_STREAM_COND(req.arm_1_desired_path.size() != req.n_step, "MultiStepMPC::solve(): the length of req.arm_1_desired_path is wrong.");
        ROS_ERROR_STREAM_COND(req.step_weight.size() != req.n_step, "MultiStepMPC::solve(): the size of req.step_weight is wrong.");

        // ------------------- prepare desired path -----------------------
        Eigen::VectorXd arm_0_desired_path = Eigen::VectorXd::Zero(arm_0->joint_num_ * req.n_step);
        for (size_t t = 0; t < req.n_step; t++)
        {
            arm_0_desired_path.block(arm_0->joint_num_ * t, 0, arm_0->joint_num_, 1) = req.arm_0_desired_path[t];
        }
        Eigen::VectorXd arm_1_desired_path = Eigen::VectorXd::Zero(arm_1->joint_num_ * req.n_step);
        for (size_t t = 0; t < req.n_step; t++)
        {
            arm_1_desired_path.block(arm_1->joint_num_ * t, 0, arm_1->joint_num_, 1) = req.arm_1_desired_path[t];
        }
        Eigen::VectorXd dlo_desired_path = Eigen::VectorXd::Zero(3 * dlo->num_fps_ * req.n_step);
        for (size_t t = 0; t < req.n_step; t++)
        {
            dlo_desired_path.block(3 * dlo->num_fps_ * t, 0, 3 * dlo->num_fps_, 1) = req.dlo_desired_path[t].fps_pos_;
        }

        // ------------------- prepare initial values for variables (with or without warm-start) -----------------------

        // modify the initial state according to the control_input_delay_time
        Eigen::VectorXd arm_0_init_joint_pos = req.arm_0_init_joint_pos + req.control_input_delay_time * req.arm_0_last_joint_vel;
        Eigen::VectorXd arm_1_init_joint_pos = req.arm_1_init_joint_pos + req.control_input_delay_time * req.arm_1_last_joint_vel;
        // dlo
        Eigen::MatrixXd jaco_dlo = jaco_model->calcJacobianMatrix(req.dlo_init_state, dlo->dlo_length_);
        Eigen::MatrixXd jaco_arm_0 = dual_arm->arm_0_->getTcpJacobianMatrix(req.arm_0_init_joint_pos);
        Eigen::MatrixXd jaco_arm_1 = dual_arm->arm_1_->getTcpJacobianMatrix(req.arm_1_init_joint_pos);
        Eigen::MatrixXd jaco_dual_arm = Eigen::MatrixXd::Zero(jaco_arm_0.rows() + jaco_arm_1.rows(), jaco_arm_0.cols() + jaco_arm_1.cols());
        jaco_dual_arm.block(0, 0, jaco_arm_0.rows(), jaco_arm_0.cols()) = jaco_arm_0;
        jaco_dual_arm.block(jaco_arm_0.rows(), jaco_arm_0.cols(), jaco_arm_1.rows(), jaco_arm_1.cols()) = jaco_arm_1;
        Eigen::VectorXd dlo_init_fps_pos = req.dlo_init_state.fps_pos_ +
                                           req.control_input_delay_time * jaco_dlo * jaco_dual_arm * Utils::concatenateTwoVector(req.arm_0_last_joint_vel, req.arm_1_last_joint_vel);

        // prepare the initial values
        Eigen::MatrixXd dlo_fps_pos_var_init = Eigen::MatrixXd::Zero(3 * dlo->num_fps_, n_step + 1);
        Eigen::MatrixXd arm_0_joint_pos_var_init = Eigen::MatrixXd::Zero(arm_0->joint_num_, n_step + 1);
        Eigen::MatrixXd arm_1_joint_pos_var_init = Eigen::MatrixXd::Zero(arm_1->joint_num_, n_step + 1);
        for (size_t t = 0; t <= n_step; t++)
        {
            if (!b_warm_start || t == 0)
            { // if not using warm-start, then assign all initial variable values as the start configuration
                dlo_fps_pos_var_init.col(t) = dlo_init_fps_pos;
                arm_0_joint_pos_var_init.col(t) = arm_0_init_joint_pos;
                arm_1_joint_pos_var_init.col(t) = arm_1_init_joint_pos;
            }
            else if (t == n_step)
            {
                dlo_fps_pos_var_init.col(t) = req.dlo_desired_path[t - 1].fps_pos_;
                arm_0_joint_pos_var_init.col(t) = req.arm_0_desired_path[t - 1];
                arm_1_joint_pos_var_init.col(t) = req.arm_1_desired_path[t - 1];
            }
            else
            {
                dlo_fps_pos_var_init.col(t) = dlo_fps_pos_var_->GetValues().reshaped(3 * dlo->num_fps_, n_step + 1).col(t + 1);
                arm_0_joint_pos_var_init.col(t) = arm_0_joint_pos_var_->GetValues().reshaped(arm_0->joint_num_, n_step + 1).col(t + 1);
                arm_1_joint_pos_var_init.col(t) = arm_1_joint_pos_var_->GetValues().reshaped(arm_1->joint_num_, n_step + 1).col(t + 1);
            }
        }
        Eigen::MatrixXd dlo_fps_vel_var_init = Eigen::MatrixXd::Zero(3 * dlo->num_fps_, n_step);
        Eigen::MatrixXd arm_0_joint_vel_var_init = Eigen::MatrixXd::Zero(arm_0->joint_num_, n_step);
        Eigen::MatrixXd arm_1_joint_vel_var_init = Eigen::MatrixXd::Zero(arm_1->joint_num_, n_step);
        if (b_warm_start)
        { // if not using warm-start, then keep all initial variable values as zero
            for (size_t t = 0; t < n_step; t++)
            {
                if (t == n_step - 1)
                {
                    dlo_fps_vel_var_init.col(t) = dlo_fps_vel_var_->GetValues().reshaped(3 * dlo->num_fps_, n_step).col(t);
                    arm_0_joint_vel_var_init.col(t) = arm_0_joint_vel_var_->GetValues().reshaped(arm_0->joint_num_, n_step).col(t);
                    arm_1_joint_vel_var_init.col(t) = arm_1_joint_vel_var_->GetValues().reshaped(arm_1->joint_num_, n_step).col(t);
                }
                else
                {
                    dlo_fps_vel_var_init.col(t) = dlo_fps_vel_var_->GetValues().reshaped(3 * dlo->num_fps_, n_step).col(t + 1);
                    arm_0_joint_vel_var_init.col(t) = arm_0_joint_vel_var_->GetValues().reshaped(arm_0->joint_num_, n_step).col(t + 1);
                    arm_1_joint_vel_var_init.col(t) = arm_1_joint_vel_var_->GetValues().reshaped(arm_1->joint_num_, n_step).col(t + 1);
                }
            }
        }

        // ------------------- set initial values for variables -----------------------
        dlo_fps_pos_var_->SetVariables(dlo_fps_pos_var_init.reshaped(dlo_fps_pos_var_init.size(), 1));

        arm_0_joint_pos_var_->SetVariables(arm_0_joint_pos_var_init.reshaped(arm_0_joint_pos_var_init.size(), 1));
        arm_1_joint_pos_var_->SetVariables(arm_1_joint_pos_var_init.reshaped(arm_1_joint_pos_var_init.size(), 1));

        dlo_fps_vel_var_->SetVariables(dlo_fps_vel_var_init.reshaped(dlo_fps_vel_var_init.size(), 1));

        arm_0_joint_vel_var_->SetVariables(arm_0_joint_vel_var_init.reshaped(arm_0_joint_vel_var_init.size(), 1));
        arm_1_joint_vel_var_->SetVariables(arm_1_joint_vel_var_init.reshaped(arm_1_joint_vel_var_init.size(), 1));

        dual_arm_critial_points_var_->SetVariablesByJointPath(arm_0_joint_pos_var_->GetValues(), arm_1_joint_pos_var_->GetValues());
        dlo_edge_points_var_->setVariablesByFpsPos(dlo_fps_pos_var_->GetValues());

        // ------------------- set parameters -----------------------
        // variables
        arm_0_joint_vel_var_->SetParams(req.arm_0_max_joint_vel);
        arm_1_joint_vel_var_->SetParams(req.arm_1_max_joint_vel);

        dlo_fps_vel_var_->SetParams(req.dlo_fps_max_vel);

        // constraints
        dlo_transition_constraint_->SetParams(req.delta_t);

        arm_0_transition_constraint_->SetParams(req.delta_t);
        arm_1_transition_constraint_->SetParams(req.delta_t);

        arm_0_start_constraint_->SetParams(arm_0_init_joint_pos); // start configurations
        arm_1_start_constraint_->SetParams(arm_1_init_joint_pos);

        dlo_start_constraint_->SetParams(dlo_init_fps_pos); // start configurations

        dual_arm_world_dist_constraint_->SetParams(/*min_dist_thres*/ req.min_dist_thres);

        dlo_world_dist_constraint_->SetParams(/*min_dist_thres*/ req.min_dist_thres);

        dlo_edge_world_dist_constraint_->SetParams(/*min_dist_thres*/ req.min_dist_thres);

        dlo_overstretch_constraint_->SetParams(req.overstretch_thres);

        // costs
        dlo_fps_pos_cost_->SetParams(/*weight*/ req.dlo_fps_pos_cost_weight, req.step_weight, dlo_desired_path);

        arm_0_joint_pos_cost_->SetParams(/*weight*/ req.arm_joint_pos_cost_weight, req.step_weight, req.arm_0_joint_pos_weights, arm_0_desired_path);
        arm_1_joint_pos_cost_->SetParams(/*weight*/ req.arm_joint_pos_cost_weight, req.step_weight, req.arm_1_joint_pos_weights, arm_1_desired_path);

        arm_0_joint_vel_cost_->SetParams(/*weight*/ req.arm_joint_vel_cost_weight, req.arm_0_joint_vel_weights);
        arm_1_joint_vel_cost_->SetParams(/*weight*/ req.arm_joint_vel_cost_weight, req.arm_1_joint_vel_weights);

        arm_0_joint_acce_cost_->SetParams(/*weight*/ req.arm_joint_acce_cost_weight, req.arm_0_joint_vel_weights,
                                          req.arm_0_last_joint_vel, req.delta_t);
        arm_1_joint_acce_cost_->SetParams(/*weight*/ req.arm_joint_acce_cost_weight, req.arm_1_joint_vel_weights,
                                          req.arm_1_last_joint_vel, req.delta_t);

        // ------------------- solving -----------------------
        ipopt_->SetOption("max_wall_time", req.delta_t * 0.8); // limit the max computation time
        ipopt_->SetOption("print_level", req.b_print_solving_log_level);

        ipopt_->Solve(*(nlp_.get()));

        res.arm_0_joint_vel = arm_0_joint_vel_var_->GetValues().block(0, 0, arm_0->joint_num_, 1); // 只取 q_vel_{t=0}
        res.arm_1_joint_vel = arm_1_joint_vel_var_->GetValues().block(0, 0, arm_1->joint_num_, 1);

        ROS_DEBUG_STREAM("MultiStepMPC::solve(): finish.");

        return true;
    }

} // end namespace
