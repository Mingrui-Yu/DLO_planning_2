#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dlo_model/derm_ceres.h"
#include "dlo_arm_planning_pkg/dlo_model/derm_eigen.h"
#include "dlo_arm_planning_pkg/dlo_model/derm_ipopt.h"

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

namespace dlo_arm_planning_pkg
{

    // ------------------------------------------------------------
    DLO::DLO(int num_fps, double length, int dim)
    {
        num_fps_ = num_fps;
        dlo_length_ = length;
        dim_ = dim;
    }

    // ------------------------------------------------------------
    DLO::DLO(
        const ros::NodeHandle &nh) : nh_(nh)
    {
        loadParams();
    }

    // ------------------------------------------------------------
    DLO::DLO(
        const ros::NodeHandle &nh,
        double length) : nh_(nh)
    {
        dlo_length_ = length;
        loadParams();
    }

    // -------------------------------------------------------
    void DLO::loadParams()
    {
        std::string param_name;

        param_name = "dlo_configs/num_fps";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, num_fps_);

        param_name = "dlo_configs/num_point_per_edge";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, num_point_per_edge_);

        param_name = "dlo_configs/interpolation_type";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_interpolation_type_);

        std::vector<double> dlo_sample_lb, dlo_sample_ub;
        param_name = "planner_configs/common/dlo_sample_range/lb";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_sample_lb);
        param_name = "planner_configs/common/dlo_sample_range/ub";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, dlo_sample_ub);
        if (dlo_sample_lb[2] == dlo_sample_ub[2])
        {
            dim_ = 2;
            ROS_INFO_STREAM("The dimension of the environment is 2.");
        }
        else if (dlo_sample_lb[2] < dlo_sample_ub[2])
        {
            dim_ = 3;
            ROS_INFO_STREAM("The dimension of the environment is 3.");
        }
        else
        {
            ROS_ERROR_STREAM("Invalid dlo_sample_range.");
        }

        param_name = "dlo_identification_configs/pso/n_particles";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, identification_pso_n_particles_);

        param_name = "dlo_identification_configs/pso/n_iter";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, identification_pso_n_iter_);

        param_name = "dlo_identification_configs/pso/omega";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, identification_pso_omega_);

        param_name = "dlo_identification_configs/pso/c_1";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, identification_pso_c_1_);

        param_name = "dlo_identification_configs/pso/c_2";
        ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
        nh_.getParam(param_name, identification_pso_c_2_);
    }

    // ------------------------------------------------------------
    void DLO::setLength(double length)
    {
        dlo_length_ = length;
    }

    // ------------------------------------------------------------
    void DLO::setDermParams(const DLOEnergyModelParams &params)
    {
        derm_params_ = params;
    }

    /** ------------------------------------------------------------
     * @brief compute the reference height for gravity energy
     */
    double DLO::gravityRefHeight(
        const Eigen::VectorXd &fps_pos,
        const double &dlo_length)
    {
        // min height of the DLO feature points
        double min_z = 1e8;
        for (int k = 0; k < fps_pos.size() / 3; k++)
        {
            double z = fps_pos[3 * k + 2];
            if (z < min_z)
                min_z = z;
        }
        // reference_height = lowest_height - DLO_length
        double ref_height = min_z - dlo_length;
        return ref_height;
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDerm(
        const DLOState &dlo_state_init,
        const OptimizeOptions optimize_options)
    {
        return optimizeShapeDerm(dlo_state_init, derm_params_, optimize_options);
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDerm(
        const DLOState &dlo_state_init,
        const DLOEnergyModelParams &model_params,
        const OptimizeOptions optimize_options)
    {
        if (optimize_options.solver == "Ceres")
            return optimizeShapeDermCeres(dlo_state_init, model_params, optimize_options);
        else if (optimize_options.solver == "Ipopt")
            return optimizeShapeDermIpopt(dlo_state_init, model_params, optimize_options);
        else if (optimize_options.solver == "GD")
            return optimizeShapeDermGD(dlo_state_init, model_params, optimize_options);
        else
            ROS_ERROR_STREAM("DLO::optimizeShapeDerm() doesn't support solver: " << optimize_options.solver);

        return DLOState();
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDermGD(
        const DLOState &dlo_state_init,
        const DLOEnergyModelParams &model_params,
        const OptimizeOptions &optimize_options)
    {
        int num_fps = num_fps_; // vertex 编号为 0 ~ (num_fps + 1)
        double dlo_length;
        Eigen::VectorXd extend_edges_length(num_fps + 1);

        if (optimize_options.use_origin_edges_length)
        {
            dlo_length = dlo_state_init.getLength();
            extend_edges_length = dlo_state_init.getExtendEdgesLength();
        }
        else
        {
            dlo_length = dlo_length_;
            double ave_edge_length = dlo_length / (num_fps - 1.0);
            for (int k = 0; k < num_fps + 1; k++)
                extend_edges_length(k) = ave_edge_length;
        }

        Eigen::VectorXd fps_pos_init = dlo_state_init.extend_fps_pos_;
        Eigen::Vector3d m1_0 = dlo_state_init.m1_0_;
        Eigen::Vector3d m1_n = dlo_state_init.m1_n_;
        double theta_n_init = dlo_state_init.getThetaN();
        double gravity_ref_height = gravityRefHeight(fps_pos_init, dlo_length_);

        // --------------------- optimization : begin ------------------------
        int iter = 0;
        double last_cost = 1e10;
        Eigen::VectorXd extend_fps_pos_opt = fps_pos_init;

        while (ros::ok())
        {
            // 计算当前 cost
            double bend_energy = derm_eigen::bendEnergy(extend_fps_pos_opt, dlo_length, model_params.bend_stiffness);
            double twist_energy = derm_eigen::twistEnergy(extend_fps_pos_opt, dlo_length, m1_0, m1_n, model_params.twist_stiffness, theta_n_init);
            double gravity_energy = derm_eigen::gravityEnergy(extend_fps_pos_opt, dlo_length, model_params.density, gravity_ref_height);
            double cost = bend_energy + twist_energy + gravity_energy;

            Eigen::VectorXd bend_energy_gradient = model_params.bend_stiffness * derm_eigen::bendEnergyGradientWithUnitStiffness(
                                                                                     extend_fps_pos_opt, dlo_length);

            Eigen::VectorXd twist_energy_gradient = model_params.twist_stiffness * derm_eigen::twistEnergyGradientWithUnitStiffness(
                                                                                       extend_fps_pos_opt, dlo_length, m1_0, m1_n, theta_n_init);

            Eigen::VectorXd gravity_energy_gradient = model_params.density * derm_eigen::gravityEnergyGradientWithUnitStiffness(
                                                                                 extend_fps_pos_opt, dlo_length);

            Eigen::VectorXd total_energy_gradient = bend_energy_gradient + twist_energy_gradient + gravity_energy_gradient;

            double step_size = optimize_options.step_size;
            double max_grad = optimize_options.max_grad;

            // restrict the max norm of the gradient
            for (size_t j = 0; j < total_energy_gradient.size(); j++)
            {
                total_energy_gradient(j) = std::max(-max_grad, std::min(total_energy_gradient(j), max_grad));
            }

            // gradient descent，update the variable (unconstrained)
            extend_fps_pos_opt -= step_size * total_energy_gradient;
            // project the variable onto the inextensible constraint manifold
            extend_fps_pos_opt = derm_eigen::inextensibleProjection(extend_fps_pos_opt, extend_edges_length);

            // termination conditions
            if (std::abs(cost - last_cost) / cost < optimize_options.function_tolerance)
            {
                ROS_DEBUG("DLO::optimizeShapeDermGD(): reach function_tolerance.");
                break;
            }
            if (total_energy_gradient.norm() < optimize_options.gradient_tolerance)
            {
                ROS_DEBUG("DLO::optimizeShapeDermGD(): reach gradient_tolerance.");
                break;
            }
            if (iter > optimize_options.max_num_iterations)
            {
                ROS_DEBUG("DLO::optimizeShapeDermGD(): reach max_num_iterations.");
                break;
            }

            last_cost = cost;
            iter++;
        }
        // --------------------- optimization: end ------------------------

        // optimized DLO state
        Eigen::VectorXd fps_pos_opt = extend_fps_pos_opt.block(3, 0, 3 * num_fps, 1); // use the vertex (i = 1 to m) as the feature point
        DLOState dlo_state_opt(fps_pos_opt, dlo_state_init.end_quat_0_, dlo_state_init.end_quat_1_);
        dlo_state_opt.updateDependentInfo(dlo_state_init.getThetaN());

        return dlo_state_opt;
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDermCeres(
        const DLOState &dlo_state_init,
        const OptimizeOptions optimize_options)
    {
        return optimizeShapeDermCeres(dlo_state_init, derm_params_, optimize_options);
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDermCeres(
        const DLOState &dlo_state_init,
        const DLOEnergyModelParams &model_params,
        const OptimizeOptions optimize_options)
    {
        using namespace derm_ceres;

        int num_fps = num_fps_; // DER vertex id is 0 ~ (num_fps + 1)
        double dlo_length, ave_edge_length;
        Eigen::VectorXd extend_edges_length(num_fps + 1);
        if (optimize_options.use_origin_edges_length)
        {
            dlo_length = dlo_state_init.getLength();
            ave_edge_length = dlo_length / (num_fps_ - 1.0);
            extend_edges_length = dlo_state_init.getExtendEdgesLength();
        }
        else
        {
            dlo_length = dlo_length_;
            ave_edge_length = dlo_length / (num_fps_ - 1.0);
            for (int k = 0; k < num_fps + 1; k++)
                extend_edges_length(k) = ave_edge_length;
        }

        Eigen::VectorXd fps_pos_init = dlo_state_init.extend_fps_pos_;
        Eigen::Vector3d m1_0 = dlo_state_init.m1_0_;
        Eigen::Vector3d m1_n = dlo_state_init.m1_n_;
        double theta_n_init = dlo_state_init.getThetaN();

        double gravity_ref_height = gravityRefHeight(fps_pos_init, dlo_length);

        // --------------------- optimization based on ceres: begin ------------------------

        // Eigen::VectorXd to double[][]
        double fps_pos[num_fps + 2][3];
        for (int k = 0; k < num_fps + 2; k++)
        {
            for (int j = 0; j < 3; j++)
                fps_pos[k][j] = fps_pos_init[3 * k + j];
        }

        ceres::Problem problem;

        // bend energy cost
        for (int k = 1; k <= num_fps; k++)
        {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<BendEnergyCost, 1, 3, 3, 3>(new BendEnergyCost(ave_edge_length, model_params.bend_stiffness)),
                nullptr, fps_pos[k - 1], fps_pos[k], fps_pos[k + 1]);
        }

        // stretch energy cost
        for (int k = 1; k <= num_fps - 1; k++)
        {
            double this_edge_length;
            if (optimize_options.use_origin_edges_length)
            {
                this_edge_length = (DLO::getFpPos(fps_pos_init, k + 1) - DLO::getFpPos(fps_pos_init, k)).norm();
            }
            else
            {
                this_edge_length = ave_edge_length;
            }
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<StretchEnergyCost, 1, 3, 3>(new StretchEnergyCost(this_edge_length, model_params.stretch_stiffness)),
                nullptr, fps_pos[k], fps_pos[k + 1]);
        }

        // cost for fixing some DLO feature points
        std::vector<int> fixed_vertex_idx{0, 1, num_fps, num_fps + 1};
        for (int k : fixed_vertex_idx)
        {
            for (int j = 0; j < 3; j++)
            {
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<FixedPointCost, 1, 3>(new FixedPointCost(
                        fps_pos_init.block<3, 1>(3 * k, 0), j, ceres_fixed_fps_cost_weight_)),
                    nullptr, fps_pos[k]);
            }
        }

        // twist energy cost
        ceres::DynamicAutoDiffCostFunction<TwistEnergyCost, 50> *twist_energy_cost_function = // DynamicAuto... http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres27DynamicAutoDiffCostFunctionE 和 https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/robot_pose_mle.cc
            new ceres::DynamicAutoDiffCostFunction<TwistEnergyCost, 50>(
                new TwistEnergyCost(num_fps, ave_edge_length, model_params.twist_stiffness, m1_0, m1_n, theta_n_init));
        std::vector<double *> parameter_blocks;
        parameter_blocks.reserve(num_fps + 2);
        for (int k = 0; k < num_fps + 2; k++)
        {
            parameter_blocks.push_back((fps_pos[k]));
            twist_energy_cost_function->AddParameterBlock(3);
        }
        twist_energy_cost_function->SetNumResiduals(1);
        problem.AddResidualBlock(twist_energy_cost_function, nullptr, parameter_blocks);

        // gravity energy cost
        if (model_params.density >= 1e-3)
        { // if density < 1e-3, there seems to be some numerical problems (do not know why).
            for (int k = 2; k <= num_fps - 1; k++)
            {
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<GravityEnergyCost, 1, 3>(new GravityEnergyCost(
                        ave_edge_length, model_params.density, gravity_ref_height)),
                    nullptr, fps_pos[k]);
            }
        }

        // Run the solver!
        ceres::Solver::Options ceres_options;

        // set the parameters of the ceres solver
        if (optimize_options.minimizer_type == "line_search")
            ceres_options.minimizer_type = ceres::LINE_SEARCH;
        else if (optimize_options.minimizer_type == "trust_region")
            ceres_options.minimizer_type = ceres::TRUST_REGION;
        else
            ROS_ERROR_STREAM("Ceres doesn't support minimizer_type: " << optimize_options.minimizer_type);

        if (optimize_options.line_search_direction_type == "BFGS")
            ceres_options.line_search_direction_type = ceres::BFGS;
        else if (optimize_options.line_search_direction_type == "LBFGS")
            ceres_options.line_search_direction_type = ceres::LBFGS;
        else if (optimize_options.line_search_direction_type == "NCG")
            ceres_options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
        else if (optimize_options.line_search_direction_type == "STEEPEST_DESCENT")
            ceres_options.line_search_direction_type = ceres::STEEPEST_DESCENT;
        else
            ROS_ERROR_STREAM("Ceres doesn't support line_search_direction_type: " << optimize_options.line_search_direction_type);

        ceres_options.linear_solver_type = ceres::DENSE_QR;
        ceres_options.max_num_line_search_step_size_iterations = optimize_options.max_num_line_search_step_size_iterations;

        ceres_options.function_tolerance = optimize_options.function_tolerance;
        ceres_options.gradient_tolerance = optimize_options.gradient_tolerance;
        ceres_options.max_num_iterations = optimize_options.max_num_iterations;
        ceres_options.minimizer_progress_to_stdout = optimize_options.minimizer_progress_to_stdout;

        ceres::Solver::Summary summary;

        // solving
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        ceres::Solve(ceres_options, &problem, &summary);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_solving = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        if (optimize_options.print_log)
        {
            std::cout << summary.FullReport() << ", Time: " << time_used_solving.count() << "s" << std::endl;
        }

        // double[][] to Eigen::VectorXd
        Eigen::VectorXd extend_fps_pos_opt(3 * (num_fps + 2));
        for (int k = 0; k < num_fps + 2; k++)
        {
            for (int j = 0; j < 3; j++)
                extend_fps_pos_opt[3 * k + j] = fps_pos[k][j];
        }

        // --------------------- optimization based on ceres: end ------------------------

        // optimized DLO state
        Eigen::VectorXd fps_pos_opt = extend_fps_pos_opt.block(3, 0, 3 * num_fps, 1); // use the vertex (i = 1 to m) as the feature point
        DLOState dlo_state_opt(fps_pos_opt, dlo_state_init.end_quat_0_, dlo_state_init.end_quat_1_);
        dlo_state_opt.updateDependentInfo(dlo_state_init.getThetaN());

        // ROS_DEBUG_STREAM("DLO::optimizeShapeDermCeres(): dlo length after projection: " << dlo_state_opt.getLength());

        return dlo_state_opt;
    }

    // ------------------------------------------------------------
    DLOState DLO::optimizeShapeDermIpopt(
        const DLOState &dlo_state_init,
        const DLOEnergyModelParams &model_params,
        const OptimizeOptions optimize_options)
    {
        using namespace derm_ipopt;

        int num_fps = num_fps_;
        double dlo_length;
        Eigen::VectorXd extend_edges_length(num_fps + 1);

        if (optimize_options.use_origin_edges_length)
        {
            dlo_length = dlo_state_init.getLength();
            extend_edges_length = dlo_state_init.getExtendEdgesLength();
        }
        else
        {
            dlo_length = dlo_length_;
            double ave_edge_length = dlo_length / (num_fps - 1.0);
            for (int k = 0; k < num_fps + 1; k++)
                extend_edges_length(k) = ave_edge_length;
        }

        Eigen::VectorXd fps_pos_init = dlo_state_init.extend_fps_pos_;
        Eigen::Vector3d m1_0 = dlo_state_init.m1_0_;
        Eigen::Vector3d m1_n = dlo_state_init.m1_n_;
        double theta_n_init = dlo_state_init.getThetaN();
        double gravity_ref_height = gravityRefHeight(fps_pos_init, dlo_length_);

        // --------------------- optimization based on ipopt: begin ------------------------
        // variable
        std::shared_ptr<FpsPosVariables> var_fps_pos = std::make_shared<FpsPosVariables>(num_fps);
        var_fps_pos->SetParams(fps_pos_init);
        var_fps_pos->SetVariables(fps_pos_init);

        // constraint
        std::shared_ptr<InextensibleConstraint> constraint_inextensible =
            std::make_shared<InextensibleConstraint>(num_fps);
        constraint_inextensible->SetParams(extend_edges_length);

        // cost
        std::shared_ptr<BendEnergyCost> cost_bend_energy = std::make_shared<BendEnergyCost>();
        cost_bend_energy->SetParams(dlo_length, model_params.bend_stiffness);

        std::shared_ptr<TwistEnergyCost> cost_twist_energy = std::make_shared<TwistEnergyCost>();
        cost_twist_energy->SetParams(dlo_length, m1_0, m1_n, model_params.twist_stiffness, theta_n_init);

        std::shared_ptr<GravityEnergyCost> cost_gravity_energy = std::make_shared<GravityEnergyCost>();
        cost_gravity_energy->SetParams(dlo_length, model_params.density, gravity_ref_height);

        // problem
        ifopt::Problem nlp;
        nlp.AddVariableSet(var_fps_pos);
        nlp.AddConstraintSet(constraint_inextensible);
        nlp.AddCostSet(cost_bend_energy);
        nlp.AddCostSet(cost_twist_energy);
        nlp.AddCostSet(cost_gravity_energy);

        IpoptSolver ipopt;
        ipopt.SetOption("linear_solver", "ma27");
        ipopt.SetOption("jacobian_approximation", optimize_options.ipopt_jacobian_approximation);
        ipopt.SetOption("acceptable_obj_change_tol", optimize_options.function_tolerance);
        ipopt.SetOption("acceptable_iter", optimize_options.ipopt_acceptable_iter);
        ipopt.SetOption("acceptable_tol", optimize_options.ipopt_acceptable_tol);
        ipopt.SetOption("tol", optimize_options.ipopt_tol);

        if (optimize_options.print_log)
        {
            nlp.PrintCurrent();
            ipopt.SetOption("print_level", 5);
        }
        else
        {
            ipopt.SetOption("print_level", 0);
        }

        ipopt.Solve(nlp);
        Eigen::VectorXd extend_fps_pos_opt = var_fps_pos->GetValues();

        // --------------------- optimization based on ipopt: end ------------------------

        // optimized DLO state
        Eigen::VectorXd fps_pos_opt = extend_fps_pos_opt.block(3, 0, 3 * num_fps, 1); // use the vertex (i = 1 to m) as the feature point
        DLOState dlo_state_opt(fps_pos_opt, dlo_state_init.end_quat_0_, dlo_state_init.end_quat_1_);
        dlo_state_opt.updateDependentInfo(dlo_state_init.getThetaN());

        return dlo_state_opt;
    }

    // ------------------------------------------------------------
    void DLO::deformationEnergy(
        const DLOState &dlo_state,
        const DLOEnergyModelParams &params,
        const double ref_height,
        double *stretch_energy,
        double *bend_energy,
        double *twist_energy,
        double *gravity_energy,
        double *total_energy)
    {
        double dlo_length = dlo_length_;
        Eigen::VectorXd extend_fps_pos = dlo_state.extend_fps_pos_;
        Eigen::Vector3d m1_0 = dlo_state.m1_0_;
        Eigen::Vector3d m1_n = dlo_state.m1_n_;
        double theta_n_init = dlo_state.getThetaN();

        if (stretch_energy)
            *stretch_energy = derm_eigen::stretchEnergy(extend_fps_pos, dlo_length, params.stretch_stiffness); // using average edge length as original length
        if (bend_energy)
            *bend_energy = derm_eigen::bendEnergy(extend_fps_pos, dlo_length, params.bend_stiffness);
        if (twist_energy)
            *twist_energy = derm_eigen::twistEnergy(extend_fps_pos, dlo_length, m1_0, m1_n, params.twist_stiffness, theta_n_init);
        if (gravity_energy)
            *gravity_energy = derm_eigen::gravityEnergy(extend_fps_pos, dlo_length, params.density, ref_height);

        if (total_energy)
        {
            *total_energy = *stretch_energy + *bend_energy + *twist_energy + *gravity_energy;
        }
    }

    // ------------------------------------------------------------
    // first sample the middle vertex, then sample the first and last vertex, then use linear interpolation to get other vertices
    VecEigenVec3 DLO::sampleCoarseShapeType0(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        double scale_lb = 0.99;
        double dlo_length = dlo_length_;
        double segment_length = dlo_length / (num_fps_ - 1.0);
        int num_vertex = num_fps_ + 2;
        int dim = dim_;

        // check the sampling range
        if (dim == 2 && range_lb(2) != range_ub(2))
        {
            ROS_ERROR("Error: range_lb(2) should be equal to range_ub(2) when env_dim is 2.");
            return {};
        }
        for (int j = 0; j < 3; j++)
        {
            if (range_lb(j) > range_ub(j))
            {
                ROS_ERROR("Error: range_lb should <= range_ub.");
                return {};
            }
        }

        int mid_fp_idx = int(num_vertex / 2);

        // sample the middle vertex
        Eigen::Vector3d mid_fp_pos;
        for (int j = 0; j < 3; j++)
        {
            mid_fp_pos[j] = Utils::getRandomDouble(range_lb[j], range_ub[j]);
        }

        // sample the first vertex by sampling on a sphere surface centered at the middle vertex
        Eigen::Vector3d first_fp_pos;
        while (ros::ok())
        {
            double r_0 = (segment_length * mid_fp_idx) * Utils::getRandomDouble(scale_lb, 1.0);
            double phi_0 = Utils::getRandomDouble(0.0, 2 * M_PI);
            double theta_0;
            if (dim == 2)
                theta_0 = M_PI / 2.0;
            else if (dim == 3)
                theta_0 = Utils::getRandomDouble(0.0, M_PI);
            else
                std::cerr << "invalid env dim, should be either 2 or 3." << std::endl;
            first_fp_pos[0] = mid_fp_pos[0] + r_0 * std::sin(theta_0) * std::cos(phi_0);
            first_fp_pos[1] = mid_fp_pos[1] + r_0 * std::sin(theta_0) * std::sin(phi_0);
            first_fp_pos[2] = mid_fp_pos[2] + r_0 * std::cos(theta_0);

            bool in_range = true;
            if (in_range)
                break;
        }

        // sample the last vertex by sampling on a sphere surface centered at the middle vertex
        Eigen::Vector3d last_fp_pos;
        while (ros::ok())
        {
            double r_1 = (segment_length * (num_vertex - mid_fp_idx - 1)) * Utils::getRandomDouble(scale_lb, 1.0);
            double phi_1 = Utils::getRandomDouble(0.0, 2 * M_PI);
            double theta_1;
            if (dim == 2)
                theta_1 = M_PI / 2.0;
            else if (dim == 3)
                theta_1 = Utils::getRandomDouble(0.0, M_PI);
            last_fp_pos[0] = mid_fp_pos[0] + r_1 * std::sin(theta_1) * std::cos(phi_1);
            last_fp_pos[1] = mid_fp_pos[1] + r_1 * std::sin(theta_1) * std::sin(phi_1);
            last_fp_pos[2] = mid_fp_pos[2] + r_1 * std::cos(theta_1);

            bool in_range = true;
            if (in_range)
                break;
        }

        VecEigenVec3 fps_pos(num_vertex);
        fps_pos[0] = first_fp_pos;
        fps_pos[mid_fp_idx] = mid_fp_pos;
        fps_pos[num_vertex - 1] = last_fp_pos;

        // linear interpolation to get other vertices
        for (int k = 1; k < mid_fp_idx; k++)
        {
            Eigen::Vector3d fp_pos = first_fp_pos + (mid_fp_pos - first_fp_pos) / mid_fp_idx * k;
            fps_pos[k] = fp_pos;
        }
        for (int k = 1; k < num_vertex - mid_fp_idx - 1; k++)
        {
            Eigen::Vector3d fp_pos = mid_fp_pos +
                                     (last_fp_pos - mid_fp_pos) / (num_vertex - mid_fp_idx - 1) * k;
            fps_pos[mid_fp_idx + k] = fp_pos;
        }

        return fps_pos;
    }

    // ------------------------------------------------------------
    // restrict the z-axis of the sampled DLO ends to be vertical, on the basis of type 0
    VecEigenVec3 DLO::sampleCoarseShapeType1(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        double dlo_length = dlo_length_;
        double segment_length = dlo_length / (num_fps_ - 1.0);
        int num_vertex = num_fps_ + 2;

        VecEigenVec3 fps_pos = sampleCoarseShapeType0(range_lb, range_ub, seed);

        // restrict the edges at the DLO ends to be horizontal
        fps_pos[0][2] = fps_pos[1][2];
        fps_pos[num_vertex - 1][2] = fps_pos[num_vertex - 2][2];

        // rectify the lengths of the edges at the DLO ends
        fps_pos[0] = fps_pos[1] + segment_length * (fps_pos[0] - fps_pos[1]).normalized();
        fps_pos[num_vertex - 1] = fps_pos[num_vertex - 2] + segment_length * (fps_pos[num_vertex - 1] - fps_pos[num_vertex - 2]).normalized();

        return fps_pos;
    }

    // ------------------------------------------------------------
    // add some random noises, on the basis of type 0
    VecEigenVec3 DLO::sampleCoarseShapeType2(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        double dlo_length = dlo_length_;
        double segment_length = dlo_length / (num_fps_ - 1.0);
        int num_vertex = num_fps_ + 2;

        VecEigenVec3 fps_pos = sampleCoarseShapeType0(range_lb, range_ub, seed);

        for (int i = 0; i < num_vertex; i++)
        {
            fps_pos[i] += Eigen::Vector3d::Random().normalized() * 0.5 * segment_length;
        }

        return fps_pos;
    }

    // ------------------------------------------------------------
    DLOState DLO::sampleCoarseStateType0(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        Eigen::VectorXd fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(
            sampleCoarseShapeType0(range_lb, range_ub, seed));

        Eigen::Vector4d left_quat, right_quat;
        calcDLOEndsOrientation(fps_pos, left_quat, right_quat);

        DLOState dlo_state(fps_pos.block(3, 0, 3 * num_fps_, 1), left_quat, right_quat);
        dlo_state.updateDependentInfo();

        return dlo_state;
    }

    // ------------------------------------------------------------
    DLOState DLO::sampleCoarseStateType1(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        Eigen::VectorXd fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(
            sampleCoarseShapeType1(range_lb, range_ub, seed));

        Eigen::Vector4d left_quat, right_quat;
        calcDLOEndsOrientation(fps_pos, left_quat, right_quat);

        DLOState dlo_state(fps_pos.block(3, 0, 3 * num_fps_, 1), left_quat, right_quat);
        dlo_state.updateDependentInfo();

        return dlo_state;
    }

    // ------------------------------------------------------------
    DLOState DLO::sampleCoarseStateType2(
        const Eigen::Vector3d &range_lb,
        const Eigen::Vector3d &range_ub,
        int seed)
    {
        Eigen::VectorXd fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(
            sampleCoarseShapeType2(range_lb, range_ub, seed));

        Eigen::Vector4d left_quat, right_quat;
        calcDLOEndsOrientation(fps_pos, left_quat, right_quat);

        DLOState dlo_state(fps_pos.block(3, 0, 3 * num_fps_, 1), left_quat, right_quat);
        dlo_state.updateDependentInfo();

        return dlo_state;
    }

    // ------------------------------------------------------------
    DLOState DLO::interpolateCoarseShape(
        const Eigen::Vector3d &first,
        const Eigen::Vector3d &mid,
        const Eigen::Vector3d &last)
    {
        auto fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(interpolateCoarseShape(num_fps_ + 2, first, mid, last));

        // restrict the edges at the DLO ends to be horizontal
        dloEndsHorizontalModification(fps_pos);

        Eigen::Vector4d left_quat, right_quat;
        calcDLOEndsOrientation(fps_pos, left_quat, right_quat);

        DLOState dlo_state(fps_pos.block(3, 0, 3 * num_fps_, 1), left_quat, right_quat);
        dlo_state.updateDependentInfo();

        return dlo_state;
    }

    // ------------------------------------------------------------
    VecEigenVec3 DLO::interpolateCoarseShape(
        int n_vertex,
        const Eigen::Vector3d &first,
        const Eigen::Vector3d &mid,
        const Eigen::Vector3d &last)
    {
        int mid_fps = int(n_vertex / 2);
        VecEigenVec3 fps_pos(n_vertex);

        fps_pos[0] = first;
        fps_pos[mid_fps] = mid;
        fps_pos[n_vertex - 1] = last;

        // linear interpolation
        for (int k = 1; k < mid_fps; k++)
        {
            fps_pos[k] = fps_pos[0] + (fps_pos[mid_fps] - fps_pos[0]) / mid_fps * k;
        }
        for (int k = 1; k < n_vertex - mid_fps - 1; k++)
        {
            fps_pos[mid_fps + k] = fps_pos[mid_fps] +
                                   (fps_pos[n_vertex - 1] - fps_pos[mid_fps]) / (n_vertex - mid_fps - 1) * k;
        }

        return fps_pos;
    }

    // ------------------------------------------------------------
    Eigen::Vector4d DLO::calcEndOrientationFromTangent(
        Eigen::Vector3d y_axis)
    {
        // the given y_axis is the tangent vector
        // assuming the x_axis is horiontal, compute the corresponding x_axis and z_axis
        y_axis.normalize();
        Eigen::Vector3d vertical_axis(0.0, 0.0, 1.0);
        Eigen::Vector3d x_axis = y_axis.cross(vertical_axis);
        x_axis.normalize();
        Eigen::Vector3d z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        rotation_matrix.block<3, 1>(0, 0) = x_axis;
        rotation_matrix.block<3, 1>(0, 1) = y_axis;
        rotation_matrix.block<3, 1>(0, 2) = z_axis;

        Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(rotation_matrix);

        // return quaternion in [x, y, z, w]
        return quat.coeffs();
    }

    // ------------------------------------------------------------
    // compute the orientations of the DLO ends based on the tangent vector and the assumption of horizontal x-axis
    void DLO::calcDLOEndsOrientation(
        const Eigen::VectorXd &fps_pos,
        Eigen::Vector4d &left_quat,
        Eigen::Vector4d &right_quat)
    {
        int num_fps = fps_pos.size() / 3;

        // left end
        Eigen::Vector3d y_axis_0 = fps_pos.block<3, 1>(3, 0) - fps_pos.block<3, 1>(0, 0);
        left_quat = calcEndOrientationFromTangent(y_axis_0);

        // right end
        Eigen::Vector3d y_axis_1 = fps_pos.block<3, 1>(3 * (num_fps - 1), 0) - fps_pos.block<3, 1>(3 * (num_fps - 2), 0);
        right_quat = calcEndOrientationFromTangent(y_axis_1);
    }

    // ------------------------------------------------------------
    // restrict the edges at the DLO ends to be horizontal
    void DLO::dloEndsHorizontalModification(
        Eigen::VectorXd &fps_pos)
    {
        assert(fps_pos.size() % 3 == 0);

        int num_fps = fps_pos.size() / 3;

        fps_pos.block<3, 1>(3 * 0, 0)[2] = fps_pos.block<3, 1>(3 * 1, 0)[2];
        fps_pos.block<3, 1>(3 * (num_fps - 1), 0)[2] = fps_pos.block<3, 1>(3 * (num_fps - 2), 0)[2];
    }

    // ------------------------------------------------------------
    Eigen::Vector3d DLO::getCentroid(
        const DLOState &dlo_state)
    {
        int num_fps = dlo_state.fps_pos_.size() / 3;

        Eigen::Vector3d sum_fp_pos = Eigen::Vector3d::Zero();
        for (size_t k = 0; k < num_fps; k++)
        {
            sum_fp_pos = sum_fp_pos + dlo_state.getFpPos(k);
        }

        return sum_fp_pos / num_fps;
    }

    // ------------------------------------------------------------
    void DLO::moveToCentroid(
        DLOState &dlo_state,
        const Eigen::Vector3d &new_centroid)
    {
        int num_fps = dlo_state.fps_pos_.size() / 3;

        Eigen::Vector3d current_centroid = getCentroid(dlo_state);

        Eigen::Vector3d diff_centroid = new_centroid - current_centroid;

        for (size_t k = 0; k < num_fps; k++)
        {
            Eigen::Vector3d new_fp_pos = dlo_state.getFpPos(k) + diff_centroid;
            dlo_state.setFpPos(k, new_fp_pos);
        }
        dlo_state.updateDependentInfo(/*theta_n_init*/ dlo_state.getThetaN());
    }

    // ------------------------------------------------------------
    DLOState DLO::moveToCentroidNewState(
        const DLOState &dlo_state_const,
        const Eigen::Vector3d &new_centroid)
    {
        DLOState dlo_state = dlo_state_const;
        moveToCentroid(dlo_state, new_centroid);
        return dlo_state;
    }

    // ------------------------------------------------------------
    double DLO::calcInterpolationRatio(
        const DLOState &from_state,
        const DLOState &to_state,
        const double max_translate_step,
        const double max_rotate_step)
    {
        if (dlo_interpolation_type_ == 1)
        {
            return calcInterpolationRatioType1(from_state, to_state, max_translate_step, max_rotate_step);
        }
        else if (dlo_interpolation_type_ == 2)
        {
            return calcInterpolationRatioType2(from_state, to_state, max_translate_step, max_rotate_step);
        }
        else if (dlo_interpolation_type_ == 3)
        {
            return calcInterpolationRatioType3(from_state, to_state, max_translate_step);
        }
        else
        {
            ROS_ERROR_STREAM("DLO::calcInterpolationRatio(): doesn't support interpolation type " << dlo_interpolation_type_);
        }
        return -1;
    }

    // ------------------------------------------------------------
    DLOState DLO::interpolate(
        const DLOState &from_state,
        const DLOState &to_state,
        const double t)
    {
        if (dlo_interpolation_type_ == 1)
        {
            return interpolateType1(from_state, to_state, t);
        }
        else if (dlo_interpolation_type_ == 2)
        {
            return interpolateType2(from_state, to_state, t);
        }
        else if (dlo_interpolation_type_ == 3)
        {
            return interpolateType3(from_state, to_state, t);
        }
        else
        {
            ROS_ERROR_STREAM("DLO::interpolate(): doesn't support interpolation type " << dlo_interpolation_type_);
        }
        return DLOState();
    }

    // ------------------------------------------------------------
    double DLO::calcInterpolationRatioType1(
        const DLOState &from_state,
        const DLOState &to_state,
        const double max_translate_step,
        const double max_rotate_step)
    {
        // tranlsation: linear interpolation of the centroid
        Eigen::Vector3d from_centroid = getCentroid(from_state);
        Eigen::Vector3d to_centroid = getCentroid(to_state);
        double distance = (to_centroid - from_centroid).norm();
        double t_translate = std::min(1.0, max_translate_step / distance);

        // rotation: slerp of the edge orientations

        // compute the largest angular difference of edges
        double max_angle = -1e10;
        for (size_t k = 0; k < from_state.material_frames_.size(); k++)
        {
            double angle = Eigen::Quaterniond(from_state.material_frames_[k]).angularDistance(Eigen::Quaterniond(to_state.material_frames_[k]));
            if (angle > max_angle)
                max_angle = angle;
        }
        // compute the slerp ratio
        double t_rotate = std::min(1.0, max_rotate_step / max_angle);

        // compute the final interpolation t considering both the lerp of the centroid and the slerp of the edge orientations
        double t = std::min(t_translate, t_rotate);

        return t;
    }

    // ------------------------------------------------------------
    DLOState DLO::interpolateType1(
        const DLOState &from_state,
        const DLOState &to_state,
        const double t)
    {
        ROS_ERROR_COND(from_state.fps_pos_.size() / 3 != num_fps_, "The size of fps is wrong.");
        ROS_ERROR_COND(to_state.fps_pos_.size() / 3 != num_fps_, "The size of fps is wrong.");
        ROS_ERROR_COND(from_state.material_frames_.size() != num_fps_ + 1, "The size of material_frames_ is wrong.");
        ROS_ERROR_COND(to_state.material_frames_.size() != num_fps_ + 1, "The size of material_frames_ is wrong.");

        // translation: lerp of the centroid
        Eigen::Vector3d from_centroid = getCentroid(from_state);
        Eigen::Vector3d to_centroid = getCentroid(to_state);
        Eigen::Vector3d new_centroid = from_centroid * (1 - t) + to_centroid * t;

        // rotation: slerp of the edge orientations
        VecEigenVector4d slerp_material_frames(from_state.material_frames_.size());
        for (size_t k = 0; k < from_state.material_frames_.size(); k++)
        {
            slerp_material_frames[k] = Eigen::Quaterniond(from_state.material_frames_[k]).slerp(t, Eigen::Quaterniond(to_state.material_frames_[k])).coeffs().transpose();
        }

        // get new fps_pos based on frames and edge lengths (the centroid will be rectified later)
        VecEigenVec3 slerp_extend_fps_pos(num_fps_ + 2);
        slerp_extend_fps_pos[0] = Eigen::Vector3d::Zero(); // 随便设置一个初值
        double edge_length = dlo_length_ / (num_fps_ - 1.0);

        for (size_t k = 0; k <= num_fps_; k++)
        {
            Eigen::Matrix3d slerp_material_frame_rotmat = Eigen::Quaterniond(slerp_material_frames[k]).normalized().toRotationMatrix();
            slerp_extend_fps_pos[k + 1] = slerp_extend_fps_pos[k] + edge_length * slerp_material_frame_rotmat.block<3, 1>(0, 1);
        }

        // compute the new DLOState
        Eigen::VectorXd fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(slerp_extend_fps_pos).block(3, 0, 3 * num_fps_, 1);
        Eigen::Vector4d end_quat_0 = slerp_material_frames[0];
        Eigen::Vector4d end_quat_1 = slerp_material_frames[slerp_material_frames.size() - 1];
        DLOState dlo_state(fps_pos, end_quat_0, end_quat_1);
        dlo_state.updateDependentInfo(/*theta_n_init*/ from_state.getThetaN());

        // rectify the centroid
        moveToCentroid(dlo_state, new_centroid);

        return dlo_state;
    }

    /** ------------------------------------------------------------
     * @brief interpolation of the ICRA version
     */
    double DLO::calcInterpolationRatioType2(
        const DLOState &from_state,
        const DLOState &to_state,
        const double max_translate_step,
        const double max_rotate_step)
    {
        // translation: lerp of the centroid
        Eigen::Vector3d from_centroid = getCentroid(from_state);
        Eigen::Vector3d to_centroid = getCentroid(to_state);
        double distance = (to_centroid - from_centroid).norm();
        double t_translate = std::min(1.0, max_translate_step / distance);

        // slerp of fps_pos, using the centroid as the slerp center
        DLOState from_state_zero_centroid = DLO::moveToCentroidNewState(from_state, Eigen::Vector3d::Zero());
        DLOState to_state_zero_centroid = DLO::moveToCentroidNewState(to_state, Eigen::Vector3d::Zero());

        std::vector<double> fps_angles(num_fps_);
        for (int k = 0; k < num_fps_; k++)
        {
            Eigen::Vector3d fp_pos_0 = from_state_zero_centroid.getFpPos(k);
            Eigen::Vector3d fp_pos_1 = to_state_zero_centroid.getFpPos(k);
            fps_angles[k] = Utils::twoVecAngle(fp_pos_0, fp_pos_1);
        }
        double max_fp_angle = *std::max_element(fps_angles.begin(), fps_angles.end());
        double t_rotate = std::min(1.0, max_rotate_step / max_fp_angle);

        double t = std::min(t_translate, t_rotate);
        return t;
    }

    DLOState DLO::interpolateType2(
        const DLOState &from_state,
        const DLOState &to_state,
        const double t)
    {
        // translation: lerp of the centroid
        Eigen::Vector3d from_centroid = getCentroid(from_state);
        Eigen::Vector3d to_centroid = getCentroid(to_state);

        // slerp of fps_pos, using the centroid as the slerp center
        DLOState from_state_zero_centroid = DLO::moveToCentroidNewState(from_state, Eigen::Vector3d::Zero());
        DLOState to_state_zero_centroid = DLO::moveToCentroidNewState(to_state, Eigen::Vector3d::Zero());

        // compute new centroid
        Eigen::Vector3d new_centroid = from_centroid * (1 - t) + to_centroid * t;

        // compute new shape (the centroid will be rectified later)
        VecEigenVec3 slerp_fps_pos(num_fps_);
        for (int k = 0; k < num_fps_; k++)
        {
            Eigen::Vector3d fp_pos_0 = from_state_zero_centroid.getFpPos(k);
            Eigen::Vector3d fp_pos_1 = to_state_zero_centroid.getFpPos(k);
            double theta = Utils::twoVecAngle(fp_pos_0, fp_pos_1);
            // slerp
            slerp_fps_pos[k] = std::sin((1 - t) * theta) / std::sin(theta) * fp_pos_0 + std::sin(t * theta) / std::sin(theta) * fp_pos_1;
        }

        // 新的 DLOState
        Eigen::VectorXd fps_pos = Utils::stdVecEigenVec3ToEigenVectorXd(slerp_fps_pos);
        Eigen::Vector4d end_quat_0 = Eigen::Quaterniond(from_state.end_quat_0_).slerp(t, Eigen::Quaterniond(to_state.end_quat_0_)).coeffs().transpose();
        Eigen::Vector4d end_quat_1 = Eigen::Quaterniond(from_state.end_quat_1_).slerp(t, Eigen::Quaterniond(to_state.end_quat_1_)).coeffs().transpose();
        DLOState dlo_state(fps_pos, end_quat_0, end_quat_1);

        // rectify the centroid
        moveToCentroid(dlo_state, new_centroid);

        dlo_state.updateDependentInfo(/*theta_n_init*/ from_state.getThetaN());

        return dlo_state;
    }

    /** ------------------------------------------------------------
     * @brief Purely linear interpolation
     */
    double DLO::calcInterpolationRatioType3(
        const DLOState &from_state,
        const DLOState &to_state,
        const double max_translate_step)
    {
        double max_dist = 0.0;
        for (size_t k = 0; k < num_fps_; k++)
        {
            double dist = (from_state.getFpPos(k) - to_state.getFpPos(k)).norm();
            max_dist = std::max(max_dist, dist);
        }
        double t = std::min(1.0, max_translate_step / max_dist);
        return t;
    }

    DLOState DLO::interpolateType3(
        const DLOState &from_state,
        const DLOState &to_state,
        const double t)
    {
        Eigen::VectorXd fps_pos = from_state.fps_pos_ * (1 - t) + to_state.fps_pos_ * t;
        Eigen::Vector4d end_quat_0 = Eigen::Quaterniond(from_state.end_quat_0_).slerp(t, Eigen::Quaterniond(to_state.end_quat_0_)).coeffs().transpose();
        Eigen::Vector4d end_quat_1 = Eigen::Quaterniond(from_state.end_quat_1_).slerp(t, Eigen::Quaterniond(to_state.end_quat_1_)).coeffs().transpose();
        DLOState dlo_state(fps_pos, end_quat_0, end_quat_1);
        dlo_state.updateDependentInfo(/*theta_n_init*/ from_state.getThetaN());

        return dlo_state;
    }

    // ------------------------------------------------------------
    double DLO::distanceBetweenTwoDLOStates(
        const DLOState &state_1,
        const DLOState &state_2,
        const std::string dist_type)
    {
        double distance = -1;
        if (dist_type == "L2_norm")
        {
            distance = (state_1.fps_pos_ - state_2.fps_pos_).norm();
        }
        else if (dist_type == "max_norm")
        {
            distance = (state_1.fps_pos_ - state_2.fps_pos_).lpNorm<Eigen::Infinity>();
        }
        else if (dist_type == "max_distance")
        {
            double max_d = 0.0;
            for (size_t k = 0; k < state_1.num_fps_; k++)
            {
                double d = (state_1.getFpPos(k) - state_2.getFpPos(k)).norm();
                max_d = std::max(d, max_d);
            }
            distance = max_d;
        }
        else
        {
            ROS_ERROR_STREAM("DLO::distanceBetweenTwoDLOStates() doesn't support dist_type: " << dist_type);
        }

        return distance;
    }

    /** ------------------------------------------------------------
     */
    double DLO::aveErrorBetweenOriginalShapesAndOptimizedShapes(
        const std::vector<DLOState> &dlo_states,
        const DLOEnergyModelParams &model_params,
        const OptimizeOptions &optimize_options)
    {
        double sum_error = 0.0;
        for (auto &dlo_state : dlo_states)
        {
            DLOState dlo_state_opt = optimizeShapeDermCeres(dlo_state, model_params, optimize_options);
            // DLOState dlo_state_opt = optimizeShapeDermGD(dlo_state, model_params, optimize_options);

            double error = (dlo_state_opt.fps_pos_ - dlo_state.fps_pos_).norm();
            sum_error += error;
        }

        return sum_error / dlo_states.size();
    }

    /** ------------------------------------------------------------
     * @brief
     * @param vars: log_twist_stiffness, log_density
     */
    double DLO::costFunctionOfPSO(
        const Eigen::VectorXd &vars,
        const DLOEnergyModelParams &init_model_params,
        const std::vector<DLOState> &dlo_states)
    {
        DLOEnergyModelParams params = init_model_params;
        params.twist_stiffness = std::pow(10, vars(0));
        params.density = std::pow(10, vars(1));

        OptimizeOptions optimize_options;
        optimize_options.use_origin_edges_length = true; // use the observed DLO edge lengths
        optimize_options.function_tolerance = 1e-8;

        return aveErrorBetweenOriginalShapesAndOptimizedShapes(dlo_states, params, optimize_options);
    }

    /** ------------------------------------------------------------
     * @brief optimization variables: log_twist_stiffness & log_density
     */
    void DLO::optimizeParametersByPSO(
        const std::vector<DLOState> &dlo_states,
        DLOEnergyModelParams &params)
    {
        int dim_x = 2;

        Eigen::VectorXd x_lb(dim_x);
        x_lb << params.log_twist_stiffness_range(0), params.log_density_range(0);
        Eigen::VectorXd x_ub(dim_x);
        x_ub << params.log_twist_stiffness_range(1), params.log_density_range(1);

        // parameters for PSO
        int n_particles = identification_pso_n_particles_;
        int n_iter = identification_pso_n_iter_;
        double omega = identification_pso_omega_;
        double c_1 = identification_pso_c_1_;
        double c_2 = identification_pso_c_2_;

        // initialize the particles
        std::vector<ParticlePSO> particles(n_particles);
        Eigen::VectorXd x_global_best;
        double f_global_best = 1e10; // minimize

        for (auto &particle : particles)
        {
            if (!ros::ok())
                break;

            particle.x = Eigen::VectorXd::Zero(dim_x);
            particle.v = Eigen::VectorXd::Zero(dim_x);

            for (size_t j = 0; j < dim_x; j++)
                particle.x(j) = Utils::getRandomDouble(x_lb(j), x_ub(j));
            particle.f = costFunctionOfPSO(particle.x, params, dlo_states);

            particle.x_local_best = particle.x;
            particle.f_local_best = particle.f;

            if (particle.f_local_best < f_global_best)
            {
                x_global_best = particle.x_local_best;
                f_global_best = particle.f_local_best;
            }
        }

        // iteration for optimization
        for (size_t iter = 0; iter < n_iter; iter++)
        {
            for (auto &particle : particles)
            {
                if (!ros::ok())
                    break;

                particle.v = omega * particle.v + c_1 * Utils::getRandomDouble() * (particle.x_local_best - particle.x) + c_2 * Utils::getRandomDouble() * (x_global_best - particle.x);
                particle.x += particle.v;

                // prevent x from being out of the range
                for (size_t j = 0; j < dim_x; j++)
                {
                    particle.x(j) = std::max(x_lb(j), std::min(particle.x(j), x_ub(j)));
                }
                particle.f = costFunctionOfPSO(particle.x, params, dlo_states);

                // update the local optimal of each particle
                if (particle.f < particle.f_local_best)
                {
                    particle.f_local_best = particle.f;
                    particle.x_local_best = particle.x;
                }
                // update the global optimal
                if (particle.f_local_best < f_global_best)
                {
                    x_global_best = particle.x_local_best;
                    f_global_best = particle.f_local_best;
                }
            }
            ROS_DEBUG_STREAM("DLO::optimizeParametersByPSO(): iter: " << iter << ", x_global_best: " << x_global_best.transpose()
                                                                      << ", f_global_best: " << f_global_best);
        }

        params.twist_stiffness = std::pow(10, x_global_best(0));
        params.density = std::pow(10, x_global_best(1));
    }

} // end namespace