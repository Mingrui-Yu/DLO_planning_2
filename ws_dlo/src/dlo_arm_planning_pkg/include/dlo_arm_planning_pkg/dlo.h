#ifndef DLO_ARM_PLANNING_DLO_H
#define DLO_ARM_PLANNING_DLO_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ceres/ceres.h>

namespace dlo_arm_planning_pkg
{

    // ----------------------------------------------
    struct DLOEnergyModelParams
    {
        double stretch_stiffness = 1e3;
        double bend_stiffness = 1e-2;
        double twist_stiffness = 1e-2;
        double density = 1e-4; // (density < 1e-3) is equivalent to (density = 0) to avoid numerical problems

        Eigen::Vector2d log_twist_stiffness_range = Eigen::Vector2d(-4, 0); // lower_bound, upper_bound
        Eigen::Vector2d log_density_range = Eigen::Vector2d(-4, 1);
    };

    // ----------------------------------------------
    struct OptimizeOptions
    {
        std::string solver = "Ceres"; // "Ceres" or "Ipopt" or "GD"

        // shared
        bool use_origin_edges_length = false;
        double function_tolerance = 1e-5; // default: 1e-8 (经测试，高于1e-3则效果不好)
        double max_num_iterations = 10000;
        bool print_log = false;

        // only for ceres
        double gradient_tolerance = 1e-3;
        double max_num_line_search_step_size_iterations = 100; // default: 20
        std::string minimizer_type = "line_search";            // "line_search" or "trust_region"
        std::string line_search_direction_type = "BFGS";
        bool minimizer_progress_to_stdout = false;

        // only for ipopt
        int ipopt_acceptable_iter = 5;
        double ipopt_acceptable_tol = 1e10;
        double ipopt_tol = 1e-6;
        std::string ipopt_jacobian_approximation = "exact"; // "exact" or "finite-difference-values"

        // only for mannually implemented gradient decent
        double step_size = 1e-3;
        double max_grad = 10;
    };

    // ----------------------------------------------
    class DLO
    {
    public:
        typedef std::shared_ptr<DLO> Ptr;

        DLO() {}

        DLO(int num_fps, double length, int dim);

        DLO(const ros::NodeHandle &nh);

        DLO(const ros::NodeHandle &nh, double length);

        void loadParams();

        void setLength(double length);

        void setDermParams(
            const DLOEnergyModelParams &params);

        void deformationEnergy(
            const DLOState &dlo_state,
            const DLOEnergyModelParams &params,
            const double ref_height = 0.0,
            double *stretch_energy = nullptr,
            double *bend_energy = nullptr,
            double *twist_energy = nullptr,
            double *gravity_energy = nullptr,
            double *total_energy = nullptr);

        double gravityRefHeight(
            const Eigen::VectorXd &fps_pos,
            const double &dlo_length);

        DLOState optimizeShapeDerm(
            const DLOState &dlo_state_init,
            const OptimizeOptions optimize_options = OptimizeOptions());

        DLOState optimizeShapeDerm(
            const DLOState &dlo_state_init,
            const DLOEnergyModelParams &model_params,
            const OptimizeOptions optimize_options = OptimizeOptions());

        DLOState optimizeShapeDermCeres(
            const DLOState &dlo_state_init,
            const OptimizeOptions optimize_options = OptimizeOptions());

        DLOState optimizeShapeDermCeres(
            const DLOState &dlo_state_init,
            const DLOEnergyModelParams &model_params,
            const OptimizeOptions optimize_options = OptimizeOptions());

        DLOState optimizeShapeDermIpopt(
            const DLOState &dlo_state_init,
            const DLOEnergyModelParams &model_params,
            const OptimizeOptions optimize_options);

        DLOState optimizeShapeDermGD(
            const DLOState &dlo_state_init,
            const DLOEnergyModelParams &model_params,
            const OptimizeOptions &optimize_options = OptimizeOptions());

        DLOState interpolateCoarseShape(
            const Eigen::Vector3d &first,
            const Eigen::Vector3d &mid,
            const Eigen::Vector3d &last);

        VecEigenVec3 interpolateCoarseShape(
            int num_fps,
            const Eigen::Vector3d &first,
            const Eigen::Vector3d &mid,
            const Eigen::Vector3d &last);

        DLOState sampleCoarseStateType0(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        DLOState sampleCoarseStateType1(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        DLOState sampleCoarseStateType2(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        VecEigenVec3 sampleCoarseShapeType0(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        VecEigenVec3 sampleCoarseShapeType1(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        VecEigenVec3 sampleCoarseShapeType2(
            const Eigen::Vector3d &range_lb,
            const Eigen::Vector3d &range_ub,
            int seed = -1);

        Eigen::Vector4d calcEndOrientationFromTangent(
            Eigen::Vector3d y_axis);

        void calcDLOEndsOrientation(
            const Eigen::VectorXd &fps_pos,
            Eigen::Vector4d &left_quat,
            Eigen::Vector4d &right_quat);

        void dloEndsHorizontalModification(
            Eigen::VectorXd &fps_pos);

        Eigen::Vector3d getCentroid(
            const DLOState &dlo_state);

        void moveToCentroid(
            DLOState &dlo_state,
            const Eigen::Vector3d &new_centroid);

        DLOState moveToCentroidNewState(
            const DLOState &dlo_state_const,
            const Eigen::Vector3d &new_centroid);

        double calcInterpolationRatio(
            const DLOState &from_state,
            const DLOState &to_state,
            const double max_translate_step,
            const double max_rotate_step);

        DLOState interpolate(
            const DLOState &from_state,
            const DLOState &to_state,
            const double t);

        double calcInterpolationRatioType1(
            const DLOState &from_state,
            const DLOState &to_state,
            const double max_translate_step,
            const double max_rotate_step);

        DLOState interpolateType1(
            const DLOState &from_state,
            const DLOState &to_state,
            const double t);

        double calcInterpolationRatioType2(
            const DLOState &from_state,
            const DLOState &to_state,
            const double max_translate_step,
            const double max_rotate_step);

        DLOState interpolateType2(
            const DLOState &from_state,
            const DLOState &to_state,
            const double t);

        double calcInterpolationRatioType3(
            const DLOState &from_state,
            const DLOState &to_state,
            const double max_translate_step);

        DLOState interpolateType3(
            const DLOState &from_state,
            const DLOState &to_state,
            const double t);

        static double distanceBetweenTwoDLOStates(
            const DLOState &state_1,
            const DLOState &state_2,
            const std::string dist_type = "L2_norm");

        double aveErrorBetweenOriginalShapesAndOptimizedShapes(
            const std::vector<DLOState> &dlo_states,
            const DLOEnergyModelParams &model_params,
            const OptimizeOptions &optimize_options);

        double costFunctionOfPSO(
            const Eigen::VectorXd &vars,
            const DLOEnergyModelParams &init_model_params,
            const std::vector<DLOState> &dlo_states);

        void optimizeParametersByPSO(
            const std::vector<DLOState> &dlo_states,
            DLOEnergyModelParams &params);

    public:
        int num_fps_;            // number of feature points (n)
        int num_point_per_edge_; // only for collision

        double dlo_length_;
        int dim_;
        int dlo_interpolation_type_;

        ros::NodeHandle nh_;

        // parameters for DLO
        double ceres_fixed_fps_cost_weight_ = 1e5; // fixed (add fixed fps constraints to the cost function)

        DLOEnergyModelParams derm_params_;

        int identification_pso_n_particles_;
        int identification_pso_n_iter_;
        int identification_pso_omega_;
        int identification_pso_c_1_;
        int identification_pso_c_2_;

        // -------------------------------
    public:
        static Eigen::Vector3d getFpPos(
            const Eigen::VectorXd &fps_pos, int index)
        {
            return fps_pos.block<3, 1>(3 * index, 0);
        }

        static Eigen::VectorXd getEdgesLength(
            const Eigen::VectorXd &fps_pos)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::VectorXd origin_edges_length(num_fps + 1);
            for (int k = 0; k < num_fps + 1; k++)
            {
                origin_edges_length(k) =
                    (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).norm();
            }

            return origin_edges_length;
        }

    }; // class

    // ---------------------------------------------------------
    struct ParticlePSO
    {
    public:
        Eigen::VectorXd x;
        Eigen::VectorXd v;
        double f;

        double f_local_best;
        Eigen::VectorXd x_local_best;
    };

} // end namespace

#endif