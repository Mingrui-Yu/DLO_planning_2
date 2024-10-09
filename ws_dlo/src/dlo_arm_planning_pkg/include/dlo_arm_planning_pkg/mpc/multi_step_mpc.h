#ifndef DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_H
#define DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/jacobian_model.h"
#include "dlo_arm_planning_pkg/mpc/mpc_base.h"

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include "dlo_arm_planning_pkg/mpc/multi_step_mpc_ifopt/variables.h"
#include "dlo_arm_planning_pkg/mpc/multi_step_mpc_ifopt/constraints.h"
#include "dlo_arm_planning_pkg/mpc/multi_step_mpc_ifopt/costs.h"

namespace dlo_arm_planning_pkg
{

    using namespace ifopt;
    using namespace multi_step_mpc_ifopt;

    // ----------------------------------------------------------
    class MultiStepMPC : public MPCBase
    {
    public:
        typedef std::shared_ptr<MultiStepMPC> Ptr;

        MultiStepMPC(
            const JacobianModel::Ptr &jaco_model);

        MultiStepMPC(
            const Scene::Ptr &scene,
            const JacobianModel::Ptr &jaco_model,
            const int n_step);

        void initializeComponents(
            const int &n_step);

        void initializeNlpProblem(const bool b_print = false);

        void initializeIpopt();

        bool solve(
            const Scene::Ptr &scene,
            const MPCRequest &req,
            MPCResponse &res);

    public:
        std::string ALGORITHM_NAME = "MultiStepMPC";

        Scene::Ptr scene_;
        JacobianModel::Ptr jaco_model_;

        // variables
        std::shared_ptr<ArmJointVelVariables> arm_0_joint_vel_var_, arm_1_joint_vel_var_;
        std::shared_ptr<ArmJointPosVariables> arm_0_joint_pos_var_, arm_1_joint_pos_var_;
        std::shared_ptr<DualArmCriticalPointsVariables> dual_arm_critial_points_var_;
        std::shared_ptr<DLOFpsPosVariables> dlo_fps_pos_var_;
        std::shared_ptr<DLOEdgePointsPosVariables> dlo_edge_points_var_;
        std::shared_ptr<DLOFpsVelVariables> dlo_fps_vel_var_;
        // constraints
        std::shared_ptr<DLOTransitionConstraint> dlo_transition_constraint_;
        std::shared_ptr<DLOJacobianConstraint> dlo_jacobian_constraint_;
        std::shared_ptr<DLOEdgePointsConstraint> dlo_edge_points_constraint_;
        std::shared_ptr<ArmTransitionConstraint> arm_0_transition_constraint_, arm_1_transition_constraint_;
        std::shared_ptr<DualArmCriticalPointsFKConstraint> dual_arm_critical_points_fk_constraint_;
        std::shared_ptr<ArmJointPosStartConstraint> arm_0_start_constraint_, arm_1_start_constraint_;
        std::shared_ptr<DLOStartConstraint> dlo_start_constraint_;
        std::shared_ptr<DualArmWorldDistanceConstraint> dual_arm_world_dist_constraint_;
        std::shared_ptr<DLOWorldDistanceConstraint> dlo_world_dist_constraint_;
        std::shared_ptr<DLOEdgeWorldDistanceConstraint> dlo_edge_world_dist_constraint_;
        std::shared_ptr<DLOOverstretchConstraint> dlo_overstretch_constraint_;
        // cost
        std::shared_ptr<DLOFpsPosCost> dlo_fps_pos_cost_;
        std::shared_ptr<ArmJointPosCost> arm_0_joint_pos_cost_, arm_1_joint_pos_cost_;
        std::shared_ptr<ArmJointVelCost> arm_0_joint_vel_cost_, arm_1_joint_vel_cost_;
        std::shared_ptr<ArmJointAcceCost> arm_0_joint_acce_cost_, arm_1_joint_acce_cost_;

        // nlp problem
        std::shared_ptr<ifopt::Problem> nlp_;

        // ipopt solver
        std::shared_ptr<IpoptSolver> ipopt_;
    };

} // end namespace dlo_arm_planning_pkg

#endif