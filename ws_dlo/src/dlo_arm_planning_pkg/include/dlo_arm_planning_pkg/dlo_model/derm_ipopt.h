#ifndef DLO_ARM_PLANNING_DERM_IPOPT_H
#define DLO_ARM_PLANNING_DERM_IPOPT_H

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_model/derm_eigen.h"

namespace dlo_arm_planning_pkg
{
    namespace derm_ipopt
    {

        using namespace ifopt;

        // ----------------------------------------------------------
        class FpsPosVariables : public VariableSet
        {
        public:
            FpsPosVariables(const int &num_fps) : VariableSet(3 * (num_fps + 2), "var_fps_pos")
            {
                fps_pos_ = Eigen::VectorXd::Zero(3 * (num_fps + 2));
            }

            // ---------------------------
            void SetParams(const Eigen::VectorXd &fps_pos_init)
            {
                fps_pos_init_ = fps_pos_init;
            }

            // ---------------------------
            void SetVariables(const Eigen::VectorXd &x) override
            {
                fps_pos_ = x;
            }

            // ---------------------------
            Eigen::VectorXd GetValues() const override
            {
                return fps_pos_;
            }

            // ---------------------------
            VecBound GetBounds() const override
            {
                VecBound bounds(GetRows());
                // constrain the four points at the two ends
                for (int j = 0; j < GetRows(); j++)
                {
                    if (j < 3 * 2 || j >= GetRows() - 3 * 2)
                    {
                        bounds.at(j) = Bounds(fps_pos_init_(j), fps_pos_init_(j));
                    }
                    else
                    {
                        bounds.at(j) = NoBound;
                    }
                }
                return bounds;
            }

        private:
            Eigen::VectorXd fps_pos_;      // variables
            Eigen::VectorXd fps_pos_init_; // parameters
        }; // end class

        // ----------------------------------------------------------
        class InextensibleConstraint : public ConstraintSet
        {
        public:
            InextensibleConstraint(const int &num_fps) : ConstraintSet(num_fps + 1, "constraint_inextensible")
            {
                num_fps_ = num_fps;
            }

            // ---------------------------
            void SetParams(const Eigen::VectorXd &edges_length)
            {
                edges_length_ = edges_length;
            }

            // ---------------------------
            Eigen::VectorXd GetValues() const override
            {
                Eigen::VectorXd fps_pos = GetVariables()->GetComponent("var_fps_pos")->GetValues();
                return derm_eigen::inextensibleConstraints(fps_pos, edges_length_);
            }

            // ---------------------------
            VecBound GetBounds() const override
            {
                VecBound bounds(GetRows());
                bounds.at(0) = NoBound;
                bounds.at(num_fps_) = NoBound;
                for (int j = 1; j < num_fps_; j++)
                    bounds.at(j) = BoundZero;
                return bounds;
            }

            void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override
            {
                if (var_set == "var_fps_pos")
                {
                    Eigen::VectorXd fps_pos = GetVariables()->GetComponent("var_fps_pos")->GetValues();
                    Eigen::MatrixXd jacobian = derm_eigen::inextensibleConstraintsJacobian(fps_pos, edges_length_);

                    for (int i = 0; i < jacobian.rows(); i++)
                    {
                        for (int j = 0; j < jacobian.cols(); j++)
                        {
                            if (jacobian(i, j) != 0)
                                jac_block.coeffRef(i, j) = jacobian(i, j);
                        }
                    }
                }
            }

        private:
            Eigen::VectorXd edges_length_;
            int num_fps_;
        };

        // ----------------------------------------------------------
        class BendEnergyCost : public CostTerm
        {
        public:
            BendEnergyCost() : CostTerm("cost_bend_energy") {}

            // ---------------------------
            void SetParams(
                double dlo_length,
                double bend_stiffness)
            {
                dlo_length_ = dlo_length;
                bend_stiffness_ = bend_stiffness;
            }

            // ---------------------------
            double GetCost() const override
            {
                Eigen::VectorXd fps_pos = GetVariables()->GetComponent(
                                                            "var_fps_pos")
                                              ->GetValues();

                double bend_energy = derm_eigen::bendEnergy(
                    fps_pos, dlo_length_, bend_stiffness_);

                return bend_energy;
            }

            // ---------------------------
            void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override
            {
                if (var_set == "var_fps_pos")
                {
                    Eigen::VectorXd fps_pos = GetVariables()->GetComponent("var_fps_pos")->GetValues();

                    Eigen::VectorXd gradient = derm_eigen::bendEnergyGradientWithUnitStiffness(
                        fps_pos, dlo_length_);
                    gradient *= bend_stiffness_;

                    for (int j = 0; j < gradient.size(); j++)
                        jac_block.coeffRef(0, j) = gradient(j);
                }
            }

        private:
            // parameters
            double dlo_length_;
            double bend_stiffness_;
        };

        // ----------------------------------------------------------
        class TwistEnergyCost : public CostTerm
        {
        public:
            TwistEnergyCost() : CostTerm("cost_twist_energy") {}

            // ---------------------------
            void SetParams(
                double dlo_length,
                Eigen::Vector3d m1_0,
                Eigen::Vector3d m1_n,
                double twist_stiffness,
                double theta_n_init = 0.0)
            {
                dlo_length_ = dlo_length;
                m1_0_ = m1_0;
                m1_n_ = m1_n;
                twist_stiffness_ = twist_stiffness;
                theta_n_init_ = theta_n_init;
            }

            // ---------------------------
            double GetCost() const override
            {
                Eigen::VectorXd fps_pos = GetVariables()->GetComponent(
                                                            "var_fps_pos")
                                              ->GetValues();

                double twist_energy = derm_eigen::twistEnergy(
                    fps_pos, dlo_length_, m1_0_, m1_n_, twist_stiffness_, theta_n_init_);

                return twist_energy;
            }

            // ---------------------------
            void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override
            {
                if (var_set == "var_fps_pos")
                {
                    Eigen::VectorXd fps_pos = GetVariables()->GetComponent("var_fps_pos")->GetValues();

                    Eigen::VectorXd gradient = derm_eigen::twistEnergyGradientWithUnitStiffness(
                        fps_pos, dlo_length_, m1_0_, m1_n_, theta_n_init_);
                    gradient *= twist_stiffness_;

                    for (int j = 0; j < gradient.size(); j++)
                        jac_block.coeffRef(0, j) = gradient(j);
                }
            }

        private:
            double dlo_length_;
            Eigen::Vector3d m1_0_, m1_n_;
            double theta_n_init_;
            double twist_stiffness_;
        };

        // ----------------------------------------------------------
        class GravityEnergyCost : public CostTerm
        {
        public:
            GravityEnergyCost() : CostTerm("cost_gravity_energy") {}

            // ---------------------------
            void SetParams(
                double dlo_length, double density, double ref_height)
            {
                dlo_length_ = dlo_length;
                density_ = density;
                ref_height_ = ref_height;
            }

            // ---------------------------
            double GetCost() const override
            {
                Eigen::VectorXd fps_pos = GetVariables()->GetComponent(
                                                            "var_fps_pos")
                                              ->GetValues();

                double gravity_energy = derm_eigen::gravityEnergy(
                    fps_pos, dlo_length_, density_, ref_height_);

                return gravity_energy;
            }

            // ---------------------------
            void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override
            {
                if (var_set == "var_fps_pos")
                {
                    Eigen::VectorXd fps_pos = GetVariables()->GetComponent("var_fps_pos")->GetValues();

                    Eigen::VectorXd gradient = derm_eigen::gravityEnergyGradientWithUnitStiffness(
                        fps_pos, dlo_length_);
                    gradient *= density_;

                    for (int j = 0; j < gradient.size(); j++)
                        jac_block.coeffRef(0, j) = gradient(j);
                }
            }

        private:
            double density_;
            double dlo_length_;
            double ref_height_;
        };

    } // end namespace derm_ipopt
} // end namespace dlo_arm_planning_pkg

#endif