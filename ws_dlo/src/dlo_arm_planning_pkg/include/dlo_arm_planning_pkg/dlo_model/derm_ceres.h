#ifndef DLO_ARM_PLANNING_DERM_CERES_H
#define DLO_ARM_PLANNING_DERM_CERES_H

#include "dlo_arm_planning_pkg/common_include.h"

#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace dlo_arm_planning_pkg
{
    namespace derm_ceres
    {

        // ----------------------------
        template <typename T>
        inline T Dist(const T point_0[3], const T point_1[3])
        {
            return ceres::sqrt((point_0[0] - point_1[0]) * (point_0[0] - point_1[0]) + (point_0[1] - point_1[1]) * (point_0[1] - point_1[1]) + (point_0[2] - point_1[2]) * (point_0[2] - point_1[2]));
        }

        // ----------------------------
        template <typename T>
        inline T Norm(const T x[3])
        {
            T square = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

            if (square < T(1e-15)) // the norm of a zero vector cannot be calcuated by |x|/x
            {
                return T(0.0);
            }
            else
            {
                return ceres::sqrt(square);
            }
        }

        // ----------------------------
        template <typename T>
        inline void CeresNormalize(const T x[3], T x_normalized[3])
        {
            T norm = Norm(x);

            if (norm < T(1e-15)) // deal with zero vectors
            {
                for (int j = 0; j < 3; j++)
                    x_normalized[j] = T(0.0);
            }
            else
            {
                for (int j = 0; j < 3; j++)
                    x_normalized[j] = x[j] / norm;
            }
        }

        // ----------------------------
        // edge from point_0 to point_1
        template <typename T>
        inline void Edge(const T point_0[3], const T point_1[3], T edge[3])
        {
            edge[0] = point_1[0] - point_0[0];
            edge[1] = point_1[1] - point_0[1];
            edge[2] = point_1[2] - point_0[2];
        }

        // ----------------------------
        // unit tangent vector from point_0 to point_1
        template <typename T>
        inline void Tangent(const T point_0[3], const T point_1[3], T tangent[3])
        {
            T edge[3];
            Edge(point_0, point_1, edge);
            CeresNormalize(edge, tangent);
        }

        // ----------------------------
        template <typename T>
        inline void getFpPos(T const *const *fps_pos, const int index, T fp_pos[3])
        {
            for (int j = 0; j < 3; j++)
                fp_pos[j] = fps_pos[index][j];
        }

        // ----------------------------
        // curvature binormal at a vertex
        template <typename T>
        inline void Kb(
            const T edge_0[3],
            const T edge_1[3],
            T kb[3])
        {
            T cross_product[3];
            ceres::CrossProduct(edge_0, edge_1, cross_product);
            T dot_product = ceres::DotProduct(edge_0, edge_1);

            T denominator = Norm(edge_0) * Norm(edge_1) + dot_product;

            kb[0] = (2.0 * cross_product[0]) / denominator;
            kb[1] = (2.0 * cross_product[1]) / denominator;
            kb[2] = (2.0 * cross_product[2]) / denominator;
        }

        // ----------------------------
        template <typename T>
        inline void LastBishopFrame(
            T const *const *fps_pos,
            const int num_fps,
            const T u_0[3],
            T u_n[3])
        {
            T u_k_minus_1[3], u_k[3];
            for (int j = 0; j < 3; j++)
                u_k_minus_1[j] = u_0[j];

            for (int k = 1; k <= num_fps; k++)
            {
                /** tangent_0 is t_{k-1}，tangent_1 is t_{k}
                 * fp_0 is x_{k-1}，fp_1 is x_{k}，fp_2 is x_{k+1}
                 * u_k_minus_1 is u_{k-1}, u_k is u_{k}
                 * angle_axis is P_{k}
                 */
                T fp_0[3], fp_1[3], fp_2[3];
                getFpPos(fps_pos, k - 1, fp_0);
                getFpPos(fps_pos, k, fp_1);
                getFpPos(fps_pos, k + 1, fp_2);

                T tangent_0[3], tangent_1[3];
                Tangent(fp_0, fp_1, tangent_0);
                Tangent(fp_1, fp_2, tangent_1);

                // axis-angle
                T cross_product[3], normalized_axis[3], angle_axis[3];
                ceres::CrossProduct(tangent_0, tangent_1, cross_product);
                CeresNormalize(cross_product, normalized_axis);
                T angle = ceres::atan2(Norm(cross_product), ceres::DotProduct(tangent_0, tangent_1));
                for (int j = 0; j < 3; j++)
                {
                    angle_axis[j] = normalized_axis[j] * angle;
                }
                // compute u_{k}
                ceres::AngleAxisRotatePoint(angle_axis, u_k_minus_1, u_k);

                for (int j = 0; j < 3; j++)
                    u_k_minus_1[j] = u_k[j];
            }

            // return the last u
            for (int j = 0; j < 3; j++)
                u_n[j] = u_k[j];
        }

        // ----------------------------
        // compute the theta angle from bishop frame (u) to material frame (m1)
        template <typename T>
        inline T ThetaBetweenBishopAndMaterial(
            const T t[3],
            const T u[3],
            const T m1[3],
            const T theta_init)
        {
            T u_cross_m1[3];
            ceres::CrossProduct(u, m1, u_cross_m1);

            T theta;
            if (ceres::DotProduct(u_cross_m1, t) >= T(0.0))
            {
                theta = ceres::atan2(Norm(u_cross_m1), ceres::DotProduct(u, m1));
            }
            else
            {
                theta = -ceres::atan2(Norm(u_cross_m1), ceres::DotProduct(u, m1));
            }

            // add nc * 2pi to the theta, getting an equivalent angle whose value is closest to theta_init
            T two_pi(2.0 * M_PI);
            T nc = ceres::floor((theta_init - theta) / two_pi + T(1.0 / 2.0));
            theta += nc * two_pi;

            return theta;
        }

        // -----------------------------------------------------------
        struct StretchEnergyCost
        {
            StretchEnergyCost(double origin_edge_length, double stiffness)
                : origin_edge_length_(origin_edge_length), stiffness_(stiffness) {}

            template <typename T>
            bool operator()(
                const T *const fp_0,
                const T *const fp_1,
                T *residual) const
            {
                T edge_length = Dist(fp_0, fp_1);

                residual[0] = std::sqrt(stiffness_ / origin_edge_length_) *
                              (edge_length - origin_edge_length_);

                return true;
            }

        private:
            const double origin_edge_length_;
            const double stiffness_;
        };

        // -----------------------------------------------------------
        struct BendEnergyCost
        {
            BendEnergyCost(double origin_edge_length, double stiffness)
                : origin_edge_length_(origin_edge_length), stiffness_(stiffness) {}

            template <typename T>
            bool operator()(
                const T *const fp_0,
                const T *const fp_1,
                const T *const fp_2,
                T *residual) const
            {
                T edge_0[3], edge_1[3];
                Edge(fp_0, fp_1, edge_0);
                Edge(fp_1, fp_2, edge_1);

                double origin_l = origin_edge_length_ * 2.0;

                T kb[3];
                Kb(edge_0, edge_1, kb);

                residual[0] = std::sqrt(2.0 * stiffness_ / origin_l) * Norm(kb);
                return true;
            }

        private:
            const double origin_edge_length_;
            const double stiffness_;
        };

        // -----------------------------------------------------------
        struct TwistEnergyCost
        {
            TwistEnergyCost(int num_fps, double origin_edge_length,
                            double stiffness, Eigen::Vector3d m1_0, Eigen::Vector3d m1_n,
                            double theta_n_init = 0.0) : num_fps_(num_fps),
                                                         origin_edge_length_(origin_edge_length),
                                                         stiffness_(stiffness),
                                                         m1_0_(m1_0), m1_n_(m1_n),
                                                         theta_n_init_(theta_n_init)
            {
            }

            template <typename T>
            bool operator()(
                T const *const *fps_pos, // all points
                T *residual) const
            {

                T m1_0[3], u_0[3], m1_n[3], u_n[3];
                for (int j = 0; j < 3; j++)
                {
                    m1_0[j] = T(m1_0_(j)); // Eigen::Vector3d to T[3]
                    m1_n[j] = T(m1_n_(j));
                    u_0[j] = m1_0[j]; // let the bishop frame and material frame at edge_0 to be the same
                }

                // compute the u_n of bishop frame
                LastBishopFrame(fps_pos, num_fps_, u_0, u_n);

                // compute theta_0 and theta_n
                T theta_0 = T(0.0); // defaultly assume the bishop frame and material frame at edge_0 are the same，so theta_0 = 0.0

                T fp_n[3], fp_n_plus_1[3];
                getFpPos(fps_pos, num_fps_, fp_n);
                getFpPos(fps_pos, num_fps_ + 1, fp_n_plus_1);
                T tangent_n[3];
                Tangent(fp_n, fp_n_plus_1, tangent_n);
                T theta_n = ThetaBetweenBishopAndMaterial(tangent_n, u_n, m1_n, T(theta_n_init_));

                double origin_l = origin_edge_length_ * 2.0;

                // compute twist energy
                residual[0] = std::sqrt(stiffness_ * 2.0 / (num_fps_ * origin_l)) *
                              (theta_n - theta_0);

                return true;
            }

        private:
            const double origin_edge_length_;
            const int num_fps_;
            const double stiffness_;
            const Eigen::Vector3d m1_0_, m1_n_;
            const double theta_n_init_;
        };

        // -----------------------------------------------------------
        struct GravityEnergyCost
        {
            GravityEnergyCost(double origin_edge_length, double density, double ref_height) : origin_edge_length_(origin_edge_length),
                                                                                              density_(density),
                                                                                              ref_height_(ref_height) {}

            template <typename T>
            bool operator()(const T *const fp, T *residual) const
            {
                T height = fp[2] - ref_height_;
                double origin_l = origin_edge_length_ * 2.0;

                residual[0] = std::sqrt(density_ * origin_l * g_) * ceres::sqrt(ceres::fmax(1e-15, height));
                return true;
            }

        private:
            const double density_;
            const double g_ = 9.8;
            const double ref_height_;
            const double origin_edge_length_;
        };

        // -----------------------------------------------------------
        struct FixedPointCost
        {
            FixedPointCost(Eigen::Vector3d fp_init, int dim, double weight) : fp_init_(fp_init), dim_(dim), weight_(weight) {}

            template <typename T>
            bool operator()(const T *const fp, T *residual) const
            {
                residual[0] = std::sqrt(weight_) * (fp[dim_] - fp_init_[dim_]);
                return true;
            }

        private:
            const Eigen::Vector3d fp_init_;
            const int dim_;
            const double weight_;
        };

        // -----------------------------------------------------------
        static double stretchEnergy(
            const Eigen::VectorXd &fps_pos_eigen,
            const double dlo_length,
            const double stiffness,
            Eigen::VectorXd origin_edges_length = Eigen::VectorXd::Zero(0) // the original length of each edge; if not assigned, then assume the length of each edge is the average of the total length
        )
        {
            int num_fps = fps_pos_eigen.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            // Eigen::VectorXd to double[][]
            double fps_pos[num_fps + 2][3];
            for (int k = 0; k < num_fps + 2; k++)
            {
                for (int j = 0; j < 3; j++)
                    fps_pos[k][j] = fps_pos_eigen[3 * k + j];
            }

            double energy = 0.0;
            // stretch energy cost
            for (int k = 1; k <= num_fps - 1; k++)
            {

                double origin_this_edge_length;
                if (origin_edges_length.size() != 0)
                {
                    origin_this_edge_length = origin_edges_length[k];
                }
                else
                {
                    origin_this_edge_length = origin_edge_length;
                }

                StretchEnergyCost stretch_cost = StretchEnergyCost(origin_this_edge_length, stiffness);
                double cost;
                stretch_cost(fps_pos[k], fps_pos[k + 1], &cost);
                energy += 1.0 / 2.0 * cost * cost;
            }

            return energy;
        }

        // ----------------------------------------------------------
        static double bendEnergy(
            const Eigen::VectorXd &fps_pos_eigen,
            const double dlo_length,
            const double stiffness)
        {
            int num_fps = fps_pos_eigen.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            // Eigen::VectorXd to double[][]
            double fps_pos[num_fps + 2][3];
            for (int k = 0; k < num_fps + 2; k++)
            {
                for (int j = 0; j < 3; j++)
                    fps_pos[k][j] = fps_pos_eigen[3 * k + j];
            }

            double energy = 0.0;
            for (int k = 1; k <= num_fps; k++)
            {
                BendEnergyCost bend_cost = BendEnergyCost(origin_edge_length, stiffness);
                double cost;
                bend_cost(fps_pos[k - 1], fps_pos[k], fps_pos[k + 1], &cost);
                energy += 1.0 / 2.0 * cost * cost;
            }

            return energy;
        }

        // ----------------------------------------------------------
        static double twistEnergy(
            const Eigen::VectorXd &fps_pos_eigen,
            const double dlo_length,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double stiffness,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos_eigen.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            // Eigen::VectorXd to double[][]
            double fps_pos[num_fps + 2][3];
            for (int k = 0; k < num_fps + 2; k++)
            {
                for (int j = 0; j < 3; j++)
                    fps_pos[k][j] = fps_pos_eigen[3 * k + j];
            }

            // double[][] to double*[]
            double *fps_pos_star[num_fps + 2];
            for (int k = 0; k < num_fps + 2; k++)
            {
                fps_pos_star[k] = fps_pos[k];
            }

            TwistEnergyCost twist_cost = TwistEnergyCost(num_fps, origin_edge_length, stiffness, m1_0, m1_n, theta_n_init);
            double cost;
            twist_cost(fps_pos_star, &cost);
            double energy = 1.0 / 2.0 * cost * cost;

            return energy;
        }

        // ----------------------------------------------------------
        static double gravityEnergy(
            const Eigen::VectorXd &fps_pos_eigen,
            const double dlo_length,
            const double density,
            double ref_height = 0.0)
        {
            int num_fps = fps_pos_eigen.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            // Eigen::VectorXd to double[][]
            double fps_pos[num_fps + 2][3];
            for (int k = 0; k < num_fps + 2; k++)
            {
                for (int j = 0; j < 3; j++)
                    fps_pos[k][j] = fps_pos_eigen[3 * k + j];
            }

            double energy = 0.0;

            if (density > 0.0)
            {
                for (int k = 2; k <= num_fps - 1; k++)
                {
                    GravityEnergyCost gravity_energy_cost(origin_edge_length, density, ref_height);
                    double cost;
                    gravity_energy_cost(fps_pos[k], &cost);
                    energy += 1.0 / 2.0 * cost * cost;
                }
            }

            return energy;
        }

        // ----------------------------------------------------------
        static double totalEnergy(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double stretch_stiffness,
            const double bend_stiffness,
            const double twist_stiffness,
            const double density,
            const double ref_height = 0.0,
            Eigen::VectorXd origin_edges_length = Eigen::VectorXd::Zero(0))
        {
            double stretch_energy = stretchEnergy(
                fps_pos, dlo_length, stretch_stiffness, origin_edges_length);

            double bend_energy = bendEnergy(
                fps_pos, dlo_length, bend_stiffness);

            double twist_energy = twistEnergy(
                fps_pos, dlo_length, m1_0, m1_n, twist_stiffness);

            double gravity_energy = gravityEnergy(
                fps_pos, dlo_length, density, ref_height);

            double total_energy = stretch_energy + bend_energy + twist_energy + gravity_energy;

            return total_energy;
        }

        // ----------------------------------------------------------
        static bool evaluateEnergyGradientWithUnitStiffness(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            Eigen::VectorXd &stretch_gradient,
            Eigen::VectorXd &bend_gradient,
            Eigen::VectorXd &twist_gradient,
            Eigen::VectorXd &gravity_gradient,
            Eigen::VectorXd origin_edges_length = Eigen::VectorXd::Zero(0),
            bool use_finite_difference_approximation = true,
            double epsilon = 1e-8)
        {
            int n_val = fps_pos.size();

            stretch_gradient = Eigen::VectorXd::Zero(n_val);
            bend_gradient = Eigen::VectorXd::Zero(n_val);
            twist_gradient = Eigen::VectorXd::Zero(n_val);
            gravity_gradient = Eigen::VectorXd::Zero(n_val);

            if (use_finite_difference_approximation)
            {
                double step_size = epsilon;
                Eigen::VectorXd fps_pos_disturb = fps_pos;

                for (int i = 3 * 2; i < n_val - 3 * 2; i++)
                {                                    // exclude the four points at the two ends
                    fps_pos_disturb[i] += step_size; // positive disturbance

                    double stretch_energy_pos_disturb = stretchEnergy(
                        fps_pos_disturb, dlo_length, 1.0, origin_edges_length);
                    double bend_energy_pos_disturb = bendEnergy(
                        fps_pos_disturb, dlo_length, 1.0);
                    double twist_energy_pos_disturb = twistEnergy(
                        fps_pos_disturb, dlo_length, m1_0, m1_n, 1.0);
                    double gravity_energy_pos_disturb = gravityEnergy(
                        fps_pos_disturb, dlo_length, 1.0);

                    fps_pos_disturb[i] = fps_pos[i]; // reset

                    fps_pos_disturb[i] -= step_size; // negative disturbance

                    double stretch_energy_neg_disturb = stretchEnergy(
                        fps_pos_disturb, dlo_length, 1.0, origin_edges_length);
                    double bend_energy_neg_disturb = bendEnergy(
                        fps_pos_disturb, dlo_length, 1.0);
                    double twist_energy_neg_disturb = twistEnergy(
                        fps_pos_disturb, dlo_length, m1_0, m1_n, 1.0);
                    double gravity_energy_neg_disturb = gravityEnergy(
                        fps_pos_disturb, dlo_length, 1.0);

                    // numerical differentiation
                    stretch_gradient(i) = (stretch_energy_pos_disturb - stretch_energy_neg_disturb) / (2.0 * step_size);
                    bend_gradient(i) = (bend_energy_pos_disturb - bend_energy_neg_disturb) / (2.0 * step_size);
                    twist_gradient(i) = (twist_energy_pos_disturb - twist_energy_neg_disturb) / (2.0 * step_size);
                    gravity_gradient(i) = (gravity_energy_pos_disturb - gravity_energy_neg_disturb) / (2.0 * step_size);

                    fps_pos_disturb[i] = fps_pos[i]; // reset for next iteration
                }

                return true;
            }
            else
            {
                ROS_ERROR("No exact gradient function has been specified.");
                return false;
            }
        }

    } // end namespace derm_ceres
} // end namespace dlo_arm_planning_pkg

#endif