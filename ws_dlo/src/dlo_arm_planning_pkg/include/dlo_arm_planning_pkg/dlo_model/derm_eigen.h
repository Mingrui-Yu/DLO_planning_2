#ifndef DLO_ARM_PLANNING_DERM_EIGEN_H
#define DLO_ARM_PLANNING_DERM_EIGEN_H

#include "dlo_arm_planning_pkg/common_include.h"

namespace dlo_arm_planning_pkg
{

    class derm_eigen
    {

    public:
        // ----------------------------------------------------------
        static Eigen::Vector3d getFpPos(const Eigen::VectorXd &fps_pos, int index)
        {
            return fps_pos.block<3, 1>(3 * index, 0);
        }

        // ----------------------------------------------------------
        static Eigen::Vector3d Kb(
            const Eigen::Vector3d &edge_0,
            const Eigen::Vector3d &edge_1)
        {
            return (2.0 * edge_0.cross(edge_1)) /
                   (edge_0.norm() * edge_1.norm() + edge_0.dot(edge_1));
        }

        // ----------------------------------------------------------
        static Eigen::Vector3d Kb(
            const Eigen::VectorXd &fps_pos,
            int k)
        {
            Eigen::Vector3d edge_0 = getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1);
            Eigen::Vector3d edge_1 = getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k);

            return Kb(edge_0, edge_1);
        }

        // ----------------------------------------------------------
        static Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d &t)
        {
            Eigen::Matrix3d t_hat;
            t_hat << 0, -t(2), t(1),
                t(2), 0, -t(0),
                -t(1), t(0), 0;
            return t_hat;
        }

        // ----------------------------------------------------------
        static Eigen::Matrix3d d_kb_d_last_x(
            const Eigen::VectorXd &fps_pos,
            int k)
        {
            Eigen::Vector3d edge_0 = getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1);
            Eigen::Vector3d edge_1 = getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k);

            return (2 * skewSymmetricMatrix(edge_1) + Kb(fps_pos, k) * (edge_1.norm() * (edge_0.normalized() + edge_1.normalized())).transpose()) / (edge_0.norm() * edge_1.norm() + edge_0.dot(edge_1));
        }

        static Eigen::Matrix3d d_kb_d_next_x(
            const Eigen::VectorXd &fps_pos,
            int k)
        {
            Eigen::Vector3d edge_0 = getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1);
            Eigen::Vector3d edge_1 = getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k);

            return (2 * skewSymmetricMatrix(edge_0) - Kb(fps_pos, k) * (edge_0.norm() * (edge_0.normalized() + edge_1.normalized())).transpose()) / (edge_0.norm() * edge_1.norm() + edge_0.dot(edge_1));
        }

        static Eigen::Matrix3d d_kb_d_x(
            const Eigen::VectorXd &fps_pos,
            int k)
        {
            return -(d_kb_d_last_x(fps_pos, k) + d_kb_d_next_x(fps_pos, k));
        }

        // ----------------------------------------------------------
        static VecEigenVec3 bishopFrame(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &u_0)
        {

            int num_fps = fps_pos.size() / 3 - 2;
            VecEigenVec3 u_all(num_fps + 1); // each edge has one u

            u_all[0] = u_0;

            for (int k = 1; k <= num_fps; k++)
            {
                // tangent_0 is t_{k-1}，tangent_1 is t_{k}
                Eigen::Vector3d tangent_0 = (getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1)).normalized();
                Eigen::Vector3d tangent_1 = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).normalized();

                Eigen::Vector3d t0_cross_t1 = tangent_0.cross(tangent_1);
                Eigen::Vector3d axis = t0_cross_t1.normalized();
                double angle = std::atan2(t0_cross_t1.norm(), tangent_0.dot(tangent_1));

                Eigen::AngleAxisd rot_vec(angle, axis);

                u_all[k] = rot_vec * u_all[k - 1];
            }

            return u_all;
        }

        // ----------------------------------------------------------
        static double thetaBetweenBishopAndMaterial(
            const Eigen::Vector3d &t,
            const Eigen::Vector3d &u,
            const Eigen::Vector3d &m1,
            const double angle_init)
        {
            Eigen::Vector3d u_cross_m1 = u.cross(m1);

            double angle = std::atan2(u_cross_m1.norm(), u.dot(m1));

            if (t.dot(u_cross_m1) < 0.0)
            {
                angle = -angle;
            }

            // add nc * 2pi to the angle, getting an equivalent angle whose value is closest to angle_init
            double two_pi = 2.0 * M_PI;
            int nc = int(std::floor((angle_init - angle) / two_pi + 1.0 / 2.0));
            angle += nc * two_pi;

            return angle;
        }

        // ----------------------------------------------------------
        static double stretchEnergy(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const double stiffness,
            Eigen::VectorXd origin_edges_length = Eigen::VectorXd::Zero(0) // the original length of each edge; if not assigned, then assume the length of each edge is the average of the total length
        )
        {
            int num_fps = fps_pos.size() / 3 - 2;

            double energy = 0.0;
            for (int k = 1; k <= num_fps - 1; k++)
            {
                double edge_length = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).norm();

                if (origin_edges_length.size() == 0)
                { // if no assigned the original length of each edge
                    double origin_edge_length = dlo_length / (num_fps - 1.0);
                    energy += std::pow(edge_length - origin_edge_length, 2) / origin_edge_length;
                }
                else
                {
                    energy += std::pow(edge_length - origin_edges_length(k), 2) / origin_edges_length(k);
                }
            }

            energy *= stiffness / 2.0;
            return energy;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd stretchEnergyGradientWithUnitStiffness(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);
            Eigen::VectorXd gradient = Eigen::VectorXd::Zero(fps_pos.size());

            for (int k = 2; k <= num_fps - 1; k++)
            {
                // edge_0 is edge_{i-1}, edge_1 is edge_{i}
                Eigen::Vector3d edge_0 = getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1);
                Eigen::Vector3d edge_1 = getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k);

                Eigen::Vector3d fp_gradient =
                    +(edge_0.norm() / origin_edge_length - 1.0) * edge_0.normalized() - (edge_1.norm() / origin_edge_length - 1.0) * edge_1.normalized();

                gradient.block<3, 1>(3 * k, 0) = fp_gradient;
            }

            return gradient;
        }

        // ----------------------------------------------------------
        static double bendEnergy(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const double stiffness)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);
            double origin_l = origin_edge_length * 2.0;

            double energy = 0.0;

            for (int k = 1; k <= num_fps; k++)
            {
                // edge_0 is edge_{i-1}, edge_1 is edge_{i}
                Eigen::Vector3d edge_0 = getFpPos(fps_pos, k) - getFpPos(fps_pos, k - 1);
                Eigen::Vector3d edge_1 = getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k);

                Eigen::Vector3d kb = Kb(edge_0, edge_1);
                energy += kb.dot(kb);
            }

            energy *= stiffness / origin_l;

            return energy;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd bendEnergyGradientWithUnitStiffness(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);
            double origin_l = origin_edge_length * 2.0;

            Eigen::VectorXd gradient = Eigen::VectorXd::Zero(fps_pos.size());

            for (int k = 2; k <= num_fps - 1; k++)
            {
                Eigen::Vector3d fp_gradient = 2.0 / origin_l * (d_kb_d_next_x(fps_pos, k - 1).transpose() * Kb(fps_pos, k - 1) + d_kb_d_x(fps_pos, k).transpose() * Kb(fps_pos, k) + d_kb_d_last_x(fps_pos, k + 1).transpose() * Kb(fps_pos, k + 1));

                gradient.block<3, 1>(3 * k, 0) = fp_gradient;
            }

            return gradient;
        }

        // ----------------------------------------------------------
        static double calcThetaN(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::Vector3d u_0 = m1_0;
            VecEigenVec3 u_all = bishopFrame(fps_pos, u_0);

            Eigen::Vector3d tangent_n = (getFpPos(fps_pos, num_fps + 1) -
                                         getFpPos(fps_pos, num_fps))
                                            .normalized();
            double theta_n = thetaBetweenBishopAndMaterial(tangent_n, u_all[num_fps], m1_n, theta_n_init);

            return theta_n;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd calcThetas(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::Vector3d u_0 = m1_0;

            // calculate the bishop frame
            VecEigenVec3 u_all = bishopFrame(fps_pos, u_0);
            VecEigenVec3 t_all(u_all.size());
            VecEigenVec3 v_all(u_all.size());
            for (size_t k = 0; k <= num_fps; k++)
            {
                t_all[k] = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).normalized();
                v_all[k] = t_all[k].cross(u_all[k]);
            }

            // calculate theta_n
            double theta_n = thetaBetweenBishopAndMaterial(t_all[num_fps], u_all[num_fps], m1_n, theta_n_init);

            // calculate all internal theta
            Eigen::VectorXd thetas = Eigen::VectorXd::Zero(num_fps + 1);
            for (size_t k = 1; k <= num_fps; k++)
            {
                thetas[k] = 0.0 + (theta_n - 0.0) / num_fps * k;
            }

            return thetas;
        }

        // ----------------------------------------------------------
        static VecEigenMatrix3d calcMaterialFrames(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::Vector3d u_0 = m1_0;

            // calculate bishop frame
            VecEigenVec3 u_all = bishopFrame(fps_pos, u_0);
            VecEigenVec3 t_all(u_all.size());
            VecEigenVec3 v_all(u_all.size());
            for (size_t k = 0; k <= num_fps; k++)
            {
                t_all[k] = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).normalized();
                v_all[k] = t_all[k].cross(u_all[k]);
            }

            // calculate theta_n
            double theta_n = thetaBetweenBishopAndMaterial(t_all[num_fps], u_all[num_fps], m1_n, theta_n_init);

            // calcualte all internal theta
            Eigen::VectorXd thetas = Eigen::VectorXd::Zero(num_fps + 1);
            for (size_t k = 1; k <= num_fps; k++)
            {
                thetas[k] = 0.0 + (theta_n - 0.0) / num_fps * k;
            }

            // calculate all material frames (rotation matrix)
            VecEigenMatrix3d material_frames(num_fps + 1);
            for (size_t k = 0; k <= num_fps; k++)
            {
                Eigen::Vector3d m1 = std::cos(thetas[k]) * u_all[k] + std::sin(thetas[k]) * v_all[k];
                Eigen::Vector3d m2 = -std::sin(thetas[k]) * u_all[k] + std::cos(thetas[k]) * v_all[k];

                Eigen::Matrix3d rot_mat;
                rot_mat.block<3, 1>(0, 0) = m2;
                rot_mat.block<3, 1>(0, 1) = t_all[k];
                rot_mat.block<3, 1>(0, 2) = m1;

                material_frames[k] = rot_mat;
            }

            return material_frames;
        }

        // ----------------------------------------------------------
        static VecEigenVector4d calcMaterialFramesQuat(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double theta_n_init = 0.0)
        {
            VecEigenMatrix3d material_frames = calcMaterialFrames(fps_pos, m1_0, m1_n, theta_n_init);

            VecEigenVector4d material_frame_quats(material_frames.size());
            for (size_t k = 0; k < material_frames.size(); k++)
            {
                material_frame_quats[k] = Eigen::Quaterniond(material_frames[k]).coeffs();
            }

            return material_frame_quats;
        }

        // ----------------------------------------------------------
        static void calcMaterialFrames(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            Eigen::VectorXd &thetas,
            VecEigenMatrix3d &material_frames,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::Vector3d u_0 = m1_0;

            // calculate bishop frame
            VecEigenVec3 u_all = bishopFrame(fps_pos, u_0);
            VecEigenVec3 t_all(u_all.size());
            VecEigenVec3 v_all(u_all.size());
            for (size_t k = 0; k <= num_fps; k++)
            {
                t_all[k] = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).normalized();
                v_all[k] = t_all[k].cross(u_all[k]);
            }

            // calculate theta_n
            double theta_n = thetaBetweenBishopAndMaterial(t_all[num_fps], u_all[num_fps], m1_n, theta_n_init);

            // calcualte all internal theta
            thetas = Eigen::VectorXd::Zero(num_fps + 1);
            for (size_t k = 1; k <= num_fps; k++)
            {
                thetas[k] = 0.0 + (theta_n - 0.0) / num_fps * k;
            }

            // calculate all material frames (rotation matrix)
            material_frames = VecEigenMatrix3d(num_fps + 1);
            for (size_t k = 0; k <= num_fps; k++)
            {
                Eigen::Vector3d m1 = std::cos(thetas[k]) * u_all[k] + std::sin(thetas[k]) * v_all[k];
                Eigen::Vector3d m2 = -std::sin(thetas[k]) * u_all[k] + std::cos(thetas[k]) * v_all[k];

                Eigen::Matrix3d rot_mat;
                rot_mat.block<3, 1>(0, 0) = m2;
                rot_mat.block<3, 1>(0, 1) = t_all[k];
                rot_mat.block<3, 1>(0, 2) = m1;

                material_frames[k] = rot_mat;
            }
        }

        // ----------------------------------------------------------
        static void calcMaterialFrames(
            const Eigen::VectorXd &fps_pos,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            Eigen::VectorXd &thetas,
            VecEigenVector4d &material_frame_quats,
            const double theta_n_init = 0.0)
        {
            VecEigenMatrix3d material_frames;
            calcMaterialFrames(fps_pos, m1_0, m1_n, thetas, material_frames, theta_n_init);

            material_frame_quats = VecEigenVector4d(material_frames.size());
            for (size_t k = 0; k < material_frames.size(); k++)
            {
                material_frame_quats[k] = Eigen::Quaterniond(material_frames[k]).coeffs();
            }
        }

        // ----------------------------------------------------------
        static double twistEnergy(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double stiffness,
            const double theta_n_init = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);
            double origin_l = origin_edge_length * 2.0;

            double theta_0 = 0.0;
            double theta_n = calcThetaN(fps_pos, m1_0, m1_n, theta_n_init);

            double energy = stiffness / (num_fps * origin_l) *
                            std::pow(theta_n - theta_0, 2);

            return energy;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd twistEnergyGradientWithUnitStiffness(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const Eigen::Vector3d &m1_0,
            const Eigen::Vector3d &m1_n,
            const double theta_n_init = 0.0)
        {
            Eigen::VectorXd twist_gradient = Eigen::VectorXd::Zero(fps_pos.size());

            double step_size = 1e-8;
            Eigen::VectorXd fps_pos_disturb = fps_pos;

            double twist_energy = twistEnergy(fps_pos, dlo_length, m1_0, m1_n, 1.0);

            for (int i = 3 * 2; i < fps_pos.size() - 3 * 2; i++) // excude the four points at the two ends
            {
                fps_pos_disturb[i] += step_size; // positive disturbance

                double twist_energy_pos_disturb = twistEnergy(
                    fps_pos_disturb, dlo_length, m1_0, m1_n, 1.0);

                // numerical differentiation
                twist_gradient(i) = (twist_energy_pos_disturb - twist_energy) / (step_size);

                fps_pos_disturb[i] = fps_pos[i]; // reset for next iteration
            }

            return twist_gradient;
        }

        // ----------------------------------------------------------
        static double gravityEnergy(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length,
            const double density,
            double ref_height = 0.0)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            double energy = 0.0;
            for (int k = 2; k <= num_fps - 1; k++)
            {
                Eigen::Vector3d fp_pos = getFpPos(fps_pos, k);
                energy += density * origin_edge_length *
                          9.8 * (fp_pos[2] - ref_height);
            }

            return energy;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd gravityEnergyGradientWithUnitStiffness(
            const Eigen::VectorXd &fps_pos,
            const double dlo_length)
        {
            int num_fps = fps_pos.size() / 3 - 2;
            double origin_edge_length = dlo_length / (num_fps - 1.0);

            Eigen::VectorXd gradient = Eigen::VectorXd::Zero(fps_pos.size());

            for (int k = 2; k <= num_fps - 1; k++)
            {
                gradient(3 * k + 2) = 1.0 * origin_edge_length * 9.8;
            }
            return gradient;
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
                {                                    // excude the four points at the two ends
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

        // ----------------------------------------------------------
        static Eigen::VectorXd inextensibleConstraints(
            const Eigen::VectorXd &fps_pos, // extend_fps_pos
            const Eigen::VectorXd &edges_length)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::VectorXd constraints = Eigen::VectorXd::Zero(num_fps + 1);
            for (int k = 0; k < constraints.size(); k++)
            {
                constraints(k) = (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).squaredNorm() / edges_length(k) - edges_length(k); // = 0
            }

            return constraints;
        }

        // ----------------------------------------------------------
        static Eigen::MatrixXd inextensibleConstraintsJacobian(
            const Eigen::VectorXd &fps_pos, // extend_fps_pos
            const Eigen::VectorXd &edges_length)
        {
            int num_fps = fps_pos.size() / 3 - 2;

            Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(num_fps + 1, 3 * (num_fps + 2));
            for (int k = 0; k <= num_fps; k++)
            {
                jacobian.block<1, 3>(k, 3 * k) = -2 * (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).transpose() / edges_length(k);
                jacobian.block<1, 3>(k, 3 * k + 3) = 2 * (getFpPos(fps_pos, k + 1) - getFpPos(fps_pos, k)).transpose() / edges_length(k);
            }

            return jacobian;
        }

        // ----------------------------------------------------------
        static Eigen::VectorXd inextensibleProjection(
            const Eigen::VectorXd &fps_pos_init, // extend_fps_pos
            const Eigen::VectorXd &edges_length)
        {
            int num_fps = fps_pos_init.size() / 3 - 2;
            if (edges_length.size() != num_fps + 1)
                std::cerr << "inextensibleConstraintMatrix(): the input size is wrong." << std::endl;

            double h = 1.0; // 1.0
            Eigen::MatrixXd inv_M = Eigen::MatrixXd::Identity(fps_pos_init.size(), fps_pos_init.size());

            Eigen::VectorXd fps_pos = fps_pos_init; // variable

            size_t iter = 0;
            size_t max_iter = 50;
            while (ros::ok() & iter < max_iter)
            {
                Eigen::VectorXd constraints = inextensibleConstraints(fps_pos, edges_length);
                Eigen::MatrixXd jaco_C = inextensibleConstraintsJacobian(fps_pos, edges_length);

                double constraints_error = constraints.block(1, 0, num_fps - 1, 1).cwiseAbs().mean(); // 不考虑最两边的extend edges

                ROS_DEBUG_STREAM("inextensibleProjection(): constraints error: " << constraints_error);

                if (constraints_error < 1e-4)
                {
                    ROS_DEBUG_STREAM("inextensibleProjection(): satisfy the error threshold.");
                    break;
                }

                // calculate delta_lambda_{j+1}
                Eigen::MatrixXd A = h * h * jaco_C * inv_M * jaco_C.transpose();
                Eigen::MatrixXd b = constraints;
                Eigen::VectorXd delta_lambda = A.colPivHouseholderQr().solve(b);

                // calculate delta_x_{j+1}
                Eigen::VectorXd delta_x = -h * h * inv_M * jaco_C.transpose() * delta_lambda;

                // calculate x_{j+1} = x_{j} + delta_x_{j+1} (the four points at the two ends are fixed)
                fps_pos.block(3 * 2, 0, 3 * (num_fps - 2), 1) += delta_x.block(3 * 2, 0, 3 * (num_fps - 2), 1);

                iter++;
            }
            if (iter == max_iter)
                ROS_INFO_STREAM("inextensibleProjection(): projection fails.");

            return fps_pos;
        }

    }; // end class
} // end namespace dlo_arm_planning_pkg

#endif