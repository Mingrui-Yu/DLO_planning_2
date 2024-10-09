#include "dlo_arm_planning_pkg/dlo_state.h"
#include "dlo_arm_planning_pkg/dlo_model/derm_eigen.h"

namespace dlo_arm_planning_pkg
{

    // ------------------------------------------------------------
    DLOState::DLOState(
        const Eigen::VectorXd &fps_pos,
        const Eigen::Vector4d &end_quat_0,
        const Eigen::Vector4d &end_quat_1)
    {
        if (fps_pos.size() % 3 != 0)
            ROS_ERROR("DLOState(): the size of fps_pos is not the multiple of 3.");

        fps_pos_ = fps_pos;
        num_fps_ = fps_pos_.size() / 3;

        end_quat_0_ = end_quat_0;
        end_quat_1_ = end_quat_1;
    }

    // ------------------------------------------------------------
    void DLOState::updateDependentInfo(
        double theta_n_init)
    {
        if (end_quat_0_.size() == 0)
            ROS_ERROR("DLOState::updateDependentInfo(): end_quat_0_ is empty.");
        if (end_quat_1_.size() == 0)
            ROS_ERROR("DLOState::updateDependentInfo(): end_quat_1_ is empty.");
        if (fps_pos_.size() == 0)
            ROS_ERROR("DLOState::updateDependentInfo(): fps_pos_ is empty.");

        // get the end orientations
        Eigen::Matrix3d end_rot_mat_0 = Eigen::Quaterniond(end_quat_0_).normalized().toRotationMatrix();
        Eigen::Matrix3d end_rot_mat_1 = Eigen::Quaterniond(end_quat_1_).normalized().toRotationMatrix();

        // use the z-axis of the rotation matrix as m1
        m1_0_ = end_rot_mat_0.block<3, 1>(0, 2);
        m1_n_ = end_rot_mat_1.block<3, 1>(0, 2);

        // use the y-axis of the rotation matrix as t (tangent vector)
        Eigen::Vector3d t_0 = end_rot_mat_0.block<3, 1>(0, 1);
        Eigen::Vector3d t_n = end_rot_mat_1.block<3, 1>(0, 1);

        // compute the extended feature point positions (i = 0 and m+1 in DER)
        double approx_edge_length = getLength() / (num_fps_ - 1.0);
        Eigen::Vector3d fp_left = getFpPos(0) - approx_edge_length * t_0;
        Eigen::Vector3d fp_right = getFpPos(num_fps_ - 1) + approx_edge_length * t_n;

        extend_fps_pos_ = Eigen::VectorXd::Zero(3 * (num_fps_ + 2));
        extend_fps_pos_.block<3, 1>(0, 0) = fp_left;                 // 0 ~ 0 + 3
        extend_fps_pos_.block(3, 0, 3 * num_fps_, 1) = fps_pos_;     // 3 ~ 3 + 3*num_fps
        extend_fps_pos_.block<3, 1>(3 + 3 * num_fps_, 0) = fp_right; // 3 + 3*num_fps ~ 3 + 3*num_fps + 3

        // compute the edge frames
        derm_eigen::calcMaterialFrames(extend_fps_pos_, m1_0_, m1_n_, /*return*/ angle_b2m_, /*return*/ material_frames_, theta_n_init);
    }

    // ------------------------------------------------------------
    double DLOState::getLength() const
    {
        double length = 0.0;
        for (int k = 0; k < num_fps_ - 1; k++)
        {
            length += (getFpPos(k + 1) - getFpPos(k)).norm();
        }
        return length;
    }

    // ------------------------------------------------------------
    Eigen::VectorXd DLOState::getEdgesLength() const
    {
        Eigen::VectorXd edges_length(num_fps_ - 1);

        for (int k = 0; k < num_fps_ - 1; k++)
        {
            edges_length(k) = (getFpPos(k + 1) - getFpPos(k)).norm();
        }
        return edges_length;
    }

    // ------------------------------------------------------------
    Eigen::VectorXd DLOState::getExtendEdgesLength() const
    {
        Eigen::VectorXd edges_length(num_fps_ + 1);

        for (int k = 0; k < num_fps_ + 1; k++)
        {
            edges_length(k) = (extend_fps_pos_.block<3, 1>(3 * (k + 1), 0) - extend_fps_pos_.block<3, 1>(3 * k, 0)).norm();
        }

        return edges_length;
    }

} // end namespace