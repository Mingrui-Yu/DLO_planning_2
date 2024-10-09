#ifndef DLO_ARM_PLANNING_DLO_STATE_H
#define DLO_ARM_PLANNING_DLO_STATE_H

#include "dlo_arm_planning_pkg/common_include.h"


namespace dlo_arm_planning_pkg{

// ------------------------------------------------------------
class DLOState{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<DLOState> Ptr;

    DLOState(){}

    DLOState(
        const Eigen::VectorXd &fps_pos,
        const Eigen::Vector4d &end_quat_0 = Eigen::Vector4d::Zero(),
        const Eigen::Vector4d &end_quat_1 = Eigen::Vector4d::Zero()
    );

    void updateDependentInfo(
        double theta_n_init = 0.0
    );

    double getThetaN() const
    {
        if(angle_b2m_.size() != 0){
            return angle_b2m_(angle_b2m_.size() - 1);
        }else{
            ROS_ERROR("DLOState::getThetaN(): angle_b2m_ is empty.");
            return 0.0;
        }
    }

    // ------------------------------------------------
    Eigen::Vector3d getFpPos(int index) const{
        ROS_ERROR_COND(fps_pos_.size() == 0, "DLOState: fps_pos_ is empty.");
        return fps_pos_.block<3, 1>(3 * index, 0);
    }  

    // ------------------------------------------------
    void setFpPos(int index, const Eigen::Vector3d &pos){
        ROS_ERROR_COND(fps_pos_.size() == 0, "DLOState: fps_pos_ is empty.");
        fps_pos_.block<3, 1>(3 * index, 0) = pos;
    }

    // ------------------------------------------------
    Eigen::Vector3d getLeftEndPos() const
    {
        return getFpPos(0);
    }

    // ------------------------------------------------
    Eigen::Vector3d getRightEndPos() const
    {
        int num_fps = fps_pos_.size() / 3;
        return getFpPos(num_fps-1);
    }

    // ------------------------------------------------
    Eigen::Isometry3d getLeftEndPose() const
    {
        ROS_ERROR_COND(fps_pos_.size() == 0, "DLOState: fps_pos_ is empty.");
        ROS_ERROR_COND(end_quat_0_.size() == 0, "DLOState: end_quat_1_ is empty.");

        int num_fps = fps_pos_.size() / 3;
        return Utils::EigenPosQuatVec2Isometry3d(getFpPos(0), end_quat_0_);
    }

    // ------------------------------------------------
    Eigen::Isometry3d getRightEndPose() const
    {
        ROS_ERROR_COND(fps_pos_.size() == 0, "DLOState: fps_pos_ is empty.");
        ROS_ERROR_COND(end_quat_1_.size() == 0, "DLOState: end_quat_1_ is empty.");

        int num_fps = fps_pos_.size() / 3;
        return Utils::EigenPosQuatVec2Isometry3d(getFpPos(num_fps-1), end_quat_1_);
    }


    // ------------------------------------------------
    double getLength() const;

    Eigen::VectorXd getEdgesLength() const;

    Eigen::VectorXd getExtendEdgesLength() const;



public:
    Eigen::VectorXd fps_pos_; // size: num_fps * 3
    Eigen::Vector4d end_quat_0_, end_quat_1_; // (x, y, z, w)
       
    // dependent information
    int num_fps_;
    Eigen::VectorXd extend_fps_pos_; // size: (num_fps + 2) * 3 
    Eigen::VectorXd angle_b2m_; // size: (num_fps+1) * 1 
    Eigen::Vector3d m1_0_, m1_n_;
    VecEigenVector4d material_frames_; // size: (num_fps+1) * (x, y, z, w)


}; // end class


} // end namespace

#endif