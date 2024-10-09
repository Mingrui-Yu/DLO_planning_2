#ifndef DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_VARIABLES_H
#define DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_VARIABLES_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dual_arm.h"

#include <ifopt/variable_set.h>


namespace dlo_arm_planning_pkg{
namespace multi_step_mpc_ifopt{

using namespace ifopt;

/** -----------------------------------------------------------------------------------------
 * @brief arm joint velocities at {t = 0,...,T-1}
 * actual control input
 */
class ArmJointVelVariables : public VariableSet 
{
public:
    ArmJointVelVariables(const int &n_step, const Arm::Ptr &arm)
        : VariableSet(arm->joint_num_ * n_step, arm->arm_group_name_ + "_joint_vel")
    {
        n_step_ = n_step;
        arm_ = arm;
        joint_vel_ = Eigen::VectorXd::Zero(arm->joint_num_ * n_step_); // default initial value
    }

    // ---------------------------
    void SetParams(
        const Eigen::VectorXd &max_joint_vel
    ){
        max_joint_vel_ = max_joint_vel;
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &x) override{
        joint_vel_ = x;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return joint_vel_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        // if the max_joint_vel has not been set, then no bounds.
        if(max_joint_vel_.size() == 0){
            for(size_t i = 0; i < GetRows(); i++) 
                bounds.at(i) = NoBound;
        }
        // if the max_joint_vel has been set
        else{
            for(size_t t = 0; t < n_step_; t++){
                for(size_t i = 0; i < arm_->joint_num_; i++){
                    bounds.at(arm_->joint_num_ * t + i) = Bounds(-max_joint_vel_(i), max_joint_vel_(i));
                }
            }
        }
        return bounds;
    }

private:
    Eigen::VectorXd joint_vel_; // variable
    int n_step_; // T
    Arm::Ptr arm_;
    // params
    Eigen::VectorXd max_joint_vel_ = Eigen::VectorXd::Zero(0);
};


/** -----------------------------------------------------------------------------------------
 * @brief arm joint pos at {t = 0,...,T}
 */
class ArmJointPosVariables : public VariableSet 
{
public:
    ArmJointPosVariables(const int &n_step, const Arm::Ptr &arm)
        : VariableSet(arm->joint_num_ * (n_step+1), arm->arm_group_name_ + "_joint_pos")
    {
        n_step_ = n_step;
        arm_ = arm;
        joint_pos_ = Eigen::VectorXd::Zero(arm->joint_num_ * (n_step_+1)); // default initial value
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &x) override{
        joint_pos_ = x;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return joint_pos_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        for(size_t t = 0; t <= n_step_; t++){
            for(size_t i = 0; i < arm_->joint_num_; i++){
                bounds.at(arm_->joint_num_ * t + i) = Bounds(arm_->joint_sample_lb_[i], arm_->joint_sample_ub_[i]);
            }
        }
        return bounds;
    }

private:
    Eigen::VectorXd joint_pos_; // variable
    int n_step_; // T
    Arm::Ptr arm_;
};


/** -----------------------------------------------------------------------------------------
 * @brief dual arm critical points' positions (for collision) at {t = 0,...T}
 * 
 */
class DualArmCriticalPointsVariables : public VariableSet 
{
public:
    DualArmCriticalPointsVariables(const int &n_step, const Scene::Ptr &scene): 
        VariableSet(scene->cd_fcl_->objects_dual_arm_.size() * 3 * (n_step+1), "dual_arm_critical_points")
    {
        n_step_ = n_step;
        scene_ = scene;
        int points_dim = scene_->cd_fcl_->objects_dual_arm_.size() * 3;
        critical_points_ = Eigen::VectorXd::Zero(points_dim * (n_step+1)); // default initial value
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &x) override{
        critical_points_ = x;
    }

    // ---------------------------
    void SetVariablesByJointPath(
        const Eigen::VectorXd &arm_0_path, 
        const Eigen::VectorXd &arm_1_path)
    {
        int points_dim = scene_->cd_fcl_->objects_dual_arm_.size() * 3;
        int arm_0_joint_num = scene_->dual_arm_->arm_0_->joint_num_;
        int arm_1_joint_num = scene_->dual_arm_->arm_1_->joint_num_;

        for(size_t t = 0; t <= n_step_; t++){
            critical_points_.block(points_dim * t, 0, points_dim, 1) 
                = scene_->cd_fcl_->getCriticalPointsPosVec(arm_0_path.reshaped(arm_0_joint_num, n_step_+1).col(t),
                                                           arm_1_path.reshaped(arm_1_joint_num, n_step_+1).col(t));
        }
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return critical_points_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        for(size_t i = 0; i < GetRows(); i++) bounds.at(i) = NoBound;
        return bounds;
    }

private:
    Eigen::VectorXd critical_points_; // variable
    int n_step_; // T
    Scene::Ptr scene_;
};


/** -----------------------------------------------------------------------------------------
 * @brief DLO fps pos at {t = 0,...,T}
 */
class DLOFpsPosVariables : public VariableSet
{
public:
    DLOFpsPosVariables(const int &n_step, const int &num_fps)
        : VariableSet(3*num_fps * (n_step+1), "dlo_fps_pos")
    {
        n_step_ = n_step;
        fps_pos_ = Eigen::VectorXd::Zero(3*num_fps * (n_step+1));
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &fps_pos) override{
        fps_pos_ = fps_pos;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return fps_pos_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        for(size_t i = 0; i < GetRows(); i++) bounds.at(i) = NoBound;
        return bounds;
    }

private:
    Eigen::VectorXd fps_pos_; // variable
    int n_step_; // T
};


/** -----------------------------------------------------------------------------------------
 * @brief DLO fps vel at {t = 0,...,T-1}
 */
class DLOFpsVelVariables : public VariableSet
{
public:
    DLOFpsVelVariables(const int &n_step, const int &num_fps)
        : VariableSet(3*num_fps * n_step, "dlo_fps_vel")
    {
        n_step_ = n_step;
        fps_vel_ = Eigen::VectorXd::Zero(3*num_fps * n_step);
    }

    // ---------------------------
    void SetParams(
        const double &max_vel
    ){
        max_vel_ = max_vel;
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &fps_vel) override{
        fps_vel_ = fps_vel;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return fps_vel_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        if(max_vel_ > 0){
            for(size_t i = 0; i < GetRows(); i++) 
                bounds.at(i) = Bounds(-max_vel_, max_vel_); // test: 感觉是很有效的
        }else{
            for(size_t i = 0; i < GetRows(); i++) 
                bounds.at(i) = NoBound;
        }
        return bounds;
    }

private:
    Eigen::VectorXd fps_vel_; // variable
    int n_step_; // T
    // params
    double max_vel_ = 0; // max velocity of each feature point on each dimension
};


/** -----------------------------------------------------------------------------------------
 * @brief DLO edge points at {t = 1,...,T}
 */
class DLOEdgePointsPosVariables : public VariableSet
{
public:
    DLOEdgePointsPosVariables(const int n_step, const DLO::Ptr &dlo)
        : VariableSet(3 * (dlo->num_point_per_edge_ * (dlo->num_fps_-1)) * n_step, "dlo_edge_points_pos")
    {
        dlo_ = dlo;
        n_step_ = n_step;
        points_ = Eigen::VectorXd::Zero(3 * (dlo->num_point_per_edge_ * (dlo_->num_fps_-1)) * n_step);
    }

    // ---------------------------
    void setVariablesByFpsPos(const Eigen::VectorXd &fps_pos_all){
        int num_fps = dlo_->num_fps_;
        int num_edge = num_fps - 1;
        int n = dlo_->num_point_per_edge_;

        Eigen::MatrixXd fps_pos = fps_pos_all.reshaped(3*num_fps, n_step_+1).block(0, 1, 3*num_fps, n_step_); // 取 t = 1, ..., T
        Eigen::MatrixXd edge_points = Eigen::MatrixXd::Zero((num_edge*n)*3, n_step_);
        
        for(size_t t = 0; t < n_step_; t++){
            for(size_t k = 0; k < num_edge; k++){
                Eigen::Vector3d fp_0 = fps_pos.col(t).block<3, 1>(3 * k, 0);
                Eigen::Vector3d fp_1 = fps_pos.col(t).block<3, 1>(3 * (k+1), 0);
                for(size_t i = 0; i < n; i++){
                    double ratio = double(i+1) / double(n+1);
                    edge_points.col(t).block<3, 1>(k*(n*3) + i*3, 0) = (1.0 - ratio) * fp_0 + ratio * fp_1;
                }
            }
        }
        points_ = edge_points.reshaped(n_step_ * num_edge * n * 3, 1);
    }

    // ---------------------------
    void SetVariables(const Eigen::VectorXd &points) override{
        points_ = points;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        return points_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound bounds(GetRows());
        for(size_t i = 0; i < GetRows(); i++) bounds.at(i) = NoBound;
        return bounds;
    }

private:
    Eigen::VectorXd points_; // variable
    DLO::Ptr dlo_;
    int n_step_; // T
};




} // end namespace 
} // end namespace 

#endif