#ifndef DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_CONSTRAINTS_H
#define DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_CONSTRAINTS_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/jacobian_model.h"
#include "dlo_arm_planning_pkg/scene.h"

#include <ifopt/constraint_set.h>

namespace dlo_arm_planning_pkg{
namespace multi_step_mpc_ifopt{

using namespace ifopt;

/** -----------------------------------------------------------------------------------------
 * @brief arm transition constraint
 * g_{t} = (q_{t} + q_vel_{t} * delta_t) - q_{t+1},  t = 0,...,T-1
 * g_{t} = 0
 */
class ArmTransitionConstraint: public ConstraintSet
{
public:
    ArmTransitionConstraint(const int &n_step, const Arm::Ptr &arm)
        : ConstraintSet(arm->joint_num_ * n_step, arm->arm_group_name_ + "_transition_constraint")
    {
        n_step_ = n_step;
        group_name_ = arm->arm_group_name_;
        arm_ = arm;
    }

    // ---------------------------
    void SetParams(
        const double &delta_t
    ){
        delta_t_ = delta_t;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        Eigen::MatrixXd q_vel = (GetVariables()->GetComponent(group_name_ + "_joint_vel")->GetValues()).reshaped(arm_->joint_num_, n_step_);
        Eigen::MatrixXd q = (GetVariables()->GetComponent(group_name_ + "_joint_pos")->GetValues()).reshaped(arm_->joint_num_, n_step_+1);

        Eigen::MatrixXd g_mat = Eigen::MatrixXd::Zero(arm_->joint_num_ , n_step_);
        for(size_t t = 0; t < n_step_; t++){
            g_mat.col(t) = (q.col(t) + q_vel.col(t) * delta_t_) - q.col(t+1);
        }

        return g_mat.reshaped(arm_->joint_num_ * n_step_, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        auto &n_joint = arm_->joint_num_;
        if(var_set == group_name_ + "_joint_vel"){
            for(size_t t = 0; t < n_step_; t++){
                for(int j = 0; j < n_joint; j++){
                    jac_block.coeffRef(n_joint * t + j, n_joint * t + j) = delta_t_; // g_{t}(j) / q_vel_{t}(j)
                }
            }
        }
        if(var_set == group_name_ + "_joint_pos"){
            for(size_t t = 0; t < n_step_; t++){
                for(int j = 0; j < n_joint; j++){
                    jac_block.coeffRef(n_joint * t + j, n_joint * t + j) = 1.0; // g_{t}(j) / q_{t}(j)
                    jac_block.coeffRef(n_joint * t + j, n_joint * (t+1) + j) = -1.0; // g_{t}(j) / q_{t+1}(j)
                }
            }
        }
    }

private: 
    int n_step_;
    Arm::Ptr arm_;
    std::string group_name_;
    // parameters
    double delta_t_;
};


/** -----------------------------------------------------------------------------------------
 * @brief arm joint pos start constraint
 * g = q_{0} - q_s 
 * g = 0
 */
class ArmJointPosStartConstraint: public ConstraintSet
{
public:
    ArmJointPosStartConstraint(const Arm::Ptr &arm)
        : ConstraintSet(arm->joint_num_, arm->arm_group_name_ + "_joint_pos_start_constraint")
    {
        arm_ = arm;
        group_name_ = arm->arm_group_name_;
        q_start_ = Eigen::VectorXd::Zero(arm->joint_num_);
    }

    // ---------------------------
    void SetParams(
        const Eigen::VectorXd &q_start
    ){
        q_start_ = q_start;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        Eigen::VectorXd q = GetVariables()->GetComponent(group_name_ + "_joint_pos")->GetValues();
        Eigen::VectorXd g = q.block(0, 0, arm_->joint_num_, 1) - q_start_; // q_{0} - q_s
        return g;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        auto &n_joint = arm_->joint_num_;
        if(var_set == group_name_ + "_joint_pos"){
            for(int j = 0; j < n_joint; j++){
                jac_block.coeffRef(j, 0 + j) = 1.0; // g(j) / q_{0}(j)
            }
        }
    }

private:
    Arm::Ptr arm_;
    std::string group_name_;
    // parameters
    Eigen::VectorXd q_start_;
};


/** -----------------------------------------------------------------------------------------
 * @brief dual arm critical points forward kinematics constraint
 * FK(q_{t}) - points_{t} = 0, t = 0,...,T
 */
class DualArmCriticalPointsFKConstraint: public ConstraintSet
{
public:
    DualArmCriticalPointsFKConstraint(const int &n_step, const Scene::Ptr &scene): 
        ConstraintSet(scene->cd_fcl_->objects_dual_arm_.size() * 3 * (n_step+1), "dual_arm_critical_points_fk_constraint")
    {
        n_step_ = n_step;
        scene_ = scene;
        cd_fcl_ = scene->cd_fcl_;
        dual_arm_ = scene->dual_arm_;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int points_dim = scene_->cd_fcl_->objects_dual_arm_.size() * 3;
        int arm_0_joint_num = scene_->dual_arm_->arm_0_->joint_num_;
        int arm_1_joint_num = scene_->dual_arm_->arm_1_->joint_num_;

        Eigen::MatrixXd critical_points = (GetVariables()->GetComponent("dual_arm_critical_points")->GetValues()).reshaped(points_dim, n_step_+1);
        Eigen::MatrixXd arm_0_joint_pos = (GetVariables()->GetComponent("arm_0_joint_pos")->GetValues()).reshaped(arm_0_joint_num, n_step_+1);
        Eigen::MatrixXd arm_1_joint_pos = (GetVariables()->GetComponent("arm_1_joint_pos")->GetValues()).reshaped(arm_1_joint_num, n_step_+1);

        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(points_dim, n_step_+1);
        for(size_t t = 0; t <= n_step_; t++){
            g.col(t) = cd_fcl_->getCriticalPointsPosVec(arm_0_joint_pos.col(t), arm_1_joint_pos.col(t)) - critical_points.col(t);
        }

        return g.reshaped(points_dim * (n_step_+1), 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        int points_dim = scene_->cd_fcl_->objects_dual_arm_.size() * 3;

        if(var_set == "dual_arm_critical_points"){
            for(size_t t = 0; t <= n_step_; t++){
                for(int i = 0; i < points_dim; i++){
                    jac_block.coeffRef(points_dim * t + i, points_dim * t + i) = -1.0;
                }
            }
            
        }
        if(var_set == "arm_0_joint_pos" || var_set == "arm_1_joint_pos"){
            Arm::Ptr arm;
            if(var_set == "arm_0_joint_pos")
                arm = dual_arm_->arm_0_;
            else
                arm = dual_arm_->arm_1_;

            Eigen::MatrixXd joint_pos = (GetVariables()->GetComponent(var_set)->GetValues()).reshaped(arm->joint_num_, (n_step_+1));

            // 遍历每个时间t
            for(size_t t = 0; t <= n_step_; t++){
                arm->setJointPositions(joint_pos.col(t));
                // 遍历所有 critical points
                for(size_t k = 0; k < cd_fcl_->objects_dual_arm_.size(); k++){
                    auto &robot_data = cd_fcl_->objects_dual_arm_userdata_[k].robot_data;

                    if(robot_data.group_name != arm->arm_group_name_)
                        continue;

                    Eigen::MatrixXd jacobian = arm->getJacobianMatrix(
                        robot_data.link_name, /*reference_point*/robot_data.relative_pose.translation());

                    for(int i = 0; i < 3; i++){ // g_{t}_{k} / q_{t}
                        for(int j = 0; j < arm->joint_num_; j++){
                            jac_block.coeffRef(points_dim*t + 3*k + i, arm->joint_num_*t + j) = jacobian(i, j);
                        }
                    }
                }
            }
        }
    }

private: 
    int n_step_; // T
    Scene::Ptr scene_;
    CollisionDetectionFCL::Ptr cd_fcl_;
    DualArm::Ptr dual_arm_;
}; 


/** -----------------------------------------------------------------------------------------
 * @brief constain the minimal distance between dual_arm and world
 * g_{t} = distance_{t} - min_dist_thres_ >= 0, t = 1,...,T
 */
class DualArmWorldDistanceConstraint: public ConstraintSet
{
public:
    DualArmWorldDistanceConstraint(const int &n_step, const Scene::Ptr &scene): 
        ConstraintSet(scene->cd_fcl_->objects_dual_arm_.size() * n_step, "dual_arm_world_dist_constraint") // 这里不能用(n_step+1)，因为若初态(t=0)不满足约束，则无法进行优化
    {
        n_step_ = n_step;
        scene_ = scene;
        cd_fcl_ = scene->cd_fcl_;
    }

    // ---------------------------
    void SetParams(const double min_dist_thres){
        min_dist_thres_ = min_dist_thres;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int n_points = scene_->cd_fcl_->objects_dual_arm_.size();
        Eigen::MatrixXd critical_points = (GetVariables()->GetComponent("dual_arm_critical_points")->GetValues()).reshaped(n_points*3, (n_step_+1));
        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(n_points, n_step_);

        for(size_t t = 1; t <= n_step_; t++){
            VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(critical_points.col(t));
            if(points.size() != n_points) std::cerr << "DualArmWorldDistanceConstraint: wrong size." << std::endl;

            for(size_t i = 0; i < n_points; i++){
                g.col(t-1)(i) = (scene_->getPrecisePointDist(points[i]) - cd_fcl_->objects_dual_arm_userdata_[i].geometry_params[0]) - min_dist_thres_;
            }
        }

        return g.reshaped(n_points * n_step_, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows());
        for(size_t i = 0; i < GetRows(); i++) 
            b.at(i) = BoundGreaterZero; // >=0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dual_arm_critical_points"){
            int n_points = scene_->cd_fcl_->objects_dual_arm_.size();
            Eigen::MatrixXd critical_points = (GetVariables()->GetComponent("dual_arm_critical_points")->GetValues()).reshaped(n_points*3, (n_step_+1));

            for(size_t t = 1; t <= n_step_; t++){
                VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(critical_points.col(t));

                for(size_t i = 0; i < points.size(); i++){ // g_{t}_{i} / p_{t}_{i} 
                    Eigen::Vector3d dist_gradient = scene_->getPointDistGradient(points[i]);
                    Eigen::Vector3d normalized_grad = dist_gradient.normalized();
                    
                    for(size_t j = 0; j < 3; j++){ // g_{t}_{i} / p_{t}_{i}_{j}
                        jac_block.coeffRef(n_points*(t-1) + i, n_points*3*t + 3*i + j) = normalized_grad(j); // g从t=1开始
                    }
                }
            }
        }   
    }

private: 
    int n_step_; // T
    Scene::Ptr scene_;
    CollisionDetectionFCL::Ptr cd_fcl_;
    double min_dist_thres_ = 0.01;
}; 


/** -----------------------------------------------------------------------------------------
 * @brief DLO transition constraint
 * g_{t} = (fps_pos_{t} + fps_vel_{t} * delta_t) - fps_pos_{t+1} = 0, t = 0,...,T-1
 */
class DLOTransitionConstraint: public ConstraintSet
{
public:
    DLOTransitionConstraint(
        const int &n_step, const DLO::Ptr &dlo
    ): ConstraintSet(3 * dlo->num_fps_ * n_step, "dlo_transition_constraint")
    {
        n_step_ = n_step;
        dlo_ = dlo;
    }

    // ---------------------------
    void SetParams(
        const double &delta_t
    ){
        delta_t_ = delta_t;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int fps_dim = 3*dlo_->num_fps_;
        Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(fps_dim, (n_step_+1));
        Eigen::MatrixXd fps_vel = (GetVariables()->GetComponent("dlo_fps_vel")->GetValues()).reshaped(fps_dim, n_step_);

        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(fps_dim, n_step_);
        for(size_t t = 0; t < n_step_; t++){
            g.col(t) = (fps_pos.col(t) + fps_vel.col(t) * delta_t_) - fps_pos.col(t+1);
        }
        return g.reshaped(fps_dim * n_step_, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        int fps_dim = 3*dlo_->num_fps_;
        if(var_set == "dlo_fps_vel"){
            for(size_t t = 0; t < n_step_; t++){ // g_{t} / fps_vel_{t}
                for(int j = 0; j < fps_dim; j++){
                    jac_block.coeffRef(fps_dim * t + j, fps_dim * t + j) = delta_t_;
                }
            }
        }
        if(var_set == "dlo_fps_pos"){ 
            for(size_t t = 0; t < n_step_; t++){
                for(int j = 0; j < fps_dim; j++){ 
                    jac_block.coeffRef(fps_dim * t + j, fps_dim * t + j) = 1.0; // g_{t} / fps_pos_{t}
                    jac_block.coeffRef(fps_dim * t + j, fps_dim * (t+1) + j) = -1.0; // g_{t} / fps_pos_{t+1}
                }
            }   
        }
    }

private:
    int n_step_; // T
    // parameters
    DLO::Ptr dlo_;
    double delta_t_;
};


/** -----------------------------------------------------------------------------------------
 * @brief DLO transition constraint
 * g = fps_pos_{0} - fps_pos_s = 0
 */
class DLOStartConstraint: public ConstraintSet
{
public:
    DLOStartConstraint(
        const DLO::Ptr &dlo
    ): ConstraintSet(3 * dlo->num_fps_, "dlo_start_constraint")
    {
        dlo_ = dlo;
        fps_pos_start_ = Eigen::VectorXd::Zero(3 * dlo->num_fps_);
    }

    // ---------------------------
    void SetParams(
        const Eigen::VectorXd &fps_pos_start
    ){
        fps_pos_start_ = fps_pos_start;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        Eigen::VectorXd fps_pos_0 = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).block(0, 0, 3*dlo_->num_fps_, 1);
        return fps_pos_0 - fps_pos_start_;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_fps_pos"){ 
            for(int j = 0; j < 3*dlo_->num_fps_; j++){ 
                jac_block.coeffRef(j, 0 + j) = 1.0; // g / fps_pos_0
            }
        }
    }

private:
    int n_step_; // T
    DLO::Ptr dlo_;
    // parameters
    Eigen::VectorXd fps_pos_start_;
};


/** -----------------------------------------------------------------------------------------
 * @brief constraint between DLO fps_vel & dual_arm joint_vel
 * g_{t} = fps_vel_{t} - J_dlo_{t} * J_dual_arm_{t} * q_vel{t}, t = 0,...,T-1
 */
class DLOJacobianConstraint: public ConstraintSet
{
public:
    DLOJacobianConstraint(
        const int &n_step,
        const DLO::Ptr &dlo, 
        const JacobianModel::Ptr &dlo_jaco_model,
        const DualArm::Ptr &dual_arm
    ): ConstraintSet(3 * dlo->num_fps_ * n_step, "dlo_jacobian_constraint")
    {
        n_step_ = n_step;
        dlo_ = dlo;
        dlo_jaco_model_ = dlo_jaco_model;
        dual_arm_ = dual_arm;
    }

    // ---------------------------
    Eigen::MatrixXd CalcJacobianAll(
        const Eigen::VectorXd &dlo_fps_pos,
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos
    ) const {
        Eigen::Vector4d end_0_quat = Eigen::Quaterniond(dual_arm_->arm_0_->getTcpPose(arm_0_joint_pos).rotation()).coeffs();
        Eigen::Vector4d end_1_quat = Eigen::Quaterniond(dual_arm_->arm_1_->getTcpPose(arm_1_joint_pos).rotation()).coeffs();
        DLOState dlo_state(dlo_fps_pos, end_0_quat, end_1_quat);

        // calculate the Jacobians
        Eigen::MatrixXd jaco_dlo = dlo_jaco_model_->calcJacobianMatrix(dlo_state, dlo_->dlo_length_);
        Eigen::MatrixXd jaco_arm_0 = dual_arm_->arm_0_->getTcpJacobianMatrix(arm_0_joint_pos);
        Eigen::MatrixXd jaco_arm_1 = dual_arm_->arm_1_->getTcpJacobianMatrix(arm_1_joint_pos);
        Eigen::MatrixXd jaco_dual_arm = Eigen::MatrixXd::Zero(jaco_arm_0.rows() + jaco_arm_1.rows(), jaco_arm_0.cols() + jaco_arm_1.cols());
        jaco_dual_arm.block(0, 0, jaco_arm_0.rows(), jaco_arm_0.cols()) = jaco_arm_0;
        jaco_dual_arm.block(jaco_arm_0.rows(), jaco_arm_0.cols(), jaco_arm_1.rows(), jaco_arm_1.cols()) = jaco_arm_1;

        return jaco_dlo * jaco_dual_arm;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        Eigen::MatrixXd dlo_fps_vel = (GetVariables()->GetComponent("dlo_fps_vel")->GetValues()).reshaped(3*dlo_->num_fps_, n_step_);
        Eigen::MatrixXd arm_0_q_vel = (GetVariables()->GetComponent("arm_0_joint_vel")->GetValues()).reshaped(dual_arm_->arm_0_->joint_num_, n_step_);
        Eigen::MatrixXd arm_1_q_vel = (GetVariables()->GetComponent("arm_1_joint_vel")->GetValues()).reshaped(dual_arm_->arm_1_->joint_num_, n_step_);

        Eigen::MatrixXd dlo_fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3*dlo_->num_fps_, n_step_+1);
        Eigen::MatrixXd arm_0_q = (GetVariables()->GetComponent("arm_0_joint_pos")->GetValues()).reshaped(dual_arm_->arm_0_->joint_num_, n_step_+1);
        Eigen::MatrixXd arm_1_q = (GetVariables()->GetComponent("arm_1_joint_pos")->GetValues()).reshaped(dual_arm_->arm_1_->joint_num_, n_step_+1);

        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3 * dlo_->num_fps_, n_step_);

        for(size_t t = 0; t < n_step_; t++){
            Eigen::MatrixXd jaco_all = CalcJacobianAll(dlo_fps_pos.col(t), arm_0_q.col(t), arm_1_q.col(t));
            Eigen::VectorXd dual_arm_q_vel = Utils::concatenateTwoVector(arm_0_q_vel.col(t), arm_1_q_vel.col(t));
            g.col(t) = dlo_fps_vel.col(t) - (jaco_all * dual_arm_q_vel);
        }

        return g.reshaped(3*dlo_->num_fps_ * n_step_, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        int fps_dim = 3 * dlo_->num_fps_;
        int arm_0_n_joint = dual_arm_->arm_0_->joint_num_;
        int arm_1_n_joint = dual_arm_->arm_1_->joint_num_;

        Eigen::MatrixXd dlo_fps_vel = (GetVariables()->GetComponent("dlo_fps_vel")->GetValues()).reshaped(fps_dim, n_step_);
        Eigen::MatrixXd arm_0_q_vel = (GetVariables()->GetComponent("arm_0_joint_vel")->GetValues()).reshaped(arm_0_n_joint, n_step_);
        Eigen::MatrixXd arm_1_q_vel = (GetVariables()->GetComponent("arm_1_joint_vel")->GetValues()).reshaped(arm_1_n_joint, n_step_);

        Eigen::MatrixXd dlo_fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(fps_dim, n_step_+1);
        Eigen::MatrixXd arm_0_q = (GetVariables()->GetComponent("arm_0_joint_pos")->GetValues()).reshaped(arm_0_n_joint, n_step_+1);
        Eigen::MatrixXd arm_1_q = (GetVariables()->GetComponent("arm_1_joint_pos")->GetValues()).reshaped(arm_1_n_joint, n_step_+1);

        if(var_set == "dlo_fps_vel"){
            for(size_t t = 0; t < n_step_; t++){
                for(int j = 0; j < fps_dim; j++){
                    jac_block.coeffRef(fps_dim*t + j, fps_dim*t + j) = 1.0;
                }
            }
        }
        if(var_set == "arm_0_joint_vel"){
            for(size_t t = 0; t < n_step_; t++){
                Eigen::MatrixXd jaco_all = CalcJacobianAll(dlo_fps_pos.col(t), arm_0_q.col(t), arm_1_q.col(t));
                for(int i = 0; i < fps_dim; i++){ // g_{t} / q0_vel_{t}
                    for(int j = 0; j < arm_0_n_joint; j++){
                        jac_block.coeffRef(fps_dim * t + i, arm_0_n_joint * t + j) = -jaco_all(i, j);
                    }
                }
            }
        }
        if(var_set == "arm_1_joint_vel"){
            for(size_t t = 0; t < n_step_; t++){
                Eigen::MatrixXd jaco_all = CalcJacobianAll(dlo_fps_pos.col(t), arm_0_q.col(t), arm_1_q.col(t));
                for(int i = 0; i < fps_dim; i++){ // g_{t} / q1_vel_{t}
                    for(int j = 0; j < arm_1_n_joint; j++){
                        jac_block.coeffRef(fps_dim * t + i, arm_1_n_joint * t + j) = -jaco_all(i, arm_0_n_joint + j);
                    }
                }
            }
        }
    }

private:
    int n_step_; // T
    DLO::Ptr dlo_;
    JacobianModel::Ptr dlo_jaco_model_;
    DualArm::Ptr dual_arm_;
};


/** -----------------------------------------------------------------------------------------
 * @brief constraint between DLO fps_pos and DLO edge points (线性插值得到)
 * g_{t} = edge_points_{t} - ((1-ratio) * fp_0_{t} + ratio * fp_1_{t}), t = 1,...,T
 */
class DLOEdgePointsConstraint: public ConstraintSet
{
public:
    DLOEdgePointsConstraint(
        const int n_step,
        const DLO::Ptr &dlo
    ): ConstraintSet(3 * (dlo->num_point_per_edge_ * (dlo->num_fps_-1)) * n_step, "dlo_edge_points_constraint")
    {
        n_step_ = n_step;
        dlo_ = dlo;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override{
        int num_fps = dlo_->num_fps_;
        int num_edge = dlo_->num_fps_ - 1;
        int n = dlo_->num_point_per_edge_;

        Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(num_fps * 3, n_step_+1);
        Eigen::MatrixXd edge_points = (GetVariables()->GetComponent("dlo_edge_points_pos")->GetValues()).reshaped(
            num_edge*n*3, n_step_);

        Eigen::MatrixXd edge_point_gt = Eigen::MatrixXd::Zero(num_edge*n*3, n_step_);
        
        for(size_t t = 0; t < n_step_; t++){
            for(size_t k = 0; k < num_edge; k++){
                Eigen::Vector3d fp_0 = fps_pos.col(t+1).block<3, 1>(3 * k, 0);
                Eigen::Vector3d fp_1 = fps_pos.col(t+1).block<3, 1>(3 * (k+1), 0);
                for(size_t i = 0; i < n; i++){
                    double ratio = double(i+1) / double(n+1);
                    edge_point_gt.col(t).block<3, 1>(k*(n*3) + i*3, 0) = (1.0 - ratio) * fp_0 + ratio * fp_1;
                }
            }
        }

        return (edge_points - edge_point_gt).reshaped(n_step_*num_edge*n*3, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows()); // default bound: 0.0 ~ 0.0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_edge_points_pos"){
            int num_edge = dlo_->num_fps_ - 1;
            int n = dlo_->num_point_per_edge_;
            for(size_t i = 0; i < n_step_*num_edge*n*3; i++){
                jac_block.coeffRef(i, i) = 1.0; // identity matrix
            }
        }
        if(var_set == "dlo_fps_pos"){
            int num_fps = dlo_->num_fps_;
            int num_edge = dlo_->num_fps_ - 1;
            int n = dlo_->num_point_per_edge_;

            for(size_t t = 0; t < n_step_; t++){
                for(size_t k = 0; k < num_edge; k++){
                    for(size_t i = 0; i < n; i++){
                        double ratio = double(i+1) / double(n+1);
                        for(size_t j = 0; j < 3; j++){
                            jac_block.coeffRef(t*(num_edge*n*3) + k*(n*3) + i*3 + j, (t+1)*(num_fps*3) + k*3 + j) = -(1 - ratio); // d(p)/d(fp_0)
                            jac_block.coeffRef(t*(num_edge*n*3) + k*(n*3) + i*3 + j, (t+1)*(num_fps*3) + (k+1)*3 + j) = -ratio; // d(p)/d(fp_1)
                        }
                    }
                }
            }
        }
    }

private:
    int n_step_; // T
    DLO::Ptr dlo_;
};


/** -----------------------------------------------------------------------------------------
 * @brief constain the minimal distance between DLO and world
 * g_{t} = distance_{t} - min_dist_thres_ >= 0, t = 1,...,T
 */
class DLOWorldDistanceConstraint: public ConstraintSet
{
public:
    DLOWorldDistanceConstraint(const int &n_step, const Scene::Ptr &scene): 
        ConstraintSet(scene->dlo_->num_fps_ * n_step, "dlo_world_dist_constraint")
    {
        n_step_ = n_step;
        scene_ = scene;
    }

    // ---------------------------
    void SetParams(const double min_dist_thres){
        min_dist_thres_ = min_dist_thres;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int num_fps = scene_->dlo_->num_fps_;
        Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3 * num_fps, n_step_+1);
        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(num_fps, n_step_);

        for(size_t t = 1; t <= n_step_; t++){
            VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(fps_pos.col(t));
            for(size_t i = 0; i < num_fps; i++){
                g.col(t-1)(i) = scene_->getPrecisePointDist(points[i]) - min_dist_thres_;
            }
        }

        return g.reshaped(num_fps * n_step_, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows());
        for(size_t i = 0; i < GetRows(); i++) 
            b.at(i) = BoundGreaterZero; // >=0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_fps_pos"){
            int num_fps = scene_->dlo_->num_fps_;
            Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3 * num_fps, n_step_+1);

            for(size_t t = 1; t <= n_step_; t++){
                VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(fps_pos.col(t));

                for(size_t i = 0; i < points.size(); i++){
                    Eigen::Vector3d dist_gradient = scene_->getPointDistGradient(points[i]);
                    Eigen::Vector3d normalized_grad = dist_gradient.normalized();
                    for(size_t j = 0; j < 3; j++){
                        jac_block.coeffRef(num_fps*(t-1) + i, num_fps*3*t + 3*i + j) = normalized_grad(j); // g从t=1开始
                    }
                }
            }
        }
    }

private: 
    int n_step_; // T
    Scene::Ptr scene_;
    double min_dist_thres_ = 0.01;
}; 


/** -----------------------------------------------------------------------------------------
 * @brief constain the minimal distance between DLO edges and world
 * g_{t} = distance_{t} - min_dist_thres_ >= 0, t = 1,...,T
 */
class DLOEdgeWorldDistanceConstraint: public ConstraintSet
{
public:
    DLOEdgeWorldDistanceConstraint(const int &n_step, const Scene::Ptr &scene): 
        ConstraintSet(n_step * (scene->dlo_->num_fps_-1) * scene->dlo_->num_point_per_edge_, "dlo_edge_world_dist_constraint")
    {
        n_step_ = n_step;
        scene_ = scene;
    }

    // ---------------------------
    void SetParams(const double min_dist_thres){
        min_dist_thres_ = min_dist_thres;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int num_edge = scene_->dlo_->num_fps_ - 1;
        int n = scene_->dlo_->num_point_per_edge_;
        Eigen::MatrixXd edge_points = (GetVariables()->GetComponent("dlo_edge_points_pos")->GetValues()).reshaped(
            num_edge*n*3, n_step_);

        Eigen::MatrixXd g = Eigen::MatrixXd::Zero(num_edge * n, n_step_);

        for(size_t t = 0; t < n_step_; t++){
            VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(edge_points.col(t));
            for(size_t i = 0; i < points.size(); i++){
                g.col(t)(i) = scene_->getPrecisePointDist(points[i]) - min_dist_thres_;
            }
        }

        return g.reshaped(n_step_ * num_edge * n, 1);
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows());
        for(size_t i = 0; i < GetRows(); i++) 
            b.at(i) = BoundGreaterZero; // >=0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_edge_points_pos"){
            int num_edge = scene_->dlo_->num_fps_ - 1;
            int n = scene_->dlo_->num_point_per_edge_;
            Eigen::MatrixXd edge_points = (GetVariables()->GetComponent("dlo_edge_points_pos")->GetValues()).reshaped(
                num_edge*n*3, n_step_);

            for(size_t t = 0; t < n_step_; t++){
                VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(edge_points.col(t));

                for(size_t i = 0; i < points.size(); i++){
                    Eigen::Vector3d dist_gradient = scene_->getPointDistGradient(points[i]);
                    Eigen::Vector3d normalized_grad = dist_gradient.normalized();
                    for(size_t j = 0; j < 3; j++){
                        jac_block.coeffRef(t*points.size() + i, t*(points.size()*3) + i*3 + j) = normalized_grad(j);
                    }
                }
            }
        }
    }

private: 
    int n_step_; // T
    Scene::Ptr scene_;
    double min_dist_thres_ = 0.01;
}; 


/** -----------------------------------------------------------------------------------------
 * @brief constrain not to overstretch the DLO
 * g_{t} = dlo_length - two_end_dist_{t} - thres >= 0, t = 1,...,T
 */
class DLOOverstretchConstraint: public ConstraintSet
{
public:
    DLOOverstretchConstraint(const int &n_step, const DLO::Ptr &dlo):
        ConstraintSet(n_step, "dlo_overstretch_constraint")
    {
        n_step_ = n_step;
        dlo_ = dlo;
    }

    // ---------------------------
    void SetParams(const double thres){
        thres_ = thres;
    }

    // ---------------------------
    Eigen::VectorXd GetValues() const override
    {
        int num_fps = dlo_->num_fps_;
        Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3 * num_fps, n_step_+1);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n_step_);

        for(size_t t = 1; t <= n_step_; t++){
            VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(fps_pos.col(t));
            Eigen::Vector3d left_end_pos = points[0];
            Eigen::Vector3d right_end_pos = points[points.size() - 1];
            double distance = (left_end_pos - right_end_pos).norm();
            g(t-1) = dlo_->dlo_length_ - distance - thres_;
        }

        return g;
    }

    // ---------------------------
    VecBound GetBounds() const override{
        VecBound b(GetRows());
        for(size_t i = 0; i < GetRows(); i++) 
            b.at(i) = BoundGreaterZero; // >=0
        return b;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_fps_pos"){
            int num_fps = dlo_->num_fps_;
            Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3 * num_fps, n_step_+1);

            for(size_t t = 1; t <= n_step_; t++){
                VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(fps_pos.col(t));
                Eigen::Vector3d left_end_pos = points[0];
                Eigen::Vector3d right_end_pos = points[points.size() - 1];
                Eigen::Vector3d normalized_diff = (left_end_pos - right_end_pos).normalized();
                
                for(size_t j = 0; j < 3; j++){
                    jac_block.coeffRef(t-1, num_fps*3*t + 3*0 + j) = -normalized_diff(j);
                    jac_block.coeffRef(t-1, num_fps*3*t + 3*(num_fps-1) + j) = normalized_diff(j);
                }
            }
        }
    }

private:
    int n_step_;
    DLO::Ptr dlo_;
    double thres_ = 0.0;
};




} // end namespace 
} // end namespace 

#endif