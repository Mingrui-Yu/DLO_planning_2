#ifndef DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_COSTS_H
#define DLO_ARM_PLANNING_MPC_MULTI_STEP_MPC_IFOPT_COSTS_H

#include "dlo_arm_planning_pkg/common_include.h"

#include <ifopt/cost_term.h>

namespace dlo_arm_planning_pkg{
namespace multi_step_mpc_ifopt{

using namespace ifopt;

/** -----------------------------------------------------------------------------------------
 * @brief dlo fps pos cost
 * cost = weight/2 * \sum_{t=1}^T{(x_{t} - x_d_{t})^2}
 */
class DLOFpsPosCost: public CostTerm
{
public:
    DLOFpsPosCost(const int &n_step, const DLO::Ptr &dlo): CostTerm("dlo_fps_pos_cost") 
    {
        n_step_ = n_step;
        dlo_ = dlo;
    }

    // ---------------------------
    void SetParams(
        const double weight,
        const std::vector<double> &step_weight, // t = 1,...T
        const Eigen::VectorXd &fps_pos_desired // x_d_{t = 1,...,T}
    ){
        weight_ = weight;
        step_weight_ = step_weight;
        fps_pos_desired_ = fps_pos_desired;
        if(fps_pos_desired.size() != 3*dlo_->num_fps_ * n_step_)
            ROS_ERROR_STREAM("DLOFpsPosCost(): the size of fps_pos_desired is wrong.");
    }

    // ---------------------------
    double GetCost() const override
    {
        if(fps_pos_desired_.size()==0 || step_weight_.size() == 0){
            ROS_WARN("DLOFpsPosCost(): haven't assigned fps_pos_desired.");
            return 1e10; // 若还未指定desired path，则返回一个无效的cost 
        }

        Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(3*dlo_->num_fps_, n_step_+1);
        Eigen::MatrixXd fps_pos_desired = fps_pos_desired_.reshaped(3*dlo_->num_fps_, n_step_); // t = 1,...,T

        double cost = 0.0;
        for(size_t t = 1; t <= n_step_; t++){
            cost += weight_/2.0 * step_weight_[t-1] * (fps_pos.col(t) - fps_pos_desired.col(t-1)).squaredNorm();
        }
        return cost;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == "dlo_fps_pos"){
            int fps_dim = 3*dlo_->num_fps_;
            Eigen::MatrixXd fps_pos = (GetVariables()->GetComponent("dlo_fps_pos")->GetValues()).reshaped(fps_dim, n_step_+1);
            Eigen::MatrixXd fps_pos_desired = fps_pos_desired_.reshaped(fps_dim, n_step_); // t = 1,...,T

            for(size_t t = 1; t <= n_step_; t++){
                for(int j = 0; j < fps_dim; j++){ // cost / x_{t}
                    jac_block.coeffRef(0, fps_dim * t + j) = weight_ * step_weight_[t-1] * (fps_pos.col(t)(j) - fps_pos_desired.col(t-1)(j));
                }
            }
        }
    }

private:
    int n_step_;
    DLO::Ptr dlo_;
    // parameters
    double weight_ = 1.0;
    std::vector<double> step_weight_;
    Eigen::VectorXd fps_pos_desired_ = Eigen::VectorXd::Zero(0);
};


/** -----------------------------------------------------------------------------------------
 * @brief arm joint pos cost
 * cost = weight/2 * \sum_{t=1}^{T}{(q_{t} - q_d_{t})^2}
 */
class ArmJointPosCost: public CostTerm
{
public:
    ArmJointPosCost(const int &n_step, const Arm::Ptr &arm)
        : CostTerm(arm->arm_group_name_ + "_joint_pos_cost") 
    {
        n_step_ = n_step;
        arm_ = arm;
        group_name_ = arm->arm_group_name_;
    }

    // ---------------------------
    void SetParams(
        const double weight,
        const std::vector<double> &step_weight, // t = 1,...T
        const Eigen::VectorXd &weights_for_joints,
        const Eigen::VectorXd &q_d // q_d_{t = 1,...,T}
    ){
        weight_ = weight;
        step_weight_ = step_weight;
        q_d_ = q_d;

        if(q_d.size() != arm_->joint_num_ * n_step_)
            ROS_ERROR_STREAM("ArmJointPosCost(): the size of q_d is wrong.");

        // vector to diagnal matrix
        weight_matrix_for_joints_ = Eigen::MatrixXd::Zero(weights_for_joints.size(), weights_for_joints.size());
        for(size_t i = 0; i < weights_for_joints.size(); i++)
            weight_matrix_for_joints_(i, i) = weights_for_joints(i);
    }

    // ---------------------------
    double GetCost() const override
    {
        if(q_d_.size()==0 || step_weight_.size() == 0){
            ROS_WARN("ArmJointPosCost(): haven't assigned q_d.");
            return 1e10; // 若还未指定desired path，则返回一个无效的cost 
        }

        Eigen::MatrixXd q = (GetVariables()->GetComponent(group_name_ + "_joint_pos")->GetValues()).reshaped(arm_->joint_num_, n_step_+1); // t = 0,...,T
        Eigen::MatrixXd q_d = q_d_.reshaped(arm_->joint_num_, n_step_); // t = 1,...,T

        double cost = 0.0;
        for(size_t t = 1; t <= n_step_; t++){
            cost += weight_/2.0 * step_weight_[t-1] 
                * ( weight_matrix_for_joints_ * (q.col(t) - q_d.col(t-1)) ).squaredNorm();
        }
        return cost;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == group_name_ + "_joint_pos"){
            auto &n_joint = arm_->joint_num_;
            Eigen::MatrixXd q = (GetVariables()->GetComponent(group_name_ + "_joint_pos")->GetValues()).reshaped(n_joint, n_step_+1); // t = 0,...,T
            Eigen::MatrixXd q_d = q_d_.reshaped(arm_->joint_num_, n_step_); // t = 1,...,T

            for(size_t t = 1; t <= n_step_; t++){
                for(int j = 0; j < n_joint; j++){
                    jac_block.coeffRef(0, n_joint * t + j) = weight_ * step_weight_[t-1]
                                                            * weight_matrix_for_joints_(j,j) * weight_matrix_for_joints_(j,j)
                                                            * (q.col(t)(j) - q_d.col(t-1)(j)); // cost / q_{t}(j)
                }
            }
        }
    }

private:
    int n_step_;
    Arm::Ptr arm_;
    std::string group_name_;
    // parameters
    double weight_ = 1.0;
    std::vector<double> step_weight_;
    Eigen::VectorXd q_d_;
    Eigen::MatrixXd weight_matrix_for_joints_;
};


/** -----------------------------------------------------------------------------------------
 * @brief arm joint vel cost
 * cost = weight/2 * \sum_{t=0}^{T-1}{(K * q_vel_{t})^2}
 */
class ArmJointVelCost: public CostTerm
{
public:
    ArmJointVelCost(const int &n_step, const Arm::Ptr &arm)
        : CostTerm(arm->arm_group_name_ + "_joint_vel_cost") 
    {
        n_step_ = n_step;
        arm_ = arm;
        group_name_ = arm->arm_group_name_;
        weight_matrix_for_joints_ = Eigen::MatrixXd::Identity(arm->joint_num_, arm->joint_num_);
    }

    // ---------------------------
    void SetParams(
        const double weight,
        const Eigen::VectorXd &weights_for_joints
    ){
        weight_ = weight;

        // vector to diagnal matrix
        weight_matrix_for_joints_ = Eigen::MatrixXd::Zero(weights_for_joints.size(), weights_for_joints.size());
        for(size_t i = 0; i < weights_for_joints.size(); i++)
            weight_matrix_for_joints_(i, i) = weights_for_joints(i);
    }

    // ---------------------------
    double GetCost() const override
    {
        Eigen::MatrixXd q_vel = (GetVariables()->GetComponent(group_name_ + "_joint_vel")->GetValues()).reshaped(arm_->joint_num_, n_step_);
        double cost = 0.0;
        for(size_t t = 0; t < n_step_; t++){
            cost += weight_/2.0 * (weight_matrix_for_joints_ * q_vel.col(t)).squaredNorm();
        }
        return cost;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == group_name_ + "_joint_vel"){
            Eigen::MatrixXd q_vel = (GetVariables()->GetComponent(group_name_ + "_joint_vel")->GetValues()).reshaped(arm_->joint_num_, n_step_);
            Eigen::MatrixXd weighted_q_vel = q_vel;

            for(size_t t = 0; t < n_step_; t++){
                weighted_q_vel.col(t) = weight_matrix_for_joints_.transpose() * weight_matrix_for_joints_ * q_vel.col(t);
                for(int j = 0; j < arm_->joint_num_; j++){
                    jac_block.coeffRef(0, arm_->joint_num_ * t + j) = weight_ *  weighted_q_vel.col(t)(j); // cost / q_vel_{t}(j)
                }
            }
        }
    }

private:
    int n_step_;
    Arm::Ptr arm_;
    std::string group_name_;
    // parameters
    double weight_ = 1.0;
    Eigen::MatrixXd weight_matrix_for_joints_;
};


/** -----------------------------------------------------------------------------------------
 * @brief arm joint acceleration cost
 * cost = weight/2 * \sum_{t=0}^{T-1}{(K * a_{t})^2}
 * a_{t} = (q_vel{t} - q_vel{t-1}) / delta_t
 */
class ArmJointAcceCost: public CostTerm
{
public:
    ArmJointAcceCost(const int &n_step, const Arm::Ptr &arm)
        : CostTerm(arm->arm_group_name_ + "_joint_vel_cost") 
    {
        n_step_ = n_step;
        arm_ = arm;
        group_name_ = arm->arm_group_name_;
        weight_matrix_for_joints_ = Eigen::MatrixXd::Identity(arm->joint_num_, arm->joint_num_);
        q_vel_last_ = Eigen::VectorXd::Zero(arm->joint_num_);
    }

    // ---------------------------
    void SetParams(
        const double weight,
        const Eigen::VectorXd &weights_for_joints,
        const Eigen::VectorXd &q_vel_last,
        const double delta_t
    ){
        weight_ = weight;

        // vector to diagnal matrix
        weight_matrix_for_joints_ = Eigen::MatrixXd::Zero(weights_for_joints.size(), weights_for_joints.size());
        for(size_t i = 0; i < weights_for_joints.size(); i++)
            weight_matrix_for_joints_(i, i) = weights_for_joints(i);

        q_vel_last_ = q_vel_last;
        delta_t_ = delta_t;
    }

    // ---------------------------
    double GetCost() const override
    {
        Eigen::MatrixXd q_vel = (GetVariables()->GetComponent(group_name_ + "_joint_vel")->GetValues()).reshaped(arm_->joint_num_, n_step_);

        double cost = 0.0;
        for(size_t t = 0; t < n_step_; t++){
            Eigen::VectorXd a_t;
            if(t == 0) a_t = (q_vel.col(t) - q_vel_last_) / delta_t_;
            else       a_t = (q_vel.col(t) - q_vel.col(t-1)) / delta_t_;

            cost += weight_/2.0 * (weight_matrix_for_joints_ * a_t).squaredNorm();
        }
        return cost;
    }

    // ---------------------------
    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override 
    {
        if(var_set == group_name_ + "_joint_vel"){
            Eigen::MatrixXd q_vel = (GetVariables()->GetComponent(group_name_ + "_joint_vel")->GetValues()).reshaped(arm_->joint_num_, n_step_);

            Eigen::MatrixXd acce = Eigen::MatrixXd::Zero(arm_->joint_num_, n_step_);
            for(int t = 0; t < n_step_; t++){
                if(t == 0) acce.col(t) = (q_vel.col(t) - q_vel_last_) / delta_t_;
                else       acce.col(t) = (q_vel.col(t) - q_vel.col(t-1)) / delta_t_;
            }
            
            for(size_t t = 0; t < n_step_; t++){
                Eigen::VectorXd grad; // cost / q_vel_{t}
                if(t < n_step_ - 1){
                    grad = weight_ / delta_t_ * 
                        weight_matrix_for_joints_.transpose() * weight_matrix_for_joints_ * (acce.col(t) - acce.col(t+1));
                }else{
                    grad = weight_ / delta_t_ * weight_matrix_for_joints_.transpose() * weight_matrix_for_joints_ * acce.col(t);
                }
                for(int j = 0; j < arm_->joint_num_; j++){
                    jac_block.coeffRef(0, arm_->joint_num_ * t + j) = grad(j);
                }
            }
        }
    }

private:
    int n_step_;
    Arm::Ptr arm_;
    std::string group_name_;
    // parameters
    double weight_ = 1.0;
    Eigen::MatrixXd weight_matrix_for_joints_;
    Eigen::VectorXd q_vel_last_;
    double delta_t_;
};




} // end namespace 
} // end namespace 

#endif