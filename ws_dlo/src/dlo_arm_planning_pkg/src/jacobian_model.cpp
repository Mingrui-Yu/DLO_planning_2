#include "dlo_arm_planning_pkg/jacobian_model.h"

namespace dlo_arm_planning_pkg{

// -------------------------------------------------------
JacobianModel::JacobianModel(
    const int &num_fps,
    const int &num_hidden_unit
){
    num_fps_ = num_fps;
    num_hidden_unit_ = num_hidden_unit;

    input_dim_ = 3*num_fps_ + 3 + 4 + 4;
    output_dim_ = num_fps_ * 3 * 12;

    centres_ = VecEigenVectorXd(num_hidden_unit_);
    sigma_inv_ = Eigen::VectorXd::Zero(num_hidden_unit_);
    weight_matrix_ = Eigen::MatrixXd::Zero(output_dim_, num_hidden_unit_);
}


// -------------------------------------------------------
JacobianModel::JacobianModel(
    const ros::NodeHandle& nh
): nh_(nh){
    loadParams();

    input_dim_ = 3*num_fps_ + 3 + 4 + 4;
    output_dim_ = num_fps_ * 3 * 12;

    centres_ = VecEigenVectorXd(num_hidden_unit_);
    sigma_inv_ = Eigen::VectorXd::Zero(num_hidden_unit_);
    weight_matrix_ = Eigen::MatrixXd::Zero(output_dim_, num_hidden_unit_);
}


// -------------------------------------------------------
void JacobianModel::loadParams()
{
    std::string param_name;

    param_name = "dlo_configs/num_fps";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, num_fps_);

    param_name = "dlo_jacobian_model_configs/offline_model_dir";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, offline_model_dir_);

    param_name = "dlo_jacobian_model_configs/offline_model_name";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, offline_model_name_);

    param_name = "dlo_jacobian_model_configs/rbfn_num_hidden_unit";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, num_hidden_unit_);

    param_name = "dlo_jacobian_model_configs/online_learning/window_time";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_window_time_);

    param_name = "dlo_jacobian_model_configs/online_learning/min_valid_fps_vel";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_min_valid_fps_vel_);

    param_name = "dlo_jacobian_model_configs/online_learning/max_valid_fps_vel";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_max_valid_fps_vel_);

    param_name = "dlo_jacobian_model_configs/online_learning/fps_vel_normalize_thres";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_fps_vel_thres_);

    param_name = "dlo_jacobian_model_configs/online_learning/learning_rate";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_learning_rate_);

    param_name = "dlo_jacobian_model_configs/online_learning/update_rate";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_update_rate_);

    param_name = "controller_configs/control_rate";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, online_data_rate_);
}


// -------------------------------------------------------
void JacobianModel::loadAllWeights(
    const std::string &dir
){
    // load the centres
    std::string data_file_name = dir + "centres.npy";
    cnpy::NpyArray data_npy = cnpy::npy_load(data_file_name);

    if((data_npy.shape[0] != num_hidden_unit_) || (data_npy.shape[1] != input_dim_)){
        std::cerr << "The shape of loaded centres " << data_npy.shape[0] << ", " << data_npy.shape[1] << " is wrong." << std::endl;
    }

    float* loaded_data = data_npy.data<float>();
    for(size_t i = 0; i < num_hidden_unit_; i++){
        centres_[i] = Eigen::VectorXd::Zero(input_dim_);
        for(size_t j = 0; j < input_dim_; j++){
            centres_[i](j) = loaded_data[i * input_dim_ + j];
        }
    }

    // load the sigmas
    data_file_name = dir + "sigmas.npy";
    data_npy = cnpy::npy_load(data_file_name);

    if(data_npy.shape[0] != num_hidden_unit_){
        std::cerr << "The shape of loaded sigmas " << data_npy.shape[0] << " is wrong." << std::endl;
    }

    loaded_data = data_npy.data<float>();
    for(size_t i = 0; i < num_hidden_unit_; i++){
        sigma_inv_(i) = loaded_data[i];
    }

    // load the weight_matrix
    data_file_name = dir + "weight_matrix.npy";
    data_npy = cnpy::npy_load(data_file_name);

    if((data_npy.shape[0] != output_dim_ || data_npy.shape[1] != num_hidden_unit_)){
        std::cerr << "The shape of loaded weight_matrix " << data_npy.shape[0] << ", " << data_npy.shape[1] << " is wrong." << std::endl;
    }

    loaded_data = data_npy.data<float>();
    for(size_t i = 0; i < output_dim_; i++){
        for(size_t j = 0; j < num_hidden_unit_; j++){
            weight_matrix_(i, j) = loaded_data[i * num_hidden_unit_ + j];
        }
    }

    b_load_weights_ = true;
    ROS_DEBUG_STREAM("JacobianModel::loadAllWeights(): finished");
}


// -------------------------------------------------------
Eigen::VectorXd JacobianModel::calcTheta(
    const Eigen::VectorXd &x
){
    if(!b_load_weights_) ROS_WARN("JacobianModel: haven't loaded network weights.");
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(num_hidden_unit_);

    for(size_t i = 0; i < num_hidden_unit_; i++){
        theta(i) = std::exp( - sigma_inv_(i) * sigma_inv_(i) * (x - centres_[i]).transpose() * (x - centres_[i]) );
    }

    return theta;
}


// -------------------------------------------------------
Eigen::VectorXd JacobianModel::calcN(
    const Eigen::VectorXd &theta
){
    if(!b_load_weights_) ROS_WARN("JacobianModel: haven't loaded network weights.");
    return weight_matrix_ * theta;
}


// -------------------------------------------------------
Eigen::VectorXd JacobianModel::relativeStateRepresentation(
    const DLOState &dlo_state
){
    Eigen::VectorXd relative_state = Eigen::VectorXd::Zero(3*num_fps_ + 3 + 4 + 4);

    for(size_t k = 1; k < num_fps_; k++){
        relative_state.block<3, 1>(3 * k, 0) = (dlo_state.getFpPos(k) - dlo_state.getFpPos(k - 1)).normalized();
    }

    relative_state.block<3, 1>(3 * num_fps_, 0) = (dlo_state.getRightEndPos() - dlo_state.getLeftEndPos()).normalized();

    relative_state.block<4, 1>(3*num_fps_ + 3, 0) = dlo_state.end_quat_0_;
    relative_state.block<4, 1>(3*num_fps_ + 3 + 4, 0) = dlo_state.end_quat_1_;

    return relative_state;
}


// -------------------------------------------------------
Eigen::MatrixXd JacobianModel::nnOutputToJacobianMatrix(
    const Eigen::VectorXd &nn_output,
    const double &dlo_length
){
    if(!b_load_weights_) ROS_WARN("JacobianModel: haven't loaded network weights.");

    /** reshaped():
     * https://eigen.tuxfamily.org/dox/group__TutorialReshape.html
     * require eigen >= 3.4.0
     * column-major
     * assigning a reshaped matrix to itself is currently not supported: 
     */
    Eigen::MatrixXd jaco_matrix = Eigen::MatrixXd::Zero(3*num_fps_, 12);
    for(size_t k = 0; k < num_fps_; k++){
        jaco_matrix.block<3, 12>(k * 3, 0) = nn_output.block<12*3, 1>(k * (12*3), 0).reshaped(3, 12);
    }

    jaco_matrix.col(3) *= dlo_length;
    jaco_matrix.col(4) *= dlo_length;
    jaco_matrix.col(5) *= dlo_length;
    jaco_matrix.col(9) *= dlo_length;
    jaco_matrix.col(10) *= dlo_length;
    jaco_matrix.col(11) *= dlo_length;

    return jaco_matrix;
}


// -------------------------------------------------------
Eigen::MatrixXd JacobianModel::calcJacobianMatrix(
    const DLOState &dlo_state,
    const double &dlo_length
){
    // ROS_DEBUG_STREAM("dlo_state.fps_pos_: " << dlo_state.fps_pos_.transpose());
    // ROS_DEBUG_STREAM("dlo_state.end_quat_0_: " << dlo_state.end_quat_0_.transpose());
    // ROS_DEBUG_STREAM("dlo_state.end_quat_1_: " << dlo_state.end_quat_1_.transpose());

    if(!b_load_weights_) ROS_WARN("JacobianModel: haven't loaded network weights.");

    Eigen::VectorXd relative_state = relativeStateRepresentation(dlo_state);
    // ROS_DEBUG_STREAM("relative_state: " << relative_state.transpose());

    Eigen::VectorXd theta = calcTheta(relative_state);
    // ROS_DEBUG_STREAM("theta: " << theta.transpose());

    Eigen::VectorXd nn_output = calcN(theta);
    // ROS_DEBUG_STREAM("nn_output: " << nn_output.transpose());
    
    return nnOutputToJacobianMatrix(nn_output, dlo_length);
}


// -------------------------------------------------------
void JacobianModel::onlineUpdating(
    const OnlineData &online_data,
    const double &dlo_length
){
    ROS_ERROR_COND(online_window_time_ == 0.0, "JacobianModel::onlineUpdating(): haven't specified the online learning parameters.");
    if(!b_load_weights_) ROS_WARN("JacobianModel: haven't loaded network weights.");

    std::chrono::steady_clock::time_point t1, t2;
    std::chrono::duration<double> time_used;

    double online_dataset_size = online_window_time_ * online_data_rate_;
    double epsilon_v = online_fps_vel_thres_;
    double learning_rate = online_learning_rate_;

    // 对于速度过大的数据，剔除。不进行更新。
    if(online_data.fps_vel.norm() > online_max_valid_fps_vel_ || online_data.fps_vel.norm() < online_min_valid_fps_vel_){
        return;
    }

    // 添加新数据，并维持 online_dataset 的 size
    online_dataset_.push_back(online_data);
    if(online_dataset_.size() > online_dataset_size){
        online_dataset_.erase(online_dataset_.begin()); // 删除第一个元素
    }

    Eigen::VectorXd T = Eigen::VectorXd::Zero(12);
    T << 1, 1, 1, dlo_length, dlo_length, dlo_length, 1, 1, 1, dlo_length, dlo_length, dlo_length;

    // ------------------------- updating ------------------------------
    // t1 = std::chrono::steady_clock::now();

    Eigen::VectorXd sum, relative_state, theta, nn_output, pred_error;
    Eigen::MatrixXd jaco;
    VecEigenVectorXd dataset_pred_error(online_dataset_.size());
    VecEigenVectorXd dataset_theta(online_dataset_.size());
    std::vector<double> dataset_n_v(online_dataset_.size());

    // online update rate (Hz)
    for(size_t iter = 0; iter < (online_update_rate_ / online_data_rate_); iter++){ // 保证在使用更大 learning rate 的情况下，能收敛而不发散
        // 计算 prediction error 和其他相关项
        for(size_t t = 0; t < online_dataset_.size(); t++){
            auto &data = online_dataset_[t]; 

            relative_state = relativeStateRepresentation(data.dlo_state);
            theta = calcTheta(relative_state);
            nn_output = calcN(theta);
            jaco = nnOutputToJacobianMatrix(nn_output, dlo_length);
            pred_error = data.fps_vel - jaco * data.ends_vel;
            double n_v = data.fps_vel.norm() >= epsilon_v ? data.fps_vel.norm() : epsilon_v; // velocity normalization factor

            dataset_pred_error[t] = pred_error;
            dataset_theta[t] = theta;
            dataset_n_v[t] = n_v;
        }
        // 更新 weights (根据 TRO 论文公式)
        for(size_t k = 0; k < num_fps_; k++){
            for(size_t i = 0; i < 12; i++){
                for(size_t j = 0; j < 3; j++){
                    sum = Eigen::VectorXd::Zero(num_hidden_unit_);
                    for(size_t t = 0; t < online_dataset_.size(); t++){
                        auto &data = online_dataset_[t]; 
                        auto &theta = dataset_theta[t];
                        auto &pred_error = dataset_pred_error[t];
                        double &n_v = dataset_n_v[t];
                        sum += theta * (T(i) * (data.ends_vel(i) / n_v) * (pred_error(3*k + j) / n_v));
                    }
                    weight_matrix_.block((3*12)*k + 3*i + j, 0, 1, num_hidden_unit_)/*W_kij*/ += 
                        (1.0 / online_update_rate_ * learning_rate / online_dataset_size) * sum.transpose();
                }
            }
        }
    }
    ROS_DEBUG_STREAM("JacobianModel::onlineUpdating(): finished.");

    // t2 = std::chrono::steady_clock::now();    
    // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << std::setprecision(6) << "updating time cost: " << time_used.count() << std::endl;
}




} // end namespace








// -------------------------------------------- backup ----------------------------------------------------


// // ------------------------- testing ------------------------------
    // {
    //     for(size_t t = 0; t < online_dataset_.size(); t++){
    //         auto &data = online_dataset_[t]; 

    //         Eigen::VectorXd relative_state = relativeStateRepresentation(data.dlo_state);
    //         Eigen::VectorXd theta = calcTheta(relative_state);
    //         Eigen::VectorXd nn_output = calcN(theta);
    //         Eigen::MatrixXd jaco = nnOutputToJacobianMatrix(nn_output, dlo_length);
    //         Eigen::VectorXd pred_error = data.fps_vel - jaco * data.ends_vel;
    //         double n_v = data.fps_vel.norm() >= epsilon_v ? data.fps_vel.norm() : epsilon_v; // velocity normalization factor

    //         dataset_pred_error[t] = pred_error;
    //         dataset_theta[t] = theta;
    //         dataset_n_v[t] = n_v;
    //     }

    //     Eigen::MatrixXd weight_matrix_1 = weight_matrix_;
    //     Eigen::MatrixXd weight_matrix_2 = weight_matrix_;

    //     t1 = std::chrono::steady_clock::now();

    //     Eigen::VectorXd sum;
    //     for(size_t k = 0; k < num_fps_; k++){
    //         for(size_t i = 0; i < 12; i++){
    //             for(size_t j = 0; j < 3; j++){
    //                 sum = Eigen::VectorXd::Zero(num_hidden_unit_);
    //                 for(size_t t = 0; t < online_dataset_.size(); t++){
    //                     auto &data = online_dataset_[t]; 
    //                     auto &theta = dataset_theta[t];
    //                     auto &pred_error = dataset_pred_error[t];
    //                     double &n_v = dataset_n_v[t];
    //                     sum += theta * (T(i) * (data.ends_vel(i) / n_v) * (pred_error(3*k + j) / n_v));
    //                 }
    //                 weight_matrix_1.block((3*12)*k + 3*i + j, 0, 1, num_hidden_unit_)/*W_kij*/ += 
    //                     (1.0 / online_update_rate_ * learning_rate / online_dataset_size) * sum.transpose();
    //             }
    //         }
    //     }

    //     t2 = std::chrono::steady_clock::now();    
    //     time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    //     std::cout << std::setprecision(6) << "1 cost: " << time_used.count() << std::endl;

    //     t1 = std::chrono::steady_clock::now();

    //     Eigen::MatrixXd sum_2;
    //     for(size_t k = 0; k < num_fps_; k++){
    //         for(size_t i = 0; i < 12; i++){
    //             sum_2 = Eigen::MatrixXd::Zero(3, num_hidden_unit_);

    //             for(size_t t = 0; t < online_dataset_.size(); t++){
    //                 auto &data = online_dataset_[t]; 
    //                 auto &theta = dataset_theta[t];
    //                 auto &pred_error = dataset_pred_error[t];
    //                 double &n_v = dataset_n_v[t];

    //                 for(int j = 0; j < 3; j++){
    //                     sum_2.row(j) += theta.transpose() * T(i) * (data.ends_vel(i) / n_v) * (pred_error(3*k + j) / n_v);
    //                 }
    //             }
    //             weight_matrix_2.block((3*12)*k + 3*i, 0, 3, num_hidden_unit_)/*W_ki*/ += 
    //                 1.0 / online_update_rate_ * learning_rate / online_dataset_size * sum_2;
    //         }
    //     }

    //     t2 = std::chrono::steady_clock::now();    
    //     time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    //     std::cout << std::setprecision(6) << "2 cost: " << time_used.count() << std::endl;

    //     // std::cout << " ------------------------------------ 1:" << std::endl;
    //     std::cout << (weight_matrix_1 - weight_matrix_2).norm() << std::endl;
    // }