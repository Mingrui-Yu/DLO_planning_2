#ifndef DLO_ARM_PLANNING_JACOBIAN_MODEL_H
#define DLO_ARM_PLANNING_JACOBIAN_MODEL_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"

namespace dlo_arm_planning_pkg{

// -------------------------------------------------------
struct OnlineData
{
    DLOState dlo_state;
    Eigen::VectorXd fps_vel;
    Eigen::VectorXd ends_vel;
};

// -------------------------------------------------------
class JacobianModel
{
public:
    typedef std::shared_ptr<JacobianModel> Ptr;

    JacobianModel(
        const int &num_fps,
        const int &num_hidden_unit
    );

    // -------------------------------------------------------
    JacobianModel(
        const ros::NodeHandle& nh
    );

    void loadParams();

    void loadAllWeights(
        const std::string &dir
    );

    // layer 1
    Eigen::VectorXd calcTheta(
        const Eigen::VectorXd &x
    );

    // layer 2
    Eigen::VectorXd calcN(
        const Eigen::VectorXd &theta
    );

    Eigen::VectorXd relativeStateRepresentation(
        const DLOState &dlo_state
    );

    Eigen::MatrixXd nnOutputToJacobianMatrix(
        const Eigen::VectorXd &nn_output,
        const double &dlo_length
    );

    Eigen::MatrixXd calcJacobianMatrix(
        const DLOState &dlo_state,
        const double &dlo_length
    );

    void onlineUpdating(
        const OnlineData &data,
        const double &dlo_length
    );


public:
    ros::NodeHandle nh_;

    // network architecture
    int num_fps_;
    int num_hidden_unit_;
    int input_dim_, output_dim_;
    // model path
    std::string offline_model_dir_;
    std::string offline_model_name_;
    // network parameters
    bool b_load_weights_ = false;
    VecEigenVectorXd centres_;
    Eigen::VectorXd sigma_inv_;
    Eigen::MatrixXd weight_matrix_;

    // online updating
    double online_window_time_ = 0.0;
    double online_min_valid_fps_vel_;
    double online_max_valid_fps_vel_;
    double online_fps_vel_thres_;
    double online_learning_rate_;
    double online_data_rate_ ; // Hz
    double online_update_rate_; // Hz

    int online_dataset_size_;
    std::vector<OnlineData> online_dataset_;
};

} // end namespace

#endif