#include "dlo_arm_planning_pkg/arm_reach_space.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    ArmReachSpace::ArmReachSpace(
        const Eigen::Vector3d &position_lb,
        const Eigen::Vector3d &position_ub,
        const double &position_resolution,
        const double &angle_resolution)
    {
        position_lb_ = position_lb;
        position_ub_ = position_ub;
        position_resolution_ = position_resolution;
        angle_resolution_ = angle_resolution;

        grid_shape_ = std::vector<size_t>(6);
        for (int i = 0; i < 3; i++)
        {
            grid_shape_[i] = int((position_ub_[i] - position_lb_[i]) / position_resolution_);
        }
        for (int i = 0; i < 3; i++)
        {
            grid_shape_[i + 3] = int((rpy_ub_[i] - rpy_lb_[i]) / angle_resolution_);
        }

        int total_size = grid_shape_[0] * grid_shape_[1] * grid_shape_[2] *
                         grid_shape_[3] * grid_shape_[4] * grid_shape_[5];

        ROS_INFO_STREAM("ArmReachSpace(): grid_space_shape: " << grid_shape_[0] << " "
                                                              << grid_shape_[1] << " " << grid_shape_[2] << " " << grid_shape_[3]
                                                              << " " << grid_shape_[4] << " " << grid_shape_[5] << ", total: " << total_size);

        grid_space_ = new int[total_size];
        for (size_t i = 0; i < total_size; i++)
        {
            grid_space_[i] = 0;
        }
    }

    // -------------------------------------------------------
    ArmReachSpace::ArmReachSpace(
        const std::string &file_dir)
    {
        cnpy::NpyArray params_npy = cnpy::npy_load(file_dir + params_file_name_);
        double *params = params_npy.data<double>();

        position_lb_ = Eigen::Vector3d(params[0], params[1], params[2]);
        position_ub_ = Eigen::Vector3d(params[3], params[4], params[5]);
        position_resolution_ = params[6];
        angle_resolution_ = params[7];

        grid_shape_ = std::vector<size_t>(6);
        for (int i = 0; i < 3; i++)
        {
            grid_shape_[i] = int((position_ub_[i] - position_lb_[i]) / position_resolution_);
        }
        for (int i = 0; i < 3; i++)
        {
            grid_shape_[i + 3] = int((rpy_ub_[i] - rpy_lb_[i]) / angle_resolution_);
        }
        int total_size = grid_shape_[0] * grid_shape_[1] * grid_shape_[2] *
                         grid_shape_[3] * grid_shape_[4] * grid_shape_[5];

        cnpy::NpyArray grid_space_npy = cnpy::npy_load(file_dir + grid_space_file_name_);

        ROS_ERROR_COND(grid_space_npy.shape.size() != 6, "ArmReachSpace(): the shape of loaded data is wrong.");
        for (int i = 0; i < grid_space_npy.shape.size(); i++)
        {
            ROS_ERROR_COND(grid_space_npy.shape[i] != int(grid_shape_[i]), "ArmReachSpace(): the shape of loaded data is wrong.");
        }

        grid_space_ = new int[total_size];
        for (size_t i = 0; i < total_size; i++)
        {
            grid_space_[i] = grid_space_npy.data<int>()[i];
        }
    }

    // ------------------------------------------------------------
    int ArmReachSpace::multiDimIndexToOneDimIndex(
        const std::vector<int> &multi_dim_index)
    {
        ROS_ERROR_COND(multi_dim_index.size() != 6, "ArmReachSpace::multiDimIndexToOneDimIndex(): input size is wrong.");

        int one_dim_index = 0;

        for (int i = 0; i < 6; i++)
        {
            int dimension = 1;
            for (int j = i + 1; j < 6; j++)
                dimension *= grid_shape_[j];

            one_dim_index += multi_dim_index[i] * dimension;
        }

        return one_dim_index;
    }

    // ------------------------------------------------------------
    std::vector<int> ArmReachSpace::pose2GridIndex(
        const Eigen::Isometry3d &pose)
    {
        Eigen::VectorXd pose_vec = Utils::isometryToPosAndRPYAngle(pose);

        std::vector<int> grid_index(6);

        for (int j = 0; j < 3; j++)
        {
            int idx = int((pose_vec[j] - position_lb_[j]) / position_resolution_);
            grid_index[j] = std::min(std::max(0, idx), int(grid_shape_[j]) - 1);
        }
        for (int j = 3; j < 6; j++)
        {
            int idx = int((pose_vec[j] - rpy_lb_[j - 3]) / angle_resolution_);
            grid_index[j] = std::min(std::max(0, idx), int(grid_shape_[j]) - 1);
        }

        return grid_index;
    }

    // ------------------------------------------------------------
    void ArmReachSpace::addReachablePoint(
        const Eigen::Isometry3d &pose)
    {
        std::vector<int> grid_idx = pose2GridIndex(pose);

        grid_space_[multiDimIndexToOneDimIndex(grid_idx)] = 1;
    }

    // ------------------------------------------------------------
    bool ArmReachSpace::checkReachability(
        const Eigen::Isometry3d &pose)
    {
        std::vector<int> grid_idx = pose2GridIndex(pose);

        return (grid_space_[multiDimIndexToOneDimIndex(grid_idx)] == 1);
    }

    // ------------------------------------------------------------
    void ArmReachSpace::saveAsFile(const std::string &files_dir)
    {
        std::vector<double> params;
        for (int i = 0; i < 3; i++)
            params.push_back(position_lb_[i]);
        for (int i = 0; i < 3; i++)
            params.push_back(position_ub_[i]);
        params.push_back(position_resolution_);
        params.push_back(angle_resolution_);

        cnpy::npy_save(files_dir + params_file_name_, &params[0], {1, params.size()}, "w");

        cnpy::npy_save(files_dir + grid_space_file_name_,
                       &grid_space_[0], {grid_shape_[0], grid_shape_[1], grid_shape_[2], grid_shape_[3], grid_shape_[4], grid_shape_[5]}, "w");
    }

} // end namespace