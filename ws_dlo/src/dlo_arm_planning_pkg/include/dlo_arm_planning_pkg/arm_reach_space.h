#ifndef DLO_ARM_PLANNING_ARMREACHSPACE_H
#define DLO_ARM_PLANNING_ARMREACHSPACE_H

#include "dlo_arm_planning_pkg/common_include.h"


namespace dlo_arm_planning_pkg{

class ArmReachSpace
{
public:
    typedef std::shared_ptr<ArmReachSpace> Ptr;

    ArmReachSpace(
        const Eigen::Vector3d &position_lb,
        const Eigen::Vector3d &position_ub,
        const double &position_resolution,
        const double &angle_resolution
    );

    ArmReachSpace(
        const std::string &file_dir
    );

    int multiDimIndexToOneDimIndex(
        const std::vector<int> &multi_dim_index
    );

    std::vector<int> pose2GridIndex(
        const Eigen::Isometry3d &pose // pos + rpy
    );

    void addReachablePoint(
        const Eigen::Isometry3d &pose
    );

    bool checkReachability(
        const Eigen::Isometry3d &pose
    );

    void saveAsFile(const std::string &files_dir);




public:
    Eigen::Vector3d position_lb_;
    Eigen::Vector3d position_ub_;
    double position_resolution_;
    Eigen::Vector3d rpy_lb_ = Eigen::Vector3d(-M_PI, -M_PI/2, -M_PI);
    Eigen::Vector3d rpy_ub_ = Eigen::Vector3d(M_PI, M_PI/2, M_PI);
    double angle_resolution_;

    std::vector<size_t> grid_shape_;

    int *grid_space_;
    // int ******grid_space_;

    const std::string grid_space_file_name_ = "reach_space.npy";
    const std::string params_file_name_ = "params.npy";


}; // end class

} // end namespace

#endif
