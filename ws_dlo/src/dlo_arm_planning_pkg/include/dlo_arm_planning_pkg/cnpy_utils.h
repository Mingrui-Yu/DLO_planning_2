#ifndef DLO_ARM_PLANNING_CNPY_UTILS_H
#define DLO_ARM_PLANNING_CNPY_UTILS_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/planner/planner_base.h"

// in linux
#include <sys/io.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

namespace dlo_arm_planning_pkg
{

    // ------------------------------------------------------------
    class CnpyUtils
    {
    public:
        // ------------------------------------------------------------
        static void saveDLOStates(
            const std::string &dir,
            const std::vector<DLOState> &dlo_states)
        {
            size_t n = dlo_states.size();
            std::vector<double> all_dlo_fps_pos, all_dlo_end_0_quat, all_dlo_end_1_quat;

            for (auto &dlo_state : dlo_states)
            {
                Utils::stdVecExpand(all_dlo_fps_pos, Utils::eigenVectorXd2StdVector(dlo_state.fps_pos_));
                Utils::stdVecExpand(all_dlo_end_0_quat, Utils::eigenVectorXd2StdVector(dlo_state.end_quat_0_));
                Utils::stdVecExpand(all_dlo_end_1_quat, Utils::eigenVectorXd2StdVector(dlo_state.end_quat_1_));
            }

            Utils::createDirectory(dir);
            cnpy::npy_save(dir + "all_dlo_fps_pos.npy", &all_dlo_fps_pos[0], {n, static_cast<size_t>(dlo_states[0].fps_pos_.size())}, "w");
            cnpy::npy_save(dir + "all_dlo_end_0_quat.npy", &all_dlo_end_0_quat[0], {n, 4}, "w");
            cnpy::npy_save(dir + "all_dlo_end_1_quat.npy", &all_dlo_end_1_quat[0], {n, 4}, "w");
        }

        // ------------------------------------------------------------
        static void loadDLOStates(
            const std::string &dir,
            std::vector<DLOState> &dlo_states)
        {
            dlo_states.clear();

            cnpy::NpyArray all_dlo_fps_pos_arr = cnpy::npy_load(dir + "all_dlo_fps_pos.npy");
            cnpy::NpyArray all_dlo_end_0_quat_arr = cnpy::npy_load(dir + "all_dlo_end_0_quat.npy");
            cnpy::NpyArray all_dlo_end_1_quat_arr = cnpy::npy_load(dir + "all_dlo_end_1_quat.npy");

            int N = all_dlo_fps_pos_arr.shape[0];

            // check the shape
            if (all_dlo_fps_pos_arr.shape[1] % 3 != 0)
                ROS_ERROR_STREAM("loadPath(): dlo_fps_pos shape is wrong.");
            if (all_dlo_end_0_quat_arr.shape[1] != 4)
                ROS_ERROR_STREAM("loadPath(): dlo_end_0_quat shape is wrong.");
            if (all_dlo_end_1_quat_arr.shape[1] != 4)
                ROS_ERROR_STREAM("loadPath(): dlo_end_1_quat shape is wrong.");

            double *all_dlo_fps_pos_data = all_dlo_fps_pos_arr.data<double>();
            double *all_dlo_end_0_quat_data = all_dlo_end_0_quat_arr.data<double>();
            double *all_dlo_end_1_quat_data = all_dlo_end_1_quat_arr.data<double>();

            for (size_t i = 0; i < N; i++)
            {
                // DLO
                double *start = &all_dlo_fps_pos_data[i * all_dlo_fps_pos_arr.shape[1]];
                Eigen::VectorXd fps_pos = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + all_dlo_fps_pos_arr.shape[1]));

                start = &all_dlo_end_0_quat_data[i * all_dlo_end_0_quat_arr.shape[1]];
                Eigen::VectorXd end_0_quat = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + all_dlo_end_0_quat_arr.shape[1]));

                start = &all_dlo_end_1_quat_data[i * all_dlo_end_1_quat_arr.shape[1]];
                Eigen::VectorXd end_1_quat = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + all_dlo_end_1_quat_arr.shape[1]));

                DLOState dlo_state(fps_pos, end_0_quat, end_1_quat);
                dlo_state.updateDependentInfo();

                dlo_states.push_back(dlo_state);
            }
        }

        // -------------------------------------------------------
        static void loadPlanningGoal(
            const std::string &dir,
            DLOState &dlo_state,
            Eigen::VectorXd &arm_0_joint_pos,
            Eigen::VectorXd &arm_1_joint_pos,
            const DLO::Ptr &dlo,
            const DualArm::Ptr &dual_arm)
        {
            // dual arm joint pos
            cnpy::NpyArray arr = cnpy::npy_load(dir + "dual_arm_joint_pos.npy");
            if (arr.shape[0] != dual_arm->arm_0_->joint_num_ + dual_arm->arm_1_->joint_num_)
            {
                ROS_ERROR_STREAM("loadPlanningGoal(): the size of the loaded dual arm joint pos is wrong.");
            }
            double *loaded_data = arr.data<double>();
            std::vector<double> dual_arm_joint_pos(arr.shape[0]);
            for (size_t i = 0; i < arr.shape[0]; i++)
            {
                dual_arm_joint_pos[i] = loaded_data[i];
            }
            dual_arm->splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(dual_arm_joint_pos), arm_0_joint_pos, arm_1_joint_pos);

            // dlo shape -> DLOState
            arr = cnpy::npy_load(dir + "dlo_shape.npy");
            if (arr.shape[0] != dlo->num_fps_ * 3)
            {
                ROS_ERROR_STREAM("loadPlanningGoal(): the size of the loaded dlo shape is wrong.");
            }
            loaded_data = arr.data<double>();
            std::vector<double> dlo_shape(arr.shape[0]);
            for (size_t i = 0; i < arr.shape[0]; i++)
            {
                dlo_shape[i] = loaded_data[i];
            }

            Eigen::Isometry3d end_0_pose = dual_arm->arm_0_->getTcpPose(arm_0_joint_pos);
            Eigen::Isometry3d end_1_pose = dual_arm->arm_1_->getTcpPose(arm_1_joint_pos);
            Eigen::Vector4d end_0_quat = Eigen::Quaterniond(end_0_pose.rotation()).coeffs();
            Eigen::Vector4d end_1_quat = Eigen::Quaterniond(end_1_pose.rotation()).coeffs();

            dlo_state = DLOState(Utils::stdVector2EigenVectorXd(dlo_shape), end_0_quat, end_1_quat);
            dlo_state.updateDependentInfo();
        }

        // -------------------------------------------------------
        static void savePlanningGoal(
            const std::string &dir,
            const DLOState &dlo_state,
            const Eigen::VectorXd &arm_0_joint_pos,
            const Eigen::VectorXd &arm_1_joint_pos)
        {
            std::vector<double> dlo_fps_pos = Utils::eigenVectorXd2StdVector(dlo_state.fps_pos_);
            std::vector<double> dual_arm_joint_pos = Utils::concatenateTwoVector(Utils::eigenVectorXd2StdVector(arm_0_joint_pos),
                                                                                 Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

            Utils::createDirectory(dir);
            cnpy::npy_save(dir + "dlo_shape.npy", &dlo_fps_pos[0], {static_cast<size_t>(dlo_fps_pos.size())}, "w");
            cnpy::npy_save(dir + "dual_arm_joint_pos.npy", &dual_arm_joint_pos[0], {static_cast<size_t>(dual_arm_joint_pos.size())}, "w");
        }

        // -------------------------------------------------------
        static void savePath(
            const std::string &dir,
            const Path &path)
        {
            Utils::createDirectory(dir);

            size_t path_size = path.arm_0_path_.size();

            std::vector<double> arm_0_path, arm_1_path, dlo_fps_pos_path, dlo_end_0_quat_path, dlo_end_1_quat_path;

            for (size_t waypoint_idx = 0; waypoint_idx < path_size; waypoint_idx++)
            {
                Utils::stdVecExpand(arm_0_path, path.arm_0_path_[waypoint_idx]);
                Utils::stdVecExpand(arm_1_path, path.arm_1_path_[waypoint_idx]);

                auto &dlo_state = path.dlo_path_[waypoint_idx];
                Utils::stdVecExpand(dlo_fps_pos_path, Utils::eigenVectorXd2StdVector(dlo_state.fps_pos_));
                Utils::stdVecExpand(dlo_end_0_quat_path, Utils::eigenVectorXd2StdVector(dlo_state.end_quat_0_));
                Utils::stdVecExpand(dlo_end_1_quat_path, Utils::eigenVectorXd2StdVector(dlo_state.end_quat_1_));
            }

            cnpy::npy_save(dir + "arm_0_path.npy", &arm_0_path[0], {path_size, static_cast<size_t>(path.arm_0_path_[0].size())}, "w");
            cnpy::npy_save(dir + "arm_1_path.npy", &arm_1_path[0], {path_size, static_cast<size_t>(path.arm_1_path_[0].size())}, "w");

            cnpy::npy_save(dir + "dlo_fps_pos_path.npy", &dlo_fps_pos_path[0], {path_size, static_cast<size_t>(path.dlo_path_[0].fps_pos_.size())}, "w");
            cnpy::npy_save(dir + "dlo_end_0_quat_path.npy", &dlo_end_0_quat_path[0], {path_size, 4}, "w");
            cnpy::npy_save(dir + "dlo_end_1_quat_path.npy", &dlo_end_1_quat_path[0], {path_size, 4}, "w");
        }

        // -------------------------------------------------------
        static Path loadPath(
            const std::string &dir,
            const DLO::Ptr dlo = nullptr,
            const DualArm::Ptr dual_arm = nullptr)
        {
            Path path;

            cnpy::NpyArray arm_0_path_arr = cnpy::npy_load(dir + "arm_0_path.npy");
            cnpy::NpyArray arm_1_path_arr = cnpy::npy_load(dir + "arm_1_path.npy");
            cnpy::NpyArray dlo_fps_pos_path_arr = cnpy::npy_load(dir + "dlo_fps_pos_path.npy");
            cnpy::NpyArray dlo_end_0_quat_path_arr = cnpy::npy_load(dir + "dlo_end_0_quat_path.npy");
            cnpy::NpyArray dlo_end_1_quat_path_arr = cnpy::npy_load(dir + "dlo_end_1_quat_path.npy");

            int path_size = arm_0_path_arr.shape[0];

            // check the shape
            if (dlo)
            {
                if (dlo_fps_pos_path_arr.shape[1] != dlo->num_fps_ * 3)
                    ROS_ERROR_STREAM("Controller::loadPath(): dlo_fps_pos_path shape is wrong.");
            }
            if (dlo_end_0_quat_path_arr.shape[1] != 4)
                ROS_ERROR_STREAM("Controller::loadPath(): dlo_end_0_quat_path shape is wrong.");
            if (dlo_end_1_quat_path_arr.shape[1] != 4)
                ROS_ERROR_STREAM("Controller::loadPath(): dlo_end_1_quat_path shape is wrong.");
            if (dual_arm)
            {
                if (arm_0_path_arr.shape[1] != dual_arm->arm_0_->joint_num_)
                    ROS_ERROR_STREAM("Controller::loadPath(): arm_0_path shape is wrong.");
                if (arm_1_path_arr.shape[1] != dual_arm->arm_1_->joint_num_)
                    ROS_ERROR_STREAM("Controller::loadPath(): arm_1_path shape is wrong.");
            }

            double *arm_0_path_data = arm_0_path_arr.data<double>();
            double *arm_1_path_data = arm_1_path_arr.data<double>();
            double *dlo_fps_pos_path_data = dlo_fps_pos_path_arr.data<double>();
            double *dlo_end_0_quat_path_data = dlo_end_0_quat_path_arr.data<double>();
            double *dlo_end_1_quat_path_data = dlo_end_1_quat_path_arr.data<double>();

            for (size_t waypoint_idx = 0; waypoint_idx < path_size; waypoint_idx++)
            {
                // dual arm
                double *start = &arm_0_path_data[waypoint_idx * arm_0_path_arr.shape[1]];
                std::vector<double> arm_0_joint_pos(start, start + arm_0_path_arr.shape[1]);

                start = &arm_1_path_data[waypoint_idx * arm_1_path_arr.shape[1]];
                std::vector<double> arm_1_joint_pos(start, start + arm_1_path_arr.shape[1]);

                // DLO
                start = &dlo_fps_pos_path_data[waypoint_idx * dlo_fps_pos_path_arr.shape[1]];
                Eigen::VectorXd fps_pos = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + dlo_fps_pos_path_arr.shape[1]));

                start = &dlo_end_0_quat_path_data[waypoint_idx * dlo_end_0_quat_path_arr.shape[1]];
                Eigen::VectorXd end_0_quat = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + dlo_end_0_quat_path_arr.shape[1]));

                start = &dlo_end_1_quat_path_data[waypoint_idx * dlo_end_1_quat_path_arr.shape[1]];
                Eigen::VectorXd end_1_quat = Utils::stdVector2EigenVectorXd(std::vector<double>(start, start + dlo_end_1_quat_path_arr.shape[1]));

                DLOState dlo_state(fps_pos, end_0_quat, end_1_quat);
                dlo_state.updateDependentInfo();

                path.arm_0_path_.push_back(arm_0_joint_pos);
                path.arm_1_path_.push_back(arm_1_joint_pos);
                path.dlo_path_.push_back(dlo_state);
            }

            return path;
        }

    }; // end class

} // end namespace

#endif