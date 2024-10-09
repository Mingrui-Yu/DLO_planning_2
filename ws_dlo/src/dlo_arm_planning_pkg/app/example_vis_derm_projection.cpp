#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo.h"

using namespace dlo_arm_planning_pkg;

std::random_device rd;
// int random_seed = rd();
int random_seed = 1075911105;
std::default_random_engine Utils::rng(random_seed); // 设置随机种子，需要在 main() 函数之前进行初始化

/*
 * 用于论文作图：展示DERM projection 的效果。
 */

void saveDLOStateAsNpy(DLOState &dlo_state, std::string file_name)
{
    std::vector<double> data;
    int num_fps = dlo_state.fps_pos_.size() / 3;
    size_t one_state_dim = 3 * num_fps + 4 + 4 + 1;

    Utils::pushBackEigenVecToStdVec(data, dlo_state.fps_pos_);
    Utils::pushBackEigenVecToStdVec(data, dlo_state.end_quat_0_);
    Utils::pushBackEigenVecToStdVec(data, dlo_state.end_quat_1_);
    data.push_back(dlo_state.angle_b2m_(dlo_state.angle_b2m_.size() - 1));

    Utils::createDirectory(file_name);
    cnpy::npy_save(file_name, &data[0], {one_state_dim}, "w");
}

// ----------------------------------------------------------
Eigen::Vector3d getFpPos(const Eigen::VectorXd &fps_pos, int index)
{
    return fps_pos.block<3, 1>(3 * index, 0);
}

// -----------------------------------------------------------------
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // change the logger level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_STREAM("Random seed for this run: " << random_seed);

    std::string project_dir, param_name;
    param_name = "project_configs/project_dir";
    ROS_WARN_STREAM_COND(!nh.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh.getParam(param_name, project_dir);

    int env_dim = 3;
    int num_fps = 10;
    double dlo_length = 0.5;

    DLO::Ptr dlo = DLO::Ptr(new DLO(num_fps, dlo_length, env_dim));

    OptimizeOptions optimize_options;
    optimize_options.function_tolerance = 1e-8;
    DLOEnergyModelParams params;

    // sample a random DLO configuration
    DLOState dlo_state_rand = dlo->sampleCoarseStateType1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    // project the sample to a stable DLO configuration
    DLOState dlo_state_opt = dlo->optimizeShapeDerm(dlo_state_rand, params, optimize_options);

    saveDLOStateAsNpy(dlo_state_rand, project_dir + "data/code_test/vis_derm_projection/rand.npy");
    saveDLOStateAsNpy(dlo_state_opt, project_dir + "data/code_test/vis_derm_projection/opt.npy");
    std::cout << "Save results to " << project_dir + "data/code_test/vis_derm_projection/ ." << std::endl;

    return 0;
}