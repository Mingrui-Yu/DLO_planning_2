#include "dlo_arm_planning_pkg/scene.h"

// #include <Mathematics/Vector.h>
// #include <Mathematics/ArbitraryPrecision.h>
// #include <Mathematics/DistSegmentSegment.h>


namespace dlo_arm_planning_pkg{

// -------------------------------------------------------
Scene::Scene(
    const ros::NodeHandle& nh,
    const DLO::Ptr &dlo,
    const DualArm::Ptr &dual_arm
): nh_(nh)
{
    dlo_ = dlo;
    dual_arm_ = dual_arm;
    cd_fcl_ = std::make_shared<CollisionDetectionFCL>(nh_, dlo_, dual_arm_);
}


// -------------------------------------------------------
Scene::Scene(
    const ros::NodeHandle& nh,
    const DLO::Ptr &dlo,
    const DualArm::Ptr &dual_arm,
    const planning_scene::PlanningSceneConstPtr& planning_scene
): nh_(nh)
{
    dlo_ = dlo;
    dual_arm_ = dual_arm;
    cd_fcl_ = std::make_shared<CollisionDetectionFCL>(nh_, dlo_, dual_arm_);

    setPlanningScene(planning_scene);
}


// -------------------------------------------------------
void Scene::setPlanningScene(
    const planning_scene::PlanningSceneConstPtr& planning_scene
){
    planning_scene_ = planning_scene->diff();
    planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);

    collision_env_hybrid_ = dynamic_cast<const collision_detection::CollisionEnvHybrid*>(
        planning_scene_->getCollisionEnv("HYBRID").get());
    if (!collision_env_hybrid_){
        ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
        return;
    }

    distance_field_ = collision_env_hybrid_->getCollisionWorldDistanceField()->getWorldDistanceFieldConstPtr();

    closeGrippers();
}


// -------------------------------------------------------
void Scene::closeGrippers()
{
    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    double angle = 0.7;
    current_state.setJointPositions("gripper_0_finger_joint", &angle);
    current_state.setJointPositions("gripper_1_finger_joint", &angle);
    current_state.update();
}


// ------------------------------------------------------------
bool Scene::checkRobotCollisionByMoveit(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos
){   
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_0_->arm_group_name_);
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_1_->arm_group_name_);

    // 输入关节角
    current_state.setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
    current_state.setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

    current_state.update(); // 更新所有transformation，用于碰撞检测

    collision_detection::CollisionRequest collision_request;
    // collision_request.contacts = true;
    // collision_request.group_name = ""; // if empty, assume the complete robot
    collision_detection::CollisionResult collision_result;
    
    // self collision & collision with world
    collision_env_hybrid_->checkCollision(
        collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    // for(auto iter = collision_result.contacts.begin(); iter != collision_result.contacts.end(); iter++){
    //     std::cout << iter->first.first << " " << iter->first.second << std::endl;
    // }

    return collision_result.collision; 
    // true: 有碰撞; false: 无碰撞
}


// ------------------------------------------------------------
bool Scene::checkRobotCollision(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos,
    double min_dist_thres,
    bool b_self_collision
){   
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    // 判断是否与环境碰撞
    Eigen::VectorXd critical_points = cd_fcl_->getCriticalPointsPosVec(arm_0_joint_pos, arm_1_joint_pos);
    VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(critical_points);

    for(size_t i = 0; i < points.size(); i++){
        double dist = getPointDist(points[i]) - (cd_fcl_->objects_dual_arm_userdata_[i].geometry_params[0]); // (sphere_radius)
        if(dist <= 0.02){
            dist = getPrecisePointDist(points[i]) - (cd_fcl_->objects_dual_arm_userdata_[i].geometry_params[0]); // (sphere_radius)
        }
        if(dist <= min_dist_thres){
            return true;
        }
    }

    // 判断是否自碰撞
    if(b_self_collision){
        cd_fcl_->setRobotObjectsTransform(arm_0_joint_pos, arm_1_joint_pos);
        if(cd_fcl_->checkRobotSelfCollision()){
            // std::cout << "robot self collision. " << std::endl;
            return true;
        }
    }
    
    return false; 
    // true: 有碰撞; false: 无碰撞
}


// ------------------------------------------------------------
bool Scene::checkPointsAndWorldCollision(
    const VecEigenVec3 points,
    double min_dist_thres
){
    for(auto &point: points){
        double dist = getPointDist(point);
        if(dist <= 0.02){
            dist = getPrecisePointDist(point);
        }
        if(dist <= min_dist_thres){
            return true;
        }
    }
    return false;
    // true: 有碰撞; false: 无碰撞
}


// ------------------------------------------------------------
bool Scene::checkDLOAndWorldCollision(
    const DLOState &dlo_state,
    double min_dist_thres
){
    if(dlo_state.fps_pos_.size() == 0) ROS_ERROR("Scene::checkDLOAndWorldCollision(): the input dlo_state.fps_pos_ is empty.");

    // include the feature points and edge points
    int num_edge = dlo_->num_fps_ - 1;
    int n = dlo_->num_point_per_edge_;
    for(size_t k = 0; k < num_edge; k++){
        Eigen::Vector3d fp_0 = dlo_state.getFpPos(k);
        Eigen::Vector3d fp_1 = dlo_state.getFpPos(k+1);
        for(size_t i = 0; i <= n+1; i++){
            double ratio = double(i) / double(n+1);
            Eigen::Vector3d point = (1.0 - ratio) * fp_0 + ratio * fp_1;
            VecEigenVec3 points = {point};
            if(checkPointsAndWorldCollision(points, min_dist_thres))
                return true;
        }
    }

    // bool edges_collision = checkPointsAndWorldCollision(points, min_dist_thres);
    // if(edges_collision) return true;

    return false;
    // true: 有碰撞; false: 无碰撞
}



// // ------------------------------------------------------------
// // based on GeometricTools
// bool Scene::checkDLOSelfCollision(
//     const DLOState &dlo_state,
//     double min_dist_thres
// ){
//     using namespace gte;
    
//     // 建立 segments
//     int num_edges = dlo_->num_fps_ - 1;
//     std::vector<Segment<3, double>> segments(num_edges);
//     for(size_t k = 0; k < num_edges; k++){
//         for(int j = 0; j < 3; j++) segments[k].p[0][j] = dlo_state.getFpPos(k)[j];
//         for(int j = 0; j < 3; j++) segments[k].p[1][j] = dlo_state.getFpPos(k + 1)[j];
//     }

//     typedef DCPQuery<double, Segment<3, double>, Segment<3, double>> RobustQuery;
//     RobustQuery query{};
//     RobustQuery::Result result{};

//     for(size_t i = 0; i < num_edges; i++){
//         for(size_t j = i + 1; j < num_edges; j++){
//             if(j == i+1 || j == i-1) // 相邻的 edge 不进行检测
//                 continue; 
//             result = query.ComputeRobust(segments[i], segments[j]);
//             if(result.distance <= min_dist_thres){
//                 return true;
//             }
//         }
//     }

//     return false;
//     // true: 有碰撞; false: 无碰撞
// }


// ------------------------------------------------------------
void Scene::getPlanningSceneMsg(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos,
    moveit_msgs::PlanningScene &planning_scene_msg
){
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_0_->arm_group_name_);
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_1_->arm_group_name_);

    // 输入关节角
    current_state.setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
    current_state.setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));
    current_state.update(); // 更新所有transformation，用于碰撞检测

    planning_scene_->getPlanningSceneMsg(planning_scene_msg);
}


// ------------------------------------------------------------
double Scene::getPointDist(
    const Eigen::Vector3d &point
){
    return distance_field_->getDistance(point(0), point(1), point(2));
}


/** ------------------------------------------------------------
 * @brief The gradient is computed as a function of the distances of near-by cells.
 * The gradient is not normalized !
 * getDistanceGradient(): https://docs.ros.org/en/noetic/api/moveit_core/html/classdistance__field_1_1DistanceField.html#a5808c7630a3f194f2608fda16cc4546f
 */
Eigen::Vector3d Scene::getPointDistGradient(
    const Eigen::Vector3d &point
){
    Eigen::Vector3d grad;
    bool in_bounds;
    double dist = distance_field_->getDistanceGradient(point(0), point(1), point(2), grad(0), grad(1), grad(2), in_bounds);

    if(!in_bounds) 
        grad = Eigen::Vector3d::Zero();

    return grad;
}


/** ------------------------------------------------------------
 * @brief use trilinear interpolation to get continous distance
 * time cost: about 5e-7s
 */
double Scene::getPrecisePointDist(
    const Eigen::Vector3d &point
){
    Eigen::Vector3i cell_idx;
    distance_field_->worldToGrid(point(0), point(1), point(2), cell_idx(0), cell_idx(1), cell_idx(2));
    Eigen::Vector3d cell_center_pos;
    distance_field_->gridToWorld(cell_idx(0), cell_idx(1), cell_idx(2), cell_center_pos(0), cell_center_pos(1), cell_center_pos(2));

    // 计算相邻的8个顶点对应的索引 (2^3个)
    std::vector<std::vector<int> > v_idx(3);
    for(int j = 0; j < 3; j++){
        if(cell_center_pos(j) < point(j)){
            v_idx[j] = std::vector<int>{cell_idx(j), cell_idx(j) + 1};
        }else{
            v_idx[j] = std::vector<int>{cell_idx(j) - 1, cell_idx(j)};
        }
    }

    // 计算每个顶点的坐标 及 离障碍物的距离
    std::vector<std::vector<std::vector<double>>> distances(2);
    std::vector<std::vector<std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >>> positions(2);

    for(int i = 0; i < 2; i++){
        distances[i] = std::vector<std::vector<double>>(2);
        positions[i] = std::vector<std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >>(2);

        for(int j = 0; j < 2; j++){
            distances[i][j] = std::vector<double>(2);
            positions[i][j] = std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >(2);

            for(int k = 0; k < 2; k++){
                double dist = distance_field_->getDistance(v_idx[0][i], v_idx[1][j], v_idx[2][k]);
                Eigen::Vector3d position;
                distance_field_->gridToWorld(v_idx[0][i], v_idx[1][j], v_idx[2][k], position(0), position(1), position(2));

                distances[i][j][k] = dist;
                positions[i][j][k] = position;
            }
        }
    }

    // trilinear interpolation
    Eigen::Vector3d point_0 = positions[0][0][0];
    Eigen::Vector3d point_1 = positions[1][1][1];
    double x_d = (point(0) - point_0(0)) / (point_1(0) - point_0(0));
    double y_d = (point(1) - point_0(1)) / (point_1(1) - point_0(1));
    double z_d = (point(2) - point_0(2)) / (point_1(2) - point_0(2));

    double c00 = distances[0][0][0] * (1 - x_d) + distances[1][0][0] * x_d;
    double c01 = distances[0][0][1] * (1 - x_d) + distances[1][0][1] * x_d;
    double c10 = distances[0][1][0] * (1 - x_d) + distances[1][1][0] * x_d;
    double c11 = distances[0][1][1] * (1 - x_d) + distances[1][1][1] * x_d;
   
    double c0 = c00 * (1 - y_d) + c10 * y_d;
    double c1 = c01 * (1 - y_d) + c11 * y_d;

    double c = c0 * (1 - z_d) + c1 * z_d;
    
    return c;
}


// ------------------------------------------------------------
void Scene::minDistBetweenPointsAndWorld(
    const VecEigenVec3 points,
    double &min_dist,
    Eigen::Vector3d &min_point_pos,
    int &min_point_index
){
    min_point_index = 0;
    min_dist = 1e10;
    for(size_t i = 0; i < points.size(); i++){
        Eigen::Vector3d point = points[i];
        // double dist = getPointDist(point);
        double dist = getPrecisePointDist(point);

        if(dist < min_dist){
            min_dist = dist;
            min_point_index = i;
        }
    }

    min_point_pos = points[min_point_index];
}


// ------------------------------------------------------------
double Scene::minDistBetweenDLOAndWorld(
    const DLOState &dlo_state
){
    return minDistBetweenDLOAndWorld(dlo_state.fps_pos_);
}


// ------------------------------------------------------------
double Scene::minDistBetweenDLOAndWorld(
    const Eigen::VectorXd &fps_pos
){
    if(fps_pos.size() == 0) ROS_ERROR_STREAM("Scene::minDistBetweenDLOAndWorld(): empty input fps_pos");

    // include the feature points
    VecEigenVec3 points = Utils::eigenVectorXd2StdVecEigenVec3(fps_pos);
    
    int num_edge = dlo_->num_fps_ - 1;
    int n = dlo_->num_point_per_edge_;
    points.reserve(dlo_->num_fps_ + num_edge * n);

    // include the edge points
    for(size_t k = 0; k < num_edge; k++){
        Eigen::Vector3d fp_0 = fps_pos.block<3, 1>(3*k, 0);
        Eigen::Vector3d fp_1 = fps_pos.block<3, 1>(3*(k+1), 0);
        for(size_t i = 1; i <= n; i++){
            double ratio = double(i) / double(n+1);
            points.push_back((1.0-ratio) * fp_0 + ratio * fp_1);
        }
    }

    // double resolution = dlo_edge_resolution_;
    // VecEigenVec3 points;
    // points.reserve(dlo_->dlo_length_ / resolution);
    
    // // add points along the DLO edge
    // for (int k = 0; k < dlo_->num_fps_ - 1; k++){
    //     Eigen::Vector3d edge = feature_points[k+1] - feature_points[k];
    //     double edge_length = edge.norm();
    //     for(double s = 0.0; s < edge_length; s += resolution){
    //         Eigen::Vector3d point = feature_points[k] + s * edge.normalized();
    //         points.push_back(point);
    //     }
    // }
    // points.push_back(feature_points[feature_points.size() - 1]);

    // calculate the min dist
    double min_dist;
    Eigen::Vector3d min_point_pos;
    int min_point_idx;
    minDistBetweenPointsAndWorld(points, min_dist, min_point_pos, min_point_idx);

    std::cout << "min_point_idx: " << min_point_idx << std::endl;
    std::cout << "min_point_pos: " << min_point_pos.transpose() << std::endl;
    std::cout << "min_dist: " << min_dist << std::endl;

    return min_dist;
}


// ------------------------------------------------------------
double Scene::minDistBetweenRobotAndWorld(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos
){   
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_0_->arm_group_name_);
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_1_->arm_group_name_);

    // 输入关节角
    current_state.setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
    current_state.setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

    current_state.update(); // 更新所有transformation，用于碰撞检测

    collision_detection::CollisionRequest collision_request;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;
    
    // self collision & collision with world
    collision_env_hybrid_->checkCollision(
        collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    return collision_result.distance; 

    return 0;
}


} // end namespace