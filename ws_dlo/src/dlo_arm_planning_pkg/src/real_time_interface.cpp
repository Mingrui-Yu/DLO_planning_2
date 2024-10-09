#include "dlo_arm_planning_pkg/real_time_interface.h"
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

namespace dlo_arm_planning_pkg{

// -------------------------------------------------------
RealTimeInterface::RealTimeInterface(
    const ros::NodeHandle& nh,
    const DLO::Ptr &dlo,
    const DualArm::Ptr &dual_arm
): nh_(nh)
{
    dual_arm_ = dual_arm;
    dlo_ = dlo;

    loadParams();

    // planning scene monitor (synchronous with /move_group/monitored_planning_scene)
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
        new robot_model_loader::RobotModelLoader(dual_arm_->robot_description_name_));
    psm_ = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    psm_->requestPlanningSceneState("/get_planning_scene");

    // real-time observation subscriber
    robot_obs_sub_ = nh_.subscribe("state/dual_arm/joint_states", 1, &RealTimeInterface::robotObservationCb, this);
    dlo_obs_sub_ = nh_.subscribe("state/dlo/raw_state", 1, &RealTimeInterface::dloObservationCb, this);

    // client
    grasp_dlo_ends_client_ = nh_.serviceClient<std_srvs::Trigger>("/control/dual_gripper/grasp_dlo_ends");
    unity_reset_scene_client_ = nh_.serviceClient<std_srvs::Trigger>("/unity/reset_scene");
    clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");

    // publisher for the control command
    arm_0_joint_traj_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>("/control/arm_0/joint_trajectory_command", 1);
    arm_1_joint_traj_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>("/control/arm_1/joint_trajectory_command", 1);
    dual_arm_joint_traj_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>("/control/dual_arm/joint_trajectory_command", 1);
    dual_arm_joint_vel_pub_ = nh_.advertise<my_msgs::VectorStamped>("/control/dual_arm/joint_vel_command", 1);

    dlo_goal_pub_ = nh_.advertise<my_msgs::VectorStamped>("/state/dlo/desired_shape", 1);

    video_record_command_pub_ = nh_.advertise<std_msgs::String>("/record_video_command", 1);

    planning_visualizer_ = std::make_shared<Visualize>(nh_, dual_arm_, "/planning");
    rt_visualizer_ = std::make_shared<Visualize>(nh_, dual_arm_, "/real_time");
}


// -------------------------------------------------------
RealTimeInterface::~RealTimeInterface(){
    dlo_obs_sub_.shutdown(); // 否则会在程序运行到最后的时候报各种奇怪的错，推测是因为在 rt_visualizer_->publishDLOState() 的过程中，rt_visualizer_被删除了。
    robot_obs_sub_.shutdown();
}


// -------------------------------------------------------
void RealTimeInterface::loadParams()
{
    std::string param_name;
    param_name = "env_configs/sim_or_real";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, sim_or_real_);
}


// -------------------------------------------------------
void RealTimeInterface::waitForLatestObservation()
{
    while ( ros::ok() && (ros::Time::now() - get_latest_observation_time_).toSec() > 1e-3 ){
        ROS_DEBUG_ONCE("RealTimeInterface::waitForLatestObservation(): wait for latest observations");
    }
    // ROS_DEBUG_STREAM("RealTimeInterface::waitForLatestObservation(): delay: " << (ros::Time::now() - dlo_rt_obs_.header.stamp).toSec());
    return;
}

// -------------------------------------------------------
void RealTimeInterface::waitForNextObservation(
    const int dlo_last_seq,
    const int robot_last_seq
){
    while ( ros::ok() && dlo_rt_obs_.header.seq <= dlo_last_seq && robot_rt_obs_.header.seq <= robot_last_seq){
        ROS_DEBUG_ONCE("RealTimeInterface::waitForNextObservation(): wait for next observations");
    }
    return;
}



// -------------------------------------------------------
void RealTimeInterface::robotObservationCb(
    const sensor_msgs::JointState &msg
){
    b_received_robot_rt_obs_ = true;
    robot_rt_obs_ = msg;
}


// -------------------------------------------------------
void RealTimeInterface::dloObservationCb(
    const my_msgs::DLOStateStamped &msg
){
    if(b_received_dlo_rt_obs_ == false){ // 第一次收到msg时，设置DLO的长度
        dlo_->setLength(msg.length);
        ROS_INFO_STREAM("RealTimeInterface::dloObservationCb(): received DLO length: " << msg.length);
    }
    b_received_dlo_rt_obs_ = true;
    dlo_rt_obs_ = msg;
    get_latest_observation_time_ = ros::Time::now();

    ROS_DEBUG_STREAM("RealTimeInterface::dloObservationCb(): delay: " << (ros::Time::now() - dlo_rt_obs_.header.stamp).toSec());

    if(rt_visualizer_){
        rt_visualizer_->publishDLOState(DLOMsgToDLOState(dlo_rt_obs_), /*color*/Eigen::Vector3d(1,0,0));
    }
}


// -------------------------------------------------------
my_msgs::DLOStateStamped RealTimeInterface::getDLOobservation()
{
    if(!b_received_dlo_rt_obs_){
        ROS_ERROR("RealTimeInterface::getDLOobservation(): didn't receive dlo observation!");
    }
    return dlo_rt_obs_;
}


// -------------------------------------------------------
DLOState RealTimeInterface::getDLOState()
{
    return DLOMsgToDLOState(getDLOobservation());
}


// -------------------------------------------------------
sensor_msgs::JointState RealTimeInterface::getRobotObservation()
{
    if(!b_received_robot_rt_obs_){
        ROS_ERROR("RealTimeInterface::getDLOobservation(): didn't receive robot observation!");
    }
    return robot_rt_obs_;
}

// -------------------------------------------------------
Eigen::VectorXd RealTimeInterface::getRobotJointPos(
    const std::string &group_name
){
    sensor_msgs::JointState robot_obs = getRobotObservation();

    Eigen::VectorXd dual_arm_joint_pos, arm_0_joint_pos, arm_1_joint_pos;
    dual_arm_joint_pos = Utils::stdVector2EigenVectorXd(robot_obs.position);
    dual_arm_->splitTwoArmJointPos(dual_arm_joint_pos, arm_0_joint_pos, arm_1_joint_pos);

    if(group_name == dual_arm_->arm_group_name_){
        return dual_arm_joint_pos;
    }
    else if(group_name == dual_arm_->arm_0_->arm_group_name_){
        return arm_0_joint_pos;
    }
    else if(group_name == dual_arm_->arm_1_->arm_group_name_){
        return arm_1_joint_pos;
    }
    else{
        ROS_ERROR_STREAM("RealTimeInterface::getRobotJointPos(): do not support group_name: " << group_name);
    }

    return Eigen::VectorXd::Zero(0);
}



// -------------------------------------------------------
planning_scene::PlanningSceneConstPtr RealTimeInterface::getLockedPlanningScene()
{
    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm_); // read only

    return lscene;
}


// -------------------------------------------------------
void RealTimeInterface::clearMoveGroupPlanningScene()
{
    std_srvs::Empty empty;
    clear_octomap_client_.call(empty);
}


// -------------------------------------------------------
void RealTimeInterface::sendVideoRecordCommand(
    const std::string &command
){
    std_msgs::String string;
    string.data = command;
    video_record_command_pub_.publish(string);
}


// -------------------------------------------------------
void RealTimeInterface::publishGoalDLOState(
    const DLOState &dlo_state
){
    my_msgs::VectorStamped msg;
    msg.header.frame_id = "world";
    msg.data = Utils::stdVectorDouble2Float(Utils::eigenVectorXd2StdVector(dlo_state.fps_pos_));

    dlo_goal_pub_.publish(msg);
}


// -------------------------------------------------------
bool RealTimeInterface::waitUntilReceiveAllMessages()
{
    ros::Rate rate(30);
    while(ros::ok() && b_received_dlo_rt_obs_ == false){
        ROS_INFO_ONCE("RealTimeInterface: waiting for DLO observations ...");
        rate.sleep();
    }
    while(ros::ok() && b_received_robot_rt_obs_ == false){
        ROS_INFO_ONCE("RealTimeInterface: waiting for Robot observations ...");
        rate.sleep();
    }
    return true;
}


// -------------------------------------------------------
bool RealTimeInterface::planArmTrajectoryToTargetJointPosByMoveGroup(
    const std::string &group_name,
    const Eigen::VectorXd &start_joint_pos,
    const Eigen::VectorXd &target_joint_pos,
    moveit_msgs::RobotTrajectory &res_trajectory
){
    moveit::planning_interface::MoveGroupInterface move_group_interface(group_name);

    move_group_interface.setJointValueTarget(Utils::eigenVectorXd2StdVector(target_joint_pos));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    res_trajectory = my_plan.trajectory_;
    return success;
}


// -------------------------------------------------------
moveit_msgs::RobotTrajectory RealTimeInterface::planArmTrajectoryToTargetPoseByMoveGroup(
    const std::string &group_name,
    const Eigen::Isometry3d &target_pose,
    const Eigen::VectorXd &start_joint_pos
){
    moveit::planning_interface::MoveGroupInterface move_group_interface(group_name);

    move_group_interface.setPoseTarget(RosUtils::eigenPose2RosPose(target_pose));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return my_plan.trajectory_;
}


// -------------------------------------------------------
bool RealTimeInterface::waitUntilArmReachTargetJointPos(
    const std::string &group_name,
    const Eigen::VectorXd &target_joint_pos,
    const double error_thres
){
    ros::Rate rate(30);
    while(ros::ok() && b_received_robot_rt_obs_ == false){
        ROS_INFO_ONCE("RealTimeInterface: waiting for Robot observations ...");
        rate.sleep();
    }

    // blocking
    while(ros::ok()){
        ROS_INFO_STREAM_ONCE(group_name + " waiting for reaching the target joint pos ...");

        // 获取当前机械臂关节角
        Eigen::VectorXd arm_0_current_joint_pos, arm_1_current_joint_pos;
        dual_arm_->splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(getRobotObservation().position), arm_0_current_joint_pos, arm_1_current_joint_pos);

        Eigen::VectorXd current_joint_pos;
        if(group_name == dual_arm_->arm_0_->arm_group_name_){
            current_joint_pos = arm_0_current_joint_pos;
        }else if(group_name == dual_arm_->arm_1_->arm_group_name_){
            current_joint_pos = arm_1_current_joint_pos;
        }else if(group_name == dual_arm_->dual_arm_group_name_){
            current_joint_pos = Utils::stdVector2EigenVectorXd(getRobotObservation().position);
        }else{
            ROS_ERROR_STREAM("RealTimeInterface::waitUntilArmReachTargetJointPos(): do not support group_name: " << group_name);
            return false;
        }
        if(target_joint_pos.size() != current_joint_pos.size()){
            ROS_ERROR_STREAM("RealTimeInterface::waitUntilArmReachTargetJointPos(): the joint_pos dimensions are mismatched.");
            return false;
        }

        ROS_DEBUG_STREAM("RealTimeInterface::waitUntilArmReachTargetJointPos(): error between target joint pos and current joint pos: " 
            << (target_joint_pos - current_joint_pos).norm());
            
        // 判断是否到达
        if((target_joint_pos - current_joint_pos).norm() < error_thres){
            ROS_INFO_STREAM(group_name + " reached the target joint pos.");
            break;
        }

        rate.sleep();
    }

    return true;
}


// -------------------------------------------------------
bool RealTimeInterface::graspDLOEnds(){
    std_srvs::Trigger srv;
    if(grasp_dlo_ends_client_.call(srv)){
        return true;
    }else{
        ROS_ERROR("RealTimeInterface::graspDLOEnds(): failed.");
        return false;
    }
    return false;
}


// -------------------------------------------------------
bool RealTimeInterface::resetUnityScene(){
    std_srvs::Trigger srv;
    if(unity_reset_scene_client_.call(srv)){
        ROS_INFO("RealTimeInterface::resetUnityScene(): done.");
        ros::Duration(1.0).sleep(); // 等待接收到reset后的新的state
        return true;
    }else{
        ROS_ERROR("RealTimeInterface::resetUnityScene(): failed.");
        return false;
    }
    return false;
}


// -------------------------------------------------------
bool RealTimeInterface::moveAndGraspDLO()
{
    waitUntilReceiveAllMessages();

    // 获取当前机械臂关节角
    Eigen::VectorXd arm_0_current_joint_pos, arm_1_current_joint_pos;
    dual_arm_->splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(getRobotObservation().position), arm_0_current_joint_pos, arm_1_current_joint_pos);

    // 获取当前 DLO 状态
    Eigen::VectorXd fps_pos = Utils::stdVector2EigenVectorXd(getDLOobservation().fps_pos);
    Eigen::Vector4d end_quat_0 = Utils::stdVector2EigenVectorXd(getDLOobservation().end_quat_0);
    Eigen::Vector4d end_quat_1 = Utils::stdVector2EigenVectorXd(getDLOobservation().end_quat_1);
    int num_fps = fps_pos.size() / 3;
    Eigen::Vector3d end_pos_0 = fps_pos.block<3, 1>(0, 0);
    Eigen::Vector3d end_pos_1 = fps_pos.block<3, 1>(3*(num_fps-1), 0);

    // 得到 DLO 末端位姿 
    Eigen::Isometry3d end_0_pose = Utils::EigenPosQuatVec2Isometry3d(end_pos_0, end_quat_0);
    Eigen::Isometry3d end_1_pose = Utils::EigenPosQuatVec2Isometry3d(end_pos_1, end_quat_1);

    // 使用 IK 计算抓取到DLO时，机械臂的关节角
    Eigen::VectorXd arm_0_target_joint_pos, arm_1_target_joint_pos;
    bool arm_0_ik_success = dual_arm_->arm_0_->armTcpClosestIK(arm_0_current_joint_pos, end_0_pose, arm_0_target_joint_pos);
    bool arm_1_ik_success = dual_arm_->arm_1_->armTcpClosestIK(arm_1_current_joint_pos, end_1_pose, arm_1_target_joint_pos);
    if(!arm_0_ik_success || !arm_1_ik_success){
        ROS_WARN_COND(!arm_0_ik_success, "RealTimeInterface::moveAndGraspDLO(): arm_0 IK failed.");
        ROS_WARN_COND(!arm_1_ik_success, "RealTimeInterface::moveAndGraspDLO(): arm_1 IK failed.");
        return false;
    }
    
    // 使用 MoveGroup 进行规划轨迹
    moveit_msgs::RobotTrajectory arm_0_traj, arm_1_traj;
    bool arm_0_plan_success = planArmTrajectoryToTargetJointPosByMoveGroup(
        dual_arm_->arm_0_->arm_group_name_, arm_0_current_joint_pos, arm_0_target_joint_pos, arm_0_traj);
    bool arm_1_plan_success = planArmTrajectoryToTargetJointPosByMoveGroup(
        dual_arm_->arm_1_->arm_group_name_, arm_1_current_joint_pos, arm_1_target_joint_pos, arm_1_traj);
    if(!arm_0_plan_success || !arm_1_plan_success){
        ROS_WARN_COND(!arm_0_plan_success, "RealTimeInterface::moveAndGraspDLO(): arm_0 planning failed.");
        ROS_WARN_COND(!arm_1_plan_success, "RealTimeInterface::moveAndGraspDLO(): arm_1 planning failed.");
        return false;
    }

    // 发布 control command
    arm_0_joint_traj_pub_.publish(arm_0_traj);
    arm_1_joint_traj_pub_.publish(arm_1_traj);
    
    // blocking，等待机械臂到达期望位置
    waitUntilArmReachTargetJointPos(dual_arm_->arm_0_->arm_group_name_, arm_0_target_joint_pos, 1e-3);
    waitUntilArmReachTargetJointPos(dual_arm_->arm_1_->arm_group_name_, arm_1_target_joint_pos, 1e-3);

    ros::Duration(1.0).sleep();

    // 在 unity 中建立 DLO 末端与机械臂末端的 attachment
    if(graspDLOEnds() == false){
        return false;
    }

    return true;
}


// -------------------------------------------------------
void RealTimeInterface::dualArmJointVelControl(
    const Eigen::VectorXd &dual_arm_joint_vel
){
    my_msgs::VectorStamped msg;
    msg.header.frame_id = dual_arm_->base_link_name_;
    msg.data = Utils::stdVectorDouble2Float(Utils::eigenVectorXd2StdVector(dual_arm_joint_vel));

    dual_arm_joint_vel_pub_.publish(msg);
}


// -------------------------------------------------------
DLOState RealTimeInterface::DLOMsgToDLOState(
    const my_msgs::DLOStateStamped &msg
){
    DLOState dlo_state = DLOState(Utils::stdVector2EigenVectorXd(msg.fps_pos),
                                  Utils::stdVector2EigenVectorXd(msg.end_quat_0),
                                  Utils::stdVector2EigenVectorXd(msg.end_quat_1));
    dlo_state.updateDependentInfo();
    return dlo_state;
}


// // -------------------------------------------------------
// moveit_msgs::RobotTrajectory RealTimeInterface::dualArmJointPathToTrajectory(
//     const std::vector<std::vector<double> > &arm_0_path,
//     const std::vector<std::vector<double> > &arm_1_path,
//     const double time_between_points
// ){
//     ROS_ERROR_COND(arm_0_path.size() != arm_1_path.size(), "RealTimeInterface::dualArmJointPathToTrajectory(): lengths of arm_0_path and arm_1_path are mismatched");
    
//     moveit_msgs::RobotTrajectory traj;
//     traj.joint_trajectory.header.frame_id = dual_arm_->base_link_name_;
//     traj.joint_trajectory.joint_names = Utils::concatenateTwoVector(
//         dual_arm_->arm_0_->active_joint_names_, dual_arm_->arm_1_->active_joint_names_);

//     double time_from_start = 0.0;
//     for(size_t i = 0; i < arm_0_path.size(); i++){
//         ROS_ERROR_COND(arm_0_path[i].size() != dual_arm_->arm_0_->joint_num_, "RealTimeInterface::dualArmJointPathToTrajectory(): size of arm_0_joint_pos is wrong");
//         ROS_ERROR_COND(arm_1_path[i].size() != dual_arm_->arm_1_->joint_num_, "RealTimeInterface::dualArmJointPathToTrajectory(): size of arm_1_joint_pos is wrong");

//         trajectory_msgs::JointTrajectoryPoint point;
//         point.positions = Utils::concatenateTwoVector(arm_0_path[i], arm_1_path[i]);
//         point.velocities = std::vector<double>(dual_arm_->arm_0_->joint_num_ + dual_arm_->arm_1_->joint_num_, 0.0);
//         point.time_from_start = ros::Duration(time_from_start);

//         traj.joint_trajectory.points.push_back(point);
//         time_from_start += time_between_points;
//     }

//     return traj;
// }


// -------------------------------------------------------
moveit_msgs::RobotTrajectory RealTimeInterface::armJointPathToTrajectory(
    const std::string &group_name,
    const std::vector<std::vector<double> > &path,
    const double time_between_points
){  
    moveit_msgs::RobotTrajectory traj;

    if(group_name == dual_arm_->arm_0_->arm_group_name_){
        traj.joint_trajectory.header.frame_id = dual_arm_->arm_0_->base_link_name_;
        traj.joint_trajectory.joint_names = dual_arm_->arm_0_->active_joint_names_;
    }else if(group_name == dual_arm_->arm_1_->arm_group_name_){
        traj.joint_trajectory.header.frame_id = dual_arm_->arm_1_->base_link_name_;
        traj.joint_trajectory.joint_names = dual_arm_->arm_1_->active_joint_names_;
    }else if(group_name == dual_arm_->arm_group_name_){
        traj.joint_trajectory.header.frame_id = dual_arm_->base_link_name_;
        traj.joint_trajectory.joint_names = dual_arm_->active_joint_names_;
    }else{
        ROS_ERROR_STREAM("RealTimeInterface::armJointPathToTrajectory(): do not support group_name: " << group_name);
    }
    
    double time_from_start = 0.0;
    for(size_t i = 0; i < path.size(); i++){
        ROS_ERROR_COND(path[i].size() != traj.joint_trajectory.joint_names.size(), "RealTimeInterface::armJointPathToTrajectory(): size of a path point is wrong");
    
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = path[i];
        point.velocities = std::vector<double>(path[i].size(), 0.0);
        point.time_from_start = ros::Duration(time_from_start);

        traj.joint_trajectory.points.push_back(point);
        time_from_start += time_between_points;
    }

    return traj;
}



// -------------------------------------------------------
void RealTimeInterface::armTrajectoryControl(
    const std::string &group_name,
    const moveit_msgs::RobotTrajectory &trajectory,
    const bool b_block
){
    if(group_name == dual_arm_->arm_0_->arm_group_name_){
        if(trajectory.joint_trajectory.points[0].positions.size() != dual_arm_->arm_0_->joint_num_){
            ROS_ERROR("RealTimeInterface::armTrajectoryControl(): the size of a trajectory point does not match the group_name.");
            return;
        }
        arm_0_joint_traj_pub_.publish(trajectory);
    }
    else if(group_name == dual_arm_->arm_1_->arm_group_name_){
        if(trajectory.joint_trajectory.points[0].positions.size() != dual_arm_->arm_1_->joint_num_){
            ROS_ERROR("RealTimeInterface::armTrajectoryControl(): the size of a trajectory point does not match the group_name.");
            return;
        }
        arm_1_joint_traj_pub_.publish(trajectory);
    }
    else if(group_name == dual_arm_->dual_arm_group_name_){
        if(trajectory.joint_trajectory.points[0].positions.size() != dual_arm_->arm_0_->joint_num_ + dual_arm_->arm_1_->joint_num_){
            ROS_ERROR("RealTimeInterface::armTrajectoryControl(): the size of a trajectory point does not match the group_name.");
            return;
        }
        dual_arm_joint_traj_pub_.publish(trajectory);
    }
    else{
        ROS_ERROR_STREAM("RealTimeInterface::armTrajectoryControl(): do not support group_name: " << group_name);
        return;
    }

    // 如果需要等到机械臂完成轨迹再退出
    if(b_block){
        int traj_length = trajectory.joint_trajectory.points.size();
        Eigen::VectorXd target_joint_pos = Utils::stdVector2EigenVectorXd(trajectory.joint_trajectory.points[traj_length-1].positions);
        waitUntilArmReachTargetJointPos(group_name, target_joint_pos);
    }
}


// -------------------------------------------------------
void RealTimeInterface::armPathControl(
    const std::string &group_name,
    const std::vector<std::vector<double> > path,
    const double time_between_points,
    const bool b_block
){
    moveit_msgs::RobotTrajectory traj = armJointPathToTrajectory(group_name, path, time_between_points);
    armTrajectoryControl(group_name, traj, b_block);
}


// -------------------------------------------------------
void RealTimeInterface::dualArmPathControl(
    const std::vector<std::vector<double> > arm_0_path,
    const std::vector<std::vector<double> > arm_1_path,
    const double time_between_points,
    const bool b_block
){
    ROS_ERROR_COND(arm_0_path.size() != arm_1_path.size(), "RealTimeInterface::dualArmPathControl(): lengths of arm_0_path and arm_1_path are mismatched");

    std::vector<std::vector<double> > path(arm_0_path.size());
    for(size_t i = 0; i < arm_0_path.size(); i++){
       path[i] =  Utils::concatenateTwoVector(arm_0_path[i], arm_1_path[i]);
    }

    armPathControl(dual_arm_->arm_group_name_, path, time_between_points, b_block);
}


// -------------------------------------------------------
bool RealTimeInterface::dualArmMoveToTcpPoses(
    const Eigen::Isometry3d &end_0_pose,
    const Eigen::Isometry3d &end_1_pose,
    bool b_block
){
    ros::Rate rate(30);
    while(ros::ok() && b_received_robot_rt_obs_ == false){
        ROS_INFO_ONCE("RealTimeInterface: waiting for Robot observations ...");
        rate.sleep();
    }

    // 获取当前机械臂关节角
    Eigen::VectorXd arm_0_current_joint_pos, arm_1_current_joint_pos;
    dual_arm_->splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(getRobotObservation().position), arm_0_current_joint_pos, arm_1_current_joint_pos);

    // 使用 IK 计算抓取到DLO时，机械臂的关节角
    Eigen::VectorXd arm_0_target_joint_pos, arm_1_target_joint_pos;
    bool arm_0_ik_success = dual_arm_->arm_0_->armTcpClosestIK(arm_0_current_joint_pos, end_0_pose, arm_0_target_joint_pos);
    bool arm_1_ik_success = dual_arm_->arm_1_->armTcpClosestIK(arm_1_current_joint_pos, end_1_pose, arm_1_target_joint_pos);
    if(!arm_0_ik_success || !arm_1_ik_success){
        ROS_WARN_COND(!arm_0_ik_success, "RealTimeInterface::dualArmMoveToTcpPoses(): arm_0 IK failed.");
        ROS_WARN_COND(!arm_1_ik_success, "RealTimeInterface::dualArmMoveToTcpPoses(): arm_1 IK failed.");
        return false;
    }

    // std::cout << arm_0_target_joint_pos.transpose() << std::endl;
    
    // 使用 MoveGroup 进行规划轨迹
    moveit_msgs::RobotTrajectory arm_0_traj, arm_1_traj;
    bool arm_0_plan_success = planArmTrajectoryToTargetJointPosByMoveGroup(
        dual_arm_->arm_0_->arm_group_name_, arm_0_current_joint_pos, arm_0_target_joint_pos, arm_0_traj);
    bool arm_1_plan_success = planArmTrajectoryToTargetJointPosByMoveGroup(
        dual_arm_->arm_1_->arm_group_name_, arm_1_current_joint_pos, arm_1_target_joint_pos, arm_1_traj);
    if(!arm_0_plan_success || !arm_1_plan_success){
        ROS_WARN_COND(!arm_0_plan_success, "RealTimeInterface::dualArmMoveToTcpPoses(): arm_0 planning failed.");
        ROS_WARN_COND(!arm_1_plan_success, "RealTimeInterface::dualArmMoveToTcpPoses(): arm_1 planning failed.");
        return false;
    }

    // 发布 control command
    arm_0_joint_traj_pub_.publish(arm_0_traj);
    arm_1_joint_traj_pub_.publish(arm_1_traj);
    
    if(b_block){
        // blocking，等待机械臂到达期望位置
        waitUntilArmReachTargetJointPos(dual_arm_->arm_0_->arm_group_name_, arm_0_target_joint_pos);
        waitUntilArmReachTargetJointPos(dual_arm_->arm_1_->arm_group_name_, arm_1_target_joint_pos);
    }

    return true;
}








} // end namespace