#include "dlo_arm_planning_pkg/visualize.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    Visualize::Visualize(
        const ros::NodeHandle &nh,
        const Scene::Ptr &scene,
        const std::string topic_prefix) : nh_(nh)
    {
        scene_ = scene;
        dual_arm_ = scene_->dual_arm_;
        topic_prefix_ = topic_prefix;
        arm_base_link_name_ = dual_arm_->base_link_name_;
        loadParams();
        initiate();
    }

    // -------------------------------------------------------
    Visualize::Visualize(
        const ros::NodeHandle &nh,
        const DualArm::Ptr &dual_arm,
        const std::string topic_prefix) : nh_(nh)
    {
        dual_arm_ = dual_arm;
        topic_prefix_ = topic_prefix;
        arm_base_link_name_ = dual_arm_->base_link_name_;
        loadParams();
        initiate();
    }

    // -------------------------------------------------------
    void Visualize::setScene(
        const Scene::Ptr &scene)
    {
        scene_ = scene;
    }

    // -------------------------------------------------------
    void Visualize::loadParams()
    {
    }

    // -------------------------------------------------------
    void Visualize::initiate()
    {
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(arm_base_link_name_,
                                                                                 topic_prefix_ + "/moveit_visual_tools", dual_arm_->robot_model_loader_->getModel());

        planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(topic_prefix_ + "/planning_scene", 1);
        dlo_state_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_prefix_ + "/dlo_state", 10);

        dlo_collision_shape_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_prefix_ + "/dlo_collision_shape", 50);
        robot_collision_shape_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_prefix_ + "/robot_collision_shape", 50);

        ROS_DEBUG("Visualize: sleep 0.2 second to intialize the publishers.");
        ros::Duration(0.2).sleep();

        visual_tools_->setRobotStateTopic(topic_prefix_ + "/robot_state");
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();
    }

    // -------------------------------------------------------
    void Visualize::publishText(
        const std::string &text)
    {
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = -0.6;
        text_pose.translation().y() = 0.5;

        visual_tools_->publishText(text_pose, text, rviz_visual_tools::GREY, rviz_visual_tools::XLARGE);
        visual_tools_->trigger();
    }

    // -------------------------------------------------------
    void Visualize::publishRobotState(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos)
    {
        publishRobotState(Utils::eigenVectorXd2StdVector(arm_0_joint_pos), Utils::eigenVectorXd2StdVector(arm_1_joint_pos));
    }

    // -------------------------------------------------------
    void Visualize::publishRobotState(
        const std::vector<double> &arm_0_joint_pos,
        const std::vector<double> &arm_1_joint_pos)
    {
        dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos.size());
        dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos.size());

        dual_arm_->robot_state_->setJointGroupPositions(dual_arm_->arm_0_->arm_group_name_, arm_0_joint_pos);
        dual_arm_->robot_state_->setJointGroupPositions(dual_arm_->arm_1_->arm_group_name_, arm_1_joint_pos);

        visual_tools_->publishRobotState(dual_arm_->robot_state_);
        visual_tools_->trigger();
    }

    // -------------------------------------------------------
    void Visualize::publishPlanningScene(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos)
    {
        ROS_ERROR_COND(scene_ == nullptr, "Visualize doesn't have the member scene_.");

        dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos.size());
        dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos.size());

        moveit_msgs::PlanningScene planning_scene_msg;
        scene_->getPlanningSceneMsg(arm_0_joint_pos, arm_1_joint_pos, planning_scene_msg);

        planning_scene_pub_.publish(planning_scene_msg);
    }

    // -------------------------------------------------------
    void Visualize::publishDLOState(
        const DLOState &dlo_state,
        const Eigen::Vector3d color)
    {
        // 发布两端pose对应的Axis
        visual_tools_->deleteAllMarkers(); // 注意：用于清除上一次publish的Axis

        // visual_tools_->publishAxis(dlo_state.getLeftEndPose(), rviz_visual_tools::XXXSMALL); // XXXSMALL / XSMALL for visualizing DER
        // visual_tools_->publishAxis(dlo_state.getRightEndPose(), rviz_visual_tools::XXXSMALL);

        // double length = 0.03;
        // double radius = 0.003;
        // publishAxis(dlo_state.getLeftEndPose(), length, radius, color);
        // publishAxis(dlo_state.getRightEndPose(), length, radius, color);

        publishAxis2(dlo_state.getLeftEndPose(), color);
        publishAxis2(dlo_state.getRightEndPose(), color);

        visual_tools_->trigger();

        // 发布 points
        auto shape_vis = getMarkerForShapeVis(dlo_state.fps_pos_, color);
        dlo_state_pub_.publish(shape_vis[0]);
        dlo_state_pub_.publish(shape_vis[1]);
    }

    // -------------------------------------------------------
    bool Visualize::publishAxis(const Eigen::Isometry3d &pose, double length, double radius,
                                const Eigen::Vector3d color)
    {
        std_msgs::ColorRGBA color_msg;
        color_msg.r = color[0];
        color_msg.g = color[1];
        color_msg.b = color[2];
        color_msg.a = 1.0;

        // Publish x axis
        Eigen::Isometry3d x_pose =
            Eigen::Translation3d(length / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
        x_pose = pose * x_pose;
        visual_tools_->publishCylinder(visual_tools_->convertPose(x_pose), color_msg, length, radius, "Axis");

        // Publish y axis
        Eigen::Isometry3d y_pose =
            Eigen::Translation3d(0, length / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
        y_pose = pose * y_pose;
        visual_tools_->publishCylinder(visual_tools_->convertPose(y_pose), color_msg, length, radius, "Axis");

        // Publish z axis
        Eigen::Isometry3d z_pose = Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        z_pose = pose * z_pose;
        visual_tools_->publishCylinder(visual_tools_->convertPose(z_pose), color_msg, length, radius, "Axis");

        return true;
    }

    // -------------------------------------------------------
    bool Visualize::publishAxis2(const Eigen::Isometry3d &pose,
                                 const Eigen::Vector3d color)
    {
        rviz_visual_tools::colors color_t;
        if (color[0] == 1.0)
        {
            color_t = rviz_visual_tools::BRIGHT_RED;
        }
        else if (color[1] == 1.0)
        {
            color_t = rviz_visual_tools::GREEN;
        }
        else if (color[2] == 1.0)
        {
            color_t = rviz_visual_tools::BLUE;
        }
        else
        {
            color_t = rviz_visual_tools::BLACK;
        }

        // visual_tools_->publishXArrow(pose, color_t, rviz_visual_tools::XXXSMALL);
        // visual_tools_->publishYArrow(pose, color_t, rviz_visual_tools::XXXSMALL);
        // visual_tools_->publishZArrow(pose, color_t, rviz_visual_tools::XXXSMALL);

        visual_tools_->publishXArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::XXXSMALL);
        visual_tools_->publishYArrow(pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXSMALL);
        visual_tools_->publishZArrow(pose, rviz_visual_tools::BLUE, rviz_visual_tools::XXXSMALL);

        return true;
    }

    // -------------------------------------------------------
    void Visualize::publishPoints(
        const Eigen::VectorXd &points,
        const Eigen::Vector3d color)
    {
        // 发布 points
        auto shape_vis = getMarkerForShapeVis(points, color);
        dlo_state_pub_.publish(shape_vis[0]);
        dlo_state_pub_.publish(shape_vis[1]);
    }

    // -------------------------------------------------------
    void Visualize::publishDLOCollisionShape(
        const VecEigenIsometry3d &poses,
        const double &capsule_length,
        const double &capsule_radius)
    {
        std::vector<double> capsule_lengths(poses.size());
        std::vector<double> capsule_radiuses(poses.size());
        for (size_t k = 0; k < poses.size(); k++)
        {
            capsule_lengths[k] = capsule_length;
            capsule_radiuses[k] = capsule_radius;
        }

        auto shape_vis = getCapsuleMarkers(poses, capsule_lengths, capsule_radiuses);

        for (size_t k = 0; k < shape_vis.size(); k++)
        {
            dlo_collision_shape_pub_.publish(shape_vis[k]);
        }
    }

    // -------------------------------------------------------
    void Visualize::publishRobotCollisionShape(
        const VecEigenIsometry3d &poses,
        const std::vector<double> &capsule_lengths,
        const std::vector<double> &capsule_radiuses)
    {
        auto shape_vis = getCapsuleMarkers(poses, capsule_lengths, capsule_radiuses, Eigen::Vector3d(1.0, 0.0, 0.0));

        for (size_t k = 0; k < shape_vis.size(); k++)
        {
            robot_collision_shape_pub_.publish(shape_vis[k]);
        }
    }

    // -------------------------------------------------------
    void Visualize::publishRobotCollisionShape(
        const VecEigenIsometry3d &poses,
        const std::vector<std::string> &geometry_types,
        const std::vector<std::vector<double>> &geometry_params)
    {
        int marker_id = 0;
        for (size_t k = 0; k < poses.size(); k++)
        {
            if (geometry_types[k] == "sphere")
            {
                visualization_msgs::Marker marker = getSphereMarker(poses[k], /*radius*/ geometry_params[k][0], /*id*/ marker_id, /*color*/ Eigen::Vector3d(1.0, 0.0, 0.0));
                robot_collision_shape_pub_.publish(marker);
                marker_id++;
            }
            else if (geometry_types[k] == "capsule")
            {
                std::vector<visualization_msgs::Marker> markers = getCapsuleMarker(poses[k],
                                                                                   /*radius*/ geometry_params[k][0], /*length*/ geometry_params[k][1],
                                                                                   /*id*/ marker_id, /*color*/ Eigen::Vector3d(1.0, 0.0, 0.0));
                for (auto &marker : markers)
                    robot_collision_shape_pub_.publish(marker);
                marker_id += 3;
            }
            else
            {
                ROS_ERROR_STREAM("Visualize::publishRobotCollisionShape() doesn't support geometry type: " << geometry_types[k]);
            }
        }
    }

    // -------------------------------------------------------
    void Visualize::publishNode(
        const Node::Ptr &node,
        const std::string text)
    {
        ROS_ERROR_COND(node == nullptr, "publishNode(): the input node is nullptr.");
        // publishRobotState(Utils::eigenVectorXd2StdVector(node->arm_0_joint_pos_),
        //                   Utils::eigenVectorXd2StdVector(node->arm_1_joint_pos_));

        publishDLOState(node->dlo_state_, Eigen::Vector3d(0, 0, 1));

        if (node->arm_0_joint_pos_.size() != 0)
        {
            publishPlanningScene(node->arm_0_joint_pos_, node->arm_1_joint_pos_);
        }

        publishText(text);
    }

    // ------------------------------------------------------------
    void Visualize::publishNodePath(
        const std::vector<Node::Ptr> &path_list,
        double ros_rate)
    {
        ros::Rate rate(ros_rate);

        for (size_t i = 0; ros::ok() && i < path_list.size(); i++)
        {
            publishNode(path_list[i]);
            rate.sleep();
        }
    }

    // --------------------------------------------------------
    std::vector<visualization_msgs::Marker> Visualize::getMarkerForShapeVis(
        const Eigen::VectorXd &fps_pos,
        const Eigen::Vector3d &color)
    {
        static int id = 0;

        visualization_msgs::Marker Points, Line, point_left, point_right;

        Points.header.frame_id = Line.header.frame_id = point_left.header.frame_id = point_right.header.frame_id = arm_base_link_name_;
        Points.header.stamp = Line.header.stamp = point_left.header.stamp = point_right.header.stamp = ros::Time::now();
        Points.action = Line.action = point_left.action = point_right.action = visualization_msgs::Marker::ADD;
        Points.pose.orientation.w = Line.pose.orientation.w = point_left.pose.orientation.w = point_right.pose.orientation.w = 1.0;

        Points.id = 0;
        Line.id = 1;
        point_left.id = 2;
        point_right.id = 3;

        Points.type = visualization_msgs::Marker::POINTS;
        point_left.type = visualization_msgs::Marker::POINTS;
        point_right.type = visualization_msgs::Marker::POINTS;
        Line.type = visualization_msgs::Marker::LINE_STRIP;

        Points.scale.x = 0.01 / 2;
        Points.scale.y = 0.01 / 2;
        point_left.scale.x = point_left.scale.y = 0.02;
        point_right.scale.x = point_right.scale.y = 0.02;

        Line.scale.x = 0.01 / 2;

        Points.color.r = 1.0;
        Points.color.g = 1.0;
        Points.color.b = 1.0;
        Points.color.a = 1.0;

        Line.color.r = color[0];
        Line.color.g = color[1];
        Line.color.b = color[2];
        Line.color.a = 1.0;

        point_left.color.r = 1.0;
        point_left.color.b = 1.0;
        point_left.color.a = 1.0;

        point_right.color.g = 1.0;
        point_right.color.b = 1.0;
        point_right.color.a = 1.0;

        geometry_msgs::Point pt;
        int num_fps = fps_pos.size() / 3;

        for (int i = 0; i < num_fps; i++)
        {
            Eigen::Vector3d coord = fps_pos.block<3, 1>(3 * i, 0);
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);

            Points.points.push_back(pt);
            Line.points.push_back(pt);

            if (i == 0)
                point_left.points.push_back(pt);
            if (i == num_fps - 1)
                point_right.points.push_back(pt);
        }

        std::vector<visualization_msgs::Marker> shape_vis;
        shape_vis.push_back(Points);
        shape_vis.push_back(Line);
        shape_vis.push_back(point_left);
        shape_vis.push_back(point_right);

        return shape_vis;
    }

    // --------------------------------------------------------
    std::vector<visualization_msgs::Marker> Visualize::getCapsuleMarkersForDLO(
        const VecEigenIsometry3d &poses,
        const double &capsule_length,
        const double &capsule_radius)
    {
        std::vector<visualization_msgs::Marker> capsule_markers;

        for (size_t k = 0; k < poses.size(); k++)
        {
            visualization_msgs::Marker capsule;

            capsule.header.frame_id = arm_base_link_name_;
            capsule.header.stamp = ros::Time::now();
            capsule.action = visualization_msgs::Marker::ADD;
            capsule.type = visualization_msgs::Marker::CYLINDER;
            capsule.id = k;

            capsule.color.r = 1.0;
            capsule.color.g = 1.0;
            capsule.color.b = 1.0;
            capsule.color.a = 0.5;

            capsule.scale.x = capsule.scale.y = 2 * capsule_radius;
            capsule.scale.z = capsule_length;
            capsule.pose = RosUtils::eigenPose2RosPose(poses[k]);

            capsule_markers.push_back(capsule);
        }

        return capsule_markers;
    }

    // --------------------------------------------------------
    std::vector<visualization_msgs::Marker> Visualize::getCapsuleMarkers(
        const VecEigenIsometry3d &poses,
        const std::vector<double> &capsule_lengths,
        const std::vector<double> &capsule_radiuses,
        const Eigen::Vector3d color)
    {
        std::vector<visualization_msgs::Marker> capsule_markers;

        int id = 0;
        for (size_t k = 0; k < poses.size(); k++)
        {
            visualization_msgs::Marker capsule;
            visualization_msgs::Marker sphere_0, sphere_1;

            capsule.header.frame_id = sphere_0.header.frame_id = sphere_1.header.frame_id = arm_base_link_name_;
            capsule.header.stamp = sphere_0.header.stamp = sphere_1.header.stamp = ros::Time::now();
            capsule.action = sphere_0.action = sphere_1.action = visualization_msgs::Marker::ADD;
            capsule.type = visualization_msgs::Marker::CYLINDER;
            sphere_0.type = sphere_1.type = visualization_msgs::Marker::SPHERE;
            capsule.id = id++;
            sphere_0.id = id++;
            sphere_1.id = id++;

            capsule.color.r = sphere_0.color.r = sphere_1.color.r = color[0];
            capsule.color.g = sphere_0.color.g = sphere_1.color.g = color[1];
            capsule.color.b = sphere_0.color.b = sphere_1.color.b = color[2];
            capsule.color.a = sphere_0.color.a = sphere_1.color.a = 0.5;

            capsule.scale.x = capsule.scale.y = 2 * capsule_radiuses[k];
            capsule.scale.z = capsule_lengths[k];
            sphere_0.scale.x = sphere_0.scale.y = sphere_0.scale.z = 2 * capsule_radiuses[k];
            sphere_1.scale.x = sphere_1.scale.y = sphere_1.scale.z = 2 * capsule_radiuses[k];

            capsule.pose = RosUtils::eigenPose2RosPose(poses[k]);
            Eigen::Vector3d sphere_0_pos = poses[k] * Eigen::Vector3d(0, 0, capsule_lengths[k] / 2);
            sphere_0.pose = RosUtils::eigenPose2RosPose(Utils::EigenPosQuatVec2Isometry3d(sphere_0_pos, Eigen::Quaterniond::Identity()));
            Eigen::Vector3d sphere_1_pos = poses[k] * Eigen::Vector3d(0, 0, -capsule_lengths[k] / 2);
            sphere_1.pose = RosUtils::eigenPose2RosPose(Utils::EigenPosQuatVec2Isometry3d(sphere_1_pos, Eigen::Quaterniond::Identity()));

            capsule_markers.push_back(capsule);
            capsule_markers.push_back(sphere_0);
            capsule_markers.push_back(sphere_1);
        }

        return capsule_markers;
    }

    // --------------------------------------------------------
    std::vector<visualization_msgs::Marker> Visualize::getSphereMarkers(
        const VecEigenIsometry3d &poses,
        const std::vector<double> &radiuses,
        const Eigen::Vector3d color)
    {
        std::vector<visualization_msgs::Marker> sphere_markers;

        int id = 0;
        for (size_t k = 0; k < poses.size(); k++)
        {
            visualization_msgs::Marker sphere;

            sphere.header.frame_id = arm_base_link_name_;
            sphere.header.stamp = ros::Time::now();
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.id = id++;

            sphere.color.r = color[0];
            sphere.color.g = color[1];
            sphere.color.b = color[2];
            sphere.color.a = 0.5;

            sphere.scale.x = sphere.scale.y = sphere.scale.z = 2 * radiuses[k];

            sphere.pose = RosUtils::eigenPose2RosPose(poses[k]);

            sphere_markers.push_back(sphere);
        }

        return sphere_markers;
    }

    // --------------------------------------------------------
    visualization_msgs::Marker Visualize::getSphereMarker(
        const Eigen::Isometry3d &pose,
        const double &radius,
        const int id,
        const Eigen::Vector3d color)
    {
        visualization_msgs::Marker sphere;

        sphere.header.frame_id = arm_base_link_name_;
        sphere.header.stamp = ros::Time::now();
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.id = id;

        sphere.color.r = color[0];
        sphere.color.g = color[1];
        sphere.color.b = color[2];
        sphere.color.a = 0.5;

        sphere.scale.x = sphere.scale.y = sphere.scale.z = 2 * radius;

        sphere.pose = RosUtils::eigenPose2RosPose(pose);

        return sphere;
    }

    // --------------------------------------------------------
    std::vector<visualization_msgs::Marker> Visualize::getCapsuleMarker(
        const Eigen::Isometry3d &pose,
        const double &radius,
        const double &length,
        const int id,
        const Eigen::Vector3d color)
    {
        std::vector<visualization_msgs::Marker> capsule_marker;

        visualization_msgs::Marker capsule;
        visualization_msgs::Marker sphere_0, sphere_1;

        capsule.header.frame_id = sphere_0.header.frame_id = sphere_1.header.frame_id = arm_base_link_name_;
        capsule.header.stamp = sphere_0.header.stamp = sphere_1.header.stamp = ros::Time::now();
        capsule.action = sphere_0.action = sphere_1.action = visualization_msgs::Marker::ADD;
        capsule.type = visualization_msgs::Marker::CYLINDER;
        sphere_0.type = sphere_1.type = visualization_msgs::Marker::SPHERE;
        capsule.id = id;
        sphere_0.id = id + 1;
        sphere_1.id = id + 2;

        capsule.color.r = sphere_0.color.r = sphere_1.color.r = color[0];
        capsule.color.g = sphere_0.color.g = sphere_1.color.g = color[1];
        capsule.color.b = sphere_0.color.b = sphere_1.color.b = color[2];
        capsule.color.a = sphere_0.color.a = sphere_1.color.a = 0.5;

        capsule.scale.x = capsule.scale.y = 2 * radius;
        capsule.scale.z = length;
        sphere_0.scale.x = sphere_0.scale.y = sphere_0.scale.z = 2 * radius;
        sphere_1.scale.x = sphere_1.scale.y = sphere_1.scale.z = 2 * radius;

        capsule.pose = RosUtils::eigenPose2RosPose(pose);
        Eigen::Vector3d sphere_0_pos = pose * Eigen::Vector3d(0, 0, length / 2);
        sphere_0.pose = RosUtils::eigenPose2RosPose(Utils::EigenPosQuatVec2Isometry3d(sphere_0_pos, Eigen::Quaterniond::Identity()));
        Eigen::Vector3d sphere_1_pos = pose * Eigen::Vector3d(0, 0, -length / 2);
        sphere_1.pose = RosUtils::eigenPose2RosPose(Utils::EigenPosQuatVec2Isometry3d(sphere_1_pos, Eigen::Quaterniond::Identity()));

        capsule_marker.push_back(capsule);
        capsule_marker.push_back(sphere_0);
        capsule_marker.push_back(sphere_1);

        return capsule_marker;
    }

} // end namespace