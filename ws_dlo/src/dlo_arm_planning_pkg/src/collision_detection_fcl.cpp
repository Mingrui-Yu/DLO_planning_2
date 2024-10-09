#include "dlo_arm_planning_pkg/collision_detection_fcl.h"

namespace dlo_arm_planning_pkg
{

    // -------------------------------------------------------
    // allowed collision matrix 现在手动定义在这里
    template <typename S>
    bool allowedCollision(fcl::CollisionObject<S> *o1, fcl::CollisionObject<S> *o2)
    {
        const CollisionObjectUserData *cd1 = static_cast<const CollisionObjectUserData *>(o1->collisionGeometry()->getUserData());
        const CollisionObjectUserData *cd2 = static_cast<const CollisionObjectUserData *>(o2->collisionGeometry()->getUserData());

        // DLO self collision: no collision check for the adjacent edges
        if (cd1->object_type == "dlo" and cd2->object_type == "dlo")
        {
            if (std::abs(cd1->dlo_data.edge_index - cd2->dlo_data.edge_index) <= 1)
                return true;
        }
        // robot self collision: no collision check for the adjacent edges of one arm
        else if (cd1->object_type == "arm" and cd2->object_type == "arm")
        {
            if (cd1->robot_data.group_name == cd2->robot_data.group_name)
            {
                if (std::abs(cd1->robot_data.link_index - cd2->robot_data.link_index) <= 2)
                    return true;
            }
        }
        // collision check between the robot and the DLO
        else if (cd1->object_type == "dlo" and cd2->object_type == "arm")
        {
            if (cd1->dlo_data.b_left_end == true && cd2->robot_data.link_name == "arm_0_wrist_3_link")
                return true;
            if (cd1->dlo_data.b_right_end == true && cd2->robot_data.link_name == "arm_1_wrist_3_link")
                return true;
        }
        else if (cd2->object_type == "dlo" and cd1->object_type == "arm")
        {
            if (cd2->dlo_data.b_left_end == true && cd1->robot_data.link_name == "arm_0_wrist_3_link")
                return true;
            if (cd2->dlo_data.b_right_end == true && cd1->robot_data.link_name == "arm_1_wrist_3_link")
                return true;
        }

        return false;
        // true: allow collision; false: do not allow collision.
    }

    // -------------------------------------------------------
    template <typename S>
    bool collisionCallback(fcl::CollisionObject<S> *o1, fcl::CollisionObject<S> *o2,
                           void *data)
    {
        assert(data != nullptr);
        auto *collision_data = static_cast<fcl::DefaultCollisionData<S> *>(data);
        const fcl::CollisionRequest<S> &request = collision_data->request;
        fcl::CollisionResult<S> &result = collision_data->result;

        // if allowing collision, then do not check collision
        if (allowedCollision(o1, o2))
            return false;

        if (collision_data->done)
            return true;

        fcl::collide(o1, o2, request, result);

        if (!request.enable_cost && result.isCollision() &&
            result.numContacts() >= request.num_max_contacts)
        {
            collision_data->done = true;
        }

        return collision_data->done;
    }

    // -------------------------------------------------------
    template <typename S>
    bool distanceCallback(fcl::CollisionObject<S> *o1, fcl::CollisionObject<S> *o2,
                          void *data, S &dist)
    {
        assert(data != nullptr);
        auto *cdata = static_cast<fcl::DefaultDistanceData<S> *>(data);
        const fcl::DistanceRequest<S> &request = cdata->request;
        fcl::DistanceResult<S> &result = cdata->result;

        // if allowing collision, then do not check collision
        if (allowedCollision(o1, o2))
            return false;

        if (cdata->done)
        {
            dist = result.min_distance;
            return true;
        }

        fcl::distance(o1, o2, request, result);
        dist = result.min_distance;
        if (dist <= 0)
            return true; // in collision or in touch
        return cdata->done;
    }

    // -------------------------------------------------------
    CollisionDetectionFCL::CollisionDetectionFCL(
        const ros::NodeHandle &nh,
        const DLO::Ptr &dlo,
        const DualArm::Ptr &dual_arm) : nh_(nh)
    {
        dlo_ = dlo;
        dual_arm_ = dual_arm;

        initializeDLOManager();
        initializeRobotManager();
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::initializeDLOManager()
    {
        ROS_DEBUG_STREAM("CollisionDetectionFCL::initializeDLOManager(): start.");

        int n_edges = dlo_->num_fps_ - 1;
        double edge_length = dlo_->dlo_length_ / n_edges;
        double edge_radius = dlo_capsule_radius_;

        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

        objects_dlo_userdata_ = std::vector<CollisionObjectUserData>(n_edges);
        // create a capsule for each edge
        for (int k = 0; k < n_edges; k++)
        {
            auto &user_data = objects_dlo_userdata_[k];
            user_data.object_type = "dlo";
            user_data.dlo_data.edge_index = k;
            user_data.geometry_type = "capsule";
            user_data.geometry_params = std::vector<double>{edge_radius, edge_length};
            // the edges corresponding to the DLO ends
            if (k <= 1)
                user_data.dlo_data.b_left_end = true;
            else if (k >= n_edges - 2)
                user_data.dlo_data.b_right_end = true;

            CollisionGeometryPtr_t geometry;
            if (user_data.geometry_type == "capsule")
            {
                geometry = CollisionGeometryPtr_t(new fcl::Capsule<double>(
                    user_data.geometry_params[0], user_data.geometry_params[1]));
            }
            else
            {
                ROS_ERROR_STREAM("CollisionDetectionFCL: doesn't support the geometry type " << user_data.geometry_type);
            }
            geometry->setUserData(&user_data);

            fcl::CollisionObjectd object(geometry, Eigen::Isometry3d::Identity());
            objects_dlo_.push_back(object);
        }

        manager_dlo_ = new fcl::DynamicAABBTreeCollisionManagerd();
        for (int k = 0; k < objects_dlo_.size(); k++)
        {
            manager_dlo_->registerObject(&objects_dlo_[k]);
        }

        manager_dlo_->setup();

        ROS_DEBUG_STREAM("CollisionDetectionFCL::initializeDLOManager(): finish.");
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::loadRobotCoarseCollisionShape()
    {
        std::string param_name;
        int object_idx = 0;

        while (ros::ok)
        {
            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/attached_link/link_name";
            if (!nh_.hasParam(param_name))
            {
                if (object_idx == 0)
                    ROS_WARN_STREAM("No robot coarse collision shape parameters.");
                else
                    ROS_INFO_STREAM("Load robot coarse collision shape: end at object_" << std::to_string(object_idx - 1));
                break;
            }

            CollisionObjectUserData user_data;
            user_data.object_type = "arm";

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/attached_link/group_name";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, user_data.robot_data.group_name);

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/attached_link/link_index";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, user_data.robot_data.link_index);

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/attached_link/link_name";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, user_data.robot_data.link_name);

            std::vector<double> pos, euler_xyz;
            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/relative_pose/pos";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, pos);

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/relative_pose/euler_xyz";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, euler_xyz);

            user_data.robot_data.relative_pose = Utils::EigenPosQuatVec2Isometry3d(Utils::stdVector2EigenVectorXd(pos),
                                                                                   Utils::eulerAngleToQuat(Utils::stdVector2EigenVectorXd(euler_xyz), Eigen::Vector3d(0, 1, 2)));

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/geometry/type";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, user_data.geometry_type);

            param_name = "robot_coarse_collision_shape/object_" + std::to_string(object_idx) + "/geometry/params";
            ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
            nh_.getParam(param_name, user_data.geometry_params);

            objects_dual_arm_userdata_.push_back(user_data);
            object_idx++;
        }
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::initializeRobotManager()
    {
        ROS_DEBUG_STREAM("CollisionDetectionFCL::initializeRobotManager(): start.");

        loadRobotCoarseCollisionShape();

        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

        for (size_t k = 0; k < objects_dual_arm_userdata_.size(); k++)
        {
            auto &user_data = objects_dual_arm_userdata_[k];

            CollisionGeometryPtr_t geometry;
            if (user_data.geometry_type == "capsule")
            {
                geometry = CollisionGeometryPtr_t(new fcl::Capsule<double>(
                    user_data.geometry_params[0], user_data.geometry_params[1]));
            }
            else if (user_data.geometry_type == "sphere")
            {
                geometry = CollisionGeometryPtr_t(new fcl::Sphere<double>(
                    user_data.geometry_params[0]));
            }
            else
            {
                ROS_ERROR_STREAM("CollisionDetectionFCL: doesn't support the geometry type " << user_data.geometry_type);
            }
            geometry->setUserData(&user_data);

            fcl::CollisionObjectd object(geometry, Eigen::Isometry3d::Identity());
            objects_dual_arm_.push_back(object);
        }

        manager_dual_arm_ = new fcl::DynamicAABBTreeCollisionManagerd();
        for (int k = 0; k < objects_dual_arm_.size(); k++)
        {
            manager_dual_arm_->registerObject(&objects_dual_arm_[k]);
        }

        manager_dual_arm_->setup();

        ROS_DEBUG_STREAM("CollisionDetectionFCL::initializeRobotManager(): finish.");
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::setDloObjectsTransform(
        const DLOState &dlo_state)
    {
        int n_edges = dlo_->num_fps_ - 1;
        for (int k = 0; k < n_edges; k++)
        {
            Eigen::Vector3d fp_0 = dlo_state.getFpPos(k);
            Eigen::Vector3d fp_1 = dlo_state.getFpPos(k + 1);

            Eigen::Vector3d capsule_pos = (fp_0 + fp_1) / 2.0;

            Eigen::Vector3d edge = fp_1 - fp_0;
            Eigen::Vector3d z_axis_capsule = edge.normalized();
            Eigen::Vector3d z_axis_world(0.0, 0.0, 1.0);
            Eigen::Vector3d cross = z_axis_world.cross(z_axis_capsule);
            Eigen::Vector3d capsule_rot_axis = cross.normalized();
            double capsule_rot_angle = std::atan2(cross.norm(), z_axis_world.dot(z_axis_capsule));
            Eigen::AngleAxisd capsule_rot_vec(capsule_rot_angle, capsule_rot_axis);

            Eigen::Isometry3d capsule_pose = Utils::EigenPosQuatVec2Isometry3d(capsule_pos, Eigen::Quaterniond(capsule_rot_vec));

            objects_dlo_[k].setTransform(capsule_pose);
            objects_dlo_[k].computeAABB();
        }

        manager_dlo_->update();
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::setRobotObjectsTransform(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos)
    {
        dual_arm_->arm_0_->setJointPositions(arm_0_joint_pos);
        dual_arm_->arm_1_->setJointPositions(arm_1_joint_pos);

        for (size_t k = 0; k < objects_dual_arm_.size(); k++)
        {
            auto &robot_data = objects_dual_arm_userdata_[k].robot_data;

            Eigen::Isometry3d link_pose;
            if (robot_data.group_name == "arm_0")
                link_pose = dual_arm_->arm_0_->getLinkPose(robot_data.link_name);
            else if (robot_data.group_name == "arm_1")
                link_pose = dual_arm_->arm_1_->getLinkPose(robot_data.link_name);
            else if (robot_data.group_name == "dual_base")
                link_pose = Eigen::Isometry3d::Identity();
            else
                ROS_ERROR_STREAM("CollisionDetectionFCL: invalid robot group_name: " << robot_data.group_name);

            objects_dual_arm_[k].setTransform(link_pose * robot_data.relative_pose);
            objects_dual_arm_[k].computeAABB();
        }
    }

    // -------------------------------------------------------
    Eigen::VectorXd CollisionDetectionFCL::getCriticalPointsPosVec(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos)
    {
        Eigen::VectorXd critical_points_by_fk = Eigen::VectorXd::Zero(objects_dual_arm_.size() * 3);

        dual_arm_->arm_0_->setJointPositions(arm_0_joint_pos);
        dual_arm_->arm_1_->setJointPositions(arm_1_joint_pos);

        for (size_t k = 0; k < objects_dual_arm_.size(); k++)
        {
            auto &robot_data = objects_dual_arm_userdata_[k].robot_data;

            Eigen::Isometry3d link_pose;
            if (robot_data.group_name == "arm_0")
                link_pose = dual_arm_->arm_0_->getLinkPose(robot_data.link_name);
            else if (robot_data.group_name == "arm_1")
                link_pose = dual_arm_->arm_1_->getLinkPose(robot_data.link_name);
            else if (robot_data.group_name == "dual_base")
                link_pose = Eigen::Isometry3d::Identity();
            else
                ROS_ERROR_STREAM("CollisionDetectionFCL: invalid robot group_name: " << robot_data.group_name);

            Eigen::Isometry3d critical_point_pose = link_pose * robot_data.relative_pose;
            critical_points_by_fk.block<3, 1>(3 * k, 0) = critical_point_pose.translation();
        }

        return critical_points_by_fk;
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::getInfoForVisualizeDLO(
        VecEigenIsometry3d &poses,
        double &capsule_length,
        double &capsule_radius)
    {
        poses.clear();
        poses = VecEigenIsometry3d(objects_dlo_.size());

        for (int k = 0; k < objects_dlo_.size(); k++)
        {
            poses[k] = objects_dlo_[k].getTransform();
        }

        capsule_length = dlo_->dlo_length_ / (dlo_->num_fps_ - 1);
        capsule_radius = dlo_capsule_radius_;
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::getInfoForVisualizeRobot(
        VecEigenIsometry3d &poses,
        std::vector<double> &capsule_lengths,
        std::vector<double> &capsule_radiuses)
    {
        poses.clear();
        capsule_lengths.clear();
        capsule_radiuses.clear();

        for (int i = 0; i < objects_dual_arm_.size(); i++)
        {
            if (objects_dual_arm_userdata_[i].geometry_type != "capsule")
                ROS_ERROR_STREAM("CollisionDetectionFCL::getInfoForVisualizeRobot() only supports capsule collision objects, current: " << objects_dual_arm_userdata_[i].geometry_type);

            poses.push_back(objects_dual_arm_[i].getTransform());
            capsule_radiuses.push_back(objects_dual_arm_userdata_[i].geometry_params[0]);
            capsule_lengths.push_back(objects_dual_arm_userdata_[i].geometry_params[1]);
        }
    }

    // -------------------------------------------------------
    void CollisionDetectionFCL::getInfoForVisualizeRobot(
        VecEigenIsometry3d &poses,
        std::vector<std::string> &geometry_types,
        std::vector<std::vector<double>> &geometry_params)
    {
        poses = VecEigenIsometry3d(objects_dual_arm_.size());
        geometry_types = std::vector<std::string>(objects_dual_arm_.size());
        geometry_params = std::vector<std::vector<double>>(objects_dual_arm_.size());

        for (int i = 0; i < objects_dual_arm_.size(); i++)
        {
            poses[i] = objects_dual_arm_[i].getTransform();
            geometry_types[i] = objects_dual_arm_userdata_[i].geometry_type;
            geometry_params[i] = objects_dual_arm_userdata_[i].geometry_params;
        }
    }

    // -------------------------------------------------------
    bool CollisionDetectionFCL::checkDLOSelfCollision()
    {
        fcl::DefaultCollisionData<double> collision_data;
        manager_dlo_->collide(&collision_data, collisionCallback);

        return collision_data.result.isCollision();
    }

    // -------------------------------------------------------
    double CollisionDetectionFCL::calcDLOSelfDistance()
    {
        fcl::DefaultDistanceData<double> distance_data; // https://docs.ros.org/en/noetic/api/fcl/html/structfcl_1_1DistanceResult.html
        manager_dlo_->distance(&distance_data, distanceCallback);

        return distance_data.result.min_distance;
    }

    // -------------------------------------------------------
    bool CollisionDetectionFCL::checkRobotSelfCollision()
    {
        fcl::DefaultCollisionData<double> collision_data;
        manager_dual_arm_->collide(&collision_data, collisionCallback);

        return collision_data.result.isCollision();
    }

    // -------------------------------------------------------
    bool CollisionDetectionFCL::checkDLOAndRobotCollision()
    {
        fcl::DefaultCollisionData<double> collision_data;
        manager_dual_arm_->collide(manager_dlo_, &collision_data, collisionCallback);

        return collision_data.result.isCollision();
    }

} // end namespace