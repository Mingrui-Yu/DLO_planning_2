#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/dlo_state.h"
#include "dlo_arm_planning_pkg/dlo.h"
#include "dlo_arm_planning_pkg/visualize.h"
#include "dlo_arm_planning_pkg/planning_interface.h"

/**
 * add dual_base as collision obstacles to the scene
 */

using namespace dlo_arm_planning_pkg;
std::random_device rd;
std::default_random_engine Utils::rng(0);

// -------------------------------------------------------
void addObstacle()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "robot_base_1";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.08;
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.72;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose pose;
        pose.orientation.w = 0.7071;
        pose.orientation.x = 0.7071;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "robot_base_2";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 1.7;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.7;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();

    // add new collision objects
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep(); // required: leave some time for publishing the message to Move Group
}

// -----------------------------------------------------------------
int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    addObstacle();

    return 0;
}