#ifndef DLO_ARM_PLANNING_PLANNER_BASE_H
#define DLO_ARM_PLANNING_PLANNER_BASE_H

#include "dlo_arm_planning_pkg/common_include.h"
#include "dlo_arm_planning_pkg/scene.h"
#include "dlo_arm_planning_pkg/dual_arm.h"
#include "dlo_arm_planning_pkg/visualize.h"

#include "dlo_arm_planning_pkg/node.h"


namespace dlo_arm_planning_pkg{

// -----------------------------------
struct PlanningRequest
{
    // dlo
    double dlo_length_;
    DLOState start_dlo_state_;
    DLOState goal_dlo_state_;

    // dual arm
    std::string arm_0_group_name_;
    std::string arm_1_group_name_;
    std::string dual_arm_group_name_;

    std::vector<double> arm_0_start_joint_pos_;
    std::vector<double> arm_1_start_joint_pos_;

    std::string goal_type_;
    std::vector<double> arm_0_goal_joint_pos_;
    std::vector<double> arm_1_goal_joint_pos_;

    bool b_visualize_start_goal_ = false;
    bool b_visualize_planning_process_ = false;
    bool b_visualize_res_path_ = false;

    bool b_check_start_goal_collision_ = true;
};

// --------------------------------
struct Path
{
    std::vector<DLOState> dlo_path_;
    std::vector<std::vector<double> > arm_0_path_;
    std::vector<std::vector<double> > arm_1_path_;

    void clear(){
        dlo_path_.clear();
        arm_0_path_.clear();
        arm_1_path_.clear();
    }
};

// --------------------------------
struct PlanningResponse
{
    Path smoothed_path_;

    bool success_;
    double total_time_cost_;
    int total_iter_;
    double path_cost_ = 0.0;

    // details
    std::map<std::string, double> details_;
};


// -----------------------------------------------------------------
class PlannerBase
{
public:
    typedef std::shared_ptr<PlannerBase> Ptr;

    PlannerBase(
        const ros::NodeHandle& nh, 
        const Scene::Ptr &scene
    );

    void setVisualizer(Visualize::Ptr &visualizer);

    void loadCommonParams();

    virtual void loadParams() = 0;

    virtual void updateNode(
        const Node::Ptr &node
    );

    virtual void updateNodeDualArmJointPos(
        const Node::Ptr &node
    );

    virtual void updateNodeTcpPose(
        const Node::Ptr &node
    );

    virtual void updateNodeLinksPos(
        const Node::Ptr &node
    );

    virtual Node::Ptr randomSampleNode() = 0;

    virtual double twoNodeDistance(
        const Node::Ptr &node_0,
        const Node::Ptr &node_1,
        std::string dist_metric
    );

    virtual Node::Ptr getNearestNode(
        const std::vector<Node::Ptr> &node_list,
        const Node::Ptr &target_node,
        const std::string &dist_metric
    );

    virtual bool checkNodeCollision(const Node::Ptr &node);

    virtual bool checkTwoNodePathCollision(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    virtual bool checkTwoNodeCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        const double dlo_max_dist,
        const double arm_max_dist
    ) = 0;

    virtual Node::Ptr oneStepSteer(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    ) = 0;

    virtual Node::Ptr extend(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        bool greedy = false
    ) = 0;

    virtual std::vector<Node::Ptr> pathExtract(
        const Node::Ptr &node_forward_end, 
        const Node::Ptr &node_backward_end
    );

    virtual void pathSmooth(
        std::vector<Node::Ptr> &path_list
    );

    virtual std::vector<Node::Ptr> pathInterpolation(
        const std::vector<Node::Ptr> &path_list
    );

    virtual bool checkPlanningRequest(
        const PlanningRequest &req
    );

    virtual bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    ) = 0;

    virtual void swapTrees(
        std::vector<Node::Ptr> &node_list_a,
        std::vector<Node::Ptr> &node_list_b
    );

    virtual void swapNodes(
        Node::Ptr &node_a,
        Node::Ptr &node_b
    );

    virtual Eigen::VectorXd constraintError(
        const Eigen::VectorXd &pose,  // pos + rpy angle
        const Eigen::VectorXd &pose_lb,
        const Eigen::VectorXd &pose_ub
    );

    virtual bool constrainConfig(
        Node::Ptr &node,
        const Node::Ptr &node_old
    );

    virtual Node::Ptr sampleGoalIKNode(
        const DLOState &dlo_state
    );

    virtual void addNewGoalIKNode(
        const DLOState &dlo_state,
        std::vector<Node::Ptr> &node_list_goal
    );

    virtual double pathLength(
        const std::vector<Node::Ptr> &path,
        int begin_idx = -1,
        int end_idx = -1
    );

    virtual void nodeListToPath(
        const std::vector<Node::Ptr> &node_list,
        Path &path
    );

    virtual void printPlanningDetails();

    virtual void writePlanningDetailsToResponse(
        PlanningResponse &res
    );
    

    
public:

    std::string ALGORITHM_NAME;

    ros::NodeHandle nh_;

    Scene::Ptr scene_;
    DLO::Ptr dlo_;
    DualArm::Ptr dual_arm_;
    Visualize::Ptr visualizer_;

    PlanningRequest req_;

    // parameters
    double new_goal_ik_probability_;
    int num_random_sample_goal_;

    int rrt_max_iter_;
    int extend_max_steps_;
    int path_smooth_max_iter_;

    double path_collision_check_cartesian_step_size_;
    double path_collision_check_step_size_;
    double collision_check_min_dist_thres_;

    double steer_joint_step_size_;
    double steer_dlo_cartesian_step_size_;
    double steer_dlo_angle_step_size_;

    double connect_cartesian_max_dist_;
    double connect_joint_max_dist_;

    double pose_dist_pos_weight_, pose_dist_rot_weight_;

    bool use_underactuated_steer_;

    double path_interpolation_step_size_;

    double constraint_error_thres_;
    int constraint_projection_max_iter_;
    double constraint_step_gain_;

    Eigen::Vector3d dlo_sample_lb_, dlo_sample_ub_;

    double dlo_projection_function_tolerance_;
    double dlo_projection_max_num_iterations_;

    bool b_node_tcp_pose_ = false;
    bool b_node_links_pos_ = false;


    // 用于记录各部分的时间开销及其它细节
    int rrt_iter_find_feasible_path_ = 0.0;
    double time_cost_find_feasible_path_ = 0.0;
    double feasible_path_length_;
    double time_cost_path_smooth_ = 0.0;
    double smooth_path_length_;

    int count_arm_ik_ = 0;
    double time_cost_arm_ik_ = 0.0;
    int count_dlo_projection_ = 0;
    double time_cost_dlo_projection_ = 0.0;
    int count_collision_detection_ = 0.0;
    double time_cost_collision_detection_ = 0.0;
    int count_nearest_node_search_ = 0.0;
    double time_cost_nearest_node_search_ = 0.0;
    int count_random_sample_ = 0;
    int count_random_sample_iter_ = 0;
    double time_cost_random_sample_ = 0.0;
    int count_arm_projection_ = 0;
    double time_cost_arm_projection_ = 0.0;
    
    


}; // end class


} // end namespace

#endif