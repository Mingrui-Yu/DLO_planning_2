using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.My;
using RosMessageTypes.Moveit;
using RosMessageTypes.Trajectory;
using RosMessageTypes.BuiltinInterfaces;

// using RosMessageTypes.Control;


public class Robot: MonoBehaviour
{
    // robot 相关
    [HideInInspector] public string arm0Prefix = "dual_base/arm_0_base_link/arm_0_base_link_inertia";
    [HideInInspector] public string arm1Prefix = "dual_base/arm_1_base_link/arm_1_base_link_inertia";
    [HideInInspector] public string[] arm0LinkNames = {"/arm_0_shoulder_link", "/arm_0_upper_arm_link", "/arm_0_forearm_link", 
                            "/arm_0_wrist_1_link", "/arm_0_wrist_2_link", "/arm_0_wrist_3_link"};
    [HideInInspector] public string[] arm1LinkNames = {"/arm_1_shoulder_link", "/arm_1_upper_arm_link", "/arm_1_forearm_link", 
                            "/arm_1_wrist_1_link","/arm_1_wrist_2_link", "/arm_1_wrist_3_link"};
    [HideInInspector] public string gripper0Prefix = "/arm_0_flange/arm_0_tool0";
    [HideInInspector] public string gripper1Prefix = "/arm_1_flange/arm_1_tool0";
    [HideInInspector] public string[] dualArmGripperJointNames = {"arm_0_shoulder_pan_joint", 
                                        "arm_0_shoulder_lift_joint",
                                        "arm_0_elbow_joint",
                                        "arm_0_wrist_1_joint",
                                        "arm_0_wrist_2_joint",
                                        "arm_0_wrist_3_joint",
                                        "arm_1_shoulder_pan_joint",
                                        "arm_1_shoulder_lift_joint",
                                        "arm_1_elbow_joint",
                                        "arm_1_wrist_1_joint",
                                        "arm_1_wrist_2_joint",
                                        "arm_1_wrist_3_joint",
                                        "gripper_0_left_inner_knuckle_joint",
                                        "gripper_0_left_inner_finger_joint",
                                        "gripper_0_finger_joint",
                                        "gripper_0_right_inner_knuckle_joint",
                                        "gripper_0_right_inner_finger_joint",
                                        "gripper_0_right_finger_joint",
                                        "gripper_1_left_inner_knuckle_joint",
                                        "gripper_1_left_inner_finger_joint",
                                        "gripper_1_finger_joint",
                                        "gripper_1_right_inner_knuckle_joint",
                                        "gripper_1_right_inner_finger_joint",
                                        "gripper_1_right_finger_joint"};

    int numArm0Joints, numArm1Joints, numDualArmJoints, numGripper0Joints, numGripper1Joints;
    ArticulationBody[] dualArmJointArticulationBodies;
    string[] dualArmLinkNames;

    ArticulationBody[] gripper0JointArticulationBodies, gripper1JointArticulationBodies;

    // ROS 相关
    private ROSConnection m_Ros;
    string m_DualArmJointStatesTopicName = "state/dual_arm/joint_states";
    string m_DualArmGripperJointStatesTopicName = "state/dual_arm_gripper/joint_states";
    string m_DualArmJointVelCommandTopicName = "control/dual_arm/joint_vel_command";
    string m_DualArmJointTrajectoryTopicName = "control/dual_arm/joint_trajectory_command";
    string m_Arm0JointTrajectoryTopicName = "control/arm_0/joint_trajectory_command";
    string m_Arm1JointTrajectoryTopicName = "control/arm_1/joint_trajectory_command";

    DateTime startTime;

    string controlMode = "";
    float[] dualArmJointVelCommand;
    double[] dualArmLastTargetJointPos;
    // RobotTrajectoryMsg trajectoryCommand;
    // string trajectoryGroupName;
    // int trajectoryId = 0;

    long last_timestamp = 0;


    // ----------------------------------------------------
    void InitializeROS(){
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<JointStateMsg>(m_DualArmJointStatesTopicName);
        m_Ros.RegisterPublisher<JointStateMsg>(m_DualArmGripperJointStatesTopicName);
        m_Ros.Subscribe<VectorStampedMsg>(m_DualArmJointVelCommandTopicName, dualArmJointsVelCommandCb);
        m_Ros.Subscribe<RobotTrajectoryMsg>(m_DualArmJointTrajectoryTopicName, dualArmJointTrajectoryCommandCb);
        m_Ros.Subscribe<RobotTrajectoryMsg>(m_Arm0JointTrajectoryTopicName, arm0JointTrajectoryCommandCb);
        m_Ros.Subscribe<RobotTrajectoryMsg>(m_Arm1JointTrajectoryTopicName, arm1JointTrajectoryCommandCb);
    }


    // ----------------------------------------------------
    void Start(){
        Application.runInBackground = true;

        InitializeArms();

        InitializeGrippers();

        InitializeROS();
    
        startTime = System.DateTime.Now;

        
    }

    
    // ----------------------------------------------------
    void FixedUpdate(){

        publishDualArmJointStates();
        publishDualArmGripperJointStates();

        if(controlMode == "joint_velocity_control"){
            ExecuteDualArmJointVelocity(dualArmJointVelCommand);
        }

        // else if(controlMode == "joint_trajectory_control"){
        //     ExecuteDualArmJointPoint(trajectoryGroupName, trajectoryCommand.joint_trajectory.points[trajectoryId], Time.fixedDeltaTime);
        //     trajectoryId++;

        //     // stop & reset
        //     if(trajectoryId >= trajectoryCommand.joint_trajectory.points.Length){
        //         controlMode = "";
        //         trajectoryCommand = new RobotTrajectoryMsg();
        //         trajectoryGroupName = "";
        //         trajectoryId = 0;
        //     }
        // }

        // long current_timestamp = System.DateTime.Now.Ticks;
        // Debug.Log("dt in FixedUpdate: " + (current_timestamp - last_timestamp) / 10000000.0f);
        // last_timestamp = current_timestamp;

    }


    // ----------------------------------------------------
    void InitializeArms(){

        numArm0Joints = arm0LinkNames.Length;
        numArm1Joints = arm1LinkNames.Length;
        numDualArmJoints = numArm0Joints + numArm1Joints;

        // 提取所有的 arm joint 的 ArticulationBody
        dualArmJointArticulationBodies = new ArticulationBody[numArm0Joints + numArm1Joints];
        var linkName = arm0Prefix;
        for (var i = 0; i < numArm0Joints; i++){
            linkName += arm0LinkNames[i];
            dualArmJointArticulationBodies[i] = transform.Find(linkName).GetComponent<ArticulationBody>();
        }
        linkName = arm1Prefix;
        for (var i = 0; i < numArm1Joints; i++){
            linkName += arm1LinkNames[i];
            dualArmJointArticulationBodies[numArm0Joints + i] = transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // 预设置合适的stiffness和damping
        for(var i = 0; i < numDualArmJoints; i++){
            dualArmJointArticulationBodies[i].linearDamping = 0.05f;
            dualArmJointArticulationBodies[i].angularDamping = 0.05f;
            dualArmJointArticulationBodies[i].jointFriction = 0.05f;

            var jointXDrive = dualArmJointArticulationBodies[i].xDrive;
            jointXDrive.stiffness = 10000.0f;
            jointXDrive.damping = 100.0f;
            jointXDrive.forceLimit = 1000000.0f;
            dualArmJointArticulationBodies[i].xDrive = jointXDrive;
        }

        ResetArms();
    }


    // ----------------------------------------------------
    public void ResetArms(){
         // 初始化机械臂关节角
        resetDualArmJointAngle(0, Mathf.PI);
        resetDualArmJointAngle(1, -3/4.0f * Mathf.PI);
        resetDualArmJointAngle(2, -1/2.0f * Mathf.PI);
        resetDualArmJointAngle(3, -3/4.0f * Mathf.PI);
        resetDualArmJointAngle(4, 0.0f);
        resetDualArmJointAngle(5, -1/2.0f * Mathf.PI);
        
        resetDualArmJointAngle(6, Mathf.PI);
        resetDualArmJointAngle(7, -1/4.0f * Mathf.PI);
        resetDualArmJointAngle(8, 1/2.0f * Mathf.PI);
        resetDualArmJointAngle(9, -1/4.0f * Mathf.PI);
        resetDualArmJointAngle(10, 0.0f);
        resetDualArmJointAngle(11, 1/2.0f * Mathf.PI);

        // 重置机械臂控制模式
        controlMode = "";
    }


    // ----------------------------------------------------
    void InitializeGrippers(){
        // gripper 0
        // 顺序：left_inner_knuckle, left_inner_finger, left_outer_knuckle(finger_joint), right_inner_knuckle, right_inner_finger, right_outer_knuckle(right_finger_joint)
        // mimic finger joint: *1, *-1, *1, *1, *-1, *1
        numGripper0Joints = 6;
        gripper0JointArticulationBodies = new ArticulationBody[numGripper0Joints];
        var linkName = arm0Prefix;
        for (var i = 0; i < numArm0Joints; i++){
            linkName += arm0LinkNames[i];
        } 
        linkName += gripper0Prefix + "/gripper_0_base_link/gripper_0_robotiq_arg2f_base_link";
        gripper0JointArticulationBodies[0] = transform.Find(linkName + "/gripper_0_left_inner_knuckle").GetComponent<ArticulationBody>();
        gripper0JointArticulationBodies[1] = transform.Find(linkName + 
            "/gripper_0_left_outer_knuckle/gripper_0_left_outer_finger/gripper_0_left_inner_finger").GetComponent<ArticulationBody>();
        gripper0JointArticulationBodies[2] = transform.Find(linkName + "/gripper_0_left_outer_knuckle").GetComponent<ArticulationBody>();
        gripper0JointArticulationBodies[3] = transform.Find(linkName + "/gripper_0_right_inner_knuckle").GetComponent<ArticulationBody>();
        gripper0JointArticulationBodies[4] = transform.Find(linkName + 
            "/gripper_0_right_outer_knuckle/gripper_0_right_outer_finger/gripper_0_right_inner_finger").GetComponent<ArticulationBody>();
        gripper0JointArticulationBodies[5] = transform.Find(linkName + "/gripper_0_right_outer_knuckle").GetComponent<ArticulationBody>();

        // gripper 1
        numGripper1Joints = 6;
        gripper1JointArticulationBodies = new ArticulationBody[numGripper1Joints];
        linkName = arm1Prefix;
        for (var i = 0; i < numArm1Joints; i++){
            linkName += arm1LinkNames[i];
        } 
        linkName += gripper1Prefix + "/gripper_1_base_link/gripper_1_robotiq_arg2f_base_link";
        gripper1JointArticulationBodies[0] = transform.Find(linkName + "/gripper_1_left_inner_knuckle").GetComponent<ArticulationBody>();
        gripper1JointArticulationBodies[1] = transform.Find(linkName + 
            "/gripper_1_left_outer_knuckle/gripper_1_left_outer_finger/gripper_1_left_inner_finger").GetComponent<ArticulationBody>();
        gripper1JointArticulationBodies[2] = transform.Find(linkName + "/gripper_1_left_outer_knuckle").GetComponent<ArticulationBody>();
        gripper1JointArticulationBodies[3] = transform.Find(linkName + "/gripper_1_right_inner_knuckle").GetComponent<ArticulationBody>();
        gripper1JointArticulationBodies[4] = transform.Find(linkName + 
            "/gripper_1_right_outer_knuckle/gripper_1_right_outer_finger/gripper_1_right_inner_finger").GetComponent<ArticulationBody>();
        gripper1JointArticulationBodies[5] = transform.Find(linkName + "/gripper_1_right_outer_knuckle").GetComponent<ArticulationBody>();

        // // gripper 闭合到一定角度
        // Gripper0Move(0.6f);
        // Gripper1Move(0.6f);

        ResetGrippers();
    }


    // ----------------------------------------------------
    public void ResetGrippers(){
        Gripper0Move(0.0f);
        Gripper1Move(0.0f);
    }


    // ----------------------------------------------------
    JointStateMsg getDualArmJointStates(){
        var jointStates = new JointStateMsg();
        // jointStates.header.stamp = 

        // 读取关节角和关节角速度
        double[] jointPos = new double[numDualArmJoints];
        double[] jointVel = new double[numDualArmJoints];
        string[] jointNames = new string[numDualArmJoints];

        for(int i = 0; i < numDualArmJoints; i++){
            jointPos[i] = dualArmJointArticulationBodies[i].jointPosition[0];
            jointVel[i] = dualArmJointArticulationBodies[i].jointVelocity[0];
            jointNames[i] = dualArmGripperJointNames[i];
        }
        jointStates.position = jointPos;
        jointStates.velocity = jointVel;
        jointStates.name = jointNames;

        return jointStates;
    }


    // ----------------------------------------------------
    JointStateMsg getGripper0JointStates(){
        var jointStates = new JointStateMsg();
        // 读取关节角和关节角速度
        double[] jointPos = new double[numGripper0Joints];
        double[] jointVel = new double[numGripper0Joints];
        string[] jointNames = new string[numGripper0Joints];

        for(int i = 0; i < numGripper0Joints; i++){
            jointPos[i] = gripper0JointArticulationBodies[i].jointPosition[0];
            jointVel[i] = gripper0JointArticulationBodies[i].jointVelocity[0];
            jointNames[i] = dualArmGripperJointNames[numDualArmJoints + i];
        }
        jointStates.position = jointPos;
        jointStates.velocity = jointVel;
        jointStates.name = jointNames;

        return jointStates;
    }
    JointStateMsg getGripper1JointStates(){
        var jointStates = new JointStateMsg();
        // 读取关节角和关节角速度
        double[] jointPos = new double[numGripper1Joints];
        double[] jointVel = new double[numGripper1Joints];
        string[] jointNames = new string[numGripper1Joints];

        for(int i = 0; i < numGripper1Joints; i++){
            jointPos[i] = gripper1JointArticulationBodies[i].jointPosition[0];
            jointVel[i] = gripper1JointArticulationBodies[i].jointVelocity[0];
            jointNames[i] = dualArmGripperJointNames[numDualArmJoints + numGripper0Joints + i];
        }
        jointStates.position = jointPos;
        jointStates.velocity = jointVel;
        jointStates.name = jointNames;

        return jointStates;
    }


    // ----------------------------------------------------
    void publishDualArmJointStates(){
        m_Ros.Publish(m_DualArmJointStatesTopicName, getDualArmJointStates());
    }


    // ----------------------------------------------------
    // 发布包括dual arm + gripper的joint states
    void publishDualArmGripperJointStates(){
        var dualArmJointState = getDualArmJointStates();
        var gripper0JointState = getGripper0JointStates();
        var gripper1JointState = getGripper1JointStates();

        // 把dualArmJointState, gripper0JointState, gripper1JointState数据拼接在一起
        var jointStates = new JointStateMsg();
        double[] jointPos = new double[numDualArmJoints + numGripper0Joints + numGripper1Joints];
        double[] jointVel = new double[numDualArmJoints + numGripper0Joints + numGripper1Joints];
        string[] jointNames = new string[numDualArmJoints + numGripper0Joints + numGripper1Joints];
        for(int i = 0; i < numDualArmJoints; i++){
            jointPos[i] = dualArmJointState.position[i];
            jointVel[i] = dualArmJointState.velocity[i];
            jointNames[i] = dualArmJointState.name[i];
        }
        for(int i = 0; i < numGripper0Joints; i++){
            jointPos[numDualArmJoints + i] = gripper0JointState.position[i];
            jointVel[numDualArmJoints + i] = gripper0JointState.velocity[i];
            jointNames[numDualArmJoints + i] = gripper0JointState.name[i];
        }
        for(int i = 0; i < numGripper1Joints; i++){
            jointPos[numDualArmJoints + numGripper0Joints + i] = gripper1JointState.position[i];
            jointVel[numDualArmJoints + numGripper0Joints + i] = gripper1JointState.velocity[i];
            jointNames[numDualArmJoints + numGripper0Joints + i] = gripper1JointState.name[i];
        }
        jointStates.position = jointPos;
        jointStates.velocity = jointVel;
        jointStates.name = jointNames;

        m_Ros.Publish(m_DualArmGripperJointStatesTopicName, jointStates);
    }


    // ----------------------------------------------------
    public void Gripper0Move(float target_angle){
        // target_angle 的单位为 rad
        for(int i = 0; i < gripper0JointArticulationBodies.Length; i++){
            var jointXDrive = gripper0JointArticulationBodies[i].xDrive;
            // mimic finger joint
            if(i == 1 || i == 4) jointXDrive.target = -target_angle * Mathf.Rad2Deg;
            else jointXDrive.target = target_angle * Mathf.Rad2Deg;
            gripper0JointArticulationBodies[i].xDrive = jointXDrive;
        }
    }
    public void Gripper1Move(float target_angle){
        // target_angle 的单位为 rad，上限 0.8757
        for(int i = 0; i < gripper1JointArticulationBodies.Length; i++){
            var jointXDrive = gripper1JointArticulationBodies[i].xDrive;
            // mimic finger joint
            if(i == 1 || i == 4) jointXDrive.target = -target_angle * Mathf.Rad2Deg;
            else jointXDrive.target = target_angle * Mathf.Rad2Deg;
            gripper1JointArticulationBodies[i].xDrive = jointXDrive;
        }
    }


    // ----------------------------------------------------
    void resetDualArmJointAngle(int joint_id, float angle) // angle unit: rad
    {
        var jointPos = dualArmJointArticulationBodies[joint_id].jointPosition;
        var jointXDrive = dualArmJointArticulationBodies[joint_id].xDrive;

        jointPos[0] = angle;
        jointXDrive.target = angle * Mathf.Rad2Deg;

        dualArmJointArticulationBodies[joint_id].jointPosition = jointPos;
        dualArmJointArticulationBodies[joint_id].xDrive = jointXDrive;
    }


    // ----------------------------------------------------
    void dualArmJointsVelCommandCb(VectorStampedMsg msg){
        float[] jointVelCommand = msg.data;
        // ExecuteDualArmJointVelocity(joint_vel_command);
        dualArmJointVelCommand = jointVelCommand;
        controlMode = "joint_velocity_control";
    }


    // ----------------------------------------------------
    void ExecuteDualArmJointVelocity(float[] joint_vel_command){
        for (var joint_id = 0; joint_id < joint_vel_command.Length; joint_id++){
            var jointXDrive = dualArmJointArticulationBodies[joint_id].xDrive;
            // float jointPos = dualArmJointArticulationBodies[joint_id].jointPosition[0]; // unit: rad

            jointXDrive.stiffness = 10000.0f;
            jointXDrive.damping = 100.0f;

            jointXDrive.target += (Time.fixedDeltaTime * joint_vel_command[joint_id]) * Mathf.Rad2Deg;
            
            jointXDrive.targetVelocity = joint_vel_command[joint_id] * Mathf.Rad2Deg;

            dualArmJointArticulationBodies[joint_id].xDrive = jointXDrive;
        }
    }


    // ----------------------------------------------------
    void dualArmJointTrajectoryCommandCb(RobotTrajectoryMsg trajectory){
        // ExecuteDualArmTrajectory("dual_arm", trajectory);
        StartCoroutine(ExecuteDualArmTrajectory("dual_arm", trajectory));
        controlMode = "joint_trajectory_control";
    }
    // ----------------------------------------------------
    void arm0JointTrajectoryCommandCb(RobotTrajectoryMsg trajectory){
        // ExecuteDualArmTrajectory("arm_0", trajectory);
        StartCoroutine(ExecuteDualArmTrajectory("arm_0", trajectory));
        controlMode = "joint_trajectory_control";
    }
    // ----------------------------------------------------
    void arm1JointTrajectoryCommandCb(RobotTrajectoryMsg trajectory){
        // ExecuteDualArmTrajectory("arm_1", trajectory);
        StartCoroutine(ExecuteDualArmTrajectory("arm_1", trajectory));
        controlMode = "joint_trajectory_control";
    }

    // ----------------------------------------------------
    IEnumerator ExecuteDualArmTrajectory(string group_name, RobotTrajectoryMsg trajectory){
        var interpolate_trajectory = TrajectoryInterpolation(trajectory);

        long t_start = DateTime.Now.Ticks;
        int idx = 0;

        foreach (var waypoint in interpolate_trajectory.joint_trajectory.points){
            yield return StartCoroutine(ExecuteDualArmJointPoint(group_name, waypoint, 0.0f));

            // 维持时间上准确的轨迹跟踪
            while(true){
                long t_now = System.DateTime.Now.Ticks;
                if((t_now - t_start)/ 10000000.0f >= SecsNsecs2Secs(waypoint.time_from_start.sec, waypoint.time_from_start.nanosec)){
                    break;
                }
            }

            var dualArmJointState = getDualArmJointStates();
            if(dualArmJointState.position.Length == waypoint.positions.Length){
                for(int i = 0; i < dualArmJointState.position.Length; i++)
                    Debug.Log(i + ": " + (dualArmJointState.position[i] - waypoint.positions[i]));
            }
            

            idx++;
        }

        // long t_end = System.DateTime.Now.Ticks;
        // Debug.Log("real time duration: " + (t_end - t_start) / 10000000.0f );
        // int traj_length = trajectory.joint_trajectory.points.Length;
        // var point = trajectory.joint_trajectory.points[traj_length-1];
        // Debug.Log("planned time duration: " + SecsNsecs2Secs(point.time_from_start.sec, point.time_from_start.nanosec));
    }


    // ----------------------------------------------------
    // 适用于精细的有速度信息的trajectory（以0.02s为间隔的waypoints）
    IEnumerator ExecuteDualArmJointPoint(string group_name, JointTrajectoryPointMsg target_point, float dt){
        double[] jointPos_;
        ArticulationBody[] jointArticulationBodies;

        Assert.IsTrue(group_name == "dual_arm" || group_name == "arm_0" || group_name == "arm_1");
        // 根据不同的group_name，设置不同的变量
        if(group_name == "dual_arm"){
            Assert.IsTrue(target_point.positions.Length == numDualArmJoints);
            jointPos_ = getDualArmJointStates().position;
            jointArticulationBodies = dualArmJointArticulationBodies;
        }else if(group_name == "arm_0"){
            Assert.IsTrue(target_point.positions.Length == numArm0Joints);
            jointPos_ = getDualArmJointStates().position.Skip(0).Take(numArm0Joints).ToArray(); // 截取数组某一段，eg: 去掉前0个，截取之后的numArm0Joints个
            jointArticulationBodies = dualArmJointArticulationBodies.Skip(0).Take(numArm0Joints).ToArray();
        }else{
            Assert.IsTrue(target_point.positions.Length == numArm1Joints);
            jointPos_ = getDualArmJointStates().position.Skip(numArm0Joints).Take(numArm1Joints).ToArray();
            jointArticulationBodies = dualArmJointArticulationBodies.Skip(numArm0Joints).Take(numArm1Joints).ToArray();
        }

        // double[] to float[]
        float[] jointTargetPositions = toFloatArray(target_point.positions);
        float[] jointTargetVelocities = toFloatArray(target_point.velocities);
        float[] jointPos = toFloatArray(jointPos_);

        // Set the joint target position for every joint
        for(int joint_id = 0; joint_id < jointTargetPositions.Length; joint_id++){
            var jointXDrive = jointArticulationBodies[joint_id].xDrive;

            jointXDrive.stiffness = 10000.0f; // 比较合适
            jointXDrive.damping = 100.0f;

            jointXDrive.target = jointTargetPositions[joint_id] * Mathf.Rad2Deg;
            jointXDrive.targetVelocity = jointTargetVelocities[joint_id] * Mathf.Rad2Deg;

            jointArticulationBodies[joint_id].xDrive = jointXDrive;
        }

        yield return new WaitForSeconds(dt); // 控制频率50Hz
    }


    // ----------------------------------------------------
    // 将粗糙的trajectory插值成0.02s为间隔的trajectory（精细、有速度信息）
    RobotTrajectoryMsg TrajectoryInterpolation(RobotTrajectoryMsg trajectory){
        
        List<JointTrajectoryPointMsg> new_points_list = new List<JointTrajectoryPointMsg>();
        float t = 0.0f;
        int trajectory_length = trajectory.joint_trajectory.points.Length;

        for(int i = 0; i < trajectory_length - 1; i++){
            // 两个相邻的waypoint之间做线性插值
            var current_point = trajectory.joint_trajectory.points[i];
            var next_point = trajectory.joint_trajectory.points[i + 1];
            var t0 = SecsNsecs2Secs(current_point.time_from_start.sec, current_point.time_from_start.nanosec); // 注意这里和ROS中定义的不一样， 参考Runtime/Messages/HandwrittenMessages/msg/DurationMsg.cs
            var t1 = SecsNsecs2Secs(next_point.time_from_start.sec, next_point.time_from_start.nanosec);
            int numJoints = current_point.positions.Length;

            // 每个0.02s，加一个新waypoint
            while (t < t1){
                JointTrajectoryPointMsg point = new JointTrajectoryPointMsg();
                double[] jointPos = new double[numJoints];
                double[] jointVel = new double[numJoints];

                for(int j = 0; j < numJoints; j++){
                    float x0 = (float)current_point.positions[j];
                    float v0 = (float)current_point.velocities[j];
                    float x1 = (float)next_point.positions[j];
                    float v1 = (float)next_point.velocities[j];

                    // 如果送进来的waypoint没有速度信息（但有时间信息），则认为是匀速直线运动
                    if(v0 == 0 || v1 == 0){ 
                        float v = (x1 - x0) / (t1 - t0);
                        jointVel[j] = (double)v;
                        jointPos[j] = (double)(x0 + v * (t - t0));
                    }
                    // 如果送进来的waypoint有速度信息，则认为是匀加速直线运动
                    else{ 
                        // 速度计算出的加速度 和 位置计算出的加速度，分别计算是为了保证t1时刻速度一定是v1，位置一定是x1.
                        float av = (v1 - v0) / (t1 - t0);
                        float ax = 2.0f / ((t1 - t0)*(t1 - t0)) * ((x1 - x0) - v0 * (t1 - t0));
                        jointVel[j] = (double)(v0 + av * (t - t0));
                        jointPos[j] = (double)(x0 + v0 * (t - t0) + ax / 2.0f * (t - t0) * (t - t0));
                    }
                }

                point.positions = jointPos;
                point.velocities = jointVel;

                int secs = (int)t;
                int nanosec = (int)(1e9 * (t - secs));
                point.time_from_start = new DurationMsg(secs, nanosec); // 时间戳

                new_points_list.Add(point);

                t += Time.fixedDeltaTime;
            }
        }
        // 最后，加上原trajectory的destination
        new_points_list.Add(trajectory.joint_trajectory.points[trajectory_length - 1]);

        RobotTrajectoryMsg new_trajectory = new RobotTrajectoryMsg();
        new_trajectory.joint_trajectory.points = new_points_list.ToArray();
        return new_trajectory;
    }



    // ----------------------------------------------------
    float SecsNsecs2Secs(int secs, int nsecs){
        return (float)secs + (float)nsecs * 1e-9f;
    }

    // ----------------------------------------------------
    float[] Saturate(float[] currentPos, float[] targetPos, float thres){
        Assert.IsTrue(currentPos.Length == targetPos.Length);

        float dist = Distance(currentPos, targetPos);
        float[] targetSat = new float[targetPos.Length];

        if(dist < thres) targetSat = targetPos;
        else{
            for(int i = 0; i < targetPos.Length; i++){
                targetSat[i] = currentPos[i] + (targetPos[i] - currentPos[i]) / dist * thres;
            }
        }
        return targetSat;
    }

    // ----------------------------------------------------
    float Distance(float[] arr1, float[] arr2){
        Assert.IsTrue(arr1.Length == arr2.Length);
        float res = 0.0f;
        for(int i = 0; i < arr1.Length; i++){
            res += (arr1[i] - arr2[i]) * (arr1[i] - arr2[i]);
        }
        res = Mathf.Sqrt(res);
        return res;
    }

    // ----------------------------------------------------
    float[] toFloatArray(double[] arr) {
        if (arr == null) return null;
        int n = arr.Length;
        float[] ret = new float[n];
        for (int i = 0; i < n; i++) {
            ret[i] = (float)arr[i];
        }
        return ret;
    }




}; // class


















// ----------------------------------------------------
    // ExecuteJointTrajectoryServiceResponse TrajectoryServiceCb(ExecuteJointTrajectoryServiceRequest request){

    //     ExecuteJointTrajectoryServiceResponse response = new ExecuteJointTrajectoryServiceResponse();

    //     if (request.trajectories.Length > 0){
    //         StartCoroutine(ExecuteDualArmTrajectories(request));
    //         response.success = true;

    //     }else{
    //         Debug.Log("The request trajectory is null.");
    //         response.success = false;
    //     }

    //     return response;
    // }


    // // ----------------------------------------------------
    // IEnumerator ExecuteDualArmTrajectories(ExecuteJointTrajectoryServiceRequest request){
    //     // For every trajectory plan returned
    //     for (int trajIdx = 0; trajIdx < request.trajectories.Length; trajIdx++){

    //         // For every waypoint in trajectory plan
    //         int waypoint_id = 0;
    //         foreach (var waypoint in request.trajectories[trajIdx].joint_trajectory.points){
    //             var jointTargetPositions = waypoint.positions;

    //             // Set the joint values for every joint
    //             for(int joint_id = 0; joint_id < jointTargetPositions.Length; joint_id++){
    //                 var jointXDrive = dualArmJointArticulationBodies[joint_id].xDrive;
    //                 jointXDrive.stiffness = 1000.0f;
    //                 jointXDrive.damping = 100.0f;
    //                 jointXDrive.target = (float) jointTargetPositions[joint_id] * Mathf.Rad2Deg;
    //                 dualArmJointArticulationBodies[joint_id].xDrive = jointXDrive;
    //             }

    //             // // 判断是否到达该waypoint，不到达则block在这里
    //             // while(true){
    //             //     bool bReach = true;
    //             //     for(int joint_id = 0; joint_id < jointTargetPositions.Length; joint_id++){
    //             //         var jointPos = dualArmJointArticulationBodies[joint_id].jointPosition[0];
    //             //         // 差距大于5度，则认为未到达
    //             //         if(Mathf.Abs(jointPos - (float) jointTargetPositions[joint_id]) * Mathf.Rad2Deg > 5){
    //             //             bReach = false;
    //             //             break;
    //             //         }
    //             //     }
    //             //     if(bReach == true){
    //             //         Debug.Log("Reach the waypoint." + waypoint_id);
    //             //         // 到达该waypoint
    //             //         break;
    //             //     }

    //             yield return new WaitForSeconds(0.1f); // Wait for the robot to achieve the final pose from joint assignment
    //             waypoint_id++;
    //         }
    //     }

    // }





    // // ----------------------------------------------------
    // IEnumerator ExecuteDualArmJointPoint(string group_name, JointTrajectoryPointMsg target_point){
    //     int iter = 0;
    //     // 通过迭代，实现饱和的target joint position，同时实现检测是否到达目标点
    //     while(iter < 1000){ // 最长允许20s

    //         double[] jointPos_;
    //         ArticulationBody[] jointArticulationBodies;
    //         float maxDeltaJointPos;

    //         Assert.IsTrue(group_name == "dual_arm" || group_name == "arm_0" || group_name == "arm_1");
    //         // 根据不同的group_name，设置不同的变量
    //         if(group_name == "dual_arm"){
    //             Assert.IsTrue(target_point.positions.Length == numDualArmJoints);
    //             jointPos_ = getDualArmJointStates().position;
    //             jointArticulationBodies = dualArmJointArticulationBodies;
    //             maxDeltaJointPos = 0.2f;
    //         }else if(group_name == "arm_0"){
    //             Assert.IsTrue(target_point.positions.Length == numArm0Joints);
    //             jointPos_ = getDualArmJointStates().position.Skip(0).Take(numArm0Joints).ToArray(); // 截取数组某一段，eg: 去掉前0个，截取之后的numArm0Joints个
    //             jointArticulationBodies = dualArmJointArticulationBodies.Skip(0).Take(numArm0Joints).ToArray();
    //             maxDeltaJointPos = 0.1f;
    //         }else{
    //             Assert.IsTrue(target_point.positions.Length == numArm1Joints);
    //             jointPos_ = getDualArmJointStates().position.Skip(numArm0Joints).Take(numArm1Joints).ToArray();
    //             jointArticulationBodies = dualArmJointArticulationBodies.Skip(numArm0Joints).Take(numArm1Joints).ToArray();
    //             maxDeltaJointPos = 0.1f;
    //         }

    //         // double[] to float[]
    //         float[] jointTargetPositions = toFloatArray(target_point.positions);
    //         float[] jointTargetVelocities = toFloatArray(target_point.velocities);
    //         float[] jointPos = toFloatArray(jointPos_);

    //         // 计算饱和后的目标关节角
    //         float[] saturateTargetPos = Saturate(jointPos, jointTargetPositions, maxDeltaJointPos);

    //         // Set the joint target position for every joint
    //         for(int joint_id = 0; joint_id < jointTargetPositions.Length; joint_id++){
    //             var jointXDrive = jointArticulationBodies[joint_id].xDrive;

    //             jointXDrive.stiffness = 10000.0f;
    //             jointXDrive.damping = 100.0f;

    //             jointXDrive.target = jointTargetPositions[joint_id] * Mathf.Rad2Deg;
    //             jointXDrive.targetVelocity = jointTargetVelocities[joint_id] * Mathf.Rad2Deg;

    //             jointArticulationBodies[joint_id].xDrive = jointXDrive;
    //         }

    //         // 判断是否到达该waypoint，到达才能执行完毕
    //         if(Distance(jointPos, jointTargetPositions) < 0.1f){ // 到达该waypoint
    //             print("Reach waypoint.");
    //             break;
    //         }
    //         iter++;
    //         yield return new WaitForSeconds(0.02f); // 控制频率50Hz
    //     }
    // }

