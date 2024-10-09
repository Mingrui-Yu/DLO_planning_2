// 本脚本用于计算rod fps 的速度, 通过差分 positions 获得 (因为直接读particle的velocities的数据有问题)
// 可用于保存desired positions

using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.My;



public class Rod : MonoBehaviour
{ 
    ObiActor actor;
    int numRodParticles;
    int numFPs = 10;
    int[] fpsIndexes;

    List<Vector3> initialParticlePositions = new List<Vector3>();
    List<Quaternion> initialParticleOrientations = new List<Quaternion>();
    List<float> initialParticleMasses = new List<float>();
    List<float> initialParticleRotationMasses = new List<float>();

    // 以下量都是在world坐标系下
    Vector3[] currentFPsVelocities;
    Vector3[] aveFPsVelocities;
    Vector3[] lastFPsPositions;
    Vector3[] currentFPsPositions;

    // ROS 相关
    private ROSConnection m_Ros;
    string dloFpsPosTopicName = "state/dlo/fps_pos"; // VectorStamped, dim: 3*num_fps
    string dloFpsVelTopicName = "state/dlo/fps_vel";

    string dloStateTopicName = "state/dlo/raw_state";


    // ------------------------------------------------------------------
    void Start()
    {
        InitializeRod();

        InitializeROS();
    }


    // ----------------------------------------------------
    void InitializeRod(){
        actor = GetComponent<ObiActor>();
        numRodParticles = actor.solverIndices.Length;

        // feature points 对应的 particle index
        fpsIndexes = new int[numFPs];
        for(int k = 0; k < numFPs - 1; k++) 
            fpsIndexes[k] = (int)Mathf.Round((float)k / (float)(numFPs - 1) * (float)(numRodParticles - 1));
        fpsIndexes[numFPs - 1] = numRodParticles - 1;

        // feature points 的颜色设置为蓝色
        for (int k = 0; k < numFPs; k++){
            int solverIndex = actor.solverIndices[fpsIndexes[k]];
            actor.solver.colors[solverIndex] = Color.blue;
        }

        // 改变particles的质量：注意invMass和invRotationMass必须都改
        for(int i = 0; i < actor.solverIndices.Length; i++){
            int solverIndex = actor.solverIndices[i];
            actor.solver.invMasses[solverIndex] *= 100;
            actor.solver.invRotationalMasses[solverIndex] *= 100;
        }

        // 记录particles的初始位置,便于reset
        for(int i = 0; i < actor.solverIndices.Length; i++){
            int solverIndex = actor.solverIndices[i];
            initialParticlePositions.Add(actor.solver.positions[solverIndex]);
            initialParticleOrientations.Add(actor.solver.orientations[solverIndex]);
        }

        // DLO 的长度
        // length = Vector3.Distance(initialParticlePositions[0], initialParticlePositions[numRodParticles - 1]);

        // 初始化数组
        currentFPsVelocities = new Vector3[numFPs];
        aveFPsVelocities = new Vector3[numFPs];
        lastFPsPositions = new Vector3[numFPs];
        currentFPsPositions = new Vector3[numFPs];

        FixedRod();
    }


    // ----------------------------------------------------
    public void FixedRod(){
        // 让 rod 两端保持不动
        int solverIndex = actor.solverIndices[0];
        actor.solver.invMasses[solverIndex] = 0;
        actor.solver.invRotationalMasses[solverIndex] = 0;

        solverIndex = actor.solverIndices[actor.solverIndices.Length - 1];
        actor.solver.invMasses[solverIndex] = 0;
        actor.solver.invRotationalMasses[solverIndex] = 0;
    }


    // ----------------------------------------------------
    public void UnfixedRod(){
        // 解除 rod 两端保持不动
        int solverIndex = actor.solverIndices[0];
        actor.solver.invMasses[solverIndex] = actor.solver.invMasses[actor.solverIndices[1]];
        actor.solver.invRotationalMasses[solverIndex] = actor.solver.invRotationalMasses[actor.solverIndices[1]];
        
        solverIndex = actor.solverIndices[actor.solverIndices.Length - 1];
        actor.solver.invMasses[solverIndex] = actor.solver.invMasses[actor.solverIndices[1]];
        actor.solver.invRotationalMasses[solverIndex] = actor.solver.invRotationalMasses[actor.solverIndices[1]];
    }


    // ----------------------------------------------------
    public void ResetRod(){
        for(int i = 0; i < actor.solverIndices.Length; i++){
            int solverIndex = actor.solverIndices[i];
            // particles恢复到初始位置
            actor.solver.positions[solverIndex] = initialParticlePositions[i];
            actor.solver.orientations[solverIndex] = initialParticleOrientations[i];
            actor.solver.velocities[solverIndex] = Vector3.zero;
            actor.solver.angularVelocities[solverIndex] = Vector3.zero;
        }

        FixedRod();
    }


    // ----------------------------------------------------
    void InitializeROS(){
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<VectorStampedMsg>(dloFpsPosTopicName);
        m_Ros.RegisterPublisher<VectorStampedMsg>(dloFpsVelTopicName);

        m_Ros.RegisterPublisher<DLOStateStampedMsg>(dloStateTopicName);
    }


    // ------------------------------------------------------------------
    void FixedUpdate()
    {   
        // DLO 状态获取, 更新频率 50hz。其中对速度进行移动平均。
        for(int i = 0; i < numFPs; i++){
            int solverIndex = actor.solverIndices[fpsIndexes[i]];
            // 位置
            currentFPsPositions[i] = Unity2World(actor.solver.positions[solverIndex]);
            // 速度
            currentFPsVelocities[i] = (currentFPsPositions[i] - lastFPsPositions[i]) / (1 * Time.fixedDeltaTime);
            aveFPsVelocities[i] = 0.7f * aveFPsVelocities[i] + 0.3f * currentFPsVelocities[i]; // 移动平均,滤除噪声

            lastFPsPositions[i] = currentFPsPositions[i];
        }

        // 发布 dlo state 相关的 topic
        // PublishRodState();
        PublishRodState2();
    }


    // ------------------------------------------------------------------
    void PublishRodState2(){
        // Vector3[] to float[]
        float[] fpsPos = new float[numFPs * 3];
        float[] fpsVel = new float[numFPs * 3];
        for(int k = 0; k < numFPs; k++){
            for(int j = 0; j < 3; j++){
                fpsPos[3 * k + j] = GetRodPositions()[k][j];
                fpsVel[3 * k + j] = GetRodVelocities()[k][j];
            }
        }

        // // 经测试发现，obi rod 的初始朝向不是 identity，所以手动进行矫正
        var correction_rot = new Quaternion(0.0f, -0.7071f, 0.0f, 0.7071f); // var correction_rot = Quaternion.Inverse(actor.solver.orientations[actor.solverIndices[0]]) * Quaternion.identity;
        var end_0_quat = RotationUnity2World((actor.solver.orientations[actor.solverIndices[0]]).normalized * correction_rot);
        var end_1_quat = RotationUnity2World((actor.solver.orientations[actor.solverIndices[numRodParticles-1]]).normalized * correction_rot);

        float dlo_length = 0.0f;
        for(int i = 0; i < actor.solverIndices.Length - 1; i++){
            int solverIndex_0 = actor.solverIndices[i];
            int solverIndex_1 = actor.solverIndices[i+1];
            dlo_length += Vector3.Distance(actor.solver.positions[solverIndex_0], actor.solver.positions[solverIndex_1]);
        }
        dlo_length = dlo_length * 0.95f; // 补丁：修正 Unity 中 rod 可以被压缩的问题

        // 发布 DLO state
        DLOStateStampedMsg dloStateMsg = new DLOStateStampedMsg();
        dloStateMsg.header.frame_id = "world";
        dloStateMsg.num_fps = numFPs;
        dloStateMsg.length = dlo_length;
        dloStateMsg.fps_pos = fpsPos;
        dloStateMsg.fps_vel = fpsVel;
        dloStateMsg.end_quat_0 = new float[4]{end_0_quat.x, end_0_quat.y, end_0_quat.z, end_0_quat.w};
        dloStateMsg.end_quat_1 = new float[4]{end_1_quat.x, end_1_quat.y, end_1_quat.z, end_1_quat.w};
        
        m_Ros.Publish(dloStateTopicName, dloStateMsg);
    }


    // ------------------------------------------------------------------
    Vector3[] GetRodPositions(){
        return currentFPsPositions;
    }


    // ------------------------------------------------------------------
    Vector3[] GetRodVelocities(){
        return aveFPsVelocities;
    }





    // --------------------------------------------------------------
    public Vector3 World2Unity(Vector3 vector3){
        return new Vector3(vector3.y, vector3.z, -vector3.x);
    }

    public Vector3 Unity2World(Vector3 vector3){
        return new Vector3(-vector3.z, vector3.x, vector3.y);
    }
    public Vector3 RotationWorld2Unity(Vector3 vector3){
        return - new Vector3(vector3.y, vector3.z, -vector3.x);
    }

    public Vector3 RotationUnity2World(Vector3 vector3){
        return - new Vector3(-vector3.z, vector3.x, vector3.y);
    }

    public Quaternion RotationUnity2World(Quaternion q){
        var q_xyz = RotationUnity2World(new Vector3(q.x, q.y, q.z));
        q.x = q_xyz.x;
        q.y = q_xyz.y;
        q.z = q_xyz.z;
        return q;
    }

    public Quaternion RotationWorld2Unity(Quaternion q){
        var q_xyz = RotationWorld2Unity(new Vector3(q.x, q.y, q.z));
        q.x = q_xyz.x;
        q.y = q_xyz.y;
        q.z = q_xyz.z;
        return q;
    }

    
} // end class
