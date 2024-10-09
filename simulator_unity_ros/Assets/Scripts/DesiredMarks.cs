using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using NumSharp;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.My;

public class DesiredMarks : MonoBehaviour
{

    string envDim = "3D";
    int numFPs = 10;

    public Material maskMaterial;
    public int desiredShapeIdx = 0;

    List<GameObject> marksList = new List<GameObject>();

    List<float[]> desiredFPsPositions_list = new List<float[]>();
    float[] desiredFPsPositions;
    string desiredShapeFile;

    // ROS 相关
    private ROSConnection m_Ros;
    string desiredShapeTopicName = "state/dlo/desired_shape"; // VectorStamped, dim: 3*num_fps
    

    // -------------------------------------------------------------------
    // Start is called before the first frame update
    void Start()
    {
        // desiredShapeFile = Application.streamingAssetsPath + @"/desired_shape/" + envDim + "/desired_positions.npy";

        // LoadDesiredShapes();

        for(int i = 0; i < numFPs; i++){
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
            sphere.GetComponent<Renderer>().material = maskMaterial;
            Collider collider = sphere.GetComponent<Collider>();
            Destroy(collider); // 删除collider
            marksList.Add(sphere);
        }

        // 初始化ROS
        m_Ros = ROSConnection.GetOrCreateInstance();
        // m_Ros.RegisterPublisher<VectorStampedMsg>(desiredShapeTopicName);

        m_Ros.Subscribe<VectorStampedMsg>(desiredShapeTopicName, DesiredShapeCb);
    }


    // -------------------------------------------------------------------
    void DesiredShapeCb(VectorStampedMsg msg)
    {
        desiredFPsPositions = msg.data;
    }


    // -------------------------------------------------------------------
    // Update is called once per frame
    void Update()
    {
        // desiredFPsPositions = desiredFPsPositions_list[desiredShapeIdx];

        // // 发布desired shape
        // PublishDesiredShape(desiredFPsPositions);

        if (desiredFPsPositions != null){
            // 将unity中的marker移动到各自的位置
            for(int i = 1; i < numFPs - 1; i++){
                marksList[i].transform.position = World2Unity(
                    new Vector3(desiredFPsPositions[3*i+0], desiredFPsPositions[3*i+1], desiredFPsPositions[3*i+2]));
            }
        }  
    }


    // // ------------------------------------------------------------------
    // void PublishDesiredShape(float[] desiredShape){
    //     // 发布 desired shape
    //     VectorStampedMsg desiredShapeMsg = new VectorStampedMsg();
    //     desiredShapeMsg.header.frame_id = "world";
    //     desiredShapeMsg.data = desiredShape;

    //     m_Ros.Publish(desiredShapeTopicName, desiredShapeMsg);
    // }


    // // --------------------------------------------------------------------
    // public void SetDesiredPositions(float[] d){
    //     desiredFPsPositions = d;
    // }


    // // --------------------------------------------------------------
    // void LoadDesiredShapes(){
    //     // 加载 desired_positions 文件
    //     if (File.Exists(desiredShapeFile)){
    //         // 加载 desired_shape
    //         float[] temp_total = np.Load<float[]>(desiredShapeFile);
    //         for(int i = 0; i < temp_total.Length/(3*numFPs); i++){
    //             float[] temp = new float[3 * numFPs];
    //             Array.ConstrainedCopy(temp_total, i*3*numFPs, temp, 0, 3*numFPs);
    //             desiredFPsPositions_list.Add(temp);
    //         }
    //     }else{
    //         print("No desired shape file.");
    //     }
    // }


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
        float angle = 0.0f;
        Vector3 axis = Vector3.zero;
        q.ToAngleAxis(out angle, out axis);
        axis = RotationUnity2World(axis);
        return Quaternion.AngleAxis(angle, axis);
    }

    public Quaternion RotationWorld2Unity(Quaternion q){
        float angle = 0.0f;
        Vector3 axis = Vector3.zero;
        q.ToAngleAxis(out angle, out axis);
        axis = RotationWorld2Unity(axis);
        return Quaternion.AngleAxis(angle, axis);
    }


} // end class
