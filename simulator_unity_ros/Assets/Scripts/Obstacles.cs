using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using NumSharp;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.My;

public class Obstacles : MonoBehaviour
{

    float resolution_ = 0.005f;
    int pointDimension_ = 3;


    // ROS 相关
    private ROSConnection m_Ros;
    string obstaclePointCloudTopicName_ = "/planning_env/pointcloud"; // 发送障碍物点云

    int k = 0;
    

    // -------------------------------------------------------------------
    void Start()
    {
        // 初始化ROS
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<PointCloud2Msg>(obstaclePointCloudTopicName_);
    }


    // -------------------------------------------------------------------
    void FixedUpdate()
    {
        // 只在unity开始运行时，发送一次pointcloud
        if(k == 50 * 1){
            publishObstaclePointCloud();
            // publishObstacleGeometry();
        }

        k++;
    }


    // -------------------------------------------------------------------
    void publishObstaclePointCloud(){

        List<Vector3> pointList = new List<Vector3>();

        // 遍历所有的cube obstacle
        foreach(Transform child_transform in this.transform){

            // 不考虑未被激活的物体
            if(child_transform.gameObject.activeSelf == false){
                continue;
            }
            // 对于tag为"not_obstacle"的物体，不发送其点云
            if(child_transform.gameObject.tag == "not_obstacle"){
                continue;
            }

            string type_name = child_transform.gameObject.GetComponent<MeshFilter>().mesh.name;

            // 目前只支持 obstacle 类型为 cube
            if(type_name == "Cube Instance"){
                Vector3 position = child_transform.position;
                // 半边长
                Vector3 scale = child_transform.localScale / 2.0f;
                scale = scale * 0.99f; // 补丁：防止 move group 的 octomap 多扩展一个 voxel

                for(float x = position[0] - scale[0]; x <= position[0] + scale[0] + resolution_; x += resolution_){
                    float point_x = Mathf.Min(x, position[0] + scale[0]);
                    for(float y = position[1] - scale[1]; y <= position[1] + scale[1] + resolution_; y += resolution_){
                        float point_y = Mathf.Min(y, position[1] + scale[1]);
                        for(float z = position[2] - scale[2]; z <= position[2] + scale[2] + resolution_; z += resolution_){
                            float point_z = Mathf.Min(z, position[2] + scale[2]);
                            Vector3 point = Unity2World(new Vector3(point_x, point_y, point_z));
                            // point.x -= 0.5f; // 从world转移到camera_link坐标系下
                            pointList.Add(point);
                        }
                    }
                }

                // for(float x = position[0] - scale[0] + resolution_; x <= position[0] + scale[0]; x += resolution_){
                //     float point_x = Mathf.Min(x, position[0] + scale[0] - resolution_);
                //     for(float y = position[1] - scale[1] + resolution_; y <= position[1] + scale[1]; y += resolution_){
                //         float point_y = Mathf.Min(y, position[1] + scale[1] - resolution_);
                //         for(float z = position[2] - scale[2] + resolution_; z <= position[2] + scale[2]; z += resolution_){
                //             float point_z = Mathf.Min(z, position[2] + scale[2] - resolution_);
                //             pointList.Add( Unity2World(new Vector3(point_x, point_y, point_z)) );
                //         }
                //     }
                // }
            }
            else{
                Debug.LogWarning("publishObstaclePointCloud(): unsupported object type!");
            }
        }

        int numPoint = pointList.Count;

        // List<Vector3> to float[]
        float[] pointArray = new float[numPoint * pointDimension_]; // 点数 * 维度             
        for(int i = 0; i < numPoint; i++){
            for(int j = 0; j < pointDimension_; j++){
                pointArray[pointDimension_ * i + j] = pointList[i][j];
            }
        }

        // float[] to byte[]
        byte[] pointByteData = new byte[pointArray.Length * sizeof(float)]; // 每个float需要用4位byte表示
        Buffer.BlockCopy(pointArray, 0, pointByteData, 0, pointByteData.Length);

        // 创建 pointCloud2 msg
        var pointCloud = new PointCloud2Msg();
        pointCloud.header.frame_id = "world";
        // pointCloud.header.frame_id = "camera_link";
        pointCloud.height = 1;
        pointCloud.width = (uint) numPoint;

        string name1 = "x"; string name2 = "y"; string name3 = "z";
        PointFieldMsg[] fields = {new PointFieldMsg(name1, 0, PointFieldMsg.FLOAT32, 1),
                                  new PointFieldMsg(name2, 4, PointFieldMsg.FLOAT32, 1), 
                                  new PointFieldMsg(name3, 8, PointFieldMsg.FLOAT32, 1)};
        pointCloud.fields = fields;
        pointCloud.is_bigendian = false;
        pointCloud.point_step = 3 * 4;
        pointCloud.row_step = (uint) numPoint * pointCloud.point_step;
        pointCloud.is_dense = false;
        pointCloud.data = pointByteData;

        // 发送
        m_Ros.Publish(obstaclePointCloudTopicName_, pointCloud);
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
