using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Obi;
using UnityEngine.SceneManagement;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.My;


public class SimArea: MonoBehaviour
{
    public float dloLength, dloRadius;
    public float dloBendComplianceX, dloBendComplianceY, dloTorsionCompliance;

    GameObject solverObject, dloObject, robotObject;
    ObiSolver solver;
    Robot robotScript;
    Rod dloScript;

    private ROSConnection m_Ros;
    string graspRodEndsServiceName = "/control/dual_gripper/grasp_dlo_ends";
    string resetSceneServiceName = "/unity/reset_scene";
    string loadNewSceneServiceName = "/unity/load_new_scene";

    int k = 0;


    // ----------------------------------------------------
    void Start(){
        Application.runInBackground = true;

        solverObject = GameObject.Find("Solver");
        solver = solverObject.GetComponent<ObiSolver>();

        // dloObject = GameObject.Find("Solver/Rod");
        // dloScript = dloObject.GetComponent<Rod>();
        CreateNewDLO();

        robotObject = GameObject.Find("dual_ur_robotiq2f85");
        robotScript = robotObject.GetComponent<Robot>();
        
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.ImplementService<TriggerRequest, TriggerResponse>(graspRodEndsServiceName, graspRodEndsServiceCb);
        m_Ros.ImplementService<TriggerRequest, TriggerResponse>(resetSceneServiceName, resetSceneServiceCb);
        m_Ros.ImplementService<LoadNewUnitySceneRequest, LoadNewUnitySceneResponse>(loadNewSceneServiceName, loadNewSceneServiceCb);
    }


    // ----------------------------------------------------
    void FixedUpdate(){
        k++;
    }


    // ----------------------------------------------------
    TriggerResponse graspRodEndsServiceCb(TriggerRequest request){
        TriggerResponse response = new TriggerResponse();

        AttachRodEndsToRobotEnds();

        response.success = true;
        return response;
    }


    // ----------------------------------------------------
    TriggerResponse resetSceneServiceCb(TriggerRequest request){
        TriggerResponse response = new TriggerResponse();

        ResetScene();

        response.success = true;
        return response;
    }


    // ----------------------------------------------------
    LoadNewUnitySceneResponse loadNewSceneServiceCb(LoadNewUnitySceneRequest request){
        LoadNewUnitySceneResponse response = new LoadNewUnitySceneResponse();

        SceneManager.LoadScene(request.scene_name);

        response.success = true;
        return response;
    }


    // ------------------------------------------------------------------
    void CreateNewDLO(){
        Random.InitState((int)System.DateTime.Now.Ticks);

        // 随机设置DLO的长度和粗细
        // dloLength = Random.Range(0.2f, 0.7f);
        // dloRadius = Random.Range(0.0025f, 0.01f);
        float dloBlueprintResolution = 1.0f;

        // create the blueprint: (ltObiRopeBlueprint, ObiRodBlueprint)
        var blueprint = ScriptableObject.CreateInstance<ObiRodBlueprint>();
        blueprint.thickness = dloRadius;
        blueprint.resolution = dloBlueprintResolution;
        blueprint.keepInitialShape = false;
        
        // Procedurally generate the rope path (a simple straight line):
        blueprint.path.Clear();
        // 这里参考了ObiRodBlueprint的Script中的写法，用官网Mannual里的写法似乎path是有问题的
        blueprint.path.InsertControlPoint(0, dloLength/2.0f * Vector3.left, Vector3.left * dloLength/4.0f, 
            Vector3.right * dloLength/4.0f, Vector3.up, 0.1f, 0.01f, 1, 1, Color.white, "start");
        blueprint.path.InsertControlPoint(1, dloLength/2.0f * Vector3.right, Vector3.left * dloLength/4.0f, 
            Vector3.right * dloLength/4.0f, Vector3.up, 0.1f, 0.01f, 1, 1, Color.white, "end");
        blueprint.path.FlushEvents();

        // generate the particle representation of the rope (wait until it has finished):
        // yield return StartCoroutine(blueprint.Generate()); // 用官网Mannual里的写法并不好用
        blueprint.Generate();
		
        // create a rope:
        dloObject = new GameObject("Rod", typeof(ObiRod), typeof(ObiRopeExtrudedRenderer));
        // 调整到合适的位置
        dloObject.transform.position = new Vector3(0.0f, -0.32f, -0.69f);

        // get component references:
        ObiRod rod = dloObject.GetComponent<ObiRod>();
        ObiRopeExtrudedRenderer rodRenderer = dloObject.GetComponent<ObiRopeExtrudedRenderer>();

        // load the default rope section:
        rodRenderer.section = Resources.Load<ObiRopeSection>("DefaultRopeSection");

        // instantiate and set the blueprint:
        rod.rodBlueprint = ScriptableObject.Instantiate(blueprint);

        // parent the cloth under a solver to start simulation:
        rod.transform.parent = solver.transform;

        // DLO设置为白色
        dloObject.GetComponent<Renderer>().material.color = new Color(1.0f, 1.0f, 1.0f);

        // 随机设置compliance
        // dloObject.GetComponent<ObiRod>().bend1Compliance = Random.Range(0.0f, 0.01f);
        // dloObject.GetComponent<ObiRod>().bend2Compliance = Random.Range(0.0f, 0.01f);
        // dloObject.GetComponent<ObiRod>().torsionCompliance = Random.Range(0.0f, 0.1f);
        dloObject.GetComponent<ObiRod>().bend1Compliance = dloBendComplianceX;
        dloObject.GetComponent<ObiRod>().bend2Compliance = dloBendComplianceY;
        dloObject.GetComponent<ObiRod>().torsionCompliance = dloTorsionCompliance;

        // 添加 ObiParticleRender
        ObiParticleRenderer particleRenderer = dloObject.AddComponent<ObiParticleRenderer>();
        particleRenderer.shader = Shader.Find("Obi/Particles");

        // 添加 Rod Script
        dloScript = dloObject.AddComponent<Rod>();
    }


    // ----------------------------------------------------
    void AttachRodEndsToRobotEnds(){
        // 要attach的robot link的名字（arm_x tool0）
        var linkName0 = "dual_ur_robotiq2f85/" + robotScript.arm0Prefix;
        for (var i = 0; i < robotScript.arm0LinkNames.Length; i++){
            linkName0 += robotScript.arm0LinkNames[i];
        } 
        linkName0 += robotScript.gripper0Prefix;

        var linkName1 = "dual_ur_robotiq2f85/" + robotScript.arm1Prefix;
        for (var i = 0; i < robotScript.arm1LinkNames.Length; i++){
            linkName1 += robotScript.arm1LinkNames[i];
        } 
        linkName1 += robotScript.gripper1Prefix;

        // rod 的 blueprint
        var blueprint = dloObject.GetComponent<ObiRod>().blueprint;

        // 分别建立两端的attachment
        ObiParticleAttachment particleAttachment_0 = dloObject.AddComponent<ObiParticleAttachment>();
        particleAttachment_0.target = GameObject.Find(linkName0).transform;
        particleAttachment_0.particleGroup = blueprint.groups[0]; // "start"
        particleAttachment_0.constrainOrientation = true;

        ObiParticleAttachment particleAttachment_1 = dloObject.AddComponent<ObiParticleAttachment>();
        particleAttachment_1.target = GameObject.Find(linkName1).transform;
        particleAttachment_1.particleGroup = blueprint.groups[1]; // "start"
        particleAttachment_1.constrainOrientation = true;

        // 闭合夹爪
        robotScript.Gripper0Move(0.7f);
        robotScript.Gripper1Move(0.7f);

        dloScript.UnfixedRod();
    }


    // ----------------------------------------------------
    void ResetScene(){
        // // 解除rod两端的attachment
        // ObiParticleAttachment particleAttachment_0 = dloObject.GetComponent<ObiParticleAttachment>();
        // DestroyImmediate(particleAttachment_0);
        // ObiParticleAttachment particleAttachment_1 = dloObject.GetComponent<ObiParticleAttachment>();
        // DestroyImmediate(particleAttachment_1);

        // dloScript.ResetRod();

        Destroy(dloObject);

        CreateNewDLO();

        robotScript.ResetArms();
        robotScript.ResetGrippers();
    }







}; // class