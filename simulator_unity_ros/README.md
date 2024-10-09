

## Documents

* [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* [ROS–Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration)
  * Publisher/Subscriber/Service
* [Unity Articulation Body](https://docs.unity3d.com/2020.3/Documentation/ScriptReference/ArticulationBody.html)



## Installation

### Unity 端配置

* [Unity安装所需package](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md)，其中安装package时选择git url会下载不下来，可以先clone下来，再locally install。
* 选择制定的Solver Type：[Setting Up the Robot](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/1_urdf.md#setting-up-the-robot)
* Menu -> RosSetting: 选择IP为127.0.0.1。

### URDF

* disable collison：在 urdf 中添加代码，disable 部分关节之间的 collision check。[参考](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/urdf_importer/urdf_appendix.md#disable-collision-support)。
    * 例如：gripper 内部 links; base_link_inertia 和 upper_arm_link。

### 导入 dual_ur 的 URDF

* urdf文件中每个link/joint的名字中不要出现"/"，否则会影响 Script 中`transform.Find<"linkName">`
* 将urdf文件放在Asserts/URDF目录下，同时，将含有所需mesh的ros package也放在Asserts/URDF目录下。
* 导入urdf，参考 [Setting Up the Robot](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/1_urdf.md#setting-up-the-robot)。
* ```Use Gravity``` 点击 ```disable``` 使其变为 ```enable```。
* Rotation Y 设为 -180.
* 调整 stiffness 等。
* 左右两臂的base_link都要选为immovable。
* 所有link都不勾选gravity。否则关节PD控制器效果不理想。
* 挂载 Robot Script。

### ROS端配置

```bash
$ cd ws_dlo
$ git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git  src/ros_tcp_endpoint
```



## Usage

### ROS Messages

Unity Robotics Hub 中已经有预定义的常用的msg/srv了，在 ROS-TCP-Connector-main/com.unity.robotics.ros-tcp-connector/Runtime/Messages 目录下。

生成自定义的msg：[Doc](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/2_ros_tcp.md#the-unity-side)。

### ROS-TCP-Endpoint

与ros通信，使用时需要运行：

```
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

