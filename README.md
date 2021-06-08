## 使用真实机器人
### 使用moveit
1. moveit的参数配置，并打开rviz可视化界面
    roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10

2. 开始执行曲面重建及轨迹规划模块生成的轨迹点
    roslaunch robot_scanning run_track_points_moveit.launch

3. 或者执行aubo提供的demo:
    roslaunch aubo_demo MoveGroupInterface_To_Kinetic.launch

### 使用AuBoAPI函数接口
1. 首先启动
   roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10

2. 这一步不执行，如果使用moveit。启动驱动程序节点时，默认控制器为“ ros-controller”，如果要切换至“ robot-controller”
   rostopic pub -1 aubo_driver/controller_switch std_msgs/Int32 -- 0 

3. 最后启动
 roslaunch robot_scanning run_track_points_moveit.launch
或者
 rosrun aubo_driver testAuboAPI(执行时有些问题)

## 使用仿真机器人
1. roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1  
2. roslaunch aubo_gazebo aubo_i5_gazebo_control.launch
3. roslaunch robot_scanning run_track_points_moveit.launch
4. 点击Next按钮 in the RvizVisualToolsGui window(RvIZ中的)


## 扫查模块功能包(robot_scanning)说明 
1. 主要功能：解析轨迹离散点文件，并让机械臂执行
2. a.三维重建及路径规划模块生成轨迹离散点的最终坐标系是基于机械臂基坐标的，需要将相机坐标系的点转化到末端坐标系，然后在转化到机械臂的基坐标系，即生成了可供机械臂执行的点。变换矩阵：
    b.获取点云时，工件坐标系与相机坐标系重合，轨迹规划出的位姿点是基于工件坐标系(也是相机坐标系的)的，将位姿点转换到末端坐标系；当下次进行扫查时需要检测相机坐标系与工件坐标系的关系，将位姿点转换到新相机坐标系，在转换到新的机械臂末端坐标系，送给机器人执行。




3. 执行之前还需要定位：计算当前机械臂基坐标系与工件的相对位姿关系是否是进行曲面重建获得三维点云时的相对坐标系
4. 本次实验机械臂获取三维点云后，机械臂和工件均不动，上述相对坐标关系未发生改变，机械臂可以执行step2生成的点
5. 后期可以增加定位模块，分别检测获取三维点云时和执行扫查前的机械臂与工件坐标关系，执行step2生成的点之前先将其乘以一个变换矩阵。


## 注意
1. aubo本身的函数接口不太好用，头文件中使用方法说明不够详细，课题中先使用moveit的接口

2. 结合自己的机械臂固定方式，需要设置各个关节角的运动范围，防止碰撞:
    在示教器中配置
    或者修改 aubo_gazebo/urdf/aubo_i5.xacro
            aubo_description/urdf/aubo_i5.urdf 
            aubo_i5.urdf.xacro   中<joint>标签中的<limit>选项

3. 修改aubo_i5_moveit_config/config/joint_limits 中的关节最大速度，来限制轨迹点扫查的速度

3. 获取机械臂末端坐标系与基坐标系的关系
    3.1 ==使用仿真== ：调用函数move_group.getCurrentPose("wrist3_Link");
    `aubo官方提供的ROS包配置中，该函数返回的位姿是相对于aubo机械臂支撑件的坐标系的(/world)`
    `为了获得相对于机械臂基坐标系的位姿需要在z轴减去0.502,在Rviz中查看得到`

    3.2 ==真实机械臂== ：调用函数move_group.getCurrentPose("wrist3_Link");
    ` 是否需要减去-0.502 `
    测试结果：也要减



## IP参数传入分析
在 moveit_planning_execution.launch设置机械臂的ip地址
启动roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10时
加载ip地址，传入到该launch文件中
 <!-- publish the robot state (tf transforms) -->
  <node name="aubo_driver" pkg="aubo_driver" type="aubo_driver" >
  <param name="/server_host" type="str" value="$(arg robot_ip)"/>
  <!--<param name="/server_host" value="127.0.0.1"/> -->
  </node>

## 下次实验
1. 获取机械臂末端坐标系与基坐标系的关系
    调用函数move_group.getCurrentPose("wrist3_Link");

2. 由方向矢量变为四元数

3. 对一组点进行坐标变换

--------
1. 测试现有的text_base:注意机械臂是否会与环境碰撞，确定仿真环境中的工作点对应的实际环境，并确定给的点是否是基于机械臂base的
    A:目前不干扰，是基于base的
2. 生成全部轨迹点文件，先仿真然后在去执行真实robot

3. 规划多个点生成的路径执行时速度怎么控制





