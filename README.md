


5.在   设置机械臂的ip地址
启动roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10时
加载ip地址，传入到该launch文件中
 <!-- publish the robot state (tf transforms) -->
  <node name="aubo_driver" pkg="aubo_driver" type="aubo_driver" >
  <param name="/server_host" type="str" value="$(arg robot_ip)"/>
  <!--<param name="/server_host" value="127.0.0.1"/> -->
  </node>


## 使用真实机器人
### 使用rviz和moveit
1. roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10

### 使用AuBoAPI函数接口
2. 先启动
   roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.10

   然后，启动驱动程序节点时，默认控制器为“ ros-controller”，如果要切换至“ robot-controller”
   rostopic pub -1 aubo_driver/controller_switch std_msgs/Int32 -- 0 

   最后启动
   rosrun aubo_driver testAuboAPI


### 使用仿真机器人
1. roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1  
2. roslaunch aubo_gazebo aubo_i5_gazebo_control.launch


### 下次实验
1. 使用API设定一个目标点，x,y,z并指定姿态
    给定位姿信息，然后求逆解，都利用 aubo_robot_namespace::wayPoint_S 数据结构，测试函数testMoveJ2
    该实验还差一部分未完成，还没确定给定的下，是不是相对于基座标系的。
	是基于基座标系的。
2. 使用轨迹运动的函数，给定一系列点，实验室给定6个，使用离线轨迹运动函数，观察这种运动模式是怎么样的
    先通过movej指令将机械臂移动到轨迹的初始点，然后在使用轨迹运动函数
    使用路径点文件的形式

3. 测试当使用网口控制机械臂的时候，示教器的急停开关是否有用，如果没用试着按下示教器的使能键试试看。










### 轨迹运动可以满足多个离散点路径的问题  即机械臂自带的B样条曲线
```
	使用接口类中的int robotServiceTrackMove()函数，查看接口文件。 怎么设置模式，其中几个模式分别代表什么意思
	再使用离线轨迹运动相关的函数接口

### 运动中的偏移属性。
