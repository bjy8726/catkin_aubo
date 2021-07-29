#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "robot_scanning/run_track_points.h"
#include "robot_scanning/trans_matrix.h"
#include <signal.h>

using namespace std;

RunTrackPoints auboControl;

void mySigHandle(int sig)
{
    //ROS_ERROR("STOP");
    //auboControl.stop();
    ros::shutdown();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //定义一个home位置
  std::vector<double> home_joints(6,0.0);//[6] = {-0.001255,0.0,-1.406503,0.0,-1.571295,-0.002450};
  //相对相机坐标系的轨迹点文件
  std::string text_cam;
  //相对工件坐标系的轨迹点文件
  std::string text_obj;
  //相对机械臂基坐标系的轨迹点文件
  std::string text_base;

  double tool_offset_x;
  double tool_offset_y;
  double tool_offset_z;

  std::vector<double> camera_posture;
  

  n.getParam("/joints_angle/home_joints",home_joints); //必须getParam(),且需要是vector形式，不然加载不进来
  n.param<std::string>("/scaning_files/text_cam",text_cam,"/home/bianjingyang/test1.txt");
  n.param<std::string>("/scaning_files/text_obj",text_obj,"/home/bianjingyang/test2.txt");
  n.param<std::string>("/scaning_files/text_base",text_base,"/home/bianjingyang/test3.txt");
  n.getParam("/scanning_camera/camera_posture",camera_posture); 
  n.param<double>("/scanning_tool/tool_offset_x",tool_offset_x,0.0);
  n.param<double>("/scanning_tool/tool_offset_y",tool_offset_y,0.0);
  n.param<double>("/scanning_tool/tool_offset_z",tool_offset_z,-0.1);


  signal(SIGINT,mySigHandle);

  //0.创建机械臂的moveit接口，并运动到指定位置
      auboControl.initMoveit();
      auboControl.setMaxVelAndAccScalingFactor(1.0,1.0);
//step1:
      auboControl.visualBlocking("Press 'next'1 in the RvizVisualToolsGui window to go to the init pose");
      auboControl.gotoTargetByJoint(home_joints);

//step2:
      //auboControl.visualBlocking("Press 'next'2 in the RvizVisualToolsGui window to get points cloud");
 

//step3: 
      auboControl.visualBlocking("Press 'next'3 in the RvizVisualToolsGui window to get path");
  //1.调用定位模块：确定相机坐标系与工件坐标系的关系(该模块封装成类)
      //1.1现在实验相机获取点云到执行轨迹这段时间，工件和相机不发生相对移动，可认为相机坐标系与工件坐标系重合
      Eigen::Matrix4d T_obj2cam;
      T_obj2cam<<1,0,0,0,
                 0,1,0,0,
                 0,0,1,0,
                 0,0,0,1;

  //2.调用点云获取及滤波模块

  //3.调用曲面重建及轨迹规划(轨迹点：基于相机坐标系，并保存到txt文本)
      //3.1 
      //3.2调用坐标转换模块：将相机坐标系中的轨迹点转换到工件坐标系(不同时间对同一个区域扫查时，step2.3.4只需执行一次)
      TransMatrix trackPointsTransMatrix; 
      //相机坐标系的轨迹点txt文本 ==> 工件坐标系的轨迹点txt文本(通过坐标变换矩阵实现)
      //text_obj = T_obj2cam*text_cam
      trackPointsTransMatrix.newTrackPointsFileByTransMatrix(text_cam,text_obj,T_obj2cam);

  //4.调用执行轨迹点模块：将工件坐标系中的轨迹点转换到机械臂的基坐标系，然后给机械臂去执行//变换矩阵计算(旋转和平移是以原坐标系为基准的)
      //4.1工件坐标系==>相机坐标系==>末端坐标系==>基坐标系 
      Eigen::Matrix4d T_base2end; //基座标系到末端坐标系的变换矩阵
      Eigen::Matrix4d T_end2cam;  //刚性连接坐标关系已知，也可通过标定获得
      Eigen::Matrix4d T_cam2obj;  //相机到工件的坐标系由step1的结果确定，T_cam2obj = T_obj2cam.inverse()
      Eigen::Matrix4d T_base2obj;
      Eigen::Matrix4d T_tool;

      //4.1.1确定相机坐标系到工件坐标系的关系，由定位模块得到
      T_cam2obj = T_obj2cam.inverse();


      //4.1.2确定末端坐标系到相机坐标系的关系,由参数camera_posture确定(自己设定或者标定)
      T_end2cam<<1,0,0,-0.0325,
                 0,1,0,-0.065,
                 0,0,1,0.00084,
                 0,0,0,1;                     
      T_end2cam = trackPointsTransMatrix.getTransMatrix(camera_posture);
      trackPointsTransMatrix.printfMatrix4d(T_end2cam,"T_end2cam");


      //4.1.3确定机械臂末端法兰相对于工具坐标系的关系，设置工具在x,y,z三个方向上的偏移量
      T_tool<<1,0,0,0,
              0,1,0,0,
              0,0,1,-0.1,
              0,0,0,1;
      T_tool(0,3) = tool_offset_x;
      T_tool(1,3) = tool_offset_y;
      T_tool(2,3) = tool_offset_z;
      trackPointsTransMatrix.printfMatrix4d(T_tool,"T_tool");
 
      
      //4.1.4确定基坐标系到末端坐标系的关系
      T_base2end = auboControl.getTransMatrix();

      //4.1.4将工件坐标系下的目标点转换到基座标系下的目标点，然后在减去超声扫查装置的尺寸，最后转成基座标系下末端法兰的位姿
    
            
      //目前工件坐标系认为是相机坐标系的原点
      trackPointsTransMatrix.newTrackPointsFileByTransMatrix(text_obj,text_base,
                                            T_base2end,T_tool,T_end2cam,T_cam2obj);//去除刀具的偏移量获得末端法兰轨迹
    
      //用于观察不去除工具偏移量的坐标点，用于比较
      std::string text_demo = "/home/bianjingyang/catkin_aubo/src/robot_scanning/io_files/text_demo.txt";
      Eigen::Matrix4d T = T_base2end*T_end2cam*T_cam2obj;
      trackPointsTransMatrix.newTrackPointsFileByTransMatrix(text_obj,text_demo,T);

//step4:      
      //4.2执行路径点
      auboControl.visualBlocking("Press 'next'4 in the RvizVisualToolsGui window to start scanning");          
      fstream inFile;
      inFile.open(text_base,ios::in);
      if(!inFile)
      {
          cout<<"open text_base failed!"<<endl;
          return -1;
      }
      string str_pose_base;
      std::vector<geometry_msgs::Pose> waypoints;
      bool isFirstPose = true;
      while(getline(inFile,str_pose_base))//是否到达文件末尾
      {                  
          if(str_pose_base != "NEXT")
          {
                //将一行字符串转换到字符串流
                stringstream stream_pose(str_pose_base);
                geometry_msgs::Pose target_pose;
                stream_pose >> target_pose.position.x;
                stream_pose >> target_pose.position.y;
                stream_pose >> target_pose.position.z;
                stream_pose >> target_pose.orientation.x;
                stream_pose >> target_pose.orientation.y;
                stream_pose >> target_pose.orientation.z;
                stream_pose >> target_pose.orientation.w;

                waypoints.push_back(target_pose);
          }
          else
          {
            //std::cout<<waypoints[0].position.x<<std::endl;
            //std::cout<<waypoints[1].position.x<<std::endl;
            auboControl.setMaxVelAndAccScalingFactor(0.1,1.0);
            //为避免与工件碰撞，执行所有轨迹之前，先到达第一个路径点的上方，z轴上面的10cm左右（认为设定）

            ROS_INFO("run a path\n");   
            ROS_INFO("the first point in the path:%f,%f,%f\n",waypoints[0].position.x,
                                                            waypoints[0].position.y,
                                                            waypoints[0].position.z);  
            auboControl.runWayPoints(waypoints);
            //创建一个临时容器，清空waypoints
            std::vector<geometry_msgs::Pose>().swap(waypoints);//当此语句执行完成时，临时容器销毁
          }
      }

      inFile.close();

      auboControl.gotoTargetByJoint(home_joints);

  return 0;
}
