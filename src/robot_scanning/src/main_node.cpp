#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "robot_scanning/run_track_points.h"
#include "robot_scanning/trans_matrix.h"

using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> home_joints(6,0.0);//[6] = {-0.001255,0.0,-1.406503,0.0,-1.571295,-0.002450};
  //相对相机坐标系的轨迹点文件
  std::string text_cam;
  //相对工件坐标系的轨迹点文件
  std::string text_obj;
  //相对机械臂基坐标系的轨迹点文件
  std::string text_base;
  //每次发送给moveit的点数
  int points_num_per_time;

  n.getParam("/joints_angle/home_joints",home_joints); //必须getParam()不然加载不进来
  n.param<std::string>("/scaning_files/text_cam",text_cam,"/home/bianjingyang/test1.txt");
  n.param<std::string>("/scaning_files/text_obj",text_obj,"/home/bianjingyang/test2.txt");
  n.param<std::string>("/scaning_files/text_base",text_base,"/home/bianjingyang/test3.txt");
  n.param("/scanning_motion/points_num_per_time",points_num_per_time,5);
  
  //std::cout<<home_joints[0]<<std::endl;
  
  //0.创建机械臂的moveit接口，并运动到指定位置
      RunTrackPoints auboControl;
      auboControl.initMoveit();
      auboControl.setMaxVelAndAccScalingFactor(0.03,0.1);
      auboControl.visualBlocking("Press 'next'1 in the RvizVisualToolsGui window to go to the pose");
      auboControl.gotoTargetByJoint(home_joints);
      double pose[6] = {-0.3,-0.4,0.2,3.14,0,-1.57};
      //auboControl.gotoTargetByPose(pose);
  
  //1.调用定位模块：确定相机坐标系与工件坐标系的关系(该模块封装成类)
      //1.1现在实验相机获取点云到执行轨迹这段时间，工件和相机不发生相对移动，可认为相机坐标系与工件坐标系重合
      Eigen::Matrix4d T_obj2cam;
      T_obj2cam<<1,0,0,0,
                 0,1,0,0,
                 0,0,1,0,
                 0,0,0,1;

  //2.调用点云获取及滤波模块



  //3.调用曲面重建及轨迹规划(轨迹点：基于相机坐标系，并保存到txt文本)



  //4.调用坐标转换模块：将相机坐标系中的轨迹点转换到工件坐标系(不同时间对同一个区域扫查时，step2.3.4只需执行一次)

      TransMatrix trackPointsTransMatrix; 
      //相机坐标系的轨迹点txt文本 ==> 工件坐标系的轨迹点txt文本(通过坐标变换矩阵实现)
      trackPointsTransMatrix.newTrackPointsFileByTransMatrix(text_cam,text_obj,T_obj2cam);
   

  //5.调用执行轨迹点模块：将工件坐标系中的轨迹点转换到机械臂的基坐标系，然后给机械臂去执行
      //5.1工件坐标系==>相机坐标系==>末端坐标系==>基坐标系 
      Eigen::Matrix4d T_base2end; //通过读取机械臂末端的当前位姿来求解
      Eigen::Matrix4d T_end2cam;  //刚性连接坐标关系已知，也可通过标定获得
      Eigen::Matrix4d T_cam2obj;  //相机到工件的坐标系由step1的结果确定，T_cam2obj = T_obj2cam.inverse()
      Eigen::Matrix4d T_base2obj;

      T_cam2obj = T_obj2cam.inverse();

      T_end2cam<<1,0,0,0.05,
                 0,1,0,0.05,
                 0,0,1,0.05,
                 0,0,0,1;

      T_base2end = auboControl.getTransMatrix();

      T_base2obj = T_base2end*T_end2cam*T_cam2obj;

      //trackPointsTransMatrix.newTrackPointsFileByTransMatrix(text_obj,text_base,T_base2obj);
      
      //5.2执行路径点
      auboControl.visualBlocking("Press 'next'2 in the RvizVisualToolsGui window to start scanning");          
      fstream inFile;
      inFile.open(text_base,ios::in);
      if(!inFile)
      {
          cout<<"open text_base failed!"<<endl;
          return -1;
      }
      while(!inFile.eof())//是否到达文件末尾
      {
          std::vector<geometry_msgs::Pose> waypoints;
          string str_pose_base;
          for(int i=0; i<points_num_per_time && getline(inFile,str_pose_base); i++)
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
          auboControl.runWayPoints(waypoints);
          //创建一个临时容器，清空waypoints
          std::vector<geometry_msgs::Pose>().swap(waypoints);//当此语句执行完成时，临时容器销毁
      }

      inFile.close();

  // END_MAIN_NODE
  ros::shutdown();
  return 0;
}
