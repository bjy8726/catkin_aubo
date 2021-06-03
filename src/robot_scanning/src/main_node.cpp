#include <ros/ros.h>
#include <iostream>
#include "robot_scanning/run_track_points.h"
#include <vector>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> home_joints(6,0.0);//[6] = {-0.001255,0.0,-1.406503,0.0,-1.571295,-0.002450};

  n.getParam("/joints_angle/home_joints",home_joints); //必须getParam()不然加载不进来
  std::cout<<home_joints[0]<<std::endl;

  RunTrackPoints auboControl;
  auboControl.initMoveit();
  auboControl.setMaxVelAndAccScalingFactor(0.03,0.1);

  //0.创建机械臂的moveit接口

  //1.调用点云获取及滤波模块

  //2.调用曲面重建及轨迹规划

  //3.将轨迹规划的结果转换到机械臂基坐标系
  //先全部转换到txt文本，最后在提供给moveit执行

  
  //4.调用执行轨迹点模块
  auboControl.visualBlocking("Press 'next'1 in the RvizVisualToolsGui window to start the demo");
  auboControl.gotoTargetByJoint(home_joints);


  // END_MAIN_NODE
  ros::shutdown();
  return 0;
}
