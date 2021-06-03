#include "robot_scanning/trans_matrix.h"



//设计一个函数接口，传递平移矩阵和四元数
/*以下代码使用eigen库*/
  //1.计算base_link ==>  wrist3_Link(末端坐标系)的变换矩阵
  /*
  geometry_msgs::PoseStamped pose_end = move_group.getCurrentPose("wrist3_Link");
  
  std::cout<<"x:"<<pose_end.pose.position.x<<std::endl;
  std::cout<<"y:"<<pose_end.pose.position.y<<std::endl;
  std::cout<<"z:"<<pose_end.pose.position.z<<std::endl;
  std::cout<<"wx:"<<pose_end.pose.orientation.x<<std::endl;*/



  //使用四元数的成员函数matrit()对旋转矩阵赋值
  Eigen::Matrix3d R_base2end;