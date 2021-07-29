#include "robot_scanning/trans_matrix.h"

TransMatrix::TransMatrix()
{

}

TransMatrix::~TransMatrix()
{

}

//由平移和旋转(四元数)生成变换矩阵
Eigen::Matrix4d TransMatrix::getTransMatrix(Eigen::Vector3d &T,Eigen::Quaterniond &Q)
{
  Eigen::Matrix4d trans_matrix;
  Eigen::Matrix3d qua_matrix;
  qua_matrix = Q.matrix();
  //平移矩阵
  trans_matrix(0,3) = T[0];
  trans_matrix(1,3) = T[1];
  trans_matrix(2,3) = T[2];
  // 旋转矩阵
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans_matrix(i,j) = qua_matrix(i,j);
    }
  }
  //齐次坐标
  trans_matrix(3,0) = 0;
  trans_matrix(3,1) = 0;
  trans_matrix(3,2) = 0;
  trans_matrix(3,3) = 1;

  return trans_matrix;
}

//函数重载：提供不同的接口
Eigen::Matrix4d TransMatrix::getTransMatrix(std::vector<double> &posture)
{
  Eigen::Vector3d T;
  Eigen::Quaterniond Q;
  T[0] = posture[0];
  T[1] = posture[1];
  T[2] = posture[2];
  Q.x() = posture[3];
  Q.y() = posture[4];
  Q.z() = posture[5];
  Q.w() = posture[6];
    
  Eigen::Matrix4d trans_matrix;
  Eigen::Matrix3d qua_matrix;
  qua_matrix = Q.matrix();
  //平移矩阵
  trans_matrix(0,3) = T[0];
  trans_matrix(1,3) = T[1];
  trans_matrix(2,3) = T[2];
  // 旋转矩阵
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      trans_matrix(i,j) = qua_matrix(i,j);
    }
  }
  //齐次坐标
  trans_matrix(3,0) = 0;
  trans_matrix(3,1) = 0;
  trans_matrix(3,2) = 0;
  trans_matrix(3,3) = 1;

  return trans_matrix;
}

void TransMatrix::writeTrackPoints2File(Eigen::Matrix4d &TrackPoints_obj,fstream &outFile_obj)
{
  //输入x y z
  outFile_obj << TrackPoints_obj(0,3)<<"    ";
  outFile_obj << TrackPoints_obj(1,3)<<"    ";
  outFile_obj << TrackPoints_obj(2,3)<<"    ";

  Eigen::Matrix3d qua_matrix;
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
       qua_matrix(i,j) = TrackPoints_obj(i,j);
    }
  }
  Eigen::Quaterniond Q;
  Q = qua_matrix;

  outFile_obj<<Q.x()<<"    ";
  outFile_obj<<Q.y()<<"    ";
  outFile_obj<<Q.z()<<"    ";
  outFile_obj<<Q.w()<<endl;
}

//已知原始坐标系到目标坐标的变换矩阵/原始坐标系下的轨迹点，将原始坐标系下的轨迹点文件转换到目标坐标系
  /*
  input:text_init(init坐标系的轨迹点文本)
  output:text_target(target坐标系的轨迹点文本)
  input: init坐标系到target坐标系的变换矩阵

  TrackPoints_target = T_target2init*TrackPoints_init;
  */
void TransMatrix::newTrackPointsFileByTransMatrix(std::string &text_init,std::string &text_target,Eigen::Matrix4d &T_target2init)
{

      fstream inFile_init;
      inFile_init.open(text_init,ios::in);
      if(!inFile_init)
      {
          std::cout<<"open text_init failed!"<<std::endl;
          return;
      }
      fstream outFile_target;
      outFile_target.open(text_target,ios::out|ios::trunc);
      if(!outFile_target)
      {
          std::cout<<"open text_target failed!"<<std::endl;
          return;
      }
      string str_pose_init;
      Eigen::Quaterniond Q;
      //轨迹点用一个坐标系来表示，由旋转矩阵和平移矩阵组成，给出以当前点为原点的坐标系姿态
      Eigen::Matrix4d TrackPoints_init;
      Eigen::Matrix4d TrackPoints_target;
      Eigen::Vector3d T;
      while(getline(inFile_init,str_pose_init))//是否到达文件末尾
      {
          //getline(inFile_init,str_pose_init);   
          if(str_pose_init == "NEXT")
          {
            outFile_target<<"NEXT"<<std::endl;
            continue; 
          }

          stringstream stream_pose(str_pose_init);
          //x y z
          stream_pose >> T[0];
          stream_pose >> T[1];
          stream_pose >> T[2];
          //四元数
          stream_pose >> Q.x();
          stream_pose >> Q.y();
          stream_pose >> Q.z();
          stream_pose >> Q.w();
          TrackPoints_init = getTransMatrix(T,Q);    
          TrackPoints_target = T_target2init*TrackPoints_init;
          //将text_init坐标系下的位姿点写入文件text_target
          writeTrackPoints2File(TrackPoints_target,outFile_target);
      }
      inFile_init.close();
      outFile_target.close();
}


//已知原始坐标系到目标坐标的变换矩阵/原始坐标系下的轨迹点，将原始坐标系下的轨迹点文件转换到目标坐标系
  /*
  input:text_init(init坐标系的轨迹点文本)
  output:text_target(target坐标系的轨迹点文本)
  input: init坐标系到target坐标系的变换矩阵

  TrackPoints_target = T_target2init*TrackPoints_init;
  */
void TransMatrix::newTrackPointsFileByTransMatrix(std::string &text_init,std::string &text_target,
                                                    Eigen::Matrix4d &T_base2end, Eigen::Matrix4d &T_tool,
                                                    Eigen::Matrix4d &T_end2cam,  Eigen::Matrix4d &T_cam2obj)
{

      fstream inFile_init;
      inFile_init.open(text_init,ios::in);
      if(!inFile_init)
      {
          std::cout<<"open text_init failed!"<<std::endl;
          return;
      }
      fstream outFile_target;
      outFile_target.open(text_target,ios::out|ios::trunc);
      if(!outFile_target)
      {
          std::cout<<"open text_target failed!"<<std::endl;
          return;
      }
      string str_pose_init;
      Eigen::Quaterniond Q;
      //轨迹点用一个坐标系来表示，由旋转矩阵和平移矩阵组成，给出以当前点为原点的坐标系姿态
      Eigen::Matrix4d TrackPoints_init;
      Eigen::Matrix4d TrackPoints_end;
      Eigen::Matrix4d TrackPoints_target;
      Eigen::Vector3d T;
      while(getline(inFile_init,str_pose_init))//是否到达文件末尾
      {
          //getline(inFile_init,str_pose_init);   
          if(str_pose_init == "NEXT")
          {
            outFile_target<<"NEXT"<<std::endl;
            continue; 
          }

          stringstream stream_pose(str_pose_init);
          //x y z
          stream_pose >> T[0];
          stream_pose >> T[1];
          stream_pose >> T[2];
          //四元数
          stream_pose >> Q.x();
          stream_pose >> Q.y();
          stream_pose >> Q.z();
          stream_pose >> Q.w();
          TrackPoints_init = getTransMatrix(T,Q);    

          //末端坐标系下的点
          TrackPoints_end = T_end2cam*T_cam2obj*TrackPoints_init;
          //因为机械臂末端有工具，还要去掉工具的尺寸，这个偏移量实在目标点坐标系下的大小，需要转换为末端坐标系下的
          TrackPoints_end = TrackPoints_end * T_tool;

          TrackPoints_target = T_base2end*TrackPoints_end;


          //将text_init坐标系下的位姿点写入文件text_target
          writeTrackPoints2File(TrackPoints_target,outFile_target);
      }
      inFile_init.close();
      outFile_target.close();
}


void TransMatrix::printfMatrix4d(Eigen::Matrix4d &T,const std::string &matrixName)
{
      std::cout<<matrixName<<" :"<<std::endl;
      std::cout<<T(0,0)<<" ";
      std::cout<<T(0,1)<<" ";
      std::cout<<T(0,2)<<" ";
      std::cout<<T(0,3)<<std::endl;
      std::cout<<T(1,0)<<" ";
      std::cout<<T(1,1)<<" ";  
      std::cout<<T(1,2)<<" ";    
      std::cout<<T(1,3)<<std::endl;
      std::cout<<T(2,0)<<" ";
      std::cout<<T(2,1)<<" ";
      std::cout<<T(2,2)<<" ";
      std::cout<<T(2,3)<<std::endl;
      std::cout<<T(3,0)<<" ";
      std::cout<<T(3,1)<<" ";
      std::cout<<T(3,2)<<" ";
      std::cout<<T(3,3)<<std::endl;
}

