/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


using namespace aubo_driver;
using namespace std;

#define MAX_JOINT_ACC 60.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 3.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    0.5                // unit m/s^2    4
#define MAX_END_VEL    0.2                // unit m/s

double zero_poeition[ARM_DOF] = {0};
double initial_poeition[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion1[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};
double postion2[ARM_DOF] = {15.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};

double postion3[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion4[ARM_DOF] = {30.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};


void testMoveL(AuboDriver &robot_driver)
{
    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max joint acc and vel***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i = 0; i < ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
        jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
    }
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

   /** move to inital position **/
    int ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to initial postion, error code:%d", ret);


    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();
    /** Set Max END acc and vel**/
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);

     /** loop for 3 times **/
    for(int i=0; i<3; i++)
    {
        ret = robot_driver.robot_send_service_.robotServiceLineMove(postion3, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to postion3, error code:%d", ret);


        robot_driver.robot_send_service_.robotServiceLineMove(postion4, true);
            ROS_ERROR("Failed to move to postion4, error code:%d", ret);
    }
}





void initJointAngleArray(double *array, double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
{
    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;
}



void testTrackMove(AuboDriver &robot_driver)
{
    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max joint acc and vel***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i = 0; i < ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
        jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
    }
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    //robot_driver.robot_send_service_.robotServiceSetGlobalBlendRadius(0.03);
    robot_driver.robot_send_service_.robotServiceClearGlobalWayPointVector();

    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    /** 先给定一系列位姿点,并生成位姿点容器wayPointVector **/
    double startPointJointAngle[ARM_DOF] = {0};
    initJointAngleArray(startPointJointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  0.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI,0.0/180.0*M_PI);

    /*
    for(int i=0; i<nums; i++)
    {
        aubo_robot_namespace::wayPoint_S wayPoint;      
        
        wayPoint.jointpos[0] = 0+i*0.002;
        wayPoint.jointpos[1] = 0+i*0.002;
        wayPoint.jointpos[2] = 0+i*0.002;
        wayPoint.jointpos[3] = 0+i*0.002;
        wayPoint.jointpos[4] = 0+i*0.002;
        wayPoint.jointpos[5] = 0+i*0.002;
        wayPointVector.push_back(wayPoint);
        robot_driver.robot_send_service_.robotServiceAddGlobalWayPoint(wayPoint);
        */
        


      fstream inFile;
      inFile.open("/home/bianjingyang/catkin_aubo/src/text_base.txt",ios::in);
      if(!inFile)
      {
          std::cout<<"open text_base failed!"<<std::endl;
          return ;
      }
      int count = 0;
      std::string str_pose_base;
      while( getline(inFile,str_pose_base))//是否到达文件末尾
      {   
          if(str_pose_base != "NEXT")
          {
                aubo_robot_namespace::wayPoint_S wayPoint;
                //将一行字符串转换到字符串流
                std::stringstream stream_pose(str_pose_base);
                stream_pose >> wayPoint.cartPos.position.x;
                stream_pose >> wayPoint.cartPos.position.y;
                stream_pose >> wayPoint.cartPos.position.z;
                stream_pose >> wayPoint.orientation.x;
                stream_pose >> wayPoint.orientation.y;
                stream_pose >> wayPoint.orientation.z;
                stream_pose >> wayPoint.orientation.w;


                //求逆解，然后发出去,尝试以下不求逆解，将
                int ret = robot_driver.robot_send_service_.robotServiceRobotIk(startPointJointAngle,
                                        wayPoint.cartPos.position,wayPoint.orientation,wayPoint);
                if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    wayPointVector.push_back(wayPoint);
                    robot_driver.robot_send_service_.robotServiceAddGlobalWayPoint(wayPoint);
                    //std::cout<<":"<<wayPointVector.at(i).cartPos.position.x<<std::endl;
                }
                else
                {
                    std::cerr<<"调用逆解函数失败"<<std::endl;
                }     
                count++;         
        }
        else
        {
            std::cout<<count<<std::endl;
            count = 0;
            //移动到每行轨迹的初始点
            int ret = robot_driver.robot_send_service_.robotServiceJointMove(wayPointVector[0].jointpos, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("Failed to move to zero postions, error code:%d", ret);
                break;
            }
            int ret3 = robot_driver.robot_send_service_.robotServiceTrackMove(aubo_robot_namespace::JOINT_UBSPLINEINTP,true);
            if(ret3 != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("Failed to track move ,error3 code:%d", ret3);
                robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
                break;
            }
            //执行完成之后需要清除轨迹点
            robot_driver.robot_send_service_.robotServiceClearGlobalWayPointVector();
            //创建一个临时容器，清空waypoints
            std::vector<aubo_robot_namespace::wayPoint_S>().swap(wayPointVector);//当此语句执行完成时，临时容器销毁
        }
      }
      inFile.close();
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "testAuboAPI");

  ros::NodeHandle n;
  AuboDriver robot_driver;
  bool ret = robot_driver.connectToRobotController();

  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                             6        /*collision class*/,
                                             true     /* Is allowed to read robot pose*/,
                                             true,    /*default */
                                             1000,    /*default */
                                             result); /*initialize*/
  if(ret)
  {
    testTrackMove(robot_driver);
  }
  else
      ROS_INFO("Failed to connect to the robot controller");

  return 0;
}
