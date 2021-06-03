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

#define MAX_JOINT_ACC 50.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 5.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    0.2                // unit m/s^2    4
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
    int nums = 30;
    for(int i=0; i<nums; i++)
    {
        aubo_robot_namespace::wayPoint_S wayPoint;
        
        
        wayPoint.jointpos[0] = 0+i*0.02;
        wayPoint.jointpos[1] = 0+i*0.02;
        wayPoint.jointpos[2] = 0+i*0.02;
        wayPoint.jointpos[3] = 0+i*0.02;
        wayPoint.jointpos[4] = 0+i*0.02;
        wayPoint.jointpos[5] = 0+i*0.02;
        wayPointVector.push_back(wayPoint);
        robot_driver.robot_send_service_.robotServiceAddGlobalWayPoint(wayPoint);
        
        /*
        wayPoint.cartPos.position.x = 0.4+0.01*i;
        wayPoint.cartPos.position.y = -0.06;
        wayPoint.cartPos.position.z = 0.35;
        wayPoint.orientation.w = 1;
        wayPoint.orientation.x = 0;
        wayPoint.orientation.y = 0;
        wayPoint.orientation.z = 0; 
        //求逆解，然后发出去,尝试以下不求逆解，将
        int ret = robot_driver.robot_send_service_.robotServiceRobotIk(startPointJointAngle,
                             wayPoint.cartPos.position,wayPoint.orientation,wayPoint);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            wayPointVector.push_back(wayPoint);
            robot_driver.robot_send_service_.robotServiceAddGlobalWayPoint(wayPoint);
            std::cout<<":"<<wayPointVector.at(i).cartPos.position.x<<std::endl;
        }
        else
        {
            std::cerr<<"调用逆解函数失败"<<std::endl;
        }*/
    }
    std::cout<<"points:"<<wayPointVector.size()<<std::endl;
    std::cout<<":"<<wayPointVector.at(2).cartPos.position.x<<std::endl;
    


    /** 移动到机器人轨迹的初始点 **/

    for(int i=0; i<nums; i++)
{
        int ret = robot_driver.robot_send_service_.robotServiceJointMove(wayPointVector[i].jointpos, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to zero postions, error code:%d", ret);
}
        
    /*
    int ret3 = robot_driver.robot_send_service_.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_UBSPLINEINTP,true);
    if(ret3 != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("error3 code:%d", ret3);*/

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
