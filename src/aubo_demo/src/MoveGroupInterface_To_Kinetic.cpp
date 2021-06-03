/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 *
 *  Author: zhaoyu
 *  email : zhaoyu@aubo-robotics.cn
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




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>






int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveGroupInterface_To_Kinetic");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("base_link");


  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Create an object of the visualization class
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link"); //base_link
  visual_tools.deleteAllMarkers();


  // Load remote control tool
  visual_tools.loadRemoteControl();


  // Create text
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  // Text visualization takes effect
  visual_tools.trigger();


  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

 
  ROS_INFO_NAMED("tutorial", "pose reference frame link: %s", move_group.getPoseReferenceFrame().c_str());


  // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
  move_group.setMaxVelocityScalingFactor(0.03);
  move_group.setMaxAccelerationScalingFactor(0.1);


/*以下代码使用eigen库*/
  //1.计算base_link ==>  wrist3_Link(末端坐标系)的变换矩阵
  geometry_msgs::PoseStamped pose_end = move_group.getCurrentPose("wrist3_Link");
  
  std::cout<<"x:"<<pose_end.pose.position.x<<std::endl;
  std::cout<<"y:"<<pose_end.pose.position.y<<std::endl;
  std::cout<<"z:"<<pose_end.pose.position.z<<std::endl;
  std::cout<<"wx:"<<pose_end.pose.orientation.x<<std::endl;



  //使用四元数的成员函数matrit()对旋转矩阵赋值
  Eigen::Matrix3d R_base2end;





  
  
  
  /*
  读取文件，计算到
  //方向向量转欧拉角在转四元数
  x=
  y=
  z=
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57); 
  */













  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo");
  
  //***************************************************************************************************  Home Position

  std::vector<double> home_position;
  home_position.push_back(-0.001255);
  home_position.push_back(0.0);
  home_position.push_back(-1.406503);
  home_position.push_back(0.311441);
  home_position.push_back(-1.571295);
  home_position.push_back(-0.002450);
  move_group.setJointValueTarget(home_position);
  move_group.move();

visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to start the demo");
  //**************************************************************************************************   First example: planning and moving to a target pose
  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57);       //radian

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.3;
  target_pose1.position.y = -0.4;
  target_pose1.position.z = 0.10;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  move_group.setPoseTarget(target_pose1);

  // Call the planner for planning calculations Note: This is just planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


  // visual planning path in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();


  // Perform planning actions
  move_group.execute(my_plan);

  


/*
  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();
*/


visual_tools.prompt("Press 'next'3 in the RvizVisualToolsGui window to start the demo");
  // **************************************************************************************************   Fourth example: planning and moving a Cartesian interpolation path

  //  Add three waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose3 = target_pose1;
for(int i=0;i<20;i++)
{
  target_pose3.position.z -= 0.01;
  waypoints.push_back(target_pose3);  // down
}
             //target_pose3.position.y -= 0.1;
  //waypoints.push_back(target_pose3);  // right




  // We want the Cartesian path to be interpolated at a resolution of 1 cm.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;             //(The jump threshold is set to 0.0)
  //影响轨迹运动效果
  const double eef_step = 0.0005;                //(interpolation step)



  // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();


  // Move to the home point position
  my_plan.trajectory_= trajectory;
  move_group.execute(my_plan);

  // Clear path constraint
  move_group.setJointValueTarget(home_position);
  move_group.move();






   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
