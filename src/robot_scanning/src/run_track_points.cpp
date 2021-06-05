#include "robot_scanning/run_track_points.h"
#include <vector>

RunTrackPoints::RunTrackPoints()
{

}

RunTrackPoints::~RunTrackPoints()
{

}

void RunTrackPoints::initMoveit()
{
  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";

  // Create a planning group interface object and set up a planning group
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_group->setPoseReferenceFrame("base_link");

  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a robot model information object
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Create an object of the visualization class
  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();

  // Load remote control tool
  visual_tools->loadRemoteControl();

  // Create text
  text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2;
  namespace rvt = rviz_visual_tools;
  visual_tools->publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  // Text visualization takes effect
  visual_tools->trigger();

  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("scanning", "Planning frame: %s", move_group->getPlanningFrame().c_str());
  // Get the end of the basic information
  ROS_INFO_NAMED("scanning", "End effector link: %s", move_group->getEndEffectorLink().c_str());
}


void RunTrackPoints::setMaxVelAndAccScalingFactor(double vel_factor,double acc_factor)
{
  //Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
  move_group->setMaxVelocityScalingFactor(vel_factor); //0.03
  move_group->setMaxAccelerationScalingFactor(acc_factor); //0.1
}


void RunTrackPoints::gotoTargetByJoint(std::vector<double> &joints)
{
  std::vector<double> robot_position;
  robot_position.push_back(joints[0]);
  robot_position.push_back(joints[1]);
  robot_position.push_back(joints[2]);
  robot_position.push_back(joints[3]);
  robot_position.push_back(joints[4]);
  robot_position.push_back(joints[5]);
  move_group->setJointValueTarget(robot_position);
  move_group->move();
}


void RunTrackPoints::gotoTargetByPose(double pose[])
{
  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(pose[3],pose[4],pose[5]);       //radian

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = pose[0];
  target_pose1.position.y = pose[1];
  target_pose1.position.z = pose[2];
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  move_group->setPoseTarget(target_pose1);

  // Call the planner for planning calculations Note: This is just planning
  
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

  // visual planning path in Rviz
  visual_tools->deleteAllMarkers();
  visual_tools->publishAxisLabeled(target_pose1, "pose1");
  namespace rvt = rviz_visual_tools;
  visual_tools->publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools->trigger();

  // Perform planning actions
  move_group->execute(my_plan);
}



void RunTrackPoints::visualBlocking(std::string message)
{
  // Visual terminal prompt (blocking)
  visual_tools->prompt(message);
}

void RunTrackPoints::runWayPoints(std::vector<geometry_msgs::Pose> &waypoints)
{
    // We want the Cartesian path to be interpolated at a resolution of 1 cm.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;           //(The jump threshold is set to 0.0)
  //影响轨迹运动效果
  const double eef_step = 0.001;                //(interpolation step)

  // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


  // Visualize the plan in RViz
  namespace rvt = rviz_visual_tools;
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "AUBO Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
  visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools->trigger();

  my_plan.trajectory_= trajectory;
  move_group->execute(my_plan);
}


//获得当前基坐标系到末端坐标系的变换矩阵
Eigen::Matrix4d RunTrackPoints::getTransMatrix()
{
  //计算base_link ==>  wrist3_Link(末端坐标系)的变换矩阵
  geometry_msgs::PoseStamped pose_end = move_group->getCurrentPose("wrist3_Link");

  std::cout<<"x:"<<pose_end.pose.position.x<<std::endl;
  std::cout<<"y:"<<pose_end.pose.position.y<<std::endl;
  std::cout<<"z:"<<pose_end.pose.position.z-0.502<<std::endl;
  std::cout<<"wx:"<<pose_end.pose.orientation.x<<std::endl;

  Eigen::Matrix4d trans_matrix;
  Eigen::Matrix3d qua_matrix;
  Eigen::Quaterniond Q;

  //平移矩阵
  trans_matrix(0,3) = pose_end.pose.position.x;
  trans_matrix(1,3) = pose_end.pose.position.y;
  trans_matrix(2,3) = pose_end.pose.position.z-0.502;

  // 旋转矩阵
  Q.x() = pose_end.pose.orientation.x;
  Q.y() = pose_end.pose.orientation.y;
  Q.z() = pose_end.pose.orientation.z;
  Q.w() = pose_end.pose.orientation.w;
  qua_matrix = Q.matrix();

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

