#ifndef RUN_TRACK_POINTS_H_
#define RUN_TRACK_POINTS_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

class RunTrackPoints
{
public:
    RunTrackPoints();
    ~RunTrackPoints();
    void initMoveit();
    void setMaxVelAndAccScalingFactor(double vel_factor,double acc_factor);
    void gotoTargetByJoint(std::vector<double> &joints);
    void gotoTargetByPose(double pose[]);
    void visualBlocking(std::string message);
    void runWayPoints(std::vector<geometry_msgs::Pose> &waypoints);


private:
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    Eigen::Affine3d text_pose;
    const robot_state::JointModelGroup* joint_model_group;
};


#endif //RUN_TRACK_POINTS_H_