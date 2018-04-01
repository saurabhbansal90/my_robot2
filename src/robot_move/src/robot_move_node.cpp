

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <stdio.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int move_joint(ros::Publisher joint_pub, double value)
{
	trajectory_msgs::JointTrajectory traj;
	trajectory_msgs::JointTrajectoryPoint point;
	
	traj.header.stamp = ros::Time::now();
	traj.header.frame_id = "/world";
	traj.joint_names.resize(1);
	traj.points.resize(1);
	traj.joint_names[0] = "base_to_child";
	point.positions.resize(1);
	point.positions[0] = value;
	point.time_from_start = ros::Duration(2.0);
	traj.points[0] = point;
	joint_pub.publish(traj);
	
	ROS_INFO("Moving joint to %f", value);
	
	ros::spinOnce();
	return 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_move_node");
	ros::NodeHandle nh;
	
	ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/my_ns/joint_traj_controller/command", 1);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate loop_rate(10);
	
	moveit::planning_interface::MoveGroupInterface group("Arm");
	group.setPoseReferenceFrame("world");
	geometry_msgs::PoseStamped target_pose;
	target_pose = group.getCurrentPose("child_link");
	ROS_INFO("Current Pose: %lf, %lf, %lf", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
	target_pose.pose.position.x += 0.1;
	target_pose.header.stamp = ros::Time::now();
	group.setPoseTarget(target_pose, "child_link");
	group.allowReplanning(true);
	group.setStartStateToCurrentState();
	group.setNumPlanningAttempts(5);
	group.setPlanningTime(2);

	moveit::planning_interface::MoveGroupInterface::Plan myPlan;
	ROS_INFO("Planning for the group");
	group.plan(myPlan);
	sleep(5.0);
	ROS_INFO("Executing the plan");
	group.execute(myPlan);
	ROS_INFO("Moving as per Plan");
	group.move();

	ROS_INFO("Yes, it's successful");
}


