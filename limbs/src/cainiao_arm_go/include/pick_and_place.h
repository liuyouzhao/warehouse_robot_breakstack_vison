/*
  CONFIG:
    author_name: jinshaogang
    author_email: shaogang.jsg@cainiao.com
*/
#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <string>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include "aubo_msgs/SetIO.h"
#include <geometry_msgs/Pose.h>
#include "aubo_msgs/gettargetpose.h"
// =============================== aliases ===============================
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
//typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

class PickAndPlace
{
	public:
	// =============================== constructor =====================================
		PickAndPlace()
		{

		}

	// =============================== global members =====================================
		//pick_and_place_config cfg;
		//ros::Publisher marker_publisher;
		//ros::Publisher planning_scene_publisher;
		ros::ServiceClient target_recognition_client;
	//	ros::ServiceClient motion_plan_client;
		//GraspActionClientPtr grasp_action_client_ptr;
		MoveGroupPtr move_group_ptr;
		//TransformListenerPtr transform_listener_ptr;
		ros::ServiceClient setIO_client;

		//  ARM_GROUP_NAME  = "arm_group"; //规划组
		 // MARKER_TOPIC = "pick_and_place_marker";
		std::string PLANNING_SCENE_TOPIC = "planning_scene";
		//std::string TARGET_RECOGNITION_SERVICE = "target_recognition"; //小盒子的话题
		      
		std::string WRIST_LINK_NAME = "ee_Link"; //机械臂末端
		   
		std::string WORLD_FRAME_ID  = "base_Link"; //世界坐标原点，基坐标
		std::string HOME_POSE_NAME  = "home";   //原始状态
		std::string WAIT_POSE_NAME  = "wait";  //等待点
		std::string ARM_GROUP_NAME  = "arm_group"; //规划组

		std::string GOAL_POSE_NAME  = "goal";  //目标点
		std::string START_POSE_NAME  = "start";  //目标点


	// =============================== Task Functions ===============================
		bool move_to_wait_position();

		bool move_to_goal_position();
		void move_to_start_position();

		void set_sucker(bool do_grasp);

		geometry_msgs::Pose detect_box_pick(bool flag);

		void pickup_box(const geometry_msgs::Pose& box_pose);

		void place_box();


		bool create_motion_plan(const geometry_msgs::Pose &pose_target);
	//	bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target,
   // const moveit_msgs::RobotState &start_robot_state,moveit::planning_interface::MoveGroup::Plan &plan)


};


#endif /* PICK_AND_PLACE_H_ */
