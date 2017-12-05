/*
  CONFIG:
    author_name: jinshaogang
    author_email: shaogang.jsg@cainiao.com
*/
#include "pick_and_place.h"
#define PI 3.14159265359
// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  geometry_msgs::Pose box_pose, place_poses;


  // ros initialization
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place application instance
  PickAndPlace application;

  std::string  ARM_GROUP_NAME  = "arm_group"; //规划组
  std::string TARGET_RECOGNITION_SERVICE = "target_recognition"; //小盒子的话题


  application.move_group_ptr = MoveGroupPtr( new moveit::planning_interface::MoveGroup(ARM_GROUP_NAME));
  application.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");
  // motion plan client
 // application.motion_plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

  application.target_recognition_client = nh.serviceClient<aubo_msgs::gettargetpose>(TARGET_RECOGNITION_SERVICE);

  application.setIO_client = nh.serviceClient<aubo_msgs::SetIO>("/set_io");


 ros::Rate loop_rate(10);
  // waiting to establish connections
  while(ros::ok())
  {

  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // move to a "clear" position
  bool flag = application.move_to_wait_position();

  // turn off sucker
  application.set_sucker(false);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  // get the box position and orientation
  box_pose = application.detect_box_pick(flag);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);

  application.pickup_box(box_pose);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);

  application.move_to_start_position();

  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);

  application.place_box();

  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  // move back to the "clear" position
  application.move_to_wait_position();
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  ros::spinOnce();
  //Setting the loop rate
  loop_rate.sleep();

  }

  return 0;
}


//机械臂等待下一次抓取的位置
bool PickAndPlace::move_to_wait_position()
{
  
  bool success; 

  //设置等待的状态，在srdf中添加关节的等待位置即可
  move_group_ptr->setNamedTarget(WAIT_POSE_NAME);

  // 设置规划时间
  move_group_ptr->setPlanningTime(60.0f);

  //开始运动规划
  success = move_group_ptr->move();
  return success;
}

//机械臂到达目标位置
bool PickAndPlace::move_to_goal_position()
{
  
  bool success; 

  //设置等待的状态，在srdf中添加关节的等待位置即可
  move_group_ptr->setNamedTarget(GOAL_POSE_NAME);

  // 设置规划时间
  move_group_ptr->setPlanningTime(60.0f);

  //开始运动规划
  success = move_group_ptr->move();
  return success;
}


void PickAndPlace::move_to_start_position()
{
  
  bool success; 

  //设置等待的状态，在srdf中添加关节的等待位置即可
  move_group_ptr->setNamedTarget(START_POSE_NAME);

  // 设置规划时间
  move_group_ptr->setPlanningTime(60.0f);

  //开始运动规划
  success = move_group_ptr->move();
 // return success;
}

//------------------设置开闭吸盘的函数-----------------------------------
void PickAndPlace::set_sucker(bool do_grasp)
{

  aubo_msgs::SetIO srv;
  bool success;

  // 开启吸盘
  if (do_grasp)
  {
    srv.request.index = 0;
    srv.request.state = 1;
  }
  //关闭吸盘
  else
  {
    srv.request.index = 0;
    srv.request.state = 0;
  }

  if(setIO_client.call(srv))
  {
    if (do_grasp)
      ROS_INFO_STREAM("Sucker opened");
    else
      ROS_INFO_STREAM("Sucker cloned");
  }
  else
  {
    ROS_ERROR_STREAM("Sucker failure");
    exit(1);
  }
}

//获取被抓取小盒子位姿的函数
geometry_msgs::Pose PickAndPlace::detect_box_pick(bool start_pick)
{
  
  aubo_msgs::gettargetpose targetPose_srv;

  targetPose_srv.request.pick_start = start_pick;

  geometry_msgs::Pose box_pose;
  if(target_recognition_client.call(targetPose_srv))
  {
    if(targetPose_srv.response.succeeded)
    {
      box_pose = targetPose_srv.response.target_pose;
      ROS_INFO_STREAM("target recognition succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("target recognition failed");
      exit(0);

    }
  }
  else
  {
    ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
        (targetPose_srv.response.succeeded ?"SUCCESS":"FAILURE")
            <<"', exiting");
    exit(0);
  }
  return box_pose;
}

//抓取箱子的函数
void PickAndPlace::pickup_box(const geometry_msgs::Pose& box_pose)
{
 
    bool success;
    ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    success = create_motion_plan(box_pose);
    ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
       if(success)
      {
        ROS_INFO_STREAM("Pick Move Succeeded");
        set_sucker(true); 
      }
      else
      {
        ROS_ERROR_STREAM("Pick Move Failed");
        set_sucker(false);
        exit(1);
      }        

}


//放置小盒子的函数
void PickAndPlace::place_box()
{

  bool success;

  success = move_to_goal_position();


    if(success)
    {
      ROS_INFO_STREAM("Place Move Succeeded");
      set_sucker(false);
    }
    else
    {
      ROS_ERROR_STREAM("Place Move Failed");
      set_sucker(false);
      exit(1);
    }
  
}

//执行运动规划的函数
bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target)
{
    
    move_group_ptr->setEndEffectorLink(WRIST_LINK_NAME);
   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    move_group_ptr->setPlanningTime(100.0f);
    move_group_ptr->setPoseReferenceFrame(WORLD_FRAME_ID);
    // 需要发送的目标点位置
    geometry_msgs::Pose goal;


    double time=(double)ros::Time::now().toSec();
 
    goal = pose_target;
   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    move_group_ptr->setPoseTarget(goal);
   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);

   // move_group_ptr->setGoalTolerance(0.05);
    //执行规划
    move_group_ptr->move();

    time = (double)ros::Time::now().toSec() - time;
    std::cout << "cost time:" << time << std::endl;

   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    bool success = false;
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    //ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    success = move_group_ptr->plan(goal_plan);

    //time = (double)ros::Time::now().toSec() - time;
    //std::cout << "cost time:" << time << std::endl;

    return success;
}
/*

bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target)
{
  // constructing motion plan goal constraints
  std::vector<double> position_tolerances(3,0.01f);
  std::vector<double> orientation_tolerances(3,0.01f);
  move_group_ptr->setEndEffectorLink(WRIST_LINK_NAME);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);

  move_group_ptr->setPlanningTime(60.0f);
  move_group_ptr->setPoseReferenceFrame(WORLD_FRAME_ID);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  geometry_msgs::PoseStamped p;
  p.header.frame_id = WORLD_FRAME_ID;
  p.pose = pose_target;
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(WRIST_LINK_NAME,p,position_tolerances,
      orientation_tolerances);
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  // creating motion plan request
  moveit_msgs::GetMotionPlan motion_plan;
  moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
  moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;

  moveit_msgs::RobotState robot_state;
 // robot_state::RobotStatePtr current_state = move_group_ptr->getCurrentState();
 // req.start_state = current_state;
  req.start_state.is_diff = true;
  req.group_name = ARM_GROUP_NAME;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  req.allowed_planning_time = 60.0f;
  req.num_planning_attempts = 1;
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  move_group_ptr->move();
  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
  // request motion plan
  bool success = false;
  if(motion_plan_client.call(motion_plan))
  {

    success = true;
  }

  return success;
}
*/