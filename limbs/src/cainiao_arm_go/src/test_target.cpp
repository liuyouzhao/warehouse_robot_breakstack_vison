#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string>
#include <iostream>
#include "tf/tf.h"
#define PI 3.14159265359
using namespace std;
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "test_target");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // 定义运动规划组
    moveit::planning_interface::MoveGroup plan_group("arm_group");
   
 //   ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
   
    string refFrame = "base_Link";
    plan_group.setPoseReferenceFrame(refFrame);
    plan_group.setPlannerId("RRTConnectkConfigDefault");
 //   plan_group.setPlannerId("KPIECEkConfigDefault");
    // 需要发送的目标点位置
    geometry_msgs::Pose goal;
 
    goal.position.x = 0.2;
    goal.position.y = 0.1;
    goal.position.z = 0.5;
    
    double pid = PI / 180.0;
    tf::Quaternion q = tf::createQuaternionFromRPY( pid *30 , pid*20,pid  * 60 ); // roll , pitch ,yaw
   // tf::Quaternion q = tf::createQuaternionFromRPY( pid  * 9.95 ,pid  * 63.9 ,pid  * 7.31 ); // roll , pitch ,yaw
    goal.orientation.x = q.x();
    goal.orientation.y = q.y();
    goal.orientation.z = q.z();
    goal.orientation.w = q.w();
    
    //ROS_INFO("q.x()=","q.y()=","q.z()=","q.w()=",q.x(),q.y(),q,z(),q.w());
    cout<<"q.x()="<<q.x()<<endl;
    cout<<"q.y()="<<q.y()<<endl;
    cout<<"q.z()="<<q.z()<<endl;
    cout<<"q.w()="<<q.w()<<endl;
    /*
    goal.orientation.x = 0.688211;
    goal.orientation.y = 0.00274;
    goal.orientation.z = 0.34479;
    goal.orientation.w = 0.638339;
    */ 
    // 最终发送给move_group的目标点位置
    plan_group.setPoseTarget(goal);

    plan_group.move();
/*
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }
*/
    ros::shutdown();

    return 0;
}

