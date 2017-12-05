#include "aubo_msgs/gettargetpose.h"
#include <string>
#include <iostream>
#include "tf/tf.h"
#define PI 3.14159265359
using namespace std;

 bool publish(aubo_msgs::gettargetpose::Request  &req,
             aubo_msgs::gettargetpose::Response &res)
    {
    geometry_msgs::Pose goal;
 
    goal.position.x = -0.4329;
    goal.position.y = 0.3865;
    goal.position.z = -0.05;
    ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    double pid = PI / 180.0;
    tf::Quaternion q = tf::createQuaternionFromRPY( -pid *178 , pid*3.05,pid  * 40.71 ); // roll , pitch ,yaw
   // tf::Quaternion q = tf::createQuaternionFromRPY( pid  * 9.95 ,pid  * 63.9 ,pid  * 7.31 ); // roll , pitch ,yaw
    goal.orientation.x = q.x();
    goal.orientation.y = q.y();
    goal.orientation.z = q.z();
    goal.orientation.w = q.w();
    ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    res.target_pose = goal;
    res.succeeded = true;

     return true;
   }

   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "publish_pose");
     ros::NodeHandle n;
   
     ros::ServiceServer service = n.advertiseService("target_recognition", publish);
     ROS_INFO("Ready to publish.");
     ros::spin();
   
     return 0;
   }
