#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "aubo_msgs/gettargetpose.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");


  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceClient target_recognition_client = n.serviceClient<aubo_msgs::gettargetpose>("target_recognition");

  while (ros::ok())
  {


  aubo_msgs::gettargetpose targetPose_srv;

  targetPose_srv.request.pick_start = true;

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
 

    }
  }
  else
  {
    ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
        (targetPose_srv.response.succeeded ?"SUCCESS":"FAILURE")
            <<"', exiting");

  }

 cout<< box_pose.position.x << endl;

  ros::spinOnce();
  //Setting the loop rate
  loop_rate.sleep();

  }
  return 0;
}
