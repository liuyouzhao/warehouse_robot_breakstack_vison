

#include "ros/ros.h"
#include <iostream>
#include "aubo_msgs/SetIO.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "IO_test");


  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceClient client = n.serviceClient<aubo_msgs::SetIO>("/set_io");

	while (ros::ok())
	{


	  aubo_msgs::SetIO srv;

	  srv.request.index = 0;
	  srv.request.state = 1;

	  if (client.call(srv))
	  {

	    ROS_INFO("all right!");

	  }
	  else
	  {
	    ROS_ERROR("Failed to call service");
	    return 1;
	  }

	ros::spinOnce();
	//Setting the loop rate
	loop_rate.sleep();

	}
  return 0;
}
