#include <ros/ros.h>
#include "tf/tf.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include "opencv2/opencv.hpp"

#define PI 3.14159265359
// namspace
using namespace std;
using namespace sensor_msgs;
using namespace cv;

class pointPublish
{

    
public:
    pointPublish()
    {
       // target_service = nh.advertiseService("target_recognition", &pointPublish::PosePublish, this);
   
        pcl_sub = nh.subscribe("/camera/depth_registered/depth", 30, &pointPublish::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 30);

    }

    void cloudCB(const sensor_msgs::PointCloud::ConstPtr& input)
    { 


        int width = input->channels[1].values[0];
        int height = input->channels[1].values[1];

        unsigned int len = width * height;
        unsigned int siz = len * sizeof(float);
        float *buffer = (float*)malloc(siz);
        ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        for(int i = 0; i < len; i ++)
        {
            buffer[i] = input->channels[0].values[i];
          //  cout<< "打印点云数据：" <<buffer[i] << endl;
        }
        float *depth = buffer;

        sensor_msgs::PointCloud2 output;
        pcl::PointCloud<pcl::PointXYZ> cloud;
         // Fill in the cloud data
        cloud.width  = width;
        cloud.height = height;
        cloud.points.resize(cloud.width * cloud.height);

        for (size_t i = 0; i < cloud.points.size (); ++i)
        {
           
            cloud.points[i].z = input->channels[0].values[i];
            cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        }

       pcl::toROSMsg(cloud, output);
       output.header.frame_id = "camera_depth_optical_frame";

       pcl_pub.publish(output);


        //free(buffer);
        free(depth);


        
    
    }



protected:

    ros::NodeHandle nh;
   // geometry_msgs::Pose goal_;
    //sensor_msgs::PointCloud2 sensor_cloud_msg_;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
  //  ros::ServiceServer target_service;

    
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "target_recognition");

    ROS_INFO("Started target_recognition Node");
    pointPublish handler;

    ros::spin();

    return 0;
}

