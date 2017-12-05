 /*
  CONFIG:
    author_name: jinshaogang
    author_email: shaogang.jsg@cainiao.com
    date: 2017-5-18
*/
#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudview
{
public:
    cloudview()
    : viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth/filtered", 10, &cloudview::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudview::timerCB, this);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(input, cloud);

        viewer.showCloud(cloud.makeShared());
    }

    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize");

    cloudview handler;

    ros::spin();

    return 0;
}
