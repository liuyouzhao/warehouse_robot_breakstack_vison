/*
 * sys_ros_node_pointcloud2.cpp
 *
 *  Created on: 2017-3-23
 *      Author: hujia
 *  Description:
 */

#include "sys/sys_ros_node_pointcloud.h"
#include <sensor_msgs/PointCloud.h>

namespace cainiao_robot
{

sys_ros_node *sys_ros_node_pointcloud::s_p_self = 0;

static void stem_callback(const sensor_msgs::PointCloud::ConstPtr& input)
{
    if(sys_ros_node_pointcloud::instance()->get_callback() == NULL)
    {
        return;
    }
    u32 len = input->points.size() * 3;
    u32 siz = len * sizeof(float);
    float *buffer = (float*)malloc(siz);

    for(int i = 0; i < input->points.size(); i ++)
    {
        buffer[i * 3]       = input->points[i].x;
        buffer[i * 3 + 1]   = input->points[i].y;
        buffer[i * 3 + 2]   = input->points[i].z;
    }

    msg_pointcloud_t mp;
    mp.data = buffer;
    mp.width =  input->channels[0].values[0];
    mp.height =  input->channels[0].values[1];

    sys_ros_node_pointcloud::instance()->get_callback()(&mp,  len);
    free(buffer);
}

ros::Publisher sys_ros_node_pointcloud::advertise(cc8 *topic, int buf_len)
{
    return s_p_scope->advertise<sensor_msgs::PointCloud>(topic, buf_len);
}

ros::Subscriber sys_ros_node_pointcloud::subscribe(cc8 *topic, int buf_len)
{
    return s_p_scope->subscribe(topic, buf_len, stem_callback);
}

int sys_ros_node_pointcloud::publish(SYS_PUBLISHER handle, void *msg, int len)
{
    if(handle == NULL || msg == NULL)
    {
        SYS_LOGE("Fatal error, publish params incorrect. \n");
        return -1;
    }
    int rc = 0;
    ros_node_pblshr_t *p = (ros_node_pblshr_t*) handle;

    msg_pointcloud_t *ppt = (msg_pointcloud_t*)msg;

    int num_points = ppt->width * ppt->height;

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "points";
    cloud.points.resize(num_points);

    /* width && height info */
    cloud.channels.resize(1);
    cloud.channels[0].name = "width_height";
    cloud.channels[0].values.resize(2);
    cloud.channels[0].values[0] = ppt->width;
    cloud.channels[0].values[1] = ppt->height;

    float *points = ppt->data;

    for(int i = 0; i < num_points; i ++)
    {
        cloud.points[i].x = points[i * 3];
        cloud.points[i].y = points[i * 3 + 1];
        cloud.points[i].z = points[i * 3 + 2];
    }

    p->pub.publish(cloud);
    ros::spinOnce();

    if(ros::ok())
    {
        return num_points;
    }

    return -1;
}

} /* namespace cainiao_robot */
