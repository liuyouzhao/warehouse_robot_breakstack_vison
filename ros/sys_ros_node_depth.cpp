/*
 * sys_ros_node_depth2.cpp
 *
 *  Created on: 2017-3-23
 *      Author: hujia
 *  Description:
 */

#include "sys/sys_ros_node_depth.h"
#include <sensor_msgs/PointCloud.h>

namespace cainiao_robot
{

sys_ros_node *sys_ros_node_depth::s_p_self = 0;

static void stem_callback(const sensor_msgs::PointCloud::ConstPtr& input)
{
    if(sys_ros_node_depth::instance()->get_callback() == NULL)
    {
        return;
    }
    msg_depth_t mdp;

    mdp.width = input->channels[1].values[0];
    mdp.height = input->channels[1].values[1];

    u32 len = mdp.width * mdp.height;
    u32 siz = len * sizeof(float);
    float *buffer = (float*)malloc(siz);

    for(int i = 0; i < len; i ++)
    {
        buffer[i] = input->channels[0].values[i];
    }
    mdp.depth = buffer;

    sys_ros_node_depth::instance()->get_callback()(&mdp,  len);
    free(buffer);
}

ros::Publisher sys_ros_node_depth::advertise(cc8 *topic, int buf_len)
{
    return s_p_scope->advertise<sensor_msgs::PointCloud>(topic, buf_len);
}

ros::Subscriber sys_ros_node_depth::subscribe(cc8 *topic, int buf_len)
{
    return s_p_scope->subscribe(topic, buf_len, stem_callback);
}

int sys_ros_node_depth::publish(SYS_PUBLISHER handle, void *msg, int len)
{
    if(handle == NULL || msg == NULL)
    {
        SYS_LOGE("Fatal error, publish params incorrect. \n");
        return -1;
    }
    msg_depth_t *pdt = (msg_depth_t*)msg;

    int rc = 0;
    int num_points = pdt->width * pdt->height;

    ros_node_pblshr_t *p = (ros_node_pblshr_t*) handle;

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";
    cloud.channels.resize(2);

    cloud.channels[1].name = "width_height";
    cloud.channels[1].values.resize(2);
    cloud.channels[1].values[0] = pdt->width;
    cloud.channels[1].values[1] = pdt->height;

    cloud.channels[0].name = "data";
    cloud.channels[0].values.resize(num_points);

    float *points = (float*) pdt->depth;

    for(int i = 0; i < num_points; i ++)
    {
        cloud.channels[0].values[i] = points[i];
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
