/*
 * rosnode.cpp
 *
 *  Created on: 2017-3-22
 *      Author: hujia
 *  Description:
 */

#include "sys/sys_ros_node_strmsg.h"
#include <sstream>

namespace cainiao_robot
{
sys_ros_node *sys_ros_node_strmsg::s_p_self = 0;

static void stem_callback(const std_msgs::String::ConstPtr& msg)
{
    if(sys_ros_node_strmsg::instance()->get_callback() == NULL)
    {
        return;
    }
    u8 *buffer = (u8*)malloc(msg->data.length());
    memset(buffer, 0, msg->data.length());
    memcpy(buffer, msg->data.data(), msg->data.length());
    sys_ros_node_strmsg::instance()->get_callback()(buffer,  msg->data.length());
    free(buffer);
}

ros::Publisher sys_ros_node_strmsg::advertise(cc8 *topic, int buf_len)
{
    return s_p_scope->advertise<std_msgs::String>(topic, buf_len);
}

ros::Subscriber sys_ros_node_strmsg::subscribe(cc8 *topic, int buf_len)
{
    return s_p_scope->subscribe(topic, buf_len, stem_callback);
}

int sys_ros_node_strmsg::publish(SYS_PUBLISHER handle, void *msg, int len)
{
    if(handle == NULL || msg == NULL)
    {
        SYS_LOGE("Fatal error, publish params incorrect. \n");
        return -1;
    }
    int rc = 0;

    ros_node_pblshr_t *p = (ros_node_pblshr_t*) handle;
    std_msgs::String smsg;
    std::stringstream ss;
    ss << (c8*) msg;
    smsg.data = ss.str();

    p->pub.publish(smsg);
    ros::spinOnce();

    if(ros::ok())
    {
        return smsg.data.length();
    }

    return -1;
}

} /* namespace cainiao_robot */
