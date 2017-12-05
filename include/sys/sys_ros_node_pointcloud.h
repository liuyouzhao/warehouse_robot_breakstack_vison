/*
 * sys_ros_node_pointcloud.h
 *
 *  Created on: 2017-3-23
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_ROS_NODE_POINTCLOUD_H_
#define SYS_ROS_NODE_POINTCLOUD_H_

#include "sys/sys_ros_node.h"

namespace cainiao_robot
{

#define SYS_ROS_POINTCLOUD_CHANNELS 3

typedef struct msg_pointcloud_s
{
    float *data;
    u32 width;
    u32 height;
} msg_pointcloud_t;

/*
 *
 */
class sys_ros_node_pointcloud : public sys_ros_node
{
public:
    virtual ~sys_ros_node_pointcloud() {}

    virtual int publish(SYS_PUBLISHER handle, void *msg, int len);

    static sys_ros_node *instance()
    {
        if(s_p_self == 0)
        {
            s_p_self = new sys_ros_node_pointcloud();
        }
        return s_p_self;
    }
protected:
    /* Override */
    ros::Publisher advertise(cc8 *topic, int buf_len);
    ros::Subscriber subscribe(cc8 *topic, int buf_len);

    sys_ros_node_pointcloud() { sys_ros_node(); }
    static sys_ros_node *s_p_self;
};

} /* namespace cainiao_robot */
#endif /* SYS_ROS_NODE_POINTCLOUD_H_ */
