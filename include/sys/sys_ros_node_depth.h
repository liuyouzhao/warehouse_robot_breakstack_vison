/*
 * sys_ros_node_depth.h
 *
 *  Created on: 2017-3-23
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_ROS_NODE_DEPTH_H_
#define SYS_ROS_NODE_DEPTH_H_

#include "sys/sys_ros_node.h"

namespace cainiao_robot
{

typedef struct msg_depth_s
{
    float *depth;
    unsigned int width;
    unsigned int height;
} msg_depth_t;

/*
 *
 */
class sys_ros_node_depth : public sys_ros_node
{
public:
    virtual ~sys_ros_node_depth() {}

    virtual int publish(SYS_PUBLISHER handle, void *msg, int len);

    static sys_ros_node *instance()
    {
        if(s_p_self == 0)
        {
            s_p_self = new sys_ros_node_depth();
        }
        return s_p_self;
    }
protected:
    /* Override */
    ros::Publisher advertise(cc8 *topic, int buf_len);
    ros::Subscriber subscribe(cc8 *topic, int buf_len);

    sys_ros_node_depth() { sys_ros_node(); }
    static sys_ros_node *s_p_self;
};

} /* namespace cainiao_robot */
#endif /* SYS_ROS_NODE_DEPTH_H_ */
