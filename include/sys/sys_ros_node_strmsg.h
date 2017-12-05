/*
 * rosnode.h
 *
 *  Created on: 2017-3-22
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_ROS_NODE_STRMSG_H_
#define SYS_ROS_NODE_STRMSG_H_

#include <vector>
#include "sys/sys_ros_node.h"

namespace cainiao_robot
{

class sys_ros_node_strmsg : public sys_ros_node
{
public:
    virtual ~sys_ros_node_strmsg() {}

    virtual int publish(SYS_PUBLISHER handle, void *msg, int len);

    static sys_ros_node *instance()
    {
        if(sys_ros_node_strmsg::s_p_self == 0)
        {
            sys_ros_node_strmsg::s_p_self = new sys_ros_node_strmsg();
        }
        return sys_ros_node_strmsg::s_p_self;
    }

protected:
    /* Override */
    ros::Publisher advertise(cc8 *topic, int buf_len);
    ros::Subscriber subscribe(cc8 *topic, int buf_len);

    sys_ros_node_strmsg() { sys_ros_node(); }
    static sys_ros_node *s_p_self;
};

} /* namespace cainiao_robot */
#endif /* SYS_ROS_NODE_STRMSG_H_ */
