/*
 * rosnode.h
 *
 *  Created on: 2017-3-22
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_ROS_NODE_H_
#define SYS_ROS_NODE_H_

#include <vector>
#include "sys/sys_types.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cainiao_robot
{

#define MAX_NODE_NAME_LEN 128
#define MAX_TOPIC_LEN 128
#define WAIT_FOR_SPIN 1500

/* Log re-define
 * */
#define SYS_LOGE ROS_ERROR
#define SYS_LOGI ROS_INFO
#define SYS_LOGD ROS_DEBUG
#define SYS_LOGW ROS_WARN
#define SYS_LOGF ROS_FATAL

typedef struct ros_node_sbscrbr_s
{
    char name[MAX_TOPIC_LEN];
    ros::Subscriber sub;

} ros_node_sbscrbr_t;


typedef struct ros_node_pblshr_s
{
    char name[MAX_TOPIC_LEN];
    ros::Publisher pub;

} ros_node_pblshr_t;

typedef void* SYS_PUBLISHER;

#define SYS_ROS_SCOPE __ros_def_scope_handle_6239
#define SYS_ROS_NODE_SCOPE_ENTRY ros::NodeHandle SYS_ROS_SCOPE;
#define SYS_ROS_GLOBAL_ENV_INIT(c, v, n) \
        sys_ros_node::sys_initialize(c, v, n); \
        SYS_ROS_NODE_SCOPE_ENTRY \
        sys_ros_node::set_handle(&SYS_ROS_SCOPE);

class sys_ros_node
{
public:
    sys_ros_node();
    virtual ~sys_ros_node();

    static int sys_initialize(int argc, char **argv, char *name)
    {
        ros::init(argc, argv, name);
    }
    static void set_handle(ros::NodeHandle *handle) { s_p_scope = handle; }
    static void sys_run();

    int (* get_callback() ) (void*, u32) {  return this->on_message_callback; }
    void (* get_readytopub_handle())() {  return this->on_ready_to_publish; }

    int publish(cc8 *topic, void *data, int len);

    int add_subscriber(cc8 *topic, int buf_len);

    SYS_PUBLISHER add_publisher(cc8 *topic, int buf_len);

    int remove_subscriber(cc8 *topic);

    int remove_publisher(cc8 *topic);

    void set_on_message_callback_func(int (*cb)(void *data, u32 len));
    void set_on_ready_to_publish(void (*cb)());

    void run();

protected:

    /* Must be override */
    virtual ros::Publisher advertise(cc8 *topic, int buf_len) { return ros::Publisher(); };
    virtual ros::Subscriber subscribe(cc8 *topic, int buf_len) { return ros::Subscriber(); };
    virtual int publish(SYS_PUBLISHER handle, void *data, int len) { return 0; };

    std::vector<ros_node_sbscrbr_t>      m_vec_subs;
    std::vector<ros_node_pblshr_t*>      m_vec_pubs;

    int (*on_message_callback) (void *data, u32 len);
    void (*on_ready_to_publish) ();

    static ros::NodeHandle *s_p_scope;
};
} /* namespace cainiao_robot */
#endif /* SYS_ROS_NODE_H_ */
