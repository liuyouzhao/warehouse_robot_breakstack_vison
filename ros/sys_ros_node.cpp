/*
 * rosnode.cpp
 *
 *  Created on: 2017-3-22
 *      Author: hujia
 *  Description:
 */

#include "sys/sys_ros_node.h"
#include <sstream>

namespace cainiao_robot
{
ros::NodeHandle* sys_ros_node::s_p_scope = 0;
sys_ros_node::sys_ros_node() :
        on_message_callback(NULL),
        on_ready_to_publish(NULL)
{
    SYS_LOGD("[%s:%d] %s \n", __FILE__, __LINE__, __FUNCTION__);
}

sys_ros_node::~sys_ros_node()
{
    std::vector<ros_node_pblshr_t*>::iterator iter = m_vec_pubs.end();
    while(m_vec_pubs.size() > 0)
    {
        ros_node_pblshr_t *p = *iter;
        delete p;
        m_vec_pubs.pop_back();
    }
}

void sys_ros_node::set_on_message_callback_func(int (*cb)(void *data, u32 len))
{
    on_message_callback = cb;
}

void sys_ros_node::set_on_ready_to_publish(void (*cb)())
{
    on_ready_to_publish = cb;
}

int sys_ros_node::add_subscriber(cc8 *topic, int buf_len)
{
    ros::Subscriber sub = subscribe(topic, buf_len);

    ros_node_sbscrbr_t sub_t;
    memset(sub_t.name, 0, MAX_TOPIC_LEN);
    memcpy(sub_t.name, topic, strlen(topic));
    sub_t.sub = sub;

    m_vec_subs.push_back(sub_t);

    return 0;
}

int sys_ros_node::remove_subscriber(cc8 *topic)
{
    std::vector<ros_node_sbscrbr_s>::iterator iter = m_vec_subs.begin();
    for(; iter != m_vec_subs.end(); iter ++)
    {
        ros_node_sbscrbr_t *p = &(*iter);
        if(strcmp(p->name, topic) == 0)
        {
            m_vec_subs.erase(iter);
            return 0;
        }
    }

    return -1;
}

SYS_PUBLISHER sys_ros_node::add_publisher(cc8 *topic, int buf_len)
{
    ros::Publisher pub = advertise(topic, buf_len);

    ros_node_pblshr_t* pub_t = new ros_node_pblshr_t();
    memset(pub_t->name, 0, MAX_TOPIC_LEN);
    memcpy(pub_t->name, topic, strlen(topic));
    pub_t->pub = pub;

    m_vec_pubs.push_back(pub_t);
    return pub_t;
}

int sys_ros_node::remove_publisher(cc8 *topic)
{
    std::vector<ros_node_pblshr_t*>::iterator iter = m_vec_pubs.begin();
    for(; iter != m_vec_pubs.end(); iter ++)
    {
        ros_node_pblshr_t *p = (*iter);
        if(strcmp(p->name, topic) == 0)
        {
            m_vec_pubs.erase(iter);
            delete p;
            return 0;
        }
    }
    return -1;
}

int sys_ros_node::publish(cc8 *topic, void *msg, int len)
{
    ros_node_pblshr_t *p = 0;
    ros_node_pblshr_t *find = 0;
    std::vector<ros_node_pblshr_t*>::iterator iter = m_vec_pubs.begin();
    for(; iter != m_vec_pubs.end(); iter ++)
    {
        p = (*iter);
        if(strcmp(p->name, topic) == 0)
        {
            find = p;
            break;
        }
    }
    if(find != 0)
    {
        return publish(find, msg, len);
    }
    return 0;
}

static void *__pthread_func_isolated(void *p)
{
    usleep(1000 * WAIT_FOR_SPIN);
    ((sys_ros_node*)p)->get_readytopub_handle()();
}

void sys_ros_node::run()
{
   if(this->on_ready_to_publish == NULL)
   {
       printf("Fatal error, please call set_on_ready_to_publish before run()");
       return;
   }
   pthread_t thread;
   pthread_create(&thread, NULL, __pthread_func_isolated, this);
}

void sys_ros_node::sys_run()
{
    ros::spin();
}

} /* namespace cainiao_robot */
